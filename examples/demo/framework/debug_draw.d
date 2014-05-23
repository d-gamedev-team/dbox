/*
 * Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */
module framework.debug_draw;

import core.stdc.math;
import core.stdc.stdlib;
import core.vararg;

import std.exception;
import std.stdio;
import std.string;

import deimos.glfw.glfw3;

import glad.gl.enums;
import glad.gl.ext;
import glad.gl.funcs;
import glad.gl.loader;
import glad.gl.types;

import glwtf.input;
import glwtf.window;

import dbox;

import imgui;
import imgui.engine;

struct WinSize { int width, height; }

enum winSize = WinSize(1024, 768);

//
struct Camera
{
    //
    b2Vec2 ConvertScreenToWorld(b2Vec2 ps)
    {
        float32 w = cast(float32)m_width;
        float32 h = cast(float32)m_height;
        float32 u = ps.x / w;
        float32 v = (h - ps.y) / h;

        float32 ratio = w / h;
        b2Vec2  extents = b2Vec2(ratio * 25.0f, 25.0f);
        extents *= m_zoom;

        b2Vec2 lower = m_center - extents;
        b2Vec2 upper = m_center + extents;

        b2Vec2 pw;
        pw.x = (1.0f - u) * lower.x + u * upper.x;
        pw.y = (1.0f - v) * lower.y + v * upper.y;
        return pw;
    }

    //
    b2Vec2 ConvertWorldToScreen(b2Vec2 pw)
    {
        float32 w     = cast(float32)m_width;
        float32 h     = cast(float32)m_height;
        float32 ratio = w / h;
        b2Vec2  extents = b2Vec2(ratio * 25.0f, 25.0f);
        extents *= m_zoom;

        b2Vec2 lower = m_center - extents;
        b2Vec2 upper = m_center + extents;

        float32 u = (pw.x - lower.x) / (upper.x - lower.x);
        float32 v = (pw.y - lower.y) / (upper.y - lower.y);

        b2Vec2 ps;
        ps.x = u * w;
        ps.y = (1.0f - v) * h;
        return ps;
    }

    // Convert from world coordinates to normalized device coordinates.
    // http://www.songho.ca/opengl/gl_projectionmatrix.html
    void BuildProjectionMatrix(float32* m, float32 zBias)
    {
        float32 w     = cast(float32)m_width;
        float32 h     = cast(float32)m_height;
        float32 ratio = w / h;
        b2Vec2  extents = b2Vec2(ratio * 25.0f, 25.0f);
        extents *= m_zoom;

        b2Vec2 lower = m_center - extents;
        b2Vec2 upper = m_center + extents;

        m[0] = 2.0f / (upper.x - lower.x);
        m[1] = 0.0f;
        m[2] = 0.0f;
        m[3] = 0.0f;

        m[4] = 0.0f;
        m[5] = 2.0f / (upper.y - lower.y);
        m[6] = 0.0f;
        m[7] = 0.0f;

        m[8]  = 0.0f;
        m[9]  = 0.0f;
        m[10] = 1.0f;
        m[11] = 0.0f;

        m[12] = -(upper.x + lower.x) / (upper.x - lower.x);
        m[13] = -(upper.y + lower.y) / (upper.y - lower.y);
        m[14] = zBias;
        m[15] = 1.0f;
    }

    b2Vec2 m_center = b2Vec2(0, 0);
    float32 m_extent = 25.0;
    float32 m_zoom = 1.0;
    int32 m_width  = winSize.width;
    int32 m_height = winSize.height;
}

// This class implements debug drawing callbacks that are invoked
// inside b2World::Step.
class DebugDraw : b2Draw
{
    //
    this()
    {
        m_points    = null;
        m_lines     = null;
        m_triangles = null;
    }

    //
    void Create()
    {
        m_points = new GLRenderPoints;
        m_points.Create();
        m_lines = new GLRenderLines;
        m_lines.Create();
        m_triangles = new GLRenderTriangles;
        m_triangles.Create();
    }

    //
    void Destroy()
    {
        m_points.Destroy();
        delete m_points;
        m_points = null;

        m_lines.Destroy();
        delete m_lines;
        m_lines = null;

        m_triangles.Destroy();
        delete m_triangles;
        m_triangles = null;
    }

    //
    override void DrawPolygon(const(b2Vec2)* vertices, int32 vertexCount, b2Color color)
    {
        b2Vec2 p1 = vertices[vertexCount - 1];

        for (int32 i = 0; i < vertexCount; ++i)
        {
            b2Vec2 p2 = vertices[i];
            m_lines.Vertex(p1, color);
            m_lines.Vertex(p2, color);
            p1 = p2;
        }
    }

    //
    override void DrawSolidPolygon(const(b2Vec2)* vertices, int32 vertexCount, b2Color color)
    {
        b2Color fillColor = b2Color(0.5f * color.r, 0.5f * color.g, 0.5f * color.b, 0.5f);

        for (int32 i = 1; i < vertexCount - 1; ++i)
        {
            m_triangles.Vertex(vertices[0], fillColor);
            m_triangles.Vertex(vertices[i], fillColor);
            m_triangles.Vertex(vertices[i + 1], fillColor);
        }

        b2Vec2 p1 = vertices[vertexCount - 1];

        for (int32 i = 0; i < vertexCount; ++i)
        {
            b2Vec2 p2 = vertices[i];
            m_lines.Vertex(p1, color);
            m_lines.Vertex(p2, color);
            p1 = p2;
        }
    }

    //
    override void DrawCircle(b2Vec2 center, float32 radius, b2Color color)
    {
        const float32 k_segments  = 16.0f;
        const float32 k_increment = 2.0f * b2_pi / k_segments;
        float32 sinInc = sinf(k_increment);
        float32 cosInc = cosf(k_increment);
        b2Vec2  r1 = b2Vec2(1.0f, 0.0f);
        b2Vec2  v1 = center + radius * r1;

        for (int32 i = 0; i < k_segments; ++i)
        {
            // Perform rotation to avoid additional trigonometry.
            b2Vec2 r2;
            r2.x = cosInc * r1.x - sinInc * r1.y;
            r2.y = sinInc * r1.x + cosInc * r1.y;
            b2Vec2 v2 = center + radius * r2;
            m_lines.Vertex(v1, color);
            m_lines.Vertex(v2, color);
            r1 = r2;
            v1 = v2;
        }
    }

    //
    override void DrawSolidCircle(b2Vec2 center, float32 radius, b2Vec2 axis, b2Color color)
    {
        const float32 k_segments  = 16.0f;
        const float32 k_increment = 2.0f * b2_pi / k_segments;
        float32 sinInc = sinf(k_increment);
        float32 cosInc = cosf(k_increment);
        b2Vec2  v0     = center;
        b2Vec2  r1 = b2Vec2(cosInc, sinInc);
        b2Vec2  v1 = center + radius * r1;
        b2Color fillColor = b2Color(0.5f * color.r, 0.5f * color.g, 0.5f * color.b, 0.5f);

        for (int32 i = 0; i < k_segments; ++i)
        {
            // Perform rotation to avoid additional trigonometry.
            b2Vec2 r2;
            r2.x = cosInc * r1.x - sinInc * r1.y;
            r2.y = sinInc * r1.x + cosInc * r1.y;
            b2Vec2 v2 = center + radius * r2;
            m_triangles.Vertex(v0, fillColor);
            m_triangles.Vertex(v1, fillColor);
            m_triangles.Vertex(v2, fillColor);
            r1 = r2;
            v1 = v2;
        }

        r1.Set(1.0f, 0.0f);
        v1 = center + radius * r1;

        for (int32 i = 0; i < k_segments; ++i)
        {
            b2Vec2 r2;
            r2.x = cosInc * r1.x - sinInc * r1.y;
            r2.y = sinInc * r1.x + cosInc * r1.y;
            b2Vec2 v2 = center + radius * r2;
            m_lines.Vertex(v1, color);
            m_lines.Vertex(v2, color);
            r1 = r2;
            v1 = v2;
        }

        // Draw a line fixed in the circle to animate rotation.
        b2Vec2 p = center + radius * axis;
        m_lines.Vertex(center, color);
        m_lines.Vertex(p, color);
    }

    //
    override void DrawSegment(b2Vec2 p1, b2Vec2 p2, b2Color color)
    {
        m_lines.Vertex(p1, color);
        m_lines.Vertex(p2, color);
    }

    //
    override void DrawTransform(b2Transform xf)
    {
        const float32 k_axisScale = 0.4f;
        b2Color red = b2Color(1.0f, 0.0f, 0.0f);
        b2Color green = b2Color(0.0f, 1.0f, 0.0f);
        b2Vec2  p1 = xf.p, p2;

        m_lines.Vertex(p1, red);
        p2 = p1 + k_axisScale * xf.q.GetXAxis();
        m_lines.Vertex(p2, red);

        m_lines.Vertex(p1, green);
        p2 = p1 + k_axisScale * xf.q.GetYAxis();
        m_lines.Vertex(p2, green);
    }

    void DrawPoint(b2Vec2 p, float32 size, b2Color color)
    {
        m_points.Vertex(p, color, size);
    }

    void DrawString(int x, int y, string str)
    {
        int h = g_camera.m_height;
        addGfxCmdText(x, h - y, TextAlign.left, str, RGBA(230, 153, 153, 255));
    }

    void DrawString(b2Vec2 pw, string str)
    {
        b2Vec2  ps = g_camera.ConvertWorldToScreen(pw);
        int h  = g_camera.m_height;
        addGfxCmdText(cast(int)ps.x, cast(int)(h - ps.y), TextAlign.left, str, RGBA(230, 153, 153, 255));
    }

    void DrawAABB(b2AABB* aabb, b2Color c)
    {
        b2Vec2 p1 = aabb.lowerBound;
        b2Vec2 p2 = b2Vec2(aabb.upperBound.x, aabb.lowerBound.y);
        b2Vec2 p3 = aabb.upperBound;
        b2Vec2 p4 = b2Vec2(aabb.lowerBound.x, aabb.upperBound.y);

        m_lines.Vertex(p1, c);
        m_lines.Vertex(p2, c);

        m_lines.Vertex(p2, c);
        m_lines.Vertex(p3, c);

        m_lines.Vertex(p3, c);
        m_lines.Vertex(p4, c);

        m_lines.Vertex(p4, c);
        m_lines.Vertex(p1, c);
    }

    //
    void Flush()
    {
        m_triangles.Flush();
        m_lines.Flush();
        m_points.Flush();
    }

private:
    GLRenderPoints* m_points;
    GLRenderLines * m_lines;
    GLRenderTriangles* m_triangles;
}

DebugDraw g_debugDraw;
Camera g_camera;

//
void sCheckGLError()
{
    GLenum errCode = glGetError();

    if (errCode != GL_NO_ERROR)
    {
        fprintf(stderr.getFP(), "OpenGL error = %d\n", errCode);
        assert(false);
    }
}

// Prints shader compilation errors
void sPrintLog(GLuint object)
{
    GLint log_length = 0;

    if (glIsShader(object))
        glGetShaderiv(object, GL_INFO_LOG_LENGTH, &log_length);
    else if (glIsProgram(object))
        glGetProgramiv(object, GL_INFO_LOG_LENGTH, &log_length);
    else
    {
        fprintf(stderr.getFP(), "printlog: Not a shader or a program\n");
        return;
    }

    char* log = enforce(cast(char*)malloc(log_length));

    if (glIsShader(object))
        glGetShaderInfoLog(object, log_length, null, log);
    else if (glIsProgram(object))
        glGetProgramInfoLog(object, log_length, null, log);

    fprintf(stderr.getFP(), "%s", log);
    free(log);
}

//
GLuint sCreateShaderFromString(string source, GLenum type)
{
    GLuint res = glCreateShader(type);

    auto ssp = source.ptr;
    int ssl = cast(int)(source.length);
    glShaderSource(res, 1, &ssp, &ssl);
    glCompileShader(res);
    GLint compile_ok = GL_FALSE;
    glGetShaderiv(res, GL_COMPILE_STATUS, &compile_ok);

    if (compile_ok == GL_FALSE)
    {
        fprintf(stderr.getFP(), "Error compiling shader of type %d!\n", type);
        sPrintLog(res);
        glDeleteShader(res);
        return 0;
    }

    return res;
}

//
GLuint sCreateShaderProgram(string vs, string fs)
{
    GLuint vsId = sCreateShaderFromString(vs, GL_VERTEX_SHADER);
    GLuint fsId = sCreateShaderFromString(fs, GL_FRAGMENT_SHADER);
    assert(vsId != 0 && fsId != 0);

    GLuint programId = glCreateProgram();
    glAttachShader(programId, vsId);
    glAttachShader(programId, fsId);
    glBindFragDataLocation(programId, 0, "color");
    glLinkProgram(programId);

    glDeleteShader(vsId);
    glDeleteShader(fsId);

    GLint status = GL_FALSE;
    glGetProgramiv(programId, GL_LINK_STATUS, &status);
    assert(status != GL_FALSE);

    return programId;
}

//
struct GLRenderPoints
{
    void Create()
    {
        string vs =
            "#version 400\n"
            "uniform mat4 projectionMatrix;\n"
            "layout(location = 0) in vec2 v_position;\n"
            "layout(location = 1) in vec4 v_color;\n"
            "layout(location = 2) in float v_size;\n"
            "out vec4 f_color;\n"
            "void main(void)\n"
            "{\n"
            "	f_color = v_color;\n"
            "	gl_Position = projectionMatrix * vec4(v_position, 0.0f, 1.0f);\n"
            "   gl_PointSize = v_size;\n"
            "}\n";

        string fs =
            "#version 400\n"
            "in vec4 f_color;\n"
            "out vec4 color;\n"
            "void main(void)\n"
            "{\n"
            "	color = f_color;\n"
            "}\n";

        m_programId         = sCreateShaderProgram(vs, fs);
        m_projectionUniform = glGetUniformLocation(m_programId, "projectionMatrix");
        m_vertexAttribute   = 0;
        m_colorAttribute    = 1;
        m_sizeAttribute     = 2;

        // Generate
        glGenVertexArrays(1, &m_vaoId);
        glGenBuffers(3, m_vboIds.ptr);

        glBindVertexArray(m_vaoId);
        glEnableVertexAttribArray(m_vertexAttribute);
        glEnableVertexAttribArray(m_colorAttribute);
        glEnableVertexAttribArray(m_sizeAttribute);

        // Vertex buffer
        glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[0]);
        glVertexAttribPointer(m_vertexAttribute, 2, GL_FLOAT, GL_FALSE, 0, cast(const(void*))null);
        glBufferData(GL_ARRAY_BUFFER, m_vertices.sizeof, m_vertices.ptr, GL_DYNAMIC_DRAW);

        glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[1]);
        glVertexAttribPointer(m_colorAttribute, 4, GL_FLOAT, GL_FALSE, 0, cast(const(void*))null);
        glBufferData(GL_ARRAY_BUFFER, m_colors.sizeof, m_colors.ptr, GL_DYNAMIC_DRAW);

        glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[2]);
        glVertexAttribPointer(m_sizeAttribute, 1, GL_FLOAT, GL_FALSE, 0, cast(const(void*))null);
        glBufferData(GL_ARRAY_BUFFER, m_sizes.sizeof, m_sizes.ptr, GL_DYNAMIC_DRAW);

        sCheckGLError();

        // Cleanup
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);

        m_count = 0;
    }

    void Destroy()
    {
        if (m_vaoId)
        {
            glDeleteVertexArrays(1, &m_vaoId);
            glDeleteBuffers(2, m_vboIds.ptr);
            m_vaoId = 0;
        }

        if (m_programId)
        {
            glDeleteProgram(m_programId);
            m_programId = 0;
        }
    }

    void Vertex(b2Vec2 v, b2Color c, float32 size)
    {
        if (m_count == e_maxVertices)
            Flush();

        m_vertices[m_count] = v;
        m_colors[m_count]   = c;
        m_sizes[m_count]    = size;
        ++m_count;
    }

    void Flush()
    {
        if (m_count == 0)
            return;

        glUseProgram(m_programId);

        float32 proj[16] = 0.0f;
        g_camera.BuildProjectionMatrix(proj.ptr, 0.0f);

        glUniformMatrix4fv(m_projectionUniform, 1, GL_FALSE, proj.ptr);

        glBindVertexArray(m_vaoId);

        glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[0]);
        glBufferSubData(GL_ARRAY_BUFFER, 0, m_count * b2Vec2.sizeof, m_vertices.ptr);

        glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[1]);
        glBufferSubData(GL_ARRAY_BUFFER, 0, m_count * b2Color.sizeof, m_colors.ptr);

        glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[2]);
        glBufferSubData(GL_ARRAY_BUFFER, 0, m_count * float32.sizeof, m_sizes.ptr);

        glEnable(GL_PROGRAM_POINT_SIZE);
        glDrawArrays(GL_POINTS, 0, m_count);
        glDisable(GL_PROGRAM_POINT_SIZE);

        sCheckGLError();

        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
        glUseProgram(0);

        m_count = 0;
    }

    enum { e_maxVertices = 512 }
    b2Vec2 m_vertices[e_maxVertices];
    b2Color m_colors[e_maxVertices];
    float32 m_sizes[e_maxVertices];

    int32 m_count;

    GLuint m_vaoId;
    GLuint m_vboIds[3];
    GLuint m_programId;
    GLint m_projectionUniform;
    GLint m_vertexAttribute;
    GLint m_colorAttribute;
    GLint m_sizeAttribute;
}

//
struct GLRenderLines
{
    void Create()
    {
        string vs =
            "#version 400\n"
            "uniform mat4 projectionMatrix;\n"
            "layout(location = 0) in vec2 v_position;\n"
            "layout(location = 1) in vec4 v_color;\n"
            "out vec4 f_color;\n"
            "void main(void)\n"
            "{\n"
            "	f_color = v_color;\n"
            "	gl_Position = projectionMatrix * vec4(v_position, 0.0f, 1.0f);\n"
            "}\n";

        string fs =
            "#version 400\n"
            "in vec4 f_color;\n"
            "out vec4 color;\n"
            "void main(void)\n"
            "{\n"
            "	color = f_color;\n"
            "}\n";

        m_programId         = sCreateShaderProgram(vs, fs);
        m_projectionUniform = glGetUniformLocation(m_programId, "projectionMatrix");
        m_vertexAttribute   = 0;
        m_colorAttribute    = 1;

        // Generate
        glGenVertexArrays(1, &m_vaoId);
        glGenBuffers(2, m_vboIds.ptr);

        glBindVertexArray(m_vaoId);
        glEnableVertexAttribArray(m_vertexAttribute);
        glEnableVertexAttribArray(m_colorAttribute);

        // Vertex buffer
        glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[0]);
        glVertexAttribPointer(m_vertexAttribute, 2, GL_FLOAT, GL_FALSE, 0, cast(const(void*))null);
        glBufferData(GL_ARRAY_BUFFER, m_vertices.sizeof, m_vertices.ptr, GL_DYNAMIC_DRAW);

        glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[1]);
        glVertexAttribPointer(m_colorAttribute, 4, GL_FLOAT, GL_FALSE, 0, cast(const(void*))null);
        glBufferData(GL_ARRAY_BUFFER, m_colors.sizeof, m_colors.ptr, GL_DYNAMIC_DRAW);

        sCheckGLError();

        // Cleanup
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);

        m_count = 0;
    }

    void Destroy()
    {
        if (m_vaoId)
        {
            glDeleteVertexArrays(1, &m_vaoId);
            glDeleteBuffers(2, m_vboIds.ptr);
            m_vaoId = 0;
        }

        if (m_programId)
        {
            glDeleteProgram(m_programId);
            m_programId = 0;
        }
    }

    void Vertex(b2Vec2 v, b2Color c)
    {
        if (m_count == e_maxVertices)
            Flush();

        m_vertices[m_count] = v;
        m_colors[m_count]   = c;
        ++m_count;
    }

    void Flush()
    {
        if (m_count == 0)
            return;

        glUseProgram(m_programId);

        float32 proj[16] = 0.0f;
        g_camera.BuildProjectionMatrix(proj.ptr, 0.1f);

        glUniformMatrix4fv(m_projectionUniform, 1, GL_FALSE, proj.ptr);

        glBindVertexArray(m_vaoId);

        glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[0]);
        glBufferSubData(GL_ARRAY_BUFFER, 0, m_count * b2Vec2.sizeof, m_vertices.ptr);

        glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[1]);
        glBufferSubData(GL_ARRAY_BUFFER, 0, m_count * b2Color.sizeof, m_colors.ptr);

        glDrawArrays(GL_LINES, 0, m_count);

        sCheckGLError();

        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
        glUseProgram(0);

        m_count = 0;
    }

    enum { e_maxVertices = 2 * 512 }
    b2Vec2 m_vertices[e_maxVertices];
    b2Color m_colors[e_maxVertices];

    int32 m_count;

    GLuint m_vaoId;
    GLuint m_vboIds[2];
    GLuint m_programId;
    GLint m_projectionUniform;
    GLint m_vertexAttribute;
    GLint m_colorAttribute;
}

//
struct GLRenderTriangles
{
    void Create()
    {
        string vs =
            "#version 400\n"
            "uniform mat4 projectionMatrix;\n"
            "layout(location = 0) in vec2 v_position;\n"
            "layout(location = 1) in vec4 v_color;\n"
            "out vec4 f_color;\n"
            "void main(void)\n"
            "{\n"
            "	f_color = v_color;\n"
            "	gl_Position = projectionMatrix * vec4(v_position, 0.0f, 1.0f);\n"
            "}\n";

        string fs =
            "#version 400\n"
            "in vec4 f_color;\n"
            "out vec4 color;\n"
            "void main(void)\n"
            "{\n"
            "	color = f_color;\n"
            "}\n";

        m_programId         = sCreateShaderProgram(vs, fs);
        m_projectionUniform = glGetUniformLocation(m_programId, "projectionMatrix");
        m_vertexAttribute   = 0;
        m_colorAttribute    = 1;

        // Generate
        glGenVertexArrays(1, &m_vaoId);
        glGenBuffers(2, m_vboIds.ptr);

        glBindVertexArray(m_vaoId);
        glEnableVertexAttribArray(m_vertexAttribute);
        glEnableVertexAttribArray(m_colorAttribute);

        // Vertex buffer
        glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[0]);
        glVertexAttribPointer(m_vertexAttribute, 2, GL_FLOAT, GL_FALSE, 0, cast(const(void*))null);
        glBufferData(GL_ARRAY_BUFFER, m_vertices.sizeof, m_vertices.ptr, GL_DYNAMIC_DRAW);

        glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[1]);
        glVertexAttribPointer(m_colorAttribute, 4, GL_FLOAT, GL_FALSE, 0, cast(const(void*))null);
        glBufferData(GL_ARRAY_BUFFER, m_colors.sizeof, m_colors.ptr, GL_DYNAMIC_DRAW);

        sCheckGLError();

        // Cleanup
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);

        m_count = 0;
    }

    void Destroy()
    {
        if (m_vaoId)
        {
            glDeleteVertexArrays(1, &m_vaoId);
            glDeleteBuffers(2, m_vboIds.ptr);
            m_vaoId = 0;
        }

        if (m_programId)
        {
            glDeleteProgram(m_programId);
            m_programId = 0;
        }
    }

    void Vertex(b2Vec2 v, b2Color c)
    {
        if (m_count == e_maxVertices)
            Flush();

        m_vertices[m_count] = v;
        m_colors[m_count]   = c;
        ++m_count;
    }

    void Flush()
    {
        if (m_count == 0)
            return;

        glUseProgram(m_programId);

        float32 proj[16] = 0.0f;
        g_camera.BuildProjectionMatrix(proj.ptr, 0.2f);

        glUniformMatrix4fv(m_projectionUniform, 1, GL_FALSE, proj.ptr);

        glBindVertexArray(m_vaoId);

        glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[0]);
        glBufferSubData(GL_ARRAY_BUFFER, 0, m_count * b2Vec2.sizeof, m_vertices.ptr);

        glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[1]);
        glBufferSubData(GL_ARRAY_BUFFER, 0, m_count * b2Color.sizeof, m_colors.ptr);

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDrawArrays(GL_TRIANGLES, 0, m_count);
        glDisable(GL_BLEND);

        sCheckGLError();

        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
        glUseProgram(0);

        m_count = 0;
    }

    enum { e_maxVertices = 3 * 512 }
    b2Vec2 m_vertices[e_maxVertices];
    b2Color m_colors[e_maxVertices];

    int32 m_count;

    GLuint m_vaoId;
    GLuint m_vboIds[2];
    GLuint m_programId;
    GLint m_projectionUniform;
    GLint m_vertexAttribute;
    GLint m_colorAttribute;
}

