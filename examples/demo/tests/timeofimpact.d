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
module tests.timeofimpact;

import core.stdc.float_;
import core.stdc.math;

import std.string;
import std.typecons;

import deimos.glfw.glfw3;

import dbox;

import framework.debug_draw;
import framework.test;

class TimeOfImpact : Test
{
    this()
    {
        m_shapeA = new b2PolygonShape();
        m_shapeB = new b2PolygonShape();

        m_shapeA.SetAsBox(25.0f, 5.0f);
        m_shapeB.SetAsBox(2.5f, 2.5f);
    }

    override void Step(Settings* settings)
    {
        super.Step(settings);

        b2Sweep sweepA;
        sweepA.c0.Set(24.0f, -60.0f);
        sweepA.a0 = 2.95f;
        sweepA.c  = sweepA.c0;
        sweepA.a  = sweepA.a0;
        sweepA.localCenter.SetZero();

        b2Sweep sweepB;
        sweepB.c0.Set(53.474274f, -50.252514f);
        sweepB.a0 = 513.36676f;        // - 162.0f * b2_pi;
        sweepB.c.Set(54.595478f, -51.083473f);
        sweepB.a = 513.62781f;         // - 162.0f * b2_pi;
        sweepB.localCenter.SetZero();

        // sweepB.a0 -= 300.0f * b2_pi;
        // sweepB.a -= 300.0f * b2_pi;

        b2TOIInput input;
        input.proxyA.Set(m_shapeA, 0);
        input.proxyB.Set(m_shapeB, 0);
        input.sweepA = sweepA;
        input.sweepB = sweepB;
        input.tMax   = 1.0f;

        b2TOIOutput output;

        b2TimeOfImpact(&output, &input);

        g_debugDraw.DrawString(5, m_textLine, format("toi = %g", output.t));
        m_textLine += DRAW_STRING_NEW_LINE;

        g_debugDraw.DrawString(5, m_textLine, format("max toi iters = %d, max root iters = %d", b2_toiMaxIters, b2_toiMaxRootIters));
        m_textLine += DRAW_STRING_NEW_LINE;

        b2Vec2 vertices[b2_maxPolygonVertices];

        b2Transform transformA;
        sweepA.GetTransform(&transformA, 0.0f);

        for (int32 i = 0; i < m_shapeA.m_count; ++i)
        {
            vertices[i] = b2Mul(transformA, m_shapeA.m_vertices[i]);
        }

        g_debugDraw.DrawPolygon(vertices.ptr, m_shapeA.m_count, b2Color(0.9f, 0.9f, 0.9f));

        b2Transform transformB;
        sweepB.GetTransform(&transformB, 0.0f);

        // b2Vec2 localPoint(2.0f, -0.1f);

        for (int32 i = 0; i < m_shapeB.m_count; ++i)
        {
            vertices[i] = b2Mul(transformB, m_shapeB.m_vertices[i]);
        }

        g_debugDraw.DrawPolygon(vertices.ptr, m_shapeB.m_count, b2Color(0.5f, 0.9f, 0.5f));

        sweepB.GetTransform(&transformB, output.t);

        for (int32 i = 0; i < m_shapeB.m_count; ++i)
        {
            vertices[i] = b2Mul(transformB, m_shapeB.m_vertices[i]);
        }

        g_debugDraw.DrawPolygon(vertices.ptr, m_shapeB.m_count, b2Color(0.5f, 0.7f, 0.9f));

        sweepB.GetTransform(&transformB, 1.0f);

        for (int32 i = 0; i < m_shapeB.m_count; ++i)
        {
            vertices[i] = b2Mul(transformB, m_shapeB.m_vertices[i]);
        }

        g_debugDraw.DrawPolygon(vertices.ptr, m_shapeB.m_count, b2Color(0.9f, 0.5f, 0.5f));
    }

    b2PolygonShape m_shapeA;
    b2PolygonShape m_shapeB;

    static Test Create()
    {
        return new typeof(this);
    }
}
