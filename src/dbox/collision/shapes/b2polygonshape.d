/*
 * Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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
module dbox.collision.shapes.b2polygonshape;

import core.stdc.float_;
import core.stdc.stdlib;
import core.stdc.string;

import dbox.common;
import dbox.collision;
import dbox.collision.shapes;

/// A convex polygon. It is assumed that the interior of the polygon is to
/// the left of each edge.
/// Polygons have a maximum number of vertices equal to b2_maxPolygonVertices.
/// In most cases you should not need many vertices for a convex polygon.
class b2PolygonShape : b2Shape
{
    ///
    this()
    {
        m_type   = e_polygon;
        m_radius = b2_polygonRadius;
        m_count  = 0;
        m_centroid.SetZero();
    }

    /// Implement b2Shape.
    override b2PolygonShape Clone(b2BlockAllocator* allocator) const
    {
        void* mem = allocator.Allocate(b2memSizeOf!b2PolygonShape);
        b2PolygonShape clone = b2emplace!b2PolygonShape(mem);

        // clone.tupleof = this.tupleof;
        // Note: see Issue 12791: .tupleof does not take base class fields into account
        // https://issues.dlang.org/show_bug.cgi?id=12791
        clone.m_type = this.m_type;
        clone.m_radius = this.m_radius;

        clone.m_centroid = this.m_centroid;
        clone.m_vertices[] = this.m_vertices[];
        clone.m_normals[] = this.m_normals[];
        clone.m_count = this.m_count;

        return clone;
    }

    /// @see b2Shape.GetChildCount
    override int32 GetChildCount() const
    {
        return 1;
    }

    /// Create a convex hull from the given array of local points.
    /// The count must be in the range [3, b2_maxPolygonVertices].
    /// @warning the points may be re-ordered, even if they form a convex polygon
    /// @warning collinear points are handled but not removed. Collinear points
    /// may lead to poor stacking behavior.
    void Set(const(b2Vec2)* vertices, int32 count)
    {
        assert(3 <= count && count <= b2_maxPolygonVertices);

        if (count < 3)
        {
            SetAsBox(1.0f, 1.0f);
            return;
        }

        int32 n = b2Min(count, b2_maxPolygonVertices);

        // Perform welding and copy vertices into local buffer.
        b2Vec2[b2_maxPolygonVertices] ps;
        int32 tempCount = 0;

        for (int32 i = 0; i < n; ++i)
        {
            b2Vec2 v = vertices[i];

            bool unique = true;

            for (int32 j = 0; j < tempCount; ++j)
            {
                if (b2DistanceSquared(v, ps[j]) < 0.5f * b2_linearSlop)
                {
                    unique = false;
                    break;
                }
            }

            if (unique)
            {
                ps[tempCount++] = v;
            }
        }

        n = tempCount;

        if (n < 3)
        {
            // Polygon is degenerate.
            SetAsBox(1.0f, 1.0f);
            assert(0);
        }

        // Create the convex hull using the Gift wrapping algorithm
        // http://en.wikipedia.org/wiki/Gift_wrapping_algorithm

        // Find the right most point on the hull
        int32 i0   = 0;
        float32 x0 = ps[0].x;

        for (int32 i = 1; i < n; ++i)
        {
            float32 x = ps[i].x;

            if (x > x0 || (x == x0 && ps[i].y < ps[i0].y))
            {
                i0 = i;
                x0 = x;
            }
        }

        int32[b2_maxPolygonVertices] hull;
        int32 m  = 0;
        int32 ih = i0;

        for (;;)
        {
            hull[m] = ih;

            int32 ie = 0;

            for (int32 j = 1; j < n; ++j)
            {
                if (ie == ih)
                {
                    ie = j;
                    continue;
                }

                b2Vec2  r = ps[ie] - ps[hull[m]];
                b2Vec2  v = ps[j] - ps[hull[m]];
                float32 c = b2Cross(r, v);

                if (c < 0.0f)
                {
                    ie = j;
                }

                // Collinearity check
                if (c == 0.0f && v.LengthSquared() > r.LengthSquared())
                {
                    ie = j;
                }
            }

            ++m;
            ih = ie;

            if (ie == i0)
            {
                break;
            }
        }

        if (m < 3)
        {
            // Polygon is degenerate.
            SetAsBox(1.0f, 1.0f);
            assert(0);
        }

        m_count = m;

        // Copy vertices.
        for (int32 i = 0; i < m; ++i)
        {
            m_vertices[i] = ps[hull[i]];
        }

        // Compute normals. Ensure the edges have non-zero length.
        for (int32 i = 0; i < m; ++i)
        {
            int32  i1   = i;
            int32  i2   = i + 1 < m ? i + 1 : 0;
            b2Vec2 edge = m_vertices[i2] - m_vertices[i1];
            assert(edge.LengthSquared() > b2_epsilon * b2_epsilon);
            m_normals[i] = b2Cross(edge, 1.0f);
            m_normals[i].Normalize();
        }

        // Compute the polygon centroid.
        m_centroid = ComputeCentroid(m_vertices.ptr, m);
    }

    /// Convenience overload that takes a slice.
    void Set(const(b2Vec2)[] vertices)
    {
        Set(vertices.ptr, cast(int32)vertices.length);
    }

    /// Build vertices to represent an axis-aligned box centered on the local origin.
    /// @param hx the half-width.
    /// @param hy the half-height.
    void SetAsBox(float32 hx, float32 hy)
    {
        m_count = 4;
        m_vertices[0].Set(-hx, -hy);
        m_vertices[1].Set(hx, -hy);
        m_vertices[2].Set(hx, hy);
        m_vertices[3].Set(-hx, hy);
        m_normals[0].Set(0.0f, -1.0f);
        m_normals[1].Set(1.0f, 0.0f);
        m_normals[2].Set(0.0f, 1.0f);
        m_normals[3].Set(-1.0f, 0.0f);
        m_centroid.SetZero();
    }

    /// Build vertices to represent an oriented box.
    /// @param hx the half-width.
    /// @param hy the half-height.
    /// @param center the center of the box in local coordinates.
    /// @param angle the rotation of the box in local coordinates.
    void SetAsBox(float32 hx, float32 hy, b2Vec2 center, float32 angle)
    {
        m_count = 4;
        m_vertices[0].Set(-hx, -hy);
        m_vertices[1].Set(hx, -hy);
        m_vertices[2].Set(hx, hy);
        m_vertices[3].Set(-hx, hy);
        m_normals[0].Set(0.0f, -1.0f);
        m_normals[1].Set(1.0f, 0.0f);
        m_normals[2].Set(0.0f, 1.0f);
        m_normals[3].Set(-1.0f, 0.0f);
        m_centroid = center;

        b2Transform xf;
        xf.p = center;
        xf.q.Set(angle);

        // Transform vertices and normals.
        for (int32 i = 0; i < m_count; ++i)
        {
            m_vertices[i] = b2Mul(xf, m_vertices[i]);
            m_normals[i]  = b2Mul(xf.q, m_normals[i]);
        }
    }

    /// @see b2Shape.TestPoint
    override bool TestPoint(b2Transform xf, b2Vec2 p) const
    {
        b2Vec2 pLocal = b2MulT(xf.q, p - xf.p);

        for (int32 i = 0; i < m_count; ++i)
        {
            float32 dot = b2Dot(m_normals[i], pLocal - m_vertices[i]);

            if (dot > 0.0f)
            {
                return false;
            }
        }

        return true;
    }

    /// Implement b2Shape.
    override bool RayCast(b2RayCastOutput* output, b2RayCastInput input,
                          b2Transform xf, int32 childIndex) const
    {
        B2_NOT_USED(childIndex);

        // Put the ray into the polygon's frame of reference.
        b2Vec2 p1 = b2MulT(xf.q, input.p1 - xf.p);
        b2Vec2 p2 = b2MulT(xf.q, input.p2 - xf.p);
        b2Vec2 d  = p2 - p1;

        float32 lower = 0.0f, upper = input.maxFraction;

        int32 index = -1;

        for (int32 i = 0; i < m_count; ++i)
        {
            // p = p1 + a * d
            // dot(normal, p - v) = 0
            // dot(normal, p1 - v) + a * dot(normal, d) = 0
            float32 numerator   = b2Dot(m_normals[i], m_vertices[i] - p1);
            float32 denominator = b2Dot(m_normals[i], d);

            if (denominator == 0.0f)
            {
                if (numerator < 0.0f)
                {
                    return false;
                }
            }
            else
            {
                // Note: we want this predicate without division:
                // lower < numerator / denominator, where denominator < 0
                // Since denominator < 0, we have to flip the inequality:
                // lower < numerator / denominator <==> denominator * lower > numerator.
                if (denominator < 0.0f && numerator < lower * denominator)
                {
                    // Increase lower.
                    // The segment enters this half-space.
                    lower = numerator / denominator;
                    index = i;
                }
                else if (denominator > 0.0f && numerator < upper * denominator)
                {
                    // Decrease upper.
                    // The segment exits this half-space.
                    upper = numerator / denominator;
                }
            }

            // The use of epsilon here causes the assert on lower to trip
            // in some cases. Apparently the use of epsilon was to make edge
            // shapes work, but now those are handled separately.
            // if (upper < lower - b2_epsilon)
            if (upper < lower)
            {
                return false;
            }
        }

        assert(0.0f <= lower && lower <= input.maxFraction);

        if (index >= 0)
        {
            output.fraction = lower;
            output.normal   = b2Mul(xf.q, m_normals[index]);
            return true;
        }

        return false;
    }

    /// @see b2Shape.ComputeAABB
    override void ComputeAABB(b2AABB* aabb, b2Transform xf, int32 childIndex) const
    {
        B2_NOT_USED(childIndex);

        b2Vec2 lower = b2Mul(xf, m_vertices[0]);
        b2Vec2 upper = lower;

        for (int32 i = 1; i < m_count; ++i)
        {
            b2Vec2 v = b2Mul(xf, m_vertices[i]);
            lower = b2Min(lower, v);
            upper = b2Max(upper, v);
        }

        b2Vec2 r = b2Vec2(m_radius, m_radius);
        aabb.lowerBound = lower - r;
        aabb.upperBound = upper + r;
    }

    /// @see b2Shape.ComputeMass
    override void ComputeMass(b2MassData* massData, float32 density) const
    {
        // Polygon mass, centroid, and inertia.
        // Let rho be the polygon density in mass per unit area.
        // Then:
        // mass = rho * int(dA)
        // centroid.x = (1/mass) * rho * int(x * dA)
        // centroid.y = (1/mass) * rho * int(y * dA)
        // I = rho * int((x*x + y*y) * dA)
        //
        // We can compute these integrals by summing all the integrals
        // for each triangle of the polygon. To evaluate the integral
        // for a single triangle, we make a change of variables to
        // the (u,v) coordinates of the triangle:
        // x = x0 + e1x * u + e2x * v
        // y = y0 + e1y * u + e2y * v
        // where 0 <= u && 0 <= v && u + v <= 1.
        //
        // We integrate u from [0,1-v] and then v from [0,1].
        // We also need to use the Jacobian of the transformation:
        // D = cross(e1, e2)
        //
        // Simplification: triangle centroid = (1/3) * (p1 + p2 + p3)
        //
        // The rest of the derivation is handled by computer algebra.

        assert(m_count >= 3);

        b2Vec2 center;
        center.Set(0.0f, 0.0f);
        float32 area = 0.0f;
        float32 I    = 0.0f;

        // s is the reference point for forming triangles.
        // It's location doesn't change the result (except for rounding error).
        b2Vec2 s = b2Vec2(0.0f, 0.0f);

        // This code would put the reference point inside the polygon.
        for (int32 i = 0; i < m_count; ++i)
        {
            s += m_vertices[i];
        }

        s *= 1.0f / m_count;

        const float32 k_inv3 = 1.0f / 3.0f;

        for (int32 i = 0; i < m_count; ++i)
        {
            // Triangle vertices.
            b2Vec2 e1 = m_vertices[i] - s;
            b2Vec2 e2 = i + 1 < m_count ? m_vertices[i + 1] - s : m_vertices[0] - s;

            float32 D = b2Cross(e1, e2);

            float32 triangleArea = 0.5f * D;
            area += triangleArea;

            // Area weighted centroid
            center += triangleArea * k_inv3 * (e1 + e2);

            float32 ex1 = e1.x, ey1 = e1.y;
            float32 ex2 = e2.x, ey2 = e2.y;

            float32 intx2 = ex1 * ex1 + ex2 * ex1 + ex2 * ex2;
            float32 inty2 = ey1 * ey1 + ey2 * ey1 + ey2 * ey2;

            I += (0.25f * k_inv3 * D) * (intx2 + inty2);
        }

        // Total mass
        massData.mass = density * area;

        // Center of mass
        assert(area > b2_epsilon);
        center *= 1.0f / area;
        massData.center = center + s;

        // Inertia tensor relative to the local origin (point s).
        massData.I = density * I;

        // Shift to center of mass then to original body origin.
        massData.I += massData.mass * (b2Dot(massData.center, massData.center) - b2Dot(center, center));
    }

    /// Get the vertex count.
    int32 GetVertexCount() const
    {
        return m_count;
    }

    /// Get a vertex by index.
    b2Vec2 GetVertex(int32 index) const
    {
        assert(0 <= index && index < m_count);
        return m_vertices[index];
    }

    /// Validate convexity. This is a very time consuming operation.
    /// @returns true if valid
    bool Validate() const
    {
        for (int32 i = 0; i < m_count; ++i)
        {
            int32  i1 = i;
            int32  i2 = i < m_count - 1 ? i1 + 1 : 0;
            b2Vec2 p  = m_vertices[i1];
            b2Vec2 e  = m_vertices[i2] - p;

            for (int32 j = 0; j < m_count; ++j)
            {
                if (j == i1 || j == i2)
                {
                    continue;
                }

                b2Vec2  v = m_vertices[j] - p;
                float32 c = b2Cross(e, v);

                if (c < 0.0f)
                {
                    return false;
                }
            }
        }

        return true;
    }

    ///
    static b2Vec2 ComputeCentroid(const(b2Vec2)* vs, int32 count)
    {
        assert(count >= 3);

        b2Vec2 c;
        c.Set(0.0f, 0.0f);
        float32 area = 0.0f;

        // pRef is the reference point for forming triangles.
        // It's location doesn't change the result (except for rounding error).
        b2Vec2 pRef = b2Vec2(0.0f, 0.0f);

        version (none)
        {
            // This code would put the reference point inside the polygon.
            for (int32 i = 0; i < count; ++i)
            {
                pRef += vs[i];
            }

            pRef *= 1.0f / count;
        }

        const float32 inv3 = 1.0f / 3.0f;

        for (int32 i = 0; i < count; ++i)
        {
            // Triangle vertices.
            b2Vec2 p1 = pRef;
            b2Vec2 p2 = vs[i];
            b2Vec2 p3 = i + 1 < count ? vs[i + 1] : vs[0];

            b2Vec2 e1 = p2 - p1;
            b2Vec2 e2 = p3 - p1;

            float32 D = b2Cross(e1, e2);

            float32 triangleArea = 0.5f * D;
            area += triangleArea;

            // Area weighted centroid
            c += triangleArea * inv3 * (p1 + p2 + p3);
        }

        // Centroid
        assert(area > b2_epsilon);
        c *= 1.0f / area;
        return c;
    }

    /// Convenience overload that takes a slice.
    static b2Vec2 ComputeCentroid(const(b2Vec2)[] vertices)
    {
        return ComputeCentroid(vertices.ptr, cast(int32)vertices.length);
    }

    b2Vec2 m_centroid;
    b2Vec2[b2_maxPolygonVertices] m_vertices;
    b2Vec2[b2_maxPolygonVertices] m_normals;
    int32  m_count;
}
