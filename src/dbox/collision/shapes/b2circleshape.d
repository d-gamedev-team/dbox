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
module dbox.collision.shapes.b2circleshape;

import dbox.common;

import dbox.collision;
import dbox.collision.shapes;

/// A circle shape.
class b2CircleShape : b2Shape
{
    ///
    this()
    {
        m_type   = e_circle;
        m_radius = 0.0f;
        m_p.SetZero();
    }

    /// Implement b2Shape.
    override b2CircleShape Clone(b2BlockAllocator* allocator) const
    {
        void* mem = allocator.Allocate(b2memSizeOf!b2CircleShape);
        b2CircleShape clone = b2emplace!b2CircleShape(mem);

        // clone.tupleof = this.tupleof;
        // Note: see Issue 12791: .tupleof does not take base class fields into account
        // https://issues.dlang.org/show_bug.cgi?id=12791
        clone.m_type = this.m_type;
        clone.m_radius = this.m_radius;

        clone.m_p = this.m_p;

        return clone;
    }

    /// @see b2Shape.GetChildCount
    override int32 GetChildCount() const
    {
        return 1;
    }

    /// Implement b2Shape.
    override bool TestPoint(b2Transform transform, b2Vec2 p) const
    {
        b2Vec2 center = transform.p + b2Mul(transform.q, m_p);
        b2Vec2 d      = p - center;
        return b2Dot(d, d) <= m_radius * m_radius;
    }

    // Collision Detection in Interactive 3D Environments by Gino van den Bergen
    // From Section 3.1.2
    // x = s + a * r
    // norm(x) = radius
    /// Implement b2Shape.
    override bool RayCast(b2RayCastOutput* output, b2RayCastInput input,
                          b2Transform transform, int32 childIndex) const
    {
        B2_NOT_USED(childIndex);

        b2Vec2  position = transform.p + b2Mul(transform.q, m_p);
        b2Vec2  s        = input.p1 - position;
        float32 b        = b2Dot(s, s) - m_radius * m_radius;

        // Solve quadratic equation.
        b2Vec2  r     = input.p2 - input.p1;
        float32 c     = b2Dot(s, r);
        float32 rr    = b2Dot(r, r);
        float32 sigma = c * c - rr * b;

        // Check for negative discriminant and short segment.
        if (sigma < 0.0f || rr < b2_epsilon)
        {
            return false;
        }

        // Find the point of intersection of the line with the circle.
        float32 a = -(c + b2Sqrt(sigma));

        // Is the intersection point on the segment?
        if (0.0f <= a && a <= input.maxFraction * rr)
        {
            a /= rr;
            output.fraction = a;
            output.normal   = s + a * r;
            output.normal.Normalize();
            return true;
        }

        return false;
    }

    /// @see b2Shape.ComputeAABB
    override void ComputeAABB(b2AABB* aabb, b2Transform transform, int32 childIndex) const
    {
        B2_NOT_USED(childIndex);

        b2Vec2 p = transform.p + b2Mul(transform.q, m_p);
        aabb.lowerBound.Set(p.x - m_radius, p.y - m_radius);
        aabb.upperBound.Set(p.x + m_radius, p.y + m_radius);
    }

    /// @see b2Shape.ComputeMass
    override void ComputeMass(b2MassData* massData, float32 density) const
    {
        massData.mass   = density * b2_pi * m_radius * m_radius;
        massData.center = m_p;

        // inertia about the local origin
        massData.I = massData.mass * (0.5f * m_radius * m_radius + b2Dot(m_p, m_p));
    }

    /// Get the supporting vertex index in the given direction.
    int32 GetSupport(b2Vec2 d) const
    {
        B2_NOT_USED(d);
        return 0;
    }

    /// Get the supporting vertex in the given direction.
    b2Vec2 GetSupportVertex(b2Vec2 d) const
    {
        B2_NOT_USED(d);
        return m_p;
    }

    /// Get the vertex count.
    int32 GetVertexCount() const
    {
        return 1;
    }

    /// Get a vertex by index. Used by b2Distance.
    b2Vec2 GetVertex(int32 index) const
    {
        B2_NOT_USED(index);
        assert(index == 0);
        return m_p;
    }

    /// Position
    b2Vec2 m_p;
}
