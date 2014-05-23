/*
 * Copyright (c) 2006-2010 Erin Catto http://www.box2d.org
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
module dbox.collision.shapes.b2chainshape;

import core.stdc.string;

import std.typecons;

import dbox.common;
import dbox.collision;
import dbox.collision.shapes;

/// A chain shape is a free form sequence of line segments.
/// The chain has two-sided collision, so you can use inside and outside collision.
/// Therefore, you may use any winding order.
/// Since there may be many vertices, they are allocated using b2Alloc.
/// Connectivity information is used to create smooth collisions.
/// WARNING: The chain will not collide properly if there are self-intersections.
class b2ChainShape : b2Shape
{
    ///
    this()
    {
        m_type          = e_chain;
        m_radius        = b2_polygonRadius;
        m_vertices      = null;
        m_count         = 0;
        m_hasPrevVertex = false;
        m_hasNextVertex = false;
    }

    /// The destructor frees the vertices using b2Free.
    ~this()
    {
        Clear();
    }

    /// Clear all data.
    void Clear()
    {
        b2Free(m_vertices);
        m_vertices = null;
        m_count    = 0;
    }

    /// Create a loop. This automatically adjusts connectivity.
    /// @param vertices an array of vertices, these are copied
    /// @param count the vertex count
    void CreateLoop(const(b2Vec2)* vertices, int32 count)
    {
        assert(m_vertices is null && m_count == 0);
        assert(count >= 3);

        for (int32 i = 1; i < count; ++i)
        {
            b2Vec2 v1 = vertices[i - 1];
            b2Vec2 v2 = vertices[i];

            // If the code crashes here, it means your vertices are too close together.
            assert(b2DistanceSquared(v1, v2) > b2_linearSlop * b2_linearSlop);
        }

        m_count    = count + 1;
        m_vertices = cast(b2Vec2*)b2Alloc(m_count * b2memSizeOf!b2Vec2);
        memcpy(m_vertices, vertices, count * b2memSizeOf!b2Vec2);
        m_vertices[count] = m_vertices[0];
        m_prevVertex      = m_vertices[m_count - 2];
        m_nextVertex      = m_vertices[1];
        m_hasPrevVertex   = true;
        m_hasNextVertex   = true;
    }

    /// Convenience overload that takes a slice.
    void CreateLoop(const(b2Vec2)[] vertices)
    {
        CreateLoop(vertices.ptr, cast(int32)vertices.length);
    }

    /// Create a chain with isolated end vertices.
    /// @param vertices an array of vertices, these are copied
    /// @param count the vertex count
    void CreateChain(const(b2Vec2)* vertices, int32 count)
    {
        assert(m_vertices is null && m_count == 0);
        assert(count >= 2);

        for (int32 i = 1; i < count; ++i)
        {
            // If the code crashes here, it means your vertices are too close together.
            assert(b2DistanceSquared(vertices[i - 1], vertices[i]) > b2_linearSlop * b2_linearSlop);
        }

        m_count    = count;
        m_vertices = cast(b2Vec2*)b2Alloc(count * b2memSizeOf!b2Vec2);
        memcpy(m_vertices, vertices, m_count * b2memSizeOf!b2Vec2);

        m_hasPrevVertex = false;
        m_hasNextVertex = false;

        m_prevVertex.SetZero();
        m_nextVertex.SetZero();
    }

    /// Convenience overload that takes a slice.
    void CreateChain(const(b2Vec2)[] vertices)
    {
        CreateChain(vertices.ptr, cast(int32)vertices.length);
    }

    /// Establish connectivity to a vertex that precedes the first vertex.
    /// Don't call this for loops.
    void SetPrevVertex(b2Vec2 prevVertex)
    {
        m_prevVertex    = prevVertex;
        m_hasPrevVertex = true;
    }

    /// Establish connectivity to a vertex that follows the last vertex.
    /// Don't call this for loops.
    void SetNextVertex(b2Vec2 nextVertex)
    {
        m_nextVertex    = nextVertex;
        m_hasNextVertex = true;
    }

    /// Implement b2Shape. Vertices are cloned using b2Alloc.
    override b2Shape Clone(b2BlockAllocator* allocator) const
    {
        void* mem = allocator.Allocate(b2memSizeOf!b2ChainShape);
        b2ChainShape clone = b2emplace!b2ChainShape(mem);
        clone.CreateChain(m_vertices, m_count);
        clone.m_prevVertex    = m_prevVertex;
        clone.m_nextVertex    = m_nextVertex;
        clone.m_hasPrevVertex = m_hasPrevVertex;
        clone.m_hasNextVertex = m_hasNextVertex;
        return clone;
    }

    /// @see b2Shape.GetChildCount
    override int32 GetChildCount() const
    {
        // edge count = vertex count - 1
        return m_count - 1;
    }

    /// Get a child edge.
    void GetChildEdge(b2EdgeShape edge, int32 index) const
    {
        assert(0 <= index && index < m_count - 1);
        edge.m_type   = b2Shape.e_edge;
        edge.m_radius = m_radius;

        edge.m_vertex1 = m_vertices[index + 0];
        edge.m_vertex2 = m_vertices[index + 1];

        if (index > 0)
        {
            edge.m_vertex0    = m_vertices[index - 1];
            edge.m_hasVertex0 = true;
        }
        else
        {
            edge.m_vertex0    = m_prevVertex;
            edge.m_hasVertex0 = m_hasPrevVertex;
        }

        if (index < m_count - 2)
        {
            edge.m_vertex3    = m_vertices[index + 2];
            edge.m_hasVertex3 = true;
        }
        else
        {
            edge.m_vertex3    = m_nextVertex;
            edge.m_hasVertex3 = m_hasNextVertex;
        }
    }

    /// This always return false.
    /// @see b2Shape.TestPoint
    override bool TestPoint(b2Transform xf, b2Vec2 p) const
    {
        B2_NOT_USED(xf);
        B2_NOT_USED(p);
        return false;
    }

    /// Implement b2Shape.
    override bool RayCast(b2RayCastOutput* output, b2RayCastInput input,
                          b2Transform xf, int32 childIndex) const
    {
        assert(childIndex < m_count);

        auto edgeShape = scoped!b2EdgeShape();

        int32 i1 = childIndex;
        int32 i2 = childIndex + 1;

        if (i2 == m_count)
        {
            i2 = 0;
        }

        edgeShape.m_vertex1 = m_vertices[i1];
        edgeShape.m_vertex2 = m_vertices[i2];

        return edgeShape.RayCast(output, input, xf, 0);
    }

    /// @see b2Shape.ComputeAABB
    override void ComputeAABB(b2AABB* aabb, b2Transform xf, int32 childIndex) const
    {
        assert(childIndex < m_count);

        int32 i1 = childIndex;
        int32 i2 = childIndex + 1;

        if (i2 == m_count)
        {
            i2 = 0;
        }

        b2Vec2 v1 = b2Mul(xf, m_vertices[i1]);
        b2Vec2 v2 = b2Mul(xf, m_vertices[i2]);

        aabb.lowerBound = b2Min(v1, v2);
        aabb.upperBound = b2Max(v1, v2);
    }

    /// Chains have zero mass.
    /// @see b2Shape.ComputeMass
    override void ComputeMass(b2MassData* massData, float32 density) const
    {
        B2_NOT_USED(density);

        massData.mass = 0.0f;
        massData.center.SetZero();
        massData.I = 0.0f;
    }

    /// The vertices. Owned by this class.
    b2Vec2* m_vertices;

    /// The vertex count.
    int32 m_count;

    b2Vec2 m_prevVertex, m_nextVertex;
    bool m_hasPrevVertex, m_hasNextVertex;
}
