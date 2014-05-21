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
module dbox.dynamics.b2fixture;

import core.stdc.float_;
import core.stdc.stdlib;
import core.stdc.string;

import dbox.collision;
import dbox.collision.shapes;
import dbox.common;
import dbox.dynamics;
import dbox.dynamics.contacts;
import dbox.dynamics.joints;

/// This holds contact filtering data.
struct b2Filter
{
    /// The collision category bits. Normally you would just set one bit.
    uint16 categoryBits = 0x0001;

    /// The collision mask bits. This states the categories that this
    /// shape would accept for collision.
    uint16 maskBits = 0xFFFF;

    /// Collision groups allow a certain group of objects to never collide (negative)
    /// or always collide (positive). Zero means no collision group. Non-zero group
    /// filtering always wins against the mask bits.
    int16 groupIndex = 0;
}

/// A fixture definition is used to create a fixture. This class defines an
/// abstract fixture definition. You can reuse fixture definitions safely.
struct b2FixtureDef
{
    /// The shape, this must be set. The shape will be cloned, so you
    /// can create the shape on the stack.
    b2Shape shape;

    /// Use this to store application specific fixture data.
    void* userData;

    /// The friction coefficient, usually in the range [0,1].
    float32 friction = 0.2;

    /// The restitution (elasticity) usually in the range [0,1].
    float32 restitution = 0;

    /// The density, usually in kg/m^2.
    float32 density = 0;

    /// A sensor shape collects contact information but never generates a collision
    /// response.
    bool isSensor;

    /// Contact filtering data.
    b2Filter filter;
}

/// This proxy is used internally to connect fixtures to the broad-phase.
struct b2FixtureProxy
{
    b2AABB aabb;
    b2Fixture* fixture;
    int32 childIndex;
    int32 proxyId;
}

/// A fixture is used to attach a shape to a body for collision detection. A fixture
/// inherits its transform from its parent. Fixtures hold additional non-geometric data
/// such as friction, collision filters, etc.
/// Fixtures are created via b2Body.CreateFixture.
/// @warning you cannot reuse fixtures.
struct b2Fixture
{
    /// Get the type of the child shape. You can use this to down cast to the concrete shape.
    /// @return the shape type.
    b2Shape.Type GetType() const
    {
        return m_shape.GetType();
    }

    /// Get the child shape. You can modify the child shape, however you should not change the
    /// number of vertices because this will crash some collision caching mechanisms.
    /// Manipulating the shape may lead to non-physical behavior.
    inout(b2Shape) GetShape() inout
    {
        return m_shape;
    }

    /// Is this fixture a sensor (non-solid)?
    /// @return the true if the shape is a sensor.
    bool IsSensor() const
    {
        return m_isSensor;
    }

    /// Set if this fixture is a sensor.
    void SetSensor(bool sensor)
    {
        if (sensor != m_isSensor)
        {
            m_body.SetAwake(true);
            m_isSensor = sensor;
        }
    }

    /// Get the contact filtering data.
    b2Filter GetFilterData() const
    {
        return m_filter;
    }

    /// Set the contact filtering data. This will not update contacts until the next time
    /// step when either parent body is active and awake.
    /// This automatically calls Refilter.
    void SetFilterData(b2Filter filter)
    {
        m_filter = filter;

        Refilter();
    }

    /// Call this if you want to establish collision that was previously disabled by b2ContactFilter.ShouldCollide.
    void Refilter()
    {
        if (m_body is null)
        {
            return;
        }

        // Flag associated contacts for filtering.
        b2ContactEdge* edge = m_body.GetContactList();

        while (edge)
        {
            b2Contact contact  = edge.contact;
            b2Fixture* fixtureA = contact.GetFixtureA();
            b2Fixture* fixtureB = contact.GetFixtureB();

            if (fixtureA == &this || fixtureB == &this)
            {
                contact.FlagForFiltering();
            }

            edge = edge.next;
        }

        b2World* world = m_body.GetWorld();

        if (world is null)
        {
            return;
        }

        // Touch each proxy so that new pairs may be created
        b2BroadPhase* broadPhase = &world.m_contactManager.m_broadPhase;

        for (int32 i = 0; i < m_proxyCount; ++i)
        {
            broadPhase.TouchProxy(m_proxies[i].proxyId);
        }
    }

    /// Get the parent body of this fixture. This is NULL if the fixture is not attached.
    /// @return the parent body.
    inout(b2Body)* GetBody() inout
    {
        return m_body;
    }

    /// Get the next fixture in the parent body's fixture list.
    /// @return the next shape.
    inout(b2Fixture)* GetNext() inout
    {
        return m_next;
    }

    /// Get the user data that was assigned in the fixture definition. Use this to
    /// store your application specific data.
    void* GetUserData() const
    {
        return cast(void*)m_userData;
    }

    /// Set the user data. Use this to store your application specific data.
    void SetUserData(void* data)
    {
        m_userData = data;
    }

    /// Test a point for containment in this fixture.
    /// @param p a point in world coordinates.
    bool TestPoint(b2Vec2 p) const
    {
        return m_shape.TestPoint(m_body.GetTransform(), p);
    }

    /// Cast a ray against this shape.
    /// @param output the ray-cast results.
    /// @param input the ray-cast input parameters.
    bool RayCast(b2RayCastOutput* output, b2RayCastInput input, int32 childIndex) const
    {
        return m_shape.RayCast(output, input, m_body.GetTransform(), childIndex);
    }

    /// Get the mass data for this fixture. The mass data is based on the density and
    /// the shape. The rotational inertia is about the shape's origin. This operation
    /// may be expensive.
    void GetMassData(b2MassData* massData) const
    {
        m_shape.ComputeMass(massData, m_density);
    }

    /// Get the density of this fixture.
    float32 GetDensity() const
    {
        return m_density;
    }

    /// Set the density of this fixture. This will _not_ automatically adjust the mass
    /// of the body. You must call b2Body.ResetMassData to update the body's mass.
    void SetDensity(float32 density)
    {
        assert(b2IsValid(density) && density >= 0.0f);
        m_density = density;
    }

    /// Get the coefficient of friction.
    float32 GetFriction() const
    {
        return m_friction;
    }

    /// Set the coefficient of friction. This will _not_ change the friction of
    /// existing contacts.
    void SetFriction(float32 friction)
    {
        m_friction = friction;
    }

    /// Get the coefficient of restitution.
    float32 GetRestitution() const
    {
        return m_restitution;
    }

    /// Set the coefficient of restitution. This will _not_ change the restitution of
    /// existing contacts.
    void SetRestitution(float32 restitution)
    {
        m_restitution = restitution;
    }

    /// Get the fixture's AABB. This AABB may be enlarge and/or stale.
    /// If you need a more accurate AABB, compute it using the shape and
    /// the body transform.
    b2AABB GetAABB(int32 childIndex) const
    {
        assert(0 <= childIndex && childIndex < m_proxyCount);
        return m_proxies[childIndex].aabb;
    }

    /// Dump this fixture to the log file.
    void Dump(int32 bodyIndex)
    {
        b2Log("    b2FixtureDef fd;\n");
        b2Log("    fd.friction = %.15lef;\n", m_friction);
        b2Log("    fd.restitution = %.15lef;\n", m_restitution);
        b2Log("    fd.density = %.15lef;\n", m_density);
        b2Log("    fd.isSensor = bool(%d);\n", m_isSensor);
        b2Log("    fd.filter.categoryBits = uint16(%d);\n", m_filter.categoryBits);
        b2Log("    fd.filter.maskBits = uint16(%d);\n", m_filter.maskBits);
        b2Log("    fd.filter.groupIndex = int16(%d);\n", m_filter.groupIndex);

        switch (m_shape.m_type)
        {
            case b2Shape.e_circle:
            {
                b2CircleShape s = cast(b2CircleShape)m_shape;
                b2Log("    b2CircleShape shape;\n");
                b2Log("    shape.m_radius = %.15lef;\n", s.m_radius);
                b2Log("    shape.m_p.Set(%.15lef, %.15lef);\n", s.m_p.x, s.m_p.y);
            }
            break;

            case b2Shape.e_edge:
            {
                b2EdgeShape s = cast(b2EdgeShape)m_shape;
                b2Log("    b2EdgeShape shape;\n");
                b2Log("    shape.m_radius = %.15lef;\n", s.m_radius);
                b2Log("    shape.m_vertex0.Set(%.15lef, %.15lef);\n", s.m_vertex0.x, s.m_vertex0.y);
                b2Log("    shape.m_vertex1.Set(%.15lef, %.15lef);\n", s.m_vertex1.x, s.m_vertex1.y);
                b2Log("    shape.m_vertex2.Set(%.15lef, %.15lef);\n", s.m_vertex2.x, s.m_vertex2.y);
                b2Log("    shape.m_vertex3.Set(%.15lef, %.15lef);\n", s.m_vertex3.x, s.m_vertex3.y);
                b2Log("    shape.m_hasVertex0 = bool(%d);\n", s.m_hasVertex0);
                b2Log("    shape.m_hasVertex3 = bool(%d);\n", s.m_hasVertex3);
            }
            break;

            case b2Shape.e_polygon:
            {
                b2PolygonShape s = cast(b2PolygonShape)m_shape;
                b2Log("    b2PolygonShape shape;\n");
                b2Log("    b2Vec2 vs[%d];\n", b2_maxPolygonVertices);

                for (int32 i = 0; i < s.m_count; ++i)
                {
                    b2Log("    vs[%d].Set(%.15lef, %.15lef);\n", i, s.m_vertices[i].x, s.m_vertices[i].y);
                }

                b2Log("    shape.Set(vs, %d);\n", s.m_count);
            }
            break;

            case b2Shape.e_chain:
            {
                b2ChainShape s = cast(b2ChainShape)m_shape;
                b2Log("    b2ChainShape shape;\n");
                b2Log("    b2Vec2 vs[%d];\n", s.m_count);

                for (int32 i = 0; i < s.m_count; ++i)
                {
                    b2Log("    vs[%d].Set(%.15lef, %.15lef);\n", i, s.m_vertices[i].x, s.m_vertices[i].y);
                }

                b2Log("    shape.CreateChain(vs, %d);\n", s.m_count);
                b2Log("    shape.m_prevVertex.Set(%.15lef, %.15lef);\n", s.m_prevVertex.x, s.m_prevVertex.y);
                b2Log("    shape.m_nextVertex.Set(%.15lef, %.15lef);\n", s.m_nextVertex.x, s.m_nextVertex.y);
                b2Log("    shape.m_hasPrevVertex = bool(%d);\n", s.m_hasPrevVertex);
                b2Log("    shape.m_hasNextVertex = bool(%d);\n", s.m_hasNextVertex);
            }
            break;

            default:
                return;
        }

        b2Log("\n");
        b2Log("    fd.shape = &shape;\n");
        b2Log("\n");
        b2Log("    bodies[%d].CreateFixture(&fd);\n", bodyIndex);
    }

// note: this should be package but D's access implementation is lacking.
// do not use in user code.
/* package: */
public:

    // We need separation create/destroy functions from the constructor/destructor because
    // the destructor cannot access the allocator (no destructor arguments allowed by D).
    void Create(b2BlockAllocator* allocator, b2Body* body_, const(b2FixtureDef)* def)
    {
        m_userData    = cast(void*)def.userData;
        m_friction    = def.friction;
        m_restitution = def.restitution;

        m_body = body_;
        m_next = null;

        m_filter = def.filter;

        m_isSensor = def.isSensor;

        m_shape = def.shape.Clone(allocator);

        // Reserve proxy space
        int32 childCount = m_shape.GetChildCount();
        m_proxies = cast(b2FixtureProxy*)allocator.Allocate(childCount * b2memSizeOf!b2FixtureProxy);

        for (int32 i = 0; i < childCount; ++i)
        {
            m_proxies[i].fixture = null;
            m_proxies[i].proxyId = b2BroadPhase.e_nullProxy;
        }

        m_proxyCount = 0;

        m_density = def.density;
    }

    void Destroy(b2BlockAllocator* allocator)
    {
        // The proxies must be destroyed before calling this.
        assert(m_proxyCount == 0);

        // Free the proxy array.
        int32 childCount = m_shape.GetChildCount();
        allocator.Free(cast(void*)m_proxies, childCount * b2memSizeOf!b2FixtureProxy);
        m_proxies = null;

        // Free the child shape.
        switch (m_shape.m_type)
        {
            case b2Shape.e_circle:
            {
                b2CircleShape s = cast(b2CircleShape)m_shape;
                destroy(s);
                allocator.Free(cast(void*)s, b2memSizeOf!b2CircleShape);
            }
            break;

            case b2Shape.e_edge:
            {
                b2EdgeShape s = cast(b2EdgeShape)m_shape;
                destroy(s);
                allocator.Free(cast(void*)s, b2memSizeOf!b2EdgeShape);
            }
            break;

            case b2Shape.e_polygon:
            {
                b2PolygonShape s = cast(b2PolygonShape)m_shape;
                destroy(s);
                allocator.Free(cast(void*)s, b2memSizeOf!b2PolygonShape);
            }
            break;

            case b2Shape.e_chain:
            {
                b2ChainShape s = cast(b2ChainShape)m_shape;
                destroy(s);
                allocator.Free(cast(void*)s, b2memSizeOf!b2ChainShape);
            }
            break;

            default:
                assert(0);
        }

        m_shape = null;
    }

    // These support body activation/deactivation.
    void CreateProxies(b2BroadPhase* broadPhase, b2Transform xf)
    {
        assert(m_proxyCount == 0);

        // Create proxies in the broad-phase.
        m_proxyCount = m_shape.GetChildCount();

        for (int32 i = 0; i < m_proxyCount; ++i)
        {
            b2FixtureProxy* proxy = m_proxies + i;
            m_shape.ComputeAABB(&proxy.aabb, xf, i);
            proxy.proxyId    = broadPhase.CreateProxy(proxy.aabb, proxy);
            proxy.fixture    = &this;
            proxy.childIndex = i;
        }
    }

    void DestroyProxies(b2BroadPhase* broadPhase)
    {
        // Destroy proxies in the broad-phase.
        for (int32 i = 0; i < m_proxyCount; ++i)
        {
            b2FixtureProxy* proxy = m_proxies + i;
            broadPhase.DestroyProxy(proxy.proxyId);
            proxy.proxyId = b2BroadPhase.e_nullProxy;
        }

        m_proxyCount = 0;
    }

    void Synchronize(b2BroadPhase* broadPhase, b2Transform transform1, b2Transform transform2)
    {
        if (m_proxyCount == 0)
        {
            return;
        }

        for (int32 i = 0; i < m_proxyCount; ++i)
        {
            b2FixtureProxy* proxy = m_proxies + i;

            // Compute an AABB that covers the swept shape (may miss some rotation effect).
            b2AABB aabb1, aabb2;
            m_shape.ComputeAABB(&aabb1, transform1, proxy.childIndex);
            m_shape.ComputeAABB(&aabb2, transform2, proxy.childIndex);

            proxy.aabb.Combine(aabb1, aabb2);

            b2Vec2 displacement = transform2.p - transform1.p;

            broadPhase.MoveProxy(proxy.proxyId, proxy.aabb, displacement);
        }
    }

    float32 m_density = 0;

    b2Fixture* m_next;
    b2Body* m_body;

    b2Shape m_shape;

    float32 m_friction = 0;
    float32 m_restitution = 0;

    b2FixtureProxy* m_proxies;
    int32 m_proxyCount;

    b2Filter m_filter;

    bool m_isSensor;

    void* m_userData;
}
