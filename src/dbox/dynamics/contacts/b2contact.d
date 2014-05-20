module dbox.dynamics.contacts.b2contact;

import core.stdc.float_;
import core.stdc.stdlib;
import core.stdc.string;

import dbox.common;
import dbox.collision.shapes;
import dbox.common.b2math;
import dbox.dynamics;
import dbox.collision;

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

// #ifndef B2_CONTACT_H
// #define B2_CONTACT_H

import dbox.common.b2math;
import dbox.collision.b2collision;
import dbox.collision.shapes.b2shape;
import dbox.dynamics.b2fixture;

/// Friction mixing law. The idea is to allow either fixture to drive the restitution to zero.
/// For example, anything slides on ice.
float32 b2MixFriction(float32 friction1, float32 friction2)
{
    return b2Sqrt(friction1 * friction2);
}

/// Restitution mixing law. The idea is allow for anything to bounce off an inelastic surface.
/// For example, a superball bounces on anything.
float32 b2MixRestitution(float32 restitution1, float32 restitution2)
{
    return restitution1 > restitution2 ? restitution1 : restitution2;
}

alias b2ContactCreateFcn = b2Contact function(b2Fixture* fixtureA, int32 indexA,
                                       b2Fixture* fixtureB, int32 indexB,
                                       b2BlockAllocator* allocator);
alias b2ContactDestroyFcn = void function(b2Contact contact, b2BlockAllocator* allocator);

struct b2ContactRegister
{
    b2ContactCreateFcn createFcn;
    b2ContactDestroyFcn destroyFcn;
    bool primary;
}

/// A contact edge is used to connect bodies and contacts together
/// in a contact graph where each body is a node and each contact
/// is an edge. A contact edge belongs to a doubly linked list
/// maintained in each attached body_. Each contact has two contact
/// nodes, one for each attached body_.
struct b2ContactEdge
{
    b2Body* other;              ///< provides quick access to the other body attached.
    b2Contact contact;         ///< the contact
    b2ContactEdge* prev;        ///< the previous contact edge in the body's contact list
    b2ContactEdge* next;        ///< the next contact edge in the body's contact list
}

/// The class manages contact between two shapes. A contact exists for each overlapping
/// AABB in the broad-phase (except if filtered). Therefore a contact object may exist
/// that has no contact points.
class b2Contact
{
    shared static this()
    {
        b2_defaultFilter = new b2ContactFilter();
        b2_defaultListener = new b2ContactListener();

        AddType(&b2CircleContact.Create, &b2CircleContact.Destroy, b2Shape.e_circle, b2Shape.e_circle);
        AddType(&b2PolygonAndCircleContact.Create, &b2PolygonAndCircleContact.Destroy, b2Shape.e_polygon, b2Shape.e_circle);
        AddType(&b2PolygonContact.Create, &b2PolygonContact.Destroy, b2Shape.e_polygon, b2Shape.e_polygon);
        AddType(&b2EdgeAndCircleContact.Create, &b2EdgeAndCircleContact.Destroy, b2Shape.e_edge, b2Shape.e_circle);
        AddType(&b2EdgeAndPolygonContact.Create, &b2EdgeAndPolygonContact.Destroy, b2Shape.e_edge, b2Shape.e_polygon);
        AddType(&b2ChainAndCircleContact.Create, &b2ChainAndCircleContact.Destroy, b2Shape.e_chain, b2Shape.e_circle);
        AddType(&b2ChainAndPolygonContact.Create, &b2ChainAndPolygonContact.Destroy, b2Shape.e_chain, b2Shape.e_polygon);
    }

    static void AddType(b2ContactCreateFcn createFcn, b2ContactDestroyFcn destroyFcn, b2Shape.Type type1, b2Shape.Type type2)
    {
        assert(0 <= type1 && type1 < b2Shape.e_typeCount);
        assert(0 <= type2 && type2 < b2Shape.e_typeCount);

        s_registers[type1][type2].createFcn  = createFcn;
        s_registers[type1][type2].destroyFcn = destroyFcn;
        s_registers[type1][type2].primary    = true;

        if (type1 != type2)
        {
            s_registers[type2][type1].createFcn  = createFcn;
            s_registers[type2][type1].destroyFcn = destroyFcn;
            s_registers[type2][type1].primary    = false;
        }
    }

    static b2Contact Create(b2Fixture* fixtureA, int32 indexA, b2Fixture* fixtureB, int32 indexB, b2BlockAllocator* allocator)
    {
        b2Shape.Type type1 = fixtureA.GetType();
        b2Shape.Type type2 = fixtureB.GetType();

        assert(0 <= type1 && type1 < b2Shape.e_typeCount);
        assert(0 <= type2 && type2 < b2Shape.e_typeCount);

        b2ContactCreateFcn createFcn = s_registers[type1][type2].createFcn;

        if (createFcn)
        {
            if (s_registers[type1][type2].primary)
            {
                return createFcn(fixtureA, indexA, fixtureB, indexB, allocator);
            }
            else
            {
                return createFcn(fixtureB, indexB, fixtureA, indexA, allocator);
            }
        }
        else
        {
            return null;
        }
    }

    static void Destroy(b2Contact contact, b2BlockAllocator* allocator)
    {
        b2Fixture* fixtureA = contact.m_fixtureA;
        b2Fixture* fixtureB = contact.m_fixtureB;

        if (contact.m_manifold.pointCount > 0 &&
            fixtureA.IsSensor() == false &&
            fixtureB.IsSensor() == false)
        {
            fixtureA.GetBody().SetAwake(true);
            fixtureB.GetBody().SetAwake(true);
        }

        b2Shape.Type typeA = fixtureA.GetType();
        b2Shape.Type typeB = fixtureB.GetType();

        assert(0 <= typeA && typeB < b2Shape.e_typeCount);
        assert(0 <= typeA && typeB < b2Shape.e_typeCount);

        b2ContactDestroyFcn destroyFcn = s_registers[typeA][typeB].destroyFcn;
        destroyFcn(contact, allocator);
    }

    this(b2Fixture* fA, int32 indexA, b2Fixture* fB, int32 indexB)
    {
        m_flags = e_enabledFlag;

        m_fixtureA = fA;
        m_fixtureB = fB;

        m_indexA = indexA;
        m_indexB = indexB;

        m_manifold.pointCount = 0;

        m_prev = null;
        m_next = null;

        m_nodeA.contact = null;
        m_nodeA.prev    = null;
        m_nodeA.next    = null;
        m_nodeA.other   = null;

        m_nodeB.contact = null;
        m_nodeB.prev    = null;
        m_nodeB.next    = null;
        m_nodeB.other   = null;

        m_toiCount = 0;

        m_friction    = b2MixFriction(m_fixtureA.m_friction, m_fixtureB.m_friction);
        m_restitution = b2MixRestitution(m_fixtureA.m_restitution, m_fixtureB.m_restitution);

        m_tangentSpeed = 0.0f;
    }

    // Update the contact manifold and touching status.
    // Note: do not assume the fixture AABBs are overlapping or are valid.
    void Update(b2ContactListener listener)
    {
        b2Manifold oldManifold = m_manifold;

        // Re-enable this contact.
        m_flags |= e_enabledFlag;

        bool touching    = false;
        bool wasTouching = (m_flags & e_touchingFlag) == e_touchingFlag;

        bool sensorA = m_fixtureA.IsSensor();
        bool sensorB = m_fixtureB.IsSensor();
        bool sensor  = sensorA || sensorB;

        b2Body* body_A = m_fixtureA.GetBody();
        b2Body* body_B = m_fixtureB.GetBody();
        b2Transform xfA = body_A.GetTransform();
        b2Transform xfB = body_B.GetTransform();

        // Is this contact a sensor?
        if (sensor)
        {
            const(b2Shape) shapeA = m_fixtureA.GetShape();
            const(b2Shape) shapeB = m_fixtureB.GetShape();
            touching = b2TestOverlap(shapeA, m_indexA, shapeB, m_indexB, xfA, xfB);

            // Sensors don't generate manifolds.
            m_manifold.pointCount = 0;
        }
        else
        {
            Evaluate(&m_manifold, xfA, xfB);
            touching = m_manifold.pointCount > 0;

            // Match old contact ids to new contact ids and copy the
            // stored impulses to warm start the solver.
            for (int32 i = 0; i < m_manifold.pointCount; ++i)
            {
                b2ManifoldPoint* mp2 = m_manifold.points.ptr + i;
                mp2.normalImpulse  = 0.0f;
                mp2.tangentImpulse = 0.0f;
                b2ContactID id2 = mp2.id;

                for (int32 j = 0; j < oldManifold.pointCount; ++j)
                {
                    b2ManifoldPoint* mp1 = oldManifold.points.ptr + j;

                    if (mp1.id.key == id2.key)
                    {
                        mp2.normalImpulse  = mp1.normalImpulse;
                        mp2.tangentImpulse = mp1.tangentImpulse;
                        break;
                    }
                }
            }

            if (touching != wasTouching)
            {
                body_A.SetAwake(true);
                body_B.SetAwake(true);
            }
        }

        if (touching)
        {
            m_flags |= e_touchingFlag;
        }
        else
        {
            m_flags &= ~e_touchingFlag;
        }

        if (wasTouching == false && touching == true && listener)
        {
            listener.BeginContact(this);
        }

        if (wasTouching == true && touching == false && listener)
        {
            listener.EndContact(this);
        }

        if (sensor == false && touching && listener)
        {
            listener.PreSolve(this, &oldManifold);
        }
    }

    b2Manifold* GetManifold()
    {
        return &m_manifold;
    }

    const(b2Manifold)* GetManifold() const
    {
        return &m_manifold;
    }

    void GetWorldManifold(b2WorldManifold* worldManifold) const
    {
        const(b2Body*) body_A  = m_fixtureA.GetBody();
        const(b2Body*) body_B  = m_fixtureB.GetBody();
        const(b2Shape) shapeA = m_fixtureA.GetShape();
        const(b2Shape) shapeB = m_fixtureB.GetShape();

        worldManifold.Initialize(&m_manifold, body_A.GetTransform(), shapeA.m_radius, body_B.GetTransform(), shapeB.m_radius);
    }

    void SetEnabled(bool flag)
    {
        if (flag)
        {
            m_flags |= e_enabledFlag;
        }
        else
        {
            m_flags &= ~e_enabledFlag;
        }
    }

    bool IsEnabled() const
    {
        return (m_flags & e_enabledFlag) == e_enabledFlag;
    }

    bool IsTouching() const
    {
        return (m_flags & e_touchingFlag) == e_touchingFlag;
    }

    b2Contact GetNext()
    {
        return m_next;
    }

    const(b2Contact) GetNext() const
    {
        return m_next;
    }

    b2Fixture* GetFixtureA()
    {
        return m_fixtureA;
    }

    const(b2Fixture)* GetFixtureA() const
    {
        return m_fixtureA;
    }

    b2Fixture* GetFixtureB()
    {
        return m_fixtureB;
    }

    int32 GetChildIndexA() const
    {
        return m_indexA;
    }

    const(b2Fixture)* GetFixtureB() const
    {
        return m_fixtureB;
    }

    int32 GetChildIndexB() const
    {
        return m_indexB;
    }

    void FlagForFiltering()
    {
        m_flags |= e_filterFlag;
    }

    void SetFriction(float32 friction)
    {
        m_friction = friction;
    }

    float32 GetFriction() const
    {
        return m_friction;
    }

    void ResetFriction()
    {
        m_friction = b2MixFriction(m_fixtureA.m_friction, m_fixtureB.m_friction);
    }

    void SetRestitution(float32 restitution)
    {
        m_restitution = restitution;
    }

    float32 GetRestitution() const
    {
        return m_restitution;
    }

    void ResetRestitution()
    {
        m_restitution = b2MixRestitution(m_fixtureA.m_restitution, m_fixtureB.m_restitution);
    }

    void SetTangentSpeed(float32 speed)
    {
        m_tangentSpeed = speed;
    }

    float32 GetTangentSpeed() const
    {
        return m_tangentSpeed;
    }

    abstract void Evaluate(b2Manifold* manifold, b2Transform xfA, b2Transform xfB);

    // Flags stored in m_flags
    enum
    {
        // Used when crawling contact graph when forming islands.
        e_islandFlag = 0x0001,

        // Set when the shapes are touching.
        e_touchingFlag = 0x0002,

        // This contact can be disabled (by user)
        e_enabledFlag = 0x0004,

        // This contact needs filtering because a fixture filter was changed.
        e_filterFlag = 0x0008,

        // This bullet contact had a TOI event
        e_bulletHitFlag = 0x0010,

        // This contact has a valid TOI in m_toi
        e_toiFlag = 0x0020
    }

    static __gshared b2ContactRegister[b2Shape.Type.e_typeCount][b2Shape.Type.e_typeCount] s_registers;

    uint32 m_flags;

    // World pool and list pointers.
    b2Contact m_prev;
    b2Contact m_next;

    // Nodes for connecting bodies.
    b2ContactEdge m_nodeA;
    b2ContactEdge m_nodeB;

    b2Fixture* m_fixtureA;
    b2Fixture* m_fixtureB;

    int32 m_indexA;
    int32 m_indexB;

    b2Manifold m_manifold;

    int32 m_toiCount;
    float32 m_toi = 0;

    float32 m_friction = 0;
    float32 m_restitution = 0;

    float32 m_tangentSpeed = 0;
}

import dbox.dynamics.contacts.b2contact;
import dbox.dynamics.contacts.b2circlecontact;
import dbox.dynamics.contacts.b2polygonandcirclecontact;
import dbox.dynamics.contacts.b2polygoncontact;
import dbox.dynamics.contacts.b2edgeandcirclecontact;
import dbox.dynamics.contacts.b2edgeandpolygoncontact;
import dbox.dynamics.contacts.b2chainandcirclecontact;
import dbox.dynamics.contacts.b2chainandpolygoncontact;
import dbox.dynamics.contacts.b2contactsolver;

import dbox.collision.b2collision;
import dbox.collision.b2timeofimpact;
import dbox.collision.shapes.b2shape;
import dbox.common.b2blockallocator;
import dbox.dynamics.b2body;
import dbox.dynamics.b2fixture;
import dbox.dynamics.b2world;
