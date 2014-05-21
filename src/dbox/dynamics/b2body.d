module dbox.dynamics.b2body;

import core.stdc.float_;
import core.stdc.stdlib;
import core.stdc.string;

import dbox.common;
import dbox.collision;
import dbox.dynamics;
import dbox.dynamics.contacts;
import dbox.dynamics.joints;

/*
 * Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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

// #ifndef B2_BODY_H
// #define B2_BODY_H

import dbox.common.b2math;
import dbox.collision.shapes.b2shape;

/// The body type.
/// static: zero mass, zero velocity, may be manually moved
/// kinematic: zero mass, non-zero velocity set by user, moved by solver
/// dynamic: positive mass, non-zero velocity determined by forces, moved by solver
enum b2BodyType
{
    b2_staticBody = 0,
    b2_kinematicBody,
    b2_dynamicBody

    // TODO_ERIN
    // b2_bulletBody,
}

alias b2_staticBody    = b2BodyType.b2_staticBody;
alias b2_kinematicBody = b2BodyType.b2_kinematicBody;
alias b2_dynamicBody   = b2BodyType.b2_dynamicBody;

/// A body definition holds all the data needed to construct a rigid body_.
/// You can safely re-use body definitions. Shapes are added to a body after construction.
struct b2BodyDef
{
    /// The body type: static, kinematic, or dynamic.
    /// Note: if a dynamic body would have zero mass, the mass is set to one.
    b2BodyType type = b2BodyType.b2_staticBody;

    /// The world position of the body_. Avoid creating bodies at the origin
    /// since this can lead to many overlapping shapes.
    b2Vec2 position = b2Vec2(0, 0);

    /// The world angle of the body in radians.
    float32 angle = 0;

    /// The linear velocity of the body's origin in world co-ordinates.
    b2Vec2 linearVelocity = b2Vec2(0, 0);

    /// The angular velocity of the body_.
    float32 angularVelocity = 0;

    /// Linear damping is use to reduce the linear velocity. The damping parameter
    /// can be larger than 1.0f but the damping effect becomes sensitive to the
    /// time step when the damping parameter is large.
    float32 linearDamping = 0;

    /// Angular damping is use to reduce the angular velocity. The damping parameter
    /// can be larger than 1.0f but the damping effect becomes sensitive to the
    /// time step when the damping parameter is large.
    float32 angularDamping = 0;

    /// Set this flag to false if this body should never fall asleep. Note that
    /// this increases CPU usage.
    bool allowSleep = true;

    /// Is this body initially awake or sleeping?
    bool awake = true;

    /// Should this body be prevented from rotating? Useful for characters.
    bool fixedRotation;

    /// Is this a fast moving body that should be prevented from tunneling through
    /// other moving bodies? Note that all bodies are prevented from tunneling through
    /// kinematic and static bodies. This setting is only considered on dynamic bodies.
    /// @warning You should use this flag sparingly since it increases processing time.
    bool bullet;

    /// Does this body start out active?
    bool active = true;

    /// Use this to store application specific body data.
    void* userData;

    /// Scale the gravity applied to this body_.
    float32 gravityScale = 1.0;
}

/// A rigid body_. These are created via b2World*.CreateBody.
struct b2Body
{
    this(const(b2BodyDef)* bd, b2World* world)
    {
        assert(bd.position.IsValid());
        assert(bd.linearVelocity.IsValid());
        assert(b2IsValid(bd.angle));
        assert(b2IsValid(bd.angularVelocity));
        assert(b2IsValid(bd.angularDamping) && bd.angularDamping >= 0.0f);
        assert(b2IsValid(bd.linearDamping) && bd.linearDamping >= 0.0f);

        m_flags = 0;

        if (bd.bullet)
        {
            m_flags |= e_bulletFlag;
        }

        if (bd.fixedRotation)
        {
            m_flags |= e_fixedRotationFlag;
        }

        if (bd.allowSleep)
        {
            m_flags |= e_autoSleepFlag;
        }

        if (bd.awake)
        {
            m_flags |= e_awakeFlag;
        }

        if (bd.active)
        {
            m_flags |= e_activeFlag;
        }

        m_world = world;

        m_xf.p = bd.position;
        m_xf.q.Set(bd.angle);

        m_sweep.localCenter.SetZero();
        m_sweep.c0     = m_xf.p;
        m_sweep.c      = m_xf.p;
        m_sweep.a0     = bd.angle;
        m_sweep.a      = bd.angle;
        m_sweep.alpha0 = 0.0f;

        m_jointList   = null;
        m_contactList = null;
        m_prev        = null;
        m_next        = null;

        m_linearVelocity  = bd.linearVelocity;
        m_angularVelocity = bd.angularVelocity;

        m_linearDamping  = bd.linearDamping;
        m_angularDamping = bd.angularDamping;
        m_gravityScale   = bd.gravityScale;

        m_force.SetZero();
        m_torque = 0.0f;

        m_sleepTime = 0.0f;

        m_type = bd.type;

        if (m_type == b2_dynamicBody)
        {
            m_mass    = 1.0f;
            m_invMass = 1.0f;
        }
        else
        {
            m_mass    = 0.0f;
            m_invMass = 0.0f;
        }

        m_I    = 0.0f;
        m_invI = 0.0f;

        m_userData = cast(void*)bd.userData;

        m_fixtureList  = null;
        m_fixtureCount = 0;
    }

    ~this()
    {
        // shapes and joints are destroyed in b2World*.Destroy
    }

    void SetType(b2BodyType type)
    {
        assert(m_world);
        assert(m_world.IsLocked() == false);

        if (m_world.IsLocked() == true)
        {
            return;
        }

        if (m_type == type)
        {
            return;
        }

        m_type = type;

        ResetMassData();

        if (m_type == b2BodyType.b2_staticBody)
        {
            m_linearVelocity.SetZero();
            m_angularVelocity = 0.0f;
            m_sweep.a0        = m_sweep.a;
            m_sweep.c0        = m_sweep.c;
            SynchronizeFixtures();
        }

        SetAwake(true);

        m_force.SetZero();
        m_torque = 0.0f;

        // Delete the attached contacts.
        b2ContactEdge* ce = m_contactList;

        while (ce)
        {
            b2ContactEdge* ce0 = ce;
            ce = ce.next;
            m_world.m_contactManager.Destroy(ce0.contact);
        }

        m_contactList = null;

        // Touch the proxies so that new contacts will be created (when appropriate)
        b2BroadPhase* broadPhase = &m_world.m_contactManager.m_broadPhase;

        for (b2Fixture* f = m_fixtureList; f; f = f.m_next)
        {
            int32 proxyCount = f.m_proxyCount;

            for (int32 i = 0; i < proxyCount; ++i)
            {
                broadPhase.TouchProxy(f.m_proxies[i].proxyId);
            }
        }
    }

    b2Fixture* CreateFixture(const(b2FixtureDef)* def)
    {
        assert(m_world);
        assert(m_world.IsLocked() == false);

        if (m_world.IsLocked() == true)
        {
            return null;
        }

        b2BlockAllocator* allocator = &m_world.m_blockAllocator;

        void* mem = allocator.Allocate(b2memSizeOf!b2Fixture);
        b2Fixture* fixture = b2emplace!b2Fixture(mem);
        fixture.Create(allocator, &this, def);

        if (m_flags & e_activeFlag)
        {
            b2BroadPhase* broadPhase = &m_world.m_contactManager.m_broadPhase;
            fixture.CreateProxies(broadPhase, m_xf);
        }

        fixture.m_next = m_fixtureList;
        m_fixtureList   = fixture;
        ++m_fixtureCount;

        fixture.m_body = &this;

        // Adjust mass properties if needed.
        if (fixture.m_density > 0.0f)
        {
            ResetMassData();
        }

        // Let the world know we have a new fixture. This will cause new contacts
        // to be created at the beginning of the next time step.
        m_world.m_flags |= b2World.e_newFixture;

        return fixture;
    }

    b2Fixture* CreateFixture(const(b2Shape) shape, float32 density)
    {
        b2FixtureDef def;
        def.shape   = cast(b2Shape)shape;
        def.density = density;
        return CreateFixture(&def);
    }

    void DestroyFixture(b2Fixture* fixture)
    {
        assert(m_world.IsLocked() == false);

        if (m_world.IsLocked() == true)
        {
            return;
        }

        assert(fixture.m_body == &this);

        // Remove the fixture from this body's singly linked list.
        assert(m_fixtureCount > 0);
        b2Fixture** node = &m_fixtureList;
        bool found       = false;

        while (*node !is null)
        {
            if (*node == fixture)
            {
                *node = fixture.m_next;
                found = true;
                break;
            }

            node = &(*node).m_next;
        }

        // You tried to remove a shape that is not attached to this body_.
        assert(found);

        // Destroy any contacts associated with the fixture.
        b2ContactEdge* edge = m_contactList;

        while (edge)
        {
            b2Contact c = edge.contact;
            edge = edge.next;

            b2Fixture* fixtureA = c.GetFixtureA();
            b2Fixture* fixtureB = c.GetFixtureB();

            if (fixture == fixtureA || fixture == fixtureB)
            {
                // This destroys the contact and removes it from
                // this body's contact list.
                m_world.m_contactManager.Destroy(c);
            }
        }

        b2BlockAllocator* allocator = &m_world.m_blockAllocator;

        if (m_flags & e_activeFlag)
        {
            b2BroadPhase* broadPhase = &m_world.m_contactManager.m_broadPhase;
            fixture.DestroyProxies(broadPhase);
        }

        fixture.Destroy(allocator);
        fixture.m_body = null;
        fixture.m_next = null;
        destroy(fixture);
        allocator.Free(cast(void*)fixture, b2memSizeOf!b2Fixture);

        --m_fixtureCount;

        // Reset the mass data.
        ResetMassData();
    }

    void ResetMassData()
    {
        // Compute mass data from shapes. Each shape has its own density.
        m_mass    = 0.0f;
        m_invMass = 0.0f;
        m_I       = 0.0f;
        m_invI    = 0.0f;
        m_sweep.localCenter.SetZero();

        // Static and kinematic bodies have zero mass.
        if (m_type == b2_staticBody || m_type == b2_kinematicBody)
        {
            m_sweep.c0 = m_xf.p;
            m_sweep.c  = m_xf.p;
            m_sweep.a0 = m_sweep.a;
            return;
        }

        assert(m_type == b2_dynamicBody);

        // Accumulate mass over all fixtures.
        b2Vec2 localCenter = b2Vec2_zero;

        for (b2Fixture* f = m_fixtureList; f; f = f.m_next)
        {
            if (f.m_density == 0.0f)
            {
                continue;
            }

            b2MassData massData;
            f.GetMassData(&massData);
            m_mass      += massData.mass;
            localCenter += massData.mass * massData.center;
            m_I         += massData.I;
        }

        // Compute center of mass.
        if (m_mass > 0.0f)
        {
            m_invMass    = 1.0f / m_mass;
            localCenter *= m_invMass;
        }
        else
        {
            // Force all dynamic bodies to have a positive mass.
            m_mass    = 1.0f;
            m_invMass = 1.0f;
        }

        if (m_I > 0.0f && (m_flags & e_fixedRotationFlag) == 0)
        {
            // Center the inertia about the center of mass.
            m_I -= m_mass * b2Dot(localCenter, localCenter);
            assert(m_I > 0.0f);
            m_invI = 1.0f / m_I;
        }
        else
        {
            m_I    = 0.0f;
            m_invI = 0.0f;
        }

        // Move center of mass.
        b2Vec2 oldCenter = m_sweep.c;
        m_sweep.localCenter = localCenter;
        m_sweep.c0 = m_sweep.c = b2Mul(m_xf, m_sweep.localCenter);

        // Update center of mass velocity.
        m_linearVelocity += b2Cross(m_angularVelocity, m_sweep.c - oldCenter);
    }

    void SetMassData(const(b2MassData)* massData)
    {
        assert(m_world.IsLocked() == false);

        if (m_world.IsLocked() == true)
        {
            return;
        }

        if (m_type != b2_dynamicBody)
        {
            return;
        }

        m_invMass = 0.0f;
        m_I       = 0.0f;
        m_invI    = 0.0f;

        m_mass = massData.mass;

        if (m_mass <= 0.0f)
        {
            m_mass = 1.0f;
        }

        m_invMass = 1.0f / m_mass;

        if (massData.I > 0.0f && (m_flags & b2Body.e_fixedRotationFlag) == 0)
        {
            m_I = massData.I - m_mass * b2Dot(massData.center, massData.center);
            assert(m_I > 0.0f);
            m_invI = 1.0f / m_I;
        }

        // Move center of mass.
        b2Vec2 oldCenter = m_sweep.c;
        m_sweep.localCenter =  massData.center;
        m_sweep.c0 = m_sweep.c = b2Mul(m_xf, m_sweep.localCenter);

        // Update center of mass velocity.
        m_linearVelocity += b2Cross(m_angularVelocity, m_sweep.c - oldCenter);
    }

    bool ShouldCollide(const(b2Body*) other) const
    {
        // At least one body should be dynamic.
        if (m_type != b2_dynamicBody && other.m_type != b2_dynamicBody)
        {
            return false;
        }

        // Does a joint prevent collision?
        for (b2JointEdge* jn = cast(b2JointEdge*)m_jointList; jn; jn = jn.next)
        {
            if (jn.other == other)
            {
                if (jn.joint.m_collideConnected == false)
                {
                    return false;
                }
            }
        }

        return true;
    }

    void SetTransform(b2Vec2 position, float32 angle)
    {
        assert(m_world.IsLocked() == false);

        if (m_world.IsLocked() == true)
        {
            return;
        }

        m_xf.q.Set(angle);
        m_xf.p = position;

        m_sweep.c = b2Mul(m_xf, m_sweep.localCenter);
        m_sweep.a = angle;

        m_sweep.c0 = m_sweep.c;
        m_sweep.a0 = angle;

        b2BroadPhase* broadPhase = &m_world.m_contactManager.m_broadPhase;

        for (b2Fixture* f = m_fixtureList; f; f = f.m_next)
        {
            f.Synchronize(broadPhase, m_xf, m_xf);
        }
    }

    void SynchronizeFixtures()
    {
        b2Transform xf1;
        xf1.q.Set(m_sweep.a0);
        xf1.p = m_sweep.c0 - b2Mul(xf1.q, m_sweep.localCenter);

        b2BroadPhase* broadPhase = &m_world.m_contactManager.m_broadPhase;

        for (b2Fixture* f = m_fixtureList; f; f = f.m_next)
        {
            f.Synchronize(broadPhase, xf1, m_xf);
        }
    }

    void SetActive(bool flag)
    {
        assert(m_world.IsLocked() == false);

        if (flag == IsActive())
        {
            return;
        }

        if (flag)
        {
            m_flags |= e_activeFlag;

            // Create all proxies.
            b2BroadPhase* broadPhase = &m_world.m_contactManager.m_broadPhase;

            for (b2Fixture* f = m_fixtureList; f; f = f.m_next)
            {
                f.CreateProxies(broadPhase, m_xf);
            }

            // Contacts are created the next time step.
        }
        else
        {
            m_flags &= ~e_activeFlag;

            // Destroy all proxies.
            b2BroadPhase* broadPhase = &m_world.m_contactManager.m_broadPhase;

            for (b2Fixture* f = m_fixtureList; f; f = f.m_next)
            {
                f.DestroyProxies(broadPhase);
            }

            // Destroy the attached contacts.
            b2ContactEdge* ce = m_contactList;

            while (ce)
            {
                b2ContactEdge* ce0 = ce;
                ce = ce.next;
                m_world.m_contactManager.Destroy(ce0.contact);
            }

            m_contactList = null;
        }
    }

    void SetFixedRotation(bool flag)
    {
        bool status = (m_flags & e_fixedRotationFlag) == e_fixedRotationFlag;

        if (status == flag)
        {
            return;
        }

        if (flag)
        {
            m_flags |= e_fixedRotationFlag;
        }
        else
        {
            m_flags &= ~e_fixedRotationFlag;
        }

        m_angularVelocity = 0.0f;

        ResetMassData();
    }

    void Dump()
    {
        int32 bodyIndex = m_islandIndex;

        b2Log("{\n");
        b2Log("  b2BodyDef bd;\n");
        b2Log("  bd.type = b2BodyType(%d);\n", m_type);
        b2Log("  bd.position.Set(%.15lef, %.15lef);\n", m_xf.p.x, m_xf.p.y);
        b2Log("  bd.angle = %.15lef;\n", m_sweep.a);
        b2Log("  bd.linearVelocity.Set(%.15lef, %.15lef);\n", m_linearVelocity.x, m_linearVelocity.y);
        b2Log("  bd.angularVelocity = %.15lef;\n", m_angularVelocity);
        b2Log("  bd.linearDamping = %.15lef;\n", m_linearDamping);
        b2Log("  bd.angularDamping = %.15lef;\n", m_angularDamping);
        b2Log("  bd.allowSleep = bool(%d);\n", m_flags & e_autoSleepFlag);
        b2Log("  bd.awake = bool(%d);\n", m_flags & e_awakeFlag);
        b2Log("  bd.fixedRotation = bool(%d);\n", m_flags & e_fixedRotationFlag);
        b2Log("  bd.bullet = bool(%d);\n", m_flags & e_bulletFlag);
        b2Log("  bd.active = bool(%d);\n", m_flags & e_activeFlag);
        b2Log("  bd.gravityScale = %.15lef;\n", m_gravityScale);
        b2Log("  bodies[%d] = m_world.CreateBody(&bd);\n", m_islandIndex);
        b2Log("\n");

        for (b2Fixture* f = m_fixtureList; f; f = f.m_next)
        {
            b2Log("  {\n");
            f.Dump(bodyIndex);
            b2Log("  }\n");
        }

        b2Log("}\n");
    }

    b2BodyType GetType() const
    {
        return m_type;
    }

    b2Transform GetTransform() const
    {
        return m_xf;
    }

    b2Vec2 GetPosition() const
    {
        return m_xf.p;
    }

    float32 GetAngle() const
    {
        return m_sweep.a;
    }

    b2Vec2 GetWorldCenter() const
    {
        return m_sweep.c;
    }

    b2Vec2 GetLocalCenter() const
    {
        return m_sweep.localCenter;
    }

    void SetLinearVelocity(b2Vec2 v)
    {
        if (m_type == b2_staticBody)
        {
            return;
        }

        if (b2Dot(v, v) > 0.0f)
        {
            SetAwake(true);
        }

        m_linearVelocity = v;
    }

    b2Vec2 GetLinearVelocity() const
    {
        return m_linearVelocity;
    }

    void SetAngularVelocity(float32 w)
    {
        if (m_type == b2_staticBody)
        {
            return;
        }

        if (w * w > 0.0f)
        {
            SetAwake(true);
        }

        m_angularVelocity = w;
    }

    float32 GetAngularVelocity() const
    {
        return m_angularVelocity;
    }

    float32 GetMass() const
    {
        return m_mass;
    }

    float32 GetInertia() const
    {
        return m_I + m_mass * b2Dot(m_sweep.localCenter, m_sweep.localCenter);
    }

    void GetMassData(b2MassData* data) const
    {
        data.mass   = m_mass;
        data.I      = m_I + m_mass * b2Dot(m_sweep.localCenter, m_sweep.localCenter);
        data.center = m_sweep.localCenter;
    }

    b2Vec2 GetWorldPoint(b2Vec2 localPoint) const
    {
        return b2Mul(m_xf, localPoint);
    }

    b2Vec2 GetWorldVector(b2Vec2 localVector) const
    {
        return b2Mul(m_xf.q, localVector);
    }

    b2Vec2 GetLocalPoint(b2Vec2 worldPoint) const
    {
        return b2MulT(m_xf, worldPoint);
    }

    b2Vec2 GetLocalVector(b2Vec2 worldVector) const
    {
        return b2MulT(m_xf.q, worldVector);
    }

    b2Vec2 GetLinearVelocityFromWorldPoint(b2Vec2 worldPoint) const
    {
        return m_linearVelocity + b2Cross(m_angularVelocity, worldPoint - m_sweep.c);
    }

    b2Vec2 GetLinearVelocityFromLocalPoint(b2Vec2 localPoint) const
    {
        return GetLinearVelocityFromWorldPoint(GetWorldPoint(localPoint));
    }

    float32 GetLinearDamping() const
    {
        return m_linearDamping;
    }

    void SetLinearDamping(float32 linearDamping)
    {
        m_linearDamping = linearDamping;
    }

    float32 GetAngularDamping() const
    {
        return m_angularDamping;
    }

    void SetAngularDamping(float32 angularDamping)
    {
        m_angularDamping = angularDamping;
    }

    float32 GetGravityScale() const
    {
        return m_gravityScale;
    }

    void SetGravityScale(float32 scale)
    {
        m_gravityScale = scale;
    }

    void SetBullet(bool flag)
    {
        if (flag)
        {
            m_flags |= e_bulletFlag;
        }
        else
        {
            m_flags &= ~e_bulletFlag;
        }
    }

    bool IsBullet() const
    {
        return (m_flags & e_bulletFlag) == e_bulletFlag;
    }

    void SetAwake(bool flag)
    {
        if (flag)
        {
            if ((m_flags & e_awakeFlag) == 0)
            {
                m_flags    |= e_awakeFlag;
                m_sleepTime = 0.0f;
            }
        }
        else
        {
            m_flags    &= ~e_awakeFlag;
            m_sleepTime = 0.0f;
            m_linearVelocity.SetZero();
            m_angularVelocity = 0.0f;
            m_force.SetZero();
            m_torque = 0.0f;
        }
    }

    bool IsAwake() const
    {
        return (m_flags & e_awakeFlag) == e_awakeFlag;
    }

    bool IsActive() const
    {
        return (m_flags & e_activeFlag) == e_activeFlag;
    }

    bool IsFixedRotation() const
    {
        return (m_flags & e_fixedRotationFlag) == e_fixedRotationFlag;
    }

    void SetSleepingAllowed(bool flag)
    {
        if (flag)
        {
            m_flags |= e_autoSleepFlag;
        }
        else
        {
            m_flags &= ~e_autoSleepFlag;
            SetAwake(true);
        }
    }

    bool IsSleepingAllowed() const
    {
        return (m_flags & e_autoSleepFlag) == e_autoSleepFlag;
    }

    b2Fixture* GetFixtureList()
    {
        return m_fixtureList;
    }

    const(b2Fixture)* GetFixtureList() const
    {
        return m_fixtureList;
    }

    b2JointEdge* GetJointList()
    {
        return m_jointList;
    }

    const(b2JointEdge)* GetJointList() const
    {
        return m_jointList;
    }

    b2ContactEdge* GetContactList()
    {
        return m_contactList;
    }

    const(b2ContactEdge)* GetContactList() const
    {
        return m_contactList;
    }

    b2Body* GetNext()
    {
        return m_next;
    }

    const(b2Body*) GetNext() const
    {
        return m_next;
    }

    void SetUserData(void* data)
    {
        m_userData = data;
    }

    void* GetUserData() const
    {
        return cast(void*)m_userData;
    }

    void ApplyForce(b2Vec2 force, b2Vec2 point, bool wake)
    {
        if (m_type != b2_dynamicBody)
        {
            return;
        }

        if (wake && (m_flags & e_awakeFlag) == 0)
        {
            SetAwake(true);
        }

        // Don't accumulate a force if the body is sleeping.
        if (m_flags & e_awakeFlag)
        {
            m_force  += force;
            m_torque += b2Cross(point - m_sweep.c, force);
        }
    }

    void ApplyForceToCenter(b2Vec2 force, bool wake)
    {
        if (m_type != b2_dynamicBody)
        {
            return;
        }

        if (wake && (m_flags & e_awakeFlag) == 0)
        {
            SetAwake(true);
        }

        // Don't accumulate a force if the body is sleeping
        if (m_flags & e_awakeFlag)
        {
            m_force += force;
        }
    }

    void ApplyTorque(float32 torque, bool wake)
    {
        if (m_type != b2_dynamicBody)
        {
            return;
        }

        if (wake && (m_flags & e_awakeFlag) == 0)
        {
            SetAwake(true);
        }

        // Don't accumulate a force if the body is sleeping
        if (m_flags & e_awakeFlag)
        {
            m_torque += torque;
        }
    }

    void ApplyLinearImpulse(b2Vec2 impulse, b2Vec2 point, bool wake)
    {
        if (m_type != b2_dynamicBody)
        {
            return;
        }

        if (wake && (m_flags & e_awakeFlag) == 0)
        {
            SetAwake(true);
        }

        // Don't accumulate velocity if the body is sleeping
        if (m_flags & e_awakeFlag)
        {
            m_linearVelocity  += m_invMass * impulse;
            m_angularVelocity += m_invI * b2Cross(point - m_sweep.c, impulse);
        }
    }

    void ApplyAngularImpulse(float32 impulse, bool wake)
    {
        if (m_type != b2_dynamicBody)
        {
            return;
        }

        if (wake && (m_flags & e_awakeFlag) == 0)
        {
            SetAwake(true);
        }

        // Don't accumulate velocity if the body is sleeping
        if (m_flags & e_awakeFlag)
        {
            m_angularVelocity += m_invI * impulse;
        }
    }

    void SynchronizeTransform()
    {
        m_xf.q.Set(m_sweep.a);
        m_xf.p = m_sweep.c - b2Mul(m_xf.q, m_sweep.localCenter);
    }

    void Advance(float32 alpha)
    {
        // Advance to the new safe time. This doesn't sync the broad-phase.
        m_sweep.Advance(alpha);
        m_sweep.c = m_sweep.c0;
        m_sweep.a = m_sweep.a0;
        m_xf.q.Set(m_sweep.a);
        m_xf.p = m_sweep.c - b2Mul(m_xf.q, m_sweep.localCenter);
    }

    b2World* GetWorld()
    {
        return m_world;
    }

    const(b2World*) GetWorld() const
    {
        return m_world;
    }

/* private */

    // m_flags
    enum
    {
        e_islandFlag = 0x0001,
        e_awakeFlag = 0x0002,
        e_autoSleepFlag     = 0x0004,
        e_bulletFlag        = 0x0008,
        e_fixedRotationFlag = 0x0010,
        e_activeFlag        = 0x0020,
        e_toiFlag = 0x0040
    }

    b2BodyType m_type;

    uint16 m_flags;

    int32 m_islandIndex;

    b2Transform m_xf;                   // the body origin transform
    b2Sweep m_sweep;                    // the swept motion for CCD

    b2Vec2  m_linearVelocity;
    float32 m_angularVelocity = 0;

    b2Vec2  m_force;
    float32 m_torque = 0;

    b2World* m_world;
    b2Body* m_prev;
    b2Body* m_next;

    b2Fixture* m_fixtureList;
    int32 m_fixtureCount;

    b2JointEdge* m_jointList;
    b2ContactEdge* m_contactList;

    float32 m_mass = 0, m_invMass = 0;

    // Rotational inertia about the center of mass.
    float32 m_I = 0, m_invI = 0;

    float32 m_linearDamping = 0;
    float32 m_angularDamping = 0;
    float32 m_gravityScale = 0;

    float32 m_sleepTime = 0;

    void* m_userData;
}

import dbox.dynamics.b2body;
import dbox.dynamics.b2fixture;
import dbox.dynamics.b2world;
import dbox.dynamics.contacts.b2contact;
import dbox.dynamics.joints.b2joint;
