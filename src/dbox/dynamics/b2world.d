module dbox.dynamics.b2world;

import core.stdc.float_;
import core.stdc.stdlib;
import core.stdc.string;

import dbox.common;
import dbox.collision;
import dbox.collision.shapes;
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

// #ifndef B2_WORLD_H
// #define B2_WORLD_H

import dbox.common.b2math;
import dbox.common.b2blockallocator;
import dbox.common.b2stackallocator;
import dbox.dynamics.b2contactmanager;
import dbox.dynamics.b2worldcallbacks;
import dbox.dynamics.b2timestep;

/// The world class manages all physics entities, dynamic simulation,
/// and asynchronous queries. The world also contains efficient memory
/// management facilities.
struct b2World
{
    @disable this();
    @disable this(this);

    this(b2Vec2 gravity)
    {
        m_destructionListener = null;
        g_debugDraw = null;

        m_bodyList  = null;
        m_jointList = null;

        m_bodyCount  = 0;
        m_jointCount = 0;

        m_warmStarting      = true;
        m_continuousPhysics = true;
        m_subStepping       = false;

        m_stepComplete = true;

        m_allowSleep = true;
        m_gravity    = gravity;

        m_flags = e_clearForces;

        m_inv_dt0 = 0.0f;

        m_blockAllocator = b2BlockAllocator(1);

        m_contactManager = b2ContactManager(1);
        m_contactManager.m_allocator = &m_blockAllocator;

        memset(&m_profile, 0, b2memSizeOf!b2Profile);
    }

    b2Body* GetBodyList()
    {
        return m_bodyList;
    }

    const(b2Body*) GetBodyList() const
    {
        return m_bodyList;
    }

    b2Joint GetJointList()
    {
        return m_jointList;
    }

    const(b2Joint) GetJointList() const
    {
        return m_jointList;
    }

    b2Contact GetContactList()
    {
        return m_contactManager.m_contactList;
    }

    const(b2Contact) GetContactList() const
    {
        return m_contactManager.m_contactList;
    }

    int32 GetBodyCount() const
    {
        return m_bodyCount;
    }

    int32 GetJointCount() const
    {
        return m_jointCount;
    }

    int32 GetContactCount() const
    {
        return m_contactManager.m_contactCount;
    }

    void SetGravity(b2Vec2 gravity)
    {
        m_gravity = gravity;
    }

    b2Vec2 GetGravity() const
    {
        return m_gravity;
    }

    bool IsLocked() const
    {
        return (m_flags & e_locked) == e_locked;
    }

    void SetAutoClearForces(bool flag)
    {
        if (flag)
        {
            m_flags |= e_clearForces;
        }
        else
        {
            m_flags &= ~e_clearForces;
        }
    }

    /// Get the flag that controls automatic clearing of forces after each time step.
    bool GetAutoClearForces() const
    {
        return (m_flags & e_clearForces) == e_clearForces;
    }

    b2ContactManager* GetContactManager() const
    {
        return cast(b2ContactManager*)&m_contactManager;
    }

    b2Profile GetProfile() const
    {
        return m_profile;
    }

    ~this()
    {
        // Some shapes allocate using b2Alloc.
        b2Body* b = m_bodyList;

        while (b)
        {
            b2Body* bNext = b.m_next;

            b2Fixture* f = b.m_fixtureList;

            while (f)
            {
                b2Fixture* fNext = f.m_next;
                f.m_proxyCount = 0;
                f.Destroy(&m_blockAllocator);
                f = fNext;
            }

            b = bNext;
        }
    }

    void SetDestructionListener(b2DestructionListener listener)
    {
        m_destructionListener = listener;
    }

    void SetContactFilter(b2ContactFilter filter)
    {
        m_contactManager.m_contactFilter = filter;
    }

    void SetContactListener(b2ContactListener listener)
    {
        m_contactManager.m_contactListener = listener;
    }

    void SetDebugDraw(b2Draw debugDraw)
    {
        g_debugDraw = debugDraw;
    }

    b2Body* CreateBody(const(b2BodyDef)* def)
    {
        assert(IsLocked() == false);

        if (IsLocked())
        {
            return null;
        }

        void* mem = m_blockAllocator.Allocate(b2memSizeOf!b2Body);
        b2Body* b = b2emplace!b2Body(mem, def, &this);

        // Add to world doubly linked list.
        b.m_prev = null;
        b.m_next = m_bodyList;

        if (m_bodyList)
        {
            m_bodyList.m_prev = b;
        }
        m_bodyList = b;
        ++m_bodyCount;

        return b;
    }

    void DestroyBody(b2Body* b)
    {
        assert(m_bodyCount > 0);
        assert(IsLocked() == false);

        if (IsLocked())
        {
            return;
        }

        // Delete the attached joints.
        b2JointEdge* je = b.m_jointList;

        while (je)
        {
            b2JointEdge* je0 = je;
            je = je.next;

            if (m_destructionListener)
            {
                m_destructionListener.SayGoodbye(je0.joint);
            }

            DestroyJoint(je0.joint);

            b.m_jointList = je;
        }

        b.m_jointList = null;

        // Delete the attached contacts.
        b2ContactEdge* ce = b.m_contactList;

        while (ce)
        {
            b2ContactEdge* ce0 = ce;
            ce = ce.next;
            m_contactManager.Destroy(ce0.contact);
        }

        b.m_contactList = null;

        // Delete the attached fixtures. This destroys broad-phase proxies.
        b2Fixture* f = b.m_fixtureList;

        while (f)
        {
            b2Fixture* f0 = f;
            f = f.m_next;

            if (m_destructionListener)
            {
                m_destructionListener.SayGoodbye(f0);
            }

            f0.DestroyProxies(&m_contactManager.m_broadPhase);
            f0.Destroy(&m_blockAllocator);
            destroy(*f0);
            m_blockAllocator.Free(cast(void*)f0, b2memSizeOf!b2Fixture);

            b.m_fixtureList   = f;
            b.m_fixtureCount -= 1;
        }

        b.m_fixtureList  = null;
        b.m_fixtureCount = 0;

        // Remove world body list.
        if (b.m_prev)
        {
            b.m_prev.m_next = b.m_next;
        }

        if (b.m_next)
        {
            b.m_next.m_prev = b.m_prev;
        }

        if (b == m_bodyList)
        {
            m_bodyList = b.m_next;
        }

        --m_bodyCount;
        destroy(*b);
        m_blockAllocator.Free(cast(void*)b, b2memSizeOf!b2Body);
    }

    b2Joint CreateJoint(const(b2JointDef) def)
    {
        assert(IsLocked() == false);

        if (IsLocked())
        {
            return null;
        }

        b2Joint j = b2Joint.Create(def, &m_blockAllocator);

        // Connect to the world list.
        j.m_prev = null;
        j.m_next = m_jointList;

        if (m_jointList)
        {
            m_jointList.m_prev = j;
        }
        m_jointList = j;
        ++m_jointCount;

        // Connect to the bodies' doubly linked lists.
        j.m_edgeA.joint = j;
        j.m_edgeA.other = j.m_body_B;
        j.m_edgeA.prev  = null;
        j.m_edgeA.next  = j.m_body_A.m_jointList;

        if (j.m_body_A.m_jointList)
            j.m_body_A.m_jointList.prev = &j.m_edgeA;
        j.m_body_A.m_jointList = &j.m_edgeA;

        j.m_edgeB.joint = j;
        j.m_edgeB.other = j.m_body_A;
        j.m_edgeB.prev  = null;
        j.m_edgeB.next  = j.m_body_B.m_jointList;

        if (j.m_body_B.m_jointList)
            j.m_body_B.m_jointList.prev = &j.m_edgeB;
        j.m_body_B.m_jointList = &j.m_edgeB;

        b2Body* body_A = cast(b2Body*)def.body_A;
        b2Body* body_B = cast(b2Body*)def.body_B;

        // If the joint prevents collisions, then flag any contacts for filtering.
        if (def.collideConnected == false)
        {
            b2ContactEdge* edge = body_B.GetContactList();

            while (edge)
            {
                if (edge.other == body_A)
                {
                    // Flag the contact for filtering at the next time step (where either
                    // body is awake).
                    edge.contact.FlagForFiltering();
                }

                edge = edge.next;
            }
        }

        // Note: creating a joint doesn't wake the bodies.

        return j;
    }

    void DestroyJoint(b2Joint j)
    {
        assert(IsLocked() == false);

        if (IsLocked())
        {
            return;
        }

        bool collideConnected = j.m_collideConnected;

        // Remove from the doubly linked list.
        if (j.m_prev)
        {
            j.m_prev.m_next = j.m_next;
        }

        if (j.m_next)
        {
            j.m_next.m_prev = j.m_prev;
        }

        if (j == m_jointList)
        {
            m_jointList = j.m_next;
        }

        // Disconnect from island graph.
        b2Body* body_A = j.m_body_A;
        b2Body* body_B = j.m_body_B;

        // Wake up connected bodies.
        body_A.SetAwake(true);
        body_B.SetAwake(true);

        // Remove from body 1.
        if (j.m_edgeA.prev)
        {
            j.m_edgeA.prev.next = j.m_edgeA.next;
        }

        if (j.m_edgeA.next)
        {
            j.m_edgeA.next.prev = j.m_edgeA.prev;
        }

        if (&j.m_edgeA == body_A.m_jointList)
        {
            body_A.m_jointList = j.m_edgeA.next;
        }

        j.m_edgeA.prev = null;
        j.m_edgeA.next = null;

        // Remove from body 2
        if (j.m_edgeB.prev)
        {
            j.m_edgeB.prev.next = j.m_edgeB.next;
        }

        if (j.m_edgeB.next)
        {
            j.m_edgeB.next.prev = j.m_edgeB.prev;
        }

        if (&j.m_edgeB == body_B.m_jointList)
        {
            body_B.m_jointList = j.m_edgeB.next;
        }

        j.m_edgeB.prev = null;
        j.m_edgeB.next = null;

        b2Joint.Destroy(j, &m_blockAllocator);

        assert(m_jointCount > 0);
        --m_jointCount;

        // If the joint prevents collisions, then flag any contacts for filtering.
        if (collideConnected == false)
        {
            b2ContactEdge* edge = body_B.GetContactList();

            while (edge)
            {
                if (edge.other == body_A)
                {
                    // Flag the contact for filtering at the next time step (where either
                    // body is awake).
                    edge.contact.FlagForFiltering();
                }

                edge = edge.next;
            }
        }
    }

    //
    void SetAllowSleeping(bool flag)
    {
        if (flag == m_allowSleep)
        {
            return;
        }

        m_allowSleep = flag;

        if (m_allowSleep == false)
        {
            for (b2Body* b = m_bodyList; b; b = b.m_next)
            {
                b.SetAwake(true);
            }
        }
    }

    // Find islands, integrate and solve constraints, solve position constraints
    void Solve(b2TimeStep step)
    {
        m_profile.solveInit     = 0.0f;
        m_profile.solveVelocity = 0.0f;
        m_profile.solvePosition = 0.0f;

        // Size the island for the worst case.
        b2Island island = b2Island(m_bodyCount,
                        m_contactManager.m_contactCount,
                        m_jointCount,
                        &m_stackAllocator,
                        m_contactManager.m_contactListener);

        // Clear all the island flags.
        for (b2Body* b = m_bodyList; b; b = b.m_next)
        {
            b.m_flags &= ~b2Body.e_islandFlag;
        }

        for (b2Contact c = m_contactManager.m_contactList; c; c = c.m_next)
        {
            c.m_flags &= ~b2Contact.e_islandFlag;
        }

        for (b2Joint j = m_jointList; j; j = j.m_next)
        {
            j.m_islandFlag = false;
        }

        // Build and simulate all awake islands.
        int32 stackSize = m_bodyCount;
        b2Body** stack  = cast(b2Body**)m_stackAllocator.Allocate(stackSize * b2memSizeOf!b2Body);

        for (b2Body* seed = m_bodyList; seed; seed = seed.m_next)
        {
            if (seed.m_flags & b2Body.e_islandFlag)
            {
                continue;
            }

            if (seed.IsAwake() == false || seed.IsActive() == false)
            {
                continue;
            }

            // The seed can be dynamic or kinematic.
            if (seed.GetType() == b2_staticBody)
            {
                continue;
            }

            // Reset island and stack.
            island.Clear();
            int32 stackCount = 0;
            stack[stackCount++] = seed;
            seed.m_flags      |= b2Body.e_islandFlag;

            // Perform a depth first search (DFS) on the constraint graph.
            while (stackCount > 0)
            {
                // Grab the next body off the stack and add it to the island.
                b2Body* b = stack[--stackCount];
                assert(b.IsActive() == true);
                island.Add(b);

                // Make sure the body is awake.
                b.SetAwake(true);

                // To keep islands as small as possible, we don't
                // propagate islands across static bodies.
                if (b.GetType() == b2_staticBody)
                {
                    continue;
                }

                // Search all contacts connected to this body_.
                for (b2ContactEdge* ce = b.m_contactList; ce; ce = ce.next)
                {
                    b2Contact contact = ce.contact;

                    // Has this contact already been added to an island?
                    if (contact.m_flags & b2Contact.e_islandFlag)
                    {
                        continue;
                    }

                    // Is this contact solid and touching?
                    if (contact.IsEnabled() == false ||
                        contact.IsTouching() == false)
                    {
                        continue;
                    }

                    // Skip sensors.
                    bool sensorA = contact.m_fixtureA.m_isSensor;
                    bool sensorB = contact.m_fixtureB.m_isSensor;

                    if (sensorA || sensorB)
                    {
                        continue;
                    }

                    island.Add(contact);
                    contact.m_flags |= b2Contact.e_islandFlag;

                    b2Body* other = ce.other;

                    // Was the other body already added to this island?
                    if (other.m_flags & b2Body.e_islandFlag)
                    {
                        continue;
                    }

                    assert(stackCount < stackSize);
                    stack[stackCount++] = other;
                    other.m_flags     |= b2Body.e_islandFlag;
                }

                // Search all joints connect to this body_.
                for (b2JointEdge* je = b.m_jointList; je; je = je.next)
                {
                    if (je.joint.m_islandFlag == true)
                    {
                        continue;
                    }

                    b2Body* other = je.other;

                    // Don't simulate joints connected to inactive bodies.
                    if (other.IsActive() == false)
                    {
                        continue;
                    }

                    island.Add(je.joint);
                    je.joint.m_islandFlag = true;

                    if (other.m_flags & b2Body.e_islandFlag)
                    {
                        continue;
                    }

                    assert(stackCount < stackSize);
                    stack[stackCount++] = other;
                    other.m_flags     |= b2Body.e_islandFlag;
                }
            }

            b2Profile profile;
            island.Solve(&profile, step, m_gravity, m_allowSleep);
            m_profile.solveInit     += profile.solveInit;
            m_profile.solveVelocity += profile.solveVelocity;
            m_profile.solvePosition += profile.solvePosition;

            // Post solve cleanup.
            for (int32 i = 0; i < island.m_bodyCount; ++i)
            {
                // Allow static bodies to participate in other islands.
                b2Body* b = island.m_bodies[i];

                if (b.GetType() == b2_staticBody)
                {
                    b.m_flags &= ~b2Body.e_islandFlag;
                }
            }
        }

        m_stackAllocator.Free(cast(void*)stack);

        {
            auto timer = b2Timer();

            // Synchronize fixtures, check for out of range bodies.
            for (b2Body* b = m_bodyList; b; b = b.GetNext())
            {
                // If a body was not in an island then it did not move.
                if ((b.m_flags & b2Body.e_islandFlag) == 0)
                {
                    continue;
                }

                if (b.GetType() == b2_staticBody)
                {
                    continue;
                }

                // Update fixtures (for broad-phase).
                b.SynchronizeFixtures();
            }

            // Look for new contacts.
            m_contactManager.FindNewContacts();
            m_profile.broadphase = timer.GetMilliseconds();
        }
    }

    // Find TOI contacts and solve them.
    void SolveTOI(b2TimeStep step)
    {
        b2Island island = b2Island(2 * b2_maxTOIContacts, b2_maxTOIContacts, 0, &m_stackAllocator, m_contactManager.m_contactListener);

        if (m_stepComplete)
        {
            for (b2Body* b = m_bodyList; b; b = b.m_next)
            {
                b.m_flags       &= ~b2Body.e_islandFlag;
                b.m_sweep.alpha0 = 0.0f;
            }

            for (b2Contact c = m_contactManager.m_contactList; c; c = c.m_next)
            {
                // Invalidate TOI
                c.m_flags   &= ~(b2Contact.e_toiFlag | b2Contact.e_islandFlag);
                c.m_toiCount = 0;
                c.m_toi      = 1.0f;
            }
        }

        // Find TOI events and solve them.
        for (;;)
        {
            // Find the first TOI.
            b2Contact minContact = null;
            float32 minAlpha      = 1.0f;

            for (b2Contact c = m_contactManager.m_contactList; c; c = c.m_next)
            {
                // Is this contact disabled?
                if (c.IsEnabled() == false)
                {
                    continue;
                }

                // Prevent excessive sub-stepping.
                if (c.m_toiCount > b2_maxSubSteps)
                {
                    continue;
                }

                float32 alpha = 1.0f;

                if (c.m_flags & b2Contact.e_toiFlag)
                {
                    // This contact has a valid cached TOI.
                    alpha = c.m_toi;
                }
                else
                {
                    b2Fixture* fA = c.GetFixtureA();
                    b2Fixture* fB = c.GetFixtureB();

                    // Is there a sensor?
                    if (fA.IsSensor() || fB.IsSensor())
                    {
                        continue;
                    }

                    b2Body* bA = fA.GetBody();
                    b2Body* bB = fB.GetBody();

                    b2BodyType typeA = bA.m_type;
                    b2BodyType typeB = bB.m_type;
                    assert(typeA == b2_dynamicBody || typeB == b2_dynamicBody);

                    bool activeA = bA.IsAwake() && typeA != b2_staticBody;
                    bool activeB = bB.IsAwake() && typeB != b2_staticBody;

                    // Is at least one body active (awake and dynamic or kinematic)?
                    if (activeA == false && activeB == false)
                    {
                        continue;
                    }

                    bool collideA = bA.IsBullet() || typeA != b2_dynamicBody;
                    bool collideB = bB.IsBullet() || typeB != b2_dynamicBody;

                    // Are these two non-bullet dynamic bodies?
                    if (collideA == false && collideB == false)
                    {
                        continue;
                    }

                    // Compute the TOI for this contact.
                    // Put the sweeps onto the same time interval.
                    float32 alpha0 = bA.m_sweep.alpha0;

                    if (bA.m_sweep.alpha0 < bB.m_sweep.alpha0)
                    {
                        alpha0 = bB.m_sweep.alpha0;
                        bA.m_sweep.Advance(alpha0);
                    }
                    else if (bB.m_sweep.alpha0 < bA.m_sweep.alpha0)
                    {
                        alpha0 = bA.m_sweep.alpha0;
                        bB.m_sweep.Advance(alpha0);
                    }

                    assert(alpha0 < 1.0f);

                    int32 indexA = c.GetChildIndexA();
                    int32 indexB = c.GetChildIndexB();

                    // Compute the time of impact in interval [0, minTOI]
                    b2TOIInput input;
                    input.proxyA.Set(fA.GetShape(), indexA);
                    input.proxyB.Set(fB.GetShape(), indexB);
                    input.sweepA = bA.m_sweep;
                    input.sweepB = bB.m_sweep;
                    input.tMax   = 1.0f;

                    b2TOIOutput output;
                    b2TimeOfImpact(&output, &input);

                    // Beta is the fraction of the remaining portion of the .
                    float32 beta = output.t;

                    if (output.state == b2TOIOutput.e_touching)
                    {
                        alpha = b2Min(alpha0 + (1.0f - alpha0) * beta, 1.0f);
                    }
                    else
                    {
                        alpha = 1.0f;
                    }

                    c.m_toi    = alpha;
                    c.m_flags |= b2Contact.e_toiFlag;
                }

                if (alpha < minAlpha)
                {
                    // This is the minimum TOI found so far.
                    minContact = c;
                    minAlpha   = alpha;
                }
            }

            if (minContact is null || 1.0f - 10.0f * b2_epsilon < minAlpha)
            {
                // No more TOI events. Done!
                m_stepComplete = true;
                break;
            }

            // Advance the bodies to the TOI.
            b2Fixture* fA = minContact.GetFixtureA();
            b2Fixture* fB = minContact.GetFixtureB();
            b2Body* bA    = fA.GetBody();
            b2Body* bB    = fB.GetBody();

            b2Sweep backup1 = bA.m_sweep;
            b2Sweep backup2 = bB.m_sweep;

            bA.Advance(minAlpha);
            bB.Advance(minAlpha);

            // The TOI contact likely has some new contact points.
            minContact.Update(m_contactManager.m_contactListener);
            minContact.m_flags &= ~b2Contact.e_toiFlag;
            ++minContact.m_toiCount;

            // Is the contact solid?
            if (minContact.IsEnabled() == false || minContact.IsTouching() == false)
            {
                // Restore the sweeps.
                minContact.SetEnabled(false);
                bA.m_sweep = backup1;
                bB.m_sweep = backup2;
                bA.SynchronizeTransform();
                bB.SynchronizeTransform();
                continue;
            }

            bA.SetAwake(true);
            bB.SetAwake(true);

            // Build the island
            island.Clear();
            island.Add(bA);
            island.Add(bB);
            island.Add(minContact);

            bA.m_flags         |= b2Body.e_islandFlag;
            bB.m_flags         |= b2Body.e_islandFlag;
            minContact.m_flags |= b2Contact.e_islandFlag;

            // Get contacts on body_A and body_B.
            b2Body* bodies[2] = [bA, bB];

            for (int32 i = 0; i < 2; ++i)
            {
                b2Body* body_ = bodies[i];

                if (body_.m_type == b2_dynamicBody)
                {
                    for (b2ContactEdge* ce = body_.m_contactList; ce; ce = ce.next)
                    {
                        if (island.m_bodyCount == island.m_bodyCapacity)
                        {
                            break;
                        }

                        if (island.m_contactCount == island.m_contactCapacity)
                        {
                            break;
                        }

                        b2Contact contact = ce.contact;

                        // Has this contact already been added to the island?
                        if (contact.m_flags & b2Contact.e_islandFlag)
                        {
                            continue;
                        }

                        // Only add static, kinematic, or bullet bodies.
                        b2Body* other = ce.other;

                        if (other.m_type == b2_dynamicBody &&
                            body_.IsBullet() == false && other.IsBullet() == false)
                        {
                            continue;
                        }

                        // Skip sensors.
                        bool sensorA = contact.m_fixtureA.m_isSensor;
                        bool sensorB = contact.m_fixtureB.m_isSensor;

                        if (sensorA || sensorB)
                        {
                            continue;
                        }

                        // Tentatively advance the body to the TOI.
                        b2Sweep backup = other.m_sweep;

                        if ((other.m_flags & b2Body.e_islandFlag) == 0)
                        {
                            other.Advance(minAlpha);
                        }

                        // Update the contact points
                        contact.Update(m_contactManager.m_contactListener);

                        // Was the contact disabled by the user?
                        if (contact.IsEnabled() == false)
                        {
                            other.m_sweep = backup;
                            other.SynchronizeTransform();
                            continue;
                        }

                        // Are there contact points?
                        if (contact.IsTouching() == false)
                        {
                            other.m_sweep = backup;
                            other.SynchronizeTransform();
                            continue;
                        }

                        // Add the contact to the island
                        contact.m_flags |= b2Contact.e_islandFlag;
                        island.Add(contact);

                        // Has the other body already been added to the island?
                        if (other.m_flags & b2Body.e_islandFlag)
                        {
                            continue;
                        }

                        // Add the other body to the island.
                        other.m_flags |= b2Body.e_islandFlag;

                        if (other.m_type != b2_staticBody)
                        {
                            other.SetAwake(true);
                        }

                        island.Add(other);
                    }
                }
            }

            b2TimeStep subStep;
            subStep.dt      = (1.0f - minAlpha) * step.dt;
            subStep.inv_dt  = 1.0f / subStep.dt;
            subStep.dtRatio = 1.0f;
            subStep.positionIterations = 20;
            subStep.velocityIterations = step.velocityIterations;
            subStep.warmStarting       = false;
            island.SolveTOI(subStep, bA.m_islandIndex, bB.m_islandIndex);

            // Reset island flags and synchronize broad-phase proxies.
            for (int32 i = 0; i < island.m_bodyCount; ++i)
            {
                b2Body* body_ = island.m_bodies[i];
                body_.m_flags &= ~b2Body.e_islandFlag;

                if (body_.m_type != b2_dynamicBody)
                {
                    continue;
                }

                body_.SynchronizeFixtures();

                // Invalidate all contact TOIs on this displaced body_.
                for (b2ContactEdge* ce = body_.m_contactList; ce; ce = ce.next)
                {
                    ce.contact.m_flags &= ~(b2Contact.e_toiFlag | b2Contact.e_islandFlag);
                }
            }

            // Commit fixture proxy movements to the broad-phase so that new contacts are created.
            // Also, some contacts can be destroyed.
            m_contactManager.FindNewContacts();

            if (m_subStepping)
            {
                m_stepComplete = false;
                break;
            }
        }
    }

    void Step(float32 dt, int32 velocityIterations, int32 positionIterations)
    {
        auto stepTimer = b2Timer();

        // If new fixtures were added, we need to find the new contacts.
        if (m_flags & e_newFixture)
        {
            m_contactManager.FindNewContacts();
            m_flags &= ~e_newFixture;
        }

        m_flags |= e_locked;

        b2TimeStep step;
        step.dt = dt;
        step.velocityIterations = velocityIterations;
        step.positionIterations = positionIterations;

        if (dt > 0.0f)
        {
            step.inv_dt = 1.0f / dt;
        }
        else
        {
            step.inv_dt = 0.0f;
        }

        step.dtRatio = m_inv_dt0 * dt;

        step.warmStarting = m_warmStarting;

        // Update contacts. This is where some contacts are destroyed.
        {
            auto timer = b2Timer();
            m_contactManager.Collide();
            m_profile.collide = timer.GetMilliseconds();
        }

        // Integrate velocities, solve velocity constraints, and integrate positions.
        if (m_stepComplete && step.dt > 0.0f)
        {
            auto timer = b2Timer();
            Solve(step);
            m_profile.solve = timer.GetMilliseconds();
        }

        // Handle TOI events.
        if (m_continuousPhysics && step.dt > 0.0f)
        {
            auto timer = b2Timer();
            SolveTOI(step);
            m_profile.solveTOI = timer.GetMilliseconds();
        }

        if (step.dt > 0.0f)
        {
            m_inv_dt0 = step.inv_dt;
        }

        if (m_flags & e_clearForces)
        {
            ClearForces();
        }

        m_flags &= ~e_locked;

        m_profile.step = stepTimer.GetMilliseconds();
    }

    void ClearForces()
    {
        for (b2Body* body_ = m_bodyList; body_; body_ = body_.GetNext())
        {
            body_.m_force.SetZero();
            body_.m_torque = 0.0f;
        }
    }

    struct b2WorldQueryWrapper
    {
        bool QueryCallback(int32 proxyId)
        {
            b2FixtureProxy* proxy = cast(b2FixtureProxy*)broadPhase.GetUserData(proxyId);
            return callback.ReportFixture(proxy.fixture);
        }

        b2BroadPhase* broadPhase;
        b2QueryCallback callback;
    }

    void QueryAABB(b2QueryCallback callback, b2AABB aabb) const
    {
        b2WorldQueryWrapper wrapper;
        wrapper.broadPhase = cast(b2BroadPhase*)&m_contactManager.m_broadPhase;
        wrapper.callback   = callback;
        m_contactManager.m_broadPhase.Query(wrapper, aabb);
    }

    struct b2WorldRayCastWrapper
    {
        float32 RayCastCallback(b2RayCastInput input, int32 proxyId)
        {
            void* userData        = broadPhase.GetUserData(proxyId);
            b2FixtureProxy* proxy = cast(b2FixtureProxy*)userData;
            b2Fixture* fixture    = proxy.fixture;
            int32 index = proxy.childIndex;
            b2RayCastOutput output;
            bool hit = fixture.RayCast(&output, input, index);

            if (hit)
            {
                float32 fraction = output.fraction;
                b2Vec2  point    = (1.0f - fraction) * input.p1 + fraction * input.p2;
                return callback.ReportFixture(fixture, point, output.normal, fraction);
            }

            return input.maxFraction;
        }

        b2BroadPhase* broadPhase;
        b2RayCastCallback callback;
    }

    void RayCast(b2RayCastCallback callback, b2Vec2 point1, b2Vec2 point2) const
    {
        b2WorldRayCastWrapper wrapper;
        wrapper.broadPhase = cast(b2BroadPhase*)&m_contactManager.m_broadPhase;
        wrapper.callback   = callback;
        b2RayCastInput input;
        input.maxFraction = 1.0f;
        input.p1 = point1;
        input.p2 = point2;
        m_contactManager.m_broadPhase.RayCast(wrapper, input);
    }

    void DrawShape(b2Fixture* fixture, b2Transform xf, b2Color color)
    {
        switch (fixture.GetType())
        {
            case b2Shape.e_circle:
            {
                b2CircleShape circle = cast(b2CircleShape)fixture.GetShape();

                b2Vec2  center = b2Mul(xf, circle.m_p);
                float32 radius = circle.m_radius;
                b2Vec2  axis   = b2Mul(xf.q, b2Vec2(1.0f, 0.0f));

                g_debugDraw.DrawSolidCircle(center, radius, axis, color);
            }
            break;

            case b2Shape.e_edge:
            {
                b2EdgeShape edge = cast(b2EdgeShape)fixture.GetShape();
                b2Vec2 v1         = b2Mul(xf, edge.m_vertex1);
                b2Vec2 v2         = b2Mul(xf, edge.m_vertex2);
                g_debugDraw.DrawSegment(v1, v2, color);
            }
            break;

            case b2Shape.e_chain:
            {
                b2ChainShape chain    = cast(b2ChainShape)fixture.GetShape();
                int32 count            = chain.m_count;
                const(b2Vec2)* vertices = chain.m_vertices;

                b2Vec2 v1 = b2Mul(xf, vertices[0]);

                for (int32 i = 1; i < count; ++i)
                {
                    b2Vec2 v2 = b2Mul(xf, vertices[i]);
                    g_debugDraw.DrawSegment(v1, v2, color);
                    g_debugDraw.DrawCircle(v1, 0.05f, color);
                    v1 = v2;
                }
            }
            break;

            case b2Shape.e_polygon:
            {
                b2PolygonShape poly = cast(b2PolygonShape)fixture.GetShape();
                int32 vertexCount    = poly.m_count;
                assert(vertexCount <= b2_maxPolygonVertices);
                b2Vec2 vertices[b2_maxPolygonVertices];

                for (int32 i = 0; i < vertexCount; ++i)
                {
                    vertices[i] = b2Mul(xf, poly.m_vertices[i]);
                }

                g_debugDraw.DrawSolidPolygon(vertices.ptr, vertexCount, color);
            }
            break;

            default:
                break;
        }
    }

    void DrawJoint(b2Joint joint)
    {
        b2Body* body_A = joint.Getbody_A();
        b2Body* body_B = joint.Getbody_B();
        b2Transform xf1 = body_A.GetTransform();
        b2Transform xf2 = body_B.GetTransform();
        b2Vec2 x1 = xf1.p;
        b2Vec2 x2 = xf2.p;
        b2Vec2 p1 = joint.GetAnchorA();
        b2Vec2 p2 = joint.GetAnchorB();

        b2Color color = b2Color(0.5f, 0.8f, 0.8f);

        switch (joint.GetType())
        {
            case e_distanceJoint:
                g_debugDraw.DrawSegment(p1, p2, color);
                break;

            case e_pulleyJoint:
            {
                b2PulleyJoint pulley = cast(b2PulleyJoint)joint;
                b2Vec2 s1 = pulley.GetGroundAnchorA();
                b2Vec2 s2 = pulley.GetGroundAnchorB();
                g_debugDraw.DrawSegment(s1, p1, color);
                g_debugDraw.DrawSegment(s2, p2, color);
                g_debugDraw.DrawSegment(s1, s2, color);
            }
            break;

            case e_mouseJoint:

                // don't draw this
                break;

            default:
                g_debugDraw.DrawSegment(x1, p1, color);
                g_debugDraw.DrawSegment(p1, p2, color);
                g_debugDraw.DrawSegment(x2, p2, color);
        }
    }

    void DrawDebugData()
    {
        if (g_debugDraw is null)
        {
            return;
        }

        uint32 flags = g_debugDraw.GetFlags();

        if (flags & b2Draw.e_shapeBit)
        {
            for (b2Body* b = m_bodyList; b; b = b.GetNext())
            {
                b2Transform xf = b.GetTransform();

                for (b2Fixture* f = b.GetFixtureList(); f; f = f.GetNext())
                {
                    if (b.IsActive() == false)
                    {
                        DrawShape(f, xf, b2Color(0.5f, 0.5f, 0.3f));
                    }
                    else if (b.GetType() == b2_staticBody)
                    {
                        DrawShape(f, xf, b2Color(0.5f, 0.9f, 0.5f));
                    }
                    else if (b.GetType() == b2_kinematicBody)
                    {
                        DrawShape(f, xf, b2Color(0.5f, 0.5f, 0.9f));
                    }
                    else if (b.IsAwake() == false)
                    {
                        DrawShape(f, xf, b2Color(0.6f, 0.6f, 0.6f));
                    }
                    else
                    {
                        DrawShape(f, xf, b2Color(0.9f, 0.7f, 0.7f));
                    }
                }
            }
        }

        if (flags & b2Draw.e_jointBit)
        {
            for (b2Joint j = m_jointList; j; j = j.GetNext())
            {
                DrawJoint(j);
            }
        }

        if (flags & b2Draw.e_pairBit)
        {
            b2Color color = b2Color(0.3f, 0.9f, 0.9f);

            for (b2Contact c = m_contactManager.m_contactList; c; c = c.GetNext())
            {
                // b2Fixture* fixtureA = c.GetFixtureA();
                // b2Fixture* fixtureB = c.GetFixtureB();

                // b2Vec2 cA = fixtureA.GetAABB().GetCenter();
                // b2Vec2 cB = fixtureB.GetAABB().GetCenter();

                // g_debugDraw.DrawSegment(cA, cB, color);
            }
        }

        if (flags & b2Draw.e_aabbBit)
        {
            b2Color color = b2Color(0.9f, 0.3f, 0.9f);
            b2BroadPhase* bp = &m_contactManager.m_broadPhase;

            for (b2Body* b = m_bodyList; b; b = b.GetNext())
            {
                if (b.IsActive() == false)
                {
                    continue;
                }

                for (b2Fixture* f = b.GetFixtureList(); f; f = f.GetNext())
                {
                    for (int32 i = 0; i < f.m_proxyCount; ++i)
                    {
                        b2FixtureProxy* proxy = f.m_proxies + i;
                        b2AABB aabb = bp.GetFatAABB(proxy.proxyId);
                        b2Vec2 vs[4];
                        vs[0].Set(aabb.lowerBound.x, aabb.lowerBound.y);
                        vs[1].Set(aabb.upperBound.x, aabb.lowerBound.y);
                        vs[2].Set(aabb.upperBound.x, aabb.upperBound.y);
                        vs[3].Set(aabb.lowerBound.x, aabb.upperBound.y);

                        g_debugDraw.DrawPolygon(vs.ptr, 4, color);
                    }
                }
            }
        }

        if (flags & b2Draw.e_centerOfMassBit)
        {
            for (b2Body* b = m_bodyList; b; b = b.GetNext())
            {
                b2Transform xf = b.GetTransform();
                xf.p = b.GetWorldCenter();
                g_debugDraw.DrawTransform(xf);
            }
        }
    }

    int32 GetProxyCount() const
    {
        return m_contactManager.m_broadPhase.GetProxyCount();
    }

    int32 GetTreeHeight() const
    {
        return m_contactManager.m_broadPhase.GetTreeHeight();
    }

    int32 GetTreeBalance() const
    {
        return m_contactManager.m_broadPhase.GetTreeBalance();
    }

    float32 GetTreeQuality() const
    {
        return m_contactManager.m_broadPhase.GetTreeQuality();
    }

    void ShiftOrigin(b2Vec2 newOrigin)
    {
        assert((m_flags & e_locked) == 0);

        if ((m_flags & e_locked) == e_locked)
        {
            return;
        }

        for (b2Body* b = m_bodyList; b; b = b.m_next)
        {
            b.m_xf.p     -= newOrigin;
            b.m_sweep.c0 -= newOrigin;
            b.m_sweep.c  -= newOrigin;
        }

        for (b2Joint j = m_jointList; j; j = j.m_next)
        {
            j.ShiftOrigin(newOrigin);
        }

        m_contactManager.m_broadPhase.ShiftOrigin(newOrigin);
    }

    void Dump()
    {
        if ((m_flags & e_locked) == e_locked)
        {
            return;
        }

        b2Log("b2Vec2 g(%.15lef, %.15lef);\n", m_gravity.x, m_gravity.y);
        b2Log("m_world.SetGravity(g);\n");

        b2Log("b2Body** bodies = cast(b2Body**)b2Alloc(%d * (b2Body*).sizeof);\n", m_bodyCount);
        b2Log("b2Joint joints = cast(b2Joint)b2Alloc(%d * (b2Joint).sizeof);\n", m_jointCount);
        int32 i = 0;

        for (b2Body* b = m_bodyList; b; b = b.m_next)
        {
            b.m_islandIndex = i;
            b.Dump();
            ++i;
        }

        i = 0;

        for (b2Joint j = m_jointList; j; j = j.m_next)
        {
            j.m_index = i;
            ++i;
        }

        // First pass on joints, skip gear joints.
        for (b2Joint j = m_jointList; j; j = j.m_next)
        {
            if (j.m_type == e_gearJoint)
            {
                continue;
            }

            b2Log("{\n");
            j.Dump();
            b2Log("}\n");
        }

        // Second pass on joints, only gear joints.
        for (b2Joint j = m_jointList; j; j = j.m_next)
        {
            if (j.m_type != e_gearJoint)
            {
                continue;
            }

            b2Log("{\n");
            j.Dump();
            b2Log("}\n");
        }

        b2Log("b2Free(joints);\n");
        b2Log("b2Free(bodies);\n");
        b2Log("joints = null;\n");
        b2Log("bodies = null;\n");
    }

    bool GetAllowSleeping() const
    {
        return m_allowSleep;
    }

    /// Enable/disable warm starting. For testing.
    void SetWarmStarting(bool flag)
    {
        m_warmStarting = flag;
    }

    bool GetWarmStarting() const
    {
        return m_warmStarting;
    }

    /// Enable/disable continuous physics. For testing.
    void SetContinuousPhysics(bool flag)
    {
        m_continuousPhysics = flag;
    }

    bool GetContinuousPhysics() const
    {
        return m_continuousPhysics;
    }

    /// Enable/disable single stepped continuous physics. For testing.
    void SetSubStepping(bool flag)
    {
        m_subStepping = flag;
    }

    bool GetSubStepping() const
    {
        return m_subStepping;
    }

/* private */

    // m_flags
    enum
    {
        e_newFixture = 0x0001,
        e_locked      = 0x0002,
        e_clearForces = 0x0004
    }

    public b2BlockAllocator m_blockAllocator;
    public b2StackAllocator m_stackAllocator;

    int32 m_flags;

    public b2ContactManager m_contactManager;

    b2Body* m_bodyList;
    b2Joint m_jointList;

    int32 m_bodyCount;
    int32 m_jointCount;

    b2Vec2 m_gravity;
    bool m_allowSleep;

    b2DestructionListener m_destructionListener;
    b2Draw g_debugDraw;

    // This is used to compute the time step ratio to
    // support a variable time step.
    float32 m_inv_dt0 = 0;

    // These are for debugging the solver.
    bool m_warmStarting;
    bool m_continuousPhysics;
    bool m_subStepping;

    bool m_stepComplete;

    b2Profile m_profile;
}

import dbox.dynamics.b2world;
import dbox.dynamics.b2body;
import dbox.dynamics.b2fixture;
import dbox.dynamics.b2island;
import dbox.dynamics.joints.b2pulleyjoint;
import dbox.dynamics.contacts.b2contact;
import dbox.dynamics.contacts.b2contactsolver;
import dbox.collision.b2collision;
import dbox.collision.b2broadphase;
import dbox.collision.shapes.b2circleshape;
import dbox.collision.shapes.b2edgeshape;
import dbox.collision.shapes.b2chainshape;
import dbox.collision.shapes.b2polygonshape;
import dbox.collision.b2timeofimpact;
import dbox.common.b2draw;
import dbox.common.b2timer;

