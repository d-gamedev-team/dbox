module dbox.dynamics.b2contactmanager;

import core.stdc.float_;
import core.stdc.stdlib;
import core.stdc.string;

import dbox.common;
import dbox.collision;
import dbox.dynamics;
import dbox.dynamics.contacts;
import dbox.dynamics.joints;

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

// #ifndef B2_CONTACT_MANAGER_H
// #define B2_CONTACT_MANAGER_H

import dbox.collision.b2broadphase;

// Delegate of b2World*.
struct b2ContactManager
{
    @disable this();
    @disable this(this);

    this(int)
    {
        m_contactList     = null;
        m_contactCount    = 0;
        m_contactFilter   = b2_defaultFilter;
        m_contactListener = b2_defaultListener;
        m_broadPhase      = b2BroadPhase(1);
        m_allocator       = null;
    }

    b2ContactFilter m_contactFilter;
    b2ContactListener m_contactListener;

    void Destroy(b2Contact c)
    {
        b2Fixture* fixtureA = c.GetFixtureA();
        b2Fixture* fixtureB = c.GetFixtureB();
        b2Body* body_A       = fixtureA.GetBody();
        b2Body* body_B       = fixtureB.GetBody();

        if (m_contactListener && c.IsTouching())
        {
            m_contactListener.EndContact(c);
        }

        // Remove from the world.
        if (c.m_prev)
        {
            c.m_prev.m_next = c.m_next;
        }

        if (c.m_next)
        {
            c.m_next.m_prev = c.m_prev;
        }

        if (c == m_contactList)
        {
            m_contactList = c.m_next;
        }

        // Remove from body 1
        if (c.m_nodeA.prev)
        {
            c.m_nodeA.prev.next = c.m_nodeA.next;
        }

        if (c.m_nodeA.next)
        {
            c.m_nodeA.next.prev = c.m_nodeA.prev;
        }

        if (&c.m_nodeA == body_A.m_contactList)
        {
            body_A.m_contactList = c.m_nodeA.next;
        }

        // Remove from body 2
        if (c.m_nodeB.prev)
        {
            c.m_nodeB.prev.next = c.m_nodeB.next;
        }

        if (c.m_nodeB.next)
        {
            c.m_nodeB.next.prev = c.m_nodeB.prev;
        }

        if (&c.m_nodeB == body_B.m_contactList)
        {
            body_B.m_contactList = c.m_nodeB.next;
        }

        // Call the factory.
        b2Contact.Destroy(c, m_allocator);
        --m_contactCount;
    }

    // This is the top level collision call for the time step. Here
    // all the narrow phase collision is processed for the world
    // contact list.
    void Collide()
    {
        // Update awake contacts.
        b2Contact c = m_contactList;

        while (c)
        {
            b2Fixture* fixtureA = c.GetFixtureA();
            b2Fixture* fixtureB = c.GetFixtureB();
            int32 indexA        = c.GetChildIndexA();
            int32 indexB        = c.GetChildIndexB();
            b2Body* body_A       = fixtureA.GetBody();
            b2Body* body_B       = fixtureB.GetBody();

            // Is this contact flagged for filtering?
            if (c.m_flags & b2Contact.e_filterFlag)
            {
                // Should these bodies collide?
                if (body_B.ShouldCollide(body_A) == false)
                {
                    b2Contact cNuke = c;
                    c = cNuke.GetNext();
                    Destroy(cNuke);
                    continue;
                }

                // Check user filtering.
                if (m_contactFilter && m_contactFilter.ShouldCollide(fixtureA, fixtureB) == false)
                {
                    b2Contact cNuke = c;
                    c = cNuke.GetNext();
                    Destroy(cNuke);
                    continue;
                }

                // Clear the filtering flag.
                c.m_flags &= ~b2Contact.e_filterFlag;
            }

            bool activeA = body_A.IsAwake() && body_A.m_type != b2_staticBody;
            bool activeB = body_B.IsAwake() && body_B.m_type != b2_staticBody;

            // At least one body must be awake and it must be dynamic or kinematic.
            if (activeA == false && activeB == false)
            {
                c = c.GetNext();
                continue;
            }

            int32 proxyIdA = fixtureA.m_proxies[indexA].proxyId;
            int32 proxyIdB = fixtureB.m_proxies[indexB].proxyId;
            bool  overlap  = m_broadPhase.TestOverlap(proxyIdA, proxyIdB);

            // Here we destroy contacts that cease to overlap in the broad-phase.
            if (overlap == false)
            {
                b2Contact cNuke = c;
                c = cNuke.GetNext();
                Destroy(cNuke);
                continue;
            }

            // The contact persists.
            c.Update(m_contactListener);
            c = c.GetNext();
        }
    }

    void FindNewContacts()
    {
        m_broadPhase.UpdatePairs(this);
    }

    void AddPair(void* proxyUserDataA, void* proxyUserDataB)
    {
        b2FixtureProxy* proxyA = cast(b2FixtureProxy*)proxyUserDataA;
        b2FixtureProxy* proxyB = cast(b2FixtureProxy*)proxyUserDataB;

        b2Fixture* fixtureA = proxyA.fixture;
        b2Fixture* fixtureB = proxyB.fixture;

        int32 indexA = proxyA.childIndex;
        int32 indexB = proxyB.childIndex;

        b2Body* body_A = fixtureA.GetBody();
        b2Body* body_B = fixtureB.GetBody();

        // Are the fixtures on the same body?
        if (body_A == body_B)
        {
            return;
        }

        // TODO_ERIN use a hash table to remove a potential bottleneck when both
        // bodies have a lot of contacts.
        // Does a contact already exist?
        b2ContactEdge* edge = body_B.GetContactList();

        while (edge)
        {
            if (edge.other == body_A)
            {
                b2Fixture* fA = edge.contact.GetFixtureA();
                b2Fixture* fB = edge.contact.GetFixtureB();
                int32 iA      = edge.contact.GetChildIndexA();
                int32 iB      = edge.contact.GetChildIndexB();

                if (fA == fixtureA && fB == fixtureB && iA == indexA && iB == indexB)
                {
                    // A contact already exists.
                    return;
                }

                if (fA == fixtureB && fB == fixtureA && iA == indexB && iB == indexA)
                {
                    // A contact already exists.
                    return;
                }
            }

            edge = edge.next;
        }

        // Does a joint override collision? Is at least one body dynamic?
        if (body_B.ShouldCollide(body_A) == false)
        {
            return;
        }

        // Check user filtering.
        if (m_contactFilter && m_contactFilter.ShouldCollide(fixtureA, fixtureB) == false)
        {
            return;
        }

        // Call the factory.
        b2Contact c = b2Contact.Create(fixtureA, indexA, fixtureB, indexB, m_allocator);

        if (c is null)
        {
            return;
        }

        // Contact creation may swap fixtures.
        fixtureA = c.GetFixtureA();
        fixtureB = c.GetFixtureB();
        indexA   = c.GetChildIndexA();
        indexB   = c.GetChildIndexB();
        body_A    = fixtureA.GetBody();
        body_B    = fixtureB.GetBody();

        // Insert into the world.
        c.m_prev = null;
        c.m_next = m_contactList;

        if (m_contactList !is null)
        {
            m_contactList.m_prev = c;
        }
        m_contactList = c;

        // Connect to island graph.

        // Connect to body A
        c.m_nodeA.contact = c;
        c.m_nodeA.other   = body_B;

        c.m_nodeA.prev = null;
        c.m_nodeA.next = body_A.m_contactList;

        if (body_A.m_contactList !is null)
        {
            body_A.m_contactList.prev = &c.m_nodeA;
        }
        body_A.m_contactList = &c.m_nodeA;

        // Connect to body B
        c.m_nodeB.contact = c;
        c.m_nodeB.other   = body_A;

        c.m_nodeB.prev = null;
        c.m_nodeB.next = body_B.m_contactList;

        if (body_B.m_contactList !is null)
        {
            body_B.m_contactList.prev = &c.m_nodeB;
        }
        body_B.m_contactList = &c.m_nodeB;

        // Wake up the bodies
        if (fixtureA.IsSensor() == false && fixtureB.IsSensor() == false)
        {
            body_A.SetAwake(true);
            body_B.SetAwake(true);
        }

        ++m_contactCount;
    }

    b2BroadPhase m_broadPhase;
    b2Contact m_contactList;
    int32 m_contactCount;
    b2BlockAllocator* m_allocator;
}

import dbox.dynamics.b2contactmanager;
import dbox.dynamics.b2body;
import dbox.dynamics.b2fixture;
import dbox.dynamics.b2worldcallbacks;
import dbox.dynamics.contacts.b2contact;

__gshared b2ContactFilter b2_defaultFilter;
__gshared b2ContactListener b2_defaultListener;
