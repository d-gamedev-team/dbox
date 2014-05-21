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
module dbox.dynamics.contacts.b2polygoncontact;

import core.stdc.float_;
import core.stdc.stdlib;
import core.stdc.string;

import dbox.collision;
import dbox.collision.shapes;
import dbox.common;
import dbox.dynamics;
import dbox.dynamics.contacts;

///
class b2PolygonContact : b2Contact
{
    ///
    static b2Contact Create(b2Fixture* fixtureA, int32, b2Fixture* fixtureB, int32, b2BlockAllocator* allocator)
    {
        void* mem = allocator.Allocate(b2memSizeOf!b2PolygonContact);
        return b2emplace!b2PolygonContact(mem, fixtureA, fixtureB);
    }

    ///
    static void Destroy(b2Contact contact, b2BlockAllocator* allocator)
    {
        destroy(contact);
        allocator.Free(cast(void*)contact, b2memSizeOf!b2PolygonContact);
    }

    ///
    this(b2Fixture* fixtureA, b2Fixture* fixtureB)
    {
        super(fixtureA, 0, fixtureB, 0);
        assert(m_fixtureA.GetType() == b2Shape.e_polygon);
        assert(m_fixtureB.GetType() == b2Shape.e_polygon);
    }

    ///
    override void Evaluate(b2Manifold* manifold, b2Transform xfA, b2Transform xfB)
    {
        b2CollidePolygons(manifold,
                          cast(b2PolygonShape)m_fixtureA.GetShape(), xfA,
                          cast(b2PolygonShape)m_fixtureB.GetShape(), xfB);
    }
}
