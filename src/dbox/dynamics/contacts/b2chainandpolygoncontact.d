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
module dbox.dynamics.contacts.b2chainandpolygoncontact;

import core.stdc.float_;
import core.stdc.stdlib;
import core.stdc.string;

import dbox.collision;
import dbox.collision.shapes;
import dbox.common;
import dbox.dynamics;
import dbox.dynamics.contacts;

///
class b2ChainAndPolygonContact : b2Contact
{
    ///
    static b2Contact Create(b2Fixture* fixtureA, int32 indexA, b2Fixture* fixtureB, int32 indexB, b2BlockAllocator* allocator)
    {
        void* mem = allocator.Allocate(b2memSizeOf!b2ChainAndPolygonContact);
        return b2emplace!b2ChainAndPolygonContact(mem, fixtureA, indexA, fixtureB, indexB);
    }

    ///
    static void Destroy(b2Contact contact, b2BlockAllocator* allocator)
    {
        destroy(contact);
        allocator.Free(cast(void*)contact, b2memSizeOf!b2ChainAndPolygonContact);
    }

    ///
    this(b2Fixture* fixtureA, int32 indexA, b2Fixture* fixtureB, int32 indexB)
    {
        super(fixtureA, indexA, fixtureB, indexB);
        assert(m_fixtureA.GetType() == b2Shape.e_chain);
        assert(m_fixtureB.GetType() == b2Shape.e_polygon);
    }

    ///
    override void Evaluate(b2Manifold* manifold, b2Transform xfA, b2Transform xfB)
    {
        b2ChainShape chain = cast(b2ChainShape)m_fixtureA.GetShape();

        import std.typecons;
        auto edge = scoped!b2EdgeShape();

        chain.GetChildEdge(edge, m_indexA);
        b2CollideEdgeAndPolygon(manifold, edge, xfA,
                                cast(b2PolygonShape)m_fixtureB.GetShape(), xfB);
    }
}
