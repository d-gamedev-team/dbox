module dbox.dynamics.contacts.b2chainandpolygoncontact;

import core.stdc.float_;
import core.stdc.stdlib;
import core.stdc.string;

import dbox.common;
import dbox.collision.shapes;
import dbox.common.b2math;
import dbox.dynamics;
import dbox.collision;

import dbox.dynamics.contacts.b2contact;

class b2ChainAndPolygonContact : b2Contact
{
    static b2Contact Create(b2Fixture* fixtureA, int32 indexA, b2Fixture* fixtureB, int32 indexB, b2BlockAllocator* allocator)
    {
        //~ void* mem = allocator.Allocate(getSizeOf!b2ChainAndPolygonContact);
        //~ return emplace!b2ChainAndPolygonContact(mem, fixtureA, indexA, fixtureB, indexB);

        return new b2ChainAndPolygonContact(fixtureA, indexA, fixtureB, indexB);
    }

    static void Destroy(b2Contact contact, b2BlockAllocator* allocator)
    {
        typeid(cast(b2ChainAndPolygonContact)contact).destroy(&contact);
        allocator.Free(cast(void*)contact, getSizeOf!b2ChainAndPolygonContact);
    }

    this(b2Fixture* fixtureA, int32 indexA, b2Fixture* fixtureB, int32 indexB)
    {
        super(fixtureA, indexA, fixtureB, indexB);
        assert(m_fixtureA.GetType() == b2Shape.e_chain);
        assert(m_fixtureB.GetType() == b2Shape.e_polygon);
    }

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

import dbox.dynamics.contacts.b2chainandpolygoncontact;
import dbox.common.b2blockallocator;
import dbox.dynamics.b2fixture;
import dbox.collision.shapes.b2chainshape;
import dbox.collision.shapes.b2edgeshape;
