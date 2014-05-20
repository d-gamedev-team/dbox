module dbox.dynamics.contacts.b2circlecontact;

import core.stdc.float_;
import core.stdc.stdlib;
import core.stdc.string;

import dbox.common;
import dbox.collision.shapes;
import dbox.common.b2math;
import dbox.collision;
import dbox.dynamics;

import dbox.dynamics.contacts.b2contact;

class b2CircleContact : b2Contact
{
    static b2Contact Create(b2Fixture* fixtureA, int32, b2Fixture* fixtureB, int32, b2BlockAllocator* allocator)
    {
        //~ void* mem = allocator.Allocate(getSizeOf!b2CircleContact);
        //~ return emplace!b2CircleContact(mem, fixtureA, fixtureB);

        return new b2CircleContact(fixtureA, fixtureB);
    }

    static void Destroy(b2Contact contact, b2BlockAllocator* allocator)
    {
        typeid(cast(b2CircleContact)contact).destroy(&contact);
        allocator.Free(cast(void*)contact, getSizeOf!b2CircleContact);
    }

    this(b2Fixture* fixtureA, b2Fixture* fixtureB)
    {
        super(fixtureA, 0, fixtureB, 0);
        assert(m_fixtureA.GetType() == b2Shape.e_circle);
        assert(m_fixtureB.GetType() == b2Shape.e_circle);
    }

    override void Evaluate(b2Manifold* manifold, b2Transform xfA, b2Transform xfB)
    {
        b2CollideCircles(manifold,
                         cast(b2CircleShape)m_fixtureA.GetShape(), xfA,
                         cast(b2CircleShape)m_fixtureB.GetShape(), xfB);
    }
}

import dbox.dynamics.contacts.b2circlecontact;
import dbox.dynamics.b2body;
import dbox.dynamics.b2fixture;
import dbox.dynamics.b2worldcallbacks;
import dbox.common.b2blockallocator;
import dbox.collision.b2timeofimpact;
