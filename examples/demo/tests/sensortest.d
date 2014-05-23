/*
 * Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
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
module tests.sensortest;

import core.stdc.float_;
import core.stdc.math;

import std.string;
import std.typecons;

import deimos.glfw.glfw3;

import dbox;

import framework.debug_draw;
import framework.test;

// This is used to test sensor shapes.
class SensorTest : Test
{
    enum
    {
        e_count = 7
    }

    this()
    {
        {
            b2BodyDef bd;
            b2Body* ground = m_world.CreateBody(&bd);

            {
                auto shape = new b2EdgeShape();
                shape.Set(b2Vec2(-40.0f, 0.0f), b2Vec2(40.0f, 0.0f));
                ground.CreateFixture(shape, 0.0f);
            }

            {
                b2CircleShape shape = new b2CircleShape();
                shape.m_radius = 5.0f;
                shape.m_p.Set(0.0f, 10.0f);

                b2FixtureDef fd;
                fd.shape    = shape;
                fd.isSensor = true;
                m_sensor    = ground.CreateFixture(&fd);
            }
        }

        {
            b2CircleShape shape = new b2CircleShape();
            shape.m_radius = 1.0f;

            for (int32 i = 0; i < e_count; ++i)
            {
                b2BodyDef bd;
                bd.type = b2_dynamicBody;
                bd.position.Set(-10.0f + 3.0f * i, 20.0f);
                bd.userData = &m_touching[i];

                m_touching[i] = false;
                m_bodies[i]   = m_world.CreateBody(&bd);

                m_bodies[i].CreateFixture(shape, 1.0f);
            }
        }
    }

    // Implement contact listener.
    override void BeginContact(b2Contact contact)
    {
        b2Fixture* fixtureA = contact.GetFixtureA();
        b2Fixture* fixtureB = contact.GetFixtureB();

        if (fixtureA == m_sensor)
        {
            void* userData = fixtureB.GetBody().GetUserData();

            if (userData)
            {
                bool* touching = cast(bool*)userData;
                *touching = true;
            }
        }

        if (fixtureB == m_sensor)
        {
            void* userData = fixtureA.GetBody().GetUserData();

            if (userData)
            {
                bool* touching = cast(bool*)userData;
                *touching = true;
            }
        }
    }

    // Implement contact listener.
    override void EndContact(b2Contact contact)
    {
        b2Fixture* fixtureA = contact.GetFixtureA();
        b2Fixture* fixtureB = contact.GetFixtureB();

        if (fixtureA == m_sensor)
        {
            void* userData = fixtureB.GetBody().GetUserData();

            if (userData)
            {
                bool* touching = cast(bool*)userData;
                *touching = false;
            }
        }

        if (fixtureB == m_sensor)
        {
            void* userData = fixtureA.GetBody().GetUserData();

            if (userData)
            {
                bool* touching = cast(bool*)userData;
                *touching = false;
            }
        }
    }

    override void Step(Settings* settings)
    {
        super.Step(settings);

        // Traverse the contact results. Apply a force on shapes
        // that overlap the sensor.
        for (int32 i = 0; i < e_count; ++i)
        {
            if (m_touching[i] == false)
            {
                continue;
            }

            b2Body* body_  = m_bodies[i];
            b2Body* ground = m_sensor.GetBody();

            b2CircleShape circle = cast(b2CircleShape)m_sensor.GetShape();
            b2Vec2 center         = ground.GetWorldPoint(circle.m_p);

            b2Vec2 position = body_.GetPosition();

            b2Vec2 d = center - position;

            if (d.LengthSquared() < FLT_EPSILON * FLT_EPSILON)
            {
                continue;
            }

            d.Normalize();
            b2Vec2 F = 100.0f * d;
            body_.ApplyForce(F, position, false);
        }
    }

    static Test Create()
    {
        return new typeof(this);
    }

    b2Fixture* m_sensor;
    b2Body* m_bodies[e_count];
    bool m_touching[e_count];
}
