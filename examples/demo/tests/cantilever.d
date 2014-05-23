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
module tests.cantilever;

import core.stdc.math;

import std.string;
import std.typecons;

import deimos.glfw.glfw3;

import dbox;

import framework.debug_draw;
import framework.test;

class Cantilever : Test
{
    enum
    {
        e_count = 8,
    }

    this()
    {
        b2Body* ground = null;
        {
            b2BodyDef bd;
            ground = m_world.CreateBody(&bd);

            auto shape = new b2EdgeShape();
            shape.Set(b2Vec2(-40.0f, 0.0f), b2Vec2(40.0f, 0.0f));
            ground.CreateFixture(shape, 0.0f);
        }

        {
            auto shape = new b2PolygonShape();
            shape.SetAsBox(0.5f, 0.125f);

            b2FixtureDef fd;
            fd.shape   = shape;
            fd.density = 20.0f;

            b2WeldJointDef jd = new b2WeldJointDef();

            b2Body* prevBody = ground;

            for (int32 i = 0; i < e_count; ++i)
            {
                b2BodyDef bd;
                bd.type = b2_dynamicBody;
                bd.position.Set(-14.5f + 1.0f * i, 5.0f);
                b2Body* body_ = m_world.CreateBody(&bd);
                body_.CreateFixture(&fd);

                b2Vec2 anchor = b2Vec2(-15.0f + 1.0f * i, 5.0f);
                jd.Initialize(prevBody, body_, anchor);
                m_world.CreateJoint(jd);

                prevBody = body_;
            }
        }

        {
            auto shape = new b2PolygonShape();
            shape.SetAsBox(1.0f, 0.125f);

            b2FixtureDef fd;
            fd.shape   = shape;
            fd.density = 20.0f;

            b2WeldJointDef jd = new b2WeldJointDef();
            jd.frequencyHz  = 5.0f;
            jd.dampingRatio = 0.7f;

            b2Body* prevBody = ground;

            for (int32 i = 0; i < 3; ++i)
            {
                b2BodyDef bd;
                bd.type = b2_dynamicBody;
                bd.position.Set(-14.0f + 2.0f * i, 15.0f);
                b2Body* body_ = m_world.CreateBody(&bd);
                body_.CreateFixture(&fd);

                b2Vec2 anchor = b2Vec2(-15.0f + 2.0f * i, 15.0f);
                jd.Initialize(prevBody, body_, anchor);
                m_world.CreateJoint(jd);

                prevBody = body_;
            }
        }

        {
            auto shape = new b2PolygonShape();
            shape.SetAsBox(0.5f, 0.125f);

            b2FixtureDef fd;
            fd.shape   = shape;
            fd.density = 20.0f;

            b2WeldJointDef jd = new b2WeldJointDef();

            b2Body* prevBody = ground;

            for (int32 i = 0; i < e_count; ++i)
            {
                b2BodyDef bd;
                bd.type = b2_dynamicBody;
                bd.position.Set(-4.5f + 1.0f * i, 5.0f);
                b2Body* body_ = m_world.CreateBody(&bd);
                body_.CreateFixture(&fd);

                if (i > 0)
                {
                    b2Vec2 anchor = b2Vec2(-5.0f + 1.0f * i, 5.0f);
                    jd.Initialize(prevBody, body_, anchor);
                    m_world.CreateJoint(jd);
                }

                prevBody = body_;
            }
        }

        {
            auto shape = new b2PolygonShape();
            shape.SetAsBox(0.5f, 0.125f);

            b2FixtureDef fd;
            fd.shape   = shape;
            fd.density = 20.0f;

            b2WeldJointDef jd = new b2WeldJointDef();
            jd.frequencyHz  = 8.0f;
            jd.dampingRatio = 0.7f;

            b2Body* prevBody = ground;

            for (int32 i = 0; i < e_count; ++i)
            {
                b2BodyDef bd;
                bd.type = b2_dynamicBody;
                bd.position.Set(5.5f + 1.0f * i, 10.0f);
                b2Body* body_ = m_world.CreateBody(&bd);
                body_.CreateFixture(&fd);

                if (i > 0)
                {
                    b2Vec2 anchor = b2Vec2(5.0f + 1.0f * i, 10.0f);
                    jd.Initialize(prevBody, body_, anchor);
                    m_world.CreateJoint(jd);
                }

                prevBody = body_;
            }
        }

        for (int32 i = 0; i < 2; ++i)
        {
            b2Vec2 vertices[3];
            vertices[0].Set(-0.5f, 0.0f);
            vertices[1].Set(0.5f, 0.0f);
            vertices[2].Set(0.0f, 1.5f);

            auto shape = new b2PolygonShape();
            shape.Set(vertices);

            b2FixtureDef fd;
            fd.shape   = shape;
            fd.density = 1.0f;

            b2BodyDef bd;
            bd.type = b2_dynamicBody;
            bd.position.Set(-8.0f + 8.0f * i, 12.0f);
            b2Body* body_ = m_world.CreateBody(&bd);
            body_.CreateFixture(&fd);
        }

        for (int32 i = 0; i < 2; ++i)
        {
            b2CircleShape shape = new b2CircleShape();
            shape.m_radius = 0.5f;

            b2FixtureDef fd;
            fd.shape   = shape;
            fd.density = 1.0f;

            b2BodyDef bd;
            bd.type = b2_dynamicBody;
            bd.position.Set(-6.0f + 6.0f * i, 10.0f);
            b2Body* body_ = m_world.CreateBody(&bd);
            body_.CreateFixture(&fd);
        }
    }

    static Test Create()
    {
        return new typeof(this);
    }

    b2Body* m_middle;
}
