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
module tests.compoundshapes;

import core.stdc.math;

import std.algorithm;
import std.string;
import std.typecons;

import deimos.glfw.glfw3;

import dbox;

import framework.debug_draw;
import framework.test;

class CompoundShapes : Test
{
    this()
    {
        {
            b2BodyDef bd;
            bd.position.Set(0.0f, 0.0f);
            b2Body* body_ = m_world.CreateBody(&bd);

            auto shape = new b2EdgeShape();
            shape.Set(b2Vec2(50.0f, 0.0f), b2Vec2(-50.0f, 0.0f));

            body_.CreateFixture(shape, 0.0f);
        }

        {
            b2CircleShape circle1 = new b2CircleShape();
            circle1.m_radius = 0.5f;
            circle1.m_p.Set(-0.5f, 0.5f);

            b2CircleShape circle2 = new b2CircleShape();
            circle2.m_radius = 0.5f;
            circle2.m_p.Set(0.5f, 0.5f);

            for (int i = 0; i < 10; ++i)
            {
                float32 x = RandomFloat(-0.1f, 0.1f);
                b2BodyDef bd;
                bd.type = b2_dynamicBody;
                bd.position.Set(x + 5.0f, 1.05f + 2.5f * i);
                bd.angle = RandomFloat(-b2_pi, b2_pi);
                b2Body* body_ = m_world.CreateBody(&bd);
                body_.CreateFixture(circle1, 2.0f);
                body_.CreateFixture(circle2, 0.0f);
            }
        }

        {
            b2PolygonShape polygon1 = new b2PolygonShape();
            polygon1.SetAsBox(0.25f, 0.5f);

            b2PolygonShape polygon2 = new b2PolygonShape();
            polygon2.SetAsBox(0.25f, 0.5f, b2Vec2(0.0f, -0.5f), 0.5f * b2_pi);

            for (int i = 0; i < 10; ++i)
            {
                float32 x = RandomFloat(-0.1f, 0.1f);
                b2BodyDef bd;
                bd.type = b2_dynamicBody;
                bd.position.Set(x - 5.0f, 1.05f + 2.5f * i);
                bd.angle = RandomFloat(-b2_pi, b2_pi);
                b2Body* body_ = m_world.CreateBody(&bd);
                body_.CreateFixture(polygon1, 2.0f);
                body_.CreateFixture(polygon2, 2.0f);
            }
        }

        {
            b2Transform xf1;
            xf1.q.Set(0.3524f * b2_pi);
            xf1.p = xf1.q.GetXAxis();

            b2Vec2 vertices[3];

            b2PolygonShape triangle1 = new b2PolygonShape();
            vertices[0] = b2Mul(xf1, b2Vec2(-1.0f, 0.0f));
            vertices[1] = b2Mul(xf1, b2Vec2(1.0f, 0.0f));
            vertices[2] = b2Mul(xf1, b2Vec2(0.0f, 0.5f));
            triangle1.Set(vertices);

            b2Transform xf2;
            xf2.q.Set(-0.3524f * b2_pi);
            xf2.p = -xf2.q.GetXAxis();

            b2PolygonShape triangle2 = new b2PolygonShape();
            vertices[0] = b2Mul(xf2, b2Vec2(-1.0f, 0.0f));
            vertices[1] = b2Mul(xf2, b2Vec2(1.0f, 0.0f));
            vertices[2] = b2Mul(xf2, b2Vec2(0.0f, 0.5f));
            triangle2.Set(vertices);

            for (int32 i = 0; i < 10; ++i)
            {
                float32 x = RandomFloat(-0.1f, 0.1f);
                b2BodyDef bd;
                bd.type = b2_dynamicBody;
                bd.position.Set(x, 2.05f + 2.5f * i);
                bd.angle = 0.0f;
                b2Body* body_ = m_world.CreateBody(&bd);
                body_.CreateFixture(triangle1, 2.0f);
                body_.CreateFixture(triangle2, 2.0f);
            }
        }

        {
            b2PolygonShape bottom = new b2PolygonShape();
            bottom.SetAsBox(1.5f, 0.15f);

            b2PolygonShape left = new b2PolygonShape();
            left.SetAsBox(0.15f, 2.7f, b2Vec2(-1.45f, 2.35f), 0.2f);

            b2PolygonShape right = new b2PolygonShape();
            right.SetAsBox(0.15f, 2.7f, b2Vec2(1.45f, 2.35f), -0.2f);

            b2BodyDef bd;
            bd.type = b2_dynamicBody;
            bd.position.Set(0.0f, 2.0f);
            b2Body* body_ = m_world.CreateBody(&bd);
            body_.CreateFixture(bottom, 4.0f);
            body_.CreateFixture(left, 4.0f);
            body_.CreateFixture(right, 4.0f);
        }
    }

    static Test Create()
    {
        return new typeof(this);
    }
}
