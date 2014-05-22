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
module tests.addpair;

import core.stdc.math;

import std.string;
import std.typecons;

import deimos.glfw.glfw3;

import dbox;

import framework.debug_draw;
import framework.test;

class AddPair : Test
{
    this()
    {
        m_world.SetGravity(b2Vec2(0.0f, 0.0f));
        {
            auto shape = new b2CircleShape();
            shape.m_p.SetZero();
            shape.m_radius = 0.1f;

            float minX = -6.0f;
            float maxX = 0.0f;
            float minY = 4.0f;
            float maxY = 6.0f;

            for (int32 i = 0; i < 400; ++i)
            {
                b2BodyDef bd;
                bd.type     = b2_dynamicBody;
                bd.position = b2Vec2(RandomFloat(minX, maxX), RandomFloat(minY, maxY));
                b2Body* body_ = m_world.CreateBody(&bd);
                body_.CreateFixture(shape, 0.01f);
            }
        }

        {
            auto shape = new b2PolygonShape();
            shape.SetAsBox(1.5f, 1.5f);
            b2BodyDef bd;
            bd.type = b2_dynamicBody;
            bd.position.Set(-40.0f, 5.0f);
            bd.bullet = true;
            b2Body* body_ = m_world.CreateBody(&bd);
            body_.CreateFixture(shape, 1.0f);
            body_.SetLinearVelocity(b2Vec2(150.0f, 0.0f));
        }
    }

    static Test Create()
    {
        return new typeof(this);
    }
}
