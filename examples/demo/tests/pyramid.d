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
module tests.pyramid;

import core.stdc.math;

import std.string;
import std.typecons;

import deimos.glfw.glfw3;

import dbox;

import framework.debug_draw;
import framework.test;

class Pyramid : Test
{
    enum
    {
        e_count = 20
    }

    this()
    {
        {
            b2BodyDef bd;
            b2Body* ground = m_world.CreateBody(&bd);

            auto shape = new b2EdgeShape();
            shape.Set(b2Vec2(-40.0f, 0.0f), b2Vec2(40.0f, 0.0f));
            ground.CreateFixture(shape, 0.0f);
        }

        {
            float32 a = 0.5f;
            auto shape = new b2PolygonShape();
            shape.SetAsBox(a, a);

            b2Vec2 x = b2Vec2(-7.0f, 0.75f);
            b2Vec2 y;
            b2Vec2 deltaX = b2Vec2(0.5625f, 1.25f);
            b2Vec2 deltaY = b2Vec2(1.125f, 0.0f);

            for (int32 i = 0; i < e_count; ++i)
            {
                y = x;

                for (int32 j = i; j < e_count; ++j)
                {
                    b2BodyDef bd;
                    bd.type     = b2_dynamicBody;
                    bd.position = y;
                    b2Body* body_ = m_world.CreateBody(&bd);
                    body_.CreateFixture(shape, 5.0f);

                    y += deltaY;
                }

                x += deltaX;
            }
        }
    }

    static Test Create()
    {
        return new typeof(this);
    }
}
