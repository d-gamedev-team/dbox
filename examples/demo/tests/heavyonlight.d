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

module tests.heavyonlight;

import core.stdc.math;

import std.string;
import std.typecons;

import dbox;

import framework.debug_draw;
import framework.test;

/// This stress tests the dynamic tree broad-phase. This also shows that tile
/// based collision is _not_ smooth due to Box2D not knowing about adjacency.
class HeavyOnLight : Test
{
    this()
    {
        {
            b2BodyDef bd;
            b2Body* ground = m_world.CreateBody(&bd);

            auto shape = scoped!b2EdgeShape();
            shape.Set(b2Vec2(-40.0f, 0.0f), b2Vec2(40.0f, 0.0f));
            ground.CreateFixture(shape, 0.0f);
        }

        b2BodyDef bd;
        bd.type = b2_dynamicBody;
        bd.position.Set(0.0f, 0.5f);
        b2Body* body_ = m_world.CreateBody(&bd);

        auto shape = scoped!b2CircleShape();
        shape.m_radius = 0.5f;
        body_.CreateFixture(shape, 10.0f);

        bd.position.Set(0.0f, 6.0f);
        body_ = m_world.CreateBody(&bd);
        shape.m_radius = 5.0f;
        body_.CreateFixture(shape, 10.0f);
    }

    static Test Create()
    {
        return new typeof(this);
    }
}
