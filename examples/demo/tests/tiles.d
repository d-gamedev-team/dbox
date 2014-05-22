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
module tests.tiles;

import core.stdc.math;

import std.string;
import std.typecons;

import dbox;

import framework.debug_draw;
import framework.test;

/// This stress tests the dynamic tree broad-phase. This also shows that tile
/// based collision is _not_ smooth due to Box2D not knowing about adjacency.
class Tiles : Test
{
    enum
    {
        e_count = 20
    };

    this()
    {
        m_fixtureCount = 0;
        auto timer = b2Timer();

        {
            float32 a = 0.5f;
            b2BodyDef bd;
            bd.position.y = -a;
            b2Body* ground = m_world.CreateBody(&bd);

            int32  N = 200;
            int32  M = 10;
            b2Vec2 position;
            position.y = 0.0f;

            for (int32 j = 0; j < M; ++j)
            {
                position.x = -N * a;

                for (int32 i = 0; i < N; ++i)
                {
                    auto shape = new b2PolygonShape();
                    shape.SetAsBox(a, a, position, 0.0f);
                    ground.CreateFixture(shape, 0.0f);
                    ++m_fixtureCount;
                    position.x += 2.0f * a;
                }

                position.y -= 2.0f * a;
            }
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

                    // if (i == 0 && j == 0)
                    // {
                    // bd.allowSleep = false;
                    // }
                    // else
                    // {
                    // bd.allowSleep = true;
                    // }

                    b2Body* body_ = m_world.CreateBody(&bd);
                    body_.CreateFixture(shape, 5.0f);
                    ++m_fixtureCount;
                    y += deltaY;
                }

                x += deltaX;
            }
        }

        m_createTime = timer.GetMilliseconds();
    }

    override void Step(Settings* settings)
    {
        auto cm = m_world.GetContactManager();
        int32 height           = cm.m_broadPhase.GetTreeHeight();
        int32 leafCount        = cm.m_broadPhase.GetProxyCount();
        int32 minimumNodeCount = 2 * leafCount - 1;
        float32 minimumHeight  = ceilf(logf(float32(minimumNodeCount)) / logf(2.0f));
        g_debugDraw.DrawString(5, m_textLine, format("dynamic tree height = %d, min = %d", height, cast(int32)minimumHeight));
        m_textLine += DRAW_STRING_NEW_LINE;

        Test.Step(settings);

        g_debugDraw.DrawString(5, m_textLine, format("create time = %6.2f ms, fixture count = %d",
                               m_createTime, m_fixtureCount));
        m_textLine += DRAW_STRING_NEW_LINE;

        // b2DynamicTree* tree = &m_world.m_contactManager.m_broadPhase.m_tree;

        // if (m_stepCount == 400)
        // {
        // tree.RebuildBottomUp();
        // }
    }

    static Test Create()
    {
        return new typeof(this);
    }

    int32 m_fixtureCount;
    float32 m_createTime = 0;
}
