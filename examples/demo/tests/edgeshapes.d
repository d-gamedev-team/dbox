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
module tests.edgeshapes;

import core.stdc.math;
import core.stdc.stdlib;

import std.algorithm;
import std.string;
import std.typecons;

import deimos.glfw.glfw3;

import dbox;

import framework.debug_draw;
import framework.test;

class EdgeShapesCallback : public b2RayCastCallback
{
public:
    EdgeShapesCallback()
    {
        m_fixture = NULL;
    }

    float32 ReportFixture(b2Fixture* fixture, const b2Vec2& point,
                          const b2Vec2& normal, float32 fraction)
    {
        m_fixture = fixture;
        m_point   = point;
        m_normal  = normal;

        return fraction;
    }

    b2Fixture* m_fixture;
    b2Vec2 m_point;
    b2Vec2 m_normal;
};

class EdgeShapes : Test
{
    enum
    {
        e_maxBodies = 256
    }


    this()
    {
        // Ground body_
        {
            b2BodyDef bd;
            b2Body* ground = m_world.CreateBody(&bd);

            float32 x1 = -20.0f;
            float32 y1 = 2.0f * cosf(x1 / 10.0f * b2_pi);

            for (int32 i = 0; i < 80; ++i)
            {
                float32 x2 = x1 + 0.5f;
                float32 y2 = 2.0f * cosf(x2 / 10.0f * b2_pi);

                auto shape = new b2EdgeShape();
                shape.Set(b2Vec2(x1, y1), b2Vec2(x2, y2));
                ground.CreateFixture(shape, 0.0f);

                x1 = x2;
                y1 = y2;
            }
        }

        {
            b2Vec2 vertices[3];
            vertices[0].Set(-0.5f, 0.0f);
            vertices[1].Set(0.5f, 0.0f);
            vertices[2].Set(0.0f, 1.5f);
            m_polygons[0].Set(vertices, 3);
        }

        {
            b2Vec2 vertices[3];
            vertices[0].Set(-0.1f, 0.0f);
            vertices[1].Set(0.1f, 0.0f);
            vertices[2].Set(0.0f, 1.5f);
            m_polygons[1].Set(vertices, 3);
        }

        {
            float32 w = 1.0f;
            float32 b = w / (2.0f + b2Sqrt(2.0f));
            float32 s = b2Sqrt(2.0f) * b;

            b2Vec2 vertices[8];
            vertices[0].Set(0.5f * s, 0.0f);
            vertices[1].Set(0.5f * w, b);
            vertices[2].Set(0.5f * w, b + s);
            vertices[3].Set(0.5f * s, w);
            vertices[4].Set(-0.5f * s, w);
            vertices[5].Set(-0.5f * w, b + s);
            vertices[6].Set(-0.5f * w, b);
            vertices[7].Set(-0.5f * s, 0.0f);

            m_polygons[2].Set(vertices, 8);
        }

        {
            m_polygons[3].SetAsBox(0.5f, 0.5f);
        }

        {
            m_circle.m_radius = 0.5f;
        }

        m_bodyIndex = 0;
        memset(m_bodies, 0, sizeof(m_bodies));

        m_angle = 0.0f;
    }

    static Test Create()
    {
        return new typeof(this);
    }

    override void Step(Settings* settings)
    {
        B2_NOT_USED(settings);

        m_rayActor = null;

        for (int32 i = 0; i < e_actorCount; ++i)
        {
            m_actors[i].fraction = 1.0f;
            m_actors[i].overlap  = false;
        }

        if (m_automated == true)
        {
            int32 actionCount = b2Max(1, e_actorCount >> 2);

            for (int32 i = 0; i < actionCount; ++i)
            {
                Action();
            }
        }

        Query();
        RayCast();

        for (int32 i = 0; i < e_actorCount; ++i)
        {
            Actor* actor = m_actors.ptr + i;

            if (actor.proxyId == b2_nullNode)
                continue;

            b2Color c = b2Color(0.9f, 0.9f, 0.9f);

            if (actor == m_rayActor && actor.overlap)
            {
                c.Set(0.9f, 0.6f, 0.6f);
            }
            else if (actor == m_rayActor)
            {
                c.Set(0.6f, 0.9f, 0.6f);
            }
            else if (actor.overlap)
            {
                c.Set(0.6f, 0.6f, 0.9f);
            }

            g_debugDraw.DrawAABB(&actor.aabb, c);
        }

        b2Color c = b2Color(0.7f, 0.7f, 0.7f);
        g_debugDraw.DrawAABB(&m_queryAABB, c);

        g_debugDraw.DrawSegment(m_rayCastInput.p1, m_rayCastInput.p2, c);

        b2Color c1 = b2Color(0.2f, 0.9f, 0.2f);
        b2Color c2 = b2Color(0.9f, 0.2f, 0.2f);
        g_debugDraw.DrawPoint(m_rayCastInput.p1, 6.0f, c1);
        g_debugDraw.DrawPoint(m_rayCastInput.p2, 6.0f, c2);

        if (m_rayActor)
        {
            b2Color cr = b2Color(0.2f, 0.2f, 0.9f);
            b2Vec2  p  = m_rayCastInput.p1 + m_rayActor.fraction * (m_rayCastInput.p2 - m_rayCastInput.p1);
            g_debugDraw.DrawPoint(p, 6.0f, cr);
        }

        {
            int32 height = m_tree.GetHeight();
            g_debugDraw.DrawString(5, m_textLine, format("dynamic tree height = %d", height));
            m_textLine += DRAW_STRING_NEW_LINE;
        }

        ++m_stepCount;
    }

    override void Keyboard(int key)
    {
        switch (key)
        {
            case GLFW_KEY_A:
                m_automated = !m_automated;
                break;

            case GLFW_KEY_C:
                CreateProxy();
                break;

            case GLFW_KEY_D:
                DestroyProxy();
                break;

            case GLFW_KEY_M:
                MoveProxy();
                break;

            default:
                break;
        }
    }

    bool QueryCallback(int32 proxyId)
    {
        Actor* actor = cast(Actor*)m_tree.GetUserData(proxyId);
        actor.overlap = b2TestOverlap(m_queryAABB, actor.aabb);
        return true;
    }

    float32 RayCastCallback(b2RayCastInput input, int32 proxyId)
    {
        Actor* actor = cast(Actor*)m_tree.GetUserData(proxyId);

        b2RayCastOutput output;
        bool hit = actor.aabb.RayCast(&output, input);

        if (hit)
        {
            m_rayCastOutput     = output;
            m_rayActor          = actor;
            m_rayActor.fraction = output.fraction;
            return output.fraction;
        }

        return input.maxFraction;
    }

private:

    struct Actor
    {
        b2AABB aabb;
        float32 fraction;
        bool overlap;
        int32 proxyId;
    };

    void GetRandomAABB(b2AABB* aabb)
    {
        b2Vec2 w;
        w.Set(2.0f * m_proxyExtent, 2.0f * m_proxyExtent);

        // aabb.lowerBound.x = -m_proxyExtent;
        // aabb.lowerBound.y = -m_proxyExtent + m_worldExtent;
        aabb.lowerBound.x = RandomFloat(-m_worldExtent, m_worldExtent);
        aabb.lowerBound.y = RandomFloat(0.0f, 2.0f * m_worldExtent);
        aabb.upperBound   = aabb.lowerBound + w;
    }

    void MoveAABB(b2AABB* aabb)
    {
        b2Vec2 d;
        d.x = RandomFloat(-0.5f, 0.5f);
        d.y = RandomFloat(-0.5f, 0.5f);

        // d.x = 2.0f;
        // d.y = 0.0f;
        aabb.lowerBound += d;
        aabb.upperBound += d;

        b2Vec2 c0 = 0.5f * (aabb.lowerBound + aabb.upperBound);
        b2Vec2 min;
        min.Set(-m_worldExtent, 0.0f);
        b2Vec2 max;
        max.Set(m_worldExtent, 2.0f * m_worldExtent);
        b2Vec2 c = b2Clamp(c0, min, max);

        aabb.lowerBound += c - c0;
        aabb.upperBound += c - c0;
    }

    void CreateProxy()
    {
        for (int32 i = 0; i < e_actorCount; ++i)
        {
            int32  j     = rand() % e_actorCount;
            Actor* actor = m_actors.ptr + j;

            if (actor.proxyId == b2_nullNode)
            {
                GetRandomAABB(&actor.aabb);
                actor.proxyId = m_tree.CreateProxy(actor.aabb, actor);
                return;
            }
        }
    }

    void DestroyProxy()
    {
        for (int32 i = 0; i < e_actorCount; ++i)
        {
            int32  j     = rand() % e_actorCount;
            Actor* actor = m_actors.ptr + j;

            if (actor.proxyId != b2_nullNode)
            {
                m_tree.DestroyProxy(actor.proxyId);
                actor.proxyId = b2_nullNode;
                return;
            }
        }
    }

    void MoveProxy()
    {
        for (int32 i = 0; i < e_actorCount; ++i)
        {
            int32  j     = rand() % e_actorCount;
            Actor* actor = m_actors.ptr + j;

            if (actor.proxyId == b2_nullNode)
            {
                continue;
            }

            b2AABB aabb0 = actor.aabb;
            MoveAABB(&actor.aabb);
            b2Vec2 displacement = actor.aabb.GetCenter() - aabb0.GetCenter();
            m_tree.MoveProxy(actor.proxyId, actor.aabb, displacement);
            return;
        }
    }

    void Action()
    {
        int32 choice = rand() % 20;

        switch (choice)
        {
            case 0:
                CreateProxy();
                break;

            case 1:
                DestroyProxy();
                break;

            default:
                MoveProxy();
        }
    }

    void Query()
    {
        m_tree.Query(this, m_queryAABB);

        for (int32 i = 0; i < e_actorCount; ++i)
        {
            if (m_actors[i].proxyId == b2_nullNode)
            {
                continue;
            }

            bool overlap = b2TestOverlap(m_queryAABB, m_actors[i].aabb);
            B2_NOT_USED(overlap);
            assert(overlap == m_actors[i].overlap);
        }
    }

    void RayCast()
    {
        m_rayActor = null;

        b2RayCastInput input = m_rayCastInput;

        // Ray cast against the dynamic tree.
        m_tree.RayCast(this, input);

        // Brute force ray cast.
        Actor* bruteActor = null;
        b2RayCastOutput bruteOutput;

        for (int32 i = 0; i < e_actorCount; ++i)
        {
            if (m_actors[i].proxyId == b2_nullNode)
            {
                continue;
            }

            b2RayCastOutput output;
            bool hit = m_actors[i].aabb.RayCast(&output, input);

            if (hit)
            {
                bruteActor        = m_actors.ptr + i;
                bruteOutput       = output;
                input.maxFraction = output.fraction;
            }
        }

        if (bruteActor !is null)
        {
            assert(bruteOutput.fraction == m_rayCastOutput.fraction);
        }
    }

    float32 m_worldExtent = 0;
    float32 m_proxyExtent = 0;

    b2DynamicTree m_tree;
    b2AABB m_queryAABB;
    b2RayCastInput  m_rayCastInput;
    b2RayCastOutput m_rayCastOutput;
    Actor* m_rayActor;
    Actor  m_actors[e_actorCount];
    int32  m_stepCount;
    bool m_automated;
}
