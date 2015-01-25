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
module framework.test;

import core.stdc.stdlib;
import core.stdc.string;

import std.exception;
import std.stdio;
import std.string;

import deimos.glfw.glfw3;

import glad.gl.enums;
import glad.gl.ext;
import glad.gl.funcs;
import glad.gl.loader;
import glad.gl.types;

import glwtf.input;
import glwtf.window;

import dbox;

import imgui;

import framework.debug_draw;

alias TestCreateFcn = Test function();

enum RAND_LIMIT = 32767;
enum DRAW_STRING_NEW_LINE = 16;

/// Random number in range [-1,1]
float32 RandomFloat()
{
    float32 r = cast(float32)(rand() & (RAND_LIMIT));
    r /= RAND_LIMIT;
    r  = 2.0f * r - 1.0f;
    return r;
}

/// Random floating point number in range [lo, hi]
float32 RandomFloat(float32 lo, float32 hi)
{
    float32 r = cast(float32)(rand() & (RAND_LIMIT));
    r /= RAND_LIMIT;
    r  = (hi - lo) * r + lo;
    return r;
}

/// Test settings. Some can be controlled in the GUI.
struct Settings
{
    float32 hz = 60;
    float velocityIterations = 8;
    float positionIterations = 3;
    bool drawShapes = true;
    bool drawJoints = true;
    bool drawAABBs;
    bool drawContactPoints;
    bool drawContactNormals;
    bool drawContactImpulse;
    bool drawFrictionImpulse;
    bool drawCOMs;
    bool drawStats = true;
    bool drawProfile;
    bool enableWarmStarting = true;
    bool enableContinuous = true;
    bool enableSubStepping;
    bool enableSleep = true;
    bool enableVSync;
    bool pause;
    bool singleStep;
}

struct TestEntry
{
    string name;
    TestCreateFcn createFcn;
}

// This is called when a joint in the world is implicitly destroyed
// because an attached body is destroyed. This gives us a chance to
// nullify the mouse joint.
class DestructionListener : b2DestructionListener
{
    override void SayGoodbye(b2Fixture* fixture)
    {
        B2_NOT_USED(fixture);
    }

    override void SayGoodbye(b2Joint joint)
    {
        if (test.m_mouseJoint is joint)
        {
            test.m_mouseJoint = null;
        }
        else
        {
            test.JointDestroyed(joint);
        }
    }

    Test test;
}

enum k_maxContactPoints = 2048;

struct ContactPoint
{
    b2Fixture* fixtureA;
    b2Fixture* fixtureB;
    b2Vec2 normal;
    b2Vec2 position;
    b2PointState state;
    float32 normalImpulse = 0;
    float32 tangentImpulse = 0;
    float32 separation = 0;
}

class Test : b2ContactListener
{
    this()
    {
        b2Vec2 gravity;
        gravity.Set(0.0f, -10.0f);
        m_world      = new b2World(gravity);
        m_bomb       = null;
        m_textLine   = 30;
        m_mouseJoint = null;
        m_pointCount = 0;

        m_destructionListener = new DestructionListener();

        m_destructionListener.test = this;
        m_world.SetDestructionListener(m_destructionListener);
        m_world.SetContactListener(this);
        m_world.SetDebugDraw(g_debugDraw);

        m_bombSpawning = false;

        m_stepCount = 0;

        b2BodyDef bodyDef;
        m_groundBody = m_world.CreateBody(&bodyDef);

        memset(&m_maxProfile, 0, b2Profile.sizeof);
        memset(&m_totalProfile, 0, b2Profile.sizeof);
    }

    // Callbacks for derived classes.
    override void BeginContact(b2Contact contact)
    {
        B2_NOT_USED(contact);
    }

    override void EndContact(b2Contact contact)
    {
        B2_NOT_USED(contact);
    }

    override void PreSolve(b2Contact contact, const(b2Manifold)* oldManifold)
    {
        const(b2Manifold)* manifold = contact.GetManifold();

        if (manifold.pointCount == 0)
        {
            return;
        }

        b2Fixture* fixtureA = contact.GetFixtureA();
        b2Fixture* fixtureB = contact.GetFixtureB();

        b2PointState[b2_maxManifoldPoints] state1, state2;
        b2GetPointStates(state1, state2, oldManifold, manifold);

        b2WorldManifold worldManifold;
        contact.GetWorldManifold(&worldManifold);

        for (int32 i = 0; i < manifold.pointCount && m_pointCount < k_maxContactPoints; ++i)
        {
            ContactPoint* cp = &m_points[m_pointCount];
            cp.fixtureA       = fixtureA;
            cp.fixtureB       = fixtureB;
            cp.position       = worldManifold.points[i];
            cp.normal         = worldManifold.normal;
            cp.state          = state2[i];
            cp.normalImpulse  = manifold.points[i].normalImpulse;
            cp.tangentImpulse = manifold.points[i].tangentImpulse;
            cp.separation     = worldManifold.separations[i];
            ++m_pointCount;
        }
    }

    override void PostSolve(b2Contact contact, const(b2ContactImpulse)* impulse)
    {
        B2_NOT_USED(contact);
        B2_NOT_USED(impulse);
    }

    void DrawTitle(string str)
    {
        g_debugDraw.DrawString(5, DRAW_STRING_NEW_LINE, str);
        m_textLine = 3 * DRAW_STRING_NEW_LINE;
    }

    void MouseDown(b2Vec2 p)
    {
        m_mouseWorld = p;

        if (m_mouseJoint !is null)
        {
            return;
        }

        // Make a small box.
        b2AABB aabb;
        b2Vec2 d;
        d.Set(0.001f, 0.001f);
        aabb.lowerBound = p - d;
        aabb.upperBound = p + d;

        // Query the world for overlapping shapes.
        QueryCallback callback = new QueryCallback(p);
        m_world.QueryAABB(callback, aabb);

        if (callback.m_fixture)
        {
            b2Body* body_ = callback.m_fixture.GetBody();
            b2MouseJointDef md = new b2MouseJointDef;
            md.bodyA     = m_groundBody;
            md.bodyB     = body_;
            md.target    = p;
            md.maxForce  = 1000.0f * body_.GetMass();
            m_mouseJoint = cast(b2MouseJoint)m_world.CreateJoint(md);
            body_.SetAwake(true);
        }
    }

    void SpawnBomb(b2Vec2 worldPt)
    {
        m_bombSpawnPoint = worldPt;
        m_bombSpawning   = true;
    }

    void CompleteBombSpawn(b2Vec2 p)
    {
        if (m_bombSpawning == false)
        {
            return;
        }

        const float multiplier = 30.0f;
        b2Vec2 vel = m_bombSpawnPoint - p;
        vel *= multiplier;
        LaunchBomb(m_bombSpawnPoint, vel);
        m_bombSpawning = false;
    }

    void ShiftMouseDown(b2Vec2 p)
    {
        m_mouseWorld = p;

        if (m_mouseJoint !is null)
        {
            return;
        }

        SpawnBomb(p);
    }

    void MouseUp(b2Vec2 p)
    {
        if (m_mouseJoint)
        {
            m_world.DestroyJoint(m_mouseJoint);
            m_mouseJoint = null;
        }

        if (m_bombSpawning)
        {
            CompleteBombSpawn(p);
        }
    }

    void MouseMove(b2Vec2 p)
    {
        m_mouseWorld = p;

        if (m_mouseJoint)
        {
            m_mouseJoint.SetTarget(p);
        }
    }

    void LaunchBomb()
    {
        b2Vec2 p = b2Vec2(RandomFloat(-15.0f, 15.0f), 30.0f);
        b2Vec2 v = -5.0f * p;
        LaunchBomb(p, v);
    }

    void LaunchBomb(b2Vec2 position, b2Vec2 velocity)
    {
        if (m_bomb)
        {
            m_world.DestroyBody(m_bomb);
            m_bomb = null;
        }

        b2BodyDef bd;
        bd.type     = b2_dynamicBody;
        bd.position = position;
        bd.bullet   = true;
        m_bomb      = m_world.CreateBody(&bd);
        m_bomb.SetLinearVelocity(velocity);

        b2CircleShape circle = new b2CircleShape();
        circle.m_radius = 0.3f;

        b2FixtureDef fd;
        fd.shape       = circle;
        fd.density     = 20.0f;
        fd.restitution = 0.0f;

        b2Vec2 minV = position - b2Vec2(0.3f, 0.3f);
        b2Vec2 maxV = position + b2Vec2(0.3f, 0.3f);

        b2AABB aabb;
        aabb.lowerBound = minV;
        aabb.upperBound = maxV;

        m_bomb.CreateFixture(&fd);
    }

    void Draw(Settings* settings)
    {
        if (settings.pause)
        {
            g_debugDraw.DrawString(5, m_textLine, "****PAUSED****");
            m_textLine += DRAW_STRING_NEW_LINE;
        }

        uint32 flags = 0;
        flags += settings.drawShapes * b2Draw.e_shapeBit;
        flags += settings.drawJoints * b2Draw.e_jointBit;
        flags += settings.drawAABBs * b2Draw.e_aabbBit;
        flags += settings.drawCOMs * b2Draw.e_centerOfMassBit;
        g_debugDraw.SetFlags(flags);

        glfwSwapInterval(settings.enableVSync ? 1 : 0);

        m_pointCount = 0;

        m_world.DrawDebugData();
        g_debugDraw.Flush();

        if (settings.drawStats)
        {
            int32 bodyCount    = m_world.GetBodyCount();
            int32 contactCount = m_world.GetContactCount();
            int32 jointCount   = m_world.GetJointCount();
            g_debugDraw.DrawString(5, m_textLine, format("bodies/contacts/joints = %d/%d/%d", bodyCount, contactCount, jointCount));
            m_textLine += DRAW_STRING_NEW_LINE;

            int32 proxyCount = m_world.GetProxyCount();
            int32 height     = m_world.GetTreeHeight();
            int32 balance    = m_world.GetTreeBalance();
            float32 quality  = m_world.GetTreeQuality();
            g_debugDraw.DrawString(5, m_textLine, format("proxies/height/balance/quality = %d/%d/%d/%g", proxyCount, height, balance, quality));
            m_textLine += DRAW_STRING_NEW_LINE;
        }


        if (settings.drawProfile)
        {
            auto p = m_world.GetProfile();

            b2Profile aveProfile;
            memset(&aveProfile, 0, b2Profile.sizeof);

            if (m_stepCount > 0)
            {
                float32 scale = 1.0f / m_stepCount;
                aveProfile.step          = scale * m_totalProfile.step;
                aveProfile.collide       = scale * m_totalProfile.collide;
                aveProfile.solve         = scale * m_totalProfile.solve;
                aveProfile.solveInit     = scale * m_totalProfile.solveInit;
                aveProfile.solveVelocity = scale * m_totalProfile.solveVelocity;
                aveProfile.solvePosition = scale * m_totalProfile.solvePosition;
                aveProfile.solveTOI      = scale * m_totalProfile.solveTOI;
                aveProfile.broadphase    = scale * m_totalProfile.broadphase;
            }

            g_debugDraw.DrawString(5, m_textLine, format("step [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.step, aveProfile.step, m_maxProfile.step));
            m_textLine += DRAW_STRING_NEW_LINE;
            g_debugDraw.DrawString(5, m_textLine, format("collide [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.collide, aveProfile.collide, m_maxProfile.collide));
            m_textLine += DRAW_STRING_NEW_LINE;
            g_debugDraw.DrawString(5, m_textLine, format("solve [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solve, aveProfile.solve, m_maxProfile.solve));
            m_textLine += DRAW_STRING_NEW_LINE;
            g_debugDraw.DrawString(5, m_textLine, format("solve init [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solveInit, aveProfile.solveInit, m_maxProfile.solveInit));
            m_textLine += DRAW_STRING_NEW_LINE;
            g_debugDraw.DrawString(5, m_textLine, format("solve velocity [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solveVelocity, aveProfile.solveVelocity, m_maxProfile.solveVelocity));
            m_textLine += DRAW_STRING_NEW_LINE;
            g_debugDraw.DrawString(5, m_textLine, format("solve position [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solvePosition, aveProfile.solvePosition, m_maxProfile.solvePosition));
            m_textLine += DRAW_STRING_NEW_LINE;
            g_debugDraw.DrawString(5, m_textLine, format("solveTOI [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solveTOI, aveProfile.solveTOI, m_maxProfile.solveTOI));
            m_textLine += DRAW_STRING_NEW_LINE;
            g_debugDraw.DrawString(5, m_textLine, format("broad-phase [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.broadphase, aveProfile.broadphase, m_maxProfile.broadphase));
            m_textLine += DRAW_STRING_NEW_LINE;
        }

        if (m_mouseJoint)
        {
            b2Vec2 p1 = m_mouseJoint.GetAnchorB();
            b2Vec2 p2 = m_mouseJoint.GetTarget();

            b2Color c;
            c.Set(0.0f, 1.0f, 0.0f);
            g_debugDraw.DrawPoint(p1, 4.0f, c);
            g_debugDraw.DrawPoint(p2, 4.0f, c);

            c.Set(0.8f, 0.8f, 0.8f);
            g_debugDraw.DrawSegment(p1, p2, c);
        }

        if (m_bombSpawning)
        {
            b2Color c;
            c.Set(0.0f, 0.0f, 1.0f);
            g_debugDraw.DrawPoint(m_bombSpawnPoint, 4.0f, c);

            c.Set(0.8f, 0.8f, 0.8f);
            g_debugDraw.DrawSegment(m_mouseWorld, m_bombSpawnPoint, c);
        }

        if (settings.drawContactPoints)
        {
            const float32 k_impulseScale = 0.1f;
            const float32 k_axisScale    = 0.3f;

            for (int32 i = 0; i < m_pointCount; ++i)
            {
                ContactPoint* point = &m_points[i];

                if (point.state == b2_addState)
                {
                    // Add
                    g_debugDraw.DrawPoint(point.position, 10.0f, b2Color(0.3f, 0.95f, 0.3f));
                }
                else if (point.state == b2_persistState)
                {
                    // Persist
                    g_debugDraw.DrawPoint(point.position, 5.0f, b2Color(0.3f, 0.3f, 0.95f));
                }

                if (settings.drawContactNormals == 1)
                {
                    b2Vec2 p1 = point.position;
                    b2Vec2 p2 = p1 + k_axisScale * point.normal;
                    g_debugDraw.DrawSegment(p1, p2, b2Color(0.9f, 0.9f, 0.9f));
                }
                else if (settings.drawContactImpulse == 1)
                {
                    b2Vec2 p1 = point.position;
                    b2Vec2 p2 = p1 + k_impulseScale * point.normalImpulse * point.normal;
                    g_debugDraw.DrawSegment(p1, p2, b2Color(0.9f, 0.9f, 0.3f));
                }

                if (settings.drawFrictionImpulse == 1)
                {
                    b2Vec2 tangent = b2Cross(point.normal, 1.0f);
                    b2Vec2 p1      = point.position;
                    b2Vec2 p2      = p1 + k_impulseScale * point.tangentImpulse * tangent;
                    g_debugDraw.DrawSegment(p1, p2, b2Color(0.9f, 0.9f, 0.3f));
                }
            }
        }
    }

    void Step(Settings* settings)
    {
        float32 timeStep = settings.hz > 0.0f ? 1.0f / settings.hz : cast(float32)0.0f;

        if (settings.pause)
        {
            if (settings.singleStep)
            {
                settings.singleStep = 0;
            }
            else
            {
                timeStep = 0.0f;
            }

            //g_debugDraw.DrawString(5, m_textLine, "****PAUSED****");
            //m_textLine += DRAW_STRING_NEW_LINE;
        }

        //uint32 flags = 0;
        //flags += settings.drawShapes * b2Draw.e_shapeBit;
        //flags += settings.drawJoints * b2Draw.e_jointBit;
        //flags += settings.drawAABBs * b2Draw.e_aabbBit;
        //flags += settings.drawCOMs * b2Draw.e_centerOfMassBit;
        //g_debugDraw.SetFlags(flags);

        //glfwSwapInterval(settings.enableVSync ? 1 : 0);

        m_world.SetAllowSleeping(settings.enableSleep);
        m_world.SetWarmStarting(settings.enableWarmStarting);
        m_world.SetContinuousPhysics(settings.enableContinuous);
        m_world.SetSubStepping(settings.enableSubStepping);

        //m_pointCount = 0;

        m_world.Step(timeStep, cast(int)settings.velocityIterations, cast(int)settings.positionIterations);

        //m_world.DrawDebugData();
        //g_debugDraw.Flush();

        if (timeStep > 0.0f)
        {
            ++m_stepCount;
        }

        //if (settings.drawStats)
        //{
        //    int32 bodyCount    = m_world.GetBodyCount();
        //    int32 contactCount = m_world.GetContactCount();
        //    int32 jointCount   = m_world.GetJointCount();
        //    g_debugDraw.DrawString(5, m_textLine, format("bodies/contacts/joints = %d/%d/%d", bodyCount, contactCount, jointCount));
        //    m_textLine += DRAW_STRING_NEW_LINE;

        //    int32 proxyCount = m_world.GetProxyCount();
        //    int32 height     = m_world.GetTreeHeight();
        //    int32 balance    = m_world.GetTreeBalance();
        //    float32 quality  = m_world.GetTreeQuality();
        //    g_debugDraw.DrawString(5, m_textLine, format("proxies/height/balance/quality = %d/%d/%d/%g", proxyCount, height, balance, quality));
        //    m_textLine += DRAW_STRING_NEW_LINE;
        //}

        // Track maximum profile times
        {
            auto p = m_world.GetProfile();
            m_maxProfile.step          = b2Max(m_maxProfile.step, p.step);
            m_maxProfile.collide       = b2Max(m_maxProfile.collide, p.collide);
            m_maxProfile.solve         = b2Max(m_maxProfile.solve, p.solve);
            m_maxProfile.solveInit     = b2Max(m_maxProfile.solveInit, p.solveInit);
            m_maxProfile.solveVelocity = b2Max(m_maxProfile.solveVelocity, p.solveVelocity);
            m_maxProfile.solvePosition = b2Max(m_maxProfile.solvePosition, p.solvePosition);
            m_maxProfile.solveTOI      = b2Max(m_maxProfile.solveTOI, p.solveTOI);
            m_maxProfile.broadphase    = b2Max(m_maxProfile.broadphase, p.broadphase);

            m_totalProfile.step          += p.step;
            m_totalProfile.collide       += p.collide;
            m_totalProfile.solve         += p.solve;
            m_totalProfile.solveInit     += p.solveInit;
            m_totalProfile.solveVelocity += p.solveVelocity;
            m_totalProfile.solvePosition += p.solvePosition;
            m_totalProfile.solveTOI      += p.solveTOI;
            m_totalProfile.broadphase    += p.broadphase;
        }
    }

    void ShiftOrigin(b2Vec2 newOrigin)
    {
        m_world.ShiftOrigin(newOrigin);
    }

    void Keyboard(int key)
    {
        B2_NOT_USED(key);
    }

    void KeyboardUp(int key)
    {
        B2_NOT_USED(key);
    }

    void LaunchBomb(b2Vec2 position, b2Vec2 velocity);

    // Let derived tests know that a joint was destroyed.
    void JointDestroyed(b2Joint joint)
    {
        B2_NOT_USED(joint);
    }

protected:
    b2Body* m_groundBody;
    b2AABB  m_worldAABB;
    ContactPoint m_points[k_maxContactPoints];
    int32 m_pointCount;
    DestructionListener m_destructionListener;
    int32 m_textLine;
    b2World* m_world;
    b2Body * m_bomb;
    b2MouseJoint m_mouseJoint;
    b2Vec2 m_bombSpawnPoint;
    bool m_bombSpawning;
    b2Vec2 m_mouseWorld;
    int32  m_stepCount;

    b2Profile m_maxProfile;
    b2Profile m_totalProfile;
}

class QueryCallback : b2QueryCallback
{
public:
    this(b2Vec2 point)
    {
        m_point   = point;
        m_fixture = null;
    }

    override bool ReportFixture(b2Fixture* fixture)
    {
        b2Body* body_ = fixture.GetBody();

        if (body_.GetType() == b2_dynamicBody)
        {
            bool inside = fixture.TestPoint(m_point);

            if (inside)
            {
                m_fixture = fixture;

                // We are done, terminate the query.
                return false;
            }
        }

        // Continue the query.
        return true;
    }

    b2Vec2 m_point;
    b2Fixture* m_fixture;
}
