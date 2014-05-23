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
module tests.theojansen;

import core.stdc.float_;
import core.stdc.math;

import std.string;
import std.typecons;

import deimos.glfw.glfw3;

import dbox;

import framework.debug_draw;
import framework.test;

class TheoJansen : Test
{
    this()
    {
        m_offset.Set(0.0f, 8.0f);
        m_motorSpeed = 2.0f;
        m_motorOn    = true;
        b2Vec2 pivot = b2Vec2(0.0f, 0.8f);

        // Ground
        {
            b2BodyDef bd;
            b2Body* ground = m_world.CreateBody(&bd);

            auto shape = new b2EdgeShape();
            shape.Set(b2Vec2(-50.0f, 0.0f), b2Vec2(50.0f, 0.0f));
            ground.CreateFixture(shape, 0.0f);

            shape.Set(b2Vec2(-50.0f, 0.0f), b2Vec2(-50.0f, 10.0f));
            ground.CreateFixture(shape, 0.0f);

            shape.Set(b2Vec2(50.0f, 0.0f), b2Vec2(50.0f, 10.0f));
            ground.CreateFixture(shape, 0.0f);
        }

        // Balls
        for (int32 i = 0; i < 40; ++i)
        {
            b2CircleShape shape = new b2CircleShape();
            shape.m_radius = 0.25f;

            b2BodyDef bd;
            bd.type = b2_dynamicBody;
            bd.position.Set(-40.0f + 2.0f * i, 0.5f);

            b2Body* body_ = m_world.CreateBody(&bd);
            body_.CreateFixture(shape, 1.0f);
        }

        // Chassis
        {
            auto shape = new b2PolygonShape();
            shape.SetAsBox(2.5f, 1.0f);

            b2FixtureDef sd;
            sd.density = 1.0f;
            sd.shape   = shape;
            sd.filter.groupIndex = -1;
            b2BodyDef bd;
            bd.type     = b2_dynamicBody;
            bd.position = pivot + m_offset;
            m_chassis   = m_world.CreateBody(&bd);
            m_chassis.CreateFixture(&sd);
        }

        {
            b2CircleShape shape = new b2CircleShape();
            shape.m_radius = 1.6f;

            b2FixtureDef sd;
            sd.density = 1.0f;
            sd.shape   = shape;
            sd.filter.groupIndex = -1;
            b2BodyDef bd;
            bd.type     = b2_dynamicBody;
            bd.position = pivot + m_offset;
            m_wheel     = m_world.CreateBody(&bd);
            m_wheel.CreateFixture(&sd);
        }

        {
            b2RevoluteJointDef jd = new b2RevoluteJointDef();
            jd.Initialize(m_wheel, m_chassis, pivot + m_offset);
            jd.collideConnected = false;
            jd.motorSpeed       = m_motorSpeed;
            jd.maxMotorTorque   = 400.0f;
            jd.enableMotor      = m_motorOn;
            m_motorJoint        = cast(b2RevoluteJoint)m_world.CreateJoint(jd);
        }

        b2Vec2 wheelAnchor;

        wheelAnchor = pivot + b2Vec2(0.0f, -0.8f);

        CreateLeg(-1.0f, wheelAnchor);
        CreateLeg(1.0f, wheelAnchor);

        m_wheel.SetTransform(m_wheel.GetPosition(), 120.0f * b2_pi / 180.0f);
        CreateLeg(-1.0f, wheelAnchor);
        CreateLeg(1.0f, wheelAnchor);

        m_wheel.SetTransform(m_wheel.GetPosition(), -120.0f * b2_pi / 180.0f);
        CreateLeg(-1.0f, wheelAnchor);
        CreateLeg(1.0f, wheelAnchor);
    }

    void CreateLeg(float32 s, b2Vec2 wheelAnchor)
    {
        b2Vec2 p1 = b2Vec2(5.4f * s, -6.1f);
        b2Vec2 p2 = b2Vec2(7.2f * s, -1.2f);
        b2Vec2 p3 = b2Vec2(4.3f * s, -1.9f);
        b2Vec2 p4 = b2Vec2(3.1f * s, 0.8f);
        b2Vec2 p5 = b2Vec2(6.0f * s, 1.5f);
        b2Vec2 p6 = b2Vec2(2.5f * s, 3.7f);

        b2FixtureDef fd1, fd2;
        fd1.filter.groupIndex = -1;
        fd2.filter.groupIndex = -1;
        fd1.density = 1.0f;
        fd2.density = 1.0f;

        b2PolygonShape poly1 = new b2PolygonShape();
        b2PolygonShape poly2 = new b2PolygonShape();

        if (s > 0.0f)
        {
            b2Vec2 vertices[3];

            vertices[0] = p1;
            vertices[1] = p2;
            vertices[2] = p3;
            poly1.Set(vertices);

            vertices[0] = b2Vec2_zero;
            vertices[1] = p5 - p4;
            vertices[2] = p6 - p4;
            poly2.Set(vertices);
        }
        else
        {
            b2Vec2 vertices[3];

            vertices[0] = p1;
            vertices[1] = p3;
            vertices[2] = p2;
            poly1.Set(vertices);

            vertices[0] = b2Vec2_zero;
            vertices[1] = p6 - p4;
            vertices[2] = p5 - p4;
            poly2.Set(vertices);
        }

        fd1.shape = poly1;
        fd2.shape = poly2;

        b2BodyDef bd1, bd2;
        bd1.type     = b2_dynamicBody;
        bd2.type     = b2_dynamicBody;
        bd1.position = m_offset;
        bd2.position = p4 + m_offset;

        bd1.angularDamping = 10.0f;
        bd2.angularDamping = 10.0f;

        b2Body* body1 = m_world.CreateBody(&bd1);
        b2Body* body2 = m_world.CreateBody(&bd2);

        body1.CreateFixture(&fd1);
        body2.CreateFixture(&fd2);

        b2DistanceJointDef djd = new b2DistanceJointDef();

        // Using a soft distance constraint can reduce some jitter.
        // It also makes the structure seem a bit more fluid by
        // acting like a suspension system.
        djd.dampingRatio = 0.5f;
        djd.frequencyHz  = 10.0f;

        djd.Initialize(body1, body2, p2 + m_offset, p5 + m_offset);
        m_world.CreateJoint(djd);

        djd.Initialize(body1, body2, p3 + m_offset, p4 + m_offset);
        m_world.CreateJoint(djd);

        djd.Initialize(body1, m_wheel, p3 + m_offset, wheelAnchor + m_offset);
        m_world.CreateJoint(djd);

        djd.Initialize(body2, m_wheel, p6 + m_offset, wheelAnchor + m_offset);
        m_world.CreateJoint(djd);

        b2RevoluteJointDef rjd = new b2RevoluteJointDef();

        rjd.Initialize(body2, m_chassis, p4 + m_offset);
        m_world.CreateJoint(rjd);
    }

    override void Step(Settings* settings)
    {
        super.Step(settings);

        g_debugDraw.DrawString(5, m_textLine, "Keys: left = a, brake = s, right = d, toggle motor = m");
        m_textLine += DRAW_STRING_NEW_LINE;
    }

    override void Keyboard(int key)
    {
        switch (key)
        {
            case GLFW_KEY_A:
                m_motorJoint.SetMotorSpeed(-m_motorSpeed);
                break;

            case GLFW_KEY_S:
                m_motorJoint.SetMotorSpeed(0.0f);
                break;

            case GLFW_KEY_D:
                m_motorJoint.SetMotorSpeed(m_motorSpeed);
                break;

            case GLFW_KEY_M:
                m_motorJoint.EnableMotor(!m_motorJoint.IsMotorEnabled());
                break;

            default:
                break;
        }
    }

    static Test Create()
    {
        return new typeof(this);
    }

    b2Vec2  m_offset;
    b2Body* m_chassis;
    b2Body* m_wheel;
    b2RevoluteJoint m_motorJoint;
    bool m_motorOn;
    float32 m_motorSpeed;
}
