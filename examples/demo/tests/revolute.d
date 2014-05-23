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
module tests.revolute;

import core.stdc.math;

import std.string;
import std.typecons;

import deimos.glfw.glfw3;

import dbox;

import framework.debug_draw;
import framework.test;

class Revolute : Test
{
    this()
    {
        b2Body* ground = null;
        {
            b2BodyDef bd;
            ground = m_world.CreateBody(&bd);

            auto shape = new b2EdgeShape();
            shape.Set(b2Vec2(-40.0f, 0.0f), b2Vec2(40.0f, 0.0f));

            b2FixtureDef fd;
            fd.shape = shape;

            // fd.filter.categoryBits = 2;

            ground.CreateFixture(&fd);
        }

        {
            b2CircleShape shape = new b2CircleShape();
            shape.m_radius = 0.5f;

            b2BodyDef bd;
            bd.type = b2_dynamicBody;

            b2RevoluteJointDef rjd = new b2RevoluteJointDef();

            bd.position.Set(-10.0f, 20.0f);
            b2Body* body_ = m_world.CreateBody(&bd);
            body_.CreateFixture(shape, 5.0f);

            float32 w = 100.0f;
            body_.SetAngularVelocity(w);
            body_.SetLinearVelocity(b2Vec2(-8.0f * w, 0.0f));

            rjd.Initialize(ground, body_, b2Vec2(-10.0f, 12.0f));
            rjd.motorSpeed       = 1.0f * b2_pi;
            rjd.maxMotorTorque   = 10000.0f;
            rjd.enableMotor      = false;
            rjd.lowerAngle       = -0.25f * b2_pi;
            rjd.upperAngle       = 0.5f * b2_pi;
            rjd.enableLimit      = true;
            rjd.collideConnected = true;

            m_joint = cast(b2RevoluteJoint)m_world.CreateJoint(rjd);
        }

        {
            b2CircleShape circle_shape = new b2CircleShape();
            circle_shape.m_radius = 3.0f;

            b2BodyDef circle_bd;
            circle_bd.type = b2_dynamicBody;
            circle_bd.position.Set(5.0f, 30.0f);

            b2FixtureDef fd;
            fd.density         = 5.0f;
            fd.filter.maskBits = 1;
            fd.shape = circle_shape;

            m_ball = m_world.CreateBody(&circle_bd);
            m_ball.CreateFixture(&fd);

            b2PolygonShape polygon_shape = new b2PolygonShape();
            polygon_shape.SetAsBox(10.0f, 0.2f, b2Vec2(-10.0f, 0.0f), 0.0f);

            b2BodyDef polygon_bd;
            polygon_bd.position.Set(20.0f, 10.0f);
            polygon_bd.type   = b2_dynamicBody;
            polygon_bd.bullet = true;
            b2Body* polygon_body = m_world.CreateBody(&polygon_bd);
            polygon_body.CreateFixture(polygon_shape, 2.0f);

            b2RevoluteJointDef rjd = new b2RevoluteJointDef();
            rjd.Initialize(ground, polygon_body, b2Vec2(20.0f, 10.0f));
            rjd.lowerAngle  = -0.25f * b2_pi;
            rjd.upperAngle  = 0.0f * b2_pi;
            rjd.enableLimit = true;
            m_world.CreateJoint(rjd);
        }

        // Tests mass computation of a small object far from the origin
        {
            b2BodyDef bodyDef;
            bodyDef.type = b2_dynamicBody;
            b2Body* body_ = m_world.CreateBody(&bodyDef);

            b2PolygonShape polyShape = new b2PolygonShape();
            b2Vec2 verts[3];
            verts[0].Set(17.63f, 36.31f);
            verts[1].Set(17.52f, 36.69f);
            verts[2].Set(17.19f, 36.36f);
            polyShape.Set(verts);

            b2FixtureDef polyFixtureDef;
            polyFixtureDef.shape  = polyShape;
            polyFixtureDef.density = 1;

            body_.CreateFixture(&polyFixtureDef);               // assertion hits inside here
        }
    }

    override void Keyboard(int key)
    {
        switch (key)
        {
            case GLFW_KEY_L:
                m_joint.EnableLimit(!m_joint.IsLimitEnabled());
                break;

            case GLFW_KEY_M:
                m_joint.EnableMotor(!m_joint.IsMotorEnabled());
                break;

            default:
        }
    }

    override void Step(Settings* settings)
    {
        super.Step(settings);
        g_debugDraw.DrawString(5, m_textLine, "Keys: (l) limits, (m) motor");
        m_textLine += DRAW_STRING_NEW_LINE;

        // if (m_stepCount == 360)
        // {
        // m_ball.SetTransform(b2Vec2(0.0f, 0.5f), 0.0f);
        // }

        // float32 torque1 = m_joint1.GetMotorTorque();
        // g_debugDraw.DrawString(5, m_textLine, "Motor Torque = %4.0f, %4.0f : Motor Force = %4.0f", (float) torque1, (float) torque2, (float) force3);
        // m_textLine += DRAW_STRING_NEW_LINE;
    }

    static Test Create()
    {
        return new typeof(this);
    }

    b2Body* m_ball;
    b2RevoluteJoint m_joint;
}
