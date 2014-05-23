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
module tests.slidercrank;

import core.stdc.float_;
import core.stdc.math;

import std.string;
import std.typecons;

import deimos.glfw.glfw3;

import dbox;

import framework.debug_draw;
import framework.test;

// This is used to test sensor shapes.
class SliderCrank : Test
{
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
            b2Body* prevBody = ground;

            // Define crank.
            {
                auto shape = new b2PolygonShape();
                shape.SetAsBox(0.5f, 2.0f);

                b2BodyDef bd;
                bd.type = b2_dynamicBody;
                bd.position.Set(0.0f, 7.0f);
                b2Body* body_ = m_world.CreateBody(&bd);
                body_.CreateFixture(shape, 2.0f);

                b2RevoluteJointDef rjd = new b2RevoluteJointDef();
                rjd.Initialize(prevBody, body_, b2Vec2(0.0f, 5.0f));
                rjd.motorSpeed     = 1.0f * b2_pi;
                rjd.maxMotorTorque = 10000.0f;
                rjd.enableMotor    = true;
                m_joint1 = cast(b2RevoluteJoint)m_world.CreateJoint(rjd);

                prevBody = body_;
            }

            // Define follower.
            {
                auto shape = new b2PolygonShape();
                shape.SetAsBox(0.5f, 4.0f);

                b2BodyDef bd;
                bd.type = b2_dynamicBody;
                bd.position.Set(0.0f, 13.0f);
                b2Body* body_ = m_world.CreateBody(&bd);
                body_.CreateFixture(shape, 2.0f);

                b2RevoluteJointDef rjd = new b2RevoluteJointDef();
                rjd.Initialize(prevBody, body_, b2Vec2(0.0f, 9.0f));
                rjd.enableMotor = false;
                m_world.CreateJoint(rjd);

                prevBody = body_;
            }

            // Define piston
            {
                auto shape = new b2PolygonShape();
                shape.SetAsBox(1.5f, 1.5f);

                b2BodyDef bd;
                bd.type = b2_dynamicBody;
                bd.fixedRotation = true;
                bd.position.Set(0.0f, 17.0f);
                b2Body* body_ = m_world.CreateBody(&bd);
                body_.CreateFixture(shape, 2.0f);

                b2RevoluteJointDef rjd = new b2RevoluteJointDef();
                rjd.Initialize(prevBody, body_, b2Vec2(0.0f, 17.0f));
                m_world.CreateJoint(rjd);

                b2PrismaticJointDef pjd = new b2PrismaticJointDef();
                pjd.Initialize(ground, body_, b2Vec2(0.0f, 17.0f), b2Vec2(0.0f, 1.0f));

                pjd.maxMotorForce = 1000.0f;
                pjd.enableMotor   = true;

                m_joint2 = cast(b2PrismaticJoint)m_world.CreateJoint(pjd);
            }

            // Create a payload
            {
                auto shape = new b2PolygonShape();
                shape.SetAsBox(1.5f, 1.5f);

                b2BodyDef bd;
                bd.type = b2_dynamicBody;
                bd.position.Set(0.0f, 23.0f);
                b2Body* body_ = m_world.CreateBody(&bd);
                body_.CreateFixture(shape, 2.0f);
            }
        }
    }

    override void Keyboard(int key)
    {
        switch (key)
        {
            case GLFW_KEY_F:
                m_joint2.EnableMotor(!m_joint2.IsMotorEnabled());
                m_joint2.GetBodyB().SetAwake(true);
                break;

            case GLFW_KEY_M:
                m_joint1.EnableMotor(!m_joint1.IsMotorEnabled());
                m_joint1.GetBodyB().SetAwake(true);
                break;

            default:
                break;
        }
    }

    override void Step(Settings* settings)
    {
        Test.Step(settings);
        g_debugDraw.DrawString(5, m_textLine, "Keys: (f) toggle friction, (m) toggle motor");
        m_textLine += DRAW_STRING_NEW_LINE;
        float32 torque = m_joint1.GetMotorTorque(settings.hz);
        g_debugDraw.DrawString(5, m_textLine, format("Motor Torque = %5.0f", cast(float)torque));
        m_textLine += DRAW_STRING_NEW_LINE;
    }

    static Test Create()
    {
        return new typeof(this);
    }

    b2RevoluteJoint m_joint1;
    b2PrismaticJoint m_joint2;
}
