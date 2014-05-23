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
module tests.bodytypes;

import core.stdc.math;

import std.string;
import std.typecons;

import deimos.glfw.glfw3;

import dbox;

import framework.debug_draw;
import framework.test;

class BodyTypes : Test
{
    this()
    {
        b2Body* ground = null;
        {
            b2BodyDef bd;
            ground = m_world.CreateBody(&bd);

            auto shape = new b2EdgeShape();
            shape.Set(b2Vec2(-20.0f, 0.0f), b2Vec2(20.0f, 0.0f));

            b2FixtureDef fd;
            fd.shape = shape;

            ground.CreateFixture(&fd);
        }

        // Define attachment
        {
            b2BodyDef bd;
            bd.type = b2_dynamicBody;
            bd.position.Set(0.0f, 3.0f);
            m_attachment = m_world.CreateBody(&bd);

            auto shape = new b2PolygonShape();
            shape.SetAsBox(0.5f, 2.0f);
            m_attachment.CreateFixture(shape, 2.0f);
        }

        // Define platform
        {
            b2BodyDef bd;
            bd.type = b2_dynamicBody;
            bd.position.Set(-4.0f, 5.0f);
            m_platform = m_world.CreateBody(&bd);

            auto shape = new b2PolygonShape();
            shape.SetAsBox(0.5f, 4.0f, b2Vec2(4.0f, 0.0f), 0.5f * b2_pi);

            b2FixtureDef fd;
            fd.shape    = shape;
            fd.friction = 0.6f;
            fd.density  = 2.0f;
            m_platform.CreateFixture(&fd);

            b2RevoluteJointDef rjd = new b2RevoluteJointDef();
            rjd.Initialize(m_attachment, m_platform, b2Vec2(0.0f, 5.0f));
            rjd.maxMotorTorque = 50.0f;
            rjd.enableMotor    = true;
            m_world.CreateJoint(rjd);

            b2PrismaticJointDef pjd = new b2PrismaticJointDef();
            pjd.Initialize(ground, m_platform, b2Vec2(0.0f, 5.0f), b2Vec2(1.0f, 0.0f));

            pjd.maxMotorForce    = 1000.0f;
            pjd.enableMotor      = true;
            pjd.lowerTranslation = -10.0f;
            pjd.upperTranslation = 10.0f;
            pjd.enableLimit      = true;

            m_world.CreateJoint(pjd);

            m_speed = 3.0f;
        }

        // Create a payload
        {
            b2BodyDef bd;
            bd.type = b2_dynamicBody;
            bd.position.Set(0.0f, 8.0f);
            b2Body* body_ = m_world.CreateBody(&bd);

            auto shape = new b2PolygonShape();
            shape.SetAsBox(0.75f, 0.75f);

            b2FixtureDef fd;
            fd.shape    = shape;
            fd.friction = 0.6f;
            fd.density  = 2.0f;

            body_.CreateFixture(&fd);
        }
    }

    override void Keyboard(int key)
    {
        switch (key)
        {
            case GLFW_KEY_D:
                m_platform.SetType(b2_dynamicBody);
                break;

            case GLFW_KEY_S:
                m_platform.SetType(b2_staticBody);
                break;

            case GLFW_KEY_K:
                m_platform.SetType(b2_kinematicBody);
                m_platform.SetLinearVelocity(b2Vec2(-m_speed, 0.0f));
                m_platform.SetAngularVelocity(0.0f);
                break;

            default:
                break;
        }
    }

    override void Step(Settings* settings)
    {
        super.Step(settings);

        // Drive the kinematic body_.
        if (m_platform.GetType() == b2_kinematicBody)
        {
            b2Vec2 p = m_platform.GetTransform().p;
            b2Vec2 v = m_platform.GetLinearVelocity();

            if ((p.x < -10.0f && v.x < 0.0f) ||
                (p.x > 10.0f && v.x > 0.0f))
            {
                v.x = -v.x;
                m_platform.SetLinearVelocity(v);
            }
        }

        g_debugDraw.DrawString(5, m_textLine, "Keys: (d) dynamic, (s) static, (k) kinematic");
        m_textLine += DRAW_STRING_NEW_LINE;
    }

    static Test Create()
    {
        return new typeof(this);
    }

    b2Body* m_attachment;
    b2Body* m_platform;
    float32 m_speed = 0;
}
