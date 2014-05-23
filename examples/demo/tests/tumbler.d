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
module tests.tumbler;

import core.stdc.float_;
import core.stdc.math;

import std.string;
import std.typecons;

import deimos.glfw.glfw3;

import dbox;

import framework.debug_draw;
import framework.test;

class Tumbler : Test
{
    enum
    {
        e_count = 800
    }

    this()
    {
        b2Body* ground = null;
        {
            b2BodyDef bd;
            ground = m_world.CreateBody(&bd);
        }

        {
            b2BodyDef bd;
            bd.type       = b2_dynamicBody;
            bd.allowSleep = false;
            bd.position.Set(0.0f, 10.0f);
            b2Body* body_ = m_world.CreateBody(&bd);

            auto shape = new b2PolygonShape();
            shape.SetAsBox(0.5f, 10.0f, b2Vec2(10.0f, 0.0f), 0.0);
            body_.CreateFixture(shape, 5.0f);
            shape.SetAsBox(0.5f, 10.0f, b2Vec2(-10.0f, 0.0f), 0.0);
            body_.CreateFixture(shape, 5.0f);
            shape.SetAsBox(10.0f, 0.5f, b2Vec2(0.0f, 10.0f), 0.0);
            body_.CreateFixture(shape, 5.0f);
            shape.SetAsBox(10.0f, 0.5f, b2Vec2(0.0f, -10.0f), 0.0);
            body_.CreateFixture(shape, 5.0f);

            b2RevoluteJointDef jd = new b2RevoluteJointDef();
            jd.bodyA = ground;
            jd.bodyB = body_;
            jd.localAnchorA.Set(0.0f, 10.0f);
            jd.localAnchorB.Set(0.0f, 0.0f);
            jd.referenceAngle = 0.0f;
            jd.motorSpeed     = 0.05f * b2_pi;
            jd.maxMotorTorque = 1e8f;
            jd.enableMotor    = true;
            m_joint = cast(b2RevoluteJoint)m_world.CreateJoint(jd);
        }

        m_count = 0;
    }

    override void Step(Settings* settings)
    {
        super.Step(settings);

        if (m_count < e_count)
        {
            b2BodyDef bd;
            bd.type = b2_dynamicBody;
            bd.position.Set(0.0f, 10.0f);
            b2Body* body_ = m_world.CreateBody(&bd);

            auto shape = new b2PolygonShape();
            shape.SetAsBox(0.125f, 0.125f);
            body_.CreateFixture(shape, 1.0f);

            ++m_count;
        }
    }

    static Test Create()
    {
        return new typeof(this);
    }

    b2RevoluteJoint m_joint;
    int32 m_count;
}
