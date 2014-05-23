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
module tests.shapeediting;

import core.stdc.float_;
import core.stdc.math;

import std.string;
import std.typecons;

import deimos.glfw.glfw3;

import dbox;

import framework.debug_draw;
import framework.test;

// This is used to test sensor shapes.
class ShapeEditing : Test
{
    this()
    {
        {
            b2BodyDef bd;
            b2Body* ground = m_world.CreateBody(&bd);

            auto shape = new b2EdgeShape();
            shape.Set(b2Vec2(-40.0f, 0.0f), b2Vec2(40.0f, 0.0f));
            ground.CreateFixture(shape, 0.0f);
        }

        b2BodyDef bd;
        bd.type = b2_dynamicBody;
        bd.position.Set(0.0f, 10.0f);
        m_body = m_world.CreateBody(&bd);

        auto shape = new b2PolygonShape();
        shape.SetAsBox(4.0f, 4.0f, b2Vec2(0.0f, 0.0f), 0.0f);
        m_fixture1 = m_body.CreateFixture(shape, 10.0f);

        m_fixture2 = null;

        m_sensor = false;
    }

    override void Keyboard(int key)
    {
        switch (key)
        {
            case GLFW_KEY_C:

                if (m_fixture2 == null)
                {
                    b2CircleShape shape = new b2CircleShape();
                    shape.m_radius = 3.0f;
                    shape.m_p.Set(0.5f, -4.0f);
                    m_fixture2 = m_body.CreateFixture(shape, 10.0f);
                    m_body.SetAwake(true);
                }
                break;

            case GLFW_KEY_D:

                if (m_fixture2 != null)
                {
                    m_body.DestroyFixture(m_fixture2);
                    m_fixture2 = null;
                    m_body.SetAwake(true);
                }
                break;

            case GLFW_KEY_S:

                if (m_fixture2 != null)
                {
                    m_sensor = !m_sensor;
                    m_fixture2.SetSensor(m_sensor);
                }
                break;

            default:
                break;
        }
    }

    override void Step(Settings* settings)
    {
        Test.Step(settings);
        g_debugDraw.DrawString(5, m_textLine, "Press: (c) create a shape, (d) destroy a shape.");
        m_textLine += DRAW_STRING_NEW_LINE;
        g_debugDraw.DrawString(5, m_textLine, format("sensor = %d", m_sensor));
        m_textLine += DRAW_STRING_NEW_LINE;
    }

    static Test Create()
    {
        return new typeof(this);
    }

    b2Body* m_body;
    b2Fixture* m_fixture1;
    b2Fixture* m_fixture2;
    bool m_sensor;
}
