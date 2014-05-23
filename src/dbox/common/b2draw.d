/*
 * Copyright (c) 2011 Erin Catto http://box2d.org
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
module dbox.common.b2draw;

import core.stdc.float_;
import core.stdc.stdlib;
import core.stdc.string;

import dbox.common;
import dbox.common.b2math;

/// Color for debug drawing. Each value has the range [0,1].
struct b2Color
{
    void Set(float32 ri, float32 gi, float32 bi, float32 ai = 1.0f)
    {
        r = ri;
        g = gi;
        b = bi;
        a = ai;
    }

    float32 r = 0, g = 0, b = 0, a = 1.0f;
}

/// Implement and register this class with a b2World* to provide debug drawing of physics
/// entities in your game.
class b2Draw
{
    enum
    {
        e_shapeBit = 0x0001,        ///< draw shapes
        e_jointBit = 0x0002,        ///< draw joint connections
        e_aabbBit = 0x0004,         ///< draw axis aligned bounding boxes
        e_pairBit = 0x0008,         ///< draw broad-phase pairs
        e_centerOfMassBit = 0x0010  ///< draw center of mass frame
    }

    /// Set the drawing flags.
    void SetFlags(uint32 flags)
    {
        m_drawFlags = flags;
    }

    /// Get the drawing flags.
    uint32 GetFlags() const
    {
        return m_drawFlags;
    }

    /// Append flags to the current flags.
    void AppendFlags(uint32 flags)
    {
        m_drawFlags |= flags;
    }

    /// Clear flags from the current flags.
    void ClearFlags(uint32 flags)
    {
        m_drawFlags &= ~flags;
    }

    /// Draw a closed polygon provided in CCW order.
    abstract void DrawPolygon(const(b2Vec2)* vertices, int32 vertexCount, b2Color color);

    /// Draw a solid closed polygon provided in CCW order.
    abstract void DrawSolidPolygon(const(b2Vec2)* vertices, int32 vertexCount, b2Color color);

    /// Draw a circle.
    abstract void DrawCircle(b2Vec2 center, float32 radius, b2Color color);

    /// Draw a solid circle.
    abstract void DrawSolidCircle(b2Vec2 center, float32 radius, b2Vec2 axis, b2Color color);

    /// Draw a line segment.
    abstract void DrawSegment(b2Vec2 p1, b2Vec2 p2, b2Color color);

    /// Draw a transform. Choose your own length scale.
    /// @param xf a transform.
    abstract void DrawTransform(b2Transform xf);

protected:
    uint32 m_drawFlags;
}
