module dbox.common.b2draw;

import core.stdc.float_;
import core.stdc.stdlib;
import core.stdc.string;

import dbox.common;
import dbox.common.b2math;


import dbox.common.b2math;
import dbox.common;

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

    float32 r, g, b, a = 1.0f;
}

/// Implement and register this class with a b2World* to provide debug drawing of physics
/// entities in your game.
class b2Draw
{
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

    enum
    {
        e_shapeBit = 0x0001,                                    ///< draw shapes
        e_jointBit = 0x0002,                                    ///< draw joint connections
        e_aabbBit = 0x0004,                                     ///< draw axis aligned bounding boxes
        e_pairBit = 0x0008,                                     ///< draw broad-phase pairs
        e_centerOfMassBit = 0x0010                              ///< draw center of mass frame
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

    uint32 m_drawFlags;
}
