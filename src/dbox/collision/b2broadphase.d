module dbox.collision.b2broadphase;

import core.stdc.float_;
import core.stdc.stdlib;
import core.stdc.string;

import dbox.common;
import dbox.common.b2math;
import dbox.collision;
import dbox.collision.shapes;

/*
 * Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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

// #ifndef B2_BROAD_PHASE_H
// #define B2_BROAD_PHASE_H

import dbox.common;
import dbox.collision.b2collision;
import dbox.collision.b2dynamictree;
import std.algorithm;

struct b2Pair
{
    int32 proxyIdA;
    int32 proxyIdB;
}

/// The broad-phase is used for computing pairs and performing volume queries and ray casts.
/// This broad-phase does not persist pairs. Instead, this reports potentially new pairs.
/// It is up to the client to consume the new pairs and to track subsequent overlap.
struct b2BroadPhase
{
    @disable this();
    @disable this(this);

    this(int)
    {
        m_tree = b2DynamicTree(1);
        m_proxyCount = 0;

        m_pairCapacity = 16;
        m_pairCount    = 0;
        m_pairBuffer   = cast(b2Pair*)b2Alloc(m_pairCapacity * getSizeOf!b2Pair);

        m_moveCapacity = 16;
        m_moveCount    = 0;
        m_moveBuffer   = cast(int32*)b2Alloc(m_moveCapacity * getSizeOf!int32);
    }

    enum
    {
        e_nullProxy = -1
    }

    void* GetUserData(int32 proxyId) const
    {
        return m_tree.GetUserData(proxyId);
    }

    bool TestOverlap(int32 proxyIdA, int32 proxyIdB) const
    {
        b2AABB aabbA = m_tree.GetFatAABB(proxyIdA);
        b2AABB aabbB = m_tree.GetFatAABB(proxyIdB);
        return b2TestOverlap(aabbA, aabbB);
    }

    b2AABB GetFatAABB(int32 proxyId) const
    {
        return m_tree.GetFatAABB(proxyId);
    }

    int32 GetProxyCount() const
    {
        return m_proxyCount;
    }

    /// Get the height of the embedded tree.
    int32 GetTreeHeight() const
    {
        return m_tree.GetHeight();
    }

    /// Get the balance of the embedded tree.
    int32 GetTreeBalance() const
    {
        return m_tree.GetMaxBalance();
    }

    /// Get the quality metric of the embedded tree.
    float32 GetTreeQuality() const
    {
        return m_tree.GetAreaRatio();
    }

    void UpdatePairs(T)(T callback)
    {
        // Reset pair buffer
        m_pairCount = 0;

        // Perform tree queries for all moving proxies.
        for (int32 i = 0; i < m_moveCount; ++i)
        {
            m_queryProxyId = m_moveBuffer[i];

            if (m_queryProxyId == e_nullProxy)
            {
                continue;
            }

            // We have to query the tree with the fat AABB so that
            // we don't fail to create a pair that may touch later.
            b2AABB fatAABB = m_tree.GetFatAABB(m_queryProxyId);

            // Query tree, create pairs and add them pair buffer.
            m_tree.Query(&this, fatAABB);
        }

        // Reset move buffer
        m_moveCount = 0;

        // Sort the pair buffer to expose duplicates.
        // todo

        sort!b2PairLessThan(m_pairBuffer[0 .. m_pairCount]);

        // Send the pairs back to the client.
        int32 i = 0;

        while (i < m_pairCount)
        {
            b2Pair* primaryPair = m_pairBuffer + i;
            void* userDataA     = m_tree.GetUserData(primaryPair.proxyIdA);
            void* userDataB     = m_tree.GetUserData(primaryPair.proxyIdB);

            callback.AddPair(userDataA, userDataB);
            ++i;

            // Skip any duplicate pairs.
            while (i < m_pairCount)
            {
                b2Pair* pair = m_pairBuffer + i;

                if (pair.proxyIdA != primaryPair.proxyIdA || pair.proxyIdB != primaryPair.proxyIdB)
                {
                    break;
                }
                ++i;
            }
        }

        // Try to keep the tree balanced.
        // m_tree.Rebalance(4);
    }

    void Query(T)(T callback, b2AABB aabb) const
    {
        m_tree.Query(callback, aabb);
    }

    /// Ray-cast against the proxies in the tree. This relies on the callback
    /// to perform a exact ray-cast in the case were the proxy contains a shape.
    /// The callback also performs the any collision filtering. This has performance
    /// roughly equal to k * log(n), where k is the number of collisions and n is the
    /// number of proxies in the tree.
    /// @param input the ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
    /// @param callback a callback class that is called for each proxy that is hit by the ray.
    void RayCast(T)(T callback, b2RayCastInput input) const
    {
        m_tree.RayCast(callback, input);
    }

    /// Shift the world origin. Useful for large worlds.
    /// The shift formula is: position -= newOrigin
    /// @param newOrigin the new origin with respect to the old origin
    void ShiftOrigin(b2Vec2 newOrigin)
    {
        m_tree.ShiftOrigin(newOrigin);
    }

    ~this()
    {
        b2Free(m_moveBuffer);
        b2Free(m_pairBuffer);
    }

    int32 CreateProxy(b2AABB aabb, void* userData)
    {
        int32 proxyId = m_tree.CreateProxy(aabb, userData);
        ++m_proxyCount;
        BufferMove(proxyId);
        return proxyId;
    }

    void DestroyProxy(int32 proxyId)
    {
        UnBufferMove(proxyId);
        --m_proxyCount;
        m_tree.DestroyProxy(proxyId);
    }

    void MoveProxy(int32 proxyId, b2AABB aabb, b2Vec2 displacement)
    {
        bool buffer = m_tree.MoveProxy(proxyId, aabb, displacement);

        if (buffer)
        {
            BufferMove(proxyId);
        }
    }

    void TouchProxy(int32 proxyId)
    {
        BufferMove(proxyId);
    }

    void BufferMove(int32 proxyId)
    {
        if (m_moveCount == m_moveCapacity)
        {
            int32* oldBuffer = m_moveBuffer;
            m_moveCapacity *= 2;
            m_moveBuffer    = cast(int32*)b2Alloc(m_moveCapacity * getSizeOf!int32);
            memcpy(m_moveBuffer, oldBuffer, m_moveCount * getSizeOf!int32);
            b2Free(oldBuffer);
        }

        m_moveBuffer[m_moveCount] = proxyId;
        ++m_moveCount;
    }

    void UnBufferMove(int32 proxyId)
    {
        for (int32 i = 0; i < m_moveCount; ++i)
        {
            if (m_moveBuffer[i] == proxyId)
            {
                m_moveBuffer[i] = e_nullProxy;
            }
        }
    }

    // This is called from b2DynamicTree*.Query when we are gathering pairs.
    bool QueryCallback(int32 proxyId)
    {
        // A proxy cannot form a pair with itself.
        if (proxyId == m_queryProxyId)
        {
            return true;
        }

        // Grow the pair buffer as needed.
        if (m_pairCount == m_pairCapacity)
        {
            b2Pair* oldBuffer = m_pairBuffer;
            m_pairCapacity *= 2;
            m_pairBuffer    = cast(b2Pair*)b2Alloc(m_pairCapacity * getSizeOf!b2Pair);
            memcpy(m_pairBuffer, oldBuffer, m_pairCount * getSizeOf!b2Pair);
            b2Free(oldBuffer);
        }

        m_pairBuffer[m_pairCount].proxyIdA = b2Min(proxyId, m_queryProxyId);
        m_pairBuffer[m_pairCount].proxyIdB = b2Max(proxyId, m_queryProxyId);
        ++m_pairCount;

        return true;
    }

/* private */

    b2DynamicTree m_tree;

    int32 m_proxyCount;

    int32 m_moveCapacity = 16;
    int32 m_moveCount;

    int32 m_pairCapacity = 16;
    int32 m_pairCount;

    int32 m_queryProxyId;

    b2Pair* m_pairBuffer;
    int32* m_moveBuffer;
}

/// This is used to sort pairs.
bool b2PairLessThan(b2Pair pair1, b2Pair pair2)
{
    return pair1.proxyIdA < pair2.proxyIdA;
}
