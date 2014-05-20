module dbox.collision.b2dynamictree;

import core.stdc.float_;
import core.stdc.stdlib;
import core.stdc.string;

import dbox.common;
import dbox.common.b2math;
import dbox.collision;
import dbox.collision.shapes;

import std.string;

/*
 * Copyright (c) 2009 Erin Catto http://www.box2d.org
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

// #ifndef B2_DYNAMIC_TREE_H
// #define B2_DYNAMIC_TREE_H

import dbox.collision.b2collision;
import dbox.common.b2growablestack;

enum b2_nullNode = -1;

/// A node in the dynamic tree. The client does not interact with this directly.
struct b2TreeNode
{
    bool IsLeaf() const
    {
        return child1 == b2_nullNode;
    }

    /// Enlarged AABB
    b2AABB aabb;

    void* userData;

    union
    {
        int32 parent;
        int32 next;
    }

    int32 child1;
    int32 child2;

    // leaf = 0, free node = -1
    int32 height;
}

/// A dynamic AABB tree broad-phase, inspired by Nathanael Presson's btDbvt.
/// A dynamic tree arranges data in a binary tree to accelerate
/// queries such as volume queries and ray casts. Leafs are proxies
/// with an AABB. In the tree we expand the proxy AABB by b2_fatAABBFactor
/// so that the proxy AABB is bigger than the client object. This allows the client
/// object to move by small amounts without triggering a tree update.
///
/// Nodes are pooled and relocatable, so we use node indices rather than pointers.
struct b2DynamicTree
{
    @disable this();
    @disable this(this);

    this(int)
    {
        m_root = b2_nullNode;

        m_nodeCapacity = 16;
        m_nodeCount    = 0;
        m_nodes        = cast(b2TreeNode*)b2Alloc(m_nodeCapacity * getSizeOf!b2TreeNode);
        memset(m_nodes, 0, m_nodeCapacity * getSizeOf!b2TreeNode);

        // Build a linked list for the free list.
        for (int32 i = 0; i < m_nodeCapacity - 1; ++i)
        {
            m_nodes[i].next   = i + 1;
            m_nodes[i].height = -1;
        }

        m_nodes[m_nodeCapacity - 1].next   = b2_nullNode;
        m_nodes[m_nodeCapacity - 1].height = -1;
        m_freeList = 0;

        m_path = 0;

        m_insertionCount = 0;
    }

    b2TreeNode* m_nodes;

    ~this()
    {
        // This frees the entire tree in one shot.
        b2Free(this.m_nodes);
    }

    // Allocate a node from the pool. Grow the pool if necessary.
    int32 AllocateNode()
    {
        // Expand the node pool as needed.
        if (m_freeList == b2_nullNode)
        {
            assert(m_nodeCount == m_nodeCapacity);

            // The free list is empty. Rebuild a bigger pool.
            b2TreeNode* oldNodes = this.m_nodes;
            m_nodeCapacity *= 2;
            this.m_nodes         = cast(b2TreeNode*)b2Alloc(m_nodeCapacity * getSizeOf!b2TreeNode);
            memcpy(this.m_nodes, oldNodes, m_nodeCount * getSizeOf!b2TreeNode);
            b2Free(oldNodes);

            // Build a linked list for the free list. The parent
            // pointer becomes the "next" pointer.
            for (int32 i = m_nodeCount; i < m_nodeCapacity - 1; ++i)
            {
                this.m_nodes[i].next   = i + 1;
                this.m_nodes[i].height = -1;
            }

            this.m_nodes[m_nodeCapacity - 1].next   = b2_nullNode;
            this.m_nodes[m_nodeCapacity - 1].height = -1;
            m_freeList = m_nodeCount;
        }

        // Peel a node off the free list.
        int32 nodeId = m_freeList;
        m_freeList = this.m_nodes[nodeId].next;
        this.m_nodes[nodeId].parent   = b2_nullNode;
        this.m_nodes[nodeId].child1   = b2_nullNode;
        this.m_nodes[nodeId].child2   = b2_nullNode;
        this.m_nodes[nodeId].height   = 0;
        this.m_nodes[nodeId].userData = null;
        ++m_nodeCount;
        return nodeId;
    }

    // Return a node to the pool.
    void FreeNode(int32 nodeId)
    {
        assert(0 <= nodeId && nodeId < m_nodeCapacity);
        assert(0 < m_nodeCount);
        this.m_nodes[nodeId].next   = m_freeList;
        this.m_nodes[nodeId].height = -1;
        m_freeList = nodeId;
        --m_nodeCount;
    }

    // Create a proxy in the tree as a leaf node. We return the index
    // of the node instead of a pointer so that we can grow
    // the node pool.
    int32 CreateProxy(b2AABB aabb, void* userData)
    {
        int32 proxyId = AllocateNode();

        // Fatten the aabb.
        b2Vec2 r = b2Vec2(b2_aabbExtension, b2_aabbExtension);
        this.m_nodes[proxyId].aabb.lowerBound = aabb.lowerBound - r;
        this.m_nodes[proxyId].aabb.upperBound = aabb.upperBound + r;
        this.m_nodes[proxyId].userData        = userData;
        this.m_nodes[proxyId].height = 0;

        InsertLeaf(proxyId);

        return proxyId;
    }

    void DestroyProxy(int32 proxyId)
    {
        assert(0 <= proxyId && proxyId < m_nodeCapacity);
        assert(this.m_nodes[proxyId].IsLeaf());

        RemoveLeaf(proxyId);
        FreeNode(proxyId);
    }

    bool MoveProxy(int32 proxyId, b2AABB aabb, b2Vec2 displacement)
    {
        assert(0 <= proxyId && proxyId < m_nodeCapacity);

        assert(this.m_nodes[proxyId].IsLeaf());

        if (this.m_nodes[proxyId].aabb.Contains(aabb))
        {
            return false;
        }

        RemoveLeaf(proxyId);

        // Extend AABB.
        b2AABB b = aabb;
        b2Vec2 r = b2Vec2(b2_aabbExtension, b2_aabbExtension);
        b.lowerBound = b.lowerBound - r;
        b.upperBound = b.upperBound + r;

        // Predict AABB displacement.
        b2Vec2 d = b2_aabbMultiplier * displacement;

        if (d.x < 0.0f)
        {
            b.lowerBound.x += d.x;
        }
        else
        {
            b.upperBound.x += d.x;
        }

        if (d.y < 0.0f)
        {
            b.lowerBound.y += d.y;
        }
        else
        {
            b.upperBound.y += d.y;
        }

        this.m_nodes[proxyId].aabb = b;

        InsertLeaf(proxyId);
        return true;
    }

    void InsertLeaf(int32 leaf)
    {
        ++m_insertionCount;

        if (m_root == b2_nullNode)
        {
            m_root = leaf;
            this.m_nodes[m_root].parent = b2_nullNode;
            return;
        }

        // Find the best sibling for this node
        b2AABB leafAABB = this.m_nodes[leaf].aabb;
        int32  index    = m_root;

        while (this.m_nodes[index].IsLeaf() == false)
        {
            int32 child1 = this.m_nodes[index].child1;
            int32 child2 = this.m_nodes[index].child2;

            float32 area = this.m_nodes[index].aabb.GetPerimeter();

            b2AABB combinedAABB;
            combinedAABB.Combine(this.m_nodes[index].aabb, leafAABB);
            float32 combinedArea = combinedAABB.GetPerimeter();

            // Cost of creating a new parent for this node and the new leaf
            float32 cost = 2.0f * combinedArea;

            // Minimum cost of pushing the leaf further down the tree
            float32 inheritanceCost = 2.0f * (combinedArea - area);

            // Cost of descending into child1
            float32 cost1 = 0;

            if (this.m_nodes[child1].IsLeaf())
            {
                b2AABB aabb;
                aabb.Combine(leafAABB, this.m_nodes[child1].aabb);
                cost1 = aabb.GetPerimeter() + inheritanceCost;
            }
            else
            {
                b2AABB aabb;
                aabb.Combine(leafAABB, this.m_nodes[child1].aabb);
                float32 oldArea = this.m_nodes[child1].aabb.GetPerimeter();
                float32 newArea = aabb.GetPerimeter();
                cost1 = (newArea - oldArea) + inheritanceCost;
            }

            // Cost of descending into child2
            float32 cost2 = 0;

            if (this.m_nodes[child2].IsLeaf())
            {
                b2AABB aabb;
                aabb.Combine(leafAABB, this.m_nodes[child2].aabb);
                cost2 = aabb.GetPerimeter() + inheritanceCost;
            }
            else
            {
                b2AABB aabb;
                aabb.Combine(leafAABB, this.m_nodes[child2].aabb);
                float32 oldArea = this.m_nodes[child2].aabb.GetPerimeter();
                float32 newArea = aabb.GetPerimeter();
                cost2 = newArea - oldArea + inheritanceCost;
            }

            // Descend according to the minimum cost.
            if (cost < cost1 && cost < cost2)
            {
                break;
            }

            // Descend
            if (cost1 < cost2)
            {
                index = child1;
            }
            else
            {
                index = child2;
            }
        }

        int32 sibling = index;

        // Create a new parent.
        int32 oldParent = this.m_nodes[sibling].parent;
        int32 newParent = AllocateNode();
        this.m_nodes[newParent].parent   = oldParent;
        this.m_nodes[newParent].userData = null;
        this.m_nodes[newParent].aabb.Combine(leafAABB, this.m_nodes[sibling].aabb);
        this.m_nodes[newParent].height = this.m_nodes[sibling].height + 1;

        if (oldParent != b2_nullNode)
        {
            // The sibling was not the root.
            if (this.m_nodes[oldParent].child1 == sibling)
            {
                this.m_nodes[oldParent].child1 = newParent;
            }
            else
            {
                this.m_nodes[oldParent].child2 = newParent;
            }

            this.m_nodes[newParent].child1 = sibling;
            this.m_nodes[newParent].child2 = leaf;
            this.m_nodes[sibling].parent   = newParent;
            this.m_nodes[leaf].parent      = newParent;
        }
        else
        {
            // The sibling was the root.
            this.m_nodes[newParent].child1 = sibling;
            this.m_nodes[newParent].child2 = leaf;
            this.m_nodes[sibling].parent   = newParent;
            this.m_nodes[leaf].parent      = newParent;
            m_root = newParent;
        }

        // Walk back up the tree fixing heights and AABBs
        index = this.m_nodes[leaf].parent;

        while (index != b2_nullNode)
        {
            index = Balance(index);

            int32 child1 = this.m_nodes[index].child1;
            int32 child2 = this.m_nodes[index].child2;

            assert(child1 != b2_nullNode);
            assert(child2 != b2_nullNode);

            this.m_nodes[index].height = 1 + b2Max(this.m_nodes[child1].height, this.m_nodes[child2].height);
            this.m_nodes[index].aabb.Combine(this.m_nodes[child1].aabb, this.m_nodes[child2].aabb);

            index = this.m_nodes[index].parent;
        }

        // Validate();
    }

    void RemoveLeaf(int32 leaf)
    {
        if (leaf == m_root)
        {
            m_root = b2_nullNode;
            return;
        }

        int32 parent      = this.m_nodes[leaf].parent;
        int32 grandParent = this.m_nodes[parent].parent;
        int32 sibling;

        if (this.m_nodes[parent].child1 == leaf)
        {
            sibling = this.m_nodes[parent].child2;
        }
        else
        {
            sibling = this.m_nodes[parent].child1;
        }

        if (grandParent != b2_nullNode)
        {
            // Destroy parent and connect sibling to grandParent.
            if (this.m_nodes[grandParent].child1 == parent)
            {
                this.m_nodes[grandParent].child1 = sibling;
            }
            else
            {
                this.m_nodes[grandParent].child2 = sibling;
            }
            this.m_nodes[sibling].parent = grandParent;
            FreeNode(parent);

            // Adjust ancestor bounds.
            int32 index = grandParent;

            while (index != b2_nullNode)
            {
                index = Balance(index);

                int32 child1 = this.m_nodes[index].child1;
                int32 child2 = this.m_nodes[index].child2;

                this.m_nodes[index].aabb.Combine(this.m_nodes[child1].aabb, this.m_nodes[child2].aabb);
                this.m_nodes[index].height = 1 + b2Max(this.m_nodes[child1].height, this.m_nodes[child2].height);

                index = this.m_nodes[index].parent;
            }
        }
        else
        {
            m_root = sibling;
            this.m_nodes[sibling].parent = b2_nullNode;
            FreeNode(parent);
        }

        // Validate();
    }

    // Perform a left or right rotation if node A is imbalanced.
    // Returns the new root index.
    int32 Balance(int32 iA)
    {
        assert(iA != b2_nullNode);

        b2TreeNode* A = this.m_nodes + iA;

        if (A.IsLeaf() || A.height < 2)
        {
            return iA;
        }

        int32 iB = A.child1;
        int32 iC = A.child2;
        assert(0 <= iB && iB < m_nodeCapacity);
        assert(0 <= iC && iC < m_nodeCapacity);

        b2TreeNode* B = this.m_nodes + iB;
        b2TreeNode* C = this.m_nodes + iC;

        int32 balance = C.height - B.height;

        // Rotate C up
        if (balance > 1)
        {
            int32 iF      = C.child1;
            int32 iG      = C.child2;
            b2TreeNode* F = this.m_nodes + iF;
            b2TreeNode* G = this.m_nodes + iG;
            assert(0 <= iF && iF < m_nodeCapacity);
            assert(0 <= iG && iG < m_nodeCapacity);

            // Swap A and C
            C.child1 = iA;
            C.parent = A.parent;
            A.parent = iC;

            // A's old parent should point to C
            if (C.parent != b2_nullNode)
            {
                if (this.m_nodes[C.parent].child1 == iA)
                {
                    this.m_nodes[C.parent].child1 = iC;
                }
                else
                {
                    assert(this.m_nodes[C.parent].child2 == iA);
                    this.m_nodes[C.parent].child2 = iC;
                }
            }
            else
            {
                m_root = iC;
            }

            // Rotate
            if (F.height > G.height)
            {
                C.child2 = iF;
                A.child2 = iG;
                G.parent = iA;
                A.aabb.Combine(B.aabb, G.aabb);
                C.aabb.Combine(A.aabb, F.aabb);

                A.height = 1 + b2Max(B.height, G.height);
                C.height = 1 + b2Max(A.height, F.height);
            }
            else
            {
                C.child2 = iG;
                A.child2 = iF;
                F.parent = iA;
                A.aabb.Combine(B.aabb, F.aabb);
                C.aabb.Combine(A.aabb, G.aabb);

                A.height = 1 + b2Max(B.height, F.height);
                C.height = 1 + b2Max(A.height, G.height);
            }

            return iC;
        }

        // Rotate B up
        if (balance < -1)
        {
            int32 iD      = B.child1;
            int32 iE      = B.child2;
            b2TreeNode* D = this.m_nodes + iD;
            b2TreeNode* E = this.m_nodes + iE;
            assert(0 <= iD && iD < m_nodeCapacity);
            assert(0 <= iE && iE < m_nodeCapacity);

            // Swap A and B
            B.child1 = iA;
            B.parent = A.parent;
            A.parent = iB;

            // A's old parent should point to B
            if (B.parent != b2_nullNode)
            {
                if (this.m_nodes[B.parent].child1 == iA)
                {
                    this.m_nodes[B.parent].child1 = iB;
                }
                else
                {
                    assert(this.m_nodes[B.parent].child2 == iA);
                    this.m_nodes[B.parent].child2 = iB;
                }
            }
            else
            {
                m_root = iB;
            }

            // Rotate
            if (D.height > E.height)
            {
                B.child2 = iD;
                A.child1 = iE;
                E.parent = iA;
                A.aabb.Combine(C.aabb, E.aabb);
                B.aabb.Combine(A.aabb, D.aabb);

                A.height = 1 + b2Max(C.height, E.height);
                B.height = 1 + b2Max(A.height, D.height);
            }
            else
            {
                B.child2 = iE;
                A.child1 = iD;
                D.parent = iA;
                A.aabb.Combine(C.aabb, D.aabb);
                B.aabb.Combine(A.aabb, E.aabb);

                A.height = 1 + b2Max(C.height, D.height);
                B.height = 1 + b2Max(A.height, E.height);
            }

            return iB;
        }

        return iA;
    }

    int32 GetHeight() const
    {
        if (m_root == b2_nullNode)
        {
            return 0;
        }

        return this.m_nodes[m_root].height;
    }

    //
    float32 GetAreaRatio() const
    {
        if (m_root == b2_nullNode)
        {
            return 0.0f;
        }

        const b2TreeNode* root = this.m_nodes + m_root;
        float32 rootArea       = root.aabb.GetPerimeter();

        float32 totalArea = 0.0f;

        for (int32 i = 0; i < m_nodeCapacity; ++i)
        {
            const b2TreeNode* node = this.m_nodes + i;

            if (node.height < 0)
            {
                // Free node in pool
                continue;
            }

            totalArea += node.aabb.GetPerimeter();
        }

        return totalArea / rootArea;
    }

    // Compute the height of a sub-tree.
    int32 ComputeHeight(int32 nodeId) const
    {
        assert(0 <= nodeId && nodeId < m_nodeCapacity);
        b2TreeNode* node = cast(b2TreeNode*)(this.m_nodes + nodeId);

        if (node.IsLeaf())
        {
            return 0;
        }

        int32 height1 = ComputeHeight(node.child1);
        int32 height2 = ComputeHeight(node.child2);
        return 1 + b2Max(height1, height2);
    }

    int32 ComputeHeight() const
    {
        int32 height = ComputeHeight(m_root);
        return height;
    }

    void ValidateStructure(int32 index) const
    {
        if (index == b2_nullNode)
        {
            return;
        }

        if (index == m_root)
        {
            assert(this.m_nodes[index].parent == b2_nullNode);
        }

        const b2TreeNode* node = this.m_nodes + index;

        int32 child1 = node.child1;
        int32 child2 = node.child2;

        if (node.IsLeaf())
        {
            assert(child1 == b2_nullNode);
            assert(child2 == b2_nullNode);
            assert(node.height == 0);
            return;
        }

        assert(0 <= child1 && child1 < m_nodeCapacity);
        assert(0 <= child2 && child2 < m_nodeCapacity);

        assert(this.m_nodes[child1].parent == index);
        assert(this.m_nodes[child2].parent == index);

        ValidateStructure(child1);
        ValidateStructure(child2);
    }

    void ValidateMetrics(int32 index) const
    {
        if (index == b2_nullNode)
        {
            return;
        }

        const b2TreeNode* node = this.m_nodes + index;

        int32 child1 = node.child1;
        int32 child2 = node.child2;

        if (node.IsLeaf())
        {
            assert(child1 == b2_nullNode);
            assert(child2 == b2_nullNode);
            assert(node.height == 0);
            return;
        }

        assert(0 <= child1 && child1 < m_nodeCapacity);
        assert(0 <= child2 && child2 < m_nodeCapacity);

        int32 height1 = this.m_nodes[child1].height;
        int32 height2 = this.m_nodes[child2].height;
        int32 height;
        height = 1 + b2Max(height1, height2);
        assert(node.height == height);

        b2AABB aabb;
        aabb.Combine(this.m_nodes[child1].aabb, this.m_nodes[child2].aabb);

        assert(aabb.lowerBound == node.aabb.lowerBound);
        assert(aabb.upperBound == node.aabb.upperBound);

        ValidateMetrics(child1);
        ValidateMetrics(child2);
    }

    void Validate() const
    {
        ValidateStructure(m_root);
        ValidateMetrics(m_root);

        int32 freeCount = 0;
        int32 freeIndex = m_freeList;

        while (freeIndex != b2_nullNode)
        {
            assert(0 <= freeIndex && freeIndex < m_nodeCapacity);
            freeIndex = this.m_nodes[freeIndex].next;
            ++freeCount;
        }

        assert(GetHeight() == ComputeHeight());

        assert(m_nodeCount + freeCount == m_nodeCapacity);
    }

    int32 GetMaxBalance() const
    {
        int32 maxBalance = 0;

        for (int32 i = 0; i < m_nodeCapacity; ++i)
        {
            const b2TreeNode* node = this.m_nodes + i;

            if (node.height <= 1)
            {
                continue;
            }

            assert(node.IsLeaf() == false);

            int32 child1  = node.child1;
            int32 child2  = node.child2;
            int32 balance = b2Abs(this.m_nodes[child2].height - this.m_nodes[child1].height);
            maxBalance = b2Max(maxBalance, balance);
        }

        return maxBalance;
    }

    void RebuildBottomUp()
    {
        int32* nodes = cast(int32*)b2Alloc(m_nodeCount * getSizeOf!int32);
        int32  count = 0;

        // Build array of leaves. Free the rest.
        for (int32 i = 0; i < m_nodeCapacity; ++i)
        {
            if (this.m_nodes[i].height < 0)
            {
                // free node in pool
                continue;
            }

            if (this.m_nodes[i].IsLeaf())
            {
                this.m_nodes[i].parent = b2_nullNode;
                nodes[count]      = i;
                ++count;
            }
            else
            {
                FreeNode(i);
            }
        }

        while (count > 1)
        {
            float32 minCost = b2_maxFloat;
            int32 iMin      = -1, jMin = -1;

            for (int32 i = 0; i < count; ++i)
            {
                b2AABB aabbi = this.m_nodes[nodes[i]].aabb;

                for (int32 j = i + 1; j < count; ++j)
                {
                    b2AABB aabbj = this.m_nodes[nodes[j]].aabb;
                    b2AABB b;
                    b.Combine(aabbi, aabbj);
                    float32 cost = b.GetPerimeter();

                    if (cost < minCost)
                    {
                        iMin    = i;
                        jMin    = j;
                        minCost = cost;
                    }
                }
            }

            int32 index1       = nodes[iMin];
            int32 index2       = nodes[jMin];
            b2TreeNode* child1 = this.m_nodes + index1;
            b2TreeNode* child2 = this.m_nodes + index2;

            int32 parentIndex  = AllocateNode();
            b2TreeNode* parent = this.m_nodes + parentIndex;
            parent.child1 = index1;
            parent.child2 = index2;
            parent.height = 1 + b2Max(child1.height, child2.height);
            parent.aabb.Combine(child1.aabb, child2.aabb);
            parent.parent = b2_nullNode;

            child1.parent = parentIndex;
            child2.parent = parentIndex;

            nodes[jMin] = nodes[count - 1];
            nodes[iMin] = parentIndex;
            --count;
        }

        m_root = nodes[0];
        b2Free(nodes);

        Validate();
    }

    void ShiftOrigin(b2Vec2 newOrigin)
    {
        // Build array of leaves. Free the rest.
        for (int32 i = 0; i < m_nodeCapacity; ++i)
        {
            this.m_nodes[i].aabb.lowerBound -= newOrigin;
            this.m_nodes[i].aabb.upperBound -= newOrigin;
        }
    }

    void* GetUserData(int32 proxyId) const
    {
        assert(0 <= proxyId && proxyId < m_nodeCapacity);
        return cast(void*)this.m_nodes[proxyId].userData;
    }

    b2AABB GetFatAABB(int32 proxyId) const
    {
        assert(0 <= proxyId && proxyId < m_nodeCapacity,
            format("proxyId: %s m_nodeCapacity: %s", proxyId, m_nodeCapacity));

        return this.m_nodes[proxyId].aabb;
    }

    void Query(T)(T callback, b2AABB aabb) const
    {
        auto stack = b2GrowableStack!(int32, 256)(1);
        stack.Push(m_root);

        while (stack.GetCount() > 0)
        {
            int32 nodeId = stack.Pop();

            if (nodeId == b2_nullNode)
            {
                continue;
            }

            const b2TreeNode* node = this.m_nodes + nodeId;

            if (b2TestOverlap(node.aabb, aabb))
            {
                if (node.IsLeaf())
                {
                    bool proceed = callback.QueryCallback(nodeId);

                    if (proceed == false)
                    {
                        return;
                    }
                }
                else
                {
                    stack.Push(node.child1);
                    stack.Push(node.child2);
                }
            }
        }
    }

    void RayCast(T)(T callback, b2RayCastInput input) const
    {
        b2Vec2 p1 = input.p1;
        b2Vec2 p2 = input.p2;
        b2Vec2 r  = p2 - p1;
        assert(r.LengthSquared() > 0.0f);
        r.Normalize();

        // v is perpendicular to the segment.
        b2Vec2 v     = b2Cross(1.0f, r);
        b2Vec2 abs_v = b2Abs(v);

        // Separating axis for segment (Gino, p80).
        // |dot(v, p1 - c)| > dot(|v|, h)

        float32 maxFraction = input.maxFraction;

        // Build a bounding box for the segment.
        b2AABB segmentAABB;
        {
            b2Vec2 t = p1 + maxFraction * (p2 - p1);
            segmentAABB.lowerBound = b2Min(p1, t);
            segmentAABB.upperBound = b2Max(p1, t);
        }

        auto stack = b2GrowableStack!(int32, 256)(1);
        stack.Push(m_root);

        while (stack.GetCount() > 0)
        {
            int32 nodeId = stack.Pop();

            if (nodeId == b2_nullNode)
            {
                continue;
            }

            const b2TreeNode* node = this.m_nodes + nodeId;

            if (b2TestOverlap(node.aabb, segmentAABB) == false)
            {
                continue;
            }

            // Separating axis for segment (Gino, p80).
            // |dot(v, p1 - c)| > dot(|v|, h)
            b2Vec2  c = node.aabb.GetCenter();
            b2Vec2  h = node.aabb.GetExtents();
            float32 separation = b2Abs(b2Dot(v, p1 - c)) - b2Dot(abs_v, h);

            if (separation > 0.0f)
            {
                continue;
            }

            if (node.IsLeaf())
            {
                b2RayCastInput subInput;
                subInput.p1 = input.p1;
                subInput.p2 = input.p2;
                subInput.maxFraction = maxFraction;

                float32 value = callback.RayCastCallback(subInput, nodeId);

                if (value == 0.0f)
                {
                    // The client has terminated the ray cast.
                    return;
                }

                if (value > 0.0f)
                {
                    // Update segment bounding box.
                    maxFraction = value;
                    b2Vec2 t = p1 + maxFraction * (p2 - p1);
                    segmentAABB.lowerBound = b2Min(p1, t);
                    segmentAABB.upperBound = b2Max(p1, t);
                }
            }
            else
            {
                stack.Push(node.child1);
                stack.Push(node.child2);
            }
        }
    }

/* private */

    int32 m_root = b2_nullNode;

    int32 m_nodeCount;
    int32 m_nodeCapacity = 16;

    int32 m_freeList;

    /// This is used to incrementally traverse the tree for re-balancing.
    uint32 m_path;

    int32 m_insertionCount;
}
