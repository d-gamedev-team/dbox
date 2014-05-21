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
module dbox.collision.b2collision;

import core.stdc.limits;
import core.stdc.float_;
import core.stdc.stdlib;
import core.stdc.string;

import dbox.common;
import dbox.collision;
import dbox.collision.shapes;

/// @file
/// Structures and functions used for computing contact points, distance
/// queries, and TOI queries.

//
const uint8 b2_nullFeature = UCHAR_MAX;

/// The features that intersect to form the contact point
/// This must be 4 bytes or less.
struct b2ContactFeature
{
    enum Type
    {
        e_vertex = 0,
        e_face   = 1
    }

    alias e_vertex = Type.e_vertex;
    alias e_face = Type.e_face;

    uint8 indexA;  ///< Feature index on shapeA
    uint8 indexB;  ///< Feature index on shapeB
    uint8 typeA;   ///< The feature type on shapeA
    uint8 typeB;   ///< The feature type on shapeB
}

/// Contact ids to facilitate warm starting.
union b2ContactID
{
    b2ContactFeature cf;
    uint32 key;                                         ///< Used to quickly compare contact ids.
}

/// A manifold point is a contact point belonging to a contact
/// manifold. It holds details related to the geometry and dynamics
/// of the contact points.
/// The local point usage depends on the manifold type:
/// -e_circles: the local center of circleB
/// -e_faceA: the local center of cirlceB or the clip point of polygonB
/// -e_faceB: the clip point of polygonA
/// This structure is stored across time steps, so we keep it small.
/// Note: the impulses are used for internal caching and may not
/// provide reliable contact forces, especially for high speed collisions.
struct b2ManifoldPoint
{
    b2Vec2 localPoint;           ///< usage depends on manifold type
    float32 normalImpulse = 0;   ///< the non-penetration impulse
    float32 tangentImpulse = 0;  ///< the friction impulse
    b2ContactID id;              ///< uniquely identifies a contact point between two shapes
}

/// A manifold for two touching convex shapes.
/// Box2D supports multiple types of contact:
/// - clip point versus plane with radius
/// - point versus point with radius (circles)
/// The local point usage depends on the manifold type:
/// -e_circles: the local center of circleA
/// -e_faceA: the center of faceA
/// -e_faceB: the center of faceB
/// Similarly the local normal usage:
/// -e_circles: not used
/// -e_faceA: the normal on polygonA
/// -e_faceB: the normal on polygonB
/// We store contacts in this way so that position correction can
/// account for movement, which is critical for continuous physics.
/// All contact scenarios must be expressed in one of these types.
/// This structure is stored across time steps, so we keep it small.
struct b2Manifold
{
    enum Type
    {
        e_circles,
        e_faceA,
        e_faceB
    }

    alias e_circles = Type.e_circles;
    alias e_faceA = Type.e_faceA;
    alias e_faceB = Type.e_faceB;

    b2ManifoldPoint[b2_maxManifoldPoints] points;  ///< the points of contact
    b2Vec2 localNormal;                            ///< not use for Type.e_points
    b2Vec2 localPoint;                             ///< usage depends on manifold type
    Type type;
    int32 pointCount;                              ///< the number of manifold points
}

/// This is used to compute the current state of a contact manifold.
struct b2WorldManifold
{
    /// Evaluate the manifold with supplied transforms. This assumes
    /// modest motion from the original state. This does not change the
    /// point count, impulses, etc. The radii must come from the shapes
    /// that generated the manifold.
    void Initialize(const(b2Manifold)* manifold,
                    b2Transform xfA, float32 radiusA,
                    b2Transform xfB, float32 radiusB)
    {
        if (manifold.pointCount == 0)
        {
            return;
        }

        switch (manifold.type)
        {
            case b2Manifold.e_circles:
            {
                normal.Set(1.0f, 0.0f);
                b2Vec2 pointA = b2Mul(xfA, manifold.localPoint);
                b2Vec2 pointB = b2Mul(xfB, manifold.points[0].localPoint);

                if (b2DistanceSquared(pointA, pointB) > b2_epsilon * b2_epsilon)
                {
                    normal = pointB - pointA;
                    normal.Normalize();
                }

                b2Vec2 cA = pointA + radiusA * normal;
                b2Vec2 cB = pointB - radiusB * normal;
                points[0]      = 0.5f * (cA + cB);
                separations[0] = b2Dot(cB - cA, normal);
            }
            break;

            case b2Manifold.e_faceA:
            {
                normal = b2Mul(xfA.q, manifold.localNormal);
                b2Vec2 planePoint = b2Mul(xfA, manifold.localPoint);

                for (int32 i = 0; i < manifold.pointCount; ++i)
                {
                    b2Vec2 clipPoint = b2Mul(xfB, manifold.points[i].localPoint);
                    b2Vec2 cA        = clipPoint + (radiusA - b2Dot(clipPoint - planePoint, normal)) * normal;
                    b2Vec2 cB        = clipPoint - radiusB * normal;
                    points[i]      = 0.5f * (cA + cB);
                    separations[i] = b2Dot(cB - cA, normal);
                }
            }
            break;

            case b2Manifold.e_faceB:
            {
                normal = b2Mul(xfB.q, manifold.localNormal);
                b2Vec2 planePoint = b2Mul(xfB, manifold.localPoint);

                for (int32 i = 0; i < manifold.pointCount; ++i)
                {
                    b2Vec2 clipPoint = b2Mul(xfA, manifold.points[i].localPoint);
                    b2Vec2 cB        = clipPoint + (radiusB - b2Dot(clipPoint - planePoint, normal)) * normal;
                    b2Vec2 cA        = clipPoint - radiusA * normal;
                    points[i]      = 0.5f * (cA + cB);
                    separations[i] = b2Dot(cA - cB, normal);
                }

                // Ensure normal points from A to B.
                normal = -normal;
            }
            break;

            default:
                break;
        }
    }

    b2Vec2 normal;                                  ///< world vector pointing from A to B
    b2Vec2[b2_maxManifoldPoints] points;            ///< world contact point (point of intersection)
    float32[b2_maxManifoldPoints] separations = 0;  ///< a negative value indicates overlap, in meters
}

/// This is used for determining the state of contact points.
enum b2PointState
{
    b2_nullState,     ///< point does not exist
    b2_addState,      ///< point was added in the update
    b2_persistState,  ///< point persisted across the update
    b2_removeState    ///< point was removed in the update
}

alias b2_nullState = b2PointState.b2_nullState;
alias b2_addState = b2PointState.b2_addState;
alias b2_persistState = b2PointState.b2_persistState;
alias b2_removeState = b2PointState.b2_removeState;

/// Compute the point states given two manifolds. The states pertain to the transition from manifold1
/// to manifold2. So state1 is either persist or remove while state2 is either add or persist.
void b2GetPointStates(ref b2PointState[b2_maxManifoldPoints] state1,
                      ref b2PointState[b2_maxManifoldPoints] state2,
                      const(b2Manifold)* manifold1,
                      const(b2Manifold)* manifold2)
{
    for (int32 i = 0; i < b2_maxManifoldPoints; ++i)
    {
        state1[i] = b2_nullState;
        state2[i] = b2_nullState;
    }

    // Detect persists and removes.
    for (int32 i = 0; i < manifold1.pointCount; ++i)
    {
        b2ContactID id = manifold1.points[i].id;

        state1[i] = b2_removeState;

        for (int32 j = 0; j < manifold2.pointCount; ++j)
        {
            if (manifold2.points[j].id.key == id.key)
            {
                state1[i] = b2_persistState;
                break;
            }
        }
    }

    // Detect persists and adds.
    for (int32 i = 0; i < manifold2.pointCount; ++i)
    {
        b2ContactID id = manifold2.points[i].id;

        state2[i] = b2_addState;

        for (int32 j = 0; j < manifold1.pointCount; ++j)
        {
            if (manifold1.points[j].id.key == id.key)
            {
                state2[i] = b2_persistState;
                break;
            }
        }
    }
}

/// Used for computing contact manifolds.
struct b2ClipVertex
{
    b2Vec2 v;
    b2ContactID id;
}

/// Ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
struct b2RayCastInput
{
    b2Vec2 p1, p2;
    float32 maxFraction = 0;
}

/// Ray-cast output data. The ray hits at p1 + fraction * (p2 - p1), where p1 and p2
/// come from b2RayCastInput.
struct b2RayCastOutput
{
    b2Vec2 normal;
    float32 fraction = 0;
}

/// An axis aligned bounding box.
struct b2AABB
{
    /// Verify that the bounds are sorted.
    bool IsValid() const
    {
        b2Vec2 d   = upperBound - lowerBound;
        bool valid = d.x >= 0.0f && d.y >= 0.0f;
        valid = valid && lowerBound.IsValid() && upperBound.IsValid();
        return valid;
    }

    /// Get the center of the AABB.
    b2Vec2 GetCenter() const
    {
        return 0.5f * (lowerBound + upperBound);
    }

    /// Get the extents of the AABB (half-widths).
    b2Vec2 GetExtents() const
    {
        return 0.5f * (upperBound - lowerBound);
    }

    /// Get the perimeter length
    float32 GetPerimeter() const
    {
        float32 wx = upperBound.x - lowerBound.x;
        float32 wy = upperBound.y - lowerBound.y;
        return 2.0f * (wx + wy);
    }

    /// Combine an AABB into this one.
    void Combine(b2AABB aabb)
    {
        lowerBound = b2Min(lowerBound, aabb.lowerBound);
        upperBound = b2Max(upperBound, aabb.upperBound);
    }

    /// Combine two AABBs into this one.
    void Combine(b2AABB aabb1, b2AABB aabb2)
    {
        lowerBound = b2Min(aabb1.lowerBound, aabb2.lowerBound);
        upperBound = b2Max(aabb1.upperBound, aabb2.upperBound);
    }

    /// Does this aabb contain the provided AABB.
    bool Contains(b2AABB aabb) const
    {
        bool result = true;
        result = result && lowerBound.x <= aabb.lowerBound.x;
        result = result && lowerBound.y <= aabb.lowerBound.y;
        result = result && aabb.upperBound.x <= upperBound.x;
        result = result && aabb.upperBound.y <= upperBound.y;
        return result;
    }

    // From Real-time Collision Detection, p179.
    bool RayCast(b2RayCastOutput* output, b2RayCastInput input) const
    {
        float32 tmin = -b2_maxFloat;
        float32 tmax = b2_maxFloat;

        b2Vec2 p    = input.p1;
        b2Vec2 d    = input.p2 - input.p1;
        b2Vec2 absD = b2Abs(d);

        b2Vec2 normal;

        for (int32 i = 0; i < 2; ++i)
        {
            if (absD(i) < b2_epsilon)
            {
                // Parallel.
                if (p(i) < lowerBound(i) || upperBound(i) < p(i))
                {
                    return false;
                }
            }
            else
            {
                float32 inv_d = 1.0f / d(i);
                float32 t1    = (lowerBound(i) - p(i)) * inv_d;
                float32 t2    = (upperBound(i) - p(i)) * inv_d;

                // Sign of the normal vector.
                float32 s = -1.0f;

                if (t1 > t2)
                {
                    b2Swap(t1, t2);
                    s = 1.0f;
                }

                // Push the min up
                if (t1 > tmin)
                {
                    normal.SetZero();
                    normal(i) = s;
                    tmin      = t1;
                }

                // Pull the max down
                tmax = b2Min(tmax, t2);

                if (tmin > tmax)
                {
                    return false;
                }
            }
        }

        // Does the ray start inside the box?
        // Does the ray intersect beyond the max fraction?
        if (tmin < 0.0f || input.maxFraction < tmin)
        {
            return false;
        }

        // Intersection.
        output.fraction = tmin;
        output.normal   = normal;
        return true;
    }

    b2Vec2 lowerBound;  ///< the lower vertex
    b2Vec2 upperBound;  ///< the upper vertex
}

// Sutherland-Hodgman clipping.
/// Clipping for contact manifolds.
int32 b2ClipSegmentToLine(ref b2ClipVertex[2] vOut, ref const(b2ClipVertex)[2] vIn,
                          b2Vec2 normal, float32 offset, int32 vertexIndexA)
{
    // Start with no output points
    int32 numOut = 0;

    // Calculate the distance of end points to the line
    float32 distance0 = b2Dot(normal, vIn[0].v) - offset;
    float32 distance1 = b2Dot(normal, vIn[1].v) - offset;

    // If the points are behind the plane
    if (distance0 <= 0.0f)
        vOut[numOut++] = vIn[0];

    if (distance1 <= 0.0f)
        vOut[numOut++] = vIn[1];

    // If the points are on different sides of the plane
    if (distance0 * distance1 < 0.0f)
    {
        // Find intersection point of edge and plane
        float32 interp = distance0 / (distance0 - distance1);
        vOut[numOut].v = vIn[0].v + interp * (vIn[1].v - vIn[0].v);

        // VertexA is hitting edgeB.
        vOut[numOut].id.cf.indexA = cast(uint8)(vertexIndexA);
        vOut[numOut].id.cf.indexB = vIn[0].id.cf.indexB;
        vOut[numOut].id.cf.typeA  = b2ContactFeature.e_vertex;
        vOut[numOut].id.cf.typeB  = b2ContactFeature.e_face;
        ++numOut;
    }

    return numOut;
}

/// Determine if two generic shapes overlap.
bool b2TestOverlap(const(b2Shape) shapeA, int32 indexA,
                   const(b2Shape) shapeB, int32 indexB,
                   b2Transform xfA, b2Transform xfB)
{
    b2DistanceInput input;
    input.proxyA.Set(shapeA, indexA);
    input.proxyB.Set(shapeB, indexB);
    input.transformA = xfA;
    input.transformB = xfB;
    input.useRadii   = true;

    b2SimplexCache cache;
    cache.count = 0;

    b2DistanceOutput output;

    b2Distance(&output, &cache, &input);

    return output.distance < 10.0f * b2_epsilon;
}

bool b2TestOverlap(b2AABB a, b2AABB b)
{
    b2Vec2 d1, d2;
    d1 = b.lowerBound - a.upperBound;
    d2 = a.lowerBound - b.upperBound;

    if (d1.x > 0.0f || d1.y > 0.0f)
        return false;

    if (d2.x > 0.0f || d2.y > 0.0f)
        return false;

    return true;
}
