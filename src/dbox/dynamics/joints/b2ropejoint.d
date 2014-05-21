/*
 * Copyright (c) 2006-2012 Erin Catto http://www.box2d.org
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
module dbox.dynamics.joints.b2ropejoint;

import core.stdc.float_;
import core.stdc.stdlib;
import core.stdc.string;

import dbox.common;
import dbox.dynamics;
import dbox.dynamics.joints;

/// Rope joint definition. This requires two body anchor points and
/// a maximum lengths.
/// Note: by default the connected objects will not collide.
/// see collideConnected in b2JointDef.
class b2RopeJointDef : b2JointDef
{
    ///
    this()
    {
        type = e_ropeJoint;
        localAnchorA.Set(-1.0f, 0.0f);
        localAnchorB.Set(1.0f, 0.0f);
        maxLength = 0.0f;
    }

    /// The local anchor point relative to bodyA's origin.
    b2Vec2 localAnchorA;

    /// The local anchor point relative to bodyB's origin.
    b2Vec2 localAnchorB;

    /// The maximum length of the rope.
    /// Warning: this must be larger than b2_linearSlop or
    /// the joint will have no effect.
    float32 maxLength = 0;
}

/// A rope joint enforces a maximum distance between two points
/// on two bodies. It has no other effect.
/// Warning: if you attempt to change the maximum length during
/// the simulation you will get some non-physical behavior.
/// A model that would allow you to dynamically modify the length
/// would have some sponginess, so I chose not to implement it
/// that way. See b2DistanceJoint if you want to dynamically
/// control length.
class b2RopeJoint : b2Joint
{
    // Limit:
    // C = norm(pB - pA) - L
    // u = (pB - pA) / norm(pB - pA)
    // Cdot = dot(u, vB + cross(wB, rB) - vA - cross(wA, rA))
    // J = [-u -cross(rA, u) u cross(rB, u)]
    // K = J * invM * JT
    // = invMassA + invIA * cross(rA, u)^2 + invMassB + invIB * cross(rB, u)^2

    ///
    this(const(b2RopeJointDef) def)
    {
        super(def);
        m_localAnchorA = def.localAnchorA;
        m_localAnchorB = def.localAnchorB;

        m_maxLength = def.maxLength;

        m_mass    = 0.0f;
        m_impulse = 0.0f;
        m_state   = e_inactiveLimit;
        m_length  = 0.0f;
    }

    ///
    override b2Vec2 GetAnchorA() const
    {
        return m_bodyA.GetWorldPoint(m_localAnchorA);
    }

    ///
    override b2Vec2 GetAnchorB() const
    {
        return m_bodyB.GetWorldPoint(m_localAnchorB);
    }

    ///
    override b2Vec2 GetReactionForce(float32 inv_dt) const
    {
        b2Vec2 F = (inv_dt * m_impulse) * m_u;
        return F;
    }

    ///
    override float32 GetReactionTorque(float32 inv_dt) const
    {
        B2_NOT_USED(inv_dt);
        return 0.0f;
    }

    /// The local anchor point relative to bodyA's origin.
    b2Vec2 GetLocalAnchorA() const
    {
        return m_localAnchorA;
    }

    /// The local anchor point relative to bodyB's origin.
    b2Vec2 GetLocalAnchorB() const
    {
        return m_localAnchorB;
    }

    /// Get/set the maximum length of the rope.
    float32 GetMaxLength() const
    {
        return m_maxLength;
    }

    /// ditto
    void SetMaxLength(float32 length)
    {
        m_maxLength = length;
    }

    ///
    b2LimitState GetLimitState() const
    {
        return m_state;
    }

    /// Dump joint to dmLog
    override void Dump()
    {
        int32 indexA = m_bodyA.m_islandIndex;
        int32 indexB = m_bodyB.m_islandIndex;

        b2Log("  b2RopeJointDef jd;\n");
        b2Log("  jd.bodyA = bodies[%d];\n", indexA);
        b2Log("  jd.bodyB = bodies[%d];\n", indexB);
        b2Log("  jd.collideConnected = bool(%d);\n", m_collideConnected);
        b2Log("  jd.localAnchorA.Set(%.15lef, %.15lef);\n", m_localAnchorA.x, m_localAnchorA.y);
        b2Log("  jd.localAnchorB.Set(%.15lef, %.15lef);\n", m_localAnchorB.x, m_localAnchorB.y);
        b2Log("  jd.maxLength = %.15lef;\n", m_maxLength);
        b2Log("  joints[%d] = m_world.CreateJoint(&jd);\n", m_index);
    }

// note: this should be package but D's access implementation is lacking.
// do not use in user code.
/* package: */
public:

    override void InitVelocityConstraints(b2SolverData data)
    {
        m_indexA       = m_bodyA.m_islandIndex;
        m_indexB       = m_bodyB.m_islandIndex;
        m_localCenterA = m_bodyA.m_sweep.localCenter;
        m_localCenterB = m_bodyB.m_sweep.localCenter;
        m_invMassA     = m_bodyA.m_invMass;
        m_invMassB     = m_bodyB.m_invMass;
        m_invIA        = m_bodyA.m_invI;
        m_invIB        = m_bodyB.m_invI;

        b2Vec2  cA = data.positions[m_indexA].c;
        float32 aA = data.positions[m_indexA].a;
        b2Vec2  vA = data.velocities[m_indexA].v;
        float32 wA = data.velocities[m_indexA].w;

        b2Vec2  cB = data.positions[m_indexB].c;
        float32 aB = data.positions[m_indexB].a;
        b2Vec2  vB = data.velocities[m_indexB].v;
        float32 wB = data.velocities[m_indexB].w;

        b2Rot qA = b2Rot(aA);
        b2Rot qB = b2Rot(aB);

        m_rA = b2Mul(qA, m_localAnchorA - m_localCenterA);
        m_rB = b2Mul(qB, m_localAnchorB - m_localCenterB);
        m_u  = cB + m_rB - cA - m_rA;

        m_length = m_u.Length();

        float32 C = m_length - m_maxLength;

        if (C > 0.0f)
        {
            m_state = e_atUpperLimit;
        }
        else
        {
            m_state = e_inactiveLimit;
        }

        if (m_length > b2_linearSlop)
        {
            m_u *= 1.0f / m_length;
        }
        else
        {
            m_u.SetZero();
            m_mass    = 0.0f;
            m_impulse = 0.0f;
            return;
        }

        // Compute effective mass.
        float32 crA     = b2Cross(m_rA, m_u);
        float32 crB     = b2Cross(m_rB, m_u);
        float32 invMass = m_invMassA + m_invIA * crA * crA + m_invMassB + m_invIB * crB * crB;

        m_mass = invMass != 0.0f ? 1.0f / invMass : 0.0f;

        if (data.step.warmStarting)
        {
            // Scale the impulse to support a variable time step.
            m_impulse *= data.step.dtRatio;

            b2Vec2 P = m_impulse * m_u;
            vA -= m_invMassA * P;
            wA -= m_invIA * b2Cross(m_rA, P);
            vB += m_invMassB * P;
            wB += m_invIB * b2Cross(m_rB, P);
        }
        else
        {
            m_impulse = 0.0f;
        }

        data.velocities[m_indexA].v = vA;
        data.velocities[m_indexA].w = wA;
        data.velocities[m_indexB].v = vB;
        data.velocities[m_indexB].w = wB;
    }

    override void SolveVelocityConstraints(b2SolverData data)
    {
        b2Vec2  vA = data.velocities[m_indexA].v;
        float32 wA = data.velocities[m_indexA].w;
        b2Vec2  vB = data.velocities[m_indexB].v;
        float32 wB = data.velocities[m_indexB].w;

        // Cdot = dot(u, v + cross(w, r))
        b2Vec2  vpA  = vA + b2Cross(wA, m_rA);
        b2Vec2  vpB  = vB + b2Cross(wB, m_rB);
        float32 C    = m_length - m_maxLength;
        float32 Cdot = b2Dot(m_u, vpB - vpA);

        // Predictive constraint.
        if (C < 0.0f)
        {
            Cdot += data.step.inv_dt * C;
        }

        float32 impulse    = -m_mass * Cdot;
        float32 oldImpulse = m_impulse;
        m_impulse = b2Min(0.0f, m_impulse + impulse);
        impulse   = m_impulse - oldImpulse;

        b2Vec2 P = impulse * m_u;
        vA -= m_invMassA * P;
        wA -= m_invIA * b2Cross(m_rA, P);
        vB += m_invMassB * P;
        wB += m_invIB * b2Cross(m_rB, P);

        data.velocities[m_indexA].v = vA;
        data.velocities[m_indexA].w = wA;
        data.velocities[m_indexB].v = vB;
        data.velocities[m_indexB].w = wB;
    }

    override bool SolvePositionConstraints(b2SolverData data)
    {
        b2Vec2  cA = data.positions[m_indexA].c;
        float32 aA = data.positions[m_indexA].a;
        b2Vec2  cB = data.positions[m_indexB].c;
        float32 aB = data.positions[m_indexB].a;

        b2Rot qA = b2Rot(aA);
        b2Rot qB = b2Rot(aB);

        b2Vec2 rA = b2Mul(qA, m_localAnchorA - m_localCenterA);
        b2Vec2 rB = b2Mul(qB, m_localAnchorB - m_localCenterB);
        b2Vec2 u  = cB + rB - cA - rA;

        float32 length = u.Normalize();
        float32 C      = length - m_maxLength;

        C = b2Clamp(C, 0.0f, b2_maxLinearCorrection);

        float32 impulse = -m_mass * C;
        b2Vec2  P       = impulse * u;

        cA -= m_invMassA * P;
        aA -= m_invIA * b2Cross(rA, P);
        cB += m_invMassB * P;
        aB += m_invIB * b2Cross(rB, P);

        data.positions[m_indexA].c = cA;
        data.positions[m_indexA].a = aA;
        data.positions[m_indexB].c = cB;
        data.positions[m_indexB].a = aB;

        return length - m_maxLength < b2_linearSlop;
    }

    // Solver shared
    b2Vec2  m_localAnchorA;
    b2Vec2  m_localAnchorB;
    float32 m_maxLength = 0;
    float32 m_length = 0;
    float32 m_impulse = 0;

    // Solver temp
    int32   m_indexA;
    int32   m_indexB;
    b2Vec2  m_u;
    b2Vec2  m_rA;
    b2Vec2  m_rB;
    b2Vec2  m_localCenterA;
    b2Vec2  m_localCenterB;
    float32 m_invMassA = 0;
    float32 m_invMassB = 0;
    float32 m_invIA = 0;
    float32 m_invIB = 0;
    float32 m_mass = 0;
    b2LimitState m_state;
}
