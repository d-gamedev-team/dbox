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
module dbox.dynamics.joints.b2frictionjoint;

import core.stdc.float_;
import core.stdc.stdlib;
import core.stdc.string;

import dbox.common;
import dbox.dynamics;
import dbox.dynamics.joints;

/// Friction joint definition.
class b2FrictionJointDef : b2JointDef
{
    ///
    this()
    {
        type = e_frictionJoint;
        localAnchorA.SetZero();
        localAnchorB.SetZero();
        maxForce  = 0.0f;
        maxTorque = 0.0f;
    }

    // Point-to-point constraint
    // Cdot = v2 - v1
    // = v2 + cross(w2, r2) - v1 - cross(w1, r1)
    // J = [-I -r1_skew I r2_skew ]
    // Identity used:
    // w k % (rx i + ry j) = w * (-ry i + rx j)

    // Angle constraint
    // Cdot = w2 - w1
    // J = [0 0 -1 0 0 1]
    // K = invI1 + invI2

    /// Initialize the bodies, anchors, axis, and reference angle using the world
    /// anchor and world axis.
    void Initialize(b2Body* bA, b2Body* bB, b2Vec2 anchor)
    {
        bodyA        = bA;
        bodyB        = bB;
        localAnchorA = bodyA.GetLocalPoint(anchor);
        localAnchorB = bodyB.GetLocalPoint(anchor);
    }

    /// The local anchor point relative to bodyA's origin.
    b2Vec2 localAnchorA;

    /// The local anchor point relative to bodyB's origin.
    b2Vec2 localAnchorB;

    /// The maximum friction force in N.
    float32 maxForce = 0;

    /// The maximum friction torque in N-m.
    float32 maxTorque = 0;
}

/// Friction joint. This is used for top-down friction.
/// It provides 2D translational friction and angular friction.
class b2FrictionJoint : b2Joint
{
    ///
    this(const(b2FrictionJointDef) def)
    {
        super(def);
        m_localAnchorA = def.localAnchorA;
        m_localAnchorB = def.localAnchorB;

        m_linearImpulse.SetZero();
        m_angularImpulse = 0.0f;

        m_maxForce  = def.maxForce;
        m_maxTorque = def.maxTorque;
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
        return inv_dt * m_linearImpulse;
    }

    ///
    override float32 GetReactionTorque(float32 inv_dt) const
    {
        return inv_dt * m_angularImpulse;
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

    /// Get the maximum friction force in N.
    float32 GetMaxForce() const
    {
        return m_maxForce;
    }

    /// Set the maximum friction force in N.
    void SetMaxForce(float32 force)
    {
        assert(b2IsValid(force) && force >= 0.0f);
        m_maxForce = force;
    }

    /// Get the maximum friction torque in N*m.
    float32 GetMaxTorque() const
    {
        return m_maxTorque;
    }

    /// Set the maximum friction torque in N*m.
    void SetMaxTorque(float32 torque)
    {
        assert(b2IsValid(torque) && torque >= 0.0f);
        m_maxTorque = torque;
    }

    /// Dump joint to dmLog
    override void Dump()
    {
        int32 indexA = m_bodyA.m_islandIndex;
        int32 indexB = m_bodyB.m_islandIndex;

        b2Log("  b2FrictionJointDef jd;\n");
        b2Log("  jd.bodyA = bodies[%d];\n", indexA);
        b2Log("  jd.bodyB = bodies[%d];\n", indexB);
        b2Log("  jd.collideConnected = bool(%d);\n", m_collideConnected);
        b2Log("  jd.localAnchorA.Set(%.15lef, %.15lef);\n", m_localAnchorA.x, m_localAnchorA.y);
        b2Log("  jd.localAnchorB.Set(%.15lef, %.15lef);\n", m_localAnchorB.x, m_localAnchorB.y);
        b2Log("  jd.maxForce = %.15lef;\n", m_maxForce);
        b2Log("  jd.maxTorque = %.15lef;\n", m_maxTorque);
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

        float32 aA = data.positions[m_indexA].a;
        b2Vec2  vA = data.velocities[m_indexA].v;
        float32 wA = data.velocities[m_indexA].w;

        float32 aB = data.positions[m_indexB].a;
        b2Vec2  vB = data.velocities[m_indexB].v;
        float32 wB = data.velocities[m_indexB].w;

        b2Rot qA = b2Rot(aA);
        b2Rot qB = b2Rot(aB);

        // Compute the effective mass matrix.
        m_rA = b2Mul(qA, m_localAnchorA - m_localCenterA);
        m_rB = b2Mul(qB, m_localAnchorB - m_localCenterB);

        // J = [-I -r1_skew I r2_skew]
        // [ 0       -1 0       1]
        // r_skew = [-ry; rx]

        // Matlab
        // K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
        // [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
        // [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]

        float32 mA = m_invMassA, mB = m_invMassB;
        float32 iA = m_invIA, iB = m_invIB;

        b2Mat22 K;
        K.ex.x = mA + mB + iA * m_rA.y * m_rA.y + iB * m_rB.y * m_rB.y;
        K.ex.y = -iA * m_rA.x * m_rA.y - iB * m_rB.x * m_rB.y;
        K.ey.x = K.ex.y;
        K.ey.y = mA + mB + iA * m_rA.x * m_rA.x + iB * m_rB.x * m_rB.x;

        m_linearMass = K.GetInverse();

        m_angularMass = iA + iB;

        if (m_angularMass > 0.0f)
        {
            m_angularMass = 1.0f / m_angularMass;
        }

        if (data.step.warmStarting)
        {
            // Scale impulses to support a variable time step.
            m_linearImpulse  *= data.step.dtRatio;
            m_angularImpulse *= data.step.dtRatio;

            b2Vec2 P = b2Vec2(m_linearImpulse.x, m_linearImpulse.y);
            vA -= mA * P;
            wA -= iA * (b2Cross(m_rA, P) + m_angularImpulse);
            vB += mB * P;
            wB += iB * (b2Cross(m_rB, P) + m_angularImpulse);
        }
        else
        {
            m_linearImpulse.SetZero();
            m_angularImpulse = 0.0f;
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

        float32 mA = m_invMassA, mB = m_invMassB;
        float32 iA = m_invIA, iB = m_invIB;

        float32 h = data.step.dt;

        // Solve angular friction
        {
            float32 Cdot    = wB - wA;
            float32 impulse = -m_angularMass * Cdot;

            float32 oldImpulse = m_angularImpulse;
            float32 maxImpulse = h * m_maxTorque;
            m_angularImpulse = b2Clamp(m_angularImpulse + impulse, -maxImpulse, maxImpulse);
            impulse = m_angularImpulse - oldImpulse;

            wA -= iA * impulse;
            wB += iB * impulse;
        }

        // Solve linear friction
        {
            b2Vec2 Cdot = vB + b2Cross(wB, m_rB) - vA - b2Cross(wA, m_rA);

            b2Vec2 impulse    = -b2Mul(m_linearMass, Cdot);
            b2Vec2 oldImpulse = m_linearImpulse;
            m_linearImpulse += impulse;

            float32 maxImpulse = h * m_maxForce;

            if (m_linearImpulse.LengthSquared() > maxImpulse * maxImpulse)
            {
                m_linearImpulse.Normalize();
                m_linearImpulse *= maxImpulse;
            }

            impulse = m_linearImpulse - oldImpulse;

            vA -= mA * impulse;
            wA -= iA * b2Cross(m_rA, impulse);

            vB += mB * impulse;
            wB += iB * b2Cross(m_rB, impulse);
        }

        data.velocities[m_indexA].v = vA;
        data.velocities[m_indexA].w = wA;
        data.velocities[m_indexB].v = vB;
        data.velocities[m_indexB].w = wB;
    }

    override bool SolvePositionConstraints(b2SolverData data)
    {
        B2_NOT_USED(data);

        return true;
    }

    b2Vec2 m_localAnchorA;
    b2Vec2 m_localAnchorB;

    // Solver shared
    b2Vec2  m_linearImpulse;
    float32 m_angularImpulse = 0;
    float32 m_maxForce = 0;
    float32 m_maxTorque = 0;

    // Solver temp
    int32   m_indexA;
    int32   m_indexB;
    b2Vec2  m_rA;
    b2Vec2  m_rB;
    b2Vec2  m_localCenterA;
    b2Vec2  m_localCenterB;
    float32 m_invMassA = 0;
    float32 m_invMassB = 0;
    float32 m_invIA = 0;
    float32 m_invIB = 0;
    b2Mat22 m_linearMass;
    float32 m_angularMass = 0;
}
