module dbox.dynamics.b2island;

import core.stdc.float_;
import core.stdc.stdlib;
import core.stdc.string;

import dbox.common;
import dbox.collision;
import dbox.dynamics.contacts;
import dbox.dynamics.joints;

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

// #ifndef B2_ISLAND_H
// #define B2_ISLAND_H

import dbox.common.b2math;
import dbox.dynamics;
import dbox.dynamics.contacts;

/// This is an internal class.
struct b2Island
{
    this(
        int32 bodyCapacity,
        int32 contactCapacity,
        int32 jointCapacity,
        b2StackAllocator* allocator,
        b2ContactListener listener)
    {
        m_bodyCapacity    = bodyCapacity;
        m_contactCapacity = contactCapacity;
        m_jointCapacity   = jointCapacity;
        m_bodyCount       = 0;
        m_contactCount    = 0;
        m_jointCount      = 0;

        m_allocator = allocator;
        m_listener  = listener;

        m_bodies   = cast(b2Body**)m_allocator.Allocate(bodyCapacity * b2memSizeOf!b2Body);
        m_contacts = cast(b2Contact*)m_allocator.Allocate(contactCapacity * b2memSizeOf!b2Contact);
        m_joints   = cast(b2Joint*)m_allocator.Allocate(jointCapacity * b2memSizeOf!b2Joint);

        m_velocities = cast(b2Velocity*)m_allocator.Allocate(m_bodyCapacity * b2memSizeOf!b2Velocity);
        m_positions  = cast(b2Position*)m_allocator.Allocate(m_bodyCapacity * b2memSizeOf!b2Position);
    }

    ~this()
    {
        // Warning: the order should reverse the constructor order.
        m_allocator.Free(cast(void*)m_positions);
        m_allocator.Free(cast(void*)m_velocities);
        m_allocator.Free(cast(void*)m_joints);
        m_allocator.Free(cast(void*)m_contacts);
        m_allocator.Free(cast(void*)m_bodies);
    }

    void Solve(b2Profile* profile, b2TimeStep step, b2Vec2 gravity, bool allowSleep)
    {
        auto timer = b2Timer();

        float32 h = step.dt;

        // Integrate velocities and apply damping. Initialize the body state.
        for (int32 i = 0; i < m_bodyCount; ++i)
        {
            b2Body* b = m_bodies[i];

            b2Vec2  c = b.m_sweep.c;
            float32 a = b.m_sweep.a;
            b2Vec2  v = b.m_linearVelocity;
            float32 w = b.m_angularVelocity;

            // Store positions for continuous collision.
            b.m_sweep.c0 = b.m_sweep.c;
            b.m_sweep.a0 = b.m_sweep.a;

            if (b.m_type == b2_dynamicBody)
            {
                // Integrate velocities.
                v += h * (b.m_gravityScale * gravity + b.m_invMass * b.m_force);
                w += h * b.m_invI * b.m_torque;

                // Apply damping.
                // ODE: dv/dt + c * v = 0
                // Solution: v(t) = v0 * exp(-c * t)
                // Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v * exp(-c * dt)
                // v2 = exp(-c * dt) * v1
                // Pade approximation:
                // v2 = v1 * 1 / (1 + c * dt)
                v *= 1.0f / (1.0f + h * b.m_linearDamping);
                w *= 1.0f / (1.0f + h * b.m_angularDamping);
            }

            m_positions[i].c  = c;
            m_positions[i].a  = a;
            m_velocities[i].v = v;
            m_velocities[i].w = w;
        }

        timer.Reset();

        // Solver data
        b2SolverData solverData;
        solverData.step       = step;
        solverData.positions  = m_positions;
        solverData.velocities = m_velocities;

        // Initialize velocity constraints.
        b2ContactSolverDef contactSolverDef;
        contactSolverDef.step       = step;
        contactSolverDef.contacts   = m_contacts;
        contactSolverDef.count      = m_contactCount;
        contactSolverDef.positions  = m_positions;
        contactSolverDef.velocities = m_velocities;
        contactSolverDef.allocator  = m_allocator;

        b2ContactSolver contactSolver = b2ContactSolver(&contactSolverDef);
        contactSolver.InitializeVelocityConstraints();

        if (step.warmStarting)
        {
            contactSolver.WarmStart();
        }

        for (int32 i = 0; i < m_jointCount; ++i)
        {
            m_joints[i].InitVelocityConstraints(solverData);
        }

        profile.solveInit = timer.GetMilliseconds();

        // Solve velocity constraints
        timer.Reset();

        for (int32 i = 0; i < step.velocityIterations; ++i)
        {
            for (int32 j = 0; j < m_jointCount; ++j)
            {
                m_joints[j].SolveVelocityConstraints(solverData);
            }

            contactSolver.SolveVelocityConstraints();
        }

        // Store impulses for warm starting
        contactSolver.StoreImpulses();
        profile.solveVelocity = timer.GetMilliseconds();

        // Integrate positions
        for (int32 i = 0; i < m_bodyCount; ++i)
        {
            b2Vec2  c = m_positions[i].c;
            float32 a = m_positions[i].a;
            b2Vec2  v = m_velocities[i].v;
            float32 w = m_velocities[i].w;

            // Check for large velocities
            b2Vec2 translation = h * v;

            if (b2Dot(translation, translation) > b2_maxTranslationSquared)
            {
                float32 ratio = b2_maxTranslation / translation.Length();
                v *= ratio;
            }

            float32 rotation = h * w;

            if (rotation * rotation > b2_maxRotationSquared)
            {
                float32 ratio = b2_maxRotation / b2Abs(rotation);
                w *= ratio;
            }

            // Integrate
            c += h * v;
            a += h * w;

            m_positions[i].c  = c;
            m_positions[i].a  = a;
            m_velocities[i].v = v;
            m_velocities[i].w = w;
        }

        // Solve position constraints
        timer.Reset();
        bool positionSolved = false;

        for (int32 i = 0; i < step.positionIterations; ++i)
        {
            bool contactsOkay = contactSolver.SolvePositionConstraints();

            bool jointsOkay = true;

            for (int32 idx = 0; idx < m_jointCount; ++idx)
            {
                bool jointOkay = m_joints[idx].SolvePositionConstraints(solverData);
                jointsOkay = jointsOkay && jointOkay;
            }

            if (contactsOkay && jointsOkay)
            {
                // Exit early if the position errors are small.
                positionSolved = true;
                break;
            }
        }

        // Copy state buffers back to the bodies
        for (int32 i = 0; i < m_bodyCount; ++i)
        {
            b2Body* body_ = m_bodies[i];
            body_.m_sweep.c         = m_positions[i].c;
            body_.m_sweep.a         = m_positions[i].a;
            body_.m_linearVelocity  = m_velocities[i].v;
            body_.m_angularVelocity = m_velocities[i].w;
            body_.SynchronizeTransform();
        }

        profile.solvePosition = timer.GetMilliseconds();

        Report(contactSolver.m_velocityConstraints);

        if (allowSleep)
        {
            float32 minSleepTime = b2_maxFloat;

            const float32 linTolSqr = b2_linearSleepTolerance * b2_linearSleepTolerance;
            const float32 angTolSqr = b2_angularSleepTolerance * b2_angularSleepTolerance;

            for (int32 i = 0; i < m_bodyCount; ++i)
            {
                b2Body* b = m_bodies[i];

                if (b.GetType() == b2_staticBody)
                {
                    continue;
                }

                if ((b.m_flags & b2Body.e_autoSleepFlag) == 0 ||
                    b.m_angularVelocity * b.m_angularVelocity > angTolSqr ||
                    b2Dot(b.m_linearVelocity, b.m_linearVelocity) > linTolSqr)
                {
                    b.m_sleepTime = 0.0f;
                    minSleepTime   = 0.0f;
                }
                else
                {
                    b.m_sleepTime += h;
                    minSleepTime    = b2Min(minSleepTime, b.m_sleepTime);
                }
            }

            if (minSleepTime >= b2_timeToSleep && positionSolved)
            {
                for (int32 i = 0; i < m_bodyCount; ++i)
                {
                    b2Body* b = m_bodies[i];
                    b.SetAwake(false);
                }
            }
        }
    }

    void SolveTOI(b2TimeStep subStep, int32 toiIndexA, int32 toiIndexB)
    {
        assert(toiIndexA < m_bodyCount);
        assert(toiIndexB < m_bodyCount);

        // Initialize the body state.
        for (int32 i = 0; i < m_bodyCount; ++i)
        {
            b2Body* b = m_bodies[i];
            m_positions[i].c  = b.m_sweep.c;
            m_positions[i].a  = b.m_sweep.a;
            m_velocities[i].v = b.m_linearVelocity;
            m_velocities[i].w = b.m_angularVelocity;
        }

        b2ContactSolverDef contactSolverDef;
        contactSolverDef.contacts   = m_contacts;
        contactSolverDef.count      = m_contactCount;
        contactSolverDef.allocator  = m_allocator;
        contactSolverDef.step       = subStep;
        contactSolverDef.positions  = m_positions;
        contactSolverDef.velocities = m_velocities;
        b2ContactSolver contactSolver = b2ContactSolver(&contactSolverDef);

        // Solve position constraints.
        for (int32 i = 0; i < subStep.positionIterations; ++i)
        {
            bool contactsOkay = contactSolver.SolveTOIPositionConstraints(toiIndexA, toiIndexB);

            if (contactsOkay)
            {
                break;
            }
        }

        // Leap of faith to new safe state.
        m_bodies[toiIndexA].m_sweep.c0 = m_positions[toiIndexA].c;
        m_bodies[toiIndexA].m_sweep.a0 = m_positions[toiIndexA].a;
        m_bodies[toiIndexB].m_sweep.c0 = m_positions[toiIndexB].c;
        m_bodies[toiIndexB].m_sweep.a0 = m_positions[toiIndexB].a;

        // No warm starting is needed for TOI events because warm
        // starting impulses were applied in the discrete solver.
        contactSolver.InitializeVelocityConstraints();

        // Solve velocity constraints.
        for (int32 i = 0; i < subStep.velocityIterations; ++i)
        {
            contactSolver.SolveVelocityConstraints();
        }

        // Don't store the TOI contact forces for warm starting
        // because they can be quite large.

        float32 h = subStep.dt;

        // Integrate positions
        for (int32 i = 0; i < m_bodyCount; ++i)
        {
            b2Vec2  c = m_positions[i].c;
            float32 a = m_positions[i].a;
            b2Vec2  v = m_velocities[i].v;
            float32 w = m_velocities[i].w;

            // Check for large velocities
            b2Vec2 translation = h * v;

            if (b2Dot(translation, translation) > b2_maxTranslationSquared)
            {
                float32 ratio = b2_maxTranslation / translation.Length();
                v *= ratio;
            }

            float32 rotation = h * w;

            if (rotation * rotation > b2_maxRotationSquared)
            {
                float32 ratio = b2_maxRotation / b2Abs(rotation);
                w *= ratio;
            }

            // Integrate
            c += h * v;
            a += h * w;

            m_positions[i].c  = c;
            m_positions[i].a  = a;
            m_velocities[i].v = v;
            m_velocities[i].w = w;

            // Sync bodies
            b2Body* body_ = m_bodies[i];
            body_.m_sweep.c         = c;
            body_.m_sweep.a         = a;
            body_.m_linearVelocity  = v;
            body_.m_angularVelocity = w;
            body_.SynchronizeTransform();
        }

        Report(contactSolver.m_velocityConstraints);
    }

    void Report(const(b2ContactVelocityConstraint)* constraints)
    {
        if (m_listener is null)
        {
            return;
        }

        for (int32 i = 0; i < m_contactCount; ++i)
        {
            b2Contact c = m_contacts[i];

            const(b2ContactVelocityConstraint)* vc = constraints + i;

            b2ContactImpulse impulse;
            impulse.count = vc.pointCount;

            for (int32 j = 0; j < vc.pointCount; ++j)
            {
                impulse.normalImpulses[j]  = vc.points[j].normalImpulse;
                impulse.tangentImpulses[j] = vc.points[j].tangentImpulse;
            }

            m_listener.PostSolve(c, &impulse);
        }
    }

    this(int32 bodyCapacity, int32 contactCapacity, int32 jointCapacity,
             b2StackAllocator* allocator, b2ContactListener listener);

    void Clear()
    {
        m_bodyCount    = 0;
        m_contactCount = 0;
        m_jointCount   = 0;
    }

    void Solve(b2Profile* profile, b2TimeStep step, b2Vec2 gravity, bool allowSleep);

    void SolveTOI(b2TimeStep subStep, int32 toiIndexA, int32 toiIndexB);

    void Add(b2Body* body_)
    {
        assert(m_bodyCount < m_bodyCapacity);
        body_.m_islandIndex   = m_bodyCount;
        m_bodies[m_bodyCount] = body_;
        ++m_bodyCount;
    }

    void Add(b2Contact contact)
    {
        assert(m_contactCount < m_contactCapacity);
        m_contacts[m_contactCount++] = contact;
    }

    void Add(b2Joint joint)
    {
        assert(m_jointCount < m_jointCapacity);
        m_joints[m_jointCount++] = joint;
    }

    void Report(const(b2ContactVelocityConstraint)* constraints);

    b2StackAllocator* m_allocator;
    b2ContactListener m_listener;

    b2Body** m_bodies;
    b2Contact* m_contacts;
    b2Joint* m_joints;

    b2Position* m_positions;
    b2Velocity* m_velocities;

    int32 m_bodyCount;
    int32 m_jointCount;
    int32 m_contactCount;

    int32 m_bodyCapacity;
    int32 m_contactCapacity;
    int32 m_jointCapacity;
}

// #endif
/*
 * Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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

import dbox.collision.b2distance;
import dbox.dynamics.b2island;
import dbox.dynamics.b2body;
import dbox.dynamics.b2fixture;
import dbox.dynamics.b2world;
import dbox.dynamics.contacts.b2contact;
import dbox.dynamics.contacts.b2contactsolver;
import dbox.dynamics.joints.b2joint;
import dbox.common.b2stackallocator;
import dbox.common.b2timer;

/*
   Position Correction Notes
   =========================
   I tried the several algorithms for position correction of the 2D revolute joint.
   I looked at these systems:
   - simple pendulum (1m diameter sphere on massless 5m stick) with initial angular velocity of 100 rad/s.
   - suspension bridge with 30 1m long planks of length 1m.
   - multi-link chain with 30 1m long links.

   Here are the algorithms:

   Baumgarte - A fraction of the position error is added to the velocity error. There is no
   separate position solver.

   Pseudo Velocities - After the velocity solver and position integration,
   the position error, Jacobian, and effective mass are recomputed. Then
   the velocity constraints are solved with pseudo velocities and a fraction
   of the position error is added to the pseudo velocity error. The pseudo
   velocities are initialized to zero and there is no warm-starting. After
   the position solver, the pseudo velocities are added to the positions.
   This is also called the First Order World method or the Position LCP method.

   Modified Nonlinear Gauss-Seidel (NGS) - Like Pseudo Velocities except the
   position error is re-computed for each constraint and the positions are updated
   after the constraint is solved. The radius vectors (aka Jacobians) are
   re-computed too (otherwise the algorithm has horrible instability). The pseudo
   velocity states are not needed because they are effectively zero at the beginning
   of each iteration. Since we have the current position error, we allow the
   iterations to terminate early if the error becomes smaller than b2_linearSlop.

   Full NGS or just NGS - Like Modified NGS except the effective mass are re-computed
   each time a constraint is solved.

   Here are the results:
   Baumgarte - this is the cheapest algorithm but it has some stability problems,
   especially with the bridge. The chain links separate easily close to the root
   and they jitter as they struggle to pull together. This is one of the most common
   methods in the field. The big drawback is that the position correction artificially
   affects the momentum, thus leading to instabilities and false bounce. I used a
   bias factor of 0.2. A larger bias factor makes the bridge less stable, a smaller
   factor makes joints and contacts more spongy.

   Pseudo Velocities - the is more stable than the Baumgarte method. The bridge is
   stable. However, joints still separate with large angular velocities. Drag the
   simple pendulum in a circle quickly and the joint will separate. The chain separates
   easily and does not recover. I used a bias factor of 0.2. A larger value lead to
   the bridge collapsing when a heavy cube drops on it.

   Modified NGS - this algorithm is better in some ways than Baumgarte and Pseudo
   Velocities, but in other ways it is worse. The bridge and chain are much more
   stable, but the simple pendulum goes unstable at high angular velocities.

   Full NGS - stable in all tests. The joints display good stiffness. The bridge
   still sags, but this is better than infinite forces.

   Recommendations
   Pseudo Velocities are not really worthwhile because the bridge and chain cannot
   recover from joint separation. In other cases the benefit over Baumgarte is small.

   Modified NGS is not a robust method for the revolute joint due to the violent
   instability seen in the simple pendulum. Perhaps it is viable with other constraint
   types, especially scalar constraints where the effective mass is a scalar.

   This leaves Baumgarte and Full NGS. Baumgarte has small, but manageable instabilities
   and is very fast. I don't think we can escape Baumgarte, especially in highly
   demanding cases where high constraint fidelity is not needed.

   Full NGS is robust and easy on the eyes. I recommend this as an option for
   higher fidelity simulation and certainly for suspension bridges and long chains.
   Full NGS might be a good choice for ragdolls, especially motorized ragdolls where
   joint separation can be problematic. The number of NGS iterations can be reduced
   for better performance without harming robustness much.

   Each joint in a can be handled differently in the position solver. So I recommend
   a system where the user can select the algorithm on a per joint basis. I would
   probably default to the slower Full NGS and let the user select the faster
   Baumgarte method in performance critical scenarios.
 */

/*
   Cache Performance

   The Box2D solvers are dominated by cache misses. Data structures are designed
   to increase the number of cache hits. Much of misses are due to random access
   to body data. The constraint structures are iterated over linearly, which leads
   to few cache misses.

   The bodies are not accessed during iteration. Instead read only data, such as
   the mass values are stored with the constraints. The mutable data are the constraint
   impulses and the bodies velocities/positions. The impulses are held inside the
   constraint structures. The body velocities/positions are held in compact, temporary
   arrays to increase the number of cache hits. Linear and angular velocity are
   stored in a single array since multiple arrays lead to multiple misses.
 */

/*
   2D Rotation

   R = [cos(theta) -sin(theta)]
    [sin(theta) cos(theta) ]

   thetaDot = omega

   Let q1 = cos(theta), q2 = sin(theta).
   R = [q1 -q2]
    [q2  q1]

   q1Dot = -thetaDot * q2
   q2Dot = thetaDot * q1

   q1_new = q1_old - dt * w * q2
   q2_new = q2_old + dt * w * q1
   then normalize.

   This might be faster than computing sin+cos.
   However, we can compute sin+cos of the same angle fast.
 */
