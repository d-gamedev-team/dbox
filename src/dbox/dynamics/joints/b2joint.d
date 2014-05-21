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
module dbox.dynamics.joints.b2joint;

import core.stdc.float_;
import core.stdc.stdlib;
import core.stdc.string;

import dbox.common;
import dbox.dynamics;
import dbox.dynamics.joints;

enum b2JointType
{
    e_unknownJoint,
    e_revoluteJoint,
    e_prismaticJoint,
    e_distanceJoint,
    e_pulleyJoint,
    e_mouseJoint,
    e_gearJoint,
    e_wheelJoint,
    e_weldJoint,
    e_frictionJoint,
    e_ropeJoint,
    e_motorJoint
}

mixin _ExportEnumMembers!b2JointType;

enum b2LimitState
{
    e_inactiveLimit,
    e_atLowerLimit,
    e_atUpperLimit,
    e_equalLimits
}

mixin _ExportEnumMembers!b2LimitState;

struct b2Jacobian
{
    b2Vec2 linear;
    float32 angularA = 0;
    float32 angularB = 0;
}

/// A joint edge is used to connect bodies and joints together
/// in a joint graph where each body is a node and each joint
/// is an edge. A joint edge belongs to a doubly linked list
/// maintained in each attached body_. Each joint has two joint
/// nodes, one for each attached body_.
struct b2JointEdge
{
    b2Body* other;      ///< provides quick access to the other body attached.
    b2Joint joint;      ///< the joint
    b2JointEdge* prev;  ///< the previous joint edge in the body's joint list
    b2JointEdge* next;  ///< the next joint edge in the body's joint list
}

/// Joint definitions are used to construct joints.
class b2JointDef
{
    /// The joint type is set automatically for concrete joint types.
    b2JointType type             = b2JointType.e_unknownJoint;

    /// Use this to attach application specific data to your joints.
    void* userData;

    /// The first attached body_.
    b2Body* body_A;

    /// The second attached body_.
    b2Body* body_B;

    /// Set this flag to true if the attached bodies should collide.
    bool collideConnected;
}

/// The base joint class. Joints are used to constraint two bodies together in
/// various fashions. Some joints also feature limits and motors.
class b2Joint
{
    b2JointType GetType() const
    {
        return m_type;
    }

    b2Body* Getbody_A()
    {
        return m_body_A;
    }

    b2Body* Getbody_B()
    {
        return m_body_B;
    }

    b2Joint GetNext()
    {
        return m_next;
    }

    const(b2Joint) GetNext() const
    {
        return m_next;
    }

    void* GetUserData() const
    {
        return cast(void*)m_userData;
    }

    void SetUserData(void* data)
    {
        m_userData = data;
    }

    bool GetCollideConnected() const
    {
        return m_collideConnected;
    }

    static b2Joint Create(const(b2JointDef) def, b2BlockAllocator* allocator)
    {
        b2Joint joint = null;

        switch (def.type)
        {
            case e_distanceJoint:
            {
                void* mem = allocator.Allocate(b2memSizeOf!b2DistanceJoint);
                joint = b2emplace!b2DistanceJoint(mem, cast(const(b2DistanceJointDef))(def));
            }
            break;

            case e_mouseJoint:
            {
                void* mem = allocator.Allocate(b2memSizeOf!b2MouseJoint);
                joint = b2emplace!b2MouseJoint(mem, cast(const(b2MouseJointDef))(def));
            }
            break;

            case e_prismaticJoint:
            {
                void* mem = allocator.Allocate(b2memSizeOf!b2PrismaticJoint);
                joint = b2emplace!b2PrismaticJoint(mem, cast(const(b2PrismaticJointDef))(def));
            }
            break;

            case e_revoluteJoint:
            {
                void* mem = allocator.Allocate(b2memSizeOf!b2RevoluteJoint);
                joint = b2emplace!b2RevoluteJoint(mem, cast(const(b2RevoluteJointDef))(def));
            }
            break;

            case e_pulleyJoint:
            {
                void* mem = allocator.Allocate(b2memSizeOf!b2PulleyJoint);
                joint = b2emplace!b2PulleyJoint(mem, cast(const(b2PulleyJointDef))(def));
            }
            break;

            case e_gearJoint:
            {
                void* mem = allocator.Allocate(b2memSizeOf!b2GearJoint);
                joint = b2emplace!b2GearJoint(mem, cast(const(b2GearJointDef))(def));
            }
            break;

            case e_wheelJoint:
            {
                void* mem = allocator.Allocate(b2memSizeOf!b2WheelJoint);
                joint = b2emplace!b2WheelJoint(mem, cast(const(b2WheelJointDef))(def));
            }
            break;

            case e_weldJoint:
            {
                void* mem = allocator.Allocate(b2memSizeOf!b2WeldJoint);
                joint = b2emplace!b2WeldJoint(mem, cast(const(b2WeldJointDef))(def));
            }
            break;

            case e_frictionJoint:
            {
                void* mem = allocator.Allocate(b2memSizeOf!b2FrictionJoint);
                joint = b2emplace!b2FrictionJoint(mem, cast(const(b2FrictionJointDef))(def));
            }
            break;

            case e_ropeJoint:
            {
                void* mem = allocator.Allocate(b2memSizeOf!b2RopeJoint);
                joint = b2emplace!b2RopeJoint(mem, cast(const(b2RopeJointDef))(def));
            }
            break;

            case e_motorJoint:
            {
                void* mem = allocator.Allocate(b2memSizeOf!b2MotorJoint);
                joint = b2emplace!b2MotorJoint(mem, cast(const(b2MotorJointDef))(def));
            }
            break;

            default:
                assert(0);
        }

        return joint;
    }

    static void Destroy(b2Joint joint, b2BlockAllocator* allocator)
    {
        auto jointType = joint.m_type;
        destroy(joint);

        switch (jointType)
        {
            case e_distanceJoint:
                allocator.Free(cast(void*)joint, b2memSizeOf!b2DistanceJoint);
                break;

            case e_mouseJoint:
                allocator.Free(cast(void*)joint, b2memSizeOf!b2MouseJoint);
                break;

            case e_prismaticJoint:
                allocator.Free(cast(void*)joint, b2memSizeOf!b2PrismaticJoint);
                break;

            case e_revoluteJoint:
                allocator.Free(cast(void*)joint, b2memSizeOf!b2RevoluteJoint);
                break;

            case e_pulleyJoint:
                allocator.Free(cast(void*)joint, b2memSizeOf!b2PulleyJoint);
                break;

            case e_gearJoint:
                allocator.Free(cast(void*)joint, b2memSizeOf!b2GearJoint);
                break;

            case e_wheelJoint:
                allocator.Free(cast(void*)joint, b2memSizeOf!b2WheelJoint);
                break;

            case e_weldJoint:
                allocator.Free(cast(void*)joint, b2memSizeOf!b2WeldJoint);
                break;

            case e_frictionJoint:
                allocator.Free(cast(void*)joint, b2memSizeOf!b2FrictionJoint);
                break;

            case e_ropeJoint:
                allocator.Free(cast(void*)joint, b2memSizeOf!b2RopeJoint);
                break;

            case e_motorJoint:
                allocator.Free(cast(void*)joint, b2memSizeOf!b2MotorJoint);
                break;

            default:
                assert(0);
        }
    }

    this(const(b2JointDef) def)
    {
        assert(def.body_A != def.body_B);

        m_type  = def.type;
        m_prev  = null;
        m_next  = null;
        m_body_A = cast(b2Body*)def.body_A;
        m_body_B = cast(b2Body*)def.body_B;
        m_index = 0;
        m_collideConnected = def.collideConnected;
        m_islandFlag       = false;
        m_userData         = cast(void*)def.userData;

        m_edgeA.joint = null;
        m_edgeA.other = null;
        m_edgeA.prev  = null;
        m_edgeA.next  = null;

        m_edgeB.joint = null;
        m_edgeB.other = null;
        m_edgeB.prev  = null;
        m_edgeB.next  = null;
    }

    bool IsActive() const
    {
        return m_body_A.IsActive() && m_body_B.IsActive();
    }

    /// Get the anchor point on body_A in world coordinates.
    /* virtual */ b2Vec2 GetAnchorA() const;

    /// Get the anchor point on body_B in world coordinates.
    /* virtual */ b2Vec2 GetAnchorB() const;

    /// Get the reaction force on body_B at the joint anchor in Newtons.
    /* virtual */ b2Vec2 GetReactionForce(float32 inv_dt) const;

    /// Get the reaction torque on body_B in N*m.
    /* virtual */ float32 GetReactionTorque(float32 inv_dt) const;

    /// Dump this joint to the log file.
    /* virtual */ void Dump()
    {
        b2Log("// Dump is not supported for this joint type.\n");
    }

    /// Shift the origin for any points stored in world coordinates.
    /* virtual */ void ShiftOrigin(b2Vec2 newOrigin)
    {
        B2_NOT_USED(newOrigin);
    }

    abstract void InitVelocityConstraints(b2SolverData data);
    abstract void SolveVelocityConstraints(b2SolverData data);

    // This returns true if the position errors are within tolerance.
    abstract bool SolvePositionConstraints(b2SolverData data);

    b2JointType m_type;
    b2Joint m_prev;
    b2Joint m_next;
    b2JointEdge m_edgeA;
    b2JointEdge m_edgeB;
    b2Body* m_body_A;
    b2Body* m_body_B;

    int32 m_index;

    bool m_islandFlag;
    bool m_collideConnected;

    void* m_userData;
}

// #endif
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

import dbox.dynamics.joints.b2joint;
import dbox.dynamics.joints.b2distancejoint;
import dbox.dynamics.joints.b2wheeljoint;
import dbox.dynamics.joints.b2mousejoint;
import dbox.dynamics.joints.b2revolutejoint;
import dbox.dynamics.joints.b2prismaticjoint;
import dbox.dynamics.joints.b2pulleyjoint;
import dbox.dynamics.joints.b2gearjoint;
import dbox.dynamics.joints.b2weldjoint;
import dbox.dynamics.joints.b2frictionjoint;
import dbox.dynamics.joints.b2ropejoint;
import dbox.dynamics.joints.b2motorjoint;
import dbox.dynamics.b2body;
import dbox.dynamics.b2world;
import dbox.common.b2blockallocator;


