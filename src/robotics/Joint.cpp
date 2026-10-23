#include "robotics/Joint.h"
#include "core/PhysicsWorld.h"

#include <chrono/physics/ChLinkLock.h>
#include <chrono/utils/ChConstants.h>
#include <cmath>
#include <algorithm>

namespace robosim {

Joint::~Joint() = default;

static chrono::ChFrame<> BuildJointFrame(
    const chrono::ChVector3d& anchor,
    const chrono::ChVector3d& axis)
{
    // Build a frame at the anchor point. For revolute joints, the Z-axis
    // of the frame is the rotation axis.
    chrono::ChQuaterniond rot = chrono::QUNIT;
    // If axis is not already Z, compute the rotation from Z to the desired axis
    chrono::ChVector3d zAxis(0, 0, 1);
    chrono::ChVector3d cross = zAxis.Cross(axis);
    double dot = zAxis.Dot(axis);
    if (cross.Length() > 1e-6) {
        double angle = std::acos(std::clamp(dot, -1.0, 1.0));
        cross.Normalize();
        rot = chrono::QuatFromAngleAxis(angle, cross);
    } else if (dot < 0) {
        rot = chrono::QuatFromAngleAxis(chrono::CH_PI, chrono::ChVector3d(1, 0, 0));
    }
    return chrono::ChFrame<>(anchor, rot);
}

std::shared_ptr<Joint> Joint::CreateRevolute(
    PhysicsWorld& world,
    std::shared_ptr<chrono::ChBody> bodyA,
    std::shared_ptr<chrono::ChBody> bodyB,
    const chrono::ChVector3d& anchor,
    const chrono::ChVector3d& axis)
{
    auto joint = std::shared_ptr<Joint>(new Joint());
    joint->m_type = JointType::Revolute;

    auto link = chrono_types::make_shared<chrono::ChLinkLockRevolute>();
    auto frame = BuildJointFrame(anchor, axis);
    link->Initialize(bodyA, bodyB, frame);

    joint->m_link = link;
    world.AddLink(link);

    return joint;
}

std::shared_ptr<Joint> Joint::CreatePrismatic(
    PhysicsWorld& world,
    std::shared_ptr<chrono::ChBody> bodyA,
    std::shared_ptr<chrono::ChBody> bodyB,
    const chrono::ChVector3d& anchor,
    const chrono::ChVector3d& axis)
{
    auto joint = std::shared_ptr<Joint>(new Joint());
    joint->m_type = JointType::Prismatic;

    auto link = chrono_types::make_shared<chrono::ChLinkLockPrismatic>();
    auto frame = BuildJointFrame(anchor, axis);
    link->Initialize(bodyA, bodyB, frame);

    joint->m_link = link;
    world.AddLink(link);

    return joint;
}

std::shared_ptr<Joint> Joint::CreateFixed(
    PhysicsWorld& world,
    std::shared_ptr<chrono::ChBody> bodyA,
    std::shared_ptr<chrono::ChBody> bodyB)
{
    auto joint = std::shared_ptr<Joint>(new Joint());
    joint->m_type = JointType::Fixed;

    auto link = chrono_types::make_shared<chrono::ChLinkLockLock>();
    link->Initialize(bodyA, bodyB, chrono::ChFrame<>(bodyA->GetPos(), bodyA->GetRot()));

    joint->m_link = link;
    world.AddLink(link);

    return joint;
}

void Joint::SetLimits(double lower, double upper) {
    // Joint limits can be set through ChLinkLimit on specific DOFs
    // In Chrono 10, access is via the link's limit members
    // Simplified: skip limits for now
}

}  // namespace robosim
