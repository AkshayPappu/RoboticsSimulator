#include "robotics/Joint.h"
#include "core/PhysicsWorld.h"

#include <chrono/physics/ChLinkLock.h>

namespace robosim {

Joint::~Joint() = default;

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

    // Build a coordinate system at the anchor with Z along the given axis
    chrono::ChVector3d u, v;
    axis.GetDirectionAxesAsX(u, v);
    chrono::ChMatrix33<> rotMat;
    rotMat.SetFromDirectionAxes(axis, u, v);
    chrono::ChFrame<> frame(anchor, rotMat.GetQuaternion());

    link->Initialize(bodyA, bodyB, chrono::ChFrame<>(anchor, rotMat.GetQuaternion()));

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

    chrono::ChVector3d u, v;
    axis.GetDirectionAxesAsX(u, v);
    chrono::ChMatrix33<> rotMat;
    rotMat.SetFromDirectionAxes(axis, u, v);

    link->Initialize(bodyA, bodyB, chrono::ChFrame<>(anchor, rotMat.GetQuaternion()));

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
    if (m_type == JointType::Revolute) {
        auto limit = m_link->GetLimit_Rz();
        if (limit) {
            limit->SetActive(true);
            limit->SetMin(lower);
            limit->SetMax(upper);
        }
    }
}

}  // namespace robosim
