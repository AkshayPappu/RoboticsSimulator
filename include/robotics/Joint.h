#pragma once

#include <memory>
#include <chrono/physics/ChBody.h>
#include <chrono/physics/ChLinkLock.h>

namespace robosim {

class PhysicsWorld;

enum class JointType {
    Revolute,
    Prismatic,
    Fixed,
    Spherical
};

class Joint {
public:
    ~Joint();

    static std::shared_ptr<Joint> CreateRevolute(
        PhysicsWorld& world,
        std::shared_ptr<chrono::ChBody> bodyA,
        std::shared_ptr<chrono::ChBody> bodyB,
        const chrono::ChVector3d& anchor,
        const chrono::ChVector3d& axis
    );

    static std::shared_ptr<Joint> CreatePrismatic(
        PhysicsWorld& world,
        std::shared_ptr<chrono::ChBody> bodyA,
        std::shared_ptr<chrono::ChBody> bodyB,
        const chrono::ChVector3d& anchor,
        const chrono::ChVector3d& axis
    );

    static std::shared_ptr<Joint> CreateFixed(
        PhysicsWorld& world,
        std::shared_ptr<chrono::ChBody> bodyA,
        std::shared_ptr<chrono::ChBody> bodyB
    );

    static std::shared_ptr<Joint> CreateSpherical(
        PhysicsWorld& world,
        std::shared_ptr<chrono::ChBody> bodyA,
        std::shared_ptr<chrono::ChBody> bodyB,
        const chrono::ChVector3d& anchor
    );

    JointType GetType() const { return m_type; }
    std::shared_ptr<chrono::ChLinkLock> GetChLink() const { return m_link; }

    void SetLimits(double lower, double upper);

private:
    Joint() = default;

    std::shared_ptr<chrono::ChLinkLock> m_link;
    JointType m_type;
};

}  // namespace robosim
