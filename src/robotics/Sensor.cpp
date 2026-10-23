#include "robotics/Sensor.h"

#include <chrono/physics/ChBody.h>
#include <chrono/physics/ChSystem.h>
#include <chrono/collision/ChCollisionSystem.h>
#include <chrono/physics/ChContactContainer.h>

namespace robosim {

DistanceSensor::DistanceSensor(std::shared_ptr<chrono::ChBody> body,
                               const chrono::ChVector3d& localOrigin,
                               const chrono::ChVector3d& localDir,
                               double maxRange)
    : m_body(std::move(body))
    , m_localOrigin(localOrigin)
    , m_localDir(localDir)
    , m_maxRange(maxRange)
    , m_distance(maxRange)
{
    m_name = "distance";
}

void DistanceSensor::Update(chrono::ChSystem& system) {
    m_hit = false;
    m_distance = m_maxRange;
    if (!m_body) return;

    auto coll = system.GetCollisionSystem();
    if (!coll) return;

    chrono::ChVector3d from = m_body->TransformPointLocalToParent(m_localOrigin);
    chrono::ChVector3d dir = m_body->TransformDirectionLocalToParent(m_localDir);
    if (dir.Length() < 1e-8) return;
    dir.Normalize();
    chrono::ChVector3d to = from + dir * m_maxRange;

    chrono::ChCollisionSystem::ChRayhitResult result;
    if (coll->RayHit(from, to, result) && result.hit) {
        m_hit = true;
        m_distance = result.dist_factor * m_maxRange;
    }
}

IMUSensor::IMUSensor(std::shared_ptr<chrono::ChBody> body)
    : m_body(std::move(body))
{
    m_name = "imu";
}

void IMUSensor::Update(chrono::ChSystem& system) {
    if (!m_body) return;
    auto vel = m_body->GetLinVel();
    m_angVel = m_body->GetAngVelParent();
    double dt = system.GetStep();
    if (m_hasPrev && dt > 1e-9) {
        m_linAccel = (vel - m_prevVel) / dt;
    } else {
        m_linAccel = chrono::ChVector3d(0, 0, 0);
    }
    m_prevVel = vel;
    m_hasPrev = true;
}

ContactSensor::ContactSensor(std::shared_ptr<chrono::ChBody> body)
    : m_body(std::move(body))
{
    m_name = "contact";
}

void ContactSensor::Update(chrono::ChSystem& system) {
    m_inContact = false;
    m_contactCount = 0;
    if (!m_body) return;

    auto container = system.GetContactContainer();
    if (!container) return;

    auto force = container->GetContactableForce(m_body.get());
    m_inContact = force.Length() > 1e-6;
    m_contactCount = m_inContact ? 1 : 0;
}

}  // namespace robosim
