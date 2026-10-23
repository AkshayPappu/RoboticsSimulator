#include "robotics/Motor.h"
#include "core/PhysicsWorld.h"

#include <chrono/functions/ChFunctionConst.h>
#include <algorithm>
#include <cmath>

namespace robosim {

Motor::~Motor() = default;

chrono::ChFrame<> Motor::BuildFrame(
    const chrono::ChVector3d& anchor,
    const chrono::ChVector3d& axis)
{
    chrono::ChVector3d u, v;
    axis.GetDirectionAxesAsX(u, v);
    chrono::ChMatrix33<> rotMat;
    rotMat.SetFromDirectionAxes(axis, u, v);
    return chrono::ChFrame<>(anchor, rotMat.GetQuaternion());
}

std::shared_ptr<Motor> Motor::CreateSpeedMotor(
    PhysicsWorld& world,
    std::shared_ptr<chrono::ChBody> bodyA,
    std::shared_ptr<chrono::ChBody> bodyB,
    const chrono::ChVector3d& anchor,
    const chrono::ChVector3d& axis)
{
    auto motor = std::shared_ptr<Motor>(new Motor());
    motor->m_mode = MotorMode::Speed;

    auto link = chrono_types::make_shared<chrono::ChLinkMotorRotationSpeed>();
    auto frame = BuildFrame(anchor, axis);
    link->Initialize(bodyA, bodyB, frame);

    auto speedFunc = chrono_types::make_shared<chrono::ChFunctionConst>(0.0);
    link->SetSpeedFunction(speedFunc);

    motor->m_motor = link;
    world.AddLink(link);

    return motor;
}

std::shared_ptr<Motor> Motor::CreateTorqueMotor(
    PhysicsWorld& world,
    std::shared_ptr<chrono::ChBody> bodyA,
    std::shared_ptr<chrono::ChBody> bodyB,
    const chrono::ChVector3d& anchor,
    const chrono::ChVector3d& axis)
{
    auto motor = std::shared_ptr<Motor>(new Motor());
    motor->m_mode = MotorMode::Torque;

    auto link = chrono_types::make_shared<chrono::ChLinkMotorRotationTorque>();
    auto frame = BuildFrame(anchor, axis);
    link->Initialize(bodyA, bodyB, frame);

    auto torqueFunc = chrono_types::make_shared<chrono::ChFunctionConst>(0.0);
    link->SetTorqueFunction(torqueFunc);

    motor->m_motor = link;
    world.AddLink(link);

    return motor;
}

void Motor::SetTargetSpeed(double speed) {
    speed = std::clamp(speed, -m_maxSpeed, m_maxSpeed);
    m_currentTargetSpeed = speed;

    if (m_mode == MotorMode::Speed) {
        auto speedMotor = std::dynamic_pointer_cast<chrono::ChLinkMotorRotationSpeed>(m_motor);
        if (speedMotor) {
            auto func = std::dynamic_pointer_cast<chrono::ChFunctionConst>(
                speedMotor->GetSpeedFunction());
            if (func) {
                func->SetConstant(speed);
            }
        }
    }
}

void Motor::SetTorque(double torque) {
    torque = std::clamp(torque, -m_maxTorque, m_maxTorque);
    m_currentTorque = torque;

    if (m_mode == MotorMode::Torque) {
        auto torqueMotor = std::dynamic_pointer_cast<chrono::ChLinkMotorRotationTorque>(m_motor);
        if (torqueMotor) {
            auto func = std::dynamic_pointer_cast<chrono::ChFunctionConst>(
                torqueMotor->GetTorqueFunction());
            if (func) {
                func->SetConstant(torque);
            }
        }
    }
}

double Motor::GetCurrentAngle() const {
    if (m_motor) {
        return m_motor->GetMotorAngle();
    }
    return 0.0;
}

double Motor::GetCurrentSpeed() const {
    if (m_motor) {
        return m_motor->GetMotorAngleDt();
    }
    return 0.0;
}

double Motor::GetAppliedTorque() const {
    if (m_motor) {
        return m_motor->GetMotorTorque();
    }
    return 0.0;
}

}  // namespace robosim
