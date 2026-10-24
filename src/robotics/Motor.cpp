#include "robotics/Motor.h"
#include "core/PhysicsWorld.h"

#include <chrono/functions/ChFunctionConst.h>
#include <chrono/physics/ChLinkMotorLinearForce.h>
#include <chrono/physics/ChLinkMotorLinearPosition.h>
#include <chrono/physics/ChLinkMotorLinearSpeed.h>
#include <chrono/physics/ChLinkMotorRotation.h>
#include <chrono/physics/ChLinkMotorRotationAngle.h>
#include <chrono/utils/ChConstants.h>
#include <algorithm>
#include <cmath>

namespace robosim {

Motor::~Motor() = default;

chrono::ChFrame<> Motor::BuildFrame(
    const chrono::ChVector3d& anchor,
    const chrono::ChVector3d& axis)
{
    // Build a frame where the Z-axis is the motor rotation axis
    chrono::ChQuaterniond rot = chrono::QUNIT;
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

std::shared_ptr<Motor> Motor::CreatePositionMotor(
    PhysicsWorld& world,
    std::shared_ptr<chrono::ChBody> bodyA,
    std::shared_ptr<chrono::ChBody> bodyB,
    const chrono::ChVector3d& anchor,
    const chrono::ChVector3d& axis)
{
    auto motor = std::shared_ptr<Motor>(new Motor());
    motor->m_mode = MotorMode::Position;

    auto link = chrono_types::make_shared<chrono::ChLinkMotorRotationAngle>();
    link->Initialize(bodyA, bodyB, BuildFrame(anchor, axis));
    link->SetAngleFunction(chrono_types::make_shared<chrono::ChFunctionConst>(0.0));

    motor->m_motor = link;
    world.AddLink(link);
    return motor;
}

std::shared_ptr<Motor> Motor::CreateLinearSpeedMotor(
    PhysicsWorld& world,
    std::shared_ptr<chrono::ChBody> bodyA,
    std::shared_ptr<chrono::ChBody> bodyB,
    const chrono::ChVector3d& anchor,
    const chrono::ChVector3d& axis)
{
    auto motor = std::shared_ptr<Motor>(new Motor());
    motor->m_mode = MotorMode::LinearSpeed;

    auto link = chrono_types::make_shared<chrono::ChLinkMotorLinearSpeed>();
    link->Initialize(bodyA, bodyB, BuildFrame(anchor, axis));
    link->SetSpeedFunction(chrono_types::make_shared<chrono::ChFunctionConst>(0.0));

    motor->m_motor = link;
    world.AddLink(link);
    return motor;
}

std::shared_ptr<Motor> Motor::CreateLinearPositionMotor(
    PhysicsWorld& world,
    std::shared_ptr<chrono::ChBody> bodyA,
    std::shared_ptr<chrono::ChBody> bodyB,
    const chrono::ChVector3d& anchor,
    const chrono::ChVector3d& axis)
{
    auto motor = std::shared_ptr<Motor>(new Motor());
    motor->m_mode = MotorMode::LinearPosition;

    auto link = chrono_types::make_shared<chrono::ChLinkMotorLinearPosition>();
    link->Initialize(bodyA, bodyB, BuildFrame(anchor, axis));
    link->SetMotionFunction(chrono_types::make_shared<chrono::ChFunctionConst>(0.0));

    motor->m_motor = link;
    world.AddLink(link);
    return motor;
}

std::shared_ptr<Motor> Motor::CreateLinearForceMotor(
    PhysicsWorld& world,
    std::shared_ptr<chrono::ChBody> bodyA,
    std::shared_ptr<chrono::ChBody> bodyB,
    const chrono::ChVector3d& anchor,
    const chrono::ChVector3d& axis)
{
    auto motor = std::shared_ptr<Motor>(new Motor());
    motor->m_mode = MotorMode::LinearForce;

    auto link = chrono_types::make_shared<chrono::ChLinkMotorLinearForce>();
    link->Initialize(bodyA, bodyB, BuildFrame(anchor, axis));
    link->SetForceFunction(chrono_types::make_shared<chrono::ChFunctionConst>(0.0));

    motor->m_motor = link;
    world.AddLink(link);
    return motor;
}

void Motor::SetTarget(double value) {
    switch (m_mode) {
        case MotorMode::Speed:
        case MotorMode::LinearSpeed:
            SetTargetSpeed(value);
            break;
        case MotorMode::Torque:
        case MotorMode::LinearForce:
            SetTorque(value);
            break;
        case MotorMode::Position: {
            if (auto angleMotor = std::dynamic_pointer_cast<chrono::ChLinkMotorRotationAngle>(m_motor)) {
                if (auto func = std::dynamic_pointer_cast<chrono::ChFunctionConst>(angleMotor->GetAngleFunction())) {
                    func->SetConstant(value);
                }
            }
            break;
        }
        case MotorMode::LinearPosition: {
            if (auto posMotor = std::dynamic_pointer_cast<chrono::ChLinkMotorLinearPosition>(m_motor)) {
                if (auto func = std::dynamic_pointer_cast<chrono::ChFunctionConst>(posMotor->GetMotionFunction())) {
                    func->SetConstant(value);
                }
            }
            break;
        }
    }
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
    } else if (m_mode == MotorMode::LinearSpeed) {
        auto speedMotor = std::dynamic_pointer_cast<chrono::ChLinkMotorLinearSpeed>(m_motor);
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
    } else if (m_mode == MotorMode::LinearForce) {
        auto forceMotor = std::dynamic_pointer_cast<chrono::ChLinkMotorLinearForce>(m_motor);
        if (forceMotor) {
            auto func = std::dynamic_pointer_cast<chrono::ChFunctionConst>(
                forceMotor->GetForceFunction());
            if (func) {
                func->SetConstant(torque);
            }
        }
    }
}

double Motor::GetCurrentAngle() const {
    if (auto rotary = std::dynamic_pointer_cast<chrono::ChLinkMotorRotation>(m_motor)) {
        return rotary->GetMotorAngle();
    }
    if (auto linear = std::dynamic_pointer_cast<chrono::ChLinkMotorLinear>(m_motor)) {
        return linear->GetMotorPos();
    }
    return 0.0;
}

double Motor::GetCurrentSpeed() const {
    if (auto rotary = std::dynamic_pointer_cast<chrono::ChLinkMotorRotation>(m_motor)) {
        return rotary->GetMotorAngleDt();
    }
    if (auto linear = std::dynamic_pointer_cast<chrono::ChLinkMotorLinear>(m_motor)) {
        return linear->GetMotorPosDt();
    }
    return 0.0;
}

double Motor::GetAppliedTorque() const {
    if (auto rotary = std::dynamic_pointer_cast<chrono::ChLinkMotorRotation>(m_motor)) {
        return rotary->GetMotorTorque();
    }
    if (auto linear = std::dynamic_pointer_cast<chrono::ChLinkMotorLinear>(m_motor)) {
        return linear->GetMotorForce();
    }
    return 0.0;
}

}  // namespace robosim
