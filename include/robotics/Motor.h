#pragma once

#include <memory>
#include <chrono/physics/ChBody.h>
#include <chrono/physics/ChLinkMotor.h>
#include <chrono/physics/ChLinkMotorRotationSpeed.h>
#include <chrono/physics/ChLinkMotorRotationTorque.h>

namespace robosim {

class PhysicsWorld;

enum class MotorMode {
    Speed,
    Torque,
    Position,
    LinearSpeed,
    LinearPosition,
    LinearForce
};

class Motor {
public:
    ~Motor();

    static std::shared_ptr<Motor> CreateSpeedMotor(
        PhysicsWorld& world,
        std::shared_ptr<chrono::ChBody> bodyA,
        std::shared_ptr<chrono::ChBody> bodyB,
        const chrono::ChVector3d& anchor,
        const chrono::ChVector3d& axis
    );

    static std::shared_ptr<Motor> CreateTorqueMotor(
        PhysicsWorld& world,
        std::shared_ptr<chrono::ChBody> bodyA,
        std::shared_ptr<chrono::ChBody> bodyB,
        const chrono::ChVector3d& anchor,
        const chrono::ChVector3d& axis
    );

    static std::shared_ptr<Motor> CreatePositionMotor(
        PhysicsWorld& world,
        std::shared_ptr<chrono::ChBody> bodyA,
        std::shared_ptr<chrono::ChBody> bodyB,
        const chrono::ChVector3d& anchor,
        const chrono::ChVector3d& axis
    );

    static std::shared_ptr<Motor> CreateLinearSpeedMotor(
        PhysicsWorld& world,
        std::shared_ptr<chrono::ChBody> bodyA,
        std::shared_ptr<chrono::ChBody> bodyB,
        const chrono::ChVector3d& anchor,
        const chrono::ChVector3d& axis
    );

    static std::shared_ptr<Motor> CreateLinearPositionMotor(
        PhysicsWorld& world,
        std::shared_ptr<chrono::ChBody> bodyA,
        std::shared_ptr<chrono::ChBody> bodyB,
        const chrono::ChVector3d& anchor,
        const chrono::ChVector3d& axis
    );

    static std::shared_ptr<Motor> CreateLinearForceMotor(
        PhysicsWorld& world,
        std::shared_ptr<chrono::ChBody> bodyA,
        std::shared_ptr<chrono::ChBody> bodyB,
        const chrono::ChVector3d& anchor,
        const chrono::ChVector3d& axis
    );

    void SetTarget(double value);
    void SetTargetSpeed(double speed);
    void SetTorque(double torque);

    double GetCurrentAngle() const;
    double GetCurrentSpeed() const;
    double GetAppliedTorque() const;

    MotorMode GetMode() const { return m_mode; }
    std::shared_ptr<chrono::ChLinkMotor> GetChLink() const { return m_motor; }

    double GetMaxSpeed() const { return m_maxSpeed; }
    void SetMaxSpeed(double maxSpeed) { m_maxSpeed = maxSpeed; }

    double GetMaxTorque() const { return m_maxTorque; }
    void SetMaxTorque(double maxTorque) { m_maxTorque = maxTorque; }

private:
    Motor() = default;

    static chrono::ChFrame<> BuildFrame(
        const chrono::ChVector3d& anchor,
        const chrono::ChVector3d& axis
    );

    std::shared_ptr<chrono::ChLinkMotor> m_motor;
    MotorMode m_mode;
    double m_maxSpeed = 10.0;
    double m_maxTorque = 50.0;
    double m_currentTargetSpeed = 0.0;
    double m_currentTorque = 0.0;
};

}  // namespace robosim
