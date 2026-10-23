#include "robotics/Robot.h"
#include "core/PhysicsWorld.h"
#include "physics/RigidBody.h"
#include "robotics/Joint.h"
#include "robotics/Motor.h"

#include <cmath>
#include <iostream>

namespace robosim {

Robot::Robot() = default;
Robot::~Robot() = default;

void Robot::Build(PhysicsWorld& world, const RobotConfig& config) {
    m_config = config;

    Material chassisMat;
    chassisMat.friction = 0.4;

    // Build chassis
    BodyTransform chassisTransform;
    chassisTransform.px = config.spawnX;
    chassisTransform.py = config.spawnY;
    chassisTransform.pz = config.spawnZ;

    m_chassis = RigidBody::CreateBox(
        config.chassisSizeX, config.chassisSizeY, config.chassisSizeZ,
        config.chassisMass, chassisTransform, chassisMat
    );
    m_chassis->SetName("chassis");
    world.AddBody(m_chassis);

    auto chassisBody = m_chassis->GetChBody();

    // Build left wheel
    Material wheelMat;
    wheelMat.friction = 0.8;

    BodyTransform leftWheelTransform;
    leftWheelTransform.px = config.spawnX;
    leftWheelTransform.py = config.spawnY + config.wheelOffsetY;
    leftWheelTransform.pz = config.spawnZ + config.wheelOffsetZ;

    auto leftWheel = RigidBody::CreateCylinder(
        config.wheelRadius, config.wheelWidth,
        config.wheelMass, leftWheelTransform, wheelMat
    );
    leftWheel->SetName("left_wheel");
    world.AddBody(leftWheel);
    m_wheels.push_back(leftWheel);

    // Build right wheel
    BodyTransform rightWheelTransform;
    rightWheelTransform.px = config.spawnX;
    rightWheelTransform.py = config.spawnY + config.wheelOffsetY;
    rightWheelTransform.pz = config.spawnZ - config.wheelOffsetZ;

    auto rightWheel = RigidBody::CreateCylinder(
        config.wheelRadius, config.wheelWidth,
        config.wheelMass, rightWheelTransform, wheelMat
    );
    rightWheel->SetName("right_wheel");
    world.AddBody(rightWheel);
    m_wheels.push_back(rightWheel);

    // Build caster (passive sphere at the back)
    Material casterMat;
    casterMat.friction = 0.2;

    BodyTransform casterTransform;
    casterTransform.px = config.spawnX + config.casterOffsetX;
    casterTransform.py = config.spawnY + config.casterOffsetY;
    casterTransform.pz = config.spawnZ;

    auto caster = RigidBody::CreateSphere(
        config.casterRadius, config.casterMass,
        casterTransform, casterMat
    );
    caster->SetName("caster");
    world.AddBody(caster);
    m_casters.push_back(caster);

    // Attach caster to chassis with a fixed joint (acts as a skid)
    auto casterJoint = Joint::CreateFixed(
        world, chassisBody, caster->GetChBody()
    );
    m_joints.push_back(casterJoint);

    // Create motors for left and right wheels
    chrono::ChVector3d leftAnchor(
        config.spawnX,
        config.spawnY + config.wheelOffsetY,
        config.spawnZ + config.wheelOffsetZ
    );
    chrono::ChVector3d rightAnchor(
        config.spawnX,
        config.spawnY + config.wheelOffsetY,
        config.spawnZ - config.wheelOffsetZ
    );
    chrono::ChVector3d wheelAxis(0, 0, 1);  // Z-axis for wheel rotation

    auto leftMotor = Motor::CreateSpeedMotor(
        world, chassisBody, leftWheel->GetChBody(), leftAnchor, wheelAxis
    );
    leftMotor->SetMaxSpeed(config.maxMotorSpeed);
    leftMotor->SetMaxTorque(config.maxTorque);
    m_motors.push_back(leftMotor);

    auto rightMotor = Motor::CreateSpeedMotor(
        world, chassisBody, rightWheel->GetChBody(), rightAnchor, wheelAxis
    );
    rightMotor->SetMaxSpeed(config.maxMotorSpeed);
    rightMotor->SetMaxTorque(config.maxTorque);
    m_motors.push_back(rightMotor);

    std::cout << "[Robot] Built differential-drive robot at ("
              << config.spawnX << ", " << config.spawnY << ", " << config.spawnZ << ")"
              << std::endl;
}

void Robot::BuildFromJson(PhysicsWorld& world, const std::string& jsonPath) {
    // JSON loading will be implemented in Phase 3
    std::cout << "[Robot] JSON loading not yet implemented, using defaults" << std::endl;
    Build(world);
}

void Robot::SetWheelSpeeds(double left, double right) {
    if (m_motors.size() >= 2) {
        m_motors[0]->SetTargetSpeed(left);
        m_motors[1]->SetTargetSpeed(right);
    }
}

void Robot::SetLeftWheelSpeed(double speed) {
    if (m_motors.size() >= 1) {
        m_motors[0]->SetTargetSpeed(speed);
    }
}

void Robot::SetRightWheelSpeed(double speed) {
    if (m_motors.size() >= 2) {
        m_motors[1]->SetTargetSpeed(speed);
    }
}

double Robot::GetLeftWheelSpeed() const {
    if (m_motors.size() >= 1) {
        return m_motors[0]->GetCurrentSpeed();
    }
    return 0.0;
}

double Robot::GetRightWheelSpeed() const {
    if (m_motors.size() >= 2) {
        return m_motors[1]->GetCurrentSpeed();
    }
    return 0.0;
}

void Robot::GetPosition(double& x, double& y, double& z) const {
    if (m_chassis) {
        m_chassis->GetPosition(x, y, z);
    }
}

void Robot::GetOrientation(double& qw, double& qx, double& qy, double& qz) const {
    if (m_chassis) {
        m_chassis->GetRotation(qw, qx, qy, qz);
    }
}

double Robot::GetHeading() const {
    if (!m_chassis) return 0.0;

    double qw, qx, qy, qz;
    m_chassis->GetRotation(qw, qx, qy, qz);

    // Extract yaw from quaternion (rotation about Y axis)
    double siny_cosp = 2.0 * (qw * qy + qx * qz);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qx * qx);
    return std::atan2(siny_cosp, cosy_cosp);
}

void Robot::GetLinearVelocity(double& vx, double& vy, double& vz) const {
    if (m_chassis) {
        m_chassis->GetLinearVelocity(vx, vy, vz);
    }
}

bool Robot::IsFlipped() const {
    if (!m_chassis) return false;

    double qw, qx, qy, qz;
    m_chassis->GetRotation(qw, qx, qy, qz);

    // Check if the up vector (local Y) is pointing downward
    // Transform local Y axis by the quaternion
    double upY = 1.0 - 2.0 * (qx * qx + qz * qz);
    return upY < 0.0;
}

void Robot::Reset(double x, double y, double z) {
    if (m_chassis) {
        m_chassis->SetPosition(x, y, z);
        m_chassis->SetRotation(1, 0, 0, 0);
        m_chassis->SetLinearVelocity(0, 0, 0);
        m_chassis->SetAngularVelocity(0, 0, 0);
    }
    SetWheelSpeeds(0, 0);
}

}  // namespace robosim
