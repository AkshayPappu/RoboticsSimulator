#pragma once

#include <memory>
#include <vector>
#include <string>

namespace robosim {

class PhysicsWorld;
class RigidBody;
class Joint;
class Motor;

struct RobotConfig {
    // Chassis
    double chassisSizeX = 0.5;
    double chassisSizeY = 0.15;
    double chassisSizeZ = 0.3;
    double chassisMass = 5.0;

    // Wheels
    double wheelRadius = 0.1;
    double wheelWidth = 0.05;
    double wheelMass = 0.5;
    double wheelOffsetY = -0.05;
    double wheelOffsetZ = 0.2;

    // Caster
    double casterRadius = 0.05;
    double casterMass = 0.2;
    double casterOffsetX = -0.2;
    double casterOffsetY = -0.1;

    // Motor limits
    double maxMotorSpeed = 10.0;
    double maxTorque = 2.0;

    // Spawn
    double spawnX = 0.0;
    double spawnY = 0.5;
    double spawnZ = 0.0;
};

class Robot {
public:
    Robot();
    ~Robot();

    Robot(const Robot&) = delete;
    Robot& operator=(const Robot&) = delete;

    void Build(PhysicsWorld& world, const RobotConfig& config = {});
    void BuildFromJson(PhysicsWorld& world, const std::string& jsonPath);

    void SetWheelSpeeds(double left, double right);
    void SetLeftWheelSpeed(double speed);
    void SetRightWheelSpeed(double speed);

    double GetLeftWheelSpeed() const;
    double GetRightWheelSpeed() const;

    void GetPosition(double& x, double& y, double& z) const;
    void GetOrientation(double& qw, double& qx, double& qy, double& qz) const;
    double GetHeading() const;

    void GetLinearVelocity(double& vx, double& vy, double& vz) const;

    bool IsFlipped() const;

    std::shared_ptr<RigidBody> GetChassis() const { return m_chassis; }
    const std::vector<std::shared_ptr<Motor>>& GetMotors() const { return m_motors; }
    const RobotConfig& GetConfig() const { return m_config; }

    void Reset(double x, double y, double z);

private:
    RobotConfig m_config;
    std::shared_ptr<RigidBody> m_chassis;
    std::vector<std::shared_ptr<RigidBody>> m_wheels;
    std::vector<std::shared_ptr<RigidBody>> m_casters;
    std::vector<std::shared_ptr<Joint>> m_joints;
    std::vector<std::shared_ptr<Motor>> m_motors;
};

}  // namespace robosim
