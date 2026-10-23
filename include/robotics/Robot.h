#pragma once

#include <memory>
#include <vector>
#include <string>
#include <nlohmann/json.hpp>

namespace robosim {

class PhysicsWorld;
class RigidBody;
class Joint;
class Motor;
class Sensor;

struct WheelSpec {
    std::string name = "wheel";
    double radius = 0.1;
    double width = 0.05;
    double mass = 0.5;
    double offsetX = 0.0;
    double offsetY = -0.05;
    double offsetZ = 0.2;
    bool motor = true;
};

struct CasterSpec {
    std::string name = "caster";
    double radius = 0.05;
    double mass = 0.2;
    double offsetX = -0.2;
    double offsetY = -0.1;
    double offsetZ = 0.0;
};

struct LinkSpec {
    std::string name = "link";
    int parent = -1;
    std::string jointType = "revolute";
    std::string geometry = "box";
    double sizeX = 0.08;
    double sizeY = 0.08;
    double sizeZ = 0.4;
    double radius = 0.04;
    double mass = 0.8;
    double offsetX = 0.0;
    double offsetY = 0.25;
    double offsetZ = 0.0;
    double axisX = 0.0;
    double axisY = 0.0;
    double axisZ = 1.0;
    bool motor = true;
};

struct RobotConfig {
    std::string name = "robot";

    double chassisSizeX = 0.5;
    double chassisSizeY = 0.15;
    double chassisSizeZ = 0.3;
    double chassisMass = 5.0;

    double wheelRadius = 0.1;
    double wheelWidth = 0.05;
    double wheelMass = 0.5;
    double wheelOffsetY = -0.05;
    double wheelOffsetZ = 0.2;

    double casterRadius = 0.05;
    double casterMass = 0.2;
    double casterOffsetX = -0.2;
    double casterOffsetY = -0.1;

    double maxMotorSpeed = 10.0;
    double maxTorque = 2.0;

    double spawnX = 0.0;
    double spawnY = 0.5;
    double spawnZ = 0.0;

    std::vector<WheelSpec> wheels;
    std::vector<CasterSpec> casters;
    std::vector<LinkSpec> links;
};

class Robot {
public:
    Robot();
    ~Robot();

    Robot(const Robot&) = delete;
    Robot& operator=(const Robot&) = delete;

    void Build(PhysicsWorld& world, const RobotConfig& config = {});
    void BuildFromJson(PhysicsWorld& world, const std::string& jsonPath);
    nlohmann::json ToJson() const;

    void SetWheelSpeeds(double left, double right);
    void SetLeftWheelSpeed(double speed);
    void SetRightWheelSpeed(double speed);
    void SetMotorSpeed(size_t index, double speed);

    double GetLeftWheelSpeed() const;
    double GetRightWheelSpeed() const;

    void GetPosition(double& x, double& y, double& z) const;
    void GetOrientation(double& qw, double& qx, double& qy, double& qz) const;
    double GetHeading() const;

    void GetLinearVelocity(double& vx, double& vy, double& vz) const;

    bool IsFlipped() const;

    std::shared_ptr<RigidBody> GetChassis() const { return m_chassis; }
    const std::vector<std::shared_ptr<Motor>>& GetMotors() const { return m_motors; }
    const std::vector<std::shared_ptr<Sensor>>& GetSensors() const { return m_sensors; }
    const RobotConfig& GetConfig() const { return m_config; }

    void AddDefaultSensors();
    void UpdateSensors(PhysicsWorld& world);

    void Reset(double x, double y, double z);

private:
    void AttachWheelsAndCasters(PhysicsWorld& world);
    void AttachLinks(PhysicsWorld& world);

    RobotConfig m_config;
    std::shared_ptr<RigidBody> m_chassis;
    std::vector<std::shared_ptr<RigidBody>> m_wheels;
    std::vector<std::shared_ptr<RigidBody>> m_casters;
    std::vector<std::shared_ptr<RigidBody>> m_links;
    std::vector<std::shared_ptr<Joint>> m_joints;
    std::vector<std::shared_ptr<Motor>> m_motors;
    std::vector<std::shared_ptr<Sensor>> m_sensors;
};

}  // namespace robosim
