#pragma once

#include "physics/Assets.h"
#include "robotics/AssemblyGraph.h"

#include <memory>
#include <unordered_map>
#include <vector>
#include <string>
#include <nlohmann/json.hpp>

namespace chrono {
class ChBody;
}

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
    double offsetY = 0.0;
    double offsetZ = 0.16;
    bool motor = true;
    VisualSpec visual;
};

struct CasterSpec {
    std::string name = "caster";
    double radius = 0.05;
    double mass = 0.2;
    double offsetX = -0.2;
    double offsetY = 0.0;
    double offsetZ = 0.0;
    VisualSpec visual;
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
    VisualSpec visual;
};

struct ExtraVisual {
    std::string name = "extra";
    double offsetX = 0.0;
    double offsetY = 0.0;
    double offsetZ = 0.0;
    VisualSpec visual;
};

struct RobotConfig {
    std::string name = "robot";

    double chassisSizeX = 0.45;
    double chassisSizeY = 0.08;
    double chassisSizeZ = 0.22;
    double chassisMass = 5.0;

    double wheelRadius = 0.12;
    double wheelWidth = 0.05;
    double wheelMass = 0.4;
    double wheelOffsetY = 0.0;
    double wheelOffsetZ = 0.16;

    double casterRadius = 0.04;
    double casterMass = 0.15;
    double casterOffsetX = -0.16;
    double casterOffsetY = 0.0;

    double maxMotorSpeed = 10.0;
    double maxTorque = 5.0;

    double spawnX = 0.0;
    double spawnY = 0.18;
    double spawnZ = 0.0;

    VisualSpec chassisVisual;

    std::vector<WheelSpec> wheels;
    std::vector<CasterSpec> casters;
    std::vector<LinkSpec> links;
    std::vector<ExtraVisual> extras;
};

class Robot {
public:
    Robot();
    ~Robot();

    Robot(const Robot&) = delete;
    Robot& operator=(const Robot&) = delete;

    void Build(PhysicsWorld& world, const RobotConfig& config = {});
    void BuildFromJson(PhysicsWorld& world, const std::string& jsonPath);
    bool BuildFromGraph(PhysicsWorld& world, const AssemblyGraph& graph, const std::string& previewBodyId = {});
    nlohmann::json ToJson() const;

    const AssemblyGraph* GetAssembly() const { return m_fromGraph ? &m_graph : nullptr; }
    const std::vector<std::string>& GetBuildErrors() const { return m_buildErrors; }
    std::shared_ptr<RigidBody> FindBody(const std::string& id) const;
    bool SetMotorTarget(const std::string& id, double value);

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
    void RemoveFromWorld(PhysicsWorld& world);
    std::vector<std::shared_ptr<chrono::ChBody>> GetChBodies() const;

private:
    void AttachWheelsAndCasters(PhysicsWorld& world);
    void AttachLinks(PhysicsWorld& world);

    RobotConfig m_config;
    bool m_fromGraph = false;
    AssemblyGraph m_graph;
    std::vector<std::string> m_buildErrors;
    std::unordered_map<std::string, std::shared_ptr<RigidBody>> m_bodiesById;
    std::unordered_map<std::string, std::shared_ptr<Motor>> m_motorsById;
    std::vector<int> m_driveGroups;
    std::shared_ptr<RigidBody> m_chassis;
    std::vector<std::shared_ptr<RigidBody>> m_wheels;
    std::vector<std::shared_ptr<RigidBody>> m_casters;
    std::vector<std::shared_ptr<RigidBody>> m_links;
    std::vector<std::shared_ptr<Joint>> m_joints;
    std::vector<std::shared_ptr<Motor>> m_motors;
    std::vector<std::shared_ptr<Sensor>> m_sensors;
};

}  // namespace robosim
