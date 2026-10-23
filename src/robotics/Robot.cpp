#include "robotics/Robot.h"
#include "core/PhysicsWorld.h"
#include "physics/RigidBody.h"
#include "robotics/Joint.h"
#include "robotics/Motor.h"
#include "robotics/Sensor.h"

#include <nlohmann/json.hpp>
#include <fstream>
#include <cmath>
#include <iostream>

using json = nlohmann::json;

namespace robosim {

Robot::Robot() = default;
Robot::~Robot() = default;

static RobotConfig DefaultTwoWheelConfig(const RobotConfig& base) {
    RobotConfig cfg = base;
    if (cfg.wheels.empty()) {
        WheelSpec left;
        left.name = "left_wheel";
        left.radius = cfg.wheelRadius;
        left.width = cfg.wheelWidth;
        left.mass = cfg.wheelMass;
        left.offsetY = cfg.wheelOffsetY;
        left.offsetZ = cfg.wheelOffsetZ;
        WheelSpec right = left;
        right.name = "right_wheel";
        right.offsetZ = -cfg.wheelOffsetZ;
        cfg.wheels.push_back(left);
        cfg.wheels.push_back(right);
    }
    if (cfg.casters.empty() && cfg.casterRadius > 0.0) {
        CasterSpec caster;
        caster.radius = cfg.casterRadius;
        caster.mass = cfg.casterMass;
        caster.offsetX = cfg.casterOffsetX;
        caster.offsetY = cfg.casterOffsetY;
        cfg.casters.push_back(caster);
    }
    return cfg;
}

void Robot::AttachWheelsAndCasters(PhysicsWorld& world) {
    auto chassisBody = m_chassis->GetChBody();

    Material wheelMat;
    wheelMat.friction = 0.8;

    for (const auto& spec : m_config.wheels) {
        BodyTransform t;
        t.px = m_config.spawnX + spec.offsetX;
        t.py = m_config.spawnY + spec.offsetY;
        t.pz = m_config.spawnZ + spec.offsetZ;

        auto wheel = RigidBody::CreateCylinder(spec.radius, spec.width, spec.mass, t, wheelMat);
        wheel->SetName(spec.name);
        wheel->SetColor(0.12f, 0.12f, 0.12f);
        world.AddBody(wheel);
        m_wheels.push_back(wheel);

        if (spec.motor) {
            chrono::ChVector3d anchor(t.px, t.py, t.pz);
            chrono::ChVector3d axis(0, 0, 1);
            auto motor = Motor::CreateSpeedMotor(world, chassisBody, wheel->GetChBody(), anchor, axis);
            motor->SetMaxSpeed(m_config.maxMotorSpeed);
            motor->SetMaxTorque(m_config.maxTorque);
            m_motors.push_back(motor);
        } else {
            chrono::ChVector3d anchor(t.px, t.py, t.pz);
            auto joint = Joint::CreateRevolute(world, chassisBody, wheel->GetChBody(),
                                               anchor, chrono::ChVector3d(0, 0, 1));
            m_joints.push_back(joint);
        }
    }

    Material casterMat;
    casterMat.friction = 0.2;
    for (const auto& spec : m_config.casters) {
        BodyTransform t;
        t.px = m_config.spawnX + spec.offsetX;
        t.py = m_config.spawnY + spec.offsetY;
        t.pz = m_config.spawnZ + spec.offsetZ;

        auto caster = RigidBody::CreateSphere(spec.radius, spec.mass, t, casterMat);
        caster->SetName(spec.name);
        caster->SetColor(0.55f, 0.55f, 0.58f);
        world.AddBody(caster);
        m_casters.push_back(caster);

        auto joint = Joint::CreateFixed(world, chassisBody, caster->GetChBody());
        m_joints.push_back(joint);
    }
}

void Robot::AttachLinks(PhysicsWorld& world) {
    Material linkMat;
    linkMat.friction = 0.4;

    std::vector<std::shared_ptr<RigidBody>> bodies;
    bodies.push_back(m_chassis);

    for (const auto& spec : m_config.links) {
        int parentIdx = spec.parent + 1;  // -1 means chassis -> index 0
        if (parentIdx < 0 || parentIdx >= (int)bodies.size()) {
            parentIdx = 0;
        }
        auto parent = bodies[parentIdx];

        double px, py, pz;
        parent->GetPosition(px, py, pz);

        BodyTransform t;
        t.px = px + spec.offsetX;
        t.py = py + spec.offsetY;
        t.pz = pz + spec.offsetZ;

        std::shared_ptr<RigidBody> linkBody;
        if (spec.geometry == "cylinder") {
            linkBody = RigidBody::CreateCylinder(spec.radius, spec.sizeY, spec.mass, t, linkMat);
        } else if (spec.geometry == "sphere") {
            linkBody = RigidBody::CreateSphere(spec.radius, spec.mass, t, linkMat);
        } else {
            linkBody = RigidBody::CreateBox(spec.sizeX, spec.sizeY, spec.sizeZ, spec.mass, t, linkMat);
        }
        linkBody->SetName(spec.name);
        linkBody->SetColor(0.75f, 0.70f, 0.25f);
        world.AddBody(linkBody);
        m_links.push_back(linkBody);
        bodies.push_back(linkBody);

        chrono::ChVector3d anchor(t.px, t.py, t.pz);
        chrono::ChVector3d axis(spec.axisX, spec.axisY, spec.axisZ);
        if (axis.Length() < 1e-6) axis = chrono::ChVector3d(0, 0, 1);

        if (spec.motor && spec.jointType == "revolute") {
            auto motor = Motor::CreateSpeedMotor(
                world, parent->GetChBody(), linkBody->GetChBody(), anchor, axis);
            motor->SetMaxSpeed(m_config.maxMotorSpeed);
            motor->SetMaxTorque(m_config.maxTorque);
            m_motors.push_back(motor);
        } else if (spec.jointType == "prismatic") {
            auto joint = Joint::CreatePrismatic(
                world, parent->GetChBody(), linkBody->GetChBody(), anchor, axis);
            m_joints.push_back(joint);
        } else if (spec.jointType == "fixed") {
            auto joint = Joint::CreateFixed(world, parent->GetChBody(), linkBody->GetChBody());
            m_joints.push_back(joint);
        } else {
            auto joint = Joint::CreateRevolute(
                world, parent->GetChBody(), linkBody->GetChBody(), anchor, axis);
            m_joints.push_back(joint);
        }
    }
}

void Robot::Build(PhysicsWorld& world, const RobotConfig& config) {
    m_config = DefaultTwoWheelConfig(config);
    m_wheels.clear();
    m_casters.clear();
    m_links.clear();
    m_joints.clear();
    m_motors.clear();
    m_sensors.clear();

    Material chassisMat;
    chassisMat.friction = 0.4;

    BodyTransform chassisTransform;
    chassisTransform.px = m_config.spawnX;
    chassisTransform.py = m_config.spawnY;
    chassisTransform.pz = m_config.spawnZ;

    m_chassis = RigidBody::CreateBox(
        m_config.chassisSizeX, m_config.chassisSizeY, m_config.chassisSizeZ,
        m_config.chassisMass, chassisTransform, chassisMat
    );
    m_chassis->SetName(m_config.name.empty() ? "chassis" : m_config.name);
    m_chassis->SetColor(0.20f, 0.45f, 0.85f);
    world.AddBody(m_chassis);

    AttachWheelsAndCasters(world);
    AttachLinks(world);
    AddDefaultSensors();

    std::cout << "[Robot] Built '" << m_config.name << "' at ("
              << m_config.spawnX << ", " << m_config.spawnY << ", " << m_config.spawnZ
              << ") wheels=" << m_wheels.size()
              << " motors=" << m_motors.size()
              << " links=" << m_links.size() << std::endl;
}

void Robot::BuildFromJson(PhysicsWorld& world, const std::string& jsonPath) {
    std::ifstream file(jsonPath);
    if (!file.is_open()) {
        std::cerr << "[Robot] Failed to open " << jsonPath << ", using defaults" << std::endl;
        Build(world);
        return;
    }

    json j;
    try {
        file >> j;
    } catch (const json::exception& e) {
        std::cerr << "[Robot] JSON parse error: " << e.what() << std::endl;
        Build(world);
        return;
    }

    RobotConfig cfg;
    cfg.name = j.value("name", std::string("robot"));
    cfg.maxMotorSpeed = j.value("max_motor_speed", 10.0);
    cfg.maxTorque = j.value("max_torque", 2.0);
    cfg.spawnX = j.value("spawn_x", cfg.spawnX);
    cfg.spawnY = j.value("spawn_y", cfg.spawnY);
    cfg.spawnZ = j.value("spawn_z", cfg.spawnZ);

    if (j.contains("chassis")) {
        auto& c = j["chassis"];
        if (c.contains("size") && c["size"].is_array() && c["size"].size() >= 3) {
            cfg.chassisSizeX = c["size"][0].get<double>();
            cfg.chassisSizeY = c["size"][1].get<double>();
            cfg.chassisSizeZ = c["size"][2].get<double>();
        }
        cfg.chassisMass = c.value("mass", cfg.chassisMass);
    }

    if (j.contains("wheels") && j["wheels"].is_array()) {
        for (const auto& w : j["wheels"]) {
            WheelSpec spec;
            spec.name = w.value("side", std::string("wheel"));
            spec.radius = w.value("radius", 0.1);
            spec.width = w.value("width", 0.05);
            spec.mass = w.value("mass", 0.5);
            spec.motor = w.value("motor", true);
            if (w.contains("offset") && w["offset"].is_array() && w["offset"].size() >= 3) {
                spec.offsetX = w["offset"][0].get<double>();
                spec.offsetY = w["offset"][1].get<double>();
                spec.offsetZ = w["offset"][2].get<double>();
            }
            cfg.wheels.push_back(spec);
        }
    }

    if (j.contains("casters") && j["casters"].is_array()) {
        for (const auto& c : j["casters"]) {
            CasterSpec spec;
            spec.radius = c.value("radius", 0.05);
            spec.mass = c.value("mass", 0.2);
            if (c.contains("offset") && c["offset"].is_array() && c["offset"].size() >= 3) {
                spec.offsetX = c["offset"][0].get<double>();
                spec.offsetY = c["offset"][1].get<double>();
                spec.offsetZ = c["offset"][2].get<double>();
            }
            cfg.casters.push_back(spec);
        }
    } else {
        cfg.casterRadius = 0.0;  // don't auto-add caster if JSON omitted it
    }

    if (j.contains("links") && j["links"].is_array()) {
        for (const auto& l : j["links"]) {
            LinkSpec spec;
            spec.name = l.value("name", std::string("link"));
            spec.parent = l.value("parent", -1);
            spec.jointType = l.value("joint", std::string("revolute"));
            spec.geometry = l.value("geometry", std::string("box"));
            spec.mass = l.value("mass", 0.8);
            spec.motor = l.value("motor", true);
            if (l.contains("size") && l["size"].is_array()) {
                if (l["size"].size() > 0) spec.sizeX = l["size"][0].get<double>();
                if (l["size"].size() > 1) spec.sizeY = l["size"][1].get<double>();
                if (l["size"].size() > 2) spec.sizeZ = l["size"][2].get<double>();
            }
            spec.radius = l.value("radius", spec.radius);
            if (l.contains("offset") && l["offset"].is_array() && l["offset"].size() >= 3) {
                spec.offsetX = l["offset"][0].get<double>();
                spec.offsetY = l["offset"][1].get<double>();
                spec.offsetZ = l["offset"][2].get<double>();
            }
            if (l.contains("axis") && l["axis"].is_array() && l["axis"].size() >= 3) {
                spec.axisX = l["axis"][0].get<double>();
                spec.axisY = l["axis"][1].get<double>();
                spec.axisZ = l["axis"][2].get<double>();
            }
            cfg.links.push_back(spec);
        }
    }

    if (!cfg.wheels.empty()) {
        cfg.wheelRadius = cfg.wheels[0].radius;
        cfg.wheelWidth = cfg.wheels[0].width;
        cfg.wheelMass = cfg.wheels[0].mass;
        cfg.wheelOffsetY = cfg.wheels[0].offsetY;
        cfg.wheelOffsetZ = std::abs(cfg.wheels[0].offsetZ);
    }

    Build(world, cfg);
}

nlohmann::json Robot::ToJson() const {
    json j;
    j["name"] = m_config.name;
    j["chassis"] = {
        {"type", "box"},
        {"size", {m_config.chassisSizeX, m_config.chassisSizeY, m_config.chassisSizeZ}},
        {"mass", m_config.chassisMass}
    };
    j["max_motor_speed"] = m_config.maxMotorSpeed;
    j["max_torque"] = m_config.maxTorque;
    j["wheels"] = json::array();
    for (const auto& w : m_config.wheels) {
        j["wheels"].push_back({
            {"side", w.name},
            {"radius", w.radius},
            {"width", w.width},
            {"mass", w.mass},
            {"offset", {w.offsetX, w.offsetY, w.offsetZ}},
            {"motor", w.motor}
        });
    }
    j["casters"] = json::array();
    for (const auto& c : m_config.casters) {
        j["casters"].push_back({
            {"radius", c.radius},
            {"mass", c.mass},
            {"offset", {c.offsetX, c.offsetY, c.offsetZ}}
        });
    }
    j["links"] = json::array();
    for (const auto& l : m_config.links) {
        j["links"].push_back({
            {"name", l.name},
            {"parent", l.parent},
            {"joint", l.jointType},
            {"geometry", l.geometry},
            {"size", {l.sizeX, l.sizeY, l.sizeZ}},
            {"mass", l.mass},
            {"offset", {l.offsetX, l.offsetY, l.offsetZ}},
            {"axis", {l.axisX, l.axisY, l.axisZ}},
            {"motor", l.motor}
        });
    }
    return j;
}

void Robot::AddDefaultSensors() {
    if (!m_chassis) return;
    auto body = m_chassis->GetChBody();

    auto dist = std::make_shared<DistanceSensor>(
        body, chrono::ChVector3d(m_config.chassisSizeX * 0.5, 0, 0),
        chrono::ChVector3d(1, 0, 0), 8.0);
    dist->SetName("front_lidar");
    m_sensors.push_back(dist);

    auto imu = std::make_shared<IMUSensor>(body);
    imu->SetName("imu");
    m_sensors.push_back(imu);

    auto contact = std::make_shared<ContactSensor>(body);
    contact->SetName("chassis_contact");
    m_sensors.push_back(contact);
}

void Robot::UpdateSensors(PhysicsWorld& world) {
    for (auto& sensor : m_sensors) {
        sensor->Update(world.GetChSystem());
    }
}

void Robot::SetWheelSpeeds(double left, double right) {
    if (m_motors.size() >= 2) {
        m_motors[0]->SetTargetSpeed(left);
        m_motors[1]->SetTargetSpeed(right);
    }
    // 4-wheel: also drive rear pair if present
    if (m_motors.size() >= 4) {
        m_motors[2]->SetTargetSpeed(left);
        m_motors[3]->SetTargetSpeed(right);
    }
}

void Robot::SetLeftWheelSpeed(double speed) {
    if (!m_motors.empty()) m_motors[0]->SetTargetSpeed(speed);
}

void Robot::SetRightWheelSpeed(double speed) {
    if (m_motors.size() >= 2) m_motors[1]->SetTargetSpeed(speed);
}

void Robot::SetMotorSpeed(size_t index, double speed) {
    if (index < m_motors.size()) m_motors[index]->SetTargetSpeed(speed);
}

double Robot::GetLeftWheelSpeed() const {
    return m_motors.empty() ? 0.0 : m_motors[0]->GetCurrentSpeed();
}

double Robot::GetRightWheelSpeed() const {
    return m_motors.size() < 2 ? 0.0 : m_motors[1]->GetCurrentSpeed();
}

void Robot::GetPosition(double& x, double& y, double& z) const {
    if (m_chassis) m_chassis->GetPosition(x, y, z);
}

void Robot::GetOrientation(double& qw, double& qx, double& qy, double& qz) const {
    if (m_chassis) m_chassis->GetRotation(qw, qx, qy, qz);
}

double Robot::GetHeading() const {
    if (!m_chassis) return 0.0;
    double qw, qx, qy, qz;
    m_chassis->GetRotation(qw, qx, qy, qz);
    double siny_cosp = 2.0 * (qw * qy + qx * qz);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qx * qx);
    return std::atan2(siny_cosp, cosy_cosp);
}

void Robot::GetLinearVelocity(double& vx, double& vy, double& vz) const {
    if (m_chassis) m_chassis->GetLinearVelocity(vx, vy, vz);
}

bool Robot::IsFlipped() const {
    if (!m_chassis) return false;
    double qw, qx, qy, qz;
    m_chassis->GetRotation(qw, qx, qy, qz);
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
