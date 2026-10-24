#include "robotics/Robot.h"
#include "core/PhysicsWorld.h"
#include "physics/RigidBody.h"
#include "physics/Assets.h"
#include "robotics/Joint.h"
#include "robotics/Motor.h"
#include "robotics/Sensor.h"
#include "robotics/PartCatalog.h"
#include "robotics/AssemblyBuilder.h"

#include <chrono/physics/ChBody.h>
#include <chrono/physics/ChLinkBase.h>
#include <chrono/assets/ChVisualShapeBox.h>
#include <nlohmann/json.hpp>
#include <fstream>
#include <cmath>
#include <iostream>
#include <unordered_set>
#include <vector>

using json = nlohmann::json;

namespace robosim {

static VisualSpec ParseVisual(const json& obj) {
    VisualSpec spec;
    if (!obj.contains("visual") || !obj["visual"].is_object()) {
        return spec;
    }
    const auto& v = obj["visual"];
    spec.mesh = v.value("mesh", std::string());
    spec.texture = v.value("texture", std::string());
    spec.metallic = v.value("metallic", spec.metallic);
    spec.roughness = v.value("roughness", spec.roughness);
    if (v.contains("color") && v["color"].is_array() && v["color"].size() >= 3) {
        spec.colorR = v["color"][0].get<float>();
        spec.colorG = v["color"][1].get<float>();
        spec.colorB = v["color"][2].get<float>();
    }
    if (v.contains("scale")) {
        if (v["scale"].is_array() && v["scale"].size() >= 3) {
            spec.scaleX = v["scale"][0].get<double>();
            spec.scaleY = v["scale"][1].get<double>();
            spec.scaleZ = v["scale"][2].get<double>();
        } else if (v["scale"].is_number()) {
            spec.scaleX = spec.scaleY = spec.scaleZ = v["scale"].get<double>();
        }
    }
    if (v.contains("offset") && v["offset"].is_array() && v["offset"].size() >= 3) {
        spec.offsetX = v["offset"][0].get<double>();
        spec.offsetY = v["offset"][1].get<double>();
        spec.offsetZ = v["offset"][2].get<double>();
    }
    return spec;
}

static json VisualToJson(const VisualSpec& spec) {
    if (!spec.HasMesh() && !spec.HasTexture()) {
        return json();
    }
    json v;
    if (spec.HasMesh()) v["mesh"] = spec.mesh;
    if (spec.HasTexture()) v["texture"] = spec.texture;
    if (spec.scaleX != 1.0 || spec.scaleY != 1.0 || spec.scaleZ != 1.0) {
        if (spec.scaleX == spec.scaleY && spec.scaleY == spec.scaleZ) {
            v["scale"] = spec.scaleX;
        } else {
            v["scale"] = {spec.scaleX, spec.scaleY, spec.scaleZ};
        }
    }
    if (spec.colorR >= 0.0f) {
        v["color"] = {spec.colorR, spec.colorG, spec.colorB};
    }
    if (spec.metallic >= 0.0f) v["metallic"] = spec.metallic;
    if (spec.roughness >= 0.0f) v["roughness"] = spec.roughness;
    if (spec.offsetX != 0.0 || spec.offsetY != 0.0 || spec.offsetZ != 0.0) {
        v["offset"] = {spec.offsetX, spec.offsetY, spec.offsetZ};
    }
    return v;
}

static void ApplyOrDefault(const std::shared_ptr<RigidBody>& body, const VisualSpec& spec,
                           const std::string& fallbackMesh, float metallic, float roughness) {
    if (!body) return;
    VisualSpec use = spec;
    if (!use.HasMesh() && !fallbackMesh.empty()) {
        use.mesh = fallbackMesh;
    }
    if (use.HasMesh() || use.HasTexture()) {
        body->ApplyVisual(use, true);
    }
    body->SetPbr(metallic, roughness);
}

Robot::Robot() = default;
Robot::~Robot() = default;

static RobotConfig DefaultTwoWheelConfig(const RobotConfig& base) {
    RobotConfig cfg = base;
    if (cfg.wheels.empty() && cfg.wheelRadius > 0.0) {
        WheelSpec left;
        left.name = "left_wheel";
        left.radius = cfg.wheelRadius;
        left.width = cfg.wheelWidth;
        left.mass = cfg.wheelMass;
        left.offsetY = cfg.wheelOffsetY;
        left.offsetZ = -cfg.wheelOffsetZ;  // -Z is left when +X is forward, Y-up
        WheelSpec right = left;
        right.name = "right_wheel";
        right.offsetZ = cfg.wheelOffsetZ;
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
    const int kRobotFamily = 1;

    Material wheelMat;
    wheelMat.friction = 1.2;

    // 90 deg about X: cylinder's Y axis becomes Z (axle), so it looks like a wheel
    const double half = 0.7071067811865476;

    for (const auto& spec : m_config.wheels) {
        BodyTransform t;
        t.px = m_config.spawnX + spec.offsetX;
        t.py = spec.radius;  // sit on the ground plane y=0
        t.pz = m_config.spawnZ + spec.offsetZ;
        t.qw = half;
        t.qx = half;
        t.qy = 0;
        t.qz = 0;

        auto wheel = RigidBody::CreateCylinder(spec.radius, spec.width, spec.mass, t, wheelMat);
        wheel->SetName(spec.name);
        wheel->SetColor(0.08f, 0.08f, 0.08f);
        ApplyOrDefault(wheel, spec.visual, "parts/turtlebot3/left_tire.obj", 0.05f, 0.85f);
        wheel->SetCollisionFamily(kRobotFamily, false);
        world.AddBody(wheel);
        m_wheels.push_back(wheel);

        chrono::ChVector3d anchor(t.px, t.py, t.pz);
        chrono::ChVector3d axis(0, 0, 1);
        if (spec.motor) {
            auto motor = Motor::CreateSpeedMotor(world, chassisBody, wheel->GetChBody(), anchor, axis);
            motor->SetMaxSpeed(m_config.maxMotorSpeed);
            motor->SetMaxTorque(m_config.maxTorque);
            m_motors.push_back(motor);
        } else {
            auto joint = Joint::CreateRevolute(world, chassisBody, wheel->GetChBody(), anchor, axis);
            m_joints.push_back(joint);
        }
    }

    Material casterMat;
    casterMat.friction = 0.3;
    for (const auto& spec : m_config.casters) {
        BodyTransform t;
        t.px = m_config.spawnX + spec.offsetX;
        t.py = spec.radius;
        t.pz = m_config.spawnZ + spec.offsetZ;

        auto caster = RigidBody::CreateSphere(spec.radius, spec.mass, t, casterMat);
        caster->SetName(spec.name);
        caster->SetColor(0.55f, 0.55f, 0.58f);
        ApplyOrDefault(caster, spec.visual, "parts/wheel_caster.obj", 0.4f, 0.45f);
        caster->SetCollisionFamily(kRobotFamily, false);
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
        std::string fallback = (spec.geometry == "sphere") ? "parts/gripper.obj" : "parts/arm_link_a.obj";
        if (spec.name.find("elbow") != std::string::npos) fallback = "parts/arm_link_b.obj";
        if (spec.name.find("grip") != std::string::npos) fallback = "parts/gripper.obj";
        ApplyOrDefault(linkBody, spec.visual, fallback, 0.45f, 0.35f);
        linkBody->SetCollisionFamily(1, false);
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
    m_fromGraph = false;
    m_graph = {};
    m_buildErrors.clear();
    m_bodiesById.clear();
    m_motorsById.clear();
    m_driveGroups.clear();
    m_config = DefaultTwoWheelConfig(config);
    m_wheels.clear();
    m_casters.clear();
    m_links.clear();
    m_joints.clear();
    m_motors.clear();
    m_sensors.clear();

    Material chassisMat;
    chassisMat.friction = 0.4;

    double wheelR = 0.0;
    if (!m_config.wheels.empty()) wheelR = m_config.wheels.front().radius;
    // Chassis sits just above the axle so wheels, not the box, carry the robot
    m_config.spawnY = wheelR + m_config.chassisSizeY * 0.5 + 0.02;

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
    m_chassis->SetCollisionFamily(1, false);

    if (m_config.chassisVisual.HasMesh()) {
        ApplyOrDefault(m_chassis, m_config.chassisVisual, {}, 0.35f, 0.40f);
    } else {
        std::string fallback = "parts/turtlebot3/burger_base.obj";
        if (m_config.chassisSizeX >= 0.55) fallback = "parts/chassis_large.obj";
        else if (m_config.chassisSizeX <= 0.38) fallback = "parts/chassis_small.obj";
        if (m_config.links.size() >= 2 && m_config.wheels.empty()) fallback = "parts/chassis_arm.obj";
        ApplyOrDefault(m_chassis, m_config.chassisVisual, fallback, 0.35f, 0.40f);
    }

    if (!m_config.chassisVisual.HasMesh()) {
        // Nose: shows which way is forward so WASD direction is obvious
        auto nose = chrono_types::make_shared<chrono::ChVisualShapeBox>(
            m_config.chassisSizeX * 0.28, m_config.chassisSizeY * 0.45, m_config.chassisSizeZ * 0.35);
        nose->SetColor(chrono::ChColor(0.95f, 0.75f, 0.15f));
        m_chassis->GetChBody()->AddVisualShape(
            nose, chrono::ChFramed(chrono::ChVector3d(m_config.chassisSizeX * 0.38, m_config.chassisSizeY * 0.35, 0)));
    }

    for (const auto& extra : m_config.extras) {
        VisualSpec vis = extra.visual;
        vis.offsetX += extra.offsetX;
        vis.offsetY += extra.offsetY;
        vis.offsetZ += extra.offsetZ;
        if (vis.HasMesh()) {
            m_chassis->ApplyVisual(vis, false);
        }
    }

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

bool Robot::BuildFromGraph(PhysicsWorld& world, const AssemblyGraph& graph, const std::string& previewBodyId) {
    m_buildErrors.clear();
    m_wheels.clear();
    m_casters.clear();
    m_links.clear();
    m_joints.clear();
    m_motors.clear();
    m_sensors.clear();
    m_bodiesById.clear();
    m_motorsById.clear();
    m_driveGroups.clear();
    m_chassis.reset();

    AssemblyIssue issue;
    BuiltAssembly built;
    if (!BuildAssembly(world, graph, built, issue, previewBodyId)) {
        m_buildErrors = issue.errors;
        for (const auto& error : issue.errors) {
            std::cerr << "[Robot] " << error << std::endl;
        }
        m_fromGraph = false;
        return false;
    }

    m_fromGraph = true;
    m_graph = graph;
    m_config.name = graph.name;
    m_bodiesById = built.bodies;
    m_chassis = built.root;
    m_joints = built.joints;
    m_motors = built.motors;
    m_driveGroups = built.driveGroups;
    for (size_t i = 0; i < built.motors.size() && i < built.motorIds.size(); ++i) {
        m_motorsById[built.motorIds[i]] = built.motors[i];
    }
    if (m_chassis) {
        double x, y, z;
        m_chassis->GetPosition(x, y, z);
        m_config.spawnX = x;
        m_config.spawnY = y;
        m_config.spawnZ = z;
    }
    AddDefaultSensors();
    return true;
}

std::shared_ptr<RigidBody> Robot::FindBody(const std::string& id) const {
    auto it = m_bodiesById.find(id);
    if (it == m_bodiesById.end()) return nullptr;
    return it->second;
}

bool Robot::SetMotorTarget(const std::string& id, double value) {
    auto it = m_motorsById.find(id);
    if (it == m_motorsById.end() || !it->second) return false;
    it->second->SetTarget(value);
    return true;
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

    if (j.is_object() && j.contains("schema_version")) {
        const int version = j.value("schema_version", 0);
        if (version != 1) {
            m_buildErrors = {"unsupported schema_version"};
            std::cerr << "[Robot] unsupported schema_version " << version << std::endl;
            return;
        }
        AssemblyIssue issue;
        AssemblyGraph graph = AssemblyGraph::FromJson(j, issue);
        if (!issue.errors.empty()) {
            m_buildErrors = issue.errors;
            for (const auto& error : issue.errors) std::cerr << "[Robot] " << error << std::endl;
            return;
        }
        BuildFromGraph(world, graph);
        return;
    }

    std::vector<AssemblyItem> assembly;
    if (PartCatalog::ParseAssembly(j, assembly)) {
        auto& catalog = PartCatalog::Instance();
        if (!catalog.IsLoaded()) {
            catalog.Load();
        }
        json composed = PartCatalog::AssemblyToRobotJson(assembly, catalog, j.value("name", std::string("assembled")));
        composed["max_motor_speed"] = j.value("max_motor_speed", composed.value("max_motor_speed", 10.0));
        composed["max_torque"] = j.value("max_torque", composed.value("max_torque", 5.0));
        composed["spawn_x"] = j.value("spawn_x", 0.0);
        composed["spawn_y"] = j.value("spawn_y", 0.18);
        composed["spawn_z"] = j.value("spawn_z", 0.0);
        composed.erase("assembly");
        j = composed;
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
        cfg.chassisVisual = ParseVisual(c);
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
            spec.visual = ParseVisual(w);
            cfg.wheels.push_back(spec);
        }
        if (cfg.wheels.empty()) {
            cfg.wheelRadius = 0.0;  // JSON listed wheels but none — don't auto-add
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
            spec.visual = ParseVisual(c);
            cfg.casters.push_back(spec);
        }
        if (cfg.casters.empty()) {
            cfg.casterRadius = 0.0;  // JSON listed casters but none — don't auto-add
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
            spec.visual = ParseVisual(l);
            cfg.links.push_back(spec);
        }
    }

    if (j.contains("extras") && j["extras"].is_array()) {
        for (const auto& e : j["extras"]) {
            ExtraVisual extra;
            extra.name = e.value("name", std::string("extra"));
            extra.visual = ParseVisual(e);
            if (e.contains("offset") && e["offset"].is_array() && e["offset"].size() >= 3) {
                extra.offsetX = e["offset"][0].get<double>();
                extra.offsetY = e["offset"][1].get<double>();
                extra.offsetZ = e["offset"][2].get<double>();
            }
            cfg.extras.push_back(extra);
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
    if (m_fromGraph) return m_graph.ToJson();
    json j;
    j["name"] = m_config.name;
    j["chassis"] = {
        {"type", "box"},
        {"size", {m_config.chassisSizeX, m_config.chassisSizeY, m_config.chassisSizeZ}},
        {"mass", m_config.chassisMass}
    };
    if (auto v = VisualToJson(m_config.chassisVisual); !v.is_null()) {
        j["chassis"]["visual"] = v;
    }
    j["max_motor_speed"] = m_config.maxMotorSpeed;
    j["max_torque"] = m_config.maxTorque;
    j["wheels"] = json::array();
    for (const auto& w : m_config.wheels) {
        json wj = {
            {"side", w.name},
            {"radius", w.radius},
            {"width", w.width},
            {"mass", w.mass},
            {"offset", {w.offsetX, w.offsetY, w.offsetZ}},
            {"motor", w.motor}
        };
        if (auto v = VisualToJson(w.visual); !v.is_null()) wj["visual"] = v;
        j["wheels"].push_back(wj);
    }
    j["casters"] = json::array();
    for (const auto& c : m_config.casters) {
        json cj = {
            {"radius", c.radius},
            {"mass", c.mass},
            {"offset", {c.offsetX, c.offsetY, c.offsetZ}}
        };
        if (auto v = VisualToJson(c.visual); !v.is_null()) cj["visual"] = v;
        j["casters"].push_back(cj);
    }
    j["links"] = json::array();
    for (const auto& l : m_config.links) {
        json lj = {
            {"name", l.name},
            {"parent", l.parent},
            {"joint", l.jointType},
            {"geometry", l.geometry},
            {"size", {l.sizeX, l.sizeY, l.sizeZ}},
            {"mass", l.mass},
            {"offset", {l.offsetX, l.offsetY, l.offsetZ}},
            {"axis", {l.axisX, l.axisY, l.axisZ}},
            {"motor", l.motor}
        };
        if (auto v = VisualToJson(l.visual); !v.is_null()) lj["visual"] = v;
        j["links"].push_back(lj);
    }
    j["extras"] = json::array();
    for (const auto& e : m_config.extras) {
        json ej = {
            {"name", e.name},
            {"offset", {e.offsetX, e.offsetY, e.offsetZ}}
        };
        if (auto v = VisualToJson(e.visual); !v.is_null()) ej["visual"] = v;
        j["extras"].push_back(ej);
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
    bool tagged = false;
    for (int group : m_driveGroups) {
        if (group != 0) tagged = true;
    }
    if (tagged) {
        for (size_t i = 0; i < m_motors.size() && i < m_driveGroups.size(); ++i) {
            if (m_driveGroups[i] == 1) m_motors[i]->SetTargetSpeed(left);
            else if (m_driveGroups[i] == 2) m_motors[i]->SetTargetSpeed(right);
        }
        return;
    }
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

std::vector<std::shared_ptr<chrono::ChBody>> Robot::GetChBodies() const {
    std::vector<std::shared_ptr<chrono::ChBody>> out;
    std::unordered_set<const chrono::ChBody*> seen;
    auto add = [&](const std::shared_ptr<RigidBody>& body) {
        if (!body || !body->GetChBody()) return;
        if (!seen.insert(body->GetChBody().get()).second) return;
        out.push_back(body->GetChBody());
    };
    add(m_chassis);
    for (const auto& wheel : m_wheels) add(wheel);
    for (const auto& caster : m_casters) add(caster);
    for (const auto& link : m_links) add(link);
    for (const auto& entry : m_bodiesById) add(entry.second);
    return out;
}

void Robot::RemoveFromWorld(PhysicsWorld& world) {
    for (auto& motor : m_motors) {
        if (motor && motor->GetChLink()) {
            world.RemoveLink(motor->GetChLink());
        }
    }
    for (auto& joint : m_joints) {
        if (joint && joint->GetChLink()) {
            world.RemoveLink(joint->GetChLink());
        }
    }

    std::vector<std::shared_ptr<RigidBody>> bodies;
    if (m_chassis) bodies.push_back(m_chassis);
    for (auto& wheel : m_wheels) if (wheel) bodies.push_back(wheel);
    for (auto& caster : m_casters) if (caster) bodies.push_back(caster);
    for (auto& link : m_links) if (link) bodies.push_back(link);
    for (auto& entry : m_bodiesById) if (entry.second) bodies.push_back(entry.second);

    std::unordered_set<const RigidBody*> seen;
    for (auto& body : bodies) {
        if (!body || !seen.insert(body.get()).second) continue;
        world.RemoveBody(body);
    }
    world.ClearContacts();

    m_chassis.reset();
    m_wheels.clear();
    m_casters.clear();
    m_links.clear();
    m_joints.clear();
    m_motors.clear();
    m_sensors.clear();
    m_bodiesById.clear();
    m_motorsById.clear();
    m_driveGroups.clear();
    m_fromGraph = false;
}

void Robot::Reset(double x, double y, double z) {
    const double cy = m_config.spawnY;
    const double half = 0.7071067811865476;

    auto resetBody = [](const std::shared_ptr<RigidBody>& body,
                        double px, double py, double pz,
                        double qw, double qx, double qy, double qz) {
        if (!body) return;
        body->SetPosition(px, py, pz);
        body->SetRotation(qw, qx, qy, qz);
        body->SetLinearVelocity(0, 0, 0);
        body->SetAngularVelocity(0, 0, 0);
    };

    resetBody(m_chassis, x, y, z, 1, 0, 0, 0);

    for (size_t i = 0; i < m_wheels.size() && i < m_config.wheels.size(); ++i) {
        const auto& spec = m_config.wheels[i];
        resetBody(m_wheels[i],
                  x + spec.offsetX,
                  y + (spec.radius - cy),
                  z + spec.offsetZ,
                  half, half, 0, 0);
    }
    for (size_t i = 0; i < m_casters.size() && i < m_config.casters.size(); ++i) {
        const auto& spec = m_config.casters[i];
        resetBody(m_casters[i],
                  x + spec.offsetX,
                  y + (spec.radius - cy),
                  z + spec.offsetZ,
                  1, 0, 0, 0);
    }
    for (size_t i = 0; i < m_links.size() && i < m_config.links.size(); ++i) {
        const auto& spec = m_config.links[i];
        resetBody(m_links[i],
                  x + spec.offsetX,
                  y + spec.offsetY,
                  z + spec.offsetZ,
                  1, 0, 0, 0);
    }
    SetWheelSpeeds(0, 0);
}

}  // namespace robosim
