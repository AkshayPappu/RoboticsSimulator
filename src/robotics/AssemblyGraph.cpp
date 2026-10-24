#include "robotics/AssemblyGraph.h"
#include "robotics/PartCatalog.h"

#include <chrono/core/ChFrame.h>
#include <chrono/core/ChQuaternion.h>
#include <chrono/core/ChRotation.h>
#include <chrono/core/ChVector3.h>
#include <chrono/utils/ChConstants.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <unordered_map>
#include <unordered_set>

namespace robosim {

namespace {

double VolumeOf(const AssemblyBody& body) {
    const AssemblyShape shape = body.shape == AssemblyShape::Mesh ? body.collisionShape : body.shape;
    switch (shape) {
        case AssemblyShape::Box:
            return body.sizeX * body.sizeY * body.sizeZ;
        case AssemblyShape::Sphere:
            return (4.0 / 3.0) * chrono::CH_PI * body.radius * body.radius * body.radius;
        case AssemblyShape::Cylinder:
            return chrono::CH_PI * body.radius * body.radius * body.height;
        case AssemblyShape::Capsule:
            return chrono::CH_PI * body.radius * body.radius * body.height
                + (4.0 / 3.0) * chrono::CH_PI * body.radius * body.radius * body.radius;
        case AssemblyShape::Mesh:
            return 0.0;
    }
    return 0.0;
}

bool ReadVec3(const nlohmann::json& j, const char* key, double& x, double& y, double& z) {
    if (!j.contains(key) || !j[key].is_array() || j[key].size() < 3) return false;
    x = j[key][0].get<double>();
    y = j[key][1].get<double>();
    z = j[key][2].get<double>();
    return true;
}

VisualSpec ParseVisualObject(const nlohmann::json& v) {
    VisualSpec spec;
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

nlohmann::json VisualToJson(const VisualSpec& spec) {
    if (!spec.HasMesh() && !spec.HasTexture() && spec.colorR < 0.0f) return nlohmann::json();
    nlohmann::json v;
    if (spec.HasMesh()) v["mesh"] = spec.mesh;
    if (spec.HasTexture()) v["texture"] = spec.texture;
    if (spec.scaleX != 1.0 || spec.scaleY != 1.0 || spec.scaleZ != 1.0) {
        if (spec.scaleX == spec.scaleY && spec.scaleY == spec.scaleZ) v["scale"] = spec.scaleX;
        else v["scale"] = {spec.scaleX, spec.scaleY, spec.scaleZ};
    }
    if (spec.colorR >= 0.0f) v["color"] = {spec.colorR, spec.colorG, spec.colorB};
    if (spec.metallic >= 0.0f) v["metallic"] = spec.metallic;
    if (spec.roughness >= 0.0f) v["roughness"] = spec.roughness;
    if (spec.offsetX != 0.0 || spec.offsetY != 0.0 || spec.offsetZ != 0.0) {
        v["offset"] = {spec.offsetX, spec.offsetY, spec.offsetZ};
    }
    return v;
}

void ApplyCollisionJson(AssemblyBody& body, const nlohmann::json& c) {
    body.collisionSpecified = true;
    AssemblyShape shape;
    if (ParseShapeName(c.value("shape", std::string("box")), shape) && shape != AssemblyShape::Mesh) {
        body.collisionShape = shape;
    }
    body.radius = c.value("radius", body.radius);
    body.height = c.value("height", body.height);
    double sx = body.sizeX, sy = body.sizeY, sz = body.sizeZ;
    if (ReadVec3(c, "size", sx, sy, sz) || ReadVec3(c, "dimensions", sx, sy, sz)) {
        body.sizeX = sx;
        body.sizeY = sy;
        body.sizeZ = sz;
    }
}

void FillFromCatalog(AssemblyBody& body, AssemblyIssue& issue) {
    if (body.partId.empty() || body.shapeSpecified) return;
    auto& catalog = PartCatalog::Instance();
    if (!catalog.IsLoaded()) catalog.Load();
    const CatalogPart* part = catalog.Find(body.partId);
    if (!part) {
        issue.errors.push_back("unknown part_id '" + body.partId + "' on body '" + body.id + "'");
        return;
    }
    AssemblyShape shape;
    if (!ParseShapeName(part->collision.shape, shape) || shape == AssemblyShape::Mesh) {
        body.shape = AssemblyShape::Box;
    } else {
        body.shape = shape;
    }
    body.shapeSpecified = true;
    body.collisionShape = body.shape;
    body.collisionSpecified = true;
    body.sizeX = part->collision.sizeX;
    body.sizeY = part->collision.sizeY;
    body.sizeZ = part->collision.sizeZ;
    body.radius = part->collision.radius;
    body.height = part->collision.height;
    if (!body.hasMass) {
        body.mass = part->mass;
        body.hasMass = part->mass > 0.0;
    }
    if (!body.visual.HasMesh() && part->visual.HasMesh()) {
        body.visual = part->visual;
    }
    if (part->visualOnly) body.collision = false;
}

}  // namespace

const char* ShapeName(AssemblyShape shape) {
    switch (shape) {
        case AssemblyShape::Box: return "box";
        case AssemblyShape::Sphere: return "sphere";
        case AssemblyShape::Cylinder: return "cylinder";
        case AssemblyShape::Capsule: return "capsule";
        case AssemblyShape::Mesh: return "mesh";
    }
    return "box";
}

const char* JointTypeName(AssemblyJointType type) {
    switch (type) {
        case AssemblyJointType::Fixed: return "fixed";
        case AssemblyJointType::Revolute: return "revolute";
        case AssemblyJointType::Prismatic: return "prismatic";
        case AssemblyJointType::Spherical: return "spherical";
    }
    return "revolute";
}

const char* MotorModeName(AssemblyMotorMode mode) {
    switch (mode) {
        case AssemblyMotorMode::Velocity: return "velocity";
        case AssemblyMotorMode::Position: return "position";
        case AssemblyMotorMode::Torque: return "torque";
        case AssemblyMotorMode::Force: return "force";
    }
    return "velocity";
}

bool ParseShapeName(const std::string& text, AssemblyShape& out) {
    if (text == "box") { out = AssemblyShape::Box; return true; }
    if (text == "sphere") { out = AssemblyShape::Sphere; return true; }
    if (text == "cylinder") { out = AssemblyShape::Cylinder; return true; }
    if (text == "capsule") { out = AssemblyShape::Capsule; return true; }
    if (text == "mesh") { out = AssemblyShape::Mesh; return true; }
    return false;
}

bool ParseJointTypeName(const std::string& text, AssemblyJointType& out) {
    if (text == "fixed") { out = AssemblyJointType::Fixed; return true; }
    if (text == "revolute") { out = AssemblyJointType::Revolute; return true; }
    if (text == "prismatic") { out = AssemblyJointType::Prismatic; return true; }
    if (text == "spherical") { out = AssemblyJointType::Spherical; return true; }
    return false;
}

bool ParseMotorModeName(const std::string& text, AssemblyMotorMode& out) {
    if (text == "velocity" || text == "speed") { out = AssemblyMotorMode::Velocity; return true; }
    if (text == "position") { out = AssemblyMotorMode::Position; return true; }
    if (text == "torque") { out = AssemblyMotorMode::Torque; return true; }
    if (text == "force") { out = AssemblyMotorMode::Force; return true; }
    return false;
}

AssemblyPose MakePose(double x, double y, double z, double qw, double qx, double qy, double qz) {
    AssemblyPose pose;
    pose.px = x;
    pose.py = y;
    pose.pz = z;
    pose.qw = qw;
    pose.qx = qx;
    pose.qy = qy;
    pose.qz = qz;
    return pose;
}

AssemblyPose PoseAlignYToAxis(double x, double y, double z, double ax, double ay, double az) {
    chrono::ChVector3d axis(ax, ay, az);
    if (axis.Length() < 1e-8) axis = chrono::ChVector3d(0, 1, 0);
    axis.Normalize();
    chrono::ChVector3d yAxis(0, 1, 0);
    chrono::ChQuaterniond rot = chrono::QUNIT;
    chrono::ChVector3d cross = yAxis.Cross(axis);
    double dot = std::clamp(yAxis.Dot(axis), -1.0, 1.0);
    if (cross.Length() > 1e-6) {
        cross.Normalize();
        rot = chrono::QuatFromAngleAxis(std::acos(dot), cross);
    } else if (dot < 0) {
        rot = chrono::QuatFromAngleAxis(chrono::CH_PI, chrono::ChVector3d(1, 0, 0));
    }
    return MakePose(x, y, z, rot.e0(), rot.e1(), rot.e2(), rot.e3());
}

void PoseToEulerDeg(const AssemblyPose& pose, double& rx, double& ry, double& rz) {
    const double qw = pose.qw, qx = pose.qx, qy = pose.qy, qz = pose.qz;
    const double sinr = 2.0 * (qw * qx + qy * qz);
    const double cosr = 1.0 - 2.0 * (qx * qx + qy * qy);
    rx = std::atan2(sinr, cosr) * 180.0 / chrono::CH_PI;
    const double sinp = 2.0 * (qw * qy - qz * qx);
    if (std::abs(sinp) >= 1.0) ry = std::copysign(90.0, sinp);
    else ry = std::asin(std::clamp(sinp, -1.0, 1.0)) * 180.0 / chrono::CH_PI;
    const double siny = 2.0 * (qw * qz + qx * qy);
    const double cosy = 1.0 - 2.0 * (qy * qy + qz * qz);
    rz = std::atan2(siny, cosy) * 180.0 / chrono::CH_PI;
}

AssemblyPose PoseFromEulerDeg(double x, double y, double z, double rx, double ry, double rz) {
    const double deg = chrono::CH_PI / 180.0;
    chrono::ChQuaterniond q = chrono::QuatFromAngleX(rx * deg)
        * chrono::QuatFromAngleY(ry * deg)
        * chrono::QuatFromAngleZ(rz * deg);
    return MakePose(x, y, z, q.e0(), q.e1(), q.e2(), q.e3());
}

namespace {

chrono::ChFrame<> PoseFrame(const AssemblyPose& pose) {
    return chrono::ChFrame<>(
        chrono::ChVector3d(pose.px, pose.py, pose.pz),
        chrono::ChQuaterniond(pose.qw, pose.qx, pose.qy, pose.qz));
}

double DownExtent(const AssemblyBody& body, const chrono::ChFrame<>& world) {
    if (!body.collision) return 0.0;
    const AssemblyShape shape = body.shape == AssemblyShape::Mesh ? body.collisionShape : body.shape;
    if (shape == AssemblyShape::Sphere) return std::max(0.0, body.radius);
    if (shape == AssemblyShape::Cylinder || shape == AssemblyShape::Capsule) {
        const chrono::ChVector3d axis = world.GetRot().Rotate(chrono::ChVector3d(0, 1, 0));
        const double ay = std::min(1.0, std::abs(axis.y()));
        const double radial = std::sqrt(std::max(0.0, 1.0 - ay * ay));
        double extent = body.height * 0.5 * ay + body.radius * radial;
        if (shape == AssemblyShape::Capsule) extent += body.radius;
        return extent;
    }
    const auto rot = world.GetRot();
    return std::abs(rot.Rotate(chrono::ChVector3d(body.sizeX * 0.5, 0, 0)).y())
         + std::abs(rot.Rotate(chrono::ChVector3d(0, body.sizeY * 0.5, 0)).y())
         + std::abs(rot.Rotate(chrono::ChVector3d(0, 0, body.sizeZ * 0.5)).y());
}

}  // namespace

bool LiftAssemblyOntoGround(AssemblyGraph& graph) {
    if (graph.bodies.empty()) return false;

    std::unordered_map<std::string, chrono::ChFrame<>> world;
    bool progress = true;
    while (world.size() < graph.bodies.size() && progress) {
        progress = false;
        for (const auto& body : graph.bodies) {
            if (world.count(body.id)) continue;
            if (!body.parent.empty() && !world.count(body.parent)) continue;
            const chrono::ChFrame<> local = PoseFrame(body.pose);
            world[body.id] = body.parent.empty() ? local : world[body.parent] * local;
            progress = true;
        }
    }

    double minY = 1e9;
    for (const auto& body : graph.bodies) {
        const auto it = world.find(body.id);
        if (it == world.end()) continue;
        minY = std::min(minY, it->second.GetPos().y() - DownExtent(body, it->second));
    }
    if (!(minY < 0.01)) return false;
    const double delta = 0.01 - minY;
    if (delta < 1e-5) return false;

    bool moved = false;
    for (auto& body : graph.bodies) {
        if (!body.parent.empty()) continue;
        body.pose.py += delta;
        moved = true;
    }
    return moved;
}

bool AssemblyGraph::IsSchemaV1(const nlohmann::json& j) {
    return j.is_object() && j.value("schema_version", 0) == 1;
}

const AssemblyBody* AssemblyGraph::FindBody(const std::string& id) const {
    for (const auto& body : bodies) {
        if (body.id == id) return &body;
    }
    return nullptr;
}

AssemblyBody* AssemblyGraph::FindBody(const std::string& id) {
    for (auto& body : bodies) {
        if (body.id == id) return &body;
    }
    return nullptr;
}

const AssemblyJoint* AssemblyGraph::FindJoint(const std::string& id) const {
    for (const auto& joint : joints) {
        if (joint.id == id) return &joint;
    }
    return nullptr;
}

AssemblyJoint* AssemblyGraph::FindJoint(const std::string& id) {
    for (auto& joint : joints) {
        if (joint.id == id) return &joint;
    }
    return nullptr;
}

AssemblyGraph AssemblyGraph::FromJson(const nlohmann::json& j, AssemblyIssue& issue) {
    AssemblyGraph graph;
    if (!j.is_object()) {
        issue.errors.push_back("robot document is not a JSON object");
        return graph;
    }
    const int version = j.value("schema_version", 0);
    if (version != 1) {
        issue.errors.push_back("schema_version must be 1");
        return graph;
    }
    graph.schemaVersion = 1;
    graph.name = j.value("name", std::string("assembly"));
    graph.root = j.value("root", std::string());
    const std::string policy = j.value("disconnected_policy", std::string("warn"));
    if (policy == "error") graph.disconnectedPolicy = DisconnectedPolicy::Error;
    else if (policy == "warn") graph.disconnectedPolicy = DisconnectedPolicy::Warn;
    else issue.errors.push_back("disconnected_policy must be 'warn' or 'error'");

    if (!j.contains("bodies") || !j["bodies"].is_array()) {
        issue.errors.push_back("bodies must be an array");
        return graph;
    }

    for (const auto& bj : j["bodies"]) {
        AssemblyBody body;
        body.id = bj.value("id", std::string());
        body.partId = bj.value("part_id", std::string());
        if (bj.contains("shape")) {
            body.shapeSpecified = true;
            const std::string shapeText = bj.value("shape", std::string());
            if (!ParseShapeName(shapeText, body.shape)) {
                issue.errors.push_back("body '" + body.id + "' has unknown shape '" + shapeText + "'");
            }
        }
        double sx, sy, sz;
        if (ReadVec3(bj, "dimensions", sx, sy, sz) || ReadVec3(bj, "size", sx, sy, sz)) {
            body.sizeX = sx;
            body.sizeY = sy;
            body.sizeZ = sz;
        }
        if (bj.contains("radius")) body.radius = bj.value("radius", body.radius);
        if (bj.contains("height")) body.height = bj.value("height", body.height);
        if (bj.contains("mass")) {
            body.hasMass = true;
            body.mass = bj.value("mass", 0.0);
        }
        if (bj.contains("density")) {
            body.hasDensity = true;
            body.density = bj.value("density", 0.0);
        }
        body.fixedBody = bj.value("fixed", false);
        if (bj.contains("collision") && bj["collision"].is_boolean()) {
            body.collision = bj["collision"].get<bool>();
        } else if (!bj.contains("collision")) {
            body.collision = true;
        }
        if (bj.contains("collision") && bj["collision"].is_object()) {
            ApplyCollisionJson(body, bj["collision"]);
        }
        body.friction = bj.value("friction", 0.5);
        body.restitution = bj.value("restitution", 0.1);
        body.parent = bj.value("parent", std::string());
        if (bj.contains("pose") && bj["pose"].is_object()) {
            const auto& pose = bj["pose"];
            ReadVec3(pose, "position", body.pose.px, body.pose.py, body.pose.pz);
            if (pose.contains("rotation")) {
                if (!pose["rotation"].is_array() || pose["rotation"].size() < 4) {
                    issue.errors.push_back("body '" + body.id + "' rotation must be [w, x, y, z]");
                } else {
                    body.pose.qw = pose["rotation"][0].get<double>();
                    body.pose.qx = pose["rotation"][1].get<double>();
                    body.pose.qy = pose["rotation"][2].get<double>();
                    body.pose.qz = pose["rotation"][3].get<double>();
                }
            }
        }
        if (bj.contains("visual") && bj["visual"].is_object()) {
            body.visual = ParseVisualObject(bj["visual"]);
        }
        if (bj.contains("collision_shape") && bj["collision_shape"].is_object()) {
            ApplyCollisionJson(body, bj["collision_shape"]);
        }
        FillFromCatalog(body, issue);
        graph.bodies.push_back(body);
    }

    if (j.contains("joints")) {
        if (!j["joints"].is_array()) {
            issue.errors.push_back("joints must be an array");
        } else {
            for (const auto& jj : j["joints"]) {
                AssemblyJoint joint;
                joint.id = jj.value("id", std::string());
                const std::string typeText = jj.value("type", std::string("revolute"));
                if (!ParseJointTypeName(typeText, joint.type)) {
                    issue.errors.push_back("joint '" + joint.id + "' has unknown type '" + typeText + "'");
                }
                joint.bodyA = jj.value("body_a", std::string());
                joint.bodyB = jj.value("body_b", std::string());
                ReadVec3(jj, "anchor", joint.anchorX, joint.anchorY, joint.anchorZ);
                ReadVec3(jj, "axis", joint.axisX, joint.axisY, joint.axisZ);
                graph.joints.push_back(joint);
            }
        }
    }

    if (j.contains("motors")) {
        if (!j["motors"].is_array()) {
            issue.errors.push_back("motors must be an array");
        } else {
            for (const auto& mj : j["motors"]) {
                AssemblyMotor motor;
                motor.id = mj.value("id", std::string());
                motor.jointId = mj.value("joint", std::string());
                const std::string modeText = mj.value("mode", std::string("velocity"));
                if (!ParseMotorModeName(modeText, motor.mode)) {
                    issue.errors.push_back("motor '" + motor.id + "' has unknown mode '" + modeText + "'");
                }
                motor.maxSpeed = mj.value("max_speed", 10.0);
                motor.maxTorque = mj.value("max_torque", 5.0);
                const std::string drive = mj.value("drive", std::string());
                if (drive == "left") motor.drive = AssemblyDrive::Left;
                else if (drive == "right") motor.drive = AssemblyDrive::Right;
                else if (!drive.empty()) {
                    issue.errors.push_back("motor '" + motor.id + "' drive must be 'left' or 'right'");
                }
                graph.motors.push_back(motor);
            }
        }
    }

    if (graph.root.empty() && !graph.bodies.empty()) {
        graph.root = graph.bodies.front().id;
    }
    return graph;
}

nlohmann::json AssemblyGraph::ToJson() const {
    nlohmann::json j;
    j["schema_version"] = 1;
    j["name"] = name;
    if (!root.empty()) j["root"] = root;
    j["disconnected_policy"] = disconnectedPolicy == DisconnectedPolicy::Error ? "error" : "warn";
    j["bodies"] = nlohmann::json::array();
    for (const auto& body : bodies) {
        nlohmann::json b;
        b["id"] = body.id;
        if (!body.partId.empty()) b["part_id"] = body.partId;
        b["shape"] = ShapeName(body.shape);
        if (body.shape == AssemblyShape::Box || body.shape == AssemblyShape::Mesh) {
            b["dimensions"] = {body.sizeX, body.sizeY, body.sizeZ};
        }
        if (body.shape == AssemblyShape::Sphere || body.shape == AssemblyShape::Cylinder
            || body.shape == AssemblyShape::Capsule) {
            b["radius"] = body.radius;
        }
        if (body.shape == AssemblyShape::Cylinder || body.shape == AssemblyShape::Capsule) {
            b["height"] = body.height;
        }
        if (body.hasMass) b["mass"] = body.mass;
        if (body.hasDensity) b["density"] = body.density;
        if (body.fixedBody) b["fixed"] = true;
        b["collision"] = body.collision;
        b["friction"] = body.friction;
        b["restitution"] = body.restitution;
        if (!body.parent.empty()) b["parent"] = body.parent;
        b["pose"] = {
            {"position", {body.pose.px, body.pose.py, body.pose.pz}},
            {"rotation", {body.pose.qw, body.pose.qx, body.pose.qy, body.pose.qz}}
        };
        if (auto visual = VisualToJson(body.visual); !visual.is_null()) b["visual"] = visual;
        if (body.shape == AssemblyShape::Mesh || body.collisionSpecified) {
            nlohmann::json c;
            c["shape"] = ShapeName(body.collisionShape);
            if (body.collisionShape == AssemblyShape::Box) {
                c["size"] = {body.sizeX, body.sizeY, body.sizeZ};
            } else {
                c["radius"] = body.radius;
                if (body.collisionShape != AssemblyShape::Sphere) c["height"] = body.height;
            }
            b["collision_shape"] = c;
        }
        j["bodies"].push_back(b);
    }
    j["joints"] = nlohmann::json::array();
    for (const auto& joint : joints) {
        nlohmann::json jj = {
            {"id", joint.id},
            {"type", JointTypeName(joint.type)},
            {"body_a", joint.bodyA},
            {"body_b", joint.bodyB},
            {"anchor", {joint.anchorX, joint.anchorY, joint.anchorZ}},
            {"axis", {joint.axisX, joint.axisY, joint.axisZ}}
        };
        j["joints"].push_back(jj);
    }
    j["motors"] = nlohmann::json::array();
    for (const auto& motor : motors) {
        nlohmann::json mj = {
            {"id", motor.id},
            {"joint", motor.jointId},
            {"mode", MotorModeName(motor.mode)},
            {"max_speed", motor.maxSpeed},
            {"max_torque", motor.maxTorque}
        };
        if (motor.drive == AssemblyDrive::Left) mj["drive"] = "left";
        if (motor.drive == AssemblyDrive::Right) mj["drive"] = "right";
        j["motors"].push_back(mj);
    }
    return j;
}

AssemblyIssue AssemblyGraph::Validate() const {
    AssemblyIssue issue;
    if (schemaVersion != 1) issue.errors.push_back("schema_version must be 1");
    if (bodies.empty()) issue.errors.push_back("assembly has no bodies");

    std::unordered_set<std::string> ids;
    auto claim = [&](const std::string& id, const char* what) {
        if (id.empty()) {
            issue.errors.push_back(std::string(what) + " is missing an id");
            return;
        }
        if (!ids.insert(id).second) {
            issue.errors.push_back(std::string("duplicate id '") + id + "'");
        }
    };
    for (const auto& body : bodies) claim(body.id, "body");
    for (const auto& joint : joints) claim(joint.id, "joint");
    for (const auto& motor : motors) claim(motor.id, "motor");

    std::unordered_map<std::string, const AssemblyBody*> byId;
    for (const auto& body : bodies) {
        if (!body.id.empty()) byId[body.id] = &body;
    }

    if (!root.empty() && !byId.count(root)) {
        issue.errors.push_back("root '" + root + "' is not a body");
    }

    for (const auto& body : bodies) {
        if (!body.parent.empty()) {
            if (!byId.count(body.parent)) {
                issue.errors.push_back("body '" + body.id + "' parent '" + body.parent + "' does not exist");
            } else if (body.parent == body.id) {
                issue.errors.push_back("body '" + body.id + "' parents itself");
            }
        }
        const double qlen = std::sqrt(body.pose.qw * body.pose.qw + body.pose.qx * body.pose.qx
            + body.pose.qy * body.pose.qy + body.pose.qz * body.pose.qz);
        if (qlen < 1e-6) {
            issue.errors.push_back("body '" + body.id + "' has a zero rotation");
        }
        if (!body.shapeSpecified && body.partId.empty()) {
            issue.errors.push_back("body '" + body.id + "' needs a shape or a part_id");
        }
        if (body.hasMass && body.hasDensity && body.mass <= 0.0 && body.density <= 0.0) {
            issue.errors.push_back("body '" + body.id + "' needs a positive mass or density");
        }
        if (!body.hasMass && !body.hasDensity) {
            issue.errors.push_back("body '" + body.id + "' needs a mass or a density");
        } else if (body.hasMass && !body.hasDensity && body.mass <= 0.0) {
            issue.errors.push_back("body '" + body.id + "' mass must be positive");
        } else if (!body.hasMass && body.hasDensity && body.density <= 0.0) {
            issue.errors.push_back("body '" + body.id + "' density must be positive");
        }

        const AssemblyShape solid = body.shape == AssemblyShape::Mesh ? body.collisionShape : body.shape;
        if (body.shape == AssemblyShape::Mesh && !body.collisionSpecified && !body.visual.HasMesh()) {
            issue.errors.push_back("mesh body '" + body.id + "' needs a visual mesh and a collision shape");
        }
        if (body.shape == AssemblyShape::Mesh && body.visual.mesh.empty()) {
            issue.errors.push_back("mesh body '" + body.id + "' needs a visual mesh");
        }
        if (solid == AssemblyShape::Box) {
            if (body.sizeX <= 0.0 || body.sizeY <= 0.0 || body.sizeZ <= 0.0) {
                issue.errors.push_back("body '" + body.id + "' box dimensions must be positive");
            }
        } else if (solid == AssemblyShape::Sphere) {
            if (body.radius <= 0.0) issue.errors.push_back("body '" + body.id + "' radius must be positive");
        } else if (solid == AssemblyShape::Cylinder || solid == AssemblyShape::Capsule) {
            if (body.radius <= 0.0 || body.height <= 0.0) {
                issue.errors.push_back("body '" + body.id + "' radius and height must be positive");
            }
        }
        if (VolumeOf(body) <= 0.0 && body.shapeSpecified) {
            issue.errors.push_back("body '" + body.id + "' has zero volume");
        }
    }

    std::unordered_map<std::string, int> color;
    for (const auto& body : bodies) color[body.id] = 0;
    std::function<bool(const std::string&)> visit = [&](const std::string& id) {
        if (!color.count(id) || color[id] == 2) return true;
        if (color[id] == 1) return false;
        color[id] = 1;
        const AssemblyBody* body = byId.count(id) ? byId[id] : nullptr;
        if (body && !body->parent.empty()) {
            if (!visit(body->parent)) return false;
        }
        color[id] = 2;
        return true;
    };
    for (const auto& body : bodies) {
        if (!visit(body.id)) {
            issue.errors.push_back("body parent chain contains a cycle at '" + body.id + "'");
            break;
        }
    }

    std::unordered_set<std::string> motorJoints;
    for (const auto& joint : joints) {
        if (!byId.count(joint.bodyA)) {
            issue.errors.push_back("joint '" + joint.id + "' body_a '" + joint.bodyA + "' does not exist");
        }
        if (!byId.count(joint.bodyB)) {
            issue.errors.push_back("joint '" + joint.id + "' body_b '" + joint.bodyB + "' does not exist");
        }
        if (!joint.bodyA.empty() && joint.bodyA == joint.bodyB) {
            issue.errors.push_back("joint '" + joint.id + "' connects a body to itself");
        }
        if (joint.type == AssemblyJointType::Revolute || joint.type == AssemblyJointType::Prismatic) {
            const double alen = std::sqrt(joint.axisX * joint.axisX + joint.axisY * joint.axisY + joint.axisZ * joint.axisZ);
            if (alen < 1e-8) issue.errors.push_back("joint '" + joint.id + "' axis length is zero");
        }
    }

    for (const auto& motor : motors) {
        const AssemblyJoint* joint = FindJoint(motor.jointId);
        if (!joint) {
            issue.errors.push_back("motor '" + motor.id + "' joint '" + motor.jointId + "' does not exist");
            continue;
        }
        if (!motorJoints.insert(motor.jointId).second) {
            issue.errors.push_back("joint '" + motor.jointId + "' has more than one motor");
        }
        const bool linearJoint = joint->type == AssemblyJointType::Prismatic;
        const bool rotaryJoint = joint->type == AssemblyJointType::Revolute;
        if (joint->type == AssemblyJointType::Fixed || joint->type == AssemblyJointType::Spherical) {
            issue.errors.push_back("motor '" + motor.id + "' cannot drive a " + std::string(JointTypeName(joint->type)) + " joint");
        } else if (motor.mode == AssemblyMotorMode::Force && !linearJoint) {
            issue.errors.push_back("motor '" + motor.id + "' force mode requires a prismatic joint");
        } else if (motor.mode == AssemblyMotorMode::Torque && !rotaryJoint) {
            issue.errors.push_back("motor '" + motor.id + "' torque mode requires a revolute joint");
        }
        if (motor.maxSpeed < 0.0 || motor.maxTorque < 0.0) {
            issue.errors.push_back("motor '" + motor.id + "' limits must be non-negative");
        }
    }

    if (!bodies.empty()) {
        std::unordered_map<std::string, std::string> parentOf;
        for (const auto& body : bodies) parentOf[body.id] = body.id;
        std::function<std::string(const std::string&)> find = [&](const std::string& id) -> std::string {
            if (parentOf[id] == id) return id;
            parentOf[id] = find(parentOf[id]);
            return parentOf[id];
        };
        auto unite = [&](const std::string& a, const std::string& b) {
            if (!parentOf.count(a) || !parentOf.count(b)) return;
            parentOf[find(a)] = find(b);
        };
        for (const auto& joint : joints) unite(joint.bodyA, joint.bodyB);
        std::unordered_set<std::string> roots;
        for (const auto& body : bodies) roots.insert(find(body.id));
        if (roots.size() > 1) {
            const std::string message = "assembly has " + std::to_string(roots.size()) + " disconnected components";
            if (disconnectedPolicy == DisconnectedPolicy::Error) issue.errors.push_back(message);
            else issue.warnings.push_back(message);
        }
    }
    return issue;
}

}  // namespace robosim
