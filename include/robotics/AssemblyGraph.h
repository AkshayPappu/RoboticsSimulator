#pragma once

#include "physics/Assets.h"

#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace robosim {

// Schema v1 units: meters, kilograms, seconds, radians.
// Axes: Y up, +X forward, +Z right. Gravity is (0, -9.81, 0).
// A body pose is in its parent's frame. A body with an empty parent is in the world frame.
// Joint anchor and axis are in body A's local frame. Cylinder and capsule axes are local Y.

enum class AssemblyShape {
    Box,
    Sphere,
    Cylinder,
    Capsule,
    Mesh
};

enum class AssemblyJointType {
    Fixed,
    Revolute,
    Prismatic,
    Spherical
};

enum class AssemblyMotorMode {
    Velocity,
    Position,
    Torque,
    Force
};

enum class AssemblyDrive {
    None,
    Left,
    Right
};

enum class DisconnectedPolicy {
    Warn,
    Error
};

struct AssemblyPose {
    double px = 0, py = 0, pz = 0;
    double qw = 1, qx = 0, qy = 0, qz = 0;
};

struct AssemblyBody {
    std::string id;
    std::string partId;
    bool shapeSpecified = false;
    AssemblyShape shape = AssemblyShape::Box;
    double sizeX = 0.1, sizeY = 0.1, sizeZ = 0.1;
    double radius = 0.05;
    double height = 0.1;
    bool hasMass = false;
    double mass = 1.0;
    bool hasDensity = false;
    double density = 0.0;
    bool fixedBody = false;
    bool collision = true;
    double friction = 0.5;
    double restitution = 0.1;
    std::string parent;
    AssemblyPose pose;
    VisualSpec visual;
    bool collisionSpecified = false;
    AssemblyShape collisionShape = AssemblyShape::Box;
};

struct AssemblyJoint {
    std::string id;
    AssemblyJointType type = AssemblyJointType::Revolute;
    std::string bodyA;
    std::string bodyB;
    double anchorX = 0, anchorY = 0, anchorZ = 0;
    double axisX = 0, axisY = 0, axisZ = 1;
};

struct AssemblyMotor {
    std::string id;
    std::string jointId;
    AssemblyMotorMode mode = AssemblyMotorMode::Velocity;
    double maxSpeed = 10.0;
    double maxTorque = 5.0;
    AssemblyDrive drive = AssemblyDrive::None;
};

struct AssemblyIssue {
    std::vector<std::string> errors;
    std::vector<std::string> warnings;
};

struct AssemblyGraph {
    int schemaVersion = 1;
    std::string name = "assembly";
    std::string root;
    DisconnectedPolicy disconnectedPolicy = DisconnectedPolicy::Warn;
    std::vector<AssemblyBody> bodies;
    std::vector<AssemblyJoint> joints;
    std::vector<AssemblyMotor> motors;

    static bool IsSchemaV1(const nlohmann::json& j);
    static AssemblyGraph FromJson(const nlohmann::json& j, AssemblyIssue& issue);
    nlohmann::json ToJson() const;

    AssemblyIssue Validate() const;

    const AssemblyBody* FindBody(const std::string& id) const;
    AssemblyBody* FindBody(const std::string& id);
    const AssemblyJoint* FindJoint(const std::string& id) const;
    AssemblyJoint* FindJoint(const std::string& id);
};

AssemblyPose MakePose(double x, double y, double z,
                      double qw = 1, double qx = 0, double qy = 0, double qz = 0);
AssemblyPose PoseAlignYToAxis(double x, double y, double z, double ax, double ay, double az);
void PoseToEulerDeg(const AssemblyPose& pose, double& rx, double& ry, double& rz);
AssemblyPose PoseFromEulerDeg(double x, double y, double z, double rx, double ry, double rz);

// Raises bodies that have no parent so every colliding shape sits just above the ground.
// Child poses stay in the parent frame, so they rise with the root. Returns true when a pose changes.
bool LiftAssemblyOntoGround(AssemblyGraph& graph);

const char* ShapeName(AssemblyShape shape);
const char* JointTypeName(AssemblyJointType type);
const char* MotorModeName(AssemblyMotorMode mode);

bool ParseShapeName(const std::string& text, AssemblyShape& out);
bool ParseJointTypeName(const std::string& text, AssemblyJointType& out);
bool ParseMotorModeName(const std::string& text, AssemblyMotorMode& out);

}  // namespace robosim
