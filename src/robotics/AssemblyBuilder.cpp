#include "robotics/AssemblyBuilder.h"
#include "core/PhysicsWorld.h"
#include "physics/RigidBody.h"
#include "robotics/Joint.h"
#include "robotics/Motor.h"

#include <chrono/assets/ChVisualShapeBox.h>
#include <chrono/assets/ChVisualShapeCapsule.h>
#include <chrono/assets/ChVisualShapeCylinder.h>
#include <chrono/assets/ChVisualShapeSphere.h>
#include <chrono/core/ChFrame.h>
#include <chrono/utils/ChConstants.h>

#include <cmath>
#include <iostream>
#include <unordered_map>
#include <unordered_set>

namespace robosim {

namespace {

double SolidVolume(const AssemblyBody& body) {
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

chrono::ChQuaterniond TiltZToY() {
    return chrono::QuatFromAngleX(-chrono::CH_PI_2);
}

void AddCollisionPreview(chrono::ChBody& body, const AssemblyBody& spec) {
    const AssemblyShape shape = spec.shape == AssemblyShape::Mesh ? spec.collisionShape : spec.shape;
    std::shared_ptr<chrono::ChVisualShape> visual;
    chrono::ChFrame<> frame;
    switch (shape) {
        case AssemblyShape::Box:
            visual = chrono_types::make_shared<chrono::ChVisualShapeBox>(spec.sizeX, spec.sizeY, spec.sizeZ);
            break;
        case AssemblyShape::Sphere:
            visual = chrono_types::make_shared<chrono::ChVisualShapeSphere>(spec.radius);
            break;
        case AssemblyShape::Cylinder:
            visual = chrono_types::make_shared<chrono::ChVisualShapeCylinder>(spec.radius, spec.height);
            frame = chrono::ChFrame<>(chrono::ChVector3d(0, 0, 0), TiltZToY());
            break;
        case AssemblyShape::Capsule:
            visual = chrono_types::make_shared<chrono::ChVisualShapeCapsule>(spec.radius, spec.height);
            frame = chrono::ChFrame<>(chrono::ChVector3d(0, 0, 0), TiltZToY());
            break;
        case AssemblyShape::Mesh:
            break;
    }
    if (!visual) return;
    visual->SetColor(chrono::ChColor(1.0f, 0.45f, 0.15f));
    visual->SetOpacity(0.45f);
    body.AddVisualShape(visual, frame);
}

constexpr int kAssemblyFamily = 1;

}  // namespace

bool BuildAssembly(PhysicsWorld& world,
                   const AssemblyGraph& graph,
                   BuiltAssembly& out,
                   AssemblyIssue& issue,
                   const std::string& previewBodyId) {
    out = {};
    AssemblyIssue found = graph.Validate();
    issue.errors.insert(issue.errors.end(), found.errors.begin(), found.errors.end());
    issue.warnings.insert(issue.warnings.end(), found.warnings.begin(), found.warnings.end());
    if (!issue.errors.empty()) return false;

    std::unordered_map<std::string, const AssemblyBody*> byId;
    for (const auto& body : graph.bodies) byId[body.id] = &body;

    std::vector<const AssemblyBody*> ordered;
    std::unordered_set<std::string> placed;
    while (ordered.size() < graph.bodies.size()) {
        bool progress = false;
        for (const auto& body : graph.bodies) {
            if (placed.count(body.id)) continue;
            if (body.parent.empty() || placed.count(body.parent)) {
                ordered.push_back(&body);
                placed.insert(body.id);
                progress = true;
            }
        }
        if (!progress) {
            issue.errors.push_back("could not order body parents");
            return false;
        }
    }

    std::unordered_map<std::string, chrono::ChFrame<>> worldFrame;
    auto rollback = [&]() {
        for (auto& motor : out.motors) {
            if (motor && motor->GetChLink()) world.RemoveLink(motor->GetChLink());
        }
        for (auto& joint : out.joints) {
            if (joint && joint->GetChLink()) world.RemoveLink(joint->GetChLink());
        }
        for (auto& entry : out.bodies) world.RemoveBody(entry.second);
        out = {};
    };

    try {
    for (const AssemblyBody* spec : ordered) {
        chrono::ChFrame<> local(
            chrono::ChVector3d(spec->pose.px, spec->pose.py, spec->pose.pz),
            chrono::ChQuaterniond(spec->pose.qw, spec->pose.qx, spec->pose.qy, spec->pose.qz));
        chrono::ChFrame<> worldPose = local;
        if (!spec->parent.empty()) {
            worldPose = worldFrame[spec->parent] * local;
        }
        worldFrame[spec->id] = worldPose;

        double mass = spec->hasMass ? spec->mass : spec->density * SolidVolume(*spec);
        if (!(mass > 0.0)) {
            issue.errors.push_back("body '" + spec->id + "' mass must be positive");
            rollback();
            return false;
        }

        BodyTransform transform;
        transform.px = worldPose.GetPos().x();
        transform.py = worldPose.GetPos().y();
        transform.pz = worldPose.GetPos().z();
        transform.qw = worldPose.GetRot().e0();
        transform.qx = worldPose.GetRot().e1();
        transform.qy = worldPose.GetRot().e2();
        transform.qz = worldPose.GetRot().e3();

        Material material;
        material.friction = spec->friction;
        material.restitution = spec->restitution;

        const AssemblyShape solid = spec->shape == AssemblyShape::Mesh ? spec->collisionShape : spec->shape;
        std::shared_ptr<RigidBody> body;
        switch (solid) {
            case AssemblyShape::Sphere:
                body = RigidBody::CreateSphere(spec->radius, mass, transform, material);
                break;
            case AssemblyShape::Cylinder:
                body = RigidBody::CreateCylinder(spec->radius, spec->height, mass, transform, material);
                break;
            case AssemblyShape::Capsule:
                body = RigidBody::CreateCapsule(spec->radius, spec->height, mass, transform, material);
                break;
            case AssemblyShape::Box:
            case AssemblyShape::Mesh:
                body = RigidBody::CreateBox(spec->sizeX, spec->sizeY, spec->sizeZ, mass, transform, material);
                break;
        }
        if (!body) {
            issue.errors.push_back("failed to create body '" + spec->id + "'");
            rollback();
            return false;
        }
        body->SetName(spec->id);
        body->SetFixed(spec->fixedBody);
        if (!spec->collision) body->SetCollisionEnabled(false);
        body->SetCollisionFamily(kAssemblyFamily, false);
        body->SetColor(0.62f, 0.64f, 0.68f);
        if (spec->visual.HasMesh() || spec->visual.HasTexture() || spec->visual.colorR >= 0.0f) {
            body->ApplyVisual(spec->visual, true);
        }
        if (spec->id == previewBodyId) {
            AddCollisionPreview(*body->GetChBody(), *spec);
        }
        world.AddBody(body);
        out.bodies[spec->id] = body;
    }

    if (!graph.root.empty() && out.bodies.count(graph.root)) out.root = out.bodies[graph.root];
    else if (!graph.bodies.empty()) out.root = out.bodies[graph.bodies.front().id];

    std::unordered_map<std::string, const AssemblyMotor*> motorForJoint;
    for (const auto& motor : graph.motors) motorForJoint[motor.jointId] = &motor;

    for (const auto& spec : graph.joints) {
        auto bodyA = out.bodies[spec.bodyA];
        auto bodyB = out.bodies[spec.bodyB];
        if (!bodyA || !bodyB) {
            issue.errors.push_back("joint '" + spec.id + "' is missing a body");
            rollback();
            return false;
        }
        const chrono::ChFrame<>& frameA = worldFrame[spec.bodyA];
        chrono::ChVector3d anchor = frameA.TransformPointLocalToParent(
            chrono::ChVector3d(spec.anchorX, spec.anchorY, spec.anchorZ));
        chrono::ChVector3d localAxis(spec.axisX, spec.axisY, spec.axisZ);
        if (localAxis.Length() < 1e-8) localAxis = chrono::ChVector3d(0, 0, 1);
        localAxis.Normalize();
        chrono::ChVector3d axis = frameA.GetRot().Rotate(localAxis);
        if (axis.Length() > 1e-8) axis.Normalize();
        else axis = chrono::ChVector3d(0, 0, 1);

        const AssemblyMotor* motorSpec = nullptr;
        auto motorIt = motorForJoint.find(spec.id);
        if (motorIt != motorForJoint.end()) motorSpec = motorIt->second;

        if (motorSpec) {
            const bool linear = spec.type == AssemblyJointType::Prismatic;
            std::shared_ptr<Motor> motor;
            if (linear && motorSpec->mode == AssemblyMotorMode::Velocity) {
                motor = Motor::CreateLinearSpeedMotor(world, bodyA->GetChBody(), bodyB->GetChBody(), anchor, axis);
            } else if (linear && motorSpec->mode == AssemblyMotorMode::Position) {
                motor = Motor::CreateLinearPositionMotor(world, bodyA->GetChBody(), bodyB->GetChBody(), anchor, axis);
            } else if (linear && motorSpec->mode == AssemblyMotorMode::Force) {
                motor = Motor::CreateLinearForceMotor(world, bodyA->GetChBody(), bodyB->GetChBody(), anchor, axis);
            } else if (!linear && motorSpec->mode == AssemblyMotorMode::Velocity) {
                motor = Motor::CreateSpeedMotor(world, bodyA->GetChBody(), bodyB->GetChBody(), anchor, axis);
            } else if (!linear && motorSpec->mode == AssemblyMotorMode::Position) {
                motor = Motor::CreatePositionMotor(world, bodyA->GetChBody(), bodyB->GetChBody(), anchor, axis);
            } else if (!linear && motorSpec->mode == AssemblyMotorMode::Torque) {
                motor = Motor::CreateTorqueMotor(world, bodyA->GetChBody(), bodyB->GetChBody(), anchor, axis);
            }
            if (!motor) {
                issue.errors.push_back("motor '" + motorSpec->id + "' could not be created");
                rollback();
                return false;
            }
            motor->SetMaxSpeed(motorSpec->maxSpeed);
            motor->SetMaxTorque(motorSpec->maxTorque);
            out.motors.push_back(motor);
            out.motorIds.push_back(motorSpec->id);
            int drive = 0;
            if (motorSpec->drive == AssemblyDrive::Left) drive = 1;
            else if (motorSpec->drive == AssemblyDrive::Right) drive = 2;
            out.driveGroups.push_back(drive);
            continue;
        }

        std::shared_ptr<Joint> joint;
        switch (spec.type) {
            case AssemblyJointType::Fixed:
                joint = Joint::CreateFixed(world, bodyA->GetChBody(), bodyB->GetChBody());
                break;
            case AssemblyJointType::Prismatic:
                joint = Joint::CreatePrismatic(world, bodyA->GetChBody(), bodyB->GetChBody(), anchor, axis);
                break;
            case AssemblyJointType::Spherical:
                joint = Joint::CreateSpherical(world, bodyA->GetChBody(), bodyB->GetChBody(), anchor);
                break;
            case AssemblyJointType::Revolute:
                joint = Joint::CreateRevolute(world, bodyA->GetChBody(), bodyB->GetChBody(), anchor, axis);
                break;
        }
        if (joint) out.joints.push_back(joint);
    }
    } catch (...) {
        rollback();
        throw;
    }

    for (const auto& warning : issue.warnings) {
        std::cerr << "[Assembly] " << warning << std::endl;
    }
    std::cout << "[Assembly] Built '" << graph.name << "' bodies=" << out.bodies.size()
              << " joints=" << out.joints.size() << " motors=" << out.motors.size() << std::endl;
    return true;
}

}  // namespace robosim
