#pragma once

#include "physics/Assets.h"

#include <memory>
#include <string>
#include <chrono/physics/ChBody.h>
#include <chrono/physics/ChBodyEasy.h>

namespace robosim {

struct Material {
    double friction = 0.5;
    double restitution = 0.1;
    double young_modulus = 2e7;
    double poisson_ratio = 0.3;
};

struct BodyTransform {
    double px = 0, py = 0, pz = 0;        // position
    double qw = 1, qx = 0, qy = 0, qz = 0; // quaternion rotation
};

enum class BodyShape {
    Box,
    Sphere,
    Cylinder,
    Capsule,
    Custom
};

class RigidBody {
public:
    ~RigidBody();

    RigidBody(const RigidBody&) = delete;
    RigidBody& operator=(const RigidBody&) = delete;
    RigidBody(RigidBody&&) = default;
    RigidBody& operator=(RigidBody&&) = default;

    // Factory methods
    static std::shared_ptr<RigidBody> CreateBox(
        double sx, double sy, double sz,
        double mass,
        const BodyTransform& transform = {},
        const Material& material = {}
    );

    static std::shared_ptr<RigidBody> CreateSphere(
        double radius,
        double mass,
        const BodyTransform& transform = {},
        const Material& material = {}
    );

    static std::shared_ptr<RigidBody> CreateCylinder(
        double radius,
        double height,
        double mass,
        const BodyTransform& transform = {},
        const Material& material = {}
    );

    // Capsule axis is local Y. height is the cylindrical section.
    // Inertia is the solid-capsule formula: cylinder plus two hemispheres.
    static std::shared_ptr<RigidBody> CreateCapsule(
        double radius,
        double height,
        double mass,
        const BodyTransform& transform = {},
        const Material& material = {}
    );

    static std::shared_ptr<RigidBody> CreateGround(
        const Material& material = {}
    );

    static std::shared_ptr<RigidBody> Wrap(std::shared_ptr<chrono::ChBody> body, BodyShape shape = BodyShape::Custom);

    // Accessors
    std::shared_ptr<chrono::ChBody> GetChBody() const { return m_body; }

    void SetPosition(double x, double y, double z);
    void GetPosition(double& x, double& y, double& z) const;

    void SetRotation(double qw, double qx, double qy, double qz);
    void GetRotation(double& qw, double& qx, double& qy, double& qz) const;

    void SetLinearVelocity(double vx, double vy, double vz);
    void GetLinearVelocity(double& vx, double& vy, double& vz) const;

    void SetAngularVelocity(double wx, double wy, double wz);

    void SetFixed(bool fixed);
    bool IsFixed() const;

    void SetName(const std::string& name);
    const std::string& GetName() const { return m_name; }

    BodyShape GetShape() const { return m_shape; }
    double GetMass() const;

    void ApplyForce(double fx, double fy, double fz);
    void ApplyTorque(double tx, double ty, double tz);

    void SetColor(float r, float g, float b);
    void SetCollisionFamily(int family, bool collideWithSameFamily = false);
    void SetCollisionEnabled(bool enabled);

    void AddVisualMesh(const std::string& path, double scale = 1.0,
                       double ox = 0.0, double oy = 0.0, double oz = 0.0);
    void ApplyVisual(const VisualSpec& spec, bool replaceDefault = true);
    void SetPbr(float metallic, float roughness);

private:
    RigidBody() = default;

    static std::shared_ptr<chrono::ChContactMaterial> CreateChMaterial(
        const Material& mat, bool isSMC
    );

    std::shared_ptr<chrono::ChBody> m_body;
    std::string m_name;
    BodyShape m_shape = BodyShape::Custom;
};

}  // namespace robosim
