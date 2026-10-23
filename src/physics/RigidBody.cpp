#include "physics/RigidBody.h"

#include <chrono/physics/ChBodyEasy.h>
#include <chrono/assets/ChVisualShapeBox.h>
#include <chrono/assets/ChVisualShapeSphere.h>
#include <chrono/assets/ChVisualShapeCylinder.h>
#include <chrono/utils/ChConstants.h>

namespace robosim {

RigidBody::~RigidBody() = default;

std::shared_ptr<chrono::ChContactMaterial> RigidBody::CreateChMaterial(
    const Material& mat, bool isSMC)
{
    if (isSMC) {
        auto material = chrono::ChContactMaterial::DefaultMaterial(
            chrono::ChContactMethod::SMC);
        material->SetFriction(static_cast<float>(mat.friction));
        material->SetRestitution(static_cast<float>(mat.restitution));
        return material;
    } else {
        auto material = chrono::ChContactMaterial::DefaultMaterial(
            chrono::ChContactMethod::NSC);
        material->SetFriction(static_cast<float>(mat.friction));
        material->SetRestitution(static_cast<float>(mat.restitution));
        return material;
    }
}

std::shared_ptr<RigidBody> RigidBody::CreateBox(
    double sx, double sy, double sz,
    double mass,
    const BodyTransform& transform,
    const Material& material)
{
    auto rb = std::shared_ptr<RigidBody>(new RigidBody());
    rb->m_shape = BodyShape::Box;
    rb->m_name = "box";

    auto mat = CreateChMaterial(material, false);
    bool isFixed = (mass <= 0.0);
    double actualMass = isFixed ? 1.0 : mass;

    auto body = chrono_types::make_shared<chrono::ChBodyEasyBox>(
        sx, sy, sz, actualMass / (sx * sy * sz), true, true, mat);

    body->SetFixed(isFixed);
    if (isFixed) {
        body->SetMass(0);
    }

    body->SetPos(chrono::ChVector3d(transform.px, transform.py, transform.pz));
    body->SetRot(chrono::ChQuaterniond(transform.qw, transform.qx, transform.qy, transform.qz));

    rb->m_body = body;
    return rb;
}

std::shared_ptr<RigidBody> RigidBody::CreateSphere(
    double radius,
    double mass,
    const BodyTransform& transform,
    const Material& material)
{
    auto rb = std::shared_ptr<RigidBody>(new RigidBody());
    rb->m_shape = BodyShape::Sphere;
    rb->m_name = "sphere";

    auto mat = CreateChMaterial(material, false);
    bool isFixed = (mass <= 0.0);
    double actualMass = isFixed ? 1.0 : mass;
    double volume = (4.0 / 3.0) * chrono::CH_PI * radius * radius * radius;

    auto body = chrono_types::make_shared<chrono::ChBodyEasySphere>(
        radius, actualMass / volume, true, true, mat);

    body->SetFixed(isFixed);
    if (isFixed) {
        body->SetMass(0);
    }

    body->SetPos(chrono::ChVector3d(transform.px, transform.py, transform.pz));
    body->SetRot(chrono::ChQuaterniond(transform.qw, transform.qx, transform.qy, transform.qz));

    rb->m_body = body;
    return rb;
}

std::shared_ptr<RigidBody> RigidBody::CreateCylinder(
    double radius,
    double height,
    double mass,
    const BodyTransform& transform,
    const Material& material)
{
    auto rb = std::shared_ptr<RigidBody>(new RigidBody());
    rb->m_shape = BodyShape::Cylinder;
    rb->m_name = "cylinder";

    auto mat = CreateChMaterial(material, false);
    bool isFixed = (mass <= 0.0);
    double actualMass = isFixed ? 1.0 : mass;
    double volume = chrono::CH_PI * radius * radius * height;

    auto body = chrono_types::make_shared<chrono::ChBodyEasyCylinder>(
        chrono::ChAxis::Y, radius, height, actualMass / volume, true, true, mat);

    body->SetFixed(isFixed);
    if (isFixed) {
        body->SetMass(0);
    }

    body->SetPos(chrono::ChVector3d(transform.px, transform.py, transform.pz));
    body->SetRot(chrono::ChQuaterniond(transform.qw, transform.qx, transform.qy, transform.qz));

    rb->m_body = body;
    return rb;
}

std::shared_ptr<RigidBody> RigidBody::CreateGround(const Material& material) {
    auto rb = std::shared_ptr<RigidBody>(new RigidBody());
    rb->m_shape = BodyShape::Box;
    rb->m_name = "ground";

    auto mat = CreateChMaterial(material, false);

    auto body = chrono_types::make_shared<chrono::ChBodyEasyBox>(
        200.0, 0.2, 200.0, 1000.0, true, true, mat);

    body->SetFixed(true);
    body->SetPos(chrono::ChVector3d(0, -0.1, 0));
    body->SetName("ground");

    rb->m_body = body;
    return rb;
}

void RigidBody::SetPosition(double x, double y, double z) {
    m_body->SetPos(chrono::ChVector3d(x, y, z));
}

void RigidBody::GetPosition(double& x, double& y, double& z) const {
    auto pos = m_body->GetPos();
    x = pos.x();
    y = pos.y();
    z = pos.z();
}

void RigidBody::SetRotation(double qw, double qx, double qy, double qz) {
    m_body->SetRot(chrono::ChQuaterniond(qw, qx, qy, qz));
}

void RigidBody::GetRotation(double& qw, double& qx, double& qy, double& qz) const {
    auto rot = m_body->GetRot();
    qw = rot.e0();
    qx = rot.e1();
    qy = rot.e2();
    qz = rot.e3();
}

void RigidBody::SetLinearVelocity(double vx, double vy, double vz) {
    m_body->SetLinVel(chrono::ChVector3d(vx, vy, vz));
}

void RigidBody::GetLinearVelocity(double& vx, double& vy, double& vz) const {
    auto vel = m_body->GetLinVel();
    vx = vel.x();
    vy = vel.y();
    vz = vel.z();
}

void RigidBody::SetAngularVelocity(double wx, double wy, double wz) {
    m_body->SetAngVelLocal(chrono::ChVector3d(wx, wy, wz));
}

void RigidBody::SetFixed(bool fixed) {
    m_body->SetFixed(fixed);
}

bool RigidBody::IsFixed() const {
    return m_body->IsFixed();
}

void RigidBody::SetName(const std::string& name) {
    m_name = name;
    m_body->SetName(name);
}

double RigidBody::GetMass() const {
    return m_body->GetMass();
}

void RigidBody::ApplyForce(double fx, double fy, double fz) {
    m_body->EmptyAccumulator(0);
    m_body->AccumulateForce(0, chrono::ChVector3d(fx, fy, fz), chrono::ChVector3d(0, 0, 0), false);
}

void RigidBody::ApplyTorque(double tx, double ty, double tz) {
    m_body->AccumulateTorque(0, chrono::ChVector3d(tx, ty, tz), false);
}

}  // namespace robosim
