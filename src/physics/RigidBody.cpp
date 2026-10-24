#include "physics/RigidBody.h"
#include "physics/Assets.h"

#include <chrono/physics/ChBodyEasy.h>
#include <chrono/assets/ChVisualShapeBox.h>
#include <chrono/assets/ChVisualShapeSphere.h>
#include <chrono/assets/ChVisualShapeCylinder.h>
#include <chrono/assets/ChVisualShapeCapsule.h>
#include <chrono/collision/ChCollisionShapeCapsule.h>
#include <chrono/physics/ChBody.h>
#include <chrono/assets/ChVisualModel.h>
#include <chrono/assets/ChVisualMaterial.h>
#include <chrono/assets/ChColor.h>
#include <chrono/collision/ChCollisionModel.h>
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

std::shared_ptr<RigidBody> RigidBody::CreateCapsule(
    double radius,
    double height,
    double mass,
    const BodyTransform& transform,
    const Material& material)
{
    auto rb = std::shared_ptr<RigidBody>(new RigidBody());
    rb->m_shape = BodyShape::Capsule;
    rb->m_name = "capsule";

    auto mat = CreateChMaterial(material, false);
    bool isFixed = (mass <= 0.0);
    double actualMass = isFixed ? 1.0 : mass;

    // Chrono capsules are aligned with local Z. Tilt that axis onto Y so a
    // capsule matches CreateCylinder, which uses ChAxis::Y.
    // Inertia of a solid capsule about its center:
    //   cylinder mass m_c, sphere mass m_s (both hemispheres)
    //   I_axial = 0.5 m_c r^2 + 0.4 m_s r^2
    //   I_perp  = m_c (r^2/4 + h^2/12) + m_s (2 r^2/5 + h^2/4 + 3 h r/8)
    const double volume = chrono::CH_PI * radius * radius * height
        + (4.0 / 3.0) * chrono::CH_PI * radius * radius * radius;
    const double density = actualMass / volume;
    const double cylinderMass = density * chrono::CH_PI * radius * radius * height;
    const double sphereMass = density * (4.0 / 3.0) * chrono::CH_PI * radius * radius * radius;
    const double iAxial = 0.5 * cylinderMass * radius * radius + 0.4 * sphereMass * radius * radius;
    const double iPerp = cylinderMass * (radius * radius / 4.0 + height * height / 12.0)
        + sphereMass * (0.4 * radius * radius + height * height / 4.0 + 0.375 * height * radius);

    auto body = chrono_types::make_shared<chrono::ChBody>();
    body->SetMass(actualMass);
    body->SetInertiaXX(chrono::ChVector3d(iPerp, iAxial, iPerp));
    auto collision = chrono_types::make_shared<chrono::ChCollisionShapeCapsule>(mat, radius, height);
    auto visual = chrono_types::make_shared<chrono::ChVisualShapeCapsule>(radius, height);
    chrono::ChFrame<> tilt(chrono::ChVector3d(0, 0, 0), chrono::QuatFromAngleX(-chrono::CH_PI_2));
    body->AddCollisionShape(collision, tilt);
    body->AddVisualShape(visual, tilt);
    body->EnableCollision(true);

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
    rb->SetColor(0.78f, 0.76f, 0.72f);
    VisualSpec groundVis;
    groundVis.texture = "textures/studio_floor.png";
    groundVis.texScaleX = 25.0f;
    groundVis.texScaleY = 25.0f;
    groundVis.colorR = 0.78f;
    groundVis.colorG = 0.76f;
    groundVis.colorB = 0.72f;
    groundVis.metallic = 0.0f;
    groundVis.roughness = 0.88f;
    rb->ApplyVisual(groundVis, false);
    return rb;
}

std::shared_ptr<RigidBody> RigidBody::Wrap(std::shared_ptr<chrono::ChBody> body, BodyShape shape) {
    auto rb = std::shared_ptr<RigidBody>(new RigidBody());
    rb->m_shape = shape;
    rb->m_name = body ? body->GetName() : "body";
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

void RigidBody::SetColor(float r, float g, float b) {
    if (!m_body) return;
    auto vis = m_body->GetVisualModel();
    if (!vis) return;
    auto color = chrono::ChColor(r, g, b);
    for (unsigned int i = 0; i < vis->GetNumShapes(); ++i) {
        auto shape = vis->GetShape(i);
        shape->SetColor(color);
        shape->SetOpacity(1.0f);
    }
}

void RigidBody::SetCollisionEnabled(bool enabled) {
    if (!m_body) return;
    m_body->EnableCollision(enabled);
}

void RigidBody::SetCollisionFamily(int family, bool collideWithSameFamily) {
    if (!m_body) return;
    auto model = m_body->GetCollisionModel();
    if (!model) return;
    model->SetFamily(family);
    if (!collideWithSameFamily) {
        model->DisallowCollisionsWith(family);
    }
}

void RigidBody::AddVisualMesh(const std::string& path, double scale,
                              double ox, double oy, double oz) {
    VisualSpec spec;
    spec.mesh = path;
    spec.scaleX = spec.scaleY = spec.scaleZ = scale;
    spec.offsetX = ox;
    spec.offsetY = oy;
    spec.offsetZ = oz;
    ApplyVisual(spec, true);
}

void RigidBody::ApplyVisual(const VisualSpec& spec, bool replaceDefault) {
    if (!m_body) return;
    ApplyVisualSpec(*m_body, spec, replaceDefault);
}

void RigidBody::SetPbr(float metallic, float roughness) {
    if (!m_body) return;
    auto vis = m_body->GetVisualModel();
    if (!vis) return;
    for (unsigned int i = 0; i < vis->GetNumShapes(); ++i) {
        auto shape = vis->GetShape(i);
        if (shape->GetNumMaterials() == 0) {
            auto mat = MakeVisualMaterial(0.6f, 0.6f, 0.6f, metallic, roughness);
            shape->AddMaterial(mat);
        } else {
            auto mat = shape->GetMaterial(0);
            mat->SetMetallic(metallic);
            mat->SetRoughness(roughness);
            mat->SetUseSpecularWorkflow(false);
        }
    }
}

}  // namespace robosim
