#include "core/PhysicsWorld.h"
#include "physics/ChronoEngine.h"
#include "physics/RigidBody.h"

#include <chrono/collision/ChCollisionSystem.h>
#include <algorithm>

namespace robosim {

PhysicsWorld::PhysicsWorld()
    : m_engine(std::make_unique<ChronoEngine>(ContactMethod::NSC))
{
}

PhysicsWorld::~PhysicsWorld() = default;

void PhysicsWorld::Step() {
    m_engine->DoStep();
}

void PhysicsWorld::Step(double dt) {
    m_engine->DoStep(dt);
}

double PhysicsWorld::GetTime() const {
    return m_engine->GetTime();
}

double PhysicsWorld::GetTimestep() const {
    return m_engine->GetTimestep();
}

void PhysicsWorld::SetTimestep(double dt) {
    m_engine->SetTimestep(dt);
}

void PhysicsWorld::SetGravity(double x, double y, double z) {
    m_engine->SetGravity(x, y, z);
}

void PhysicsWorld::AddBody(std::shared_ptr<RigidBody> body) {
    auto chbody = body->GetChBody();
    m_engine->AddBody(chbody);
    m_bodies.push_back(body);

    // After the first physics step Chrono has initialized Bullet, and new
    // shapes are ignored until they are bound. That is why Robot-menu swaps
    // were falling through the visible ground.
    if (auto cs = m_engine->GetSystem().GetCollisionSystem()) {
        if (cs->IsInitialized()) {
            cs->BindItem(chbody);
        }
    }
}

void PhysicsWorld::RemoveBody(std::shared_ptr<RigidBody> body) {
    m_engine->RemoveBody(body->GetChBody());
    m_bodies.erase(
        std::remove(m_bodies.begin(), m_bodies.end(), body),
        m_bodies.end()
    );
}

void PhysicsWorld::AddLink(std::shared_ptr<chrono::ChLinkBase> link) {
    m_engine->AddLink(link);
}

void PhysicsWorld::RemoveLink(std::shared_ptr<chrono::ChLinkBase> link) {
    m_engine->RemoveLink(link);
}

void PhysicsWorld::Clear() {
    m_bodies.clear();
    m_engine->Clear();
}

chrono::ChSystem& PhysicsWorld::GetChSystem() {
    return m_engine->GetSystem();
}

const chrono::ChSystem& PhysicsWorld::GetChSystem() const {
    return m_engine->GetSystem();
}

}  // namespace robosim
