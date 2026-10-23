#include "physics/ChronoEngine.h"

#include <chrono/solver/ChSolverPSOR.h>
#include <chrono/collision/ChCollisionSystem.h>

namespace robosim {

ChronoEngine::ChronoEngine(ContactMethod method)
    : m_contactMethod(method)
{
    if (method == ContactMethod::SMC) {
        m_system = std::make_unique<chrono::ChSystemSMC>();
    } else {
        m_system = std::make_unique<chrono::ChSystemNSC>();
    }

    m_system->SetCollisionSystemType(chrono::ChCollisionSystem::Type::BULLET);
    m_system->SetGravitationalAcceleration(chrono::ChVector3d(0, -9.81, 0));

    if (method == ContactMethod::NSC) {
        auto solver = std::make_shared<chrono::ChSolverPSOR>();
        solver->SetMaxIterations(150);
        m_system->SetSolver(solver);
    }

    m_system->SetTimestepperType(chrono::ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
}

ChronoEngine::~ChronoEngine() = default;

void ChronoEngine::SetGravity(double x, double y, double z) {
    m_system->SetGravitationalAcceleration(chrono::ChVector3d(x, y, z));
}

void ChronoEngine::SetTimestep(double dt) {
    m_timestep = dt;
}

void ChronoEngine::DoStep() {
    m_system->DoStepDynamics(m_timestep);
}

void ChronoEngine::DoStep(double dt) {
    m_system->DoStepDynamics(dt);
}

double ChronoEngine::GetTime() const {
    return m_system->GetChTime();
}

void ChronoEngine::SetSolverMaxIterations(int iterations) {
    auto solver = std::dynamic_pointer_cast<chrono::ChSolverPSOR>(m_system->GetSolver());
    if (solver) {
        solver->SetMaxIterations(iterations);
    }
}

void ChronoEngine::SetCollisionEnvelope(double envelope) {
    // In Chrono 10 collision envelope is set per-shape, not on the system
    (void)envelope;
}

void ChronoEngine::AddBody(std::shared_ptr<chrono::ChBody> body) {
    m_system->AddBody(body);
}

void ChronoEngine::RemoveBody(std::shared_ptr<chrono::ChBody> body) {
    m_system->RemoveBody(body);
}

void ChronoEngine::AddLink(std::shared_ptr<chrono::ChLinkBase> link) {
    m_system->AddLink(link);
}

void ChronoEngine::RemoveLink(std::shared_ptr<chrono::ChLinkBase> link) {
    m_system->RemoveLink(link);
}

void ChronoEngine::Clear() {
    m_system->Clear();
}

}  // namespace robosim
