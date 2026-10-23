#include "rendering/Renderer.h"
#include "core/PhysicsWorld.h"
#include "physics/ChronoEngine.h"

#include <chrono_irrlicht/ChVisualSystemIrrlicht.h>

using namespace chrono;
using namespace chrono::irrlicht;

namespace robosim {

Renderer::Renderer() = default;
Renderer::~Renderer() = default;

void Renderer::Initialize(PhysicsWorld& world, const RendererConfig& config) {
    m_vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    m_vis->AttachSystem(&world.GetChSystem());
    m_vis->SetWindowSize(config.windowWidth, config.windowHeight);
    m_vis->SetWindowTitle(config.windowTitle);
    m_vis->SetCameraVertical(CameraVerticalDir::Y);
    m_vis->Initialize();

    m_vis->AddCamera(
        ChVector3d(config.cameraX, config.cameraY, config.cameraZ),
        ChVector3d(config.lookAtX, config.lookAtY, config.lookAtZ)
    );

    m_vis->AddTypicalLights();

    m_initialized = true;
}

bool Renderer::IsRunning() const {
    if (!m_initialized || !m_vis) return false;
    return m_vis->Run();
}

void Renderer::BeginFrame() {
    if (m_vis) {
        m_vis->BeginScene();
    }
}

void Renderer::EndFrame() {
    if (m_vis) {
        m_vis->EndScene();
    }
}

void Renderer::Render() {
    if (m_vis) {
        m_vis->Render();
    }
}

void Renderer::SetWindowTitle(const std::string& title) {
    if (m_vis) {
        m_vis->SetWindowTitle(title);
    }
}

bool Renderer::IsKeyPressed(int key) const {
    // Irrlicht key checking would be done through event receivers
    // This is a simplified interface; full keyboard handling comes in Phase 2
    return false;
}

}  // namespace robosim
