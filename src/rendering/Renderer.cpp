#include "rendering/Renderer.h"
#include "rendering/EditorUI.h"
#include "rendering/KeyCodes.h"
#include "core/PhysicsWorld.h"
#include "physics/ChronoEngine.h"

#include <chrono_vsg/ChVisualSystemVSG.h>
#include <chrono_vsg/ChGuiComponentVSG.h>
#include <chrono_vsg/ChEventHandlerVSG.h>
#include <chrono/core/ChRotation.h>
#include <chrono/utils/ChConstants.h>

#include <vsg/ui/KeyEvent.h>
#include <vsg/ui/PointerEvent.h>

#include <algorithm>
#include <cmath>
#include <iostream>

using namespace chrono;
using namespace chrono::vsg3d;

namespace robosim {

class SimGuiComponentVSG : public ChGuiComponentVSG {
public:
    explicit SimGuiComponentVSG(EditorUI* editor) : m_editor(editor) {}

    void render(vsg::CommandBuffer& /*cb*/) override {
        if (m_editor) {
            m_editor->Render();
        }
    }

private:
    EditorUI* m_editor = nullptr;
};

class SimEventHandlerVSG : public ChEventHandlerVSG {
public:
    explicit SimEventHandlerVSG(Renderer& renderer) : m_renderer(renderer) {}

    void process(vsg::KeyPressEvent& ev) override {
        const int key = static_cast<int>(ev.keyBase);
        m_renderer.m_keyState[key] = true;
        if (m_renderer.m_keyCallback) {
            m_renderer.m_keyCallback(key, true);
        }
        (void)Key::IsDriveKey(key);  // drive keys tracked; VSG trackball is mouse-only
    }

    void process(vsg::KeyReleaseEvent& ev) override {
        const int key = static_cast<int>(ev.keyBase);
        m_renderer.m_keyState[key] = false;
        if (m_renderer.m_keyCallback) {
            m_renderer.m_keyCallback(key, false);
        }
    }

    void process(vsg::ButtonPressEvent& ev) override {
        if (ev.button == 1) m_renderer.m_mouseLeft = true;
        if (ev.button == 3) m_renderer.m_mouseRight = true;
        if (m_renderer.m_mouseCallback) {
            m_renderer.m_mouseCallback(
                static_cast<int>(ev.x), static_cast<int>(ev.y),
                m_renderer.m_mouseLeft, m_renderer.m_mouseRight, 0.0f);
        }
    }

    void process(vsg::ButtonReleaseEvent& ev) override {
        if (ev.button == 1) m_renderer.m_mouseLeft = false;
        if (ev.button == 3) m_renderer.m_mouseRight = false;
        if (m_renderer.m_mouseCallback) {
            m_renderer.m_mouseCallback(
                static_cast<int>(ev.x), static_cast<int>(ev.y),
                m_renderer.m_mouseLeft, m_renderer.m_mouseRight, 0.0f);
        }
    }

    void process(vsg::MoveEvent& ev) override {
        if (m_renderer.m_mouseCallback) {
            const bool left = (ev.mask & vsg::BUTTON_MASK_1) != 0;
            const bool right = (ev.mask & vsg::BUTTON_MASK_3) != 0;
            m_renderer.m_mouseLeft = left;
            m_renderer.m_mouseRight = right;
            m_renderer.m_mouseCallback(
                static_cast<int>(ev.x), static_cast<int>(ev.y), left, right, 0.0f);
        }
    }

private:
    Renderer& m_renderer;
};

Renderer::Renderer() = default;

Renderer::~Renderer() {
    ShutdownImGui();
}

void Renderer::ShutdownImGui() {
    // ImGui lifetime is owned by Chrono::VSG / vsgImGui; nothing to tear down here.
}

void Renderer::Initialize(PhysicsWorld& world, const RendererConfig& config, EditorUI* editor) {
    m_vis = chrono_types::make_shared<ChVisualSystemVSG>();
    m_vis->AttachSystem(&world.GetChSystem());

    m_windowWidth = config.windowWidth;
    m_windowHeight = config.windowHeight;
    m_showGrid = config.showGrid;

    m_vis->SetWindowSize(config.windowWidth, config.windowHeight);
    m_vis->SetWindowTitle(config.windowTitle);
    m_vis->SetWindowPosition(60, 60);
    m_vis->SetCameraVertical(CameraVerticalDir::Y);
    // Dome panorama gives a readable horizon without depending on cubemap load paths.
    m_vis->SetBackgroundColor(ChColor(0.40f, 0.48f, 0.56f));
    m_vis->EnableSkyTexture(SkyMode::DOME);
    m_vis->SetCameraAngleDeg(46.0);
    m_vis->SetLightIntensity(1.65f);
    // Soft key from upper-front-right; fills the robot without blowing out materials.
    m_vis->SetLightDirection(1.00 * CH_PI_2, 0.78);
    // Shadows currently crash in recordAndSubmit() on this NVIDIA driver + VSG
    // 1.1.14 stack (0xC0000005). Keep off; lighting/sky/PBR still apply.
    m_vis->EnableShadows(false);
    m_vis->SetVerbose(false);
    // Product mode: hide Chrono debug chrome before Initialize builds the base GUI.
    m_vis->SetBaseGuiVisibility(false);
    m_vis->HideLogo();

    m_vis->AddCamera(
        ChVector3d(config.cameraX, config.cameraY, config.cameraZ),
        ChVector3d(config.lookAtX, config.lookAtY, config.lookAtZ));

    // Optional faint grid (off by default — textured floor carries the scene).
    if (config.showGrid) {
        m_vis->AddGrid(
            2.0, 2.0, 12, 12,
            ChCoordsys<>(ChVector3d(0, 0.003, 0), QuatFromAngleX(CH_PI_2)),
            ChColor(0.48f, 0.50f, 0.54f));
    }

    // GUI + input must be registered before Initialize().
    if (editor) {
        m_guiComponent = chrono_types::make_shared<SimGuiComponentVSG>(editor);
        m_vis->AddGuiComponent(m_guiComponent);
    }

    m_eventHandler = chrono_types::make_shared<SimEventHandlerVSG>(*this);
    m_vis->AddEventHandler(m_eventHandler);

    std::cout << "[Renderer] Initializing Chrono::VSG..." << std::endl;
    m_vis->Initialize();
    ApplyProductVisualMode();
    std::cout << "[Renderer] Chrono::VSG ready (product visual mode)" << std::endl;

    m_initialized = true;
}

void Renderer::ApplyProductVisualMode() {
    if (!m_vis) return;
    m_vis->SetBaseGuiVisibility(false);
    m_vis->HideLogo();
    m_vis->SetBodyObjVisibility(true);
    m_vis->SetCollisionVisibility(false);
    m_vis->SetContactNormalsVisibility(false);
    m_vis->SetContactForcesVisibility(false);
}

void Renderer::UpdateFollowCamera(double targetX, double targetY, double targetZ, double dt,
                                  bool enableFollow) {
    if (!m_vis || !m_initialized) return;

    auto eye = m_vis->GetCameraPosition();
    auto target = m_vis->GetCameraTarget();

    ChVector3d desired(targetX, std::max(0.18, targetY), targetZ);
    ChVector3d newTarget = target;
    if (enableFollow && m_followEnabled) {
        const double alpha = 1.0 - std::exp(-m_followStrength * std::max(dt, 1e-4));
        newTarget = target + (desired - ChVector3d(target.x(), target.y(), target.z())) * alpha;
        // Keep a stable look height so orbit pitch does not fight vertical bobbing.
        newTarget.y() = target.y() + (desired.y() - target.y()) * (alpha * 0.35);
        newTarget.y() = std::max(0.12, newTarget.y());
    }

    ChVector3d offset = ChVector3d(eye.x(), eye.y(), eye.z()) - ChVector3d(target.x(), target.y(), target.z());
    if (enableFollow && m_followEnabled) {
        // Translate the eye with the target so orbit framing stays put while the robot drives.
        eye = newTarget + offset;
    } else {
        newTarget = ChVector3d(target.x(), target.y(), target.z());
    }

    ChVector3d newEye(eye.x(), eye.y(), eye.z());
    ChVector3d dir = newEye - newTarget;
    double dist = dir.Length();
    if (dist < 1e-6) {
        dir = ChVector3d(-1.5, 1.0, -1.5);
        dist = dir.Length();
    }
    dir = dir / dist;
    dist = std::clamp(dist, m_minDistance, m_maxDistance);

    const double horiz = std::sqrt(dir.x() * dir.x() + dir.z() * dir.z());
    double pitch = std::atan2(dir.y(), std::max(horiz, 1e-6));
    pitch = std::clamp(pitch, m_minPitch, m_maxPitch);
    const double yaw = std::atan2(dir.z(), dir.x());
    const double ch = std::cos(pitch);
    dir = ChVector3d(std::cos(yaw) * ch, std::sin(pitch), std::sin(yaw) * ch);
    newEye = newTarget + dir * dist;
    newEye.y() = std::max(m_minEyeHeight, newEye.y());
    newTarget.y() = std::max(0.05, newTarget.y());

    // If clamping the eye height changed pitch, rebuild from yaw/distance again.
    dir = newEye - newTarget;
    dist = std::clamp(dir.Length(), m_minDistance, m_maxDistance);
    if (dist > 1e-6) {
        dir = dir / dir.Length();
        const double h2 = std::sqrt(dir.x() * dir.x() + dir.z() * dir.z());
        pitch = std::clamp(std::atan2(dir.y(), std::max(h2, 1e-6)), m_minPitch, m_maxPitch);
        const double yaw2 = std::atan2(dir.z(), dir.x());
        const double ch2 = std::cos(pitch);
        dir = ChVector3d(std::cos(yaw2) * ch2, std::sin(pitch), std::sin(yaw2) * ch2);
        newEye = newTarget + dir * dist;
        newEye.y() = std::max(m_minEyeHeight, newEye.y());
    }

    m_vis->SetCameraPosition(newEye);
    m_vis->SetCameraTarget(newTarget);
}

void Renderer::EnsureMeshesCompiled() {
    // Chrono::VSG ShapeBuilder compiles during CreatePbr*/CreateTrimesh* when
    // BindAll runs after Initialize (compileTraversal is assigned). No extra hook
    // is required beyond re-asserting product visibility after bind.
    ApplyProductVisualMode();
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

void Renderer::BindAll() {
    if (m_vis) {
        m_vis->BindAll();
        EnsureMeshesCompiled();
    }
}

void Renderer::UnbindItem(const std::shared_ptr<chrono::ChBody>& body) {
    if (m_vis && body) {
        m_vis->UnbindItem(body);
    }
}

void Renderer::SetShowGrid(bool show) {
    m_showGrid = show;
}

void Renderer::SetWindowTitle(const std::string& title) {
    if (m_vis) {
        m_vis->SetWindowTitle(title);
    }
}

void Renderer::SetKeyCallback(KeyCallback callback) {
    m_keyCallback = std::move(callback);
}

bool Renderer::IsKeyPressed(int key) const {
    auto it = m_keyState.find(key);
    return it != m_keyState.end() && it->second;
}

bool Renderer::GetCameraLook(double& ex, double& ey, double& ez,
                             double& tx, double& ty, double& tz) const {
    if (!m_vis) return false;
    auto eye = m_vis->GetCameraPosition();
    auto target = m_vis->GetCameraTarget();
    ex = eye.x(); ey = eye.y(); ez = eye.z();
    tx = target.x(); ty = target.y(); tz = target.z();
    return true;
}

int Renderer::GetWindowWidth() const {
    return m_windowWidth;
}

int Renderer::GetWindowHeight() const {
    return m_windowHeight;
}

}  // namespace robosim
