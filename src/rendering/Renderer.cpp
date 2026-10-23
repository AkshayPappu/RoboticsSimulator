#include "rendering/Renderer.h"
#include "core/PhysicsWorld.h"
#include "physics/ChronoEngine.h"

#include <chrono_irrlicht/ChVisualSystemIrrlicht.h>
#include <chrono/core/ChRotation.h>
#include <chrono/utils/ChConstants.h>
#include <irrlicht.h>

#include <imgui.h>
#include <imgui_impl_opengl3.h>

#include <array>
#include <iostream>

using namespace chrono;
using namespace chrono::irrlicht;

namespace robosim {

class SimEventReceiver : public irr::IEventReceiver {
public:
    SimEventReceiver(Renderer& renderer) : m_renderer(renderer) {
        m_keyState.fill(false);
    }

    bool OnEvent(const irr::SEvent& event) override {
        // Feed events to ImGui
        ImGuiIO& io = ImGui::GetIO();

        if (event.EventType == irr::EET_KEY_INPUT_EVENT) {
            int key = event.KeyInput.Key;
            bool pressed = event.KeyInput.PressedDown;

            if (key >= 0 && key < (int)m_keyState.size()) {
                m_keyState[key] = pressed;
            }

            // Feed to ImGui
            if (key < 512) {
                io.KeysDown[key] = pressed;
            }
            io.KeyShift = event.KeyInput.Shift;
            io.KeyCtrl = event.KeyInput.Control;

            if (pressed && event.KeyInput.Char != 0) {
                io.AddInputCharacter(event.KeyInput.Char);
            }

            const bool driveKey =
                key == irr::KEY_UP || key == irr::KEY_DOWN ||
                key == irr::KEY_LEFT || key == irr::KEY_RIGHT ||
                key == irr::KEY_KEY_W || key == irr::KEY_KEY_A ||
                key == irr::KEY_KEY_S || key == irr::KEY_KEY_D;

            // Only trap keys when typing into a text field — never steal drive keys
            if (io.WantTextInput && !driveKey) {
                return false;
            }

            if (m_renderer.m_keyCallback) {
                m_renderer.m_keyCallback(key, pressed);
            }
            // Consume WASD/arrows so the Irrlicht camera does not steal driving
            return driveKey;
        }

        if (event.EventType == irr::EET_MOUSE_INPUT_EVENT) {
            io.MousePos = ImVec2((float)event.MouseInput.X, (float)event.MouseInput.Y);

            switch (event.MouseInput.Event) {
                case irr::EMIE_LMOUSE_PRESSED_DOWN: io.MouseDown[0] = true; break;
                case irr::EMIE_LMOUSE_LEFT_UP:      io.MouseDown[0] = false; break;
                case irr::EMIE_RMOUSE_PRESSED_DOWN:  io.MouseDown[1] = true; break;
                case irr::EMIE_RMOUSE_LEFT_UP:       io.MouseDown[1] = false; break;
                case irr::EMIE_MMOUSE_PRESSED_DOWN:  io.MouseDown[2] = true; break;
                case irr::EMIE_MMOUSE_LEFT_UP:       io.MouseDown[2] = false; break;
                case irr::EMIE_MOUSE_WHEEL:
                    io.MouseWheel = event.MouseInput.Wheel;
                    break;
                default: break;
            }

            if (io.WantCaptureMouse) {
                return true;
            }

            if (m_renderer.m_mouseCallback) {
                m_renderer.m_mouseCallback(
                    event.MouseInput.X, event.MouseInput.Y,
                    event.MouseInput.isLeftPressed(),
                    event.MouseInput.isRightPressed(),
                    (event.MouseInput.Event == irr::EMIE_MOUSE_WHEEL) ? event.MouseInput.Wheel : 0.0f
                );
            }
            return false;
        }

        return false;
    }

    bool IsKeyDown(int key) const {
        if (key >= 0 && key < (int)m_keyState.size()) {
            return m_keyState[key];
        }
        return false;
    }

private:
    Renderer& m_renderer;
    std::array<bool, 256> m_keyState;
};

Renderer::Renderer() = default;

Renderer::~Renderer() {
    ShutdownImGui();
}

void Renderer::ShutdownImGui() {
    if (!m_imguiInitialized) return;
    // Irrlicht may have already destroyed the GL context (window closed).
    // Skip the OpenGL backend teardown in that case to avoid a segfault.
    bool deviceAlive = m_vis && m_vis->GetDevice();
    if (deviceAlive) {
        ImGui_ImplOpenGL3_Shutdown();
    }
    ImGui::DestroyContext();
    m_imguiInitialized = false;
}

void Renderer::Initialize(PhysicsWorld& world, const RendererConfig& config) {
    m_vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();

    m_vis->AttachSystem(&world.GetChSystem());

    m_windowWidth = config.windowWidth;
    m_windowHeight = config.windowHeight;
    m_vis->SetAntialias(true);
    m_vis->SetWindowSize(config.windowWidth, config.windowHeight);
    m_vis->SetWindowTitle(config.windowTitle);
    m_vis->SetCameraVertical(CameraVerticalDir::Y);
    m_vis->SetBackgroundColor(ChColor(0.58f, 0.68f, 0.78f));

    m_vis->Initialize();

    m_vis->AddCamera(
        ChVector3d(config.cameraX, config.cameraY, config.cameraZ),
        ChVector3d(config.lookAtX, config.lookAtY, config.lookAtZ)
    );

    m_vis->AddSkyBox();
    // One warm key + one cool fill. AddTypicalLights() stacked on this washes meshes white.
    m_vis->AddLightDirectional(
        48.0, 38.0,
        ChColor(0.42f, 0.44f, 0.48f),
        ChColor(0.16f, 0.16f, 0.14f),
        ChColor(1.00f, 0.95f, 0.86f)
    );
    m_vis->AddLight(ChVector3d(-5.0, 6.0, 4.0), 14.0, ChColor(0.40f, 0.48f, 0.58f));
    m_vis->AddGrid(
        1.0, 1.0, 24, 24,
        ChCoordsys<>(ChVector3d(0, 0.002, 0), QuatFromAngleX(-CH_PI_2)),
        ChColor(0.50f, 0.54f, 0.58f)
    );

    // Register event receiver for keyboard/mouse input
    m_eventReceiver = std::make_unique<SimEventReceiver>(*this);
    m_vis->AddUserEventReceiver(m_eventReceiver.get());

    // Initialize Dear ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.DisplaySize = ImVec2((float)config.windowWidth, (float)config.windowHeight);
    ImGui::StyleColorsDark();
    ImGuiStyle& style = ImGui::GetStyle();
    style.Alpha = 1.0f;
    style.WindowRounding = 4.0f;
    style.Colors[ImGuiCol_WindowBg] = ImVec4(0.08f, 0.09f, 0.11f, 0.94f);
    style.Colors[ImGuiCol_PopupBg] = ImVec4(0.08f, 0.09f, 0.11f, 0.98f);

    ImGui_ImplOpenGL3_Init("#version 130");
    m_imguiInitialized = true;

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
    if (m_imguiInitialized) {
        ImGui_ImplOpenGL3_NewFrame();
        ImGuiIO& io = ImGui::GetIO();
        io.DisplaySize = ImVec2((float)m_windowWidth, (float)m_windowHeight);
        io.DeltaTime = 1.0f / 60.0f;
        ImGui::NewFrame();
    }
}

void Renderer::EndFrame() {
    if (m_imguiInitialized) {
        ImGui::Render();
        ImDrawData* drawData = ImGui::GetDrawData();
        if (drawData) {
            ImGui_ImplOpenGL3_RenderDrawData(drawData);
        }
    }
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
    m_keyCallback = callback;
}

bool Renderer::IsKeyPressed(int key) const {
    if (m_eventReceiver) {
        return m_eventReceiver->IsKeyDown(key);
    }
    return false;
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
