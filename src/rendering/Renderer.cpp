#include "rendering/Renderer.h"
#include "core/PhysicsWorld.h"
#include "physics/ChronoEngine.h"

#include <chrono_irrlicht/ChVisualSystemIrrlicht.h>
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

            // Don't forward keyboard to app if ImGui wants it
            if (io.WantCaptureKeyboard) {
                return true;
            }

            if (m_renderer.m_keyCallback) {
                m_renderer.m_keyCallback(key, pressed);
            }
            return false;
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
    if (m_imguiInitialized) {
        ImGui_ImplOpenGL3_Shutdown();
        ImGui::DestroyContext();
    }
}

void Renderer::Initialize(PhysicsWorld& world, const RendererConfig& config) {
    m_vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();

    m_vis->AttachSystem(&world.GetChSystem());

    m_windowWidth = config.windowWidth;
    m_windowHeight = config.windowHeight;
    m_vis->SetWindowSize(config.windowWidth, config.windowHeight);
    m_vis->SetWindowTitle(config.windowTitle);
    m_vis->SetCameraVertical(CameraVerticalDir::Y);

    m_vis->Initialize();

    m_vis->AddCamera(
        ChVector3d(config.cameraX, config.cameraY, config.cameraZ),
        ChVector3d(config.lookAtX, config.lookAtY, config.lookAtZ)
    );
    m_vis->AddTypicalLights();
    m_vis->AddGrid(0.5, 0.5, 40, 40,
                   ChCoordsys<>(ChVector3d(0, 0, 0), QuatFromAngleX(CH_PI_2)),
                   ChColor(0.4f, 0.4f, 0.4f));

    // Register event receiver for keyboard/mouse input
    m_eventReceiver = std::make_unique<SimEventReceiver>(*this);
    m_vis->AddUserEventReceiver(m_eventReceiver.get());

    // Initialize Dear ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.DisplaySize = ImVec2((float)config.windowWidth, (float)config.windowHeight);
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    ImGui::StyleColorsDark();

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
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
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

int Renderer::GetWindowWidth() const {
    return m_windowWidth;
}

int Renderer::GetWindowHeight() const {
    return m_windowHeight;
}

}  // namespace robosim
