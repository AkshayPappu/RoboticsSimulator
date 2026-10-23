#pragma once

#include <memory>
#include <string>
#include <functional>
#include <array>

namespace chrono {
class ChSystem;
namespace irrlicht {
class ChVisualSystemIrrlicht;
}
}

namespace irr {
class IEventReceiver;
}

namespace robosim {

class PhysicsWorld;

struct RendererConfig {
    int windowWidth = 1280;
    int windowHeight = 720;
    std::string windowTitle = "Robotics Simulator";
    bool showGrid = true;
    bool showAxes = true;
    double cameraX = -8.0, cameraY = 5.0, cameraZ = -8.0;
    double lookAtX = 0.0, lookAtY = 0.0, lookAtZ = 0.0;
};

class SimEventReceiver;

class Renderer {
public:
    Renderer();
    ~Renderer();

    Renderer(const Renderer&) = delete;
    Renderer& operator=(const Renderer&) = delete;

    void Initialize(PhysicsWorld& world, const RendererConfig& config = {});

    bool IsRunning() const;
    void BeginFrame();
    void EndFrame();
    void Render();

    void SetWindowTitle(const std::string& title);
    void BindAll();
    void SetShowGrid(bool show);
    bool GetShowGrid() const { return m_showGrid; }

    using KeyCallback = std::function<void(int key, bool pressed)>;
    void SetKeyCallback(KeyCallback callback);

    using MouseCallback = std::function<void(int x, int y, bool leftDown, bool rightDown, float wheel)>;
    void SetMouseCallback(MouseCallback callback) { m_mouseCallback = callback; }

    bool IsKeyPressed(int key) const;

    std::shared_ptr<chrono::irrlicht::ChVisualSystemIrrlicht> GetVisualSystem() {
        return m_vis;
    }

    int GetWindowWidth() const;
    int GetWindowHeight() const;

private:
    std::shared_ptr<chrono::irrlicht::ChVisualSystemIrrlicht> m_vis;
    std::unique_ptr<SimEventReceiver> m_eventReceiver;
    KeyCallback m_keyCallback;
    MouseCallback m_mouseCallback;
    bool m_initialized = false;
    bool m_imguiInitialized = false;
    int m_windowWidth = 1280;
    int m_windowHeight = 720;
    bool m_showGrid = true;

    friend class SimEventReceiver;
};

}  // namespace robosim
