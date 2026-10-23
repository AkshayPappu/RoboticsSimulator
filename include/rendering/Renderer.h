#pragma once

#include <memory>
#include <string>
#include <functional>

namespace chrono {
class ChSystem;
namespace irrlicht {
class ChVisualSystemIrrlicht;
}
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

    using KeyCallback = std::function<void(int key, bool pressed)>;
    void SetKeyCallback(KeyCallback callback) { m_keyCallback = callback; }

    bool IsKeyPressed(int key) const;

    std::shared_ptr<chrono::irrlicht::ChVisualSystemIrrlicht> GetVisualSystem() {
        return m_vis;
    }

private:
    std::shared_ptr<chrono::irrlicht::ChVisualSystemIrrlicht> m_vis;
    KeyCallback m_keyCallback;
    bool m_initialized = false;
};

}  // namespace robosim
