#pragma once

#include <memory>
#include <string>
#include <functional>
#include <unordered_map>

namespace chrono {
class ChSystem;
class ChBody;
namespace vsg3d {
class ChVisualSystemVSG;
}
}

namespace robosim {

class PhysicsWorld;
class EditorUI;
class SimEventHandlerVSG;
class SimGuiComponentVSG;

struct RendererConfig {
    int windowWidth = 1440;
    int windowHeight = 810;
    std::string windowTitle = "Robotics Simulator";
    bool showGrid = false;  // textured floor is the reference; grid reads as debug
    bool showAxes = false;
    // Framed for DiffDrive TB3 at origin (product first-open view).
    double cameraX = -3.4, cameraY = 2.35, cameraZ = -3.1;
    double lookAtX = 0.15, lookAtY = 0.25, lookAtZ = 0.0;
};

class Renderer {
public:
    Renderer();
    ~Renderer();

    Renderer(const Renderer&) = delete;
    Renderer& operator=(const Renderer&) = delete;

    /// Create the VSG visual system. Pass EditorUI so its ImGui panels are
    /// registered as a ChGuiComponentVSG before Initialize().
    void Initialize(PhysicsWorld& world,
                    const RendererConfig& config = {},
                    EditorUI* editor = nullptr);

    bool IsRunning() const;
    void BeginFrame();
    void EndFrame();
    void Render();

    void SetWindowTitle(const std::string& title);
    void BindAll();
    void UnbindItem(const std::shared_ptr<chrono::ChBody>& body);
    void ShutdownImGui();
    void SetShowGrid(bool show);
    bool GetShowGrid() const { return m_showGrid; }

    /// Product view: solid body meshes on, collision wireframes off, Chrono debug HUD/logo hidden.
    void ApplyProductVisualMode();

    /// Soft-follow a world point and clamp pitch/height so the view cannot go under the floor.
    void UpdateFollowCamera(double targetX, double targetY, double targetZ, double dt,
                            bool enableFollow = true);

    void SetCameraFollowEnabled(bool enabled) { m_followEnabled = enabled; }
    bool GetCameraFollowEnabled() const { return m_followEnabled; }
    bool IsCameraManipulating() const { return m_mouseLeft || m_mouseRight; }

    using KeyCallback = std::function<void(int key, bool pressed)>;
    void SetKeyCallback(KeyCallback callback);

    using MouseCallback = std::function<void(int x, int y, bool leftDown, bool rightDown, float wheel)>;
    void SetMouseCallback(MouseCallback callback) { m_mouseCallback = callback; }

    bool IsKeyPressed(int key) const;

    bool GetCameraLook(double& ex, double& ey, double& ez,
                       double& tx, double& ty, double& tz) const;

    std::shared_ptr<chrono::vsg3d::ChVisualSystemVSG> GetVisualSystem() {
        return m_vis;
    }

    int GetWindowWidth() const;
    int GetWindowHeight() const;

private:
    std::shared_ptr<chrono::vsg3d::ChVisualSystemVSG> m_vis;
    std::shared_ptr<SimEventHandlerVSG> m_eventHandler;
    std::shared_ptr<SimGuiComponentVSG> m_guiComponent;
    KeyCallback m_keyCallback;
    MouseCallback m_mouseCallback;
    std::unordered_map<int, bool> m_keyState;
    void EnsureMeshesCompiled();

    bool m_initialized = false;
    int m_windowWidth = 1440;
    int m_windowHeight = 810;
    bool m_showGrid = false;
    bool m_mouseLeft = false;
    bool m_mouseRight = false;
    bool m_followEnabled = true;
    double m_followStrength = 3.2;
    double m_minEyeHeight = 0.85;
    double m_minPitch = 0.28;   // keep a clear top-down bias so the floor never reads edge-on
    double m_maxPitch = 1.20;
    double m_minDistance = 1.4;
    double m_maxDistance = 32.0;

    friend class SimEventHandlerVSG;
    friend class SimGuiComponentVSG;
};

}  // namespace robosim
