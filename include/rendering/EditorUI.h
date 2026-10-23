#pragma once

#include <memory>
#include <string>
#include <vector>
#include <functional>

namespace robosim {

class Simulation;
class PhysicsWorld;
class SceneManager;
class RigidBody;
class Robot;

enum class EditorTool {
    Select,
    Translate,
    Rotate,
    Scale
};

struct EditorState {
    bool showHierarchy = true;
    bool showInspector = true;
    bool showToolbar = true;
    bool showConsole = true;
    bool showRobotBuilder = false;
    bool showDemoWindow = false;

    EditorTool activeTool = EditorTool::Select;
    int selectedObjectIndex = -1;

    // Simulation control
    bool isPlaying = false;
    bool isPaused = false;
    float speedMultiplier = 1.0f;

    // Console log
    std::vector<std::string> logMessages;
    int maxLogMessages = 200;
};

class EditorUI {
public:
    EditorUI();
    ~EditorUI();

    EditorUI(const EditorUI&) = delete;
    EditorUI& operator=(const EditorUI&) = delete;

    void Initialize(Simulation& sim);
    void Render();
    void Shutdown();

    EditorState& GetState() { return m_state; }
    void Log(const std::string& message);

    using SceneChangeCallback = std::function<void(const std::string&)>;
    void SetSceneChangeCallback(SceneChangeCallback cb) { m_sceneChangeCb = cb; }

    using SaveSceneCallback = std::function<void(const std::string&)>;
    void SetSaveSceneCallback(SaveSceneCallback cb) { m_saveSceneCb = cb; }

private:
    void RenderMenuBar();
    void RenderToolbar();
    void RenderHierarchyPanel();
    void RenderInspectorPanel();
    void RenderConsolePanel();
    void RenderRobotBuilderPanel();
    void RenderSimulationControls();
    void RenderPerformanceOverlay();

    Simulation* m_sim = nullptr;
    EditorState m_state;
    bool m_initialized = false;

    SceneChangeCallback m_sceneChangeCb;
    SaveSceneCallback m_saveSceneCb;

    // FPS tracking
    float m_fps = 0.0f;
    float m_frameTimeMs = 0.0f;
    int m_frameCount = 0;
    float m_fpsTimer = 0.0f;

    // File dialog state
    char m_scenePathBuf[256] = {};
    char m_robotPathBuf[256] = {};

    // New object parameters
    float m_newObjSize[3] = {1.0f, 1.0f, 1.0f};
    float m_newObjPos[3] = {0.0f, 1.0f, 0.0f};
    float m_newObjMass = 1.0f;
    int m_newObjType = 0;  // 0=box, 1=sphere, 2=cylinder
};

}  // namespace robosim
