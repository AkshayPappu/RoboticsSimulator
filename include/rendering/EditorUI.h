#pragma once

#include "robotics/AssemblyGraph.h"
#include "robotics/PartCatalog.h"

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
    bool showHierarchy = false;
    bool showInspector = false;
    bool showToolbar = true;
    bool showConsole = false;
    bool showRobotBuilder = true;
    bool showDemoWindow = false;
    bool showGrid = false;
    bool showHelp = true;

    EditorTool activeTool = EditorTool::Select;
    int selectedObjectIndex = -1;

    // Simulation control
    bool isPlaying = true;
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
    void ApplyPendingAssembly();
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
    void RenderHelpOverlay();
    void SpawnRobotPreset(const char* fileName);
    void LoadRobotFromPath(const std::string& path);
    void ApplyDriveMode();
    void SetSimulationPlaying(bool playing);
    void ResetRobot();

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
    char m_robotPathBuf[256] = "robots/assembled.json";

    // New object parameters
    float m_newObjSize[3] = {1.0f, 1.0f, 1.0f};
    float m_newObjPos[3] = {0.0f, 1.0f, 0.0f};
    float m_newObjMass = 1.0f;
    int m_newObjType = 0;  // 0=box, 1=sphere, 2=cylinder

    int m_controllerIndex = 0;
    char m_saveRobotPathBuf[256] = "robots/custom_robot.json";

    std::vector<std::shared_ptr<RigidBody>> m_undoDeleted;

    AssemblyGraph m_graph;
    std::string m_selectedPartId;
    std::string m_selectedInstanceId;
    std::string m_poseEditId;
    int m_pendingClickX = -1;
    int m_pendingClickY = -1;
    bool m_mouseLeftDown = false;
    int m_instanceSerial = 1;
    int m_jointTypeIndex = 1;
    int m_selectedShape = -1;
    bool m_attachMotor = true;
    bool m_showCollisionPreview = false;
    bool m_assemblyDirty = false;
    float m_poseEdit[3] = {0, 0, 0};
    float m_eulerEdit[3] = {0, 0, 0};
    float m_axisEdit[3] = {0, 0, 1};

    void RenderAssemblerPanel();
    void RebuildFromAssembly();
    void SnapSelectedPart(const std::string& parentId, const std::string& mountId);
    void AddPrimitiveBody(AssemblyShape shape,
                          const std::string& parentId = {},
                          const std::string& mountId = {});
    void EraseBodyTree(const std::string& id);
    void DeleteBodyTree(const std::string& id);
    void DuplicateSelectedBody();
    void LoadGraphFile(const std::string& path);
    bool TryPickMount(int mx, int my);
    void HandleAssemblerMouse(int x, int y, bool leftDown, bool rightDown, float wheel);
    AssemblyJointType SelectedJointType() const;
    void RememberPoseEditor();
};

}  // namespace robosim
