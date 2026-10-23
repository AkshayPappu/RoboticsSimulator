#include "rendering/EditorUI.h"
#include "core/Simulation.h"
#include "core/PhysicsWorld.h"
#include "core/SceneManager.h"
#include "physics/RigidBody.h"
#include "robotics/Robot.h"
#include "robotics/Motor.h"

#include <imgui.h>
#include <ImGuizmo.h>

#include <cstring>
#include <cstdio>
#include <chrono>
#include <algorithm>

namespace robosim {

EditorUI::EditorUI() = default;
EditorUI::~EditorUI() = default;

void EditorUI::Initialize(Simulation& sim) {
    m_sim = &sim;
    m_initialized = true;

    Log("[Editor] Initialized");
}

void EditorUI::Render() {
    if (!m_initialized || !m_sim) return;

    // Track FPS
    static auto lastTime = std::chrono::high_resolution_clock::now();
    auto now = std::chrono::high_resolution_clock::now();
    float dt = std::chrono::duration<float>(now - lastTime).count();
    lastTime = now;
    m_frameCount++;
    m_fpsTimer += dt;
    if (m_fpsTimer >= 1.0f) {
        m_fps = m_frameCount / m_fpsTimer;
        m_frameTimeMs = (m_fpsTimer / m_frameCount) * 1000.0f;
        m_frameCount = 0;
        m_fpsTimer = 0.0f;
    }

    ImGuizmo::BeginFrame();

    RenderMenuBar();

    if (m_state.showToolbar)       RenderToolbar();
    if (m_state.showHierarchy)     RenderHierarchyPanel();
    if (m_state.showInspector)     RenderInspectorPanel();
    if (m_state.showConsole)       RenderConsolePanel();
    if (m_state.showRobotBuilder)  RenderRobotBuilderPanel();

    RenderPerformanceOverlay();

    if (m_state.showDemoWindow) {
        ImGui::ShowDemoWindow(&m_state.showDemoWindow);
    }
}

void EditorUI::Shutdown() {
    m_initialized = false;
}

void EditorUI::Log(const std::string& message) {
    m_state.logMessages.push_back(message);
    if ((int)m_state.logMessages.size() > m_state.maxLogMessages) {
        m_state.logMessages.erase(m_state.logMessages.begin());
    }
}

// --- Menu Bar ---

void EditorUI::RenderMenuBar() {
    if (ImGui::BeginMainMenuBar()) {
        if (ImGui::BeginMenu("File")) {
            if (ImGui::MenuItem("New Scene")) {
                m_sim->GetSceneManager().ClearScene();
                Log("[Editor] New scene created");
            }
            if (ImGui::MenuItem("Load Scene...")) {
                // Simple text input for path
                ImGui::OpenPopup("LoadScenePopup");
            }
            if (ImGui::MenuItem("Save Scene...")) {
                ImGui::OpenPopup("SaveScenePopup");
            }
            ImGui::Separator();
            if (ImGui::MenuItem("Exit")) {
                m_sim->Stop();
            }
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("View")) {
            ImGui::MenuItem("Hierarchy", nullptr, &m_state.showHierarchy);
            ImGui::MenuItem("Inspector", nullptr, &m_state.showInspector);
            ImGui::MenuItem("Toolbar", nullptr, &m_state.showToolbar);
            ImGui::MenuItem("Console", nullptr, &m_state.showConsole);
            ImGui::MenuItem("Robot Builder", nullptr, &m_state.showRobotBuilder);
            ImGui::Separator();
            ImGui::MenuItem("ImGui Demo", nullptr, &m_state.showDemoWindow);
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Simulation")) {
            if (ImGui::MenuItem("Play", "Space", m_state.isPlaying)) {
                if (!m_state.isPlaying) {
                    m_sim->Resume();
                    m_state.isPlaying = true;
                    m_state.isPaused = false;
                }
            }
            if (ImGui::MenuItem("Pause", "P", m_state.isPaused)) {
                m_sim->Pause();
                m_state.isPlaying = false;
                m_state.isPaused = true;
            }
            if (ImGui::MenuItem("Step", "N")) {
                m_sim->StepOnce();
            }
            if (ImGui::MenuItem("Reset", "R")) {
                if (m_sim->GetRobot()) {
                    auto spawn = m_sim->GetSceneManager().GetSpawnPoint();
                    m_sim->GetRobot()->Reset(spawn.x, spawn.y, spawn.z);
                    Log("[Editor] Robot reset");
                }
            }
            ImGui::EndMenu();
        }

        ImGui::EndMainMenuBar();
    }

    // Load scene popup
    if (ImGui::BeginPopupModal("LoadScenePopup", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::Text("Scene file path:");
        ImGui::InputText("##scenepath", m_scenePathBuf, sizeof(m_scenePathBuf));
        if (ImGui::Button("Load")) {
            if (m_sceneChangeCb) {
                m_sceneChangeCb(m_scenePathBuf);
            }
            Log(std::string("[Editor] Loading scene: ") + m_scenePathBuf);
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel")) {
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }

    // Save scene popup
    if (ImGui::BeginPopupModal("SaveScenePopup", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::Text("Save path:");
        ImGui::InputText("##savepath", m_scenePathBuf, sizeof(m_scenePathBuf));
        if (ImGui::Button("Save")) {
            if (m_saveSceneCb) {
                m_saveSceneCb(m_scenePathBuf);
            }
            Log(std::string("[Editor] Saved scene to: ") + m_scenePathBuf);
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel")) {
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }
}

// --- Toolbar ---

void EditorUI::RenderToolbar() {
    ImGui::SetNextWindowPos(ImVec2(0, 20), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(0, 0), ImGuiCond_FirstUseEver);

    ImGui::Begin("Toolbar", &m_state.showToolbar, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_AlwaysAutoResize);

    RenderSimulationControls();

    ImGui::Separator();

    // Tool selection
    bool isSelect = (m_state.activeTool == EditorTool::Select);
    bool isTranslate = (m_state.activeTool == EditorTool::Translate);
    bool isRotate = (m_state.activeTool == EditorTool::Rotate);

    if (ImGui::RadioButton("Select", isSelect)) m_state.activeTool = EditorTool::Select;
    ImGui::SameLine();
    if (ImGui::RadioButton("Move", isTranslate)) m_state.activeTool = EditorTool::Translate;
    ImGui::SameLine();
    if (ImGui::RadioButton("Rotate", isRotate)) m_state.activeTool = EditorTool::Rotate;

    ImGui::Separator();

    // Add object
    const char* objTypes[] = { "Box", "Sphere", "Cylinder" };
    ImGui::Combo("Type", &m_newObjType, objTypes, 3);
    ImGui::DragFloat3("Size", m_newObjSize, 0.1f, 0.01f, 100.0f);
    ImGui::DragFloat3("Position", m_newObjPos, 0.1f, -100.0f, 100.0f);
    ImGui::DragFloat("Mass", &m_newObjMass, 0.1f, 0.0f, 1000.0f);

    if (ImGui::Button("Add Object")) {
        Material mat;
        BodyTransform t;
        t.px = m_newObjPos[0];
        t.py = m_newObjPos[1];
        t.pz = m_newObjPos[2];

        std::shared_ptr<RigidBody> body;
        switch (m_newObjType) {
            case 0:
                body = RigidBody::CreateBox(m_newObjSize[0], m_newObjSize[1], m_newObjSize[2],
                                            m_newObjMass, t, mat);
                break;
            case 1:
                body = RigidBody::CreateSphere(m_newObjSize[0], m_newObjMass, t, mat);
                break;
            case 2:
                body = RigidBody::CreateCylinder(m_newObjSize[0], m_newObjSize[1],
                                                  m_newObjMass, t, mat);
                break;
        }

        if (body) {
            body->SetName("new_object");
            m_sim->GetSceneManager().AddObject(body);
            Log("[Editor] Added new object");
        }
    }

    ImGui::End();
}

// --- Simulation Controls ---

void EditorUI::RenderSimulationControls() {
    if (ImGui::Button(m_state.isPlaying ? "Pause" : "Play")) {
        if (m_state.isPlaying) {
            m_sim->Pause();
            m_state.isPlaying = false;
            m_state.isPaused = true;
        } else {
            m_sim->Resume();
            m_state.isPlaying = true;
            m_state.isPaused = false;
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("Step")) {
        m_sim->StepOnce();
    }
    ImGui::SameLine();
    if (ImGui::Button("Reset")) {
        if (m_sim->GetRobot()) {
            auto spawn = m_sim->GetSceneManager().GetSpawnPoint();
            m_sim->GetRobot()->Reset(spawn.x, spawn.y, spawn.z);
        }
    }

    ImGui::SameLine();
    ImGui::SetNextItemWidth(100);
    ImGui::SliderFloat("Speed", &m_state.speedMultiplier, 0.25f, 4.0f, "%.2fx");
    m_sim->SetSpeedMultiplier(m_state.speedMultiplier);

    ImGui::Text("Time: %.3f s  |  Step: %d", m_sim->GetSimTime(), m_sim->GetStepCount());
}

// --- Scene Hierarchy ---

void EditorUI::RenderHierarchyPanel() {
    ImGui::SetNextWindowPos(ImVec2(0, 300), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(250, 400), ImGuiCond_FirstUseEver);

    ImGui::Begin("Scene Hierarchy", &m_state.showHierarchy);

    auto& objects = m_sim->GetSceneManager().GetObjects();

    for (int i = 0; i < (int)objects.size(); ++i) {
        bool isSelected = (m_state.selectedObjectIndex == i);
        std::string label = objects[i]->GetName();
        if (label.empty()) label = "object_" + std::to_string(i);

        if (objects[i]->IsFixed()) {
            label += " [static]";
        }

        if (ImGui::Selectable(label.c_str(), isSelected)) {
            m_state.selectedObjectIndex = i;
        }
    }

    // Robot section
    if (m_sim->GetRobot()) {
        ImGui::Separator();
        ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "Robot");

        double x, y, z;
        m_sim->GetRobot()->GetPosition(x, y, z);
        ImGui::Text("  Position: (%.2f, %.2f, %.2f)", x, y, z);
        ImGui::Text("  Heading: %.1f deg", m_sim->GetRobot()->GetHeading() * 180.0 / 3.14159);
        ImGui::Text("  Flipped: %s", m_sim->GetRobot()->IsFlipped() ? "YES" : "no");
    }

    if (ImGui::Button("Delete Selected") && m_state.selectedObjectIndex >= 0 &&
        m_state.selectedObjectIndex < (int)objects.size()) {
        auto obj = objects[m_state.selectedObjectIndex];
        m_sim->GetSceneManager().RemoveObject(obj);
        m_state.selectedObjectIndex = -1;
        Log("[Editor] Deleted object");
    }

    ImGui::End();
}

// --- Inspector ---

void EditorUI::RenderInspectorPanel() {
    ImGui::SetNextWindowPos(ImVec2(1030, 0), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(250, 500), ImGuiCond_FirstUseEver);

    ImGui::Begin("Inspector", &m_state.showInspector);

    auto& objects = m_sim->GetSceneManager().GetObjects();
    if (m_state.selectedObjectIndex >= 0 && m_state.selectedObjectIndex < (int)objects.size()) {
        auto& obj = objects[m_state.selectedObjectIndex];

        ImGui::Text("Name: %s", obj->GetName().c_str());

        const char* shapeNames[] = { "Box", "Sphere", "Cylinder", "Custom" };
        ImGui::Text("Shape: %s", shapeNames[(int)obj->GetShape()]);
        ImGui::Text("Mass: %.2f kg", obj->GetMass());
        ImGui::Text("Fixed: %s", obj->IsFixed() ? "Yes" : "No");

        ImGui::Separator();
        ImGui::Text("Transform");

        double px, py, pz;
        obj->GetPosition(px, py, pz);
        float pos[3] = {(float)px, (float)py, (float)pz};
        if (ImGui::DragFloat3("Position", pos, 0.05f)) {
            obj->SetPosition(pos[0], pos[1], pos[2]);
        }

        double qw, qx, qy, qz;
        obj->GetRotation(qw, qx, qy, qz);
        ImGui::Text("Rotation: (%.2f, %.2f, %.2f, %.2f)", qw, qx, qy, qz);

        ImGui::Separator();
        ImGui::Text("Velocity");
        double vx, vy, vz;
        obj->GetLinearVelocity(vx, vy, vz);
        ImGui::Text("Linear: (%.2f, %.2f, %.2f)", vx, vy, vz);
    } else {
        ImGui::TextWrapped("Select an object in the hierarchy to inspect it.");
    }

    ImGui::End();
}

// --- Console ---

void EditorUI::RenderConsolePanel() {
    ImGui::SetNextWindowPos(ImVec2(0, 500), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(800, 200), ImGuiCond_FirstUseEver);

    ImGui::Begin("Console", &m_state.showConsole);

    ImGui::BeginChild("ConsoleScroll", ImVec2(0, -ImGui::GetFrameHeightWithSpacing()), true);
    for (const auto& msg : m_state.logMessages) {
        ImGui::TextUnformatted(msg.c_str());
    }
    if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY()) {
        ImGui::SetScrollHereY(1.0f);
    }
    ImGui::EndChild();

    if (ImGui::Button("Clear")) {
        m_state.logMessages.clear();
    }

    ImGui::End();
}

// --- Robot Builder ---

void EditorUI::RenderRobotBuilderPanel() {
    ImGui::SetNextWindowPos(ImVec2(300, 100), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(400, 500), ImGuiCond_FirstUseEver);

    ImGui::Begin("Robot Builder", &m_state.showRobotBuilder);

    static float chassisSize[3] = {0.5f, 0.15f, 0.3f};
    static float chassisMass = 5.0f;
    static float wheelRadius = 0.1f;
    static float wheelWidth = 0.05f;
    static float wheelOffsetZ = 0.2f;
    static float maxMotorSpeed = 10.0f;
    static float maxTorque = 2.0f;

    ImGui::Text("Chassis");
    ImGui::DragFloat3("Chassis Size", chassisSize, 0.01f, 0.01f, 5.0f);
    ImGui::DragFloat("Chassis Mass (kg)", &chassisMass, 0.1f, 0.1f, 100.0f);

    ImGui::Separator();
    ImGui::Text("Wheels");
    ImGui::DragFloat("Wheel Radius", &wheelRadius, 0.01f, 0.01f, 1.0f);
    ImGui::DragFloat("Wheel Width", &wheelWidth, 0.01f, 0.01f, 0.5f);
    ImGui::DragFloat("Wheel Offset Z", &wheelOffsetZ, 0.01f, 0.01f, 2.0f);

    ImGui::Separator();
    ImGui::Text("Motors");
    ImGui::DragFloat("Max Speed (rad/s)", &maxMotorSpeed, 0.5f, 0.0f, 50.0f);
    ImGui::DragFloat("Max Torque (Nm)", &maxTorque, 0.1f, 0.0f, 100.0f);

    ImGui::Separator();

    if (ImGui::Button("Build Robot")) {
        RobotConfig config;
        config.chassisSizeX = chassisSize[0];
        config.chassisSizeY = chassisSize[1];
        config.chassisSizeZ = chassisSize[2];
        config.chassisMass = chassisMass;
        config.wheelRadius = wheelRadius;
        config.wheelWidth = wheelWidth;
        config.wheelOffsetZ = wheelOffsetZ;
        config.maxMotorSpeed = maxMotorSpeed;
        config.maxTorque = maxTorque;

        auto spawn = m_sim->GetSceneManager().GetSpawnPoint();
        config.spawnX = spawn.x;
        config.spawnY = spawn.y;
        config.spawnZ = spawn.z;

        auto robot = std::make_shared<Robot>();
        robot->Build(m_sim->GetWorld(), config);
        m_sim->SetRobot(robot);

        Log("[Editor] Built new robot");
    }

    ImGui::SameLine();
    if (ImGui::Button("Load from JSON...")) {
        ImGui::OpenPopup("LoadRobotPopup");
    }

    if (ImGui::BeginPopupModal("LoadRobotPopup", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::Text("Robot JSON path:");
        ImGui::InputText("##robotpath", m_robotPathBuf, sizeof(m_robotPathBuf));
        if (ImGui::Button("Load")) {
            auto robot = std::make_shared<Robot>();
            robot->BuildFromJson(m_sim->GetWorld(), m_robotPathBuf);
            m_sim->SetRobot(robot);
            Log(std::string("[Editor] Loaded robot from: ") + m_robotPathBuf);
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel")) {
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }

    // Show current robot info
    if (m_sim->GetRobot()) {
        ImGui::Separator();
        ImGui::Text("Current Robot");
        auto& config = m_sim->GetRobot()->GetConfig();
        ImGui::Text("Chassis: %.2f x %.2f x %.2f, %.1f kg",
                     config.chassisSizeX, config.chassisSizeY, config.chassisSizeZ, config.chassisMass);
        ImGui::Text("Wheels: r=%.3f, w=%.3f", config.wheelRadius, config.wheelWidth);
        ImGui::Text("Motors: %zu", m_sim->GetRobot()->GetMotors().size());

        for (size_t i = 0; i < m_sim->GetRobot()->GetMotors().size(); ++i) {
            auto& motor = m_sim->GetRobot()->GetMotors()[i];
            ImGui::Text("  Motor %zu: speed=%.2f rad/s, torque=%.2f Nm",
                         i, motor->GetCurrentSpeed(), motor->GetAppliedTorque());
        }
    }

    ImGui::End();
}

// --- Performance Overlay ---

void EditorUI::RenderPerformanceOverlay() {
    ImGui::SetNextWindowPos(ImVec2(10, 30), ImGuiCond_Always);
    ImGui::SetNextWindowBgAlpha(0.5f);
    ImGui::Begin("##perfoverlay", nullptr,
                 ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize |
                 ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav |
                 ImGuiWindowFlags_NoMove);

    ImGui::Text("FPS: %.1f (%.2f ms)", m_fps, m_frameTimeMs);
    ImGui::Text("Sim: %.3f s | Step: %d", m_sim->GetSimTime(), m_sim->GetStepCount());
    ImGui::Text("Bodies: %zu", m_sim->GetWorld().GetBodies().size());

    ImGui::End();
}

}  // namespace robosim
