#include "rendering/EditorUI.h"
#include "rendering/Renderer.h"
#include "core/Simulation.h"
#include "core/PhysicsWorld.h"
#include "core/SceneManager.h"
#include "physics/RigidBody.h"
#include "robotics/Robot.h"
#include "robotics/Motor.h"
#include "robotics/Sensor.h"
#include "robotics/Controller.h"
#include "robotics/PartCatalog.h"

#include <fstream>
#include <functional>
#include <nlohmann/json.hpp>

#include <imgui.h>
#include <ImGuizmo.h>
#include <chrono/physics/ChSystem.h>

#include <cstring>
#include <cctype>
#include <cstdio>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <functional>
#include <exception>
#include <new>
#include <unordered_set>

#include <chrono/core/ChFrame.h>

namespace robosim {

EditorUI::EditorUI() = default;
EditorUI::~EditorUI() = default;

void EditorUI::Initialize(Simulation& sim) {
    m_sim = &sim;
    m_initialized = true;

    if (!PartCatalog::Instance().IsLoaded()) {
        PartCatalog::Instance().Load();
    }
    m_sim->GetRenderer().SetMouseCallback(
        [this](int x, int y, bool left, bool right, float wheel) {
            HandleAssemblerMouse(x, y, left, right, wheel);
        });

    Log("[Editor] Click the 3D view, then drive with W A S D or the arrow keys.");
    Log("[Editor] Robot builder: choose a part, then add it or click a socket.");
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
    if (m_state.showHelp)          RenderHelpOverlay();

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

namespace {

struct ExampleRobot {
    const char* label;
    const char* file;
};

const ExampleRobot kExamples[] = {
    {"Differential drive", "differential_drive.json"},
    {"Four-wheel", "four_wheel.json"},
    {"Arm", "robot_arm.json"},
    {"Assembled four-wheel", "generic_four_wheel.json"},
    {"Assembled arm", "generic_two_link_arm.json"},
    {"Assembled linkage", "generic_three_body.json"},
};

const char* kJointWords = "Welded\0Spins\0Slides\0Ball joint\0";

std::string MountLabel(const std::string& id) {
    if (id == "wheel_left") return "Left wheel";
    if (id == "wheel_right") return "Right wheel";
    if (id == "wheel_fl") return "Front left wheel";
    if (id == "wheel_fr") return "Front right wheel";
    if (id == "wheel_rl") return "Rear left wheel";
    if (id == "wheel_rr") return "Rear right wheel";
    if (id == "caster_rear") return "Rear caster";
    if (id == "deck_front") return "Front deck";
    if (id == "deck_center") return "Center deck";
    if (id == "hub_left") return "Left hub";
    if (id == "hub_right") return "Right hub";
    if (id == "face") return "Top face";
    if (id == "end") return "End";

    std::string out;
    bool cap = true;
    for (unsigned char c : id) {
        if (c == '_') {
            if (!out.empty() && out.back() != ' ') out.push_back(' ');
            cap = true;
            continue;
        }
        out.push_back(cap ? static_cast<char>(std::toupper(c)) : static_cast<char>(c));
        cap = false;
    }
    return out;
}

const char* ShapeLabel(AssemblyShape shape) {
    switch (shape) {
        case AssemblyShape::Sphere: return "Sphere";
        case AssemblyShape::Cylinder: return "Cylinder";
        case AssemblyShape::Capsule: return "Capsule";
        case AssemblyShape::Mesh: return "Mesh";
        case AssemblyShape::Box: return "Box";
    }
    return "Box";
}

bool CategoryIn(const std::string& category, std::initializer_list<const char*> names) {
    for (const char* name : names) {
        if (category == name) return true;
    }
    return false;
}

void SectionTitle(const char* title) {
    ImGui::Spacing();
    ImGui::Separator();
    ImGui::TextUnformatted(title);
}

std::vector<std::string> BodiesOnMount(const AssemblyGraph& graph, const std::string& parentId,
                                       double x, double y, double z) {
    std::vector<std::string> ids;
    for (const auto& joint : graph.joints) {
        if (joint.bodyA != parentId) continue;
        const double dx = joint.anchorX - x;
        const double dy = joint.anchorY - y;
        const double dz = joint.anchorZ - z;
        if (dx * dx + dy * dy + dz * dz < 1e-8) ids.push_back(joint.bodyB);
    }
    return ids;
}

}  // namespace

// --- Menu Bar ---

void EditorUI::RenderMenuBar() {
    if (ImGui::BeginMainMenuBar()) {
        if (ImGui::BeginMenu("File")) {
            if (ImGui::MenuItem("New scene")) {
                m_sim->GetSceneManager().ClearScene();
                Log("[Editor] New scene created");
            }
            if (ImGui::MenuItem("Open scene")) {
                ImGui::OpenPopup("LoadScenePopup");
            }
            if (ImGui::MenuItem("Save scene")) {
                ImGui::OpenPopup("SaveScenePopup");
            }
            ImGui::Separator();
            if (ImGui::MenuItem("Undo delete", "Ctrl+Z")) {
                if (!m_undoDeleted.empty()) {
                    auto body = m_undoDeleted.back();
                    m_undoDeleted.pop_back();
                    m_sim->GetSceneManager().AddObject(body);
                    m_sim->GetRenderer().BindAll();
                    Log("[Editor] Undo delete");
                }
            }
            ImGui::Separator();
            if (ImGui::MenuItem("Exit")) {
                m_sim->Stop();
            }
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("View")) {
            ImGui::MenuItem("Scene", nullptr, &m_state.showHierarchy);
            ImGui::MenuItem("Selected object", nullptr, &m_state.showInspector);
            ImGui::MenuItem("Controls", nullptr, &m_state.showToolbar);
            ImGui::MenuItem("Log", nullptr, &m_state.showConsole);
            ImGui::MenuItem("Robot builder", nullptr, &m_state.showRobotBuilder);
            ImGui::MenuItem("Hints", nullptr, &m_state.showHelp);
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Robot")) {
            for (const auto& example : kExamples) {
                if (ImGui::MenuItem(example.label)) {
                    SpawnRobotPreset(example.file);
                }
            }
            ImGui::Separator();
            ImGui::MenuItem("Open builder", nullptr, &m_state.showRobotBuilder);
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Simulation")) {
            if (ImGui::MenuItem("Play", "Space", m_state.isPlaying)) {
                if (!m_state.isPlaying) SetSimulationPlaying(true);
            }
            if (ImGui::MenuItem("Pause", "P", m_state.isPaused)) {
                SetSimulationPlaying(false);
            }
            if (ImGui::MenuItem("Step", "N")) {
                m_sim->StepOnce();
            }
            if (ImGui::MenuItem("Reset", "R")) {
                ResetRobot();
            }
            ImGui::EndMenu();
        }

        ImGui::EndMainMenuBar();
    }

    // Load scene popup
    if (ImGui::BeginPopupModal("LoadScenePopup", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::Text("Scene file");
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
        ImGui::Text("Scene file");
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
    ImGui::SetNextWindowPos(ImVec2(8, 26), ImGuiCond_Always);
    ImGui::SetNextWindowBgAlpha(0.92f);
    ImGui::Begin("##hud", &m_state.showToolbar,
                 ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_AlwaysAutoResize |
                 ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove);

    RenderSimulationControls();
    ImGui::End();
}

// --- Simulation Controls ---

void EditorUI::RenderSimulationControls() {
    if (ImGui::Button(m_state.isPlaying ? "Pause" : "Play")) {
        SetSimulationPlaying(!m_state.isPlaying);
    }
    ImGui::SameLine();
    if (ImGui::Button("Step")) {
        m_sim->StepOnce();
    }
    ImGui::SameLine();
    if (ImGui::Button("Reset")) {
        ResetRobot();
    }

    ImGui::SameLine();
    ImGui::SetNextItemWidth(100);
    ImGui::SliderFloat("Speed", &m_state.speedMultiplier, 0.25f, 4.0f, "%.2fx");
    m_sim->SetSpeedMultiplier(m_state.speedMultiplier);

    ImGui::SameLine();
    ImGui::SetNextItemWidth(120);
    if (ImGui::Combo("Drive", &m_controllerIndex, "Keyboard\0Script\0Off\0")) {
        ApplyDriveMode();
    }

    ImGui::Text("Time: %.3f s", m_sim->GetSimTime());
    if (auto* robot = m_sim->GetRobot()) {
        if (robot->IsFlipped()) {
            ImGui::SameLine();
            ImGui::TextUnformatted("Robot flipped. Press Reset.");
        }
    }
}

// --- Scene Hierarchy ---

void EditorUI::RenderHierarchyPanel() {
    const ImGuiIO& io = ImGui::GetIO();
    const float top = 96.0f;
    const float panelH = std::max(180.0f, (io.DisplaySize.y - top - 16.0f) * 0.5f);
    ImGui::SetNextWindowPos(ImVec2(8, top), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(260, panelH), ImGuiCond_FirstUseEver);

    ImGui::Begin("Scene", &m_state.showHierarchy);

    auto& objects = m_sim->GetSceneManager().GetObjects();

    for (int i = 0; i < (int)objects.size(); ++i) {
        bool isSelected = (m_state.selectedObjectIndex == i);
        std::string label = objects[i]->GetName();
        if (label.empty()) label = "Object " + std::to_string(i + 1);

        if (objects[i]->IsFixed()) {
            label += "  Fixed";
        }

        if (ImGui::Selectable(label.c_str(), isSelected)) {
            m_state.selectedObjectIndex = i;
        }
    }

    if (ImGui::Button("Delete") && m_state.selectedObjectIndex >= 0 &&
        m_state.selectedObjectIndex < (int)objects.size()) {
        auto obj = objects[m_state.selectedObjectIndex];
        m_undoDeleted.push_back(obj);
        m_sim->GetSceneManager().RemoveObject(obj);
        m_state.selectedObjectIndex = -1;
        Log("[Editor] Deleted object. Undo delete is in the File menu.");
    }

    ImGui::End();
}

// --- Inspector ---

void EditorUI::RenderInspectorPanel() {
    const ImGuiIO& io = ImGui::GetIO();
    const float top = 96.0f;
    const float panelH = std::max(180.0f, (io.DisplaySize.y - top - 16.0f) * 0.5f);
    ImGui::SetNextWindowPos(ImVec2(8, top + panelH + 8.0f), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(260, panelH), ImGuiCond_FirstUseEver);

    ImGui::Begin("Selected object", &m_state.showInspector);

    auto& objects = m_sim->GetSceneManager().GetObjects();
    if (m_state.selectedObjectIndex >= 0 && m_state.selectedObjectIndex < (int)objects.size()) {
        auto& obj = objects[m_state.selectedObjectIndex];

        ImGui::Text("Name: %s", obj->GetName().c_str());

        const char* shapeNames[] = { "Box", "Sphere", "Cylinder", "Custom" };
        ImGui::Text("Shape: %s", shapeNames[(int)obj->GetShape()]);
        ImGui::Text("Mass: %.2f kg", obj->GetMass());
        ImGui::Text("Fixed: %s", obj->IsFixed() ? "Yes" : "No");

        ImGui::Separator();
        double px, py, pz;
        obj->GetPosition(px, py, pz);
        float pos[3] = {(float)px, (float)py, (float)pz};
        if (ImGui::DragFloat3("Position", pos, 0.05f)) {
            obj->SetPosition(pos[0], pos[1], pos[2]);
        }

        if (ImGui::CollapsingHeader("More")) {
            double qw, qx, qy, qz;
            obj->GetRotation(qw, qx, qy, qz);
            float euler[3];
            {
                double sinr_cosp = 2.0 * (qw * qx + qy * qz);
                double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
                euler[0] = (float)(std::atan2(sinr_cosp, cosr_cosp) * 180.0 / 3.141592653589793);
                double sinp = 2.0 * (qw * qy - qz * qx);
                euler[1] = (float)(std::asin(std::clamp(sinp, -1.0, 1.0)) * 180.0 / 3.141592653589793);
                double siny_cosp = 2.0 * (qw * qz + qx * qy);
                double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
                euler[2] = (float)(std::atan2(siny_cosp, cosy_cosp) * 180.0 / 3.141592653589793);
            }
            if (ImGui::DragFloat3("Rotation", euler, 1.0f)) {
                double rx = euler[0] * 3.141592653589793 / 180.0;
                double ry = euler[1] * 3.141592653589793 / 180.0;
                double rz = euler[2] * 3.141592653589793 / 180.0;
                double cy = std::cos(rz * 0.5), sy = std::sin(rz * 0.5);
                double cp = std::cos(ry * 0.5), sp = std::sin(ry * 0.5);
                double cr = std::cos(rx * 0.5), sr = std::sin(rx * 0.5);
                obj->SetRotation(
                    cr * cp * cy + sr * sp * sy,
                    sr * cp * cy - cr * sp * sy,
                    cr * sp * cy + sr * cp * sy,
                    cr * cp * sy - sr * sp * cy
                );
            }

            double vx, vy, vz;
            obj->GetLinearVelocity(vx, vy, vz);
            ImGui::Text("Velocity: (%.2f, %.2f, %.2f)", vx, vy, vz);
        }
    } else if (m_sim->GetRobot()) {
        auto* robot = m_sim->GetRobot();
        double x, y, z;
        robot->GetPosition(x, y, z);
        ImGui::Text("Position: (%.2f, %.2f, %.2f)", x, y, z);
        ImGui::Text("Heading: %.0f deg", robot->GetHeading() * 180.0 / 3.141592653589793);
        ImGui::Text("Wheels: left %.1f, right %.1f rad/s",
                    robot->GetLeftWheelSpeed(), robot->GetRightWheelSpeed());

        if (!robot->GetSensors().empty()) {
            ImGui::Separator();
            for (const auto& sensor : robot->GetSensors()) {
                if (sensor->GetType() == SensorType::Distance) {
                    auto* d = static_cast<DistanceSensor*>(sensor.get());
                    ImGui::Text("Distance: %.3f m", d->GetDistance());
                    ImGui::Text("Hit: %s", d->HasHit() ? "yes" : "no");
                } else if (sensor->GetType() == SensorType::IMU) {
                    auto* imu = static_cast<IMUSensor*>(sensor.get());
                    auto a = imu->GetLinearAccel();
                    auto w = imu->GetAngularVel();
                    ImGui::Text("Motion");
                    ImGui::Text("Accel: (%.2f, %.2f, %.2f)", a.x(), a.y(), a.z());
                    ImGui::Text("Gyro: (%.2f, %.2f, %.2f)", w.x(), w.y(), w.z());
                } else if (sensor->GetType() == SensorType::Contact) {
                    auto* c = static_cast<ContactSensor*>(sensor.get());
                    ImGui::Text("Touch: %s", c->InContact() ? "yes" : "no");
                }
                ImGui::Separator();
            }
        }
    } else {
        ImGui::TextWrapped("Select an object in Scene to inspect it.");
    }

    ImGui::End();
}

// --- Console ---

void EditorUI::RenderConsolePanel() {
    const ImGuiIO& io = ImGui::GetIO();
    const float width = std::max(320.0f, io.DisplaySize.x - 480.0f - 32.0f);
    ImGui::SetNextWindowPos(ImVec2(280, std::max(120.0f, io.DisplaySize.y - 188.0f)), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(width, 170), ImGuiCond_FirstUseEver);

    ImGui::Begin("Log", &m_state.showConsole);

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
    const ImGuiIO& io = ImGui::GetIO();
    const float width = 480.0f;
    const float height = std::min(680.0f, std::max(360.0f, io.DisplaySize.y - 112.0f));
    const float x = std::max(280.0f, io.DisplaySize.x - width - 8.0f);
    ImGui::SetNextWindowPos(ImVec2(x, 96), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(width, height), ImGuiCond_FirstUseEver);

    ImGui::Begin("Robot Builder", &m_state.showRobotBuilder);

    if (ImGui::BeginTabBar("BuilderTabs")) {
        if (ImGui::BeginTabItem("Assemble")) {
            RenderAssemblerPanel();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Examples")) {
            ImGui::TextWrapped("Load a finished robot. The same list is in the Robot menu.");
            ImGui::Spacing();
            for (const auto& example : kExamples) {
                if (ImGui::Button(example.label, ImVec2(-1, 0))) {
                    SpawnRobotPreset(example.file);
                }
            }

            ImGui::Spacing();
            ImGui::Separator();
            ImGui::TextDisabled("Robot file");
            ImGui::SetNextItemWidth(-1);
            ImGui::InputText("##robotfile", m_saveRobotPathBuf, sizeof(m_saveRobotPathBuf));
            if (ImGui::Button("Save")) {
                if (m_sim->GetRobot()) {
                    std::ofstream out(m_saveRobotPathBuf);
                    if (out) {
                        out << m_sim->GetRobot()->ToJson().dump(2);
                        Log(std::string("[Editor] Saved robot to ") + m_saveRobotPathBuf);
                    }
                } else {
                    Log("[Editor] No robot to save");
                }
            }
            ImGui::SameLine();
            if (ImGui::Button("Load")) {
                LoadRobotFromPath(m_saveRobotPathBuf);
            }

            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }

    ImGui::End();
}

// --- Performance Overlay ---

void EditorUI::RenderHelpOverlay() {
    ImGuiIO& io = ImGui::GetIO();
    float y = io.DisplaySize.y > 16.0f ? io.DisplaySize.y - 8.0f : 700.0f;
    ImGui::SetNextWindowPos(ImVec2(8, y), ImGuiCond_Always, ImVec2(0, 1));
    ImGui::SetNextWindowBgAlpha(0.85f);
    ImGui::Begin("##help", nullptr,
                 ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize |
                 ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav |
                 ImGuiWindowFlags_NoMove);

    ImGui::Text("Click the 3D view. W/S drive, A/D turn.");
    ImGui::TextUnformatted("Drag to orbit, wheel to zoom, right-drag to pan.");
    ImGui::SameLine();
    ImGui::TextDisabled("%.0f FPS", m_fps);
    ImGui::End();
}

void EditorUI::SpawnRobotPreset(const char* fileName) {
    std::string path = fileName;
#ifdef SIM_ROBOTS_DIR
    path = std::string(SIM_ROBOTS_DIR) + "/" + fileName;
#endif
    LoadRobotFromPath(path);
}

void EditorUI::LoadRobotFromPath(const std::string& path) {
    if (!m_sim || path.empty()) return;
    m_sim->ReleaseRobot();
    auto robot = std::make_shared<Robot>();
    robot->BuildFromJson(m_sim->GetWorld(), path);
    m_sim->SetRobot(robot);
    if (m_sim->TryGetRenderer()) m_sim->GetRenderer().BindAll();
    Log("[Editor] Loaded " + path);
}

void EditorUI::ApplyDriveMode() {
    if (!m_sim) return;
    if (m_controllerIndex == 0) {
        auto controller = std::make_shared<KeyboardController>(m_sim->GetRenderer());
        m_sim->GetRenderer().SetKeyCallback([controller](int key, bool pressed) {
            controller->OnKeyEvent(key, pressed);
        });
        m_sim->SetController(controller);
        Log("[Editor] Keyboard drive");
    } else if (m_controllerIndex == 1) {
        auto script = std::make_shared<ScriptController>();
        script->AddCommand(4.0, 4.0, 2.0);
        script->AddCommand(2.0, -2.0, 1.0);
        script->AddCommand(4.0, 4.0, 2.0);
        m_sim->SetController(script);
        Log("[Editor] Script drive");
    } else {
        m_sim->SetController(nullptr);
        Log("[Editor] Drive off");
    }
}

void EditorUI::SetSimulationPlaying(bool playing) {
    if (!m_sim) return;
    if (playing) {
        m_sim->Resume();
        m_state.isPlaying = true;
        m_state.isPaused = false;
    } else {
        m_sim->Pause();
        m_state.isPlaying = false;
        m_state.isPaused = true;
    }
}

void EditorUI::ResetRobot() {
    if (!m_sim || !m_sim->GetRobot()) return;
    auto spawn = m_sim->GetSceneManager().GetSpawnPoint();
    const auto& cfg = m_sim->GetRobot()->GetConfig();
    m_sim->GetRobot()->Reset(spawn.x, cfg.spawnY, spawn.z);
    Log("[Editor] Robot reset");
}

void EditorUI::HandleAssemblerMouse(int x, int y, bool leftDown, bool rightDown, float wheel) {
    (void)rightDown;
    (void)wheel;
    const bool click = leftDown && !m_mouseLeftDown;
    m_mouseLeftDown = leftDown;
    if (click && m_state.showRobotBuilder &&
        (!m_selectedPartId.empty() || m_selectedShape >= 0)) {
        m_pendingClickX = x;
        m_pendingClickY = y;
    }
}

AssemblyJointType EditorUI::SelectedJointType() const {
    switch (m_jointTypeIndex) {
        case 0: return AssemblyJointType::Fixed;
        case 2: return AssemblyJointType::Prismatic;
        case 3: return AssemblyJointType::Spherical;
        default: return AssemblyJointType::Revolute;
    }
}

static double RestingY(const AssemblyBody& body) {
    const AssemblyShape shape = body.shape == AssemblyShape::Mesh ? body.collisionShape : body.shape;
    double height = 0.05;
    if (shape == AssemblyShape::Box) height = body.sizeY * 0.5;
    else if (shape == AssemblyShape::Sphere) height = body.radius;
    else if (shape == AssemblyShape::Cylinder) height = body.height * 0.5;
    else if (shape == AssemblyShape::Capsule) height = body.height * 0.5 + body.radius;
    return height + 0.01;
}

static void FillBodyFromCatalog(AssemblyBody& body, const CatalogPart& part) {
    body.partId = part.id;
    body.shapeSpecified = true;
    if (!ParseShapeName(part.collision.shape, body.shape) || body.shape == AssemblyShape::Mesh) {
        body.shape = AssemblyShape::Box;
    }
    body.collisionShape = body.shape;
    body.collisionSpecified = true;
    body.sizeX = part.collision.sizeX;
    body.sizeY = part.collision.sizeY;
    body.sizeZ = part.collision.sizeZ;
    body.radius = part.collision.radius;
    body.height = part.collision.height;
    body.mass = part.mass > 0.0 ? part.mass : 0.2;
    body.hasMass = true;
    body.visual = part.visual;
    body.collision = !part.visualOnly;
    body.friction = (body.shape == AssemblyShape::Cylinder) ? 1.0 : 0.5;
}

void EditorUI::RebuildFromAssembly() {
    m_assemblyDirty = true;
}

void EditorUI::ApplyPendingAssembly() {
    if (!m_assemblyDirty || !m_sim) return;
    m_assemblyDirty = false;
    try {
        if (LiftAssemblyOntoGround(m_graph)) {
            m_poseEditId.clear();
            Log("[Editor] Raised the robot so it sits on the ground.");
        }
        m_sim->ReleaseRobot();
        if (m_graph.bodies.empty()) {
            m_sim->SetRobot(nullptr);
            return;
        }
        if (m_graph.root.empty()) m_graph.root = m_graph.bodies.front().id;
        m_graph.name = m_graph.name.empty() ? "assembled" : m_graph.name;
        auto robot = std::make_shared<Robot>();
        const std::string preview = m_showCollisionPreview ? m_selectedInstanceId : std::string();
        if (!robot->BuildFromGraph(m_sim->GetWorld(), m_graph, preview)) {
            for (const auto& error : robot->GetBuildErrors()) Log("[Editor] " + error);
        }
        m_sim->SetRobot(robot);
        if (m_sim->TryGetRenderer()) m_sim->GetRenderer().BindAll();
    } catch (const std::bad_alloc&) {
        Log("[Editor] Ran out of memory while building the robot. Remove a part and try again.");
        try {
            m_sim->ReleaseRobot();
        } catch (...) {
        }
    } catch (const std::exception& ex) {
        Log(std::string("[Editor] Could not build the robot: ") + ex.what());
    }
}

void EditorUI::SnapSelectedPart(const std::string& parentId, const std::string& mountId) {
    if (m_selectedPartId.empty()) {
        Log("[Editor] Select a catalog part first");
        return;
    }
    auto& catalog = PartCatalog::Instance();
    const CatalogPart* part = catalog.Find(m_selectedPartId);
    if (!part) return;

    AssemblyBody body;
    body.id = part->id + "_" + std::to_string(m_instanceSerial++);
    FillBodyFromCatalog(body, *part);
    bool replaced = false;

    if (parentId.empty()) {
        body.pose = MakePose(0.0, RestingY(body), 0.0);
    } else {
        const AssemblyBody* parent = m_graph.FindBody(parentId);
        const CatalogPart* parentPart = parent ? catalog.Find(parent->partId) : nullptr;
        const MountFrame* mount = (parentPart && !mountId.empty()) ? catalog.FindMount(*parentPart, mountId) : nullptr;
        if (!parent || !mount) {
            Log("[Editor] Pick a mount on a part that is already in the assembly");
            return;
        }
        const double mountX = mount->x, mountY = mount->y, mountZ = mount->z;
        const double axisX = mount->axisX, axisY = mount->axisY, axisZ = mount->axisZ;
        const auto occupied = BodiesOnMount(m_graph, parentId, mountX, mountY, mountZ);
        replaced = !occupied.empty();
        for (const auto& id : occupied) {
            if (m_graph.FindBody(id)) EraseBodyTree(id);
        }
        const bool round = body.shape == AssemblyShape::Cylinder || body.shape == AssemblyShape::Capsule;
        if (round) {
            body.pose = PoseAlignYToAxis(mountX, mountY, mountZ, axisX, axisY, axisZ);
        } else {
            body.pose = MakePose(mountX, mountY, mountZ);
        }
        body.parent = parentId;

        AssemblyJoint joint;
        joint.id = "j_" + body.id;
        joint.type = SelectedJointType();
        joint.bodyA = parentId;
        joint.bodyB = body.id;
        joint.anchorX = mountX;
        joint.anchorY = mountY;
        joint.anchorZ = mountZ;
        joint.axisX = axisX;
        joint.axisY = axisY;
        joint.axisZ = axisZ;
        m_graph.joints.push_back(joint);

        const bool drivable = joint.type == AssemblyJointType::Revolute || joint.type == AssemblyJointType::Prismatic;
        if (m_attachMotor && drivable) {
            AssemblyMotor motor;
            motor.id = "m_" + body.id;
            motor.jointId = joint.id;
            motor.mode = AssemblyMotorMode::Velocity;
            m_graph.motors.push_back(motor);
        }
    }

    m_graph.bodies.push_back(std::move(body));
    if (m_graph.root.empty()) m_graph.root = m_graph.bodies.back().id;
    m_selectedInstanceId = m_graph.bodies.back().id;
    m_poseEditId.clear();
    RebuildFromAssembly();
    const char* action = replaced ? "Replaced with " : "Added ";
    Log(std::string("[Editor] ") + action + part->name + (mountId.empty() ? "" : " on " + MountLabel(mountId)));
}

void EditorUI::AddPrimitiveBody(AssemblyShape shape, const std::string& parentId, const std::string& mountId) {
    AssemblyBody body;
    body.id = std::string(ShapeName(shape)) + "_" + std::to_string(m_instanceSerial++);
    body.shape = shape;
    body.shapeSpecified = true;
    body.collisionShape = shape;
    body.hasMass = true;
    body.mass = 1.0;
    body.friction = 0.5;
    if (shape == AssemblyShape::Box) {
        body.sizeX = 0.2;
        body.sizeY = 0.1;
        body.sizeZ = 0.12;
    } else if (shape == AssemblyShape::Sphere) {
        body.radius = 0.06;
    } else if (shape == AssemblyShape::Cylinder || shape == AssemblyShape::Capsule) {
        body.radius = 0.05;
        body.height = 0.12;
    }

    if (!parentId.empty()) {
        const AssemblyBody* parent = m_graph.FindBody(parentId);
        auto& catalog = PartCatalog::Instance();
        const CatalogPart* parentPart = parent ? catalog.Find(parent->partId) : nullptr;
        const MountFrame* mount = (parentPart && !mountId.empty()) ? catalog.FindMount(*parentPart, mountId) : nullptr;
        if (!parent || !mount) {
            Log("[Editor] Pick a socket on a part that is already on the robot");
            return;
        }
        const double mountX = mount->x, mountY = mount->y, mountZ = mount->z;
        const double axisX = mount->axisX, axisY = mount->axisY, axisZ = mount->axisZ;
        const auto occupied = BodiesOnMount(m_graph, parentId, mountX, mountY, mountZ);
        for (const auto& id : occupied) {
            if (m_graph.FindBody(id)) EraseBodyTree(id);
        }
        const bool round = shape == AssemblyShape::Cylinder || shape == AssemblyShape::Capsule;
        if (round) {
            body.pose = PoseAlignYToAxis(mountX, mountY, mountZ, axisX, axisY, axisZ);
        } else {
            body.pose = MakePose(mountX, mountY, mountZ);
        }
        body.parent = parentId;
        AssemblyJoint joint;
        joint.id = "j_" + body.id;
        joint.type = SelectedJointType();
        joint.bodyA = parentId;
        joint.bodyB = body.id;
        joint.anchorX = mountX;
        joint.anchorY = mountY;
        joint.anchorZ = mountZ;
        joint.axisX = axisX;
        joint.axisY = axisY;
        joint.axisZ = axisZ;
        m_graph.joints.push_back(joint);
        if (m_attachMotor && (joint.type == AssemblyJointType::Revolute || joint.type == AssemblyJointType::Prismatic)) {
            AssemblyMotor motor;
            motor.id = "m_" + body.id;
            motor.jointId = joint.id;
            motor.mode = AssemblyMotorMode::Velocity;
            m_graph.motors.push_back(motor);
        }
    } else {
        body.pose = MakePose(0.0, RestingY(body), 0.0);
    }
    m_graph.bodies.push_back(std::move(body));
    if (m_graph.root.empty()) m_graph.root = m_graph.bodies.back().id;
    m_selectedInstanceId = m_graph.bodies.back().id;
    m_poseEditId.clear();
    RebuildFromAssembly();
}

void EditorUI::EraseBodyTree(const std::string& id) {
    std::vector<std::string> kill;
    std::function<void(const std::string&)> walk = [&](const std::string& current) {
        kill.push_back(current);
        for (const auto& body : m_graph.bodies) {
            if (body.parent == current) walk(body.id);
        }
    };
    walk(id);
    std::unordered_set<std::string> dead(kill.begin(), kill.end());
    std::unordered_set<std::string> deadJoints;
    m_graph.joints.erase(std::remove_if(m_graph.joints.begin(), m_graph.joints.end(), [&](const AssemblyJoint& joint) {
        const bool drop = dead.count(joint.bodyA) || dead.count(joint.bodyB);
        if (drop) deadJoints.insert(joint.id);
        return drop;
    }), m_graph.joints.end());
    m_graph.motors.erase(std::remove_if(m_graph.motors.begin(), m_graph.motors.end(), [&](const AssemblyMotor& motor) {
        return deadJoints.count(motor.jointId);
    }), m_graph.motors.end());
    m_graph.bodies.erase(std::remove_if(m_graph.bodies.begin(), m_graph.bodies.end(), [&](const AssemblyBody& body) {
        return dead.count(body.id);
    }), m_graph.bodies.end());
    if (dead.count(m_graph.root)) {
        m_graph.root = m_graph.bodies.empty() ? std::string() : m_graph.bodies.front().id;
    }
    if (dead.count(m_selectedInstanceId)) m_selectedInstanceId.clear();
    m_poseEditId.clear();
}

void EditorUI::DeleteBodyTree(const std::string& id) {
    EraseBodyTree(id);
    RebuildFromAssembly();
}

void EditorUI::DuplicateSelectedBody() {
    const AssemblyBody* source = m_graph.FindBody(m_selectedInstanceId);
    if (!source) return;
    AssemblyBody copy = *source;
    copy.id = source->id + "_copy";
    copy.pose.px += 0.06;
    if (!copy.parent.empty()) {
        for (const auto& joint : m_graph.joints) {
            if (joint.bodyB != source->id) continue;
            AssemblyJoint copied = joint;
            copied.id = "j_" + copy.id;
            copied.bodyB = copy.id;
            copied.anchorX += 0.06;
            m_graph.joints.push_back(copied);
            for (const auto& motor : m_graph.motors) {
                if (motor.jointId != joint.id) continue;
                AssemblyMotor motorCopy = motor;
                motorCopy.id = "m_" + copy.id;
                motorCopy.jointId = copied.id;
                m_graph.motors.push_back(motorCopy);
                break;
            }
            break;
        }
    }
    m_graph.bodies.push_back(std::move(copy));
    m_selectedInstanceId = m_graph.bodies.back().id;
    m_poseEditId.clear();
    RebuildFromAssembly();
}

void EditorUI::LoadGraphFile(const std::string& path) {
    std::ifstream in(path);
    if (!in) {
        Log("[Editor] Could not open " + path);
        return;
    }
    nlohmann::json j;
    try {
        in >> j;
    } catch (const nlohmann::json::exception& e) {
        Log(std::string("[Editor] JSON error: ") + e.what());
        return;
    }
    if (!AssemblyGraph::IsSchemaV1(j)) {
        Log("[Editor] That file is a legacy robot. Use Examples or the Robot menu to load it.");
        return;
    }
    AssemblyIssue issue;
    AssemblyGraph graph = AssemblyGraph::FromJson(j, issue);
    if (!issue.errors.empty()) {
        for (const auto& error : issue.errors) Log("[Editor] " + error);
        return;
    }
    m_graph = std::move(graph);
    m_selectedInstanceId = m_graph.root;
    m_poseEditId.clear();
    RebuildFromAssembly();
    Log("[Editor] Loaded " + path);
}

void EditorUI::RememberPoseEditor() {
    if (m_poseEditId == m_selectedInstanceId) return;
    m_poseEditId = m_selectedInstanceId;
    const AssemblyBody* body = m_graph.FindBody(m_selectedInstanceId);
    if (!body) return;
    m_poseEdit[0] = static_cast<float>(body->pose.px);
    m_poseEdit[1] = static_cast<float>(body->pose.py);
    m_poseEdit[2] = static_cast<float>(body->pose.pz);
    double rx, ry, rz;
    PoseToEulerDeg(body->pose, rx, ry, rz);
    m_eulerEdit[0] = static_cast<float>(rx);
    m_eulerEdit[1] = static_cast<float>(ry);
    m_eulerEdit[2] = static_cast<float>(rz);
    for (const auto& joint : m_graph.joints) {
        if (joint.bodyB != body->id) continue;
        m_axisEdit[0] = static_cast<float>(joint.axisX);
        m_axisEdit[1] = static_cast<float>(joint.axisY);
        m_axisEdit[2] = static_cast<float>(joint.axisZ);
        break;
    }
}

bool EditorUI::TryPickMount(int mx, int my) {
    if (m_graph.bodies.empty() || !m_sim || !m_sim->GetRobot()) {
        return false;
    }
    auto& catalog = PartCatalog::Instance();

    double ex, ey, ez, tx, ty, tz;
    if (!m_sim->GetRenderer().GetCameraLook(ex, ey, ez, tx, ty, tz)) return false;

    double fx = tx - ex, fy = ty - ey, fz = tz - ez;
    double fl = std::sqrt(fx * fx + fy * fy + fz * fz);
    if (fl < 1e-6) return false;
    fx /= fl; fy /= fl; fz /= fl;
    // Y-up camera
    double rx = fy * 0 - fz * 1.0;
    double ry = fz * 0 - fx * 0;
    double rz = fx * 1.0 - fy * 0;
    double rl = std::sqrt(rx * rx + ry * ry + rz * rz);
    if (rl < 1e-6) { rx = 1; ry = 0; rz = 0; rl = 1; }
    rx /= rl; ry /= rl; rz /= rl;
    double ux = ry * fz - rz * fy;
    double uy = rz * fx - rx * fz;
    double uz = rx * fy - ry * fx;

    const double fov = 40.0 * 3.141592653589793 / 180.0;
    const double tanHalf = std::tan(fov * 0.5);
    const double aspect = (double)m_sim->GetRenderer().GetWindowWidth() /
                          std::max(1, m_sim->GetRenderer().GetWindowHeight());
    const double ndcX = (2.0 * mx / std::max(1, m_sim->GetRenderer().GetWindowWidth())) - 1.0;
    const double ndcY = 1.0 - (2.0 * my / std::max(1, m_sim->GetRenderer().GetWindowHeight()));
    double dx = fx + rx * ndcX * tanHalf * aspect + ux * ndcY * tanHalf;
    double dy = fy + ry * ndcX * tanHalf * aspect + uy * ndcY * tanHalf;
    double dz = fz + rz * ndcX * tanHalf * aspect + uz * ndcY * tanHalf;
    double dl = std::sqrt(dx * dx + dy * dy + dz * dz);
    dx /= dl; dy /= dl; dz /= dl;

    std::string bestParent;
    std::string bestMount;
    double bestDist = 0.12;
    for (const auto& spec : m_graph.bodies) {
        const CatalogPart* part = catalog.Find(spec.partId);
        if (!part || part->mounts.empty()) continue;
        auto rigid = m_sim->GetRobot()->FindBody(spec.id);
        if (!rigid) continue;
        double bx, by, bz, qw, qx, qy, qz;
        rigid->GetPosition(bx, by, bz);
        rigid->GetRotation(qw, qx, qy, qz);
        chrono::ChFrame<> frame(chrono::ChVector3d(bx, by, bz), chrono::ChQuaterniond(qw, qx, qy, qz));
        for (const auto& mount : part->mounts) {
            chrono::ChVector3d world = frame.TransformPointLocalToParent(
                chrono::ChVector3d(mount.x, mount.y, mount.z));
            double vx = world.x() - ex, vy = world.y() - ey, vz = world.z() - ez;
            double t = vx * dx + vy * dy + vz * dz;
            if (t < 0) continue;
            double qx2 = ex + dx * t - world.x();
            double qy2 = ey + dy * t - world.y();
            double qz2 = ez + dz * t - world.z();
            double dist = std::sqrt(qx2 * qx2 + qy2 * qy2 + qz2 * qz2);
            if (dist < bestDist) {
                bestDist = dist;
                bestParent = spec.id;
                bestMount = mount.id;
            }
        }
    }
    if (bestMount.empty()) return false;
    SnapSelectedPart(bestParent, bestMount);
    return true;
}

void EditorUI::RenderAssemblerPanel() {
    auto& catalog = PartCatalog::Instance();
    if (!catalog.IsLoaded()) {
        if (ImGui::Button("Load part catalog")) {
            catalog.Load();
        }
        ImGui::TextWrapped("Catalog not loaded. Expected assets/parts/catalog.json");
        return;
    }

    if (m_pendingClickX >= 0 && !ImGui::GetIO().WantCaptureMouse) {
        TryPickMount(m_pendingClickX, m_pendingClickY);
        m_pendingClickX = -1;
        m_pendingClickY = -1;
    } else if (m_pendingClickX >= 0) {
        m_pendingClickX = -1;
        m_pendingClickY = -1;
    }

    const bool highlighted = !m_selectedPartId.empty() || m_selectedShape >= 0;
    if (m_graph.bodies.empty()) {
        ImGui::TextWrapped(highlighted
            ? "Add to robot places it on the ground."
            : "Choose a part, then click Add to robot.");
    } else if (!highlighted) {
        ImGui::TextWrapped("Choose a part, then click a socket.");
    } else {
        ImGui::TextWrapped("Click a socket to attach the highlighted part. A filled socket is replaced.");
    }

    auto choosePart = [&](const CatalogPart& part) {
        m_selectedPartId = part.id;
        m_selectedShape = -1;
        AssemblyJointType jointType = AssemblyJointType::Revolute;
        if (ParseJointTypeName(part.joint, jointType)) {
            if (jointType == AssemblyJointType::Fixed) m_jointTypeIndex = 0;
            else if (jointType == AssemblyJointType::Prismatic) m_jointTypeIndex = 2;
            else if (jointType == AssemblyJointType::Spherical) m_jointTypeIndex = 3;
            else m_jointTypeIndex = 1;
        }
        m_attachMotor = jointType == AssemblyJointType::Revolute
            || jointType == AssemblyJointType::Prismatic;
    };

    auto bodyLabel = [&](const AssemblyBody& spec) {
        std::string base;
        if (!spec.partId.empty()) {
            if (const CatalogPart* part = catalog.Find(spec.partId)) base = part->name;
            else base = spec.partId;
        } else {
            base = ShapeLabel(spec.shape);
        }
        int number = 1;
        for (const auto& other : m_graph.bodies) {
            if (other.id == spec.id) break;
            const bool samePart = !spec.partId.empty() && other.partId == spec.partId;
            const bool sameShape = spec.partId.empty() && other.partId.empty() && other.shape == spec.shape;
            if (samePart || sameShape) ++number;
        }
        return base + " " + std::to_string(number);
    };

    auto place = [&](const std::string& parentId, const std::string& mountId) {
        if (m_selectedShape >= 0 && m_selectedShape <= static_cast<int>(AssemblyShape::Capsule)) {
            AddPrimitiveBody(static_cast<AssemblyShape>(m_selectedShape), parentId, mountId);
        } else {
            SnapSelectedPart(parentId, mountId);
        }
    };

    SectionTitle("Library");
    ImGui::BeginChild("palette", ImVec2(0, 220), true);
    auto drawGroup = [&](const char* title, const auto& belongs) {
        std::vector<const CatalogPart*> parts;
        for (const auto& part : catalog.Parts()) {
            if (belongs(part.category)) parts.push_back(&part);
        }
        if (parts.empty()) return;
        if (!ImGui::TreeNodeEx(title, 0)) return;
        for (const auto* part : parts) {
            const std::string row = part->name + "##" + part->id;
            if (ImGui::Selectable(row.c_str(), m_selectedPartId == part->id)) choosePart(*part);
        }
        ImGui::TreePop();
    };
    const auto known = [](const std::string& category) {
        return CategoryIn(category, {
            "chassis", "wheel", "caster", "hub", "beam", "plate", "bracket",
            "axle", "spacer", "arm", "sensor", "obstacle", "ground"});
    };
    drawGroup("Bodies", [](const std::string& category) { return category == "chassis"; });
    drawGroup("Wheels", [](const std::string& category) {
        return CategoryIn(category, {"wheel", "caster", "hub"});
    });
    drawGroup("Frame", [&](const std::string& category) {
        return CategoryIn(category, {"beam", "plate", "bracket", "axle", "spacer"}) || !known(category);
    });
    drawGroup("Arm", [](const std::string& category) { return category == "arm"; });
    drawGroup("Sensors", [](const std::string& category) { return category == "sensor"; });
    if (ImGui::TreeNodeEx("Shapes", 0)) {
        const char* names[] = {"Box", "Sphere", "Cylinder", "Capsule"};
        for (int i = 0; i < 4; ++i) {
            if (ImGui::Selectable(names[i], m_selectedShape == i)) {
                m_selectedShape = i;
                m_selectedPartId.clear();
            }
        }
        ImGui::TreePop();
    }
    ImGui::EndChild();

    if (m_selectedShape >= 0 && m_selectedShape <= static_cast<int>(AssemblyShape::Capsule)) {
        ImGui::TextWrapped("%s. It uses the connection below.", ShapeLabel(static_cast<AssemblyShape>(m_selectedShape)));
    } else if (const CatalogPart* part = catalog.Find(m_selectedPartId)) {
        const char* behavior = "It will spin unless you change the connection.";
        if (m_jointTypeIndex == 0) behavior = "It will be welded unless you change the connection.";
        else if (m_jointTypeIndex == 2) behavior = "It will slide unless you change the connection.";
        else if (m_jointTypeIndex == 3) behavior = "It will be a ball joint unless you change the connection.";
        ImGui::TextWrapped("%s. %s", part->name.c_str(), behavior);
    }

    SectionTitle("Add");
    if (m_graph.bodies.empty()) {
        ImGui::BeginDisabled(!highlighted);
        if (ImGui::Button("Add to robot", ImVec2(-1, 0))) place({}, {});
        ImGui::EndDisabled();
    } else {
        bool anySocket = false;
        ImGui::BeginDisabled(!highlighted);
        const ImGuiStyle& style = ImGui::GetStyle();
        const float regionEnd = ImGui::GetWindowPos().x + ImGui::GetWindowContentRegionMax().x;
        for (const auto& spec : m_graph.bodies) {
            const CatalogPart* part = catalog.Find(spec.partId);
            if (!part || part->mounts.empty()) continue;
            anySocket = true;
            ImGui::TextUnformatted(bodyLabel(spec).c_str());
            int index = 0;
            for (const auto& mount : part->mounts) {
                std::string visible = MountLabel(mount.id);
                const auto occupied = BodiesOnMount(m_graph, spec.id, mount.x, mount.y, mount.z);
                if (!occupied.empty()) {
                    if (const AssemblyBody* occupant = m_graph.FindBody(occupied.front())) {
                        visible += " · " + bodyLabel(*occupant);
                    }
                }
                const std::string id = visible + "##" + spec.id + "/" + mount.id;
                const float width = ImGui::CalcTextSize(visible.c_str()).x + style.FramePadding.x * 2.0f;
                if (index > 0) {
                    const float next = ImGui::GetItemRectMax().x + style.ItemSpacing.x + width;
                    if (next < regionEnd) ImGui::SameLine();
                }
                if (ImGui::Button(id.c_str())) place(spec.id, mount.id);
                ++index;
            }
        }
        ImGui::EndDisabled();
        if (!anySocket) ImGui::TextDisabled("Placed parts have no sockets yet.");
    }

    ImGui::SetNextItemWidth(160);
    ImGui::Combo("Connection", &m_jointTypeIndex, kJointWords);
    if (m_jointTypeIndex == 1 || m_jointTypeIndex == 2) {
        ImGui::Checkbox("Powered", &m_attachMotor);
    }

    SectionTitle("Your robot");
    ImGui::BeginChild("assembly", ImVec2(0, 140), true);
    std::string removeId;
    std::function<void(const std::string&, int)> drawBody = [&](const std::string& parent, int depth) {
        for (const auto& spec : m_graph.bodies) {
            if (spec.parent != parent) continue;
            const std::string row = bodyLabel(spec) + "##" + spec.id;
            ImGui::SetCursorPosX(ImGui::GetWindowContentRegionMin().x + depth * 16.0f);
            ImGui::PushID(spec.id.c_str());
            if (ImGui::SmallButton("Remove")) removeId = spec.id;
            ImGui::SameLine();
            if (ImGui::Selectable(row.c_str(), m_selectedInstanceId == spec.id)) {
                m_selectedInstanceId = spec.id;
                m_poseEditId.clear();
            }
            ImGui::PopID();
            drawBody(spec.id, depth + 1);
        }
    };
    drawBody("", 0);
    ImGui::EndChild();
    if (!removeId.empty()) DeleteBodyTree(removeId);

    AssemblyBody* selected = m_graph.FindBody(m_selectedInstanceId);
    if (ImGui::Button("Duplicate") && selected) DuplicateSelectedBody();
    ImGui::SameLine();
    if (ImGui::Button("Remove") && selected) DeleteBodyTree(selected->id);
    ImGui::SameLine();
    if (ImGui::Button("Clear")) {
        m_graph = {};
        m_selectedInstanceId.clear();
        m_poseEditId.clear();
        RebuildFromAssembly();
    }
    selected = m_graph.FindBody(m_selectedInstanceId);
    RememberPoseEditor();

    if (selected) {
        SectionTitle("Selected part");
        if (ImGui::DragFloat3("Position", m_poseEdit, 0.01f)) {
            if (ImGui::IsItemDeactivatedAfterEdit()) {
                selected->pose.px = m_poseEdit[0];
                selected->pose.py = m_poseEdit[1];
                selected->pose.pz = m_poseEdit[2];
                RebuildFromAssembly();
            }
        }
        AssemblyJoint* owned = nullptr;
        for (auto& joint : m_graph.joints) {
            if (joint.bodyB == selected->id) owned = &joint;
        }
        if (owned) {
            int typeIndex = 1;
            if (owned->type == AssemblyJointType::Fixed) typeIndex = 0;
            else if (owned->type == AssemblyJointType::Prismatic) typeIndex = 2;
            else if (owned->type == AssemblyJointType::Spherical) typeIndex = 3;
            ImGui::SetNextItemWidth(160);
            if (ImGui::Combo("Connection##placed", &typeIndex, kJointWords)) {
                AssemblyJointType next = AssemblyJointType::Revolute;
                if (typeIndex == 0) next = AssemblyJointType::Fixed;
                else if (typeIndex == 2) next = AssemblyJointType::Prismatic;
                else if (typeIndex == 3) next = AssemblyJointType::Spherical;
                owned->type = next;
                if (next == AssemblyJointType::Fixed || next == AssemblyJointType::Spherical) {
                    m_graph.motors.erase(std::remove_if(m_graph.motors.begin(), m_graph.motors.end(),
                        [&](const AssemblyMotor& motor) { return motor.jointId == owned->id; }),
                        m_graph.motors.end());
                }
                RebuildFromAssembly();
            }
            const bool drivable = owned->type == AssemblyJointType::Revolute
                || owned->type == AssemblyJointType::Prismatic;
            if (drivable) {
                bool hasMotor = false;
                for (const auto& motor : m_graph.motors) {
                    if (motor.jointId == owned->id) hasMotor = true;
                }
                if (ImGui::Checkbox("Powered##placed", &hasMotor)) {
                    if (hasMotor) {
                        AssemblyMotor motor;
                        motor.id = "m_" + selected->id;
                        motor.jointId = owned->id;
                        motor.mode = AssemblyMotorMode::Velocity;
                        m_graph.motors.push_back(motor);
                    } else {
                        m_graph.motors.erase(std::remove_if(m_graph.motors.begin(), m_graph.motors.end(),
                            [&](const AssemblyMotor& motor) { return motor.jointId == owned->id; }),
                            m_graph.motors.end());
                    }
                    RebuildFromAssembly();
                }
            }
        }
        if (ImGui::Checkbox("Collision preview", &m_showCollisionPreview)) RebuildFromAssembly();
        if (ImGui::CollapsingHeader("More")) {
            if (ImGui::DragFloat3("Rotation", m_eulerEdit, 1.0f)) {
                if (ImGui::IsItemDeactivatedAfterEdit()) {
                    selected->pose = PoseFromEulerDeg(
                        selected->pose.px, selected->pose.py, selected->pose.pz,
                        m_eulerEdit[0], m_eulerEdit[1], m_eulerEdit[2]);
                    RebuildFromAssembly();
                }
            }
            if (owned && ImGui::DragFloat3("Axis", m_axisEdit, 0.05f)) {
                if (ImGui::IsItemDeactivatedAfterEdit()) {
                    owned->axisX = m_axisEdit[0];
                    owned->axisY = m_axisEdit[1];
                    owned->axisZ = m_axisEdit[2];
                    RebuildFromAssembly();
                }
            }
        }
    }

    ImGui::Separator();
    ImGui::TextDisabled("Assembly file");
    ImGui::SetNextItemWidth(-1);
    ImGui::InputText("##savegraph", m_robotPathBuf, sizeof(m_robotPathBuf));
    if (ImGui::Button("Save##graph")) {
        std::ofstream out(m_robotPathBuf);
        if (out) {
            out << m_graph.ToJson().dump(2);
            Log(std::string("[Editor] Saved assembly to ") + m_robotPathBuf);
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("Load##graph")) LoadGraphFile(m_robotPathBuf);
}

}  // namespace robosim

