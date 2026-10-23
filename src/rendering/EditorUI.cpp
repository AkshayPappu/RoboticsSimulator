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
#include <cstdio>
#include <chrono>
#include <algorithm>
#include <cmath>

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

    Log("[Editor] Click the 3D window, then drive with W A S D or arrow keys.");
    Log("[Editor] Robot Builder: pick a catalog part, then click a mount to snap.");
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
            if (ImGui::MenuItem("Undo Delete", "Ctrl+Z")) {
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
            ImGui::MenuItem("Hierarchy", nullptr, &m_state.showHierarchy);
            ImGui::MenuItem("Inspector", nullptr, &m_state.showInspector);
            ImGui::MenuItem("Toolbar", nullptr, &m_state.showToolbar);
            ImGui::MenuItem("Console", nullptr, &m_state.showConsole);
            ImGui::MenuItem("Robot Builder", nullptr, &m_state.showRobotBuilder);
            ImGui::MenuItem("Help overlay", nullptr, &m_state.showHelp);
            ImGui::Separator();
            ImGui::MenuItem("ImGui Demo", nullptr, &m_state.showDemoWindow);
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Robot")) {
            if (ImGui::MenuItem("Differential drive")) {
                SpawnRobotPreset("differential_drive.json");
            }
            if (ImGui::MenuItem("Four-wheel")) {
                SpawnRobotPreset("four_wheel.json");
            }
            if (ImGui::MenuItem("Robot arm")) {
                SpawnRobotPreset("robot_arm.json");
            }
            ImGui::Separator();
            ImGui::MenuItem("Open builder...", nullptr, &m_state.showRobotBuilder);
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
                    const auto& cfg = m_sim->GetRobot()->GetConfig();
                    m_sim->GetRobot()->Reset(spawn.x, cfg.spawnY, spawn.z);
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
            const auto& cfg = m_sim->GetRobot()->GetConfig();
            m_sim->GetRobot()->Reset(spawn.x, cfg.spawnY, spawn.z);
        }
    }

    ImGui::SameLine();
    ImGui::SetNextItemWidth(100);
    ImGui::SliderFloat("Speed", &m_state.speedMultiplier, 0.25f, 4.0f, "%.2fx");
    m_sim->SetSpeedMultiplier(m_state.speedMultiplier);

    ImGui::Text("Time: %.3f s  |  Step: %d", m_sim->GetSimTime(), m_sim->GetStepCount());

    if (auto* robot = m_sim->GetRobot()) {
        double x, y, z;
        robot->GetPosition(x, y, z);
        ImGui::Text("Robot  x=%.2f  y=%.2f  z=%.2f   heading %.0f deg",
                    x, y, z, robot->GetHeading() * 180.0 / 3.14159);
        ImGui::Text("Wheels L=%.1f  R=%.1f rad/s   %s",
                    robot->GetLeftWheelSpeed(), robot->GetRightWheelSpeed(),
                    robot->IsFlipped() ? "FLIPPED - press Reset" : "W/S drive  A/D turn");
    } else {
        ImGui::TextUnformatted("No robot. Robot menu -> Differential drive");
    }
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
        m_undoDeleted.push_back(obj);
        m_sim->GetSceneManager().RemoveObject(obj);
        m_state.selectedObjectIndex = -1;
        Log("[Editor] Deleted object (Undo from File menu)");
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
        // Convert quaternion to Euler (XYZ)
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
        if (ImGui::DragFloat3("Euler (deg)", euler, 1.0f)) {
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

        ImGui::Separator();
        ImGui::Text("Velocity");
        double vx, vy, vz;
        obj->GetLinearVelocity(vx, vy, vz);
        ImGui::Text("Linear: (%.2f, %.2f, %.2f)", vx, vy, vz);
    } else if (m_sim->GetRobot()) {
        ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "Robot Sensors");
        for (const auto& sensor : m_sim->GetRobot()->GetSensors()) {
            ImGui::Separator();
            ImGui::Text("%s", sensor->GetName().c_str());
            if (sensor->GetType() == SensorType::Distance) {
                auto* d = static_cast<DistanceSensor*>(sensor.get());
                ImGui::Text("  Distance: %.3f m  hit=%s", d->GetDistance(), d->HasHit() ? "yes" : "no");
            } else if (sensor->GetType() == SensorType::IMU) {
                auto* imu = static_cast<IMUSensor*>(sensor.get());
                auto a = imu->GetLinearAccel();
                auto w = imu->GetAngularVel();
                ImGui::Text("  Accel: (%.2f, %.2f, %.2f)", a.x(), a.y(), a.z());
                ImGui::Text("  Gyro:  (%.2f, %.2f, %.2f)", w.x(), w.y(), w.z());
            } else if (sensor->GetType() == SensorType::Contact) {
                auto* c = static_cast<ContactSensor*>(sensor.get());
                ImGui::Text("  Contact: %s", c->InContact() ? "YES" : "no");
            }
        }
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
    ImGui::SetNextWindowSize(ImVec2(440, 620), ImGuiCond_FirstUseEver);

    ImGui::Begin("Robot Builder", &m_state.showRobotBuilder);

    if (ImGui::BeginTabBar("BuilderTabs")) {
        if (ImGui::BeginTabItem("Assemble")) {
            RenderAssemblerPanel();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Presets")) {

    static float chassisSize[3] = {0.5f, 0.15f, 0.3f};
    static float chassisMass = 5.0f;
    static float wheelRadius = 0.1f;
    static float wheelWidth = 0.05f;
    static float wheelOffsetZ = 0.2f;
    static float maxMotorSpeed = 10.0f;
    static float maxTorque = 2.0f;
    static int wheelCount = 2;
    static int casterCount = 1;

    const char* presets[] = {
        "Custom",
        "robots/differential_drive.json",
        "robots/four_wheel.json",
        "robots/robot_arm.json"
    };
    auto replaceRobot = [&](const std::function<void(Robot&)>& builder) {
        if (m_sim->GetRobot()) {
            m_sim->GetRobot()->RemoveFromWorld(m_sim->GetWorld());
        }
        auto robot = std::make_shared<Robot>();
        builder(*robot);
        m_sim->SetRobot(robot);
        m_sim->GetRenderer().BindAll();
    };

    if (ImGui::Combo("Preset", &m_presetIndex, presets, 4)) {
        if (m_presetIndex > 0) {
            replaceRobot([&](Robot& robot) {
                robot.BuildFromJson(m_sim->GetWorld(), presets[m_presetIndex]);
            });
            Log(std::string("[Editor] Loaded preset ") + presets[m_presetIndex]);
        }
    }

    const char* controllers[] = { "Keyboard", "Script", "None" };
    if (ImGui::Combo("Controller", &m_controllerIndex, controllers, 3)) {
        if (m_controllerIndex == 0) {
            auto controller = std::make_shared<KeyboardController>(m_sim->GetRenderer());
            m_sim->GetRenderer().SetKeyCallback([controller](int key, bool pressed) {
                controller->OnKeyEvent(key, pressed);
            });
            m_sim->SetController(controller);
            Log("[Editor] Keyboard controller attached");
        } else if (m_controllerIndex == 1) {
            auto script = std::make_shared<ScriptController>();
            script->AddCommand(4.0, 4.0, 2.0);
            script->AddCommand(2.0, -2.0, 1.0);
            script->AddCommand(4.0, 4.0, 2.0);
            m_sim->SetController(script);
            Log("[Editor] Script controller attached");
        } else {
            m_sim->SetController(nullptr);
            Log("[Editor] Controller cleared");
        }
    }

    ImGui::Text("Chassis");
    ImGui::DragFloat3("Chassis Size", chassisSize, 0.01f, 0.01f, 5.0f);
    ImGui::DragFloat("Chassis Mass (kg)", &chassisMass, 0.1f, 0.1f, 100.0f);

    ImGui::Separator();
    ImGui::Text("Wheels / Casters");
    ImGui::SliderInt("Wheel Count", &wheelCount, 0, 6);
    ImGui::SliderInt("Caster Count", &casterCount, 0, 2);
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
        config.casterRadius = (casterCount > 0) ? 0.05 : 0.0;
        config.casters.clear();
        config.wheels.clear();

        for (int i = 0; i < wheelCount; ++i) {
            WheelSpec w;
            w.name = "wheel_" + std::to_string(i);
            w.radius = wheelRadius;
            w.width = wheelWidth;
            w.mass = 0.5;
            w.motor = true;
            if (wheelCount == 2) {
                w.offsetZ = (i == 0) ? wheelOffsetZ : -wheelOffsetZ;
            } else {
                int pair = i / 2;
                bool left = (i % 2) == 0;
                w.offsetX = (pair == 0) ? 0.18 : -0.18;
                if (wheelCount > 4 && pair == 2) w.offsetX = 0.0;
                w.offsetZ = left ? wheelOffsetZ : -wheelOffsetZ;
            }
            w.offsetY = -0.05;
            config.wheels.push_back(w);
        }
        for (int i = 0; i < casterCount; ++i) {
            CasterSpec c;
            c.offsetX = (i == 0) ? -0.2 : 0.2;
            c.offsetY = -0.1;
            config.casters.push_back(c);
        }

        auto spawn = m_sim->GetSceneManager().GetSpawnPoint();
        config.spawnX = spawn.x;
        config.spawnY = spawn.y;
        config.spawnZ = spawn.z;

        replaceRobot([&](Robot& robot) {
            robot.Build(m_sim->GetWorld(), config);
        });

        Log("[Editor] Built new robot");
    }

    ImGui::SameLine();
    if (ImGui::Button("Load from JSON...")) {
        ImGui::OpenPopup("LoadRobotPopup");
    }

    ImGui::SameLine();
    if (ImGui::Button("Save Robot JSON")) {
        if (m_sim->GetRobot()) {
            std::ofstream out(m_saveRobotPathBuf);
            if (out) {
                out << m_sim->GetRobot()->ToJson().dump(2);
                Log(std::string("[Editor] Saved robot to ") + m_saveRobotPathBuf);
            }
        }
    }
    ImGui::InputText("Save path", m_saveRobotPathBuf, sizeof(m_saveRobotPathBuf));

    if (ImGui::BeginPopupModal("LoadRobotPopup", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::Text("Robot JSON path:");
        ImGui::InputText("##robotpath", m_robotPathBuf, sizeof(m_robotPathBuf));
        if (ImGui::Button("Load")) {
            replaceRobot([&](Robot& robot) {
                robot.BuildFromJson(m_sim->GetWorld(), m_robotPathBuf);
            });
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

    ImGui::Text("Click this 3D window, then hold W to drive (yellow nose = forward)");
    ImGui::Text("W/S or Up/Down = forward/back    A/D or Left/Right = turn");
    ImGui::Text("Camera: left-drag orbit, wheel zoom, right-drag pan");
    ImGui::Text("Assemble: pick a part, then click a mount in the builder or 3D view");
    ImGui::Text("FPS %.0f   t=%.1fs", m_fps, m_sim->GetSimTime());
    ImGui::End();
}

void EditorUI::SpawnRobotPreset(const char* fileName) {
    std::string path = fileName;
#ifdef SIM_ROBOTS_DIR
    path = std::string(SIM_ROBOTS_DIR) + "/" + fileName;
#endif
    if (m_sim->GetRobot()) {
        m_sim->GetRobot()->RemoveFromWorld(m_sim->GetWorld());
    }
    auto robot = std::make_shared<Robot>();
    robot->BuildFromJson(m_sim->GetWorld(), path);
    m_sim->SetRobot(robot);
    m_sim->GetRenderer().BindAll();
    Log(std::string("[Editor] Spawned ") + path);
}

void EditorUI::HandleAssemblerMouse(int x, int y, bool leftDown, bool rightDown, float wheel) {
    (void)rightDown;
    (void)wheel;
    if (leftDown && m_state.showRobotBuilder && !m_selectedPartId.empty()) {
        m_pendingClickX = x;
        m_pendingClickY = y;
    }
}

void EditorUI::RebuildFromAssembly() {
    if (!m_sim) return;
    auto& catalog = PartCatalog::Instance();
    if (!catalog.IsLoaded()) catalog.Load();

    auto j = PartCatalog::AssemblyToRobotJson(m_assembly, catalog, "assembled");
#ifdef SIM_ROBOTS_DIR
    const std::string tmp = std::string(SIM_ROBOTS_DIR) + "/_assembled_preview.json";
#else
    const std::string tmp = "robots/_assembled_preview.json";
#endif
    {
        std::ofstream out(tmp);
        if (out) out << j.dump(2);
    }
    if (m_sim->GetRobot()) {
        m_sim->GetRobot()->RemoveFromWorld(m_sim->GetWorld());
    }
    auto robot = std::make_shared<Robot>();
    robot->BuildFromJson(m_sim->GetWorld(), tmp);
    m_sim->SetRobot(robot);
    m_sim->GetRenderer().BindAll();
}

void EditorUI::SnapSelectedPart(const std::string& parentId, const std::string& mountId) {
    if (m_selectedPartId.empty()) {
        Log("[Editor] Select a catalog part first");
        return;
    }
    auto& catalog = PartCatalog::Instance();
    const CatalogPart* part = catalog.Find(m_selectedPartId);
    if (!part) return;

    AssemblyItem item;
    item.partId = part->id;
    item.instanceId = part->id + "_" + std::to_string(m_instanceSerial++);
    item.parentId = parentId;
    item.mountId = mountId;
    item.motor = (part->category == "wheel" || part->category == "arm") && part->joint != "fixed";

    if (part->category == "chassis") {
        item.parentId.clear();
        item.mountId.clear();
        if (m_assembly.empty()) {
            m_assembly.push_back(item);
        } else {
            m_assembly.insert(m_assembly.begin(), item);
        }
    } else {
        if (m_assembly.empty()) {
            Log("[Editor] Place a chassis first");
            return;
        }
        m_assembly.push_back(item);
    }
    m_selectedInstanceId = item.instanceId;
    RebuildFromAssembly();
    Log("[Editor] Snapped " + part->name + " to " + (mountId.empty() ? "origin" : mountId));
}

bool EditorUI::TryPickMount(int mx, int my) {
    if (m_assembly.empty() || !m_sim || !m_sim->GetRobot() || !m_sim->GetRobot()->GetChassis()) {
        return false;
    }
    auto& catalog = PartCatalog::Instance();
    const CatalogPart* chassisPart = nullptr;
    std::string chassisInstance;
    for (const auto& item : m_assembly) {
        const CatalogPart* p = catalog.Find(item.partId);
        if (p && p->category == "chassis") {
            chassisPart = p;
            chassisInstance = item.instanceId;
            break;
        }
    }
    if (!chassisPart) return false;

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

    double cx, cy, cz;
    m_sim->GetRobot()->GetChassis()->GetPosition(cx, cy, cz);

    const MountFrame* best = nullptr;
    double bestDist = 0.12;
    for (const auto& mount : chassisPart->mounts) {
        double px = cx + mount.x;
        double py = cy + mount.y;
        double pz = cz + mount.z;
        double vx = px - ex, vy = py - ey, vz = pz - ez;
        double t = vx * dx + vy * dy + vz * dz;
        if (t < 0) continue;
        double qx = ex + dx * t - px;
        double qy = ey + dy * t - py;
        double qz = ez + dz * t - pz;
        double dist = std::sqrt(qx * qx + qy * qy + qz * qz);
        if (dist < bestDist) {
            bestDist = dist;
            best = &mount;
        }
    }
    if (!best) return false;
    SnapSelectedPart(chassisInstance, best->id);
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

    ImGui::TextUnformatted("1. Click a part   2. Click a mount to snap");
    ImGui::Separator();

    const char* categories[] = {"chassis", "wheel", "caster", "hub", "sensor", "arm"};
    ImGui::BeginChild("palette", ImVec2(0, 180), true);
    for (const char* cat : categories) {
        auto parts = catalog.ByCategory(cat);
        if (parts.empty()) continue;
        if (ImGui::TreeNodeEx(cat, ImGuiTreeNodeFlags_DefaultOpen)) {
            for (const auto* part : parts) {
                bool selected = (m_selectedPartId == part->id);
                if (ImGui::Selectable(part->name.c_str(), selected)) {
                    m_selectedPartId = part->id;
                    if (part->category == "chassis") {
                        SnapSelectedPart({}, {});
                    }
                }
            }
            ImGui::TreePop();
        }
    }
    ImGui::EndChild();

    if (!m_selectedPartId.empty()) {
        ImGui::Text("Selected: %s", m_selectedPartId.c_str());
    } else {
        ImGui::TextDisabled("No part selected");
    }

    ImGui::Separator();
    ImGui::TextUnformatted("Mount frames");
    const CatalogPart* chassisPart = nullptr;
    std::string chassisInstance;
    for (const auto& item : m_assembly) {
        const CatalogPart* p = catalog.Find(item.partId);
        if (p && p->category == "chassis") {
            chassisPart = p;
            chassisInstance = item.instanceId;
            break;
        }
    }
    if (chassisPart) {
        for (const auto& mount : chassisPart->mounts) {
            std::string label = mount.id + "  (" + mount.role + ")";
            if (ImGui::Button(label.c_str())) {
                SnapSelectedPart(chassisInstance, mount.id);
            }
        }
        const CatalogPart* selected = catalog.Find(m_selectedInstanceId);
        if (!selected) {
            for (auto it = m_assembly.rbegin(); it != m_assembly.rend(); ++it) {
                const CatalogPart* p = catalog.Find(it->partId);
                if (p && !p->mounts.empty() && p->category == "arm") {
                    ImGui::Separator();
                    ImGui::Text("On %s", it->instanceId.c_str());
                    for (const auto& mount : p->mounts) {
                        if (ImGui::Button((mount.id + "##" + it->instanceId).c_str())) {
                            SnapSelectedPart(it->instanceId, mount.id);
                        }
                    }
                    break;
                }
            }
        }
    } else {
        ImGui::TextDisabled("Place a chassis to expose mounts.");
    }

    ImGui::Separator();
    ImGui::Text("Assembly (%zu parts)", m_assembly.size());
    ImGui::BeginChild("assembly", ImVec2(0, 120), true);
    for (size_t i = 0; i < m_assembly.size(); ++i) {
        const auto& item = m_assembly[i];
        std::string line = item.instanceId;
        if (!item.mountId.empty()) line += " @ " + item.mountId;
        if (ImGui::Selectable(line.c_str(), m_selectedInstanceId == item.instanceId)) {
            m_selectedInstanceId = item.instanceId;
            m_selectedPartId = item.partId;
        }
    }
    ImGui::EndChild();

    if (ImGui::Button("Remove last") && !m_assembly.empty()) {
        m_assembly.pop_back();
        RebuildFromAssembly();
    }
    ImGui::SameLine();
    if (ImGui::Button("Clear assembly")) {
        m_assembly.clear();
        if (m_sim->GetRobot()) {
            m_sim->GetRobot()->RemoveFromWorld(m_sim->GetWorld());
            m_sim->SetRobot(nullptr);
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("Rebuild")) {
        RebuildFromAssembly();
    }

    ImGui::Separator();
    if (ImGui::Button("Save assembled JSON")) {
        auto j = PartCatalog::AssemblyToRobotJson(m_assembly, catalog, "assembled");
        std::ofstream out(m_saveRobotPathBuf);
        if (out) {
            out << j.dump(2);
            Log(std::string("[Editor] Saved assembly to ") + m_saveRobotPathBuf);
        }
    }
}

}  // namespace robosim

