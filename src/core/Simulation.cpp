#include "core/Simulation.h"
#include "core/PhysicsWorld.h"
#include "core/SceneManager.h"
#include "core/Telemetry.h"
#include "rendering/Renderer.h"
#include "rendering/EditorUI.h"
#include "robotics/Robot.h"
#include "robotics/Controller.h"
#include "robotics/PartCatalog.h"

#include <chrono/core/ChDataPath.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <algorithm>
#include <cstring>
#include <exception>
#include <new>

namespace robosim {

Simulation::Simulation() = default;
Simulation::~Simulation() = default;

void Simulation::Init(const SimulationConfig& config) {
#ifdef CHRONO_DATA_DIR
    chrono::SetChronoDataPath(CHRONO_DATA_DIR);
#endif
    m_config = config;

    m_world = std::make_unique<PhysicsWorld>();
    m_world->SetTimestep(config.physicsTimestep);

    m_sceneManager = std::make_unique<SceneManager>(*m_world);
    PartCatalog::Instance().Load();

    if (!config.headless) {
        m_renderer = std::make_unique<Renderer>();
        RendererConfig rConfig;
        rConfig.windowTitle = "Robotics Simulator";
        m_renderer->Initialize(*m_world, rConfig);

        m_editorUI = std::make_unique<EditorUI>();
        m_editorUI->Initialize(*this);
    }

    // Initialize telemetry
    m_telemetry = std::make_unique<Telemetry>();
    if (!config.outputPath.empty()) {
        m_telemetry->SetOutputPath(config.outputPath);
    }
    m_telemetry->SetRecordInterval(100);
    m_telemetry->BeginRecording();

    if (!config.scenePath.empty()) {
        m_sceneManager->LoadScene(config.scenePath);
    }

    m_state = SimulationState::Stopped;
    m_stepCount = 0;

    std::cout << "[Simulation] Initialized"
              << " | timestep=" << config.physicsTimestep
              << " | headless=" << (config.headless ? "true" : "false")
              << std::endl;
}

void Simulation::Run() {
    m_state = SimulationState::Running;

    if (m_renderer) {
        m_renderer->BindAll();
    }

    auto frameStart = std::chrono::high_resolution_clock::now();
    double frameDuration = 1.0 / m_config.renderFps;

    while (m_state == SimulationState::Running || m_state == SimulationState::Paused) {
        frameStart = std::chrono::high_resolution_clock::now();

        if (m_renderer && !m_renderer->IsRunning()) {
            m_renderer->ShutdownImGui();
            break;
        }

        const char* phase = "updating the robot";
        try {
        if (m_editorUI) m_editorUI->ApplyPendingAssembly();

        if (m_state == SimulationState::Running) {
            double dt = m_config.physicsTimestep;

            if (m_preStepCallback) {
                m_preStepCallback(dt);
            }

            phase = "driving";
            if (m_controller && m_robot) {
                m_controller->Update(*m_robot, dt);
            }
            phase = "simulating";
            // Sensors are sampled from the editor/inspector, not every physics tick,
            // so a bad raycast cannot take down the whole sim loop.

            // Keep sim time close to wall-clock: 1ms steps × ~16 per 60fps frame = realtime
            double frameDt = (m_config.renderFps > 1.0) ? (1.0 / m_config.renderFps) : (1.0 / 60.0);
            int substeps = static_cast<int>(std::lround(
                (frameDt / m_config.physicsTimestep) * m_speedMultiplier));
            substeps = std::clamp(substeps, 1, 64);
            for (int i = 0; i < substeps; ++i) {
                m_world->Step();
                m_stepCount++;

                // Record telemetry
                if (m_telemetry) {
                    m_telemetry->Record(m_world->GetTime(), m_stepCount, m_robot.get());
                }

                if (m_config.maxSteps > 0 && m_stepCount >= m_config.maxSteps) {
                    m_state = SimulationState::Stopped;
                    break;
                }
            }

            if (m_postStepCallback) {
                m_postStepCallback(dt);
            }
        }

        if (m_renderer) {
            phase = "drawing";
            m_renderer->BeginFrame();
            m_renderer->Render();
            if (m_editorUI) {
                m_editorUI->Render();
            }
            m_renderer->EndFrame();
        }

        // Frame rate limiting
        auto frameEnd = std::chrono::high_resolution_clock::now();
        double elapsed = std::chrono::duration<double>(frameEnd - frameStart).count();
        if (elapsed < frameDuration) {
            std::this_thread::sleep_for(
                std::chrono::duration<double>(frameDuration - elapsed));
        }

        if (m_state == SimulationState::Stepping) {
            m_state = SimulationState::Paused;
        }
        } catch (const std::exception& ex) {
            std::cerr << "[Editor] Failed while " << phase << ": " << ex.what() << std::endl;
            std::cerr << "[Editor] The robot was removed so the window can stay open." << std::endl;
            if (std::strcmp(phase, "drawing") == 0 && m_renderer) {
                try {
                    m_renderer->EndFrame();
                } catch (...) {
                }
            }
            if (m_editorUI) {
                try {
                    m_editorUI->Log(std::string("[Editor] Failed while ") + phase +
                                    ". The robot was removed so the window can stay open.");
                } catch (...) {
                }
            }
            ReleaseRobot();
            m_state = SimulationState::Paused;
        } catch (...) {
            std::cerr << "[Editor] Failed while " << phase << ". The robot was removed so the window can stay open."
                      << std::endl;
            ReleaseRobot();
            m_state = SimulationState::Paused;
        }
    }

    // End telemetry
    if (m_telemetry) {
        m_telemetry->EndRecording();
        if (!m_config.outputPath.empty()) {
            std::cout << "[Telemetry] Data written to " << m_config.outputPath << std::endl;
        }
    }

    if (m_editorUI) {
        m_editorUI->Shutdown();
    }

    std::cout << "[Simulation] Ended at t=" << m_world->GetTime()
              << " steps=" << m_stepCount << std::endl;
}

void Simulation::Stop() {
    m_state = SimulationState::Stopped;
}

void Simulation::Pause() {
    if (m_state == SimulationState::Running) {
        m_state = SimulationState::Paused;
    }
}

void Simulation::Resume() {
    if (m_state == SimulationState::Paused) {
        m_state = SimulationState::Running;
    }
}

void Simulation::StepOnce() {
    if (m_state == SimulationState::Paused) {
        double dt = m_config.physicsTimestep;
        if (m_controller && m_robot) {
            m_controller->Update(*m_robot, dt);
        }
        try {
            m_world->Step();
        } catch (const std::exception&) {
            ReleaseRobot();
            m_state = SimulationState::Paused;
            return;
        }
        if (m_robot) m_robot->UpdateSensors(*m_world);
        m_stepCount++;
        m_state = SimulationState::Stepping;
    }
}

double Simulation::GetSimTime() const {
    return m_world ? m_world->GetTime() : 0.0;
}

void Simulation::ReleaseRobot() {
    if (!m_robot) return;
    try {
        if (m_renderer) {
            for (const auto& body : m_robot->GetChBodies()) {
                m_renderer->UnbindItem(body);
            }
        }
        m_robot->RemoveFromWorld(*m_world);
    } catch (const std::exception&) {
    }
    SetRobot(nullptr);
}

void Simulation::SetRobot(std::shared_ptr<Robot> robot) {
    if (m_robot && m_robot != robot) {
        m_robots.erase(std::remove(m_robots.begin(), m_robots.end(), m_robot), m_robots.end());
    }
    m_robot = robot;
    if (robot && std::find(m_robots.begin(), m_robots.end(), robot) == m_robots.end()) {
        m_robots.push_back(robot);
    }
}

void Simulation::AddRobot(std::shared_ptr<Robot> robot) {
    if (!robot) return;
    if (std::find(m_robots.begin(), m_robots.end(), robot) == m_robots.end()) {
        m_robots.push_back(robot);
    }
    if (!m_robot) m_robot = robot;
}

void Simulation::SetController(std::shared_ptr<Controller> controller) {
    m_controller = controller;
}

void Simulation::StepPhysics() {
    double dt = m_config.physicsTimestep;
    if (m_controller && m_robot) {
        m_controller->Update(*m_robot, dt);
    }
    m_world->Step();
    if (m_robot) m_robot->UpdateSensors(*m_world);
    m_stepCount++;
}

}  // namespace robosim
