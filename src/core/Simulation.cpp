#include "core/Simulation.h"
#include "core/PhysicsWorld.h"
#include "core/SceneManager.h"
#include "core/Telemetry.h"
#include "rendering/Renderer.h"
#include "rendering/EditorUI.h"
#include "robotics/Robot.h"
#include "robotics/Controller.h"

#include <iostream>
#include <chrono>
#include <thread>

namespace robosim {

Simulation::Simulation() = default;
Simulation::~Simulation() = default;

void Simulation::Init(const SimulationConfig& config) {
    m_config = config;

    m_world = std::make_unique<PhysicsWorld>();
    m_world->SetTimestep(config.physicsTimestep);

    m_sceneManager = std::make_unique<SceneManager>(*m_world);

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
            break;
        }

        if (m_state == SimulationState::Running) {
            double dt = m_config.physicsTimestep;

            if (m_preStepCallback) {
                m_preStepCallback(dt);
            }

            if (m_controller && m_robot) {
                m_controller->Update(*m_robot, dt);
            }

            int substeps = static_cast<int>(m_speedMultiplier);
            if (substeps < 1) substeps = 1;
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

        // Render
        if (m_renderer) {
            m_renderer->BeginFrame();
            m_renderer->Render();

            // Render editor UI (ImGui) on top of the 3D scene
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
        m_world->Step();
        m_stepCount++;
        m_state = SimulationState::Stepping;
    }
}

double Simulation::GetSimTime() const {
    return m_world ? m_world->GetTime() : 0.0;
}

void Simulation::SetRobot(std::shared_ptr<Robot> robot) {
    m_robot = robot;
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
    m_stepCount++;
}

}  // namespace robosim
