#pragma once

#include <memory>
#include <vector>
#include <string>
#include <functional>

namespace robosim {

class PhysicsWorld;
class Renderer;
class Robot;
class Controller;
class SceneManager;
class EditorUI;
class Telemetry;

enum class SimulationState {
    Stopped,
    Running,
    Paused,
    Stepping
};

struct SimulationConfig {
    double physicsTimestep = 0.001;  // 1ms
    double renderFps = 60.0;
    bool headless = false;
    std::string scenePath;
    std::string robotPath;
    int maxSteps = 0;  // 0 = unlimited
    std::string outputPath;
};

class Simulation {
public:
    Simulation();
    ~Simulation();

    Simulation(const Simulation&) = delete;
    Simulation& operator=(const Simulation&) = delete;

    void Init(const SimulationConfig& config = {});
    void Run();
    void Stop();

    void Pause();
    void Resume();
    void StepOnce();

    void SetSpeedMultiplier(double multiplier) { m_speedMultiplier = multiplier; }
    double GetSpeedMultiplier() const { return m_speedMultiplier; }

    SimulationState GetState() const { return m_state; }
    double GetSimTime() const;
    int GetStepCount() const { return m_stepCount; }

    PhysicsWorld& GetWorld() { return *m_world; }
    const PhysicsWorld& GetWorld() const { return *m_world; }

    Renderer& GetRenderer() { return *m_renderer; }
    SceneManager& GetSceneManager() { return *m_sceneManager; }
    EditorUI* GetEditorUI() { return m_editorUI.get(); }
    Telemetry* GetTelemetry() { return m_telemetry.get(); }

    void SetRobot(std::shared_ptr<Robot> robot);
    Robot* GetRobot() { return m_robot.get(); }

    void SetController(std::shared_ptr<Controller> controller);
    Controller* GetController() { return m_controller.get(); }

    // For RL / scripted use: step once and return
    void StepPhysics();

    using FrameCallback = std::function<void(double dt)>;
    void SetPreStepCallback(FrameCallback cb) { m_preStepCallback = cb; }
    void SetPostStepCallback(FrameCallback cb) { m_postStepCallback = cb; }

private:
    std::unique_ptr<PhysicsWorld> m_world;
    std::unique_ptr<Renderer> m_renderer;
    std::unique_ptr<SceneManager> m_sceneManager;
    std::unique_ptr<EditorUI> m_editorUI;
    std::unique_ptr<Telemetry> m_telemetry;

    std::shared_ptr<Robot> m_robot;
    std::shared_ptr<Controller> m_controller;

    SimulationState m_state = SimulationState::Stopped;
    SimulationConfig m_config;
    double m_speedMultiplier = 1.0;
    int m_stepCount = 0;

    FrameCallback m_preStepCallback;
    FrameCallback m_postStepCallback;
};

}  // namespace robosim
