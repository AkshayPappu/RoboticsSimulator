#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

#include "core/Simulation.h"
#include "core/PhysicsWorld.h"
#include "core/SceneManager.h"
#include "physics/RigidBody.h"
#include "robotics/Robot.h"
#include "robotics/Motor.h"
#include "robotics/Controller.h"
#include "robotics/Sensor.h"

namespace py = pybind11;
using namespace robosim;

// Python controller that wraps a Python callable
class PyController : public Controller {
public:
    using PyUpdateFn = std::function<void(Robot&, double)>;

    explicit PyController(PyUpdateFn fn) : m_fn(std::move(fn)) {}

    void Update(Robot& robot, double dt) override {
        if (m_fn) m_fn(robot, dt);
    }

    std::string GetName() const override { return "PyController"; }

private:
    PyUpdateFn m_fn;
};

// Gymnasium-style step result
struct StepResult {
    py::array_t<double> observation;
    double reward;
    bool terminated;
    bool truncated;
    py::dict info;
};

// Wraps Simulation for Gymnasium-compatible RL interface
class SimEnv {
public:
    SimEnv(const std::string& scenePath = "", const std::string& robotPath = "") {
        m_sim = std::make_unique<Simulation>();
        SimulationConfig config;
        config.headless = true;
        config.scenePath = scenePath;
        config.physicsTimestep = 0.001;
        m_sim->Init(config);

        m_goalX = 0.0;
        m_goalY = 0.0;
        m_goalZ = 20.0;
        m_maxSteps = 10000;

        if (robotPath.empty()) {
            m_robot = std::make_shared<Robot>();
            m_robot->Build(m_sim->GetWorld());
            m_sim->SetRobot(m_robot);
        }
    }

    void SetGoal(double x, double y, double z) {
        m_goalX = x;
        m_goalY = y;
        m_goalZ = z;
    }

    void SetMaxSteps(int maxSteps) {
        m_maxSteps = maxSteps;
    }

    py::array_t<double> Reset() {
        m_stepCount = 0;

        if (m_robot) {
            auto spawn = m_sim->GetSceneManager().GetSpawnPoint();
            m_robot->Reset(spawn.x, spawn.y, spawn.z);
        }

        return GetObservation();
    }

    StepResult Step(py::array_t<double> action) {
        auto buf = action.request();
        double* ptr = static_cast<double*>(buf.ptr);

        double leftSpeed = 0.0, rightSpeed = 0.0;
        if (buf.size >= 2) {
            leftSpeed = ptr[0];
            rightSpeed = ptr[1];
        }

        if (m_robot) {
            m_robot->SetWheelSpeeds(leftSpeed, rightSpeed);
        }

        // Step physics multiple times per RL step for stability
        int physicsStepsPerAction = 10;
        for (int i = 0; i < physicsStepsPerAction; ++i) {
            m_sim->StepPhysics();
        }
        m_stepCount++;

        auto obs = GetObservation();
        double reward = ComputeReward();
        bool terminated = IsTerminated();
        bool truncated = (m_stepCount >= m_maxSteps);

        py::dict info;
        info["step"] = m_stepCount;
        info["sim_time"] = m_sim->GetSimTime();

        if (terminated) {
            info["termination_reason"] = IsAtGoal() ? "reached_goal" : "flipped";
        }

        return {obs, reward, terminated, truncated, info};
    }

    int GetObservationSize() const { return 10; }
    int GetActionSize() const { return 2; }

    double GetSimTime() const { return m_sim->GetSimTime(); }

private:
    py::array_t<double> GetObservation() {
        std::vector<double> obs(10, 0.0);

        if (m_robot) {
            double x, y, z;
            m_robot->GetPosition(x, y, z);
            obs[0] = x;
            obs[1] = y;
            obs[2] = z;

            double qw, qx, qy, qz;
            m_robot->GetOrientation(qw, qx, qy, qz);
            obs[3] = qw;
            obs[4] = qx;
            obs[5] = qy;
            obs[6] = qz;

            obs[7] = m_robot->GetLeftWheelSpeed();
            obs[8] = m_robot->GetRightWheelSpeed();

            // Distance to goal
            double dx = m_goalX - x;
            double dz = m_goalZ - z;
            obs[9] = std::sqrt(dx * dx + dz * dz);
        }

        return py::array_t<double>(obs.size(), obs.data());
    }

    double ComputeReward() {
        if (!m_robot) return 0.0;

        double x, y, z;
        m_robot->GetPosition(x, y, z);

        double dx = m_goalX - x;
        double dz = m_goalZ - z;
        double dist = std::sqrt(dx * dx + dz * dz);

        double reward = -dist * 0.01;  // Distance penalty

        if (dist < 1.0) {
            reward += 100.0;  // Goal bonus
        }

        if (m_robot->IsFlipped()) {
            reward -= 50.0;  // Flip penalty
        }

        return reward;
    }

    bool IsAtGoal() {
        if (!m_robot) return false;
        double x, y, z;
        m_robot->GetPosition(x, y, z);
        double dx = m_goalX - x;
        double dz = m_goalZ - z;
        return std::sqrt(dx * dx + dz * dz) < 1.0;
    }

    bool IsTerminated() {
        if (!m_robot) return true;
        return m_robot->IsFlipped() || IsAtGoal();
    }

    std::unique_ptr<Simulation> m_sim;
    std::shared_ptr<Robot> m_robot;

    double m_goalX, m_goalY, m_goalZ;
    int m_maxSteps;
    int m_stepCount = 0;
};

PYBIND11_MODULE(robosim, m) {
    m.doc() = "Robotics Simulator Python bindings";

    // RobotConfig
    py::class_<RobotConfig>(m, "RobotConfig")
        .def(py::init<>())
        .def_readwrite("chassis_size_x", &RobotConfig::chassisSizeX)
        .def_readwrite("chassis_size_y", &RobotConfig::chassisSizeY)
        .def_readwrite("chassis_size_z", &RobotConfig::chassisSizeZ)
        .def_readwrite("chassis_mass", &RobotConfig::chassisMass)
        .def_readwrite("wheel_radius", &RobotConfig::wheelRadius)
        .def_readwrite("wheel_width", &RobotConfig::wheelWidth)
        .def_readwrite("max_motor_speed", &RobotConfig::maxMotorSpeed)
        .def_readwrite("max_torque", &RobotConfig::maxTorque)
        .def_readwrite("spawn_x", &RobotConfig::spawnX)
        .def_readwrite("spawn_y", &RobotConfig::spawnY)
        .def_readwrite("spawn_z", &RobotConfig::spawnZ);

    // Robot (limited exposure)
    py::class_<Robot, std::shared_ptr<Robot>>(m, "Robot")
        .def("get_position", [](const Robot& r) {
            double x, y, z;
            r.GetPosition(x, y, z);
            return py::make_tuple(x, y, z);
        })
        .def("get_heading", &Robot::GetHeading)
        .def("is_flipped", &Robot::IsFlipped)
        .def("get_left_wheel_speed", &Robot::GetLeftWheelSpeed)
        .def("get_right_wheel_speed", &Robot::GetRightWheelSpeed)
        .def("set_wheel_speeds", &Robot::SetWheelSpeeds)
        .def("get_sensor_readings", [](const Robot& r) {
            py::dict d;
            for (const auto& sensor : r.GetSensors()) {
                if (sensor->GetType() == SensorType::Distance) {
                    auto* dist = static_cast<const DistanceSensor*>(sensor.get());
                    d[sensor->GetName().c_str()] = dist->GetDistance();
                } else if (sensor->GetType() == SensorType::Contact) {
                    auto* c = static_cast<const ContactSensor*>(sensor.get());
                    d[sensor->GetName().c_str()] = c->InContact();
                }
            }
            return d;
        });

    // StepResult
    py::class_<StepResult>(m, "StepResult")
        .def_readonly("observation", &StepResult::observation)
        .def_readonly("reward", &StepResult::reward)
        .def_readonly("terminated", &StepResult::terminated)
        .def_readonly("truncated", &StepResult::truncated)
        .def_readonly("info", &StepResult::info);

    // SimEnv (Gymnasium-style)
    py::class_<SimEnv>(m, "SimEnv")
        .def(py::init<const std::string&, const std::string&>(),
             py::arg("scene_path") = "",
             py::arg("robot_path") = "")
        .def("reset", &SimEnv::Reset)
        .def("step", &SimEnv::Step)
        .def("set_goal", &SimEnv::SetGoal)
        .def("set_max_steps", &SimEnv::SetMaxSteps)
        .def("get_observation_size", &SimEnv::GetObservationSize)
        .def("get_action_size", &SimEnv::GetActionSize)
        .def("get_sim_time", &SimEnv::GetSimTime);

    // SimulationConfig
    py::class_<SimulationConfig>(m, "SimulationConfig")
        .def(py::init<>())
        .def_readwrite("physics_timestep", &SimulationConfig::physicsTimestep)
        .def_readwrite("headless", &SimulationConfig::headless)
        .def_readwrite("scene_path", &SimulationConfig::scenePath)
        .def_readwrite("max_steps", &SimulationConfig::maxSteps);

    // Simulation
    py::class_<Simulation>(m, "Simulation")
        .def(py::init<>())
        .def("init", &Simulation::Init, py::arg("config") = SimulationConfig())
        .def("step_physics", &Simulation::StepPhysics)
        .def("get_sim_time", &Simulation::GetSimTime)
        .def("get_step_count", &Simulation::GetStepCount)
        .def("get_robot", &Simulation::GetRobot, py::return_value_policy::reference);
}
