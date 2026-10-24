#include "core/Simulation.h"
#include "core/PhysicsWorld.h"
#include "core/SceneManager.h"
#include "physics/RigidBody.h"
#include "robotics/Robot.h"
#include "robotics/Controller.h"
#include "rendering/Renderer.h"

#include <iostream>
#include <string>
#include <cstdlib>
#include <filesystem>
#include <exception>
#include <stdexcept>

using namespace robosim;

struct AppConfig {
    bool headless = false;
    std::string scenePath;
    std::string robotPath;
    int maxSteps = 0;
    std::string outputPath;
};

AppConfig ParseArgs(int argc, char* argv[]) {
    AppConfig config;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--headless") {
            config.headless = true;
        } else if (arg == "--scene" && i + 1 < argc) {
            config.scenePath = argv[++i];
        } else if (arg == "--robot" && i + 1 < argc) {
            config.robotPath = argv[++i];
#ifdef SIM_ROBOTS_DIR
            if (!std::filesystem::exists(config.robotPath)) {
                auto fromRobots = std::filesystem::path(SIM_ROBOTS_DIR) / config.robotPath;
                if (std::filesystem::exists(fromRobots)) {
                    config.robotPath = fromRobots.string();
                } else {
                    fromRobots = std::filesystem::path(SIM_ROBOTS_DIR) / std::filesystem::path(config.robotPath).filename();
                    if (std::filesystem::exists(fromRobots)) {
                        config.robotPath = fromRobots.string();
                    }
                }
            }
#endif
        } else if (arg == "--steps" && i + 1 < argc) {
            config.maxSteps = std::atoi(argv[++i]);
        } else if (arg == "--output" && i + 1 < argc) {
            config.outputPath = argv[++i];
        } else if (arg == "--help") {
            std::cout << "Robotics Simulator\n"
                      << "Usage: RoboticsSimulator [options]\n"
                      << "  --headless       Run without visualization\n"
                      << "  --scene <path>   Load a scene JSON file\n"
                      << "  --robot <path>   Load a robot JSON file\n"
                      << "  --steps <n>      Run for n physics steps then exit\n"
                      << "  --output <path>  Write telemetry output\n"
                      << "  --help           Show this help\n";
            std::exit(0);
        }
    }
    return config;
}

void RunPhysicsDemo(Simulation& sim) {
    auto& world = sim.GetWorld();

    // Create ground plane
    Material groundMat;
    groundMat.friction = 0.6;
    groundMat.restitution = 0.2;
    auto ground = RigidBody::CreateGround(groundMat);
    world.AddBody(ground);

    // Drop some dynamic objects
    Material objMat;
    objMat.friction = 0.5;
    objMat.restitution = 0.3;

    // Falling boxes
    for (int i = 0; i < 5; ++i) {
        BodyTransform t;
        t.px = -2.0 + i * 1.0;
        t.py = 3.0 + i * 0.5;
        t.pz = 0.0;
        auto box = RigidBody::CreateBox(0.4, 0.4, 0.4, 1.0, t, objMat);
        box->SetName("box_" + std::to_string(i));
        world.AddBody(box);
    }

    // Falling spheres
    for (int i = 0; i < 5; ++i) {
        BodyTransform t;
        t.px = -2.0 + i * 1.0;
        t.py = 5.0 + i * 0.3;
        t.pz = 2.0;
        auto sphere = RigidBody::CreateSphere(0.25, 0.8, t, objMat);
        sphere->SetName("sphere_" + std::to_string(i));
        world.AddBody(sphere);
    }

    std::cout << "[Demo] Created ground + 10 falling objects" << std::endl;
}

void RunRobotDemo(Simulation& sim) {
    auto& world = sim.GetWorld();

    // Create ground
    Material groundMat;
    groundMat.friction = 1.0;
    auto ground = RigidBody::CreateGround(groundMat);
    world.AddBody(ground);

    // Add some obstacle boxes
    Material obsMat;
    obsMat.friction = 0.4;

    BodyTransform t1;
    t1.px = 3.0; t1.py = 0.5; t1.pz = 0.0;
    auto obs1 = RigidBody::CreateBox(1.0, 1.0, 1.0, 0.0, t1, obsMat);
    obs1->SetName("obstacle_1");
    obs1->SetColor(0.85f, 0.28f, 0.22f);
    obs1->AddVisualMesh("parts/obstacle_crate.obj", 1.0);
    world.AddBody(obs1);

    BodyTransform t2;
    t2.px = -2.0; t2.py = 0.25; t2.pz = 3.0;
    auto obs2 = RigidBody::CreateBox(2.0, 0.5, 0.5, 0.0, t2, obsMat);
    obs2->SetName("obstacle_2");
    obs2->SetColor(0.90f, 0.55f, 0.18f);
    world.AddBody(obs2);

    BodyTransform t3;
    t3.px = 0.0; t3.py = 0.3; t3.pz = -4.0;
    auto obs3 = RigidBody::CreateCylinder(0.5, 0.6, 0.0, t3, obsMat);
    obs3->SetName("obstacle_3");
    obs3->SetColor(0.72f, 0.22f, 0.45f);
    obs3->AddVisualMesh("parts/obstacle_barrel.obj", 1.0);
    world.AddBody(obs3);

    // Build robot
    auto robot = std::make_shared<Robot>();
#ifdef SIM_ROBOTS_DIR
    const std::string robotPath =
        (std::filesystem::path(SIM_ROBOTS_DIR) / "differential_drive.json").string();
#else
    const std::string robotPath = "robots/differential_drive.json";
#endif
    robot->BuildFromJson(world, robotPath);
    sim.SetRobot(robot);

    if (auto* renderer = sim.TryGetRenderer()) {
        auto controller = std::make_shared<KeyboardController>(*renderer);
        renderer->SetKeyCallback([controller](int key, bool pressed) {
            controller->OnKeyEvent(key, pressed);
        });
        sim.SetController(controller);
        std::cout << "[Demo] Robot demo ready — use WASD or arrow keys to drive" << std::endl;
    } else {
        std::cout << "[Demo] Robot demo ready (headless)" << std::endl;
    }
}

int main(int argc, char* argv[]) {
    std::cout << "=== AI-Powered Robotics Physics Simulator ===" << std::endl;
    std::cout << "Version 0.1.0" << std::endl;
    std::cout.flush();

    try {
    auto appConfig = ParseArgs(argc, argv);

    Simulation sim;
    SimulationConfig simConfig;
    simConfig.headless = appConfig.headless;
    simConfig.scenePath = appConfig.scenePath;
    simConfig.maxSteps = appConfig.maxSteps;
    simConfig.outputPath = appConfig.outputPath;

    sim.Init(simConfig);

    if (!appConfig.scenePath.empty()) {
        if (!appConfig.robotPath.empty()) {
            auto robot = std::make_shared<Robot>();
            robot->BuildFromJson(sim.GetWorld(), appConfig.robotPath);
            sim.SetRobot(robot);
        }
    } else {
        RunRobotDemo(sim);
        if (!appConfig.robotPath.empty()) {
            sim.ReleaseRobot();
            auto robot = std::make_shared<Robot>();
            robot->BuildFromJson(sim.GetWorld(), appConfig.robotPath);
            sim.SetRobot(robot);
        }
    }

    // Print sim info periodically
    sim.SetPostStepCallback([&sim](double dt) {
        int step = sim.GetStepCount();
        if (step % 1000 == 0 && step > 0) {
            double x, y, z;
            if (sim.GetRobot()) {
                sim.GetRobot()->GetPosition(x, y, z);
                std::cout << "[t=" << sim.GetSimTime()
                          << "] Robot at (" << x << ", " << y << ", " << z << ")"
                          << std::endl;
            } else {
                std::cout << "[t=" << sim.GetSimTime()
                          << "] Step " << step << std::endl;
            }
        }
    });

    sim.Run();

    std::cout << "=== Simulation Complete ===" << std::endl;
    return 0;
    } catch (const std::exception& e) {
        std::cerr << "[Fatal] " << e.what() << std::endl;
        std::cerr << "Press Enter to close..." << std::endl;
        std::cin.get();
        return 1;
    } catch (...) {
        std::cerr << "[Fatal] Unknown error while starting the window." << std::endl;
        std::cerr << "Press Enter to close..." << std::endl;
        std::cin.get();
        return 1;
    }
}
