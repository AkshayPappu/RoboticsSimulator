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
    groundMat.friction = 0.6;
    auto ground = RigidBody::CreateGround(groundMat);
    world.AddBody(ground);

    // Add some obstacle boxes
    Material obsMat;
    obsMat.friction = 0.4;

    BodyTransform t1;
    t1.px = 3.0; t1.py = 0.5; t1.pz = 0.0;
    auto obs1 = RigidBody::CreateBox(1.0, 1.0, 1.0, 0.0, t1, obsMat);
    obs1->SetName("obstacle_1");
    world.AddBody(obs1);

    BodyTransform t2;
    t2.px = -2.0; t2.py = 0.25; t2.pz = 3.0;
    auto obs2 = RigidBody::CreateBox(2.0, 0.5, 0.5, 0.0, t2, obsMat);
    obs2->SetName("obstacle_2");
    world.AddBody(obs2);

    BodyTransform t3;
    t3.px = 0.0; t3.py = 0.3; t3.pz = -4.0;
    auto obs3 = RigidBody::CreateCylinder(0.5, 0.6, 0.0, t3, obsMat);
    obs3->SetName("obstacle_3");
    world.AddBody(obs3);

    // Build robot
    auto robot = std::make_shared<Robot>();
    RobotConfig rConfig;
    rConfig.spawnX = 0.0;
    rConfig.spawnY = 0.5;
    rConfig.spawnZ = 0.0;
    robot->Build(world, rConfig);
    sim.SetRobot(robot);

    // Attach keyboard controller
    auto controller = std::make_shared<KeyboardController>(sim.GetRenderer());

    // Register key handler with the renderer
    sim.GetRenderer().SetKeyCallback([&controller](int key, bool pressed) {
        controller->OnKeyEvent(key, pressed);
    });

    sim.SetController(controller);

    std::cout << "[Demo] Robot demo ready — use arrow keys to drive" << std::endl;
}

int main(int argc, char* argv[]) {
    std::cout << "=== AI-Powered Robotics Physics Simulator ===" << std::endl;
    std::cout << "Version 0.1.0" << std::endl;

    auto appConfig = ParseArgs(argc, argv);

    Simulation sim;
    SimulationConfig simConfig;
    simConfig.headless = appConfig.headless;
    simConfig.scenePath = appConfig.scenePath;
    simConfig.maxSteps = appConfig.maxSteps;

    sim.Init(simConfig);

    if (!appConfig.scenePath.empty()) {
        // Scene loaded from JSON by Init
    } else {
        // Default: run robot demo with falling objects
        RunRobotDemo(sim);
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
}
