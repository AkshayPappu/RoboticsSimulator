#include <gtest/gtest.h>

#include "core/PhysicsWorld.h"
#include "core/Simulation.h"
#include "physics/RigidBody.h"
#include "robotics/AssemblyGraph.h"
#include "robotics/PartCatalog.h"
#include "robotics/Motor.h"
#include "robotics/Robot.h"

#include <chrono/physics/ChSystem.h>
#include <nlohmann/json.hpp>
#include <cmath>
#include <fstream>
#include <string>

using namespace robosim;

namespace {

std::string RobotFile(const char* name) {
#ifdef SIM_ROBOTS_DIR
    return std::string(SIM_ROBOTS_DIR) + "/" + name;
#else
    return std::string("robots/") + name;
#endif
}

void AddGround(PhysicsWorld& world) {
    Material groundMat;
    groundMat.friction = 1.0;
    world.AddBody(RigidBody::CreateGround(groundMat));
}

AssemblyIssue LoadText(const std::string& text, AssemblyGraph& graph) {
    AssemblyIssue issue;
    graph = AssemblyGraph::FromJson(nlohmann::json::parse(text), issue);
    AssemblyIssue more = graph.Validate();
    issue.errors.insert(issue.errors.end(), more.errors.begin(), more.errors.end());
    issue.warnings.insert(issue.warnings.end(), more.warnings.begin(), more.warnings.end());
    return issue;
}

}  // namespace

TEST(AssemblySchema, RejectsDuplicateId) {
    AssemblyGraph graph;
    auto issue = LoadText(R"({
        "schema_version": 1,
        "bodies": [
            {"id": "a", "shape": "box", "dimensions": [0.2, 0.2, 0.2], "mass": 1}
        ],
        "joints": [
            {"id": "a", "type": "fixed", "body_a": "a", "body_b": "a"}
        ]
    })", graph);
    EXPECT_FALSE(issue.errors.empty());
}

TEST(AssemblySchema, RejectsMissingBody) {
    AssemblyGraph graph;
    auto issue = LoadText(R"({
        "schema_version": 1,
        "bodies": [
            {"id": "a", "shape": "sphere", "radius": 0.1, "mass": 1}
        ],
        "joints": [
            {"id": "j", "type": "revolute", "body_a": "a", "body_b": "missing", "axis": [0, 0, 1]}
        ]
    })", graph);
    EXPECT_FALSE(issue.errors.empty());
}

TEST(AssemblySchema, RejectsBadMassAndZeroAxis) {
    AssemblyGraph graph;
    auto mass = LoadText(R"({
        "schema_version": 1,
        "bodies": [
            {"id": "a", "shape": "box", "dimensions": [0.2, 0.2, 0.2], "mass": 0}
        ]
    })", graph);
    EXPECT_FALSE(mass.errors.empty());

    auto axis = LoadText(R"({
        "schema_version": 1,
        "bodies": [
            {"id": "a", "shape": "box", "dimensions": [0.2, 0.2, 0.2], "mass": 1},
            {"id": "b", "shape": "box", "dimensions": [0.1, 0.1, 0.1], "mass": 1, "parent": "a"}
        ],
        "joints": [
            {"id": "j", "type": "revolute", "body_a": "a", "body_b": "b", "axis": [0, 0, 0]}
        ]
    })", graph);
    EXPECT_FALSE(axis.errors.empty());
}

TEST(AssemblySchema, RejectsMotorOnFixedJoint) {
    AssemblyGraph graph;
    auto issue = LoadText(R"({
        "schema_version": 1,
        "bodies": [
            {"id": "a", "shape": "box", "dimensions": [0.2, 0.2, 0.2], "mass": 1},
            {"id": "b", "shape": "box", "dimensions": [0.1, 0.1, 0.1], "mass": 1}
        ],
        "joints": [
            {"id": "weld", "type": "fixed", "body_a": "a", "body_b": "b"}
        ],
        "motors": [
            {"id": "m", "joint": "weld", "mode": "velocity"}
        ]
    })", graph);
    EXPECT_FALSE(issue.errors.empty());
}

TEST(AssemblySchema, DisconnectedPolicyError) {
    AssemblyGraph graph;
    auto issue = LoadText(R"({
        "schema_version": 1,
        "disconnected_policy": "error",
        "bodies": [
            {"id": "a", "shape": "box", "dimensions": [0.2, 0.2, 0.2], "mass": 1},
            {"id": "b", "shape": "box", "dimensions": [0.2, 0.2, 0.2], "mass": 1,
             "pose": {"position": [1, 0, 0]}}
        ]
    })", graph);
    EXPECT_FALSE(issue.errors.empty());
}

TEST(AssemblySchema, RoundTripFixtures) {
    const char* files[] = {
        "generic_four_wheel.json",
        "generic_two_link_arm.json",
        "generic_three_body.json"
    };
    for (const char* name : files) {
        std::ifstream in(RobotFile(name));
        ASSERT_TRUE(in.is_open()) << name;
        nlohmann::json original;
        in >> original;
        AssemblyIssue issue;
        AssemblyGraph graph = AssemblyGraph::FromJson(original, issue);
        EXPECT_TRUE(issue.errors.empty()) << name;
        auto again = graph.ToJson();
        AssemblyIssue issue2;
        AssemblyGraph copy = AssemblyGraph::FromJson(again, issue2);
        EXPECT_TRUE(issue2.errors.empty()) << name;
        EXPECT_EQ(copy.bodies.size(), graph.bodies.size()) << name;
        EXPECT_EQ(copy.joints.size(), graph.joints.size()) << name;
        EXPECT_EQ(copy.motors.size(), graph.motors.size()) << name;
        ASSERT_FALSE(copy.motors.empty());
        EXPECT_EQ(copy.motors.front().mode, graph.motors.front().mode);
        EXPECT_EQ(copy.joints.front().type, graph.joints.front().type);
    }
}

class AssemblySim : public ::testing::Test {
protected:
    void SetUp() override {
        world = std::make_unique<PhysicsWorld>();
        world->SetTimestep(0.001);
        AddGround(*world);
    }
    std::unique_ptr<PhysicsWorld> world;
};

TEST_F(AssemblySim, FourWheelDrives) {
    Robot robot;
    robot.BuildFromJson(*world, RobotFile("generic_four_wheel.json"));
    EXPECT_TRUE(robot.GetBuildErrors().empty());
    ASSERT_NE(robot.GetChassis(), nullptr);
    EXPECT_EQ(robot.GetMotors().size(), 4u);
    EXPECT_NE(robot.GetAssembly(), nullptr);
    EXPECT_EQ(robot.GetAssembly()->FindBody("wheel_fl")->partId, "wheel_std");

    for (int i = 0; i < 200; ++i) world->Step();
    double x0, y0, z0;
    robot.GetPosition(x0, y0, z0);
    robot.SetWheelSpeeds(8.0, 8.0);
    for (int i = 0; i < 2000; ++i) world->Step();
    double x1, y1, z1;
    robot.GetPosition(x1, y1, z1);
    double dist = std::sqrt((x1 - x0) * (x1 - x0) + (z1 - z0) * (z1 - z0));
    EXPECT_GT(dist, 0.25);
    EXPECT_GT(y1, 0.05);
    EXPECT_FALSE(robot.IsFlipped());
}

TEST_F(AssemblySim, ArmJointMoves) {
    Robot robot;
    robot.BuildFromJson(*world, RobotFile("generic_two_link_arm.json"));
    EXPECT_TRUE(robot.GetBuildErrors().empty());
    auto shoulder = robot.FindBody("shoulder");
    ASSERT_NE(shoulder, nullptr);
    double x0, y0, z0;
    shoulder->GetPosition(x0, y0, z0);
    const double before = robot.GetMotors()[0]->GetCurrentAngle();
    EXPECT_TRUE(robot.SetMotorTarget("shoulder_motor", before + 0.35));
    for (int i = 0; i < 800; ++i) world->Step();
    double x1, y1, z1;
    shoulder->GetPosition(x1, y1, z1);
    double moved = std::sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0) + (z1 - z0) * (z1 - z0));
    EXPECT_GT(moved, 0.02);
    EXPECT_NEAR(robot.GetMotors()[0]->GetCurrentAngle(), before + 0.35, 0.08);
}

TEST_F(AssemblySim, ThreeBodyHingeAndWeld) {
    Robot robot;
    robot.BuildFromJson(*world, RobotFile("generic_three_body.json"));
    EXPECT_TRUE(robot.GetBuildErrors().empty());
    auto link = robot.FindBody("link");
    auto tool = robot.FindBody("tool");
    ASSERT_NE(link, nullptr);
    ASSERT_NE(tool, nullptr);
    double lx0, ly0, lz0, tx0, ty0, tz0;
    link->GetPosition(lx0, ly0, lz0);
    tool->GetPosition(tx0, ty0, tz0);
    const double gap0 = std::sqrt((tx0 - lx0) * (tx0 - lx0) + (ty0 - ly0) * (ty0 - ly0) + (tz0 - lz0) * (tz0 - lz0));
    EXPECT_TRUE(robot.SetMotorTarget("swing", 1.5));
    for (int i = 0; i < 600; ++i) world->Step();
    double lx1, ly1, lz1, tx1, ty1, tz1;
    link->GetPosition(lx1, ly1, lz1);
    tool->GetPosition(tx1, ty1, tz1);
    double moved = std::sqrt((lx1 - lx0) * (lx1 - lx0) + (ly1 - ly0) * (ly1 - ly0) + (lz1 - lz0) * (lz1 - lz0));
    EXPECT_GT(moved, 0.02);
    const double gap1 = std::sqrt((tx1 - lx1) * (tx1 - lx1) + (ty1 - ly1) * (ty1 - ly1) + (tz1 - lz1) * (tz1 - lz1));
    EXPECT_NEAR(gap1, gap0, 0.02);
}

TEST_F(AssemblySim, TwoAssembliesStep) {
    Robot rover;
    Robot linkage;
    rover.BuildFromJson(*world, RobotFile("generic_four_wheel.json"));
    linkage.BuildFromJson(*world, RobotFile("generic_three_body.json"));
    EXPECT_TRUE(rover.GetBuildErrors().empty());
    EXPECT_TRUE(linkage.GetBuildErrors().empty());
    for (int i = 0; i < 100; ++i) world->Step();
    double x, y, z;
    rover.GetPosition(x, y, z);
    EXPECT_TRUE(std::isfinite(y));
    linkage.GetPosition(x, y, z);
    EXPECT_TRUE(std::isfinite(x));
    EXPECT_GT(std::abs(x - 0.0), 1.0);
}

TEST(AssemblySchema, SimulationKeepsBothRobots) {
    Simulation sim;
    SimulationConfig config;
    config.headless = true;
    sim.Init(config);
    AddGround(sim.GetWorld());
    auto rover = std::make_shared<Robot>();
    auto linkage = std::make_shared<Robot>();
    rover->BuildFromJson(sim.GetWorld(), RobotFile("generic_four_wheel.json"));
    linkage->BuildFromJson(sim.GetWorld(), RobotFile("generic_three_body.json"));
    sim.AddRobot(rover);
    sim.AddRobot(linkage);
    EXPECT_EQ(sim.GetRobots().size(), 2u);
    EXPECT_EQ(sim.GetRobot(), rover.get());
    for (int i = 0; i < 30; ++i) sim.StepPhysics();
    double x, y, z;
    rover->GetPosition(x, y, z);
    EXPECT_TRUE(std::isfinite(y));
}

TEST_F(AssemblySim, CatalogPartIdFillsShape) {
    ASSERT_TRUE(PartCatalog::Instance().Load());
    const char* text = R"({
        "schema_version": 1,
        "name": "tire_only",
        "bodies": [
            {"id": "tire", "part_id": "wheel_std", "pose": {"position": [0, 0.3, 0]}}
        ]
    })";
    AssemblyIssue issue;
    AssemblyGraph graph = AssemblyGraph::FromJson(nlohmann::json::parse(text), issue);
    EXPECT_TRUE(issue.errors.empty());
    ASSERT_NE(graph.FindBody("tire"), nullptr);
    EXPECT_EQ(graph.FindBody("tire")->shape, AssemblyShape::Cylinder);
    EXPECT_NEAR(graph.FindBody("tire")->radius, 0.1, 1e-6);
    EXPECT_TRUE(graph.FindBody("tire")->hasMass);

    Robot robot;
    EXPECT_TRUE(robot.BuildFromGraph(*world, graph));
    auto tire = robot.FindBody("tire");
    ASSERT_NE(tire, nullptr);
    double x, y, z;
    tire->GetPosition(x, y, z);
    EXPECT_NEAR(y, 0.3, 1e-3);
}

TEST_F(AssemblySim, CatalogWheelSocketsRestOnGround) {
    AssemblyGraph graph;
    graph.name = "four_tires";
    graph.root = "chassis";

    AssemblyBody chassis;
    chassis.id = "chassis";
    chassis.shape = AssemblyShape::Box;
    chassis.shapeSpecified = true;
    chassis.sizeX = 0.6;
    chassis.sizeY = 0.12;
    chassis.sizeZ = 0.35;
    chassis.mass = 8.0;
    chassis.hasMass = true;
    chassis.pose = MakePose(0.0, 0.07, 0.0);
    graph.bodies.push_back(chassis);

    const double mounts[4][3] = {
        {0.2, 0.0, -0.22},
        {0.2, 0.0, 0.22},
        {-0.2, 0.0, -0.22},
        {-0.2, 0.0, 0.22},
    };
    for (int i = 0; i < 4; ++i) {
        AssemblyBody wheel;
        wheel.id = "wheel_" + std::to_string(i);
        wheel.shape = AssemblyShape::Cylinder;
        wheel.shapeSpecified = true;
        wheel.radius = 0.1;
        wheel.height = 0.05;
        wheel.mass = 0.4;
        wheel.hasMass = true;
        wheel.friction = 1.0;
        wheel.parent = "chassis";
        wheel.pose = PoseAlignYToAxis(mounts[i][0], mounts[i][1], mounts[i][2], 0, 0, 1);
        graph.bodies.push_back(wheel);

        AssemblyJoint joint;
        joint.id = "axle_" + std::to_string(i);
        joint.type = AssemblyJointType::Revolute;
        joint.bodyA = "chassis";
        joint.bodyB = wheel.id;
        joint.anchorX = mounts[i][0];
        joint.anchorY = mounts[i][1];
        joint.anchorZ = mounts[i][2];
        joint.axisX = 0;
        joint.axisY = 0;
        joint.axisZ = 1;
        graph.joints.push_back(joint);

        AssemblyMotor motor;
        motor.id = "drive_" + std::to_string(i);
        motor.jointId = joint.id;
        motor.mode = AssemblyMotorMode::Velocity;
        graph.motors.push_back(motor);
    }

    EXPECT_LT(graph.FindBody("chassis")->pose.py - 0.1, 0.0);
    EXPECT_TRUE(LiftAssemblyOntoGround(graph));
    EXPECT_GT(graph.FindBody("chassis")->pose.py, 0.10);

    Robot robot;
    EXPECT_TRUE(robot.BuildFromGraph(*world, graph));
    EXPECT_EQ(robot.GetMotors().size(), 4u);
    for (int step = 0; step < 200; ++step) world->Step();
    double x, y, z;
    robot.GetPosition(x, y, z);
    EXPECT_TRUE(std::isfinite(y));
    EXPECT_GT(y, 0.05);
    for (int i = 0; i < 4; ++i) {
        auto wheel = robot.FindBody("wheel_" + std::to_string(i));
        ASSERT_NE(wheel, nullptr);
        wheel->GetPosition(x, y, z);
        EXPECT_TRUE(std::isfinite(y));
        EXPECT_GT(y, 0.05);
    }
}

static AssemblyGraph MakeCatalogWheels(int wheelCount, double chassisY) {
    AssemblyGraph graph;
    graph.name = "assembly";
    graph.root = "chassis";
    AssemblyBody chassis;
    chassis.id = "chassis";
    chassis.shape = AssemblyShape::Box;
    chassis.shapeSpecified = true;
    chassis.sizeX = 0.6;
    chassis.sizeY = 0.12;
    chassis.sizeZ = 0.35;
    chassis.mass = 8.0;
    chassis.hasMass = true;
    chassis.pose = MakePose(0.0, chassisY, 0.0);
    graph.bodies.push_back(chassis);
    const double mounts[4][3] = {
        {0.2, 0.0, -0.22}, {0.2, 0.0, 0.22}, {-0.2, 0.0, -0.22}, {-0.2, 0.0, 0.22},
    };
    for (int i = 0; i < wheelCount; ++i) {
        AssemblyBody wheel;
        wheel.id = "wheel_" + std::to_string(i);
        wheel.shape = AssemblyShape::Cylinder;
        wheel.shapeSpecified = true;
        wheel.radius = 0.1;
        wheel.height = 0.05;
        wheel.mass = 0.4;
        wheel.hasMass = true;
        wheel.friction = 1.0;
        wheel.parent = "chassis";
        wheel.pose = PoseAlignYToAxis(mounts[i][0], mounts[i][1], mounts[i][2], 0, 0, 1);
        graph.bodies.push_back(wheel);
        AssemblyJoint joint;
        joint.id = "axle_" + std::to_string(i);
        joint.type = AssemblyJointType::Revolute;
        joint.bodyA = "chassis";
        joint.bodyB = wheel.id;
        joint.anchorX = mounts[i][0];
        joint.anchorY = mounts[i][1];
        joint.anchorZ = mounts[i][2];
        joint.axisZ = 1;
        graph.joints.push_back(joint);
        AssemblyMotor motor;
        motor.id = "drive_" + std::to_string(i);
        motor.jointId = joint.id;
        graph.motors.push_back(motor);
    }
    return graph;
}

TEST_F(AssemblySim, IncrementalWheelAddsStayBounded) {
    Robot demo;
    demo.BuildFromJson(*world, RobotFile("differential_drive.json"));
    for (int i = 0; i < 50; ++i) {
        demo.SetWheelSpeeds(8.0, 8.0);
        world->Step();
    }
    demo.RemoveFromWorld(*world);
    EXPECT_EQ(world->GetChSystem().GetLinks().size(), 0u);

    Robot robot;
    for (int n = 1; n <= 4; ++n) {
        robot.RemoveFromWorld(*world);
        auto graph = MakeCatalogWheels(n, 0.07);
        ASSERT_TRUE(LiftAssemblyOntoGround(graph)) << n;
        ASSERT_TRUE(robot.BuildFromGraph(*world, graph)) << n;
        robot.SetWheelSpeeds(8.0, 8.0);
        for (int s = 0; s < 80; ++s) {
            ASSERT_NO_THROW(world->Step()) << "wheels=" << n << " step=" << s;
            unsigned contacts = world->GetChSystem().GetNumContacts();
            EXPECT_LT(contacts, 64u) << "wheels=" << n << " step=" << s;
            if (contacts >= 64u) return;
        }
    }
    robot.RemoveFromWorld(*world);
    EXPECT_EQ(world->GetChSystem().GetBodies().size(), 1u);
    EXPECT_EQ(world->GetChSystem().GetLinks().size(), 0u);
}

TEST(AssemblySchema, LegacyFilesStillDescribeRobots) {
    PhysicsWorld world;
    world.SetTimestep(0.001);
    AddGround(world);
    Robot robot;
    robot.BuildFromJson(world, RobotFile("differential_drive.json"));
    EXPECT_EQ(robot.GetMotors().size(), 2u);
    EXPECT_EQ(robot.GetAssembly(), nullptr);
    EXPECT_NEAR(robot.GetConfig().chassisSizeX, 0.5, 1e-6);
}
