#include <gtest/gtest.h>
#include "core/PhysicsWorld.h"
#include "physics/RigidBody.h"
#include "robotics/Robot.h"
#include "robotics/Joint.h"
#include "robotics/Motor.h"
#include <cmath>

using namespace robosim;

class RobotTest : public ::testing::Test {
protected:
    void SetUp() override {
        world = std::make_unique<PhysicsWorld>();
        world->SetTimestep(0.001);

        auto ground = RigidBody::CreateGround();
        world->AddBody(ground);
    }

    std::unique_ptr<PhysicsWorld> world;
};

TEST_F(RobotTest, BuildDefaultRobot) {
    Robot robot;
    robot.Build(*world);

    EXPECT_NE(robot.GetChassis(), nullptr);
    EXPECT_EQ(robot.GetMotors().size(), 2);

    double x, y, z;
    robot.GetPosition(x, y, z);
    EXPECT_NEAR(x, 0.0, 0.01);
    EXPECT_NEAR(y, 0.5, 0.01);
    EXPECT_NEAR(z, 0.0, 0.01);
}

TEST_F(RobotTest, BuildCustomConfig) {
    RobotConfig config;
    config.chassisMass = 10.0;
    config.wheelRadius = 0.15;
    config.maxMotorSpeed = 20.0;
    config.spawnX = 5.0;
    config.spawnY = 1.0;
    config.spawnZ = 3.0;

    Robot robot;
    robot.Build(*world, config);

    double x, y, z;
    robot.GetPosition(x, y, z);
    EXPECT_NEAR(x, 5.0, 0.01);
    EXPECT_NEAR(z, 3.0, 0.01);
}

TEST_F(RobotTest, SetWheelSpeeds) {
    Robot robot;
    robot.Build(*world);

    robot.SetWheelSpeeds(5.0, 5.0);

    // Step a few times so the motor takes effect
    for (int i = 0; i < 100; ++i) {
        world->Step();
    }

    // Wheel speeds should be near the target
    double leftSpeed = robot.GetLeftWheelSpeed();
    double rightSpeed = robot.GetRightWheelSpeed();

    EXPECT_NEAR(leftSpeed, 5.0, 2.0);
    EXPECT_NEAR(rightSpeed, 5.0, 2.0);
}

TEST_F(RobotTest, DifferentialTurning) {
    Robot robot;
    robot.Build(*world);

    double headingBefore = robot.GetHeading();

    // Turn in place: left forward, right backward
    robot.SetWheelSpeeds(3.0, -3.0);
    for (int i = 0; i < 500; ++i) {
        world->Step();
    }

    double headingAfter = robot.GetHeading();

    // Heading should have changed
    EXPECT_NE(headingBefore, headingAfter);
}

TEST_F(RobotTest, RobotNotFlippedInitially) {
    Robot robot;
    robot.Build(*world);

    EXPECT_FALSE(robot.IsFlipped());
}

TEST_F(RobotTest, ResetRobot) {
    Robot robot;
    robot.Build(*world);

    // Move the robot via physics
    robot.SetWheelSpeeds(5.0, 5.0);
    for (int i = 0; i < 1000; ++i) {
        world->Step();
    }

    double x1, y1, z1;
    robot.GetPosition(x1, y1, z1);

    // Reset to origin
    robot.Reset(0, 0.5, 0);

    double x2, y2, z2;
    robot.GetPosition(x2, y2, z2);
    EXPECT_NEAR(x2, 0.0, 0.01);
    EXPECT_NEAR(y2, 0.5, 0.01);
    EXPECT_NEAR(z2, 0.0, 0.01);
}

TEST_F(RobotTest, MotorSpeedClamping) {
    Robot robot;
    RobotConfig config;
    config.maxMotorSpeed = 10.0;
    robot.Build(*world, config);

    // Set speed beyond max
    robot.SetWheelSpeeds(50.0, -50.0);

    // The motor should clamp to max
    for (int i = 0; i < 100; ++i) {
        world->Step();
    }

    double leftSpeed = std::abs(robot.GetLeftWheelSpeed());
    double rightSpeed = std::abs(robot.GetRightWheelSpeed());

    EXPECT_LE(leftSpeed, config.maxMotorSpeed + 1.0);
    EXPECT_LE(rightSpeed, config.maxMotorSpeed + 1.0);
}

TEST_F(RobotTest, GetHeadingReturnsRadians) {
    Robot robot;
    robot.Build(*world);

    double heading = robot.GetHeading();
    EXPECT_GE(heading, -M_PI);
    EXPECT_LE(heading, M_PI);
}
