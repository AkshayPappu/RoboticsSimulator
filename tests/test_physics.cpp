#include <gtest/gtest.h>
#include "core/PhysicsWorld.h"
#include "physics/ChronoEngine.h"
#include "physics/RigidBody.h"
#include <cmath>

using namespace robosim;

class PhysicsTest : public ::testing::Test {
protected:
    void SetUp() override {
        world = std::make_unique<PhysicsWorld>();
        world->SetTimestep(0.001);
    }

    std::unique_ptr<PhysicsWorld> world;
};

TEST_F(PhysicsTest, GravityAcceleration) {
    // Default gravity should be (0, -9.81, 0)
    auto& engine = world->GetEngine();
    auto g = engine.GetSystem().GetGravitationalAcceleration();
    EXPECT_NEAR(g.x(), 0.0, 1e-6);
    EXPECT_NEAR(g.y(), -9.81, 1e-6);
    EXPECT_NEAR(g.z(), 0.0, 1e-6);
}

TEST_F(PhysicsTest, SetCustomGravity) {
    world->SetGravity(0, -1.625, 0);  // Moon gravity
    auto g = world->GetEngine().GetSystem().GetGravitationalAcceleration();
    EXPECT_NEAR(g.y(), -1.625, 1e-6);
}

TEST_F(PhysicsTest, FreeFallTiming) {
    // Drop a sphere from height h=5m. Expected time: t = sqrt(2h/g)
    double h = 5.0;
    double g = 9.81;
    double expectedTime = std::sqrt(2.0 * h / g);

    BodyTransform t;
    t.py = h;
    auto sphere = RigidBody::CreateSphere(0.1, 1.0, t);
    world->AddBody(sphere);

    // Also add a ground plane
    auto ground = RigidBody::CreateGround();
    world->AddBody(ground);

    // Step until the sphere is near ground level
    double maxTime = expectedTime * 2.0;
    while (world->GetTime() < maxTime) {
        world->Step();
        double px, py, pz;
        sphere->GetPosition(px, py, pz);
        if (py <= 0.2) {  // Near ground (sphere radius + ground thickness)
            break;
        }
    }

    // Check that landing time is close to theoretical
    double actualTime = world->GetTime();
    double tolerance = expectedTime * 0.15;  // 15% tolerance for contact dynamics
    EXPECT_NEAR(actualTime, expectedTime, tolerance);
}

TEST_F(PhysicsTest, ObjectResting) {
    // Drop a box and let it settle on the ground
    BodyTransform t;
    t.py = 2.0;
    Material mat;
    mat.friction = 0.5;
    mat.restitution = 0.0;

    auto box = RigidBody::CreateBox(0.5, 0.5, 0.5, 1.0, t, mat);
    world->AddBody(box);

    auto ground = RigidBody::CreateGround(mat);
    world->AddBody(ground);

    // Simulate for 3 seconds
    while (world->GetTime() < 3.0) {
        world->Step();
    }

    double px, py, pz;
    box->GetPosition(px, py, pz);

    // Box should be resting on the ground, center at ~0.25 (half-height)
    EXPECT_GT(py, -0.1);   // Not tunneled through
    EXPECT_LT(py, 1.0);    // Not floating

    // Velocity should be near zero
    double vx, vy, vz;
    box->GetLinearVelocity(vx, vy, vz);
    EXPECT_NEAR(vx, 0.0, 0.1);
    EXPECT_NEAR(vy, 0.0, 0.1);
    EXPECT_NEAR(vz, 0.0, 0.1);
}

TEST_F(PhysicsTest, NoTunneling) {
    // High-speed sphere should not tunnel through ground
    BodyTransform t;
    t.py = 10.0;
    auto sphere = RigidBody::CreateSphere(0.2, 1.0, t);
    sphere->SetLinearVelocity(0, -50.0, 0);  // Very fast downward
    world->AddBody(sphere);

    auto ground = RigidBody::CreateGround();
    world->AddBody(ground);

    // Simulate
    while (world->GetTime() < 2.0) {
        world->Step();
    }

    double px, py, pz;
    sphere->GetPosition(px, py, pz);

    // Should not have passed through the ground
    EXPECT_GT(py, -1.0);
}

TEST_F(PhysicsTest, StaticBodyStaysFixed) {
    auto ground = RigidBody::CreateGround();
    world->AddBody(ground);

    double px0, py0, pz0;
    ground->GetPosition(px0, py0, pz0);

    // Simulate
    for (int i = 0; i < 1000; ++i) {
        world->Step();
    }

    double px1, py1, pz1;
    ground->GetPosition(px1, py1, pz1);

    EXPECT_NEAR(px0, px1, 1e-10);
    EXPECT_NEAR(py0, py1, 1e-10);
    EXPECT_NEAR(pz0, pz1, 1e-10);
}

TEST_F(PhysicsTest, BodyCreationFactories) {
    auto box = RigidBody::CreateBox(1.0, 2.0, 3.0, 5.0);
    EXPECT_NE(box, nullptr);
    EXPECT_EQ(box->GetShape(), BodyShape::Box);
    EXPECT_NEAR(box->GetMass(), 5.0, 0.01);

    auto sphere = RigidBody::CreateSphere(0.5, 2.0);
    EXPECT_NE(sphere, nullptr);
    EXPECT_EQ(sphere->GetShape(), BodyShape::Sphere);

    auto cyl = RigidBody::CreateCylinder(0.3, 1.0, 3.0);
    EXPECT_NE(cyl, nullptr);
    EXPECT_EQ(cyl->GetShape(), BodyShape::Cylinder);
}

TEST_F(PhysicsTest, WorldClear) {
    auto box = RigidBody::CreateBox(1, 1, 1, 1.0);
    world->AddBody(box);
    EXPECT_EQ(world->GetBodies().size(), 1);

    world->Clear();
    EXPECT_EQ(world->GetBodies().size(), 0);
}
