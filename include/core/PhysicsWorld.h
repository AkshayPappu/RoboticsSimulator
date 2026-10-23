#pragma once

#include <memory>
#include <vector>
#include <string>

namespace chrono {
class ChSystem;
class ChBody;
class ChLinkBase;
}

namespace robosim {

class ChronoEngine;
class RigidBody;

class PhysicsWorld {
public:
    PhysicsWorld();
    ~PhysicsWorld();

    PhysicsWorld(const PhysicsWorld&) = delete;
    PhysicsWorld& operator=(const PhysicsWorld&) = delete;

    void Step();
    void Step(double dt);

    double GetTime() const;
    double GetTimestep() const;
    void SetTimestep(double dt);

    void SetGravity(double x, double y, double z);

    void AddBody(std::shared_ptr<RigidBody> body);
    void RemoveBody(std::shared_ptr<RigidBody> body);
    void AddLink(std::shared_ptr<chrono::ChLinkBase> link);
    void RemoveLink(std::shared_ptr<chrono::ChLinkBase> link);

    const std::vector<std::shared_ptr<RigidBody>>& GetBodies() const { return m_bodies; }

    void Clear();

    ChronoEngine& GetEngine() { return *m_engine; }
    const ChronoEngine& GetEngine() const { return *m_engine; }

    chrono::ChSystem& GetChSystem();
    const chrono::ChSystem& GetChSystem() const;

private:
    std::unique_ptr<ChronoEngine> m_engine;
    std::vector<std::shared_ptr<RigidBody>> m_bodies;
};

}  // namespace robosim
