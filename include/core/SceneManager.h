#pragma once

#include <memory>
#include <vector>
#include <string>

namespace robosim {

class PhysicsWorld;
class RigidBody;

class SceneManager {
public:
    explicit SceneManager(PhysicsWorld& world);
    ~SceneManager();

    SceneManager(const SceneManager&) = delete;
    SceneManager& operator=(const SceneManager&) = delete;

    void LoadScene(const std::string& path);
    void ClearScene();

    void AddObject(std::shared_ptr<RigidBody> body);
    void RemoveObject(std::shared_ptr<RigidBody> body);

    const std::vector<std::shared_ptr<RigidBody>>& GetObjects() const { return m_objects; }

    struct SpawnPoint {
        double x = 0, y = 0.5, z = 0;
    };
    SpawnPoint GetSpawnPoint() const { return m_spawnPoint; }

    const std::string& GetSceneName() const { return m_sceneName; }

private:
    PhysicsWorld& m_world;
    std::vector<std::shared_ptr<RigidBody>> m_objects;
    std::string m_sceneName;
    SpawnPoint m_spawnPoint;
};

}  // namespace robosim
