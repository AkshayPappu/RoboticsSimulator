#include "core/SceneManager.h"
#include "core/PhysicsWorld.h"
#include "physics/RigidBody.h"
#include "environment/SceneLoader.h"
#include "environment/Terrain.h"

#include <iostream>

namespace robosim {

SceneManager::SceneManager(PhysicsWorld& world)
    : m_world(world)
{
}

SceneManager::~SceneManager() = default;

void SceneManager::LoadScene(const std::string& path) {
    ClearScene();

    SceneLoader loader;
    auto sceneData = loader.Load(path);

    m_sceneName = sceneData.name;

    // Apply scene gravity
    m_world.SetGravity(
        sceneData.gravity[0],
        sceneData.gravity[1],
        sceneData.gravity[2]
    );

    // Add ground via Terrain classes
    if (sceneData.hasGround) {
        TerrainParams params;
        params.friction = sceneData.groundFriction;
        params.restitution = sceneData.groundRestitution;

        std::unique_ptr<Terrain> terrain;
        if (sceneData.groundType == "heightmap") {
            params.sizeX = 40.0;
            params.sizeZ = 40.0;
            terrain = std::make_unique<HeightmapTerrain>(
                sceneData.heightmapPath, sceneData.heightScale, params);
        } else {
            terrain = std::make_unique<FlatTerrain>(params);
        }
        terrain->Create(m_world);
        if (terrain->GetBody()) {
            m_objects.push_back(terrain->GetBody());
        }
    }

    // Add scene objects
    for (const auto& obj : sceneData.objects) {
        AddObject(obj);
    }

    // Set spawn point
    m_spawnPoint.x = sceneData.robotSpawn[0];
    m_spawnPoint.y = sceneData.robotSpawn[1];
    m_spawnPoint.z = sceneData.robotSpawn[2];

    std::cout << "[SceneManager] Loaded scene: " << m_sceneName
              << " (" << m_objects.size() << " objects)" << std::endl;
}

void SceneManager::ClearScene() {
    for (auto& obj : m_objects) {
        m_world.RemoveBody(obj);
    }
    m_objects.clear();
    m_sceneName.clear();
}

void SceneManager::AddObject(std::shared_ptr<RigidBody> body) {
    m_world.AddBody(body);
    m_objects.push_back(body);
}

void SceneManager::RemoveObject(std::shared_ptr<RigidBody> body) {
    m_world.RemoveBody(body);
    m_objects.erase(
        std::remove(m_objects.begin(), m_objects.end(), body),
        m_objects.end()
    );
}

}  // namespace robosim
