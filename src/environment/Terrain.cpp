#include "environment/Terrain.h"
#include "core/PhysicsWorld.h"
#include "physics/RigidBody.h"

#include <iostream>

namespace robosim {

// FlatTerrain

FlatTerrain::FlatTerrain(const TerrainParams& params) {
    m_params = params;
}

FlatTerrain::~FlatTerrain() = default;

void FlatTerrain::Create(PhysicsWorld& world) {
    Material mat;
    mat.friction = m_params.friction;
    mat.restitution = m_params.restitution;

    m_body = RigidBody::CreateBox(
        m_params.sizeX, m_params.thickness, m_params.sizeZ,
        0.0,  // mass=0 means static
        {},
        mat
    );
    m_body->SetName("flat_terrain");
    m_body->SetPosition(0, -m_params.thickness / 2.0, 0);
    world.AddBody(m_body);

    std::cout << "[Terrain] Created flat terrain ("
              << m_params.sizeX << " x " << m_params.sizeZ << ")" << std::endl;
}

// HeightmapTerrain

HeightmapTerrain::HeightmapTerrain(
    const std::string& heightmapPath,
    double heightScale,
    const TerrainParams& params)
    : m_heightmapPath(heightmapPath)
    , m_heightScale(heightScale)
{
    m_params = params;
}

HeightmapTerrain::~HeightmapTerrain() = default;

void HeightmapTerrain::Create(PhysicsWorld& world) {
    // Heightmap terrain uses Chrono mesh loading
    // For now, fall back to flat terrain as a placeholder
    std::cout << "[Terrain] Heightmap terrain loading not yet fully implemented, "
              << "using flat terrain as fallback" << std::endl;

    Material mat;
    mat.friction = m_params.friction;
    mat.restitution = m_params.restitution;

    m_body = RigidBody::CreateBox(
        m_params.sizeX, m_params.thickness, m_params.sizeZ,
        0.0, {}, mat
    );
    m_body->SetName("heightmap_terrain");
    m_body->SetPosition(0, -m_params.thickness / 2.0, 0);
    world.AddBody(m_body);
}

}  // namespace robosim
