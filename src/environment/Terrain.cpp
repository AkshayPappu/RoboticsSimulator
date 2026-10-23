#include "environment/Terrain.h"
#include "core/PhysicsWorld.h"
#include "physics/RigidBody.h"

#include <chrono/core/ChTypes.h>
#include <chrono/physics/ChBody.h>
#include <chrono/assets/ChVisualShapeTriangleMesh.h>
#include <chrono/collision/ChCollisionShapeTriangleMesh.h>
#include <chrono/geometry/ChTriangleMeshConnected.h>

#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>

namespace robosim {

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
        0.0, {}, mat
    );
    m_body->SetName("flat_terrain");
    m_body->SetPosition(0, -m_params.thickness / 2.0, 0);
    m_body->SetColor(0.45f, 0.52f, 0.38f);
    world.AddBody(m_body);

    std::cout << "[Terrain] Created flat terrain ("
              << m_params.sizeX << " x " << m_params.sizeZ << ")" << std::endl;
}

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

static double SampleHeight(int i, int j, int nx, int nz, double heightScale, const std::vector<float>& samples) {
    if (!samples.empty()) {
        int idx = j * nx + i;
        if (idx >= 0 && idx < (int)samples.size()) {
            return samples[idx] * heightScale;
        }
    }
    double u = (double)i / (double)(nx - 1);
    double v = (double)j / (double)(nz - 1);
    return heightScale * (
        0.35 * std::sin(u * 4.0 * 3.141592653589793) * std::cos(v * 3.0 * 3.141592653589793) +
        0.15 * std::sin(u * 11.0 * 3.141592653589793) +
        0.10 * std::cos(v * 9.0 * 3.141592653589793)
    );
}

void HeightmapTerrain::Create(PhysicsWorld& world) {
    auto chMat = chrono::ChContactMaterial::DefaultMaterial(chrono::ChContactMethod::NSC);
    chMat->SetFriction(static_cast<float>(m_params.friction));
    chMat->SetRestitution(static_cast<float>(m_params.restitution));

    const int nx = 48;
    const int nz = 48;
    std::vector<float> samples;

    if (!m_heightmapPath.empty()) {
        std::ifstream file(m_heightmapPath, std::ios::binary);
        if (file) {
            samples.assign(nx * nz, 0.0f);
            std::vector<unsigned char> raw(nx * nz, 0);
            file.read(reinterpret_cast<char*>(raw.data()), nx * nz);
            for (int k = 0; k < nx * nz; ++k) {
                samples[k] = raw[k] / 255.0f;
            }
            std::cout << "[Terrain] Loaded heightmap from " << m_heightmapPath << std::endl;
        } else {
            std::cout << "[Terrain] Heightmap not found, generating procedural hills" << std::endl;
        }
    } else {
        std::cout << "[Terrain] Generating procedural heightmap hills" << std::endl;
    }

    auto mesh = chrono_types::make_shared<chrono::ChTriangleMeshConnected>();
    double dx = m_params.sizeX / (nx - 1);
    double dz = m_params.sizeZ / (nz - 1);
    double originX = -m_params.sizeX * 0.5;
    double originZ = -m_params.sizeZ * 0.5;

    for (int j = 0; j < nz; ++j) {
        for (int i = 0; i < nx; ++i) {
            double x = originX + i * dx;
            double z = originZ + j * dz;
            double y = SampleHeight(i, j, nx, nz, m_heightScale, samples);
            mesh->GetCoordsVertices().push_back(chrono::ChVector3d(x, y, z));
        }
    }

    for (int j = 0; j < nz - 1; ++j) {
        for (int i = 0; i < nx - 1; ++i) {
            int a = j * nx + i;
            int b = j * nx + (i + 1);
            int c = (j + 1) * nx + i;
            int d = (j + 1) * nx + (i + 1);
            mesh->GetIndicesVertexes().push_back(chrono::ChVector3i(a, c, b));
            mesh->GetIndicesVertexes().push_back(chrono::ChVector3i(b, c, d));
        }
    }
    mesh->RepairDuplicateVertexes(1e-6);

    auto body = chrono_types::make_shared<chrono::ChBody>();
    body->SetFixed(true);
    body->SetName("heightmap_terrain");
    body->SetPos(chrono::ChVector3d(0, 0, 0));
    body->EnableCollision(true);
    body->AddCollisionShape(chrono_types::make_shared<chrono::ChCollisionShapeTriangleMesh>(
        chMat, mesh, false, false, 0.002));

    auto visShape = chrono_types::make_shared<chrono::ChVisualShapeTriangleMesh>();
    visShape->SetMesh(mesh);
    visShape->SetColor(chrono::ChColor(0.40f, 0.55f, 0.32f));
    visShape->SetMutable(false);
    body->AddVisualShape(visShape);

    m_body = RigidBody::Wrap(body, BodyShape::Custom);
    m_body->SetName("heightmap_terrain");
    world.AddBody(m_body);

    std::cout << "[Terrain] Created heightmap terrain " << nx << "x" << nz
              << " size=" << m_params.sizeX << "x" << m_params.sizeZ << std::endl;
}

}  // namespace robosim
