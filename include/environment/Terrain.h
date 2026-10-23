#pragma once

#include <memory>
#include <string>

namespace robosim {

class PhysicsWorld;
class RigidBody;

struct TerrainParams {
    double friction = 0.5;
    double restitution = 0.1;
    double sizeX = 200.0;
    double sizeZ = 200.0;
    double thickness = 0.2;
};

class Terrain {
public:
    virtual ~Terrain() = default;

    virtual void Create(PhysicsWorld& world) = 0;
    virtual std::string GetType() const = 0;

    std::shared_ptr<RigidBody> GetBody() const { return m_body; }

protected:
    std::shared_ptr<RigidBody> m_body;
    TerrainParams m_params;
};

class FlatTerrain : public Terrain {
public:
    explicit FlatTerrain(const TerrainParams& params = {});
    ~FlatTerrain() override;

    void Create(PhysicsWorld& world) override;
    std::string GetType() const override { return "flat"; }
};

class HeightmapTerrain : public Terrain {
public:
    HeightmapTerrain(const std::string& heightmapPath,
                     double heightScale = 10.0,
                     const TerrainParams& params = {});
    ~HeightmapTerrain() override;

    void Create(PhysicsWorld& world) override;
    std::string GetType() const override { return "heightmap"; }

private:
    std::string m_heightmapPath;
    double m_heightScale;
};

}  // namespace robosim
