#pragma once

#include <memory>
#include <string>
#include <vector>
#include <array>

namespace robosim {

class RigidBody;

struct SceneData {
    std::string name = "untitled";
    std::array<double, 3> gravity = {0, -9.81, 0};

    bool hasGround = true;
    std::string groundType = "plane";
    double groundFriction = 0.5;
    double groundRestitution = 0.1;

    std::vector<std::shared_ptr<RigidBody>> objects;

    std::array<double, 3> robotSpawn = {0, 0.5, 0};
};

class SceneLoader {
public:
    SceneLoader() = default;
    ~SceneLoader() = default;

    SceneData Load(const std::string& path);
    void Save(const std::string& path, const SceneData& data);

    static SceneData CreateDefault();

private:
    std::shared_ptr<RigidBody> ParseObject(const std::string& type,
                                            double mass,
                                            const std::vector<double>& position,
                                            const std::vector<double>& size,
                                            double friction,
                                            double slope = 0.0);
};

}  // namespace robosim
