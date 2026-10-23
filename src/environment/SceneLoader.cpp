#include "environment/SceneLoader.h"
#include "physics/RigidBody.h"

#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <cmath>

using json = nlohmann::json;

namespace robosim {

SceneData SceneLoader::Load(const std::string& path) {
    SceneData data;

    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "[SceneLoader] Failed to open: " << path
                  << ", using defaults" << std::endl;
        return CreateDefault();
    }

    try {
        json j = json::parse(file);

        if (j.contains("name")) {
            data.name = j["name"].get<std::string>();
        }

        if (j.contains("gravity") && j["gravity"].is_array() && j["gravity"].size() == 3) {
            data.gravity = {
                j["gravity"][0].get<double>(),
                j["gravity"][1].get<double>(),
                j["gravity"][2].get<double>()
            };
        }

        if (j.contains("ground")) {
            auto& ground = j["ground"];
            data.hasGround = true;
            if (ground.contains("type"))
                data.groundType = ground["type"].get<std::string>();
            if (ground.contains("friction"))
                data.groundFriction = ground["friction"].get<double>();
            if (ground.contains("restitution"))
                data.groundRestitution = ground["restitution"].get<double>();
            if (ground.contains("heightmap_path"))
                data.heightmapPath = ground["heightmap_path"].get<std::string>();
            if (ground.contains("height_scale"))
                data.heightScale = ground["height_scale"].get<double>();
        }

        if (j.contains("objects") && j["objects"].is_array()) {
            for (const auto& obj : j["objects"]) {
                std::string type = obj.value("type", "box");
                double mass = obj.value("mass", 0.0);
                double friction = obj.value("friction", 0.5);
                double slope = obj.value("slope", 0.0);

                std::vector<double> position = {0, 0, 0};
                if (obj.contains("position") && obj["position"].is_array()) {
                    for (size_t i = 0; i < std::min(obj["position"].size(), (size_t)3); ++i) {
                        position[i] = obj["position"][i].get<double>();
                    }
                }

                std::vector<double> size = {1, 1, 1};
                if (obj.contains("size") && obj["size"].is_array()) {
                    for (size_t i = 0; i < std::min(obj["size"].size(), (size_t)3); ++i) {
                        size[i] = obj["size"][i].get<double>();
                    }
                }

                if (type == "sphere" && obj.contains("radius")) {
                    size = {obj["radius"].get<double>()};
                }

                if (type == "ramp" && obj.contains("length")) {
                    size = {obj["length"].get<double>(), 0.2, 2.0};
                }

                auto body = ParseObject(type, mass, position, size, friction, slope);
                if (body) {
                    std::string name = obj.value("name", type);
                    body->SetName(name);
                    if (type == "ramp") {
                        body->SetColor(0.62f, 0.48f, 0.28f);
                    } else if (mass <= 0.0) {
                        body->SetColor(0.82f, 0.32f, 0.22f);
                    } else {
                        body->SetColor(0.28f, 0.55f, 0.82f);
                    }
                    if (obj.contains("visual") && obj["visual"].is_object()) {
                        VisualSpec vis;
                        vis.mesh = obj["visual"].value("mesh", std::string());
                        vis.texture = obj["visual"].value("texture", std::string());
                        if (obj["visual"].contains("scale") && obj["visual"]["scale"].is_number()) {
                            vis.scaleX = vis.scaleY = vis.scaleZ = obj["visual"]["scale"].get<double>();
                        }
                        body->ApplyVisual(vis, true);
                    } else if (type == "box" || type == "wall") {
                        VisualSpec vis;
                        vis.mesh = "parts/obstacle_crate.obj";
                        vis.scaleX = size.size() > 0 ? size[0] : 1.0;
                        vis.scaleY = size.size() > 1 ? size[1] : 1.0;
                        vis.scaleZ = size.size() > 2 ? size[2] : 1.0;
                        body->ApplyVisual(vis, true);
                    } else if (type == "cylinder") {
                        VisualSpec vis;
                        vis.mesh = "parts/obstacle_barrel.obj";
                        body->ApplyVisual(vis, true);
                    }
                    data.objects.push_back(body);
                }
            }
        }

        if (j.contains("robot_spawn") && j["robot_spawn"].is_array() && j["robot_spawn"].size() == 3) {
            data.robotSpawn = {
                j["robot_spawn"][0].get<double>(),
                j["robot_spawn"][1].get<double>(),
                j["robot_spawn"][2].get<double>()
            };
        }

    } catch (const json::exception& e) {
        std::cerr << "[SceneLoader] JSON parse error: " << e.what() << std::endl;
        return CreateDefault();
    }

    std::cout << "[SceneLoader] Loaded scene '" << data.name
              << "' with " << data.objects.size() << " objects" << std::endl;
    return data;
}

void SceneLoader::Save(const std::string& path, const SceneData& data) {
    json j;
    j["name"] = data.name;
    j["gravity"] = data.gravity;
    j["ground"] = {
        {"type", data.groundType},
        {"friction", data.groundFriction},
        {"restitution", data.groundRestitution}
    };
    j["robot_spawn"] = data.robotSpawn;

    // Objects cannot be fully serialized without shape metadata stored on RigidBody
    // This is a basic implementation
    j["objects"] = json::array();

    std::ofstream file(path);
    if (file.is_open()) {
        file << j.dump(2);
        std::cout << "[SceneLoader] Saved scene to: " << path << std::endl;
    } else {
        std::cerr << "[SceneLoader] Failed to save: " << path << std::endl;
    }
}

SceneData SceneLoader::CreateDefault() {
    SceneData data;
    data.name = "default";
    return data;
}

std::shared_ptr<RigidBody> SceneLoader::ParseObject(
    const std::string& type,
    double mass,
    const std::vector<double>& position,
    const std::vector<double>& size,
    double friction,
    double slope)
{
    Material mat;
    mat.friction = friction;

    BodyTransform transform;
    if (position.size() >= 3) {
        transform.px = position[0];
        transform.py = position[1];
        transform.pz = position[2];
    }

    if (type == "box" || type == "wall") {
        double sx = size.size() > 0 ? size[0] : 1.0;
        double sy = size.size() > 1 ? size[1] : 1.0;
        double sz = size.size() > 2 ? size[2] : 1.0;
        return RigidBody::CreateBox(sx, sy, sz, mass, transform, mat);
    }
    else if (type == "sphere") {
        double radius = size.size() > 0 ? size[0] : 0.5;
        return RigidBody::CreateSphere(radius, mass, transform, mat);
    }
    else if (type == "cylinder") {
        double radius = size.size() > 0 ? size[0] : 0.5;
        double height = size.size() > 1 ? size[1] : 1.0;
        return RigidBody::CreateCylinder(radius, height, mass, transform, mat);
    }
    else if (type == "ramp") {
        double length = size.size() > 0 ? size[0] : 4.0;
        double thickness = size.size() > 1 ? size[1] : 0.2;
        double width = size.size() > 2 ? size[2] : 2.0;

        // Rotate the ramp by the slope angle around Z
        double angleRad = slope * 3.14159265358979323846 / 180.0;
        double halfAngle = angleRad / 2.0;
        transform.qw = std::cos(halfAngle);
        transform.qx = 0;
        transform.qy = 0;
        transform.qz = std::sin(halfAngle);

        return RigidBody::CreateBox(length, thickness, width, 0.0, transform, mat);
    }

    std::cerr << "[SceneLoader] Unknown object type: " << type << std::endl;
    return nullptr;
}

}  // namespace robosim
