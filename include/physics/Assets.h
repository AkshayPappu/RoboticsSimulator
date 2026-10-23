#pragma once

#include <string>
#include <memory>

namespace chrono {
class ChBody;
class ChVisualMaterial;
}

namespace robosim {

struct VisualSpec {
    std::string mesh;
    double scaleX = 1.0;
    double scaleY = 1.0;
    double scaleZ = 1.0;
    double offsetX = 0.0;
    double offsetY = 0.0;
    double offsetZ = 0.0;
    std::string texture;
    float texScaleX = 1.0f;
    float texScaleY = 1.0f;
    float colorR = -1.0f;
    float colorG = -1.0f;
    float colorB = -1.0f;
    float metallic = -1.0f;
    float roughness = -1.0f;

    bool HasMesh() const { return !mesh.empty(); }
    bool HasTexture() const { return !texture.empty(); }
};

std::string ResolveAssetPath(const std::string& path);

std::shared_ptr<chrono::ChVisualMaterial> MakeVisualMaterial(
    float r, float g, float b,
    float metallic = 0.0f,
    float roughness = 0.6f,
    const std::string& texture = {},
    float texScaleX = 1.0f,
    float texScaleY = 1.0f
);

void ApplyVisualSpec(chrono::ChBody& body, const VisualSpec& spec, bool replaceDefault = true);
void ApplyShapeTexture(chrono::ChBody& body, const std::string& texture, float scaleX, float scaleY);

}  // namespace robosim
