#include "physics/Assets.h"

#include <chrono/core/ChVector3.h>
#include <chrono/assets/ChVisualMaterial.h>
#include <chrono/assets/ChVisualModel.h>
#include <chrono/assets/ChVisualShape.h>
#include <chrono/assets/ChVisualShapeModelFile.h>
#include <chrono/assets/ChVisualShapeTriangleMesh.h>
#include <chrono/geometry/ChTriangleMeshConnected.h>
#include <chrono/physics/ChBody.h>

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <iostream>

namespace robosim {

std::string ResolveAssetPath(const std::string& path) {
    if (path.empty()) return {};

    std::filesystem::path p(path);
    if (p.is_absolute() && std::filesystem::exists(p)) {
        return p.string();
    }
    if (std::filesystem::exists(p)) {
        return std::filesystem::absolute(p).string();
    }

#ifdef SIM_ASSETS_DIR
    std::filesystem::path fromAssets = std::filesystem::path(SIM_ASSETS_DIR) / path;
    if (std::filesystem::exists(fromAssets)) {
        return std::filesystem::absolute(fromAssets).string();
    }
    std::filesystem::path bare = std::filesystem::path(SIM_ASSETS_DIR) / p.filename();
    if (std::filesystem::exists(bare)) {
        return std::filesystem::absolute(bare).string();
    }
#endif
    return path;
}

std::shared_ptr<chrono::ChVisualMaterial> MakeVisualMaterial(
    float r, float g, float b,
    float metallic,
    float roughness,
    const std::string& texture,
    float texScaleX,
    float texScaleY)
{
    auto mat = chrono_types::make_shared<chrono::ChVisualMaterial>();
    chrono::ChColor color(r, g, b);
    mat->SetDiffuseColor(color);
    // Irrlicht is Phong, not PBR: keep ambient high enough that meshes don't fall into black.
    mat->SetAmbientColor(chrono::ChColor(r * 0.50f, g * 0.50f, b * 0.50f));
    mat->SetSpecularColor(chrono::ChColor(0.12f, 0.12f, 0.12f));
    mat->SetUseSpecularWorkflow(true);
    mat->SetMetallic(metallic);
    mat->SetRoughness(roughness);
    if (!texture.empty()) {
        std::string resolved = ResolveAssetPath(texture);
        mat->SetKdTexture(resolved);
        mat->SetTextureScale(texScaleX, texScaleY);
    }
    return mat;
}

void ApplyVisualSpec(chrono::ChBody& body, const VisualSpec& spec, bool replaceDefault) {
    if (!spec.HasMesh() && !spec.HasTexture()) {
        return;
    }

    if (spec.HasMesh()) {
        std::string resolved = ResolveAssetPath(spec.mesh);
        if (!std::filesystem::exists(resolved)) {
            std::cerr << "[Assets] Mesh not found: " << spec.mesh << " (" << resolved << ")" << std::endl;
            return;
        }
        if (replaceDefault) {
            if (auto vis = body.GetVisualModel()) {
                vis->Clear();
            }
        }

        std::string ext = std::filesystem::path(resolved).extension().string();
        std::transform(ext.begin(), ext.end(), ext.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

        std::shared_ptr<chrono::ChVisualShape> shape;
        std::shared_ptr<chrono::ChTriangleMeshConnected> mesh;
        if (ext == ".stl") {
            mesh = chrono::ChTriangleMeshConnected::CreateFromSTLFile(resolved, true);
        } else if (ext == ".obj") {
            mesh = chrono::ChTriangleMeshConnected::CreateFromWavefrontFile(resolved, true, false);
        }
        if (mesh && mesh->GetNumTriangles() > 0) {
            mesh->RepairDuplicateVertexes(1e-5);
            const auto& verts = mesh->GetCoordsVertices();
            const auto& faces = mesh->GetIndicesVertexes();
            auto& norms = mesh->GetCoordsNormals();
            auto& faceNorms = mesh->GetIndicesNormals();
            norms.clear();
            faceNorms.clear();
            norms.reserve(faces.size());
            faceNorms.reserve(faces.size());
            for (const auto& face : faces) {
                const auto& a = verts[face.x()];
                const auto& b = verts[face.y()];
                const auto& c = verts[face.z()];
                auto n = Vcross(b - a, c - a);
                if (n.Length() > 1e-12) n.Normalize();
                else n = chrono::ChVector3d(0, 1, 0);
                const int ni = static_cast<int>(norms.size());
                norms.push_back(n);
                faceNorms.push_back(chrono::ChVector3i(ni, ni, ni));
            }
            auto tri = chrono_types::make_shared<chrono::ChVisualShapeTriangleMesh>();
            tri->SetMesh(mesh, false);
            tri->SetScale(chrono::ChVector3d(spec.scaleX, spec.scaleY, spec.scaleZ));
            tri->SetMutable(false);
            tri->SetBackfaceCull(true);
            shape = tri;
        } else {
            auto fileShape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>(resolved);
            fileShape->SetScale(chrono::ChVector3d(spec.scaleX, spec.scaleY, spec.scaleZ));
            shape = fileShape;
        }

        float r = spec.colorR >= 0.0f ? spec.colorR : 0.65f;
        float g = spec.colorG >= 0.0f ? spec.colorG : 0.65f;
        float b = spec.colorB >= 0.0f ? spec.colorB : 0.65f;
        float metallic = spec.metallic >= 0.0f ? spec.metallic : 0.35f;
        float roughness = spec.roughness >= 0.0f ? spec.roughness : 0.45f;
        auto mat = MakeVisualMaterial(r, g, b, metallic, roughness, spec.texture, spec.texScaleX, spec.texScaleY);
        shape->SetMaterial(0, mat);
        shape->SetColor(chrono::ChColor(r, g, b));

        body.AddVisualShape(
            shape,
            chrono::ChFramed(chrono::ChVector3d(spec.offsetX, spec.offsetY, spec.offsetZ)));
        return;
    }

    ApplyShapeTexture(body, spec.texture, spec.texScaleX, spec.texScaleY);
}

void ApplyShapeTexture(chrono::ChBody& body, const std::string& texture, float scaleX, float scaleY) {
    if (texture.empty()) return;
    std::string resolved = ResolveAssetPath(texture);
    auto vis = body.GetVisualModel();
    if (!vis) return;
    for (unsigned int i = 0; i < vis->GetNumShapes(); ++i) {
        vis->GetShape(i)->SetTexture(resolved, scaleX, scaleY);
    }
}

}  // namespace robosim
