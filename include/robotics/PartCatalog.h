#pragma once

#include "physics/Assets.h"

#include <nlohmann/json.hpp>
#include <string>
#include <vector>
#include <optional>

namespace robosim {

struct MountFrame {
    std::string id;
    double x = 0, y = 0, z = 0;
    double axisX = 0, axisY = 0, axisZ = 1;
    std::string role;
};

struct CollisionHull {
    std::string shape = "box";
    double sizeX = 0.1, sizeY = 0.1, sizeZ = 0.1;
    double radius = 0.05;
    double height = 0.05;
};

struct CatalogPart {
    std::string id;
    std::string name;
    std::string category;
    VisualSpec visual;
    CollisionHull collision;
    double mass = 1.0;
    bool visualOnly = false;
    std::string joint = "revolute";
    std::vector<MountFrame> mounts;
};

struct AssemblyItem {
    std::string instanceId;
    std::string partId;
    std::string parentId;
    std::string mountId;
    bool motor = true;
};

class PartCatalog {
public:
    static PartCatalog& Instance();

    bool Load(const std::string& path = {});
    bool IsLoaded() const { return !m_parts.empty(); }
    const std::string& Path() const { return m_path; }

    const CatalogPart* Find(const std::string& id) const;
    const std::vector<CatalogPart>& Parts() const { return m_parts; }
    std::vector<const CatalogPart*> ByCategory(const std::string& category) const;

    const MountFrame* FindMount(const CatalogPart& part, const std::string& mountId) const;

    static nlohmann::json AssemblyToRobotJson(const std::vector<AssemblyItem>& items,
                                              const PartCatalog& catalog,
                                              const std::string& name = "assembled");

    static bool ParseAssembly(const nlohmann::json& j, std::vector<AssemblyItem>& out);

private:
    PartCatalog() = default;
    std::vector<CatalogPart> m_parts;
    std::string m_path;
};

}  // namespace robosim
