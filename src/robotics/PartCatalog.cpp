#include "robotics/PartCatalog.h"

#include <fstream>
#include <iostream>
#include <unordered_map>
#include <filesystem>

using json = nlohmann::json;

namespace robosim {

PartCatalog& PartCatalog::Instance() {
    static PartCatalog catalog;
    return catalog;
}

static VisualSpec VisualFromPartJson(const json& p) {
    VisualSpec vis;
    vis.mesh = p.value("mesh", std::string());
    vis.texture = p.value("texture", std::string());
    if (p.contains("visual") && p["visual"].is_object()) {
        const auto& v = p["visual"];
        vis.mesh = v.value("mesh", vis.mesh);
        vis.texture = v.value("texture", vis.texture);
        vis.scaleX = v.value("scale", vis.scaleX);
        vis.scaleY = vis.scaleX;
        vis.scaleZ = vis.scaleX;
        if (v.contains("scale") && v["scale"].is_array() && v["scale"].size() >= 3) {
            vis.scaleX = v["scale"][0].get<double>();
            vis.scaleY = v["scale"][1].get<double>();
            vis.scaleZ = v["scale"][2].get<double>();
        }
        if (v.contains("color") && v["color"].is_array() && v["color"].size() >= 3) {
            vis.colorR = v["color"][0].get<float>();
            vis.colorG = v["color"][1].get<float>();
            vis.colorB = v["color"][2].get<float>();
        }
        vis.metallic = v.value("metallic", vis.metallic);
        vis.roughness = v.value("roughness", vis.roughness);
    }
    return vis;
}

bool PartCatalog::Load(const std::string& path) {
    std::string resolved = path;
    if (resolved.empty()) {
#ifdef SIM_ASSETS_DIR
        resolved = (std::filesystem::path(SIM_ASSETS_DIR) / "parts" / "catalog.json").string();
#else
        resolved = "assets/parts/catalog.json";
#endif
    }

    std::ifstream file(resolved);
    if (!file.is_open()) {
        std::cerr << "[PartCatalog] Failed to open " << resolved << std::endl;
        return false;
    }

    json j;
    try {
        file >> j;
    } catch (const json::exception& e) {
        std::cerr << "[PartCatalog] Parse error: " << e.what() << std::endl;
        return false;
    }

    m_parts.clear();
    m_path = resolved;
    if (!j.contains("parts") || !j["parts"].is_array()) {
        return false;
    }

    for (const auto& p : j["parts"]) {
        CatalogPart part;
        part.id = p.value("id", std::string());
        part.name = p.value("name", part.id);
        part.category = p.value("category", std::string("misc"));
        part.mass = p.value("mass", 1.0);
        part.visualOnly = p.value("visual_only", false);
        part.joint = p.value("joint", std::string("revolute"));
        part.visual = VisualFromPartJson(p);

        if (p.contains("collision") && p["collision"].is_object()) {
            const auto& c = p["collision"];
            part.collision.shape = c.value("shape", std::string("box"));
            part.collision.radius = c.value("radius", part.collision.radius);
            part.collision.height = c.value("height", part.collision.height);
            if (c.contains("size") && c["size"].is_array()) {
                if (c["size"].size() > 0) part.collision.sizeX = c["size"][0].get<double>();
                if (c["size"].size() > 1) part.collision.sizeY = c["size"][1].get<double>();
                if (c["size"].size() > 2) part.collision.sizeZ = c["size"][2].get<double>();
            }
        }

        if (p.contains("mounts") && p["mounts"].is_array()) {
            for (const auto& m : p["mounts"]) {
                MountFrame frame;
                frame.id = m.value("id", std::string());
                frame.role = m.value("role", std::string());
                if (m.contains("pos") && m["pos"].is_array() && m["pos"].size() >= 3) {
                    frame.x = m["pos"][0].get<double>();
                    frame.y = m["pos"][1].get<double>();
                    frame.z = m["pos"][2].get<double>();
                }
                if (m.contains("axis") && m["axis"].is_array() && m["axis"].size() >= 3) {
                    frame.axisX = m["axis"][0].get<double>();
                    frame.axisY = m["axis"][1].get<double>();
                    frame.axisZ = m["axis"][2].get<double>();
                }
                part.mounts.push_back(frame);
            }
        }
        if (!part.id.empty()) {
            m_parts.push_back(part);
        }
    }

    std::cout << "[PartCatalog] Loaded " << m_parts.size() << " parts from " << resolved << std::endl;
    return true;
}

const CatalogPart* PartCatalog::Find(const std::string& id) const {
    for (const auto& p : m_parts) {
        if (p.id == id) return &p;
    }
    return nullptr;
}

std::vector<const CatalogPart*> PartCatalog::ByCategory(const std::string& category) const {
    std::vector<const CatalogPart*> out;
    for (const auto& p : m_parts) {
        if (p.category == category) out.push_back(&p);
    }
    return out;
}

const MountFrame* PartCatalog::FindMount(const CatalogPart& part, const std::string& mountId) const {
    for (const auto& m : part.mounts) {
        if (m.id == mountId) return &m;
    }
    return nullptr;
}

bool PartCatalog::ParseAssembly(const json& j, std::vector<AssemblyItem>& out) {
    out.clear();
    const json* arr = nullptr;
    if (j.contains("assembly") && j["assembly"].is_array()) {
        arr = &j["assembly"];
    } else if (j.contains("parts") && j["parts"].is_array() && !j.contains("chassis")) {
        arr = &j["parts"];
    }
    if (!arr) return false;

    for (const auto& item : *arr) {
        AssemblyItem a;
        a.partId = item.value("part", item.value("part_id", std::string()));
        a.instanceId = item.value("id", item.value("instance", a.partId));
        a.parentId = item.value("parent", std::string());
        a.mountId = item.value("mount", std::string());
        a.motor = item.value("motor", true);
        if (!a.partId.empty()) {
            out.push_back(a);
        }
    }
    return !out.empty();
}

nlohmann::json PartCatalog::AssemblyToRobotJson(const std::vector<AssemblyItem>& items,
                                                const PartCatalog& catalog,
                                                const std::string& name) {
    json j;
    j["name"] = name;
    j["max_motor_speed"] = 10.0;
    j["max_torque"] = 5.0;
    j["wheels"] = json::array();
    j["casters"] = json::array();
    j["links"] = json::array();
    j["extras"] = json::array();
    j["assembly"] = json::array();

    std::unordered_map<std::string, const CatalogPart*> instancePart;
    std::unordered_map<std::string, int> linkIndex;
    int nextLink = 0;
    std::string chassisInstance;

    auto visualJson = [](const VisualSpec& vis) {
        json v;
        v["mesh"] = vis.mesh;
        if (vis.scaleX != 1.0 || vis.scaleY != 1.0 || vis.scaleZ != 1.0) {
            v["scale"] = {vis.scaleX, vis.scaleY, vis.scaleZ};
        } else {
            v["scale"] = vis.scaleX;
        }
        if (!vis.texture.empty()) v["texture"] = vis.texture;
        if (vis.colorR >= 0.0f) {
            v["color"] = {vis.colorR, vis.colorG, vis.colorB};
        }
        if (vis.metallic >= 0.0f) v["metallic"] = vis.metallic;
        if (vis.roughness >= 0.0f) v["roughness"] = vis.roughness;
        return v;
    };

    for (const auto& item : items) {
        j["assembly"].push_back({
            {"id", item.instanceId},
            {"part", item.partId},
            {"parent", item.parentId},
            {"mount", item.mountId},
            {"motor", item.motor}
        });

        const CatalogPart* part = catalog.Find(item.partId);
        if (!part) continue;
        instancePart[item.instanceId] = part;

        const MountFrame* mount = nullptr;
        if (!item.parentId.empty() && !item.mountId.empty()) {
            auto it = instancePart.find(item.parentId);
            if (it != instancePart.end() && it->second) {
                mount = catalog.FindMount(*it->second, item.mountId);
            }
        }

        if (part->category == "chassis" && chassisInstance.empty()) {
            chassisInstance = item.instanceId;
            j["chassis"] = {
                {"type", "box"},
                {"size", {part->collision.sizeX, part->collision.sizeY, part->collision.sizeZ}},
                {"mass", part->mass},
                {"visual", visualJson(part->visual)}
            };
            continue;
        }

        double ox = mount ? mount->x : 0.0;
        double oy = mount ? mount->y : 0.0;
        double oz = mount ? mount->z : 0.0;

        if (part->category == "wheel") {
            json w;
            w["side"] = item.instanceId;
            w["radius"] = part->collision.radius;
            w["width"] = part->collision.height;
            w["mass"] = part->mass;
            w["offset"] = {ox, oy, oz};
            w["motor"] = item.motor;
            w["visual"] = visualJson(part->visual);
            j["wheels"].push_back(w);
        } else if (part->category == "caster") {
            json c;
            c["radius"] = part->collision.radius;
            c["mass"] = part->mass;
            c["offset"] = {ox, oy, oz};
            c["visual"] = visualJson(part->visual);
            j["casters"].push_back(c);
        } else if (part->category == "arm") {
            int parent = -1;
            if (!item.parentId.empty() && item.parentId != chassisInstance) {
                auto it = linkIndex.find(item.parentId);
                if (it != linkIndex.end()) parent = it->second;
            }
            json l;
            l["name"] = item.instanceId;
            l["parent"] = parent;
            l["joint"] = part->joint.empty() ? "revolute" : part->joint;
            l["geometry"] = part->collision.shape;
            l["size"] = {part->collision.sizeX, part->collision.sizeY, part->collision.sizeZ};
            l["radius"] = part->collision.radius;
            l["mass"] = part->mass;
            l["offset"] = {ox, oy, oz};
            l["axis"] = {0, 0, 1};
            l["motor"] = item.motor && part->joint != "fixed";
            l["visual"] = visualJson(part->visual);
            j["links"].push_back(l);
            linkIndex[item.instanceId] = nextLink++;
        } else {
            json e;
            e["name"] = item.instanceId;
            e["offset"] = {ox, oy, oz};
            e["visual"] = visualJson(part->visual);
            j["extras"].push_back(e);
        }
    }

    if (!j.contains("chassis")) {
        j["chassis"] = {
            {"type", "box"},
            {"size", {0.45, 0.08, 0.22}},
            {"mass", 5.0}
        };
    }
    return j;
}

}  // namespace robosim
