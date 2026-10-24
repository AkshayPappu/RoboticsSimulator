#pragma once

#include "robotics/AssemblyGraph.h"

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace robosim {

class Joint;
class Motor;
class PhysicsWorld;
class RigidBody;

struct BuiltAssembly {
    std::shared_ptr<RigidBody> root;
    std::unordered_map<std::string, std::shared_ptr<RigidBody>> bodies;
    std::vector<std::shared_ptr<Joint>> joints;
    std::vector<std::shared_ptr<Motor>> motors;
    std::vector<std::string> motorIds;
    std::vector<int> driveGroups;
};

// previewBodyId draws a tinted copy of that body's collision primitive.
// Returns false when issue.errors is non-empty. Nothing is added to the world in that case.
bool BuildAssembly(PhysicsWorld& world,
                   const AssemblyGraph& graph,
                   BuiltAssembly& out,
                   AssemblyIssue& issue,
                   const std::string& previewBodyId = {});

}  // namespace robosim
