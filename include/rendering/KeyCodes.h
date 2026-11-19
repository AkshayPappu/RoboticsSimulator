#pragma once

// Backend-agnostic key ids. Values match VulkanSceneGraph KeySymbol so
// ChEventHandlerVSG can forward keyBase unchanged into KeyboardController.

namespace robosim {
namespace Key {

constexpr int W = 'w';
constexpr int A = 'a';
constexpr int S = 's';
constexpr int D = 'd';

constexpr int Left  = 0xFF51;
constexpr int Up    = 0xFF52;
constexpr int Right = 0xFF53;
constexpr int Down  = 0xFF54;

inline bool IsDriveKey(int key) {
    return key == W || key == A || key == S || key == D ||
           key == Left || key == Up || key == Right || key == Down;
}

}  // namespace Key
}  // namespace robosim
