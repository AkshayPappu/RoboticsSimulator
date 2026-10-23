#include "robotics/Controller.h"
#include "robotics/Robot.h"
#include "rendering/Renderer.h"

#include <irrlicht.h>

namespace robosim {

// Irrlicht key codes for arrow keys
static constexpr int KEY_UP = irr::KEY_UP;
static constexpr int KEY_DOWN = irr::KEY_DOWN;
static constexpr int KEY_LEFT = irr::KEY_LEFT;
static constexpr int KEY_RIGHT = irr::KEY_RIGHT;

KeyboardController::KeyboardController(Renderer& renderer)
    : m_renderer(renderer)
{
}

KeyboardController::~KeyboardController() = default;

void KeyboardController::OnKeyEvent(int key, bool pressed) {
    if (key == KEY_UP || key == irr::KEY_KEY_W)    m_keyForward = pressed;
    if (key == KEY_DOWN || key == irr::KEY_KEY_S)  m_keyBackward = pressed;
    if (key == KEY_LEFT || key == irr::KEY_KEY_A)  m_keyLeft = pressed;
    if (key == KEY_RIGHT || key == irr::KEY_KEY_D) m_keyRight = pressed;
}

void KeyboardController::Update(Robot& robot, double dt) {
    (void)dt;

    // Poll so held keys still drive even if a key-down event was missed
    const bool forward  = m_keyForward  || m_renderer.IsKeyPressed(KEY_UP)    || m_renderer.IsKeyPressed(irr::KEY_KEY_W);
    const bool backward = m_keyBackward || m_renderer.IsKeyPressed(KEY_DOWN)  || m_renderer.IsKeyPressed(irr::KEY_KEY_S);
    const bool leftTurn = m_keyLeft     || m_renderer.IsKeyPressed(KEY_LEFT)  || m_renderer.IsKeyPressed(irr::KEY_KEY_A);
    const bool rightTurn= m_keyRight    || m_renderer.IsKeyPressed(KEY_RIGHT) || m_renderer.IsKeyPressed(irr::KEY_KEY_D);

    double left = 0.0;
    double right = 0.0;

    if (forward) {
        left += m_speed;
        right += m_speed;
    }
    if (backward) {
        left -= m_speed;
        right -= m_speed;
    }
    if (leftTurn) {
        left -= m_turnSpeed;
        right += m_turnSpeed;
    }
    if (rightTurn) {
        left += m_turnSpeed;
        right -= m_turnSpeed;
    }

    robot.SetWheelSpeeds(left, right);
}

// ScriptController

ScriptController::~ScriptController() = default;

void ScriptController::AddCommand(double leftSpeed, double rightSpeed, double duration) {
    m_commands.push_back({leftSpeed, rightSpeed, duration});
}

void ScriptController::Reset() {
    m_currentCommand = 0;
    m_elapsed = 0.0;
}

bool ScriptController::IsFinished() const {
    return m_currentCommand >= m_commands.size();
}

void ScriptController::Update(Robot& robot, double dt) {
    if (IsFinished()) {
        robot.SetWheelSpeeds(0, 0);
        return;
    }

    const auto& cmd = m_commands[m_currentCommand];
    robot.SetWheelSpeeds(cmd.leftSpeed, cmd.rightSpeed);

    m_elapsed += dt;
    if (m_elapsed >= cmd.duration) {
        m_elapsed -= cmd.duration;
        m_currentCommand++;
    }
}

}  // namespace robosim
