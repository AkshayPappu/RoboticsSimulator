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
    if (key == KEY_UP)    m_keyForward = pressed;
    if (key == KEY_DOWN)  m_keyBackward = pressed;
    if (key == KEY_LEFT)  m_keyLeft = pressed;
    if (key == KEY_RIGHT) m_keyRight = pressed;
}

void KeyboardController::Update(Robot& robot, double dt) {
    double left = 0.0;
    double right = 0.0;

    if (m_keyForward) {
        left += m_speed;
        right += m_speed;
    }
    if (m_keyBackward) {
        left -= m_speed;
        right -= m_speed;
    }
    if (m_keyLeft) {
        left -= m_turnSpeed;
        right += m_turnSpeed;
    }
    if (m_keyRight) {
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
