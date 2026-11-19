#include "robotics/Controller.h"
#include "robotics/Robot.h"
#include "rendering/Renderer.h"
#include "rendering/KeyCodes.h"

#include <algorithm>
#include <cmath>

namespace robosim {

namespace {

double Approach(double current, double target, double rate, double dt) {
    if (dt <= 0.0) return target;
    const double maxStep = rate * dt;
    const double delta = target - current;
    if (std::abs(delta) <= maxStep) return target;
    return current + std::copysign(maxStep, delta);
}

}  // namespace

KeyboardController::KeyboardController(Renderer& renderer)
    : m_renderer(renderer)
{
}

KeyboardController::~KeyboardController() = default;

void KeyboardController::OnKeyEvent(int key, bool pressed) {
    if (key == Key::Up || key == Key::W)    m_keyForward = pressed;
    if (key == Key::Down || key == Key::S)  m_keyBackward = pressed;
    if (key == Key::Left || key == Key::A)  m_keyLeft = pressed;
    if (key == Key::Right || key == Key::D) m_keyRight = pressed;
}

void KeyboardController::Update(Robot& robot, double dt) {
    // Poll so held keys still drive even if a key-down event was missed.
    const bool forward  = m_keyForward  || m_renderer.IsKeyPressed(Key::Up)    || m_renderer.IsKeyPressed(Key::W);
    const bool backward = m_keyBackward || m_renderer.IsKeyPressed(Key::Down)  || m_renderer.IsKeyPressed(Key::S);
    const bool leftTurn = m_keyLeft     || m_renderer.IsKeyPressed(Key::Left)  || m_renderer.IsKeyPressed(Key::A);
    const bool rightTurn= m_keyRight    || m_renderer.IsKeyPressed(Key::Right) || m_renderer.IsKeyPressed(Key::D);

    double targetLeft = 0.0;
    double targetRight = 0.0;

    if (forward) {
        targetLeft += m_speed;
        targetRight += m_speed;
    }
    if (backward) {
        targetLeft -= m_speed;
        targetRight -= m_speed;
    }
    if (leftTurn) {
        targetLeft -= m_turnSpeed;
        targetRight += m_turnSpeed;
    }
    if (rightTurn) {
        targetLeft += m_turnSpeed;
        targetRight -= m_turnSpeed;
    }

    // Decelerate faster than accelerate so release feels snappy, press feels smooth.
    const double rate = (std::abs(targetLeft) < 1e-6 && std::abs(targetRight) < 1e-6)
                            ? m_accel * 1.8
                            : m_accel;
    const double stepDt = std::clamp(dt, 1.0 / 240.0, 1.0 / 20.0);
    m_leftCmd = Approach(m_leftCmd, targetLeft, rate, stepDt);
    m_rightCmd = Approach(m_rightCmd, targetRight, rate, stepDt);

    robot.SetWheelSpeeds(m_leftCmd, m_rightCmd);
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
