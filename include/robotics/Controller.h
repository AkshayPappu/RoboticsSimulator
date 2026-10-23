#pragma once

#include <memory>
#include <vector>
#include <functional>
#include <string>

namespace robosim {

class Robot;
class Renderer;

class Controller {
public:
    virtual ~Controller() = default;
    virtual void Update(Robot& robot, double dt) = 0;
    virtual std::string GetName() const = 0;
};

class KeyboardController : public Controller {
public:
    explicit KeyboardController(Renderer& renderer);
    ~KeyboardController() override;

    void Update(Robot& robot, double dt) override;
    std::string GetName() const override { return "KeyboardController"; }

    void SetSpeed(double speed) { m_speed = speed; }
    void SetTurnSpeed(double turnSpeed) { m_turnSpeed = turnSpeed; }

    void OnKeyEvent(int key, bool pressed);

private:
    Renderer& m_renderer;
    double m_speed = 8.0;
    double m_turnSpeed = 4.0;

    bool m_keyForward = false;
    bool m_keyBackward = false;
    bool m_keyLeft = false;
    bool m_keyRight = false;
};

struct ScriptCommand {
    double leftSpeed;
    double rightSpeed;
    double duration;  // seconds
};

class ScriptController : public Controller {
public:
    ScriptController() = default;
    ~ScriptController() override;

    void Update(Robot& robot, double dt) override;
    std::string GetName() const override { return "ScriptController"; }

    void AddCommand(double leftSpeed, double rightSpeed, double duration);
    void Reset();
    bool IsFinished() const;

private:
    std::vector<ScriptCommand> m_commands;
    size_t m_currentCommand = 0;
    double m_elapsed = 0.0;
};

class CallbackController : public Controller {
public:
    using UpdateFn = std::function<void(Robot&, double)>;

    explicit CallbackController(UpdateFn fn) : m_fn(std::move(fn)) {}
    ~CallbackController() override = default;

    void Update(Robot& robot, double dt) override {
        if (m_fn) m_fn(robot, dt);
    }
    std::string GetName() const override { return "CallbackController"; }

private:
    UpdateFn m_fn;
};

}  // namespace robosim
