#pragma once

#include <memory>
#include <chrono/physics/ChSystemNSC.h>
#include <chrono/physics/ChSystemSMC.h>
#include <chrono/physics/ChBody.h>

namespace robosim {

enum class ContactMethod {
    NSC,  // Complementarity-based (default, good for rigid contact)
    SMC   // Penalty-based (smoother, better for RL)
};

class ChronoEngine {
public:
    explicit ChronoEngine(ContactMethod method = ContactMethod::NSC);
    ~ChronoEngine();

    ChronoEngine(const ChronoEngine&) = delete;
    ChronoEngine& operator=(const ChronoEngine&) = delete;

    void SetGravity(double x, double y, double z);
    void SetTimestep(double dt);
    double GetTimestep() const { return m_timestep; }

    void DoStep();
    void DoStep(double dt);
    double GetTime() const;

    void SetSolverMaxIterations(int iterations);
    void SetCollisionEnvelope(double envelope);

    chrono::ChSystem& GetSystem() { return *m_system; }
    const chrono::ChSystem& GetSystem() const { return *m_system; }

    void AddBody(std::shared_ptr<chrono::ChBody> body);
    void RemoveBody(std::shared_ptr<chrono::ChBody> body);
    void AddLink(std::shared_ptr<chrono::ChLinkBase> link);
    void RemoveLink(std::shared_ptr<chrono::ChLinkBase> link);

    void Clear();

private:
    std::unique_ptr<chrono::ChSystem> m_system;
    double m_timestep = 0.001;  // 1ms default
    ContactMethod m_contactMethod;
};

}  // namespace robosim
