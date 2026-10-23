#pragma once

#include <memory>
#include <string>
#include <vector>
#include <chrono/core/ChVector3.h>

namespace chrono {
class ChBody;
class ChSystem;
}

namespace robosim {

enum class SensorType {
    Distance,
    IMU,
    Contact
};

class Sensor {
public:
    virtual ~Sensor() = default;
    virtual void Update(chrono::ChSystem& system) = 0;
    virtual SensorType GetType() const = 0;
    virtual std::string GetName() const { return m_name; }
    void SetName(const std::string& name) { m_name = name; }

protected:
    std::string m_name = "sensor";
};

class DistanceSensor : public Sensor {
public:
    DistanceSensor(std::shared_ptr<chrono::ChBody> body,
                   const chrono::ChVector3d& localOrigin,
                   const chrono::ChVector3d& localDir,
                   double maxRange = 10.0);

    void Update(chrono::ChSystem& system) override;
    SensorType GetType() const override { return SensorType::Distance; }

    double GetDistance() const { return m_distance; }
    bool HasHit() const { return m_hit; }
    double GetMaxRange() const { return m_maxRange; }

private:
    std::shared_ptr<chrono::ChBody> m_body;
    chrono::ChVector3d m_localOrigin;
    chrono::ChVector3d m_localDir;
    double m_maxRange;
    double m_distance = 10.0;
    bool m_hit = false;
};

class IMUSensor : public Sensor {
public:
    explicit IMUSensor(std::shared_ptr<chrono::ChBody> body);

    void Update(chrono::ChSystem& system) override;
    SensorType GetType() const override { return SensorType::IMU; }

    chrono::ChVector3d GetLinearAccel() const { return m_linAccel; }
    chrono::ChVector3d GetAngularVel() const { return m_angVel; }

private:
    std::shared_ptr<chrono::ChBody> m_body;
    chrono::ChVector3d m_prevVel{0, 0, 0};
    chrono::ChVector3d m_linAccel{0, 0, 0};
    chrono::ChVector3d m_angVel{0, 0, 0};
    bool m_hasPrev = false;
};

class ContactSensor : public Sensor {
public:
    explicit ContactSensor(std::shared_ptr<chrono::ChBody> body);

    void Update(chrono::ChSystem& system) override;
    SensorType GetType() const override { return SensorType::Contact; }

    bool InContact() const { return m_inContact; }
    int GetContactCount() const { return m_contactCount; }

private:
    std::shared_ptr<chrono::ChBody> m_body;
    bool m_inContact = false;
    int m_contactCount = 0;
};

}  // namespace robosim
