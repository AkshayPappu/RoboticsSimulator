#pragma once

#include <string>
#include <vector>
#include <fstream>
#include <memory>

namespace robosim {

class Robot;

struct TelemetryRecord {
    double time = 0.0;
    int step = 0;

    // Robot state
    double posX = 0, posY = 0, posZ = 0;
    double quatW = 1, quatX = 0, quatY = 0, quatZ = 0;
    double velX = 0, velY = 0, velZ = 0;
    double heading = 0;
    bool flipped = false;

    // Motor state
    double leftWheelSpeed = 0;
    double rightWheelSpeed = 0;
    double leftMotorTorque = 0;
    double rightMotorTorque = 0;
};

class Telemetry {
public:
    Telemetry();
    ~Telemetry();

    Telemetry(const Telemetry&) = delete;
    Telemetry& operator=(const Telemetry&) = delete;

    void SetOutputPath(const std::string& path);
    void SetRecordInterval(int everyNSteps) { m_interval = everyNSteps; }

    void BeginRecording();
    void Record(double time, int step, const Robot* robot);
    void EndRecording();

    const std::vector<TelemetryRecord>& GetRecords() const { return m_records; }

    void ExportCSV(const std::string& path) const;

    bool IsRecording() const { return m_recording; }

    struct Summary {
        double totalTime = 0;
        int totalSteps = 0;
        double maxSpeed = 0;
        double totalDistance = 0;
        double finalDistToGoal = 0;
        bool reachedGoal = false;
        bool robotFlipped = false;
    };

    Summary ComputeSummary(double goalX = 0, double goalY = 0, double goalZ = 20) const;

private:
    std::vector<TelemetryRecord> m_records;
    std::unique_ptr<std::ofstream> m_stream;
    std::string m_outputPath;
    int m_interval = 1;
    bool m_recording = false;
};

}  // namespace robosim
