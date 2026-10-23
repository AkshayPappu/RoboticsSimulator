#include "core/Telemetry.h"
#include "robotics/Robot.h"
#include "robotics/Motor.h"

#include <iostream>
#include <iomanip>
#include <cmath>

namespace robosim {

Telemetry::Telemetry() = default;
Telemetry::~Telemetry() {
    if (m_recording) {
        EndRecording();
    }
}

void Telemetry::SetOutputPath(const std::string& path) {
    m_outputPath = path;
}

void Telemetry::BeginRecording() {
    m_records.clear();
    m_recording = true;

    if (!m_outputPath.empty()) {
        m_stream = std::make_unique<std::ofstream>(m_outputPath);
        if (m_stream->is_open()) {
            *m_stream << "time,step,pos_x,pos_y,pos_z,"
                      << "quat_w,quat_x,quat_y,quat_z,"
                      << "vel_x,vel_y,vel_z,"
                      << "heading,flipped,"
                      << "left_wheel_speed,right_wheel_speed,"
                      << "left_motor_torque,right_motor_torque"
                      << std::endl;
        }
    }

    std::cout << "[Telemetry] Recording started" << std::endl;
}

void Telemetry::Record(double time, int step, const Robot* robot) {
    if (!m_recording) return;
    if (step % m_interval != 0) return;

    TelemetryRecord rec;
    rec.time = time;
    rec.step = step;

    if (robot) {
        robot->GetPosition(rec.posX, rec.posY, rec.posZ);
        robot->GetOrientation(rec.quatW, rec.quatX, rec.quatY, rec.quatZ);
        robot->GetLinearVelocity(rec.velX, rec.velY, rec.velZ);
        rec.heading = robot->GetHeading();
        rec.flipped = robot->IsFlipped();
        rec.leftWheelSpeed = robot->GetLeftWheelSpeed();
        rec.rightWheelSpeed = robot->GetRightWheelSpeed();

        auto& motors = robot->GetMotors();
        if (motors.size() >= 2) {
            rec.leftMotorTorque = motors[0]->GetAppliedTorque();
            rec.rightMotorTorque = motors[1]->GetAppliedTorque();
        }
    }

    m_records.push_back(rec);

    // Stream to CSV file if open
    if (m_stream && m_stream->is_open()) {
        *m_stream << std::fixed << std::setprecision(6)
                  << rec.time << "," << rec.step << ","
                  << rec.posX << "," << rec.posY << "," << rec.posZ << ","
                  << rec.quatW << "," << rec.quatX << "," << rec.quatY << "," << rec.quatZ << ","
                  << rec.velX << "," << rec.velY << "," << rec.velZ << ","
                  << rec.heading << "," << (rec.flipped ? 1 : 0) << ","
                  << rec.leftWheelSpeed << "," << rec.rightWheelSpeed << ","
                  << rec.leftMotorTorque << "," << rec.rightMotorTorque
                  << std::endl;
    }
}

void Telemetry::EndRecording() {
    m_recording = false;
    if (m_stream) {
        m_stream->close();
        m_stream.reset();
    }
    std::cout << "[Telemetry] Recording ended. " << m_records.size() << " records." << std::endl;
}

void Telemetry::ExportCSV(const std::string& path) const {
    std::ofstream file(path);
    if (!file.is_open()) {
        std::cerr << "[Telemetry] Failed to open: " << path << std::endl;
        return;
    }

    file << "time,step,pos_x,pos_y,pos_z,"
         << "quat_w,quat_x,quat_y,quat_z,"
         << "vel_x,vel_y,vel_z,"
         << "heading,flipped,"
         << "left_wheel_speed,right_wheel_speed,"
         << "left_motor_torque,right_motor_torque"
         << std::endl;

    for (const auto& rec : m_records) {
        file << std::fixed << std::setprecision(6)
             << rec.time << "," << rec.step << ","
             << rec.posX << "," << rec.posY << "," << rec.posZ << ","
             << rec.quatW << "," << rec.quatX << "," << rec.quatY << "," << rec.quatZ << ","
             << rec.velX << "," << rec.velY << "," << rec.velZ << ","
             << rec.heading << "," << (rec.flipped ? 1 : 0) << ","
             << rec.leftWheelSpeed << "," << rec.rightWheelSpeed << ","
             << rec.leftMotorTorque << "," << rec.rightMotorTorque
             << std::endl;
    }

    std::cout << "[Telemetry] Exported " << m_records.size() << " records to " << path << std::endl;
}

Telemetry::Summary Telemetry::ComputeSummary(double goalX, double goalY, double goalZ) const {
    Summary summary;
    if (m_records.empty()) return summary;

    summary.totalSteps = m_records.back().step;
    summary.totalTime = m_records.back().time;

    double prevX = m_records[0].posX, prevZ = m_records[0].posZ;

    for (const auto& rec : m_records) {
        double speed = std::sqrt(rec.velX * rec.velX + rec.velY * rec.velY + rec.velZ * rec.velZ);
        summary.maxSpeed = std::max(summary.maxSpeed, speed);

        double dx = rec.posX - prevX;
        double dz = rec.posZ - prevZ;
        summary.totalDistance += std::sqrt(dx * dx + dz * dz);
        prevX = rec.posX;
        prevZ = rec.posZ;

        if (rec.flipped) {
            summary.robotFlipped = true;
        }
    }

    const auto& last = m_records.back();
    double gx = goalX - last.posX;
    double gz = goalZ - last.posZ;
    summary.finalDistToGoal = std::sqrt(gx * gx + gz * gz);
    summary.reachedGoal = summary.finalDistToGoal < 1.0;

    return summary;
}

}  // namespace robosim
