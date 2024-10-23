#include <unistd.h>
#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"


#include <cmath>
#include <chrono>
#include <thread>

void sweepMotorPosition(int id, double duration_seconds, double freq_hz) {
    double start_angle = -M_PI;  // Start at -180 degrees (-π radians)
    double end_angle = M_PI;     // End at 180 degrees (π radians)
    int total_steps = duration_seconds * freq_hz;  // Total number of steps
    double step_size = (end_angle - start_angle) / total_steps;  // Angle change per step

    for (int step = 0; step <= total_steps; step++) {
        double current_angle = start_angle + step * step_size;  // Calculate current position

        // Send command to the motor
        cmd.id = id;
        cmd.q = current_angle;   // Set position (in radians)
        cmd.kp = 0.2;            // Set position stiffness (adjust as needed)
        cmd.kd = 0.01;           // Set damping (adjust as needed)
        cmd.tau = 0;             // No feedforward torque
        cmd.dq = 0;              // Velocity set to 0 (pure position control)

        // Call the motor's control function here (or publish ROS message)
        serial.sendRecv(&cmd,&data);

        // Sleep to maintain the desired frequency
        std::this_thread::sleep_for(std::chrono::milliseconds((int)(1000.0 / freq_hz)));
    }
}

// Example usage
int main() {
    int motor_id = 0;      // ID of the motor
    double duration = 2.0; // Sweep time (2 seconds)
    double frequency = 100.0; // Control frequency (100 Hz)

    // Call the function to sweep motor position
    sweepMotorPosition(motor_id, duration, frequency);

    return 0;
}
