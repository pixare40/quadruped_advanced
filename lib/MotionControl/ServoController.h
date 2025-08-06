/**
 * @file ServoController.h
 * @brief Low-level servo control interface for quadruped robot
 * @author Advanced Quadruped Team
 * @date 2024
 */

#pragma once
#include <Arduino.h>
#include <Servo.h>

namespace MotionControl {

constexpr uint8_t NUM_SERVOS = 12;
constexpr uint8_t SERVOS_PER_LEG = 3;
constexpr uint8_t NUM_LEGS = 4;

// Servo indices for each leg (Front Left, Front Right, Rear Left, Rear Right)
enum class ServoIndex : uint8_t {
    // Front Left Leg (0-2)
    FL_HIP = 0,      // Hip joint (forward/backward)
    FL_SHOULDER = 1, // Shoulder joint (up/down)
    FL_KNEE = 2,     // Knee joint (extend/retract)
    
    // Front Right Leg (3-5)
    FR_HIP = 3,
    FR_SHOULDER = 4,
    FR_KNEE = 5,
    
    // Rear Left Leg (6-8)
    RL_HIP = 6,
    RL_SHOULDER = 7,
    RL_KNEE = 8,
    
    // Rear Right Leg (9-11)
    RR_HIP = 9,
    RR_SHOULDER = 10,
    RR_KNEE = 11
};

enum class LegID : uint8_t {
    FRONT_LEFT = 0,
    FRONT_RIGHT = 1,
    REAR_LEFT = 2,
    REAR_RIGHT = 3
};

struct ServoConfig {
    uint8_t pin;
    uint16_t min_pulse = 500;   // Minimum pulse width (microseconds)
    uint16_t max_pulse = 2500;  // Maximum pulse width (microseconds)
    float min_angle = 0.0f;     // Minimum angle (degrees)
    float max_angle = 180.0f;   // Maximum angle (degrees)
    float home_angle = 90.0f;   // Home/neutral position
    bool inverted = false;      // Invert direction
};

struct LegPosition {
    float hip;      // Hip angle (-90 to 90 degrees)
    float shoulder; // Shoulder angle (0 to 180 degrees)
    float knee;     // Knee angle (0 to 180 degrees)
};

class ServoController {
public:
    ServoController();
    
    // Initialization
    bool initialize(const uint8_t servo_pins[NUM_SERVOS]);
    void setServoConfig(ServoIndex servo, const ServoConfig& config);
    
    // Individual servo control
    void setServoAngle(ServoIndex servo, float angle);
    float getServoAngle(ServoIndex servo) const;
    void setServoSpeed(ServoIndex servo, float speed); // degrees per second
    
    // Leg-based control
    void setLegPosition(LegID leg, const LegPosition& position);
    LegPosition getLegPosition(LegID leg) const;
    void setLegHome(LegID leg);
    
    // Batch operations
    void setAllHome();
    void setAllAngles(const float angles[NUM_SERVOS]);
    void smoothMoveTo(ServoIndex servo, float target_angle, uint16_t duration_ms);
    
    // Safety and monitoring
    void enableServo(ServoIndex servo, bool enable = true);
    void disableServo(ServoIndex servo);
    void emergencyStop();
    bool isServoEnabled(ServoIndex servo) const;
    
    // Update and maintenance
    void update(); // Call in main loop
    bool isMoving() const;
    void calibrate(); // Auto-calibration routine
    
private:
    Servo servos[NUM_SERVOS];
    ServoConfig configs[NUM_SERVOS];
    float current_angles[NUM_SERVOS];
    float target_angles[NUM_SERVOS];
    bool servo_enabled[NUM_SERVOS];
    bool is_initialized;
    
    // Smooth movement state
    struct SmoothMove {
        bool active = false;
        float start_angle = 0.0f;
        float end_angle = 0.0f;
        uint32_t start_time = 0;
        uint32_t duration = 0;
    } smooth_moves[NUM_SERVOS];
    
    // Helper methods
    ServoIndex getLegServo(LegID leg, uint8_t joint) const;
    float constrainAngle(ServoIndex servo, float angle) const;
    uint16_t angleToPulse(ServoIndex servo, float angle) const;
    void updateSmoothMovements();
};

} // namespace MotionControl