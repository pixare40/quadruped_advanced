/**
 * @file QuadrupedController.h
 * @brief High-level quadruped robot motion controller
 * @author Advanced Quadruped Team
 * @date 2024
 */

#pragma once
#include <Arduino.h>

namespace MotionControl {

enum class GaitType {
    WALK,
    TROT,
    BOUND,
    GALLOP,
    CUSTOM
};

enum class MovementMode {
    MANUAL,           // Direct control
    AUTONOMOUS,       // AI-driven movement
    REMOTE_CONTROL,   // RF24/WiFi control
    CALIBRATION       // Calibration mode
};

struct Vector3D {
    float x, y, z;
    Vector3D(float x = 0, float y = 0, float z = 0) : x(x), y(y), z(z) {}
};

struct RobotState {
    Vector3D position;
    Vector3D rotation;
    Vector3D velocity;
    Vector3D angular_velocity;
    bool is_stable;
    float battery_voltage;
    uint32_t timestamp;
};

// Forward declarations
struct LegPosition;
enum class LegID : uint8_t;

class QuadrupedController {
public:
    QuadrupedController();
    ~QuadrupedController();
    
    // Initialization and configuration
    bool initialize();
    void setGait(GaitType gait);
    void setMode(MovementMode mode);
    
    // Basic movement commands
    void moveForward(float speed = 0.5f);
    void moveBackward(float speed = 0.5f);
    void turnLeft(float speed = 0.5f);
    void turnRight(float speed = 0.5f);
    void stop();
    
    // Advanced movement
    void setVelocity(const Vector3D& linear, const Vector3D& angular);
    void moveToPosition(const Vector3D& target_position);
    void rotateToOrientation(const Vector3D& target_rotation);
    
    // Body manipulation
    void adjustBody(const Vector3D& position, const Vector3D& rotation);
    void adjustLeg(uint8_t leg_index, const Vector3D& position);
    
    // System control
    void update();
    void emergencyStop();
    void calibrate();
    
    // State queries
    RobotState getCurrentState() const;
    bool isStable() const;
    bool isMoving() const;
    
    // Configuration
    void setMaxSpeed(float max_speed);
    void setStepHeight(float height);
    void setBodyHeight(float height);
    
private:
    GaitType current_gait;
    MovementMode current_mode;
    RobotState robot_state;
    
    // Internal state
    bool is_initialized;
    bool emergency_stop_active;
    uint32_t last_update_time;
    
    // Movement parameters
    float max_speed;
    float step_height;
    float body_height;
    
    // Private implementation (pImpl pattern)
    class Impl;
    Impl* pImpl;
    
    // Private methods
    void configureServoLimits();
    void standingPosition();
    void updateGait();
    LegPosition calculateLegPosition(LegID leg, float phase);
    void checkSafety();
};

} // namespace MotionControl