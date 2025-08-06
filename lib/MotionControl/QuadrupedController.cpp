/**
 * @file QuadrupedController.cpp
 * @brief Implementation of high-level quadruped motion control
 * @author Advanced Quadruped Team
 * @date 2024
 */

#include "QuadrupedController.h"
#include "ServoController.h"
#include <math.h>

namespace MotionControl {

// Hardware pin assignments (from Config::Pins::SERVO_BASE in main.cpp)
const uint8_t SERVO_PINS[NUM_SERVOS] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};

class QuadrupedController::Impl {
public:
    ServoController servo_controller;
    
    // Gait timing and patterns
    struct GaitState {
        uint32_t cycle_start_time = 0;
        uint32_t cycle_duration = 2000; // 2 seconds per cycle
        float phase_offset[NUM_LEGS] = {0.0f, 0.5f, 0.5f, 0.0f}; // Trot pattern
        bool is_swing_phase[NUM_LEGS] = {false};
    } gait_state;
    
    // Basic movement state
    Vector3D current_velocity{0, 0, 0};
    Vector3D target_velocity{0, 0, 0};
    float movement_speed = 0.0f;
    bool is_moving = false;
};

QuadrupedController::QuadrupedController() 
    : current_gait(GaitType::TROT)
    , current_mode(MovementMode::MANUAL)
    , is_initialized(false)
    , emergency_stop_active(false)
    , last_update_time(0)
    , max_speed(1.0f)
    , step_height(30.0f)
    , body_height(80.0f)
{
    pImpl = new Impl();
    
    // Initialize robot state
    robot_state.position = Vector3D(0, 0, body_height);
    robot_state.rotation = Vector3D(0, 0, 0);
    robot_state.velocity = Vector3D(0, 0, 0);
    robot_state.angular_velocity = Vector3D(0, 0, 0);
    robot_state.is_stable = false;
    robot_state.battery_voltage = 0.0f;
    robot_state.timestamp = 0;
}

QuadrupedController::~QuadrupedController() {
    if (pImpl) {
        delete pImpl;
        pImpl = nullptr;
    }
}

bool QuadrupedController::initialize() {
    // Initialize servo controller
    if (!pImpl->servo_controller.initialize(SERVO_PINS)) {
        return false;
    }
    
    // Configure servo limits and home positions for quadruped
    configureServoLimits();
    
    // Move to initial standing position
    standingPosition();
    
    robot_state.is_stable = true;
    is_initialized = true;
    last_update_time = millis();
    
    return true;
}

void QuadrupedController::setGait(GaitType gait) {
    current_gait = gait;
    
    // Update gait timing based on type
    switch (gait) {
        case GaitType::WALK:
            pImpl->gait_state.cycle_duration = 4000; // Slow walk
            // Walk: one leg at a time
            pImpl->gait_state.phase_offset[0] = 0.0f;   // FL
            pImpl->gait_state.phase_offset[1] = 0.5f;   // FR  
            pImpl->gait_state.phase_offset[2] = 0.75f;  // RL
            pImpl->gait_state.phase_offset[3] = 0.25f;  // RR
            break;
            
        case GaitType::TROT:
            pImpl->gait_state.cycle_duration = 2000; // Medium trot
            // Trot: diagonal legs together
            pImpl->gait_state.phase_offset[0] = 0.0f;   // FL
            pImpl->gait_state.phase_offset[1] = 0.5f;   // FR
            pImpl->gait_state.phase_offset[2] = 0.5f;   // RL  
            pImpl->gait_state.phase_offset[3] = 0.0f;   // RR
            break;
            
        case GaitType::BOUND:
            pImpl->gait_state.cycle_duration = 1500; // Fast bound
            // Bound: front and rear pairs
            pImpl->gait_state.phase_offset[0] = 0.0f;   // FL
            pImpl->gait_state.phase_offset[1] = 0.0f;   // FR
            pImpl->gait_state.phase_offset[2] = 0.5f;   // RL
            pImpl->gait_state.phase_offset[3] = 0.5f;   // RR
            break;
            
        default:
            // Default to trot
            setGait(GaitType::TROT);
            break;
    }
}

void QuadrupedController::setMode(MovementMode mode) {
    current_mode = mode;
}

void QuadrupedController::moveForward(float speed) {
    if (!is_initialized || emergency_stop_active) return;
    
    speed = constrain(speed, 0.0f, max_speed);
    pImpl->target_velocity = Vector3D(speed, 0, 0);
    pImpl->is_moving = (speed > 0.01f);
}

void QuadrupedController::moveBackward(float speed) {
    if (!is_initialized || emergency_stop_active) return;
    
    speed = constrain(speed, 0.0f, max_speed);
    pImpl->target_velocity = Vector3D(-speed, 0, 0);
    pImpl->is_moving = (speed > 0.01f);
}

void QuadrupedController::turnLeft(float speed) {
    if (!is_initialized || emergency_stop_active) return;
    
    speed = constrain(speed, 0.0f, max_speed);
    pImpl->target_velocity = Vector3D(0, 0, speed); // Angular velocity around Z
    pImpl->is_moving = (speed > 0.01f);
}

void QuadrupedController::turnRight(float speed) {
    if (!is_initialized || emergency_stop_active) return;
    
    speed = constrain(speed, 0.0f, max_speed);
    pImpl->target_velocity = Vector3D(0, 0, -speed); // Angular velocity around Z
    pImpl->is_moving = (speed > 0.01f);
}

void QuadrupedController::stop() {
    pImpl->target_velocity = Vector3D(0, 0, 0);
    pImpl->is_moving = false;
    
    // Return to standing position
    standingPosition();
}

void QuadrupedController::update() {
    if (!is_initialized) return;
    
    uint32_t now = millis();
    float dt = (now - last_update_time) / 1000.0f; // Delta time in seconds
    
    // Update servo controller
    pImpl->servo_controller.update();
    
    // Update gait if moving
    if (pImpl->is_moving) {
        updateGait();
    }
    
    // Update robot state
    robot_state.velocity = pImpl->current_velocity;
    robot_state.timestamp = now;
    
    // Safety monitoring
    checkSafety();
    
    last_update_time = now;
}

void QuadrupedController::emergencyStop() {
    emergency_stop_active = true;
    pImpl->is_moving = false;
    pImpl->target_velocity = Vector3D(0, 0, 0);
    pImpl->current_velocity = Vector3D(0, 0, 0);
    
    // Stop all servo movements
    pImpl->servo_controller.emergencyStop();
    
    robot_state.is_stable = false;
}

RobotState QuadrupedController::getCurrentState() const {
    return robot_state;
}

bool QuadrupedController::isStable() const {
    return robot_state.is_stable && !emergency_stop_active;
}

bool QuadrupedController::isMoving() const {
    return pImpl->is_moving || pImpl->servo_controller.isMoving();
}

void QuadrupedController::setMaxSpeed(float max_speed) {
    this->max_speed = constrain(max_speed, 0.1f, 2.0f);
}

void QuadrupedController::setStepHeight(float height) {
    this->step_height = constrain(height, 10.0f, 50.0f);
}

void QuadrupedController::setBodyHeight(float height) {
    this->body_height = constrain(height, 60.0f, 120.0f);
    robot_state.position.z = height;
}

// Private implementation methods

void QuadrupedController::configureServoLimits() {
    // Configure each servo's range and home position
    // These values may need adjustment based on your specific hardware
    
    for (uint8_t leg = 0; leg < NUM_LEGS; leg++) {
        // Hip servos (forward/backward movement)
        ServoConfig hip_config{
            .pin = SERVO_PINS[leg * 3 + 0],
            .min_pulse = 500,
            .max_pulse = 2500, 
            .min_angle = 45.0f,   // -45 degrees from center
            .max_angle = 135.0f,  // +45 degrees from center
            .home_angle = 90.0f,  // Center position
            .inverted = (leg == 1 || leg == 3) // Invert right side legs
        };
        pImpl->servo_controller.setServoConfig(static_cast<ServoIndex>(leg * 3 + 0), hip_config);
        
        // Shoulder servos (up/down movement)
        ServoConfig shoulder_config{
            .pin = SERVO_PINS[leg * 3 + 1],
            .min_pulse = 500,
            .max_pulse = 2500,
            .min_angle = 30.0f,   // Highest position
            .max_angle = 150.0f,  // Lowest position
            .home_angle = 90.0f,  // Standing position
            .inverted = false
        };
        pImpl->servo_controller.setServoConfig(static_cast<ServoIndex>(leg * 3 + 1), shoulder_config);
        
        // Knee servos (extend/retract)
        ServoConfig knee_config{
            .pin = SERVO_PINS[leg * 3 + 2],
            .min_pulse = 500,
            .max_pulse = 2500,
            .min_angle = 30.0f,   // Fully extended
            .max_angle = 150.0f,  // Fully retracted
            .home_angle = 90.0f,  // Standing position
            .inverted = false
        };
        pImpl->servo_controller.setServoConfig(static_cast<ServoIndex>(leg * 3 + 2), knee_config);
    }
}

void QuadrupedController::standingPosition() {
    // Basic standing position - all legs supporting the body
    LegPosition standing{0.0f, 90.0f, 90.0f};
    
    for (uint8_t leg = 0; leg < NUM_LEGS; leg++) {
        pImpl->servo_controller.setLegPosition(static_cast<LegID>(leg), standing);
    }
    
    robot_state.is_stable = true;
}

void QuadrupedController::updateGait() {
    uint32_t now = millis();
    uint32_t cycle_time = (now - pImpl->gait_state.cycle_start_time) % pImpl->gait_state.cycle_duration;
    float cycle_progress = (float)cycle_time / (float)pImpl->gait_state.cycle_duration;
    
    // Update each leg based on gait pattern
    for (uint8_t leg = 0; leg < NUM_LEGS; leg++) {
        float leg_phase = fmod(cycle_progress + pImpl->gait_state.phase_offset[leg], 1.0f);
        
        LegPosition leg_pos = calculateLegPosition(static_cast<LegID>(leg), leg_phase);
        pImpl->servo_controller.setLegPosition(static_cast<LegID>(leg), leg_pos);
        
        // Track swing phase for stability
        pImpl->gait_state.is_swing_phase[leg] = (leg_phase > 0.5f);
    }
    
    // Smooth velocity transitions
    pImpl->current_velocity.x += (pImpl->target_velocity.x - pImpl->current_velocity.x) * 0.1f;
    pImpl->current_velocity.z += (pImpl->target_velocity.z - pImpl->current_velocity.z) * 0.1f;
}

LegPosition QuadrupedController::calculateLegPosition(LegID leg, float phase) {
    LegPosition pos;
    
    // Basic gait pattern - simplified walking motion
    if (phase < 0.5f) {
        // Stance phase - leg on ground, moving backward relative to body
        float stance_progress = phase * 2.0f; // 0 to 1
        
        pos.hip = pImpl->current_velocity.x * 20.0f * (0.5f - stance_progress); // Forward/backward
        pos.shoulder = 90.0f; // Standing height
        pos.knee = 90.0f;
    } else {
        // Swing phase - leg in air, moving forward
        float swing_progress = (phase - 0.5f) * 2.0f; // 0 to 1
        
        pos.hip = pImpl->current_velocity.x * 20.0f * (swing_progress - 0.5f);
        pos.shoulder = 90.0f - step_height * sin(swing_progress * PI); // Lift leg
        pos.knee = 90.0f - step_height * 0.5f * sin(swing_progress * PI);
    }
    
    // Add turning motion
    if (abs(pImpl->current_velocity.z) > 0.01f) {
        float turn_offset = pImpl->current_velocity.z * 15.0f;
        if (leg == 0 || leg == 2) { // Left legs
            pos.hip += turn_offset;
        } else { // Right legs  
            pos.hip -= turn_offset;
        }
    }
    
    return pos;
}

void QuadrupedController::checkSafety() {
    // Basic safety checks
    robot_state.is_stable = true; // Will implement more sophisticated checks later
    
    // Check if any critical servos are disabled
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        if (!pImpl->servo_controller.isServoEnabled(static_cast<ServoIndex>(i))) {
            robot_state.is_stable = false;
            break;
        }
    }
}

} // namespace MotionControl