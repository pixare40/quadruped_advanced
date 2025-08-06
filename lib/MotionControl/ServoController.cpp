/**
 * @file ServoController.cpp
 * @brief Implementation of low-level servo control
 * @author Advanced Quadruped Team
 * @date 2024
 */

#include "ServoController.h"

namespace MotionControl {

ServoController::ServoController() : is_initialized(false) {
    // Initialize arrays
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        current_angles[i] = 90.0f;
        target_angles[i] = 90.0f;
        servo_enabled[i] = false;
        smooth_moves[i] = SmoothMove();
    }
}

bool ServoController::initialize(const uint8_t servo_pins[NUM_SERVOS]) {
    // Set default configurations
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        configs[i] = ServoConfig{
            .pin = servo_pins[i],
            .min_pulse = 500,
            .max_pulse = 2500,
            .min_angle = 0.0f,
            .max_angle = 180.0f,
            .home_angle = 90.0f,
            .inverted = false
        };
        
        // Attach servo to pin
        servos[i].attach(servo_pins[i], configs[i].min_pulse, configs[i].max_pulse);
        current_angles[i] = configs[i].home_angle;
        target_angles[i] = configs[i].home_angle;
    }
    
    // Move all servos to home position
    setAllHome();
    delay(1000); // Allow time to reach home position
    
    is_initialized = true;
    return true;
}

void ServoController::setServoConfig(ServoIndex servo, const ServoConfig& config) {
    uint8_t idx = static_cast<uint8_t>(servo);
    if (idx >= NUM_SERVOS) return;
    
    configs[idx] = config;
    
    // Re-attach servo with new pulse range
    servos[idx].detach();
    servos[idx].attach(config.pin, config.min_pulse, config.max_pulse);
}

void ServoController::setServoAngle(ServoIndex servo, float angle) {
    uint8_t idx = static_cast<uint8_t>(servo);
    if (idx >= NUM_SERVOS || !servo_enabled[idx]) return;
    
    // Constrain angle to valid range
    angle = constrainAngle(servo, angle);
    
    // Apply inversion if configured
    if (configs[idx].inverted) {
        angle = configs[idx].max_angle - angle + configs[idx].min_angle;
    }
    
    // Update servo position
    uint16_t pulse = angleToPulse(servo, angle);
    servos[idx].writeMicroseconds(pulse);
    
    current_angles[idx] = angle;
    target_angles[idx] = angle;
}

float ServoController::getServoAngle(ServoIndex servo) const {
    uint8_t idx = static_cast<uint8_t>(servo);
    if (idx >= NUM_SERVOS) return 0.0f;
    return current_angles[idx];
}

void ServoController::setLegPosition(LegID leg, const LegPosition& position) {
    uint8_t leg_idx = static_cast<uint8_t>(leg);
    if (leg_idx >= NUM_LEGS) return;
    
    // Calculate servo indices for this leg
    ServoIndex hip = static_cast<ServoIndex>(leg_idx * SERVOS_PER_LEG + 0);
    ServoIndex shoulder = static_cast<ServoIndex>(leg_idx * SERVOS_PER_LEG + 1);
    ServoIndex knee = static_cast<ServoIndex>(leg_idx * SERVOS_PER_LEG + 2);
    
    // Set individual joint angles
    setServoAngle(hip, position.hip + 90.0f); // Convert from -90:90 to 0:180
    setServoAngle(shoulder, position.shoulder);
    setServoAngle(knee, position.knee);
}

LegPosition ServoController::getLegPosition(LegID leg) const {
    uint8_t leg_idx = static_cast<uint8_t>(leg);
    if (leg_idx >= NUM_LEGS) return LegPosition{90.0f, 90.0f, 90.0f};
    
    ServoIndex hip = static_cast<ServoIndex>(leg_idx * SERVOS_PER_LEG + 0);
    ServoIndex shoulder = static_cast<ServoIndex>(leg_idx * SERVOS_PER_LEG + 1);
    ServoIndex knee = static_cast<ServoIndex>(leg_idx * SERVOS_PER_LEG + 2);
    
    return LegPosition{
        .hip = getServoAngle(hip) - 90.0f, // Convert from 0:180 to -90:90
        .shoulder = getServoAngle(shoulder),
        .knee = getServoAngle(knee)
    };
}

void ServoController::setLegHome(LegID leg) {
    LegPosition home{0.0f, 90.0f, 90.0f}; // Neutral standing position
    setLegPosition(leg, home);
}

void ServoController::setAllHome() {
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        enableServo(static_cast<ServoIndex>(i), true);
        setServoAngle(static_cast<ServoIndex>(i), configs[i].home_angle);
    }
}

void ServoController::setAllAngles(const float angles[NUM_SERVOS]) {
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        setServoAngle(static_cast<ServoIndex>(i), angles[i]);
    }
}

void ServoController::smoothMoveTo(ServoIndex servo, float target_angle, uint16_t duration_ms) {
    uint8_t idx = static_cast<uint8_t>(servo);
    if (idx >= NUM_SERVOS) return;
    
    target_angle = constrainAngle(servo, target_angle);
    
    smooth_moves[idx] = SmoothMove{
        .active = true,
        .start_angle = current_angles[idx],
        .end_angle = target_angle,
        .start_time = millis(),
        .duration = duration_ms
    };
    
    target_angles[idx] = target_angle;
}

void ServoController::enableServo(ServoIndex servo, bool enable) {
    uint8_t idx = static_cast<uint8_t>(servo);
    if (idx >= NUM_SERVOS) return;
    
    servo_enabled[idx] = enable;
    
    if (enable) {
        // Ensure servo is attached
        if (!servos[idx].attached()) {
            servos[idx].attach(configs[idx].pin, configs[idx].min_pulse, configs[idx].max_pulse);
        }
    } else {
        // Detach servo to save power
        servos[idx].detach();
    }
}

void ServoController::disableServo(ServoIndex servo) {
    enableServo(servo, false);
}

void ServoController::emergencyStop() {
    // Disable all servos immediately
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        disableServo(static_cast<ServoIndex>(i));
        smooth_moves[i].active = false;
    }
}

bool ServoController::isServoEnabled(ServoIndex servo) const {
    uint8_t idx = static_cast<uint8_t>(servo);
    if (idx >= NUM_SERVOS) return false;
    return servo_enabled[idx];
}

void ServoController::update() {
    if (!is_initialized) return;
    
    updateSmoothMovements();
}

bool ServoController::isMoving() const {
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        if (smooth_moves[i].active) return true;
        if (abs(current_angles[i] - target_angles[i]) > 1.0f) return true;
    }
    return false;
}

void ServoController::calibrate() {
    // Basic calibration: move all servos through their range
    // This helps identify any mechanical issues
    
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        ServoIndex servo = static_cast<ServoIndex>(i);
        
        // Move to minimum
        setServoAngle(servo, configs[i].min_angle);
        delay(500);
        
        // Move to maximum  
        setServoAngle(servo, configs[i].max_angle);
        delay(500);
        
        // Return to home
        setServoAngle(servo, configs[i].home_angle);
        delay(500);
    }
}

// Private helper methods

ServoIndex ServoController::getLegServo(LegID leg, uint8_t joint) const {
    uint8_t leg_idx = static_cast<uint8_t>(leg);
    return static_cast<ServoIndex>(leg_idx * SERVOS_PER_LEG + joint);
}

float ServoController::constrainAngle(ServoIndex servo, float angle) const {
    uint8_t idx = static_cast<uint8_t>(servo);
    if (idx >= NUM_SERVOS) return angle;
    
    return constrain(angle, configs[idx].min_angle, configs[idx].max_angle);
}

uint16_t ServoController::angleToPulse(ServoIndex servo, float angle) const {
    uint8_t idx = static_cast<uint8_t>(servo);
    if (idx >= NUM_SERVOS) return 1500; // Default center pulse
    
    // Map angle to pulse width
    float pulse_range = configs[idx].max_pulse - configs[idx].min_pulse;
    float angle_range = configs[idx].max_angle - configs[idx].min_angle;
    float normalized = (angle - configs[idx].min_angle) / angle_range;
    
    return configs[idx].min_pulse + (uint16_t)(normalized * pulse_range);
}

void ServoController::updateSmoothMovements() {
    uint32_t now = millis();
    
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        if (!smooth_moves[i].active) continue;
        
        uint32_t elapsed = now - smooth_moves[i].start_time;
        
        if (elapsed >= smooth_moves[i].duration) {
            // Movement complete
            setServoAngle(static_cast<ServoIndex>(i), smooth_moves[i].end_angle);
            smooth_moves[i].active = false;
        } else {
            // Calculate intermediate position
            float progress = (float)elapsed / (float)smooth_moves[i].duration;
            float current_angle = smooth_moves[i].start_angle + 
                                (smooth_moves[i].end_angle - smooth_moves[i].start_angle) * progress;
            
            setServoAngle(static_cast<ServoIndex>(i), current_angle);
        }
    }
}

} // namespace MotionControl