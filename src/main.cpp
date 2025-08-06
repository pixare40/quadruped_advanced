/**
 * @file main.cpp
 * @brief Advanced Quadruped Robot Firmware - Main Application
 * @author Your Name
 * @date 2024
 * @version 1.0.0
 * 
 * This is the main entry point for the advanced quadruped robot firmware.
 * Built on the Freenove Quadruped Robot platform with modern C++ practices.
 * 
 * Hardware: ATmega2560 + ESP8266 WiFi Module + 12 Servo Motors
 * 
 * @copyright MIT License
 */

#include <Arduino.h>
#include <Servo.h>
#include <EEPROM.h>
// Use original FNQR library for testing
#include "FNQR.h"

// Debug configuration
#ifdef DEBUG_BUILD
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
  // Arduino AVR doesn't have printf, so use basic print functions
  #define DEBUG_PRINTF(...) do { \
    char buffer[128]; \
    snprintf(buffer, sizeof(buffer), __VA_ARGS__); \
    Serial.print(buffer); \
  } while(0)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x) 
  #define DEBUG_PRINTF(...)
#endif

// System configuration
namespace Config {
    constexpr uint32_t SERIAL_BAUD = 115200;
    constexpr uint32_t CONTROL_LOOP_FREQ = 50; // Hz
    constexpr uint32_t CONTROL_LOOP_PERIOD = 1000 / CONTROL_LOOP_FREQ; // ms
    
    // Hardware pins - Freenove Quadruped Robot pin mapping
    namespace Pins {
        // Servos use pins 22-39 on the Freenove controller board
        constexpr uint8_t SERVO_BASE[] = {22, 23, 24, 25, 26, 27, 34, 35, 36, 37, 38, 39};
        constexpr uint8_t VOLTAGE_MONITOR = A0;
        constexpr uint8_t STATUS_LED = LED_BUILTIN;
    }
    
    // Timing
    namespace Timing {
        constexpr uint32_t HEARTBEAT_INTERVAL = 1000; // ms
        constexpr uint32_t VOLTAGE_CHECK_INTERVAL = 2000; // ms
        constexpr uint32_t COMM_TIMEOUT = 5000; // ms
    }
}

// Global system state
struct SystemState {
    bool is_initialized = false;
    bool is_active = false;
    bool wifi_connected = false;
    bool rf24_connected = false;
    uint32_t last_heartbeat = 0;
    uint32_t last_voltage_check = 0;
    float battery_voltage = 0.0f;
    uint32_t loop_count = 0;
    uint32_t last_comm_time = 0;
} g_system;

// Use original FNQR robot for testing
FNQR robot;

// Performance monitoring
struct PerformanceMetrics {
    uint32_t loop_time_max = 0;
    uint32_t loop_time_avg = 0;
    uint32_t loop_time_min = UINT32_MAX;
    uint32_t memory_free = 0;
} g_metrics;

/**
 * @brief Calculate free RAM (useful for debugging memory usage)
 * @return Available RAM in bytes
 */
uint32_t getFreeRAM() {
    extern int __heap_start, *__brkval;
    int v;
    return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

/**
 * @brief Read battery voltage from ADC
 * @return Battery voltage in volts
 */
float readBatteryVoltage() {
    // Read ADC value (0-1023)
    int adc_value = analogRead(Config::Pins::VOLTAGE_MONITOR);
    
    // Convert to voltage (assuming voltage divider)
    // This needs to be calibrated based on your hardware
    float voltage = (adc_value / 1023.0f) * 5.0f * 2.0f; // Assuming 2:1 voltage divider
    
    return voltage;
}

/**
 * @brief Update system metrics and diagnostics
 */
void updateDiagnostics() {
    static uint32_t last_update = 0;
    uint32_t now = millis();
    
    if (now - last_update >= Config::Timing::HEARTBEAT_INTERVAL) {
        last_update = now;
        
        // Update performance metrics
        g_metrics.memory_free = getFreeRAM();
        
        // Battery voltage check (reduced frequency during calibration)
        if (now - g_system.last_voltage_check >= (Config::Timing::VOLTAGE_CHECK_INTERVAL * 5)) {
            g_system.battery_voltage = readBatteryVoltage();
            g_system.last_voltage_check = now;
            
            // DEBUG_PRINTF("Battery: %.2fV, Free RAM: %lu bytes\n", 
            //            g_system.battery_voltage, g_metrics.memory_free);
        }
        
        // Heartbeat LED
        digitalWrite(Config::Pins::STATUS_LED, !digitalRead(Config::Pins::STATUS_LED));
        g_system.last_heartbeat = now;
        
        // DEBUG_PRINTF("Loop #%lu - Max: %lums, Avg: %lums, Min: %lums\n",
        //            g_system.loop_count, g_metrics.loop_time_max, 
        //            g_metrics.loop_time_avg, g_metrics.loop_time_min);
    }
}

/**
 * @brief Initialize hardware and system components
 */
void initializeSystem() {
    // Initialize serial communication
    Serial.begin(Config::SERIAL_BAUD);
    while (!Serial && millis() < 2000) {
        // Wait for serial connection (with timeout)
    }
    
    DEBUG_PRINTLN("=================================");
    DEBUG_PRINTLN("🤖 Advanced Quadruped Robot v" QUADRUPED_VERSION);
    DEBUG_PRINTLN("=================================");
    DEBUG_PRINTLN("Initializing system...");
    
    // Initialize status LED
    pinMode(Config::Pins::STATUS_LED, OUTPUT);
    digitalWrite(Config::Pins::STATUS_LED, HIGH);
    
    // Initialize voltage monitoring
    pinMode(Config::Pins::VOLTAGE_MONITOR, INPUT);
    
    // Initialize robot using original FNQR library
    DEBUG_PRINTLN("Initializing FNQR robot...");
    robot.Start(false); // Start without communication for basic testing
    DEBUG_PRINTLN("✅ FNQR robot initialized");
    
    // Test basic movements
    DEBUG_PRINTLN("Setting robot to install state for servo test...");
    // This should move all servos to 90 degrees
    // robot.InstallState() is not directly accessible, let's try basic movement
    
    // TODO: Initialize communication modules (WiFi, RF24)
    // TODO: Initialize sensor interfaces  
    // TODO: Load configuration from EEPROM
    
    g_system.battery_voltage = readBatteryVoltage();
    g_system.is_initialized = true;
    
    // Activate robot after a short delay
    delay(2000);
    g_system.is_active = true;
    
    DEBUG_PRINTLN("✅ System initialization complete!");
    DEBUG_PRINTF("📊 Initial battery voltage: %.2fV\n", g_system.battery_voltage);
    DEBUG_PRINTF("🧠 Free RAM: %lu bytes\n", getFreeRAM());
    
    // Startup sequence indicator
    for (int i = 0; i < 3; i++) {
        digitalWrite(Config::Pins::STATUS_LED, LOW);
        delay(150);
        digitalWrite(Config::Pins::STATUS_LED, HIGH);
        delay(150);
    }
}

/**
 * @brief Main control loop - runs at fixed frequency
 */
void controlLoop() {
    static uint32_t last_control_time = 0;
    uint32_t now = millis();
    
    // Fixed frequency control loop
    if (now - last_control_time >= Config::CONTROL_LOOP_PERIOD) {
        uint32_t loop_start = micros();
        
        // Update FNQR robot
        // Note: The FNQR robot handles its own timing internally
        
        // TODO: Process sensor data
        // TODO: Execute behavior tree  
        // TODO: Update communication
        // TODO: Safety monitoring
        
        uint32_t loop_end = micros();
        uint32_t loop_time = loop_end - loop_start;
        
        // Update performance metrics
        if (loop_time > g_metrics.loop_time_max) g_metrics.loop_time_max = loop_time;
        if (loop_time < g_metrics.loop_time_min) g_metrics.loop_time_min = loop_time;
        g_metrics.loop_time_avg = (g_metrics.loop_time_avg + loop_time) / 2;
        
        last_control_time = now;
        g_system.loop_count++;
    }
}

/**
 * @brief Arduino setup function - called once at startup
 */
void setup() {
    // Disable watchdog timer if it was enabled by bootloader
    // wdt_disable(); // Commented out for now, add #include <avr/wdt.h> if needed
    
    // Initialize all systems
    initializeSystem();
    
    // Enable global interrupts
    sei();
    
    DEBUG_PRINTLN("🚀 Entering main loop...");
}

/**
 * @brief Arduino loop function - main execution loop
 */
void loop() {
    // Check if system is properly initialized
    if (!g_system.is_initialized) {
        DEBUG_PRINTLN("❌ System not initialized! Halting.");
        while (true) {
            delay(1000);
        }
    }
    
    // Main control loop (fixed frequency)
    controlLoop();
    
    // System diagnostics and monitoring
    updateDiagnostics();
    
    // Communication handling
    // Simple demo sequence - remove this when implementing proper control
    static uint32_t demo_timer = 0;
    static uint8_t demo_state = 0;
    
    // CALIBRATION MODE: Uncomment this section for calibration, comment out demo
    
    // === CALIBRATION SEQUENCE ===
    // This follows the exact Freenove calibration process
    static bool calibration_complete = false;
    
    if (g_system.is_active && !calibration_complete && millis() - demo_timer > 8000) {
        demo_timer = millis();
        
        switch(demo_state) {
            case 0:
                Serial.println();
                Serial.println("=== ASSEMBLY MODE ===");
                Serial.println("Moving all servos to install position (90 degrees)...");
                robot.InstallState(); // All servos to 90°
                Serial.println("Servos are now at 90 degrees and will stay there.");
                Serial.println("NOW: Position legs horizontally and tighten screws");
                Serial.println("(As shown in tutorial - keep power ON during assembly)");
                Serial.println("You have 3 minutes (180 seconds) for assembly...");
                
                // Give 3 minutes for assembly
                for(int i = 180; i > 0; i--) {
                    if (i % 30 == 0 || i <= 10) {
                        Serial.print("Assembly time remaining: ");
                        Serial.print(i);
                        Serial.println(" seconds");
                    }
                    delay(1000);
                }
                Serial.println("Assembly time complete! Starting calibration...");
                break;
            case 1:
                Serial.println();
                Serial.println("=== CALIBRATION STEP 2 ===");
                Serial.println("Moving legs to calibration position...");
                robot.CalibrateState(); // Move legs to calibration position
                delay(3000); // Wait for movement to complete
                Serial.println("Moved to calibration position - COMPLETE");
                break;
            case 2:
                Serial.println();
                Serial.println("=== CALIBRATION STEP 3 ===");
                Serial.println("Calculating servo offsets...");
                robot.CalibrateServos(); // Calculate offsets and save to EEPROM
                Serial.println("Calibration offsets saved to EEPROM - COMPLETE");
                delay(2000);
                break;
            case 3:
                Serial.println();
                Serial.println("=== CALIBRATION STEP 4 ===");
                Serial.println("Testing calibrated positions...");
                robot.CalibrateVerify(); // Verify calibration
                Serial.println("Calibration verification complete!");
                Serial.println();
                Serial.println("*** CALIBRATION FINISHED ***");
                Serial.println("Robot is now calibrated and ready to use!");
                calibration_complete = true;
                break;
            default:
                calibration_complete = true;
                break;
        }
        demo_state++;
    }
    
    
    // // === NORMAL DEMO MODE ===
    // if (g_system.is_active && millis() - demo_timer > 3000) {
    //     demo_timer = millis();
        
    //     switch(demo_state) {
    //         case 0:
    //             DEBUG_PRINTLN("Demo: Testing ActiveMode");
    //             robot.ActiveMode();
    //             break;
    //         case 1:
    //             DEBUG_PRINTLN("Demo: Testing SleepMode");
    //             robot.SleepMode();
    //             break;
    //         case 2:
    //             DEBUG_PRINTLN("Demo: Testing CrawlForward");
    //             robot.CrawlForward();
    //             delay(1000);
    //             break;
    //         case 3:
    //             DEBUG_PRINTLN("Demo: Testing TurnLeft");
    //             robot.TurnLeft();
    //             delay(1000);
    //             break;
    //         case 4:
    //             DEBUG_PRINTLN("Demo: Testing CrawlBackward");
    //             robot.CrawlBackward();
    //             delay(1000);
    //             break;
    //         default:
    //             DEBUG_PRINTLN("Demo: Return to ActiveMode");
    //             robot.ActiveMode();
    //             demo_state = -1;
    //             break;
    //     }
    //     demo_state++;
    // }
    
    // TODO: Process incoming commands
    // TODO: Send telemetry data
    
    // Safety checks
    // TODO: Monitor battery voltage
    // TODO: Check servo temperatures
    // TODO: Validate sensor readings
    
    // Small delay to prevent overwhelming the processor
    delay(1);
}

/**
 * @brief Emergency stop function - can be called from interrupts
 */
void emergencyStop() {
    DEBUG_PRINTLN("🚨 EMERGENCY STOP ACTIVATED!");
    
    // Emergency stop using FNQR
    robot.SleepMode();
    DEBUG_PRINTLN("Robot stopped using SleepMode");
    g_system.is_active = false;
    
    // TODO: Send emergency stop signal to all subsystems
    // TODO: Save critical data to EEPROM
    
    // Flash LED rapidly to indicate emergency state
    while (true) {
        digitalWrite(Config::Pins::STATUS_LED, HIGH);
        delay(100);
        digitalWrite(Config::Pins::STATUS_LED, LOW);
        delay(100);
    }
}