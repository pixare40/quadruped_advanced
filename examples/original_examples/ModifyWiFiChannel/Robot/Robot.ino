/*
 * Sketch     Modify Wi-Fi hotspot channel of the robot
 * Platform   Freenove Quadruped Robot (Compatible with Arduino/Genuino Mega 2560)
 * Brief      This sketch is used to show how to modify Wi-Fi hotspot channel of the robot when 
 *            using default function.
 *            When there are a lot of Wi-Fi signals around, you may not be able to connect to 
 *            the robot or the signal is poor. Then you can try to modify the channel.
 * Author     Ethan Pan @ Freenove (support@freenove.com)
 * Date       2020/04/24
 * Copyright  Copyright © Freenove (http://www.freenove.com)
 * License    Creative Commons Attribution ShareAlike 3.0
 *            (http://creativecommons.org/licenses/by-sa/3.0/legalcode)
 * -----------------------------------------------------------------------------------------------*/

#ifndef ARDUINO_AVR_MEGA2560
#error Wrong board. Please choose "Arduino/Genuino Mega or Mega 2560"
#endif

// Include FNQR (Freenove Quadruped Robot) library
#include <FNQR.h>

FNQR robot;

void setup() {
  // Set Wi-Fi channel
  // Call this function before robot.Start()
  // The Wi-Fi channel can be set to 1~13, default is 1
  robot.SetWiFiChannel(2);
  // Start Freenove Quadruped Robot with default function
  robot.Start(true);
}

void loop() {
  // Update Freenove Quadruped Robot
  robot.Update();
}
