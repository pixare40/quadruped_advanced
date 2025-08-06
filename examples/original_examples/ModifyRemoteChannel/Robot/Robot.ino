/*
 * Sketch     Modify wireless communication channel of the robot
 * Platform   Freenove Quadruped Robot (Compatible with Arduino/Genuino Mega 2560)
 * Brief      This sketch is used to show how to modify wireless communication channel between robot 
 *            and remote when using default function.
 *            The remote control should set the same channel to control robot.
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
  // Set wireless communication channel
  // Call this function before robot.Start()
  // The channel can be set to 0~125, default is 125
  robot.SetRemoteChannel(100);
  // Start Freenove Quadruped Robot with default function
  robot.Start(true);
}

void loop() {
  // Update Freenove Quadruped Robot
  robot.Update();
}
