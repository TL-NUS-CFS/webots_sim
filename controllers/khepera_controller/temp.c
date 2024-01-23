/*
 * Copyright 1996-2023 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <webots/motor.h>
#include <webots/robot.h>

#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>

#include <stdio.h>
#include <stdlib.h>

#define MAX_SPEED 47.6
#define SPEED 2

// #define NUMBER_OF_ULTRASONIC_SENSORS 5
// static const char *ultrasonic_sensors_names[NUMBER_OF_ULTRASONIC_SENSORS] = {
//   "left ultrasonic sensor", "front left ultrasonic sensor", "front ultrasonic sensor", "front right ultrasonic sensor",
//   "right ultrasonic sensor"};

#define NUMBER_OF_INFRARED_SENSORS 4
static const char *infrared_sensors_names[NUMBER_OF_INFRARED_SENSORS] = {
  // turret sensors
  "front infrared sensor","left infrared sensor", "right infrared sensor", "rear infrared sensor"
  };

int main(int argc, char **argv) {
  wb_robot_init();

  int time_step = (int)wb_robot_get_basic_time_step();
  int i;

  // get and enable the camera
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, time_step);

//   // get and enable the ultrasonic sensors
//   WbDeviceTag ultrasonic_sensors[5];
//   for (i = 0; i < 5; ++i) {
//     ultrasonic_sensors[i] = wb_robot_get_device(ultrasonic_sensors_names[i]);
//     wb_distance_sensor_enable(ultrasonic_sensors[i], time_step);
//   }

  // get and enable the infrared sensors
  WbDeviceTag infrared_sensors[NUMBER_OF_INFRARED_SENSORS];
  for (i = 0; i < NUMBER_OF_INFRARED_SENSORS; ++i) {
    infrared_sensors[i] = wb_robot_get_device(infrared_sensors_names[i]);
    wb_distance_sensor_enable(infrared_sensors[i], time_step);
  }

  // get the motors and set target position to infinity (speed control)
  WbDeviceTag left_motor, right_motor;
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  // store the last time a message was displayed
  int last_display_second = 0;

  // main loop
  while (wb_robot_step(time_step) != -1) {
    // display some sensor data every second
    int display_second = (int)wb_robot_get_time();
    if (display_second != last_display_second) {
      last_display_second = display_second;

      printf("time = %d [s]\n", display_second);
      for (i = 0; i < NUMBER_OF_INFRARED_SENSORS; ++i)
        printf("- infrared sensor('%s') = %f [m]\n", infrared_sensors_names[i],
               wb_distance_sensor_get_value(infrared_sensors[i]));
    }

    // simple obstacle avoidance algorithm (wall follow right)
    // based on the front infrared sensor
    if (wb_distance_sensor_get_value(infrared_sensors[0]) < 100){
        wb_motor_set_velocity(left_motor, SPEED);
        wb_motor_set_velocity(right_motor, 0);    
    }
    else{
        wb_motor_set_velocity(left_motor, SPEED);
        wb_motor_set_velocity(right_motor, SPEED);    
    }
  };

  wb_robot_cleanup();

  return EXIT_SUCCESS;
}





    