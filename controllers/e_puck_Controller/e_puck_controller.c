#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <stdio.h>

#define MAX_SPEED 5

int main(int argc, char **argv) {
    wb_robot_init();

    WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
    WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
    WbDeviceTag front_sensor = wb_robot_get_device("ps0");
    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_position(right_motor, INFINITY);

    wb_distance_sensor_enable(front_sensor, 1000);

    while (wb_robot_step(64) != -1) {
        double front_distance = wb_distance_sensor_get_value(front_sensor);
        printf("Front Distance: %lf\n", front_distance);

        if (front_distance > 50) {  // 1 meter = 1000 millimeters
            wb_motor_set_velocity(left_motor, MAX_SPEED);
            wb_motor_set_velocity(right_motor, MAX_SPEED);
        } else {
            wb_motor_set_velocity(left_motor, 0);
            wb_motor_set_velocity(right_motor, 0);
        }
    }

    wb_robot_cleanup();

    return 0;
}
