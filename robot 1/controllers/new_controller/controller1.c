/*
 * File:          new_controller.c
 * Date:
 * Description: Following the wall
 * Author: 
 * Modifications:
 */

#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <stdio.h>
#include <webots/distance_sensor.h>


//WE DECLARATE THE CONSTANT
#define MAX_SPEED 1200
#define TIME_STEP 64

int main(int argc, char **argv) {

    /* necessary to initialize webots stuff */
    wb_robot_init();

    // initialize the distance sensor
    int i;
    WbDeviceTag ps[8];
    char ps_names[8][4] = {"ps0", "ps1", "ps2", "ps3","ps4", "ps5", "ps6", "ps7"};

    for (i=0; i < 8; i++) {
        ps[i] = wb_robot_get_device(ps_names[i]);
        wb_distance_sensor_enable(ps[i], TIME_STEP);
    }


    /* main loop
     * Perform simulation steps of TIME_STEP milliseconds
     * and leave the loop when the simulation is over
     */
    while (wb_robot_step(TIME_STEP) != -1) {

        // read sensors outputs, distance sensor

        double ps_values[8];
        for (i=0; i<8 ; i++)
            ps_values[i] = wb_distance_sensor_get_value(ps[i]);

        // detect obstacles

        bool right_obstacle =
            ps_values[1] > 150.0 ||
            ps_values[0] > 150.0 ||
            ps_values[2] > 150.0;

        bool left_obstacle =
            ps_values[5] > 150.0 ||
            ps_values[6] > 150.0 ||
            ps_values[7] > 150.0;

        bool front_obstacle =
            ps_values[0] > 150.0 &&
            ps_values[7] > 150.0;

        // init speeds
        double left_speed  = MAX_SPEED * 0.5;
        double right_speed = MAX_SPEED * 0.5;

        //TAKE DECISION

        //If the robot is in a corner
        if((left_obstacle) && (front_obstacle)) {
            //turn left
            left_speed  = -MAX_SPEED * 0.8;
        }

        //If the robot is in a corner
        else if((right_obstacle)  && (front_obstacle)) {
            //turn right
            right_speed = -MAX_SPEED * 0.8;
        }

        //If the robot finds a front obstacle(the wall)
        else if(front_obstacle) {
            // turn right
            right_speed = -MAX_SPEED * 0.8;
        }


        //The robot follows the wall
        else if((right_obstacle) || (left_obstacle)) {
            //forward
            left_speed  = MAX_SPEED *0.5;
            right_speed = MAX_SPEED *0.5;
        }

        //Go forward, the robot wants to find the walls
        else {
            left_speed  = MAX_SPEED *0.5;
            right_speed = MAX_SPEED *0.5;
        }

        //We pass the speed that we want
        wb_differential_wheels_set_speed(left_speed, right_speed);
    }

    // cleanup the Webots API
    wb_robot_cleanup();
    return 0; //EXIT_SUCCESS
}

