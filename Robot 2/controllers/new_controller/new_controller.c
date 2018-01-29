#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <stdio.h>
#include <webots/differential_wheels.h>
#include <webots/gps.h>
#include <math.h>
#include <webots/device.h>


// time in [ms] of a simulation step
#define TIME_STEP 64
#define BLACK_MAX 500
#define GS_COUNT 3
#define MAX_SPEED 1000


static WbDeviceTag ground_sensors[GS_COUNT] ,ps[8];
static double gs_val[GS_COUNT],ps_values[8];


void initialize() {

    int i;

    char ps_names[8][4] = {
        "ps0", "ps1", "ps2", "ps3",
        "ps4", "ps5", "ps6", "ps7"
    };

    // initialize devices
    for (i = 0; i < 8 ; i++) {
        ps[i] = wb_robot_get_device(ps_names[i]);
        wb_distance_sensor_enable(ps[i], TIME_STEP);
    }


    // initialise ground sensors
    int n_dev = wb_robot_get_number_of_devices(); // all devices
    for (i=0; i<n_dev; i++) {
        WbDeviceTag dev = wb_robot_get_device_by_index(i);
        // Check to see if device is a ground sensor
        const char *nm = wb_device_get_name(dev);
        // NB Webots8: In the next line you will need change "wb_device_get_type" to "wb_device_get_node_type"
        WbNodeType type = wb_device_get_node_type(dev);
        if(type==WB_NODE_DISTANCE_SENSOR && nm[0]=='g' && nm[1]=='s') {
            int idx = nm[2] - '0';
            if (idx>=0 && idx<GS_COUNT) { // we expect 3 ground sensors
                printf("Initialising %s\n", nm);
                ground_sensors[idx] = wb_robot_get_device(nm);
                wb_distance_sensor_enable(ground_sensors[idx],TIME_STEP);
            }
        }
    }
}


void read_sensors() {
    int i;

    for (i = 0; i < 8 ; i++)
        ps_values[i] = wb_distance_sensor_get_value(ps[i]);

    for (i=0; i<GS_COUNT; i++)
        gs_val[i]=wb_distance_sensor_get_value(ground_sensors[i]);
}








// entry point of the controller
int main(int argc, char **argv) {

    // initialize the Webots API
    wb_robot_init();
    initialize();

    // internal variables
    int turn=0;


    // feedback loop: step simulation until an exit event is received
    while (wb_robot_step(TIME_STEP) != -1) {
        // read sensors outputs

        read_sensors();


        // detect obstacles

        bool right_obstacle =
            ps_values[1] > 100.0 ||
            ps_values[2] > 100.0;
        bool left_obstacle =
            ps_values[5] > 100.0 ||
            ps_values[6] > 100.0 ;
        bool front_obstacle =
            ps_values[0] > 100.0 ||
            ps_values[7] > 100.0;

        double left_speed  = 400;
        double right_speed = 400;


        //when a black line is found it will give a full turn and not just a turn
        if(turn > 0) {
            if(turn < 10) {
                left_speed = -800;
                right_speed = +800;
            }

            else if(turn >=10) {

                left_speed = +800;
                right_speed = -800;

            }

            turn --;
        }


        //If the robot finds something black in the floor(avoid)
        else if (gs_val[0] < BLACK_MAX || gs_val[1] <BLACK_MAX || gs_val[2] < BLACK_MAX) {
            srand(time(NULL));
            int n = rand() % 2;

            if(n == 0 ) {
                turn = 5;
            }

            else if(n==1) {
                turn = 15;

            }

        }

        //Push the garbage
        else if(front_obstacle) {
            left_speed  = 400;
            right_speed = 400;
        } else if(left_obstacle) {
            // turn left
            left_speed  -= 400;
            right_speed += 400;
        }

        else if (right_obstacle) {
            // turn right
            left_speed  += 400;
            right_speed -= 400;
        }

        else {
            left_speed  = 400;
            right_speed = 400;
        }

        // write actuators inputs
        wb_differential_wheels_set_speed(left_speed, right_speed);

    }

    // cleanup the Webots API
    wb_robot_cleanup();
    return 0; //EXIT_SUCCESS
}