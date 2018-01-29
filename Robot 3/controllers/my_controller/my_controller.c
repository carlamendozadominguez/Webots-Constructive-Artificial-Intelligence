/*
 * File:          C_Controller_lf.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/differential_wheels.h>, etc.
 */
#include <stdio.h>
#include <stdlib.h>  //Random number

#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/led.h>
#include <webots/device.h>
#include <webots/distance_sensor.h>
#include <webots/light_sensor.h> // light sensor library
#include <time.h>


/*
 * You may want to add macros here.
 */
#define TIME_STEP 64

#define UPPER_LIMIT 1200
#define LOWER_LIMIT 0

#define MAX_SPEED 1200
#define MAX_SPEED_LINE 500

#define LED_COUNT 10
#define PS_COUNT 8
#define GS_COUNT 3
#define BLACK_MAX 500
#define LS_COUNT 8

#define AVOID_PROX_0 200
#define AVOID_PROX_1 150
#define AVOID_PROX_2 250
#define AVOID_PROX_5 AVOID_PROX_0
#define AVOID_PROX_6 AVOID_PROX_1
#define AVOID_PROX_7 AVOID_PROX_2

 //Declarate the sensors

static WbDeviceTag ground_sensors[GS_COUNT];
static double gs_val[GS_COUNT];

static WbDeviceTag proximity_sensors[PS_COUNT];
static double ps_val[PS_COUNT];

static WbDeviceTag light_sensors[LS_COUNT];
static double ls_val[LS_COUNT];

void epuck_init() {

    int i;

// initialise proxmity sensors
    char proxname[5] = "ps0";
    for (i=0; i<PS_COUNT; i++) {
        proxname[2] = '0' + i;
        proximity_sensors[i] = wb_robot_get_device(proxname);
        wb_distance_sensor_enable(proximity_sensors[i],TIME_STEP);
    }

    // initialise light sensors
    char lightname[5] = "ls0";
    for (i=0; i<LS_COUNT; i++) {
        lightname[2] = '0' + i;
        light_sensors[i] = wb_robot_get_device(lightname);
        wb_light_sensor_enable(light_sensors[i],TIME_STEP);
    }


    // initialise ground sensors
    int n_dev = wb_robot_get_number_of_devices(); // all devices
    for (i=0; i<n_dev; i++) {
        WbDeviceTag dev = wb_robot_get_device_by_index(i);
        // Check to see if device is a ground sensor
        const char *nm = wb_device_get_name(dev);
        WbNodeType type = wb_device_get_node_type(dev);
        if(type==WB_NODE_DISTANCE_SENSOR && nm[0]=='g' && nm[1]=='s') {
            int idx = nm[2] - '0';
            if (idx>=0 && idx<GS_COUNT) { // we expect 3 ground sensors
                ground_sensors[idx] = wb_robot_get_device(nm);
                wb_distance_sensor_enable(ground_sensors[idx],TIME_STEP);
            }
        }
    }
}

//Read the different sensors that we have
void read_sensors() {

    int i;

    for (i=0; i<LS_COUNT ; i++)
        ls_val[i] = wb_light_sensor_get_value(light_sensors[i]);

    for (i=0; i<PS_COUNT ; i++)
        ps_val[i] = wb_distance_sensor_get_value(proximity_sensors[i]);

    for (i=0; i<GS_COUNT ; i++)
        gs_val[i] = wb_distance_sensor_get_value(ground_sensors[i]);
}

//Avoid the wall when the robot is not motivated or there are obstacles
void avoid_motor_values(double *left, double *right) {
    // Default values
    *left = MAX_SPEED * 0.8;
    *right = MAX_SPEED * 0.8;
    if (ps_val[0] > AVOID_PROX_0 || ps_val[7] > AVOID_PROX_7) {
        // Objct detected ahead, but is it more left, or more right?
        if (ps_val[0] > ps_val[7]) // More on FR sensor
            *left = -MAX_SPEED * 0.8; // turn left
        else
            *right = -MAX_SPEED * 0.8; // turn right
    }

    else if(ps_val[1] > 150 && ps_val[6] > 150) {
        *left = -MAX_SPEED * 0.8; // turn left
        *right = -MAX_SPEED * 0.8; // turn left

    } else if (ps_val[1] > AVOID_PROX_1)
        *left = -MAX_SPEED * 0.8; // turn left
    else if (ps_val[6] > AVOID_PROX_6)
        *right = -MAX_SPEED * 0.8; // turn right
    else if (ps_val[2] > AVOID_PROX_2)
        *left = 0; // turn left slowly
    else if (ps_val[5] > AVOID_PROX_5)
        *right = 0; // turn right slowly

}

//Follow the line when the robot is motivated, DAMAGE > UPPER_LIMIT * 0.3
void line_follow_motor_values(double *left, double *right) {
    if (gs_val[0] < BLACK_MAX) {
        *left = MAX_SPEED_LINE * 0.1;
        *right = MAX_SPEED_LINE * 0.7;
    } else if (gs_val[2] < BLACK_MAX) {
        *left = MAX_SPEED_LINE * 0.7;
        *right = MAX_SPEED_LINE * 0.1;
    } else {
        *left = MAX_SPEED_LINE * 0.5;
        *right = MAX_SPEED_LINE * 0.5;
    }
}

//When the robot is motivated, LIFE < LOWER_LIMIT *0.4 . The robot looking for the light
void looking_for_food(double *left , double *right) {

    printf("LIGHT \n");
    bool right_l = ls_val[0] < 10 ||
                   ls_val[1] < 10 ||
                   ls_val[2] < 10;

    bool left_l =  ls_val[7] < 10 ||
                   ls_val[6] < 10 ||
                   ls_val[5] < 10;

    bool front_l = ls_val[0] < 10 &&
                   ls_val[7] < 10 ;

    if(front_l) {
        *left = MAX_SPEED_LINE * 0.8;
        *right = MAX_SPEED_LINE * 0.8;
    }

    else if(right_l) {
        *right = -MAX_SPEED ; // turn right
    }

    else if(left_l) {
        *left = -MAX_SPEED ; // turn right
    }

}

//If we are under the light, then recharge
bool recharge_life(void) {

    if(ls_val[0] > 4000 && ls_val[1] > 4000 && ls_val[2] > 4000 && ls_val[3] > 4000 &&
            ls_val[6] > 4000 && ls_val[5] > 4000 && ls_val[7] > 4000 && ls_val[4] > 4000) {
        printf("RECHARGE... \n");
        return true;
    }

    else {
        return false;
    }

}



//if the robot hits a wall or object
bool detect_wall(void) {

    if(ps_val[0] >= 100 || ps_val[1] >= 100 || ps_val[2] >= 100 ||
            ps_val[6] >= 100 || ps_val[5] >= 100 || ps_val[7] >= 100) {
        return true;
    }

    else
        return false;

}

//When the robot detects something black in the floor and the wall at the same time, then recharge the damege
bool recharge_damage(void) {
    if (gs_val[0] < BLACK_MAX || gs_val[1] < BLACK_MAX || gs_val[2] < BLACK_MAX) {

        if(detect_wall()) {
            return true;
        } else {
            return false;
        }
    } else
        return false;
}



// We create the initial life in a random way
int get_life(void) {
    srand (time(NULL));      
    return rand () % (UPPER_LIMIT-LOWER_LIMIT+1) + LOWER_LIMIT;   // Este está entre LOWER_LIMIT y UPPER_LIMIT
}

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {

    
    //Create de file
    FILE * fp;
    fp=fopen("log.csv", "w");

    /* necessary to initialize webots stuff */
    wb_robot_init();

    //The  essential (physiological) variables 
    int LIFE = UPPER_LIMIT;
    int damage = LOWER_LIMIT;

    epuck_init();

    /* main loop
     * Perform simulation steps of TIME_STEP milliseconds
     * and leave the loop when the simulation is over
     */
    int ticks = 1;

    fprintf(fp, "Ticks,Energy,Damage \n");


    while (wb_robot_step(TIME_STEP) != -1) {

        printf("LIFE: %d Y DAMAGE: %d", LIFE, damage);

        //Print the varianles in the file
        fprintf(fp, "%d , ", ticks);
        fprintf(fp, "%d , ",LIFE);
        fprintf(fp, "%d \n",damage);

        /*
         * Read the sensors :
         * Enter here functions to read sensor data, like:
         *  double val = wb_distance_sensor_get_value(my_sensor);
         */
        read_sensors();

        printf("\n");

        /* Process sensor data here */
        double left, right;

        left = MAX_SPEED_LINE * 0.5;
        right = MAX_SPEED_LINE * 0.5;

        //When the robots detect the wall or an object, damage increase and the robot avoid the obstacle
        if(detect_wall()) {
            printf("WALL \n");
            damage = damage + 10;
            avoid_motor_values(&left, &right);
        }

        //If the robot needs life
        else if(LIFE < UPPER_LIMIT * 0.4) {
            looking_for_food(&left, &right);
        }

        //If the robots need less damage
        else if(damage > UPPER_LIMIT *0.3) {
            printf("GO TO THE LINE \n");
            line_follow_motor_values(&left,&right);
        }


        if(recharge_life()) {
            LIFE = UPPER_LIMIT;
        }

        //Each tick the life decrease
        else
            LIFE --;

        if(recharge_damage()) {
            damage = LOWER_LIMIT;
        }

        //When the life is 0 or damage is the UPPER_LIMIT the robot dies.
        if(LIFE <= LOWER_LIMIT || damage >= UPPER_LIMIT) {
            printf("Die \n");
            right=0;
            left=0;
            break;
            fclose(fp);

        }

        ticks ++;
        wb_differential_wheels_set_speed(left, right);
    };

    /* Enter your cleanup code here */

    /* This is necessary to cleanup webots resources */
    wb_robot_cleanup();
    fclose(fp);
    return 0;
}