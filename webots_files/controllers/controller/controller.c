#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <webots/accelerometer.h>
#include <webots/position_sensor.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>

#include "../localization_controller/odometry.h"
#include "../communication/communication.h"
#include "controller.h"


#define TIME_STEP 64
#define PROX_THRESHOLD 120

double range[ROBOT_NUMBER];
double bearing[ROBOT_NUMBER];
double prox_value[8];
WbDeviceTag prox_address[8];
char* robot_name; 
WbDeviceTag receiver;          	// Handle for the receiver node for range and bearing information
WbDeviceTag pso_receiver;          	// Handle for the receiver node for range and bearing information
WbDeviceTag emitter;          	// Handle for the emitter node for range and bearing information

double cix = 0, ciy = 0, cih = 0; // integral terms of the concensus controller
double ox = 0, oy = 0, obstacle_filter = 0.2;

pose_t obstacle_buffer[OBSTACLE_BUFFER_SIZE];
double obstacle_age[OBSTACLE_BUFFER_SIZE];
int obstacle_count = 0;

pose_t prev_pose;

void init_prox_sensor(){
	char name[] = "ps0";
	for(int i = 0; i < 8; i++) {
		prox_address[i]=wb_robot_get_device(name); // get sensor handle
		// perform distance measurements every TIME_STEP millisecond
		wb_distance_sensor_enable(prox_address[i], TIME_STEP);
		name[2]++; // increase the device name to "ps1", "ps2", etc.
	}
    for(int i  = 0; i<OBSTACLE_BUFFER_SIZE; i++){
        obstacle_age[i] = -1;
    }
}

void migration_urge(pose_t *migration, pose_t robot, pose_t goal)
{
    double dist = pow( pow(goal.x-robot.x,2) + pow(goal.y-robot.y,2), 0.5);
    if(dist > 1)
    {
        migration->x = (goal.x-robot.x)/dist;
        migration->y = (goal.y-robot.y)/dist;
    }
    else
    {
        migration->x = (goal.x-robot.x);
        migration->y = (goal.y-robot.y);   
    }
    migration->heading = goal.heading;
}

void send_ping(void) {
	char out[10];
	strcpy(out,robot_name);  // in the ping message we send the name of the robot.
	wb_emitter_send(emitter,out,strlen(out)+1); 
}

void init_range_bearing_estimates(int* robot_id, pose_t* goal_pose)
{

    double goal_range, goal_bearing;
    char *rbbuffer;

    // initializing the reciever device
	robot_name=(char*) wb_robot_get_name(); 

	receiver = wb_robot_get_device("receiver");
	emitter = wb_robot_get_device("emitter");
	sscanf(robot_name,"epuck%d",robot_id);    // read robot id from the robot's name

	wb_receiver_enable(receiver,64);



    for(int i = 0; i<ROBOT_NUMBER; i++)
    {
        goal_pose[i].x = 0;
        goal_pose[i].y = 0;
        goal_pose[i].heading = 0;         
    }

	//read the initial packets
	int initialized = 0;
    if  (ROBOT_NUMBER > 1)
    {
    while(!initialized){
            /* Wait until supervisor sent range and bearing information */
            while (wb_receiver_get_queue_length(receiver) == 0) {
                // printf("waiting for supervisor\n"); 
                wb_robot_step(64); // Executing the simulation for 64ms
                send_ping(); //send a ping to allow the other robot to initialize as well
            }  
            while (wb_receiver_get_queue_length(receiver) > 0) {
                const double *message_direction;
                double message_rssi;
                rbbuffer = (char*) wb_receiver_get_data(receiver);
                // printf("parsing buffer\n");
                int id_j = (int)(rbbuffer[5]-'0');  // since the name of the sender is in the received message. Note: this does not work for robots having id bigger than 9!
                initialized = 1;
                message_direction = wb_receiver_get_emitter_direction(receiver);
                message_rssi = wb_receiver_get_signal_strength(receiver);
                goal_bearing = -atan2(message_direction[2], message_direction[0])-M_PI/2;
                goal_range = sqrt((1/message_rssi));
                goal_pose[id_j].x = goal_range * cos(goal_bearing);
                goal_pose[id_j].y = goal_range * sin(goal_bearing);
                goal_pose[id_j].heading = 0;
                // printf("Goal of %d r.t. %d: e_x = %.2f, e_y = %.2f\n", *robot_id, id_j, goal_pose[(int)rbbuffer[id_j]].x, goal_pose[(int)rbbuffer[id_j]].y);

                printf("ID_I : %d, ID_J : %d, RANGE %f, BEARING %f\n",*robot_id,id_j,goal_range,goal_bearing);
                wb_receiver_next_packet(receiver);
            }
        }
    }
    

	

    printf("Robot %d initialized \n",*robot_id);
}   


void consensus_controller(pose_t *consensus, pose_t robot_pose, pose_t *goal_pose, double kp, double ki, int robot_id, double* w)
{
    char *rbbuffer;       
    double px = 0, py = 0, ph = 0; //proportional terms
    // double range, bearing;
    double e_x, e_y;

    send_ping();


    while(wb_receiver_get_queue_length(receiver) > 0){
        const double *message_direction;
        double message_rssi;

        rbbuffer = (char*) wb_receiver_get_data(receiver);

        int id = (int)(rbbuffer[5]-'0');

        message_direction = wb_receiver_get_emitter_direction(receiver);
        message_rssi = wb_receiver_get_signal_strength(receiver);
        bearing[id] = -atan2(message_direction[2], message_direction[0])+robot_pose.heading-M_PI/2;
        range[id] = sqrt((1/message_rssi));

        wb_receiver_next_packet(receiver);

        // printf("ID_I : %d, ID_J : %d, RANGE %f, BEARING %f\n",robot_id,id,range[id],bearing[id]);
    }

    for(int i = 0; i<ROBOT_NUMBER; i++)
    {
        if(i!= robot_id)
        {

            double global_bearing = bearing[i];
            e_x = goal_pose[i].x - cos(global_bearing)*range[i];
            e_y = goal_pose[i].y - sin(global_bearing)*range[i];



            px -= w[i] * e_x;
            py -= w[i] * e_y;            

        }
    }


    cix += ki * px;
    ciy += ki * py;
    cih += ki * ph;
    
    consensus->x = kp*px + cix;
    consensus->y = kp*py + ciy;
    consensus->heading = kp*ph + cih;


    
}

void local_avoidance_controller(pose_t *local, pose_t robot)
{
    double gamma[8] = {-M_PI/12,-M_PI/2,-3*M_PI/2,-(5/6)*M_PI,M_PI/12,M_PI/2,3*M_PI/2,(5/6)*M_PI}; // IR sensor angles
    double ox_a = 0, oy_a = 0, o_n, dot;
    double rel_angle = 0;
    for(int i = 0; i<8; i++){
        prox_value[i] = wb_distance_sensor_get_value(prox_address[i]);
        if(prox_value[i] < PROX_THRESHOLD)
            prox_value[i] = 0;
        ox_a += prox_value[i] * cos(gamma[i] + robot.heading);
        oy_a += prox_value[i] * sin(gamma[i] + robot.heading);

    }


    double lx = 0, ly = 0;
    double vx = robot.x - prev_pose.x;
    double vy = robot.y - prev_pose.y;

    for(int i  = 0; i<OBSTACLE_BUFFER_SIZE; i++){
        if(obstacle_age[i] >= 0){
            obstacle_age[i] += 1;
            if(obstacle_age[i] >= MAX_OBSTACLE_AGE)
                obstacle_age[i] = -1;

            obstacle_buffer[i].x += vx;
            obstacle_buffer[i].y += vy;

            lx -= obstacle_buffer[i].x;
            ly -= obstacle_buffer[i].y;

        }
        // printf("%f,", obstacle_age[i]);
    }

    // printf("\n");

    if(ox_a != 0 || oy_a != 0)
    {
        // only add obstacle if no other robot is close
        int close = 0;  
        for(int i = 0; i<ROBOT_NUMBER; i++)
        {
            if(range[i] < 0.01 && i != (int)(robot_name[5]-'0'))
            {
                if(fabs(bearing[i]- atan2(oy_a,ox_a)) < 3 )
                    close = 1;
                    // printf("close\n");
            }
            
        }
        for(int i = 0; i<OBSTACLE_BUFFER_SIZE; i++)
        {
            if(obstacle_age[i] != -1)
            {
                if( sqrt( (obstacle_buffer[i].x-ox_a)*(obstacle_buffer[i].x-ox_a) + (obstacle_buffer[i].y-oy_a) * (obstacle_buffer[i].y-oy_a) ) < 0.05 )
                    close = 1;
                    // printf("close\n");
            }
            
        }
        if(close == 0)
        {
            pose_t obstacle_position;
            obstacle_position.x = robot.x + ox_a;
            obstacle_position.y = robot.y + oy_a;
            obstacle_position.heading = 0;
            obstacle_buffer[obstacle_count] = obstacle_position;
            obstacle_age[obstacle_count] = 0;

            obstacle_count += 1;
            obstacle_count = obstacle_count%OBSTACLE_BUFFER_SIZE;

            // printf("added obstacle %d \n", obstacle_count);
        }
        
        
    }

    
    

    // ox = -ox_a * obstacle_filter + ox * (1-obstacle_filter);
    // oy = oy_a * obstacle_filter + oy * (1-obstacle_filter);

    // if(fabs(ox) > 0.0001 || fabs(oy) > 0.0001)
    // {
    //      //project the vector on the obstacle normalized direction vector

    //     dot = ox * control.x + oy * control.y;
    //     double norm = dot * sqrt(ox*ox+oy*oy);
    //     printf("%f\n",norm);

    //     //set the values
    //     local->x = -dot * ox/norm;
    //     local->y =  dot * oy/norm;
    //     local->heading = control.heading;
    // }
    // else
    // {
    //     // if the obstacle direction vector is 0, then just pass the control vector
    //     local->x = control.x;
    //     local->y = control.y;
    //     local->heading = control.heading;

    // }

    // if the obstacle direction vector is 0, then just pass the control vector
    local->x = lx;
    local->y = ly;
    local->heading = atan2(ly, lx);


    // printf("local: (%f,%f) \n",local->x,local->y);

    // double ph = atan2(py, px);

    // printf("%f, %f\n",px, py);
    
    prev_pose = robot;
    
}

void unicycle_controller(double *omega, double *v, pose_t robot, pose_t goal, double ka, double kb, double kc)
{   

    double beta = atan2(goal.y, goal.x);
    double alpha = robot.heading - beta;
    double theta =  robot.heading - goal.heading;
    double e = pow( pow(goal.x,2) + pow(goal.y,2), 0.5);
    // printf("%f %f,%f \n", e, beta, alpha);
    if(fabs(e) < 0.006) //avoids oscillations
        e = 0;
    if(fabs(alpha) < 0.1) //avoids oscillations
        alpha = 0;
    *v = (ka * cos(alpha)) * e;

    if(e == 0) //if we are close to the destination, the alpha value looses meaning so we just account for theta (to prevent oscillation caused by the fact that the controller is discrete)
    {
         *omega =  ka * kc * theta;
    }
    else
    {
    if(alpha != 0)
        *omega = kb * alpha + ka * (cos(alpha)*sin(alpha))/alpha * (alpha + kc * theta);
    else // when alpha tends to zero the cos*sin/alpha term tends to 1, but c doesn't know that ...
        *omega = kb * alpha + ka * (alpha + kc * theta);
    }
    // printf("e = %f, alpha = %f, v = %f, omega = %f \n", e, alpha, *v, *omega);
    // printf("theta = %f, omega = %f \n",theta, *omega);
}

void unicylce_to_wheels(double *w_left, double *w_right, double u_omega, double u_v, double l, double threshold)
{  
    
    double normalize = 1;
    double left = u_v + (l/2) * u_omega;
    double right = u_v - (l/2) * u_omega;
    if(fabs(left) > threshold)
        normalize = threshold/fabs(left);
    if (fabs(right) > threshold)
        if(fabs(right) > fabs(left))
            normalize = threshold/fabs(right);
    *w_left = left*normalize;
    *w_right = right*normalize;
    // printf("left = %f, right = %f \n",*w_left,*w_right);
}


int get_hyperparameters_from_supervisor(double *hyperparameters)
{

    int updated = 0;
    double *rbbuffer;

    while (wb_receiver_get_queue_length(pso_receiver) == 0)
    {
        rbbuffer = (double*) wb_receiver_get_data(pso_receiver);
        for(int i = 0; i < BUFFER_SIZE; i++)
        {
            hyperparameters[i] = rbbuffer[i];
            printf("%f", hyperparameters[i]);
        }
        printf("\n");
        wb_receiver_next_packet(pso_receiver);
        updated = 1;
    }

    return updated;

    
}

	

void init_pso_reciever()
{
    pso_receiver = wb_robot_get_device("receiver2");
    wb_receiver_enable(pso_receiver,64);
}    