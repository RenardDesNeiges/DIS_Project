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
#define PROX_THRESHOLD 100

double range[ROBOT_NUMBER];
double bearing[ROBOT_NUMBER];
double prox_value[8];
WbDeviceTag prox_address[8];
char* robot_name; 
WbDeviceTag receiver;          	// Handle for the receiver node for range and bearing information
WbDeviceTag emitter;          	// Handle for the emitter node for range and bearing information

double cix = 0, ciy = 0, cih = 0; // integral terms of the concensus controller

void init_prox_sensor(){
	char name[] = "ps0";
	for(int i = 0; i < 8; i++) {
		prox_address[i]=wb_robot_get_device(name); // get sensor handle
		// perform distance measurements every TIME_STEP millisecond
		wb_distance_sensor_enable(prox_address[i], TIME_STEP);
		name[2]++; // increase the device name to "ps1", "ps2", etc.
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

	receiver    = wb_robot_get_device("receiver");
	emitter    = wb_robot_get_device("emitter");
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
	while(!initialized){
		/* Wait until supervisor sent range and bearing information */
		while (wb_receiver_get_queue_length(receiver) == 0) {
			// printf("waiting for supervisor\n"); 
			wb_robot_step(64); // Executing the simulation for 64ms
            send_ping();
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

        printf("ID_I : %d, ID_J : %d, RANGE %f, BEARING %f\n",robot_id,id,range[id],bearing[id]);
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
    double gamma[8] = {M_PI/12,M_PI/4,M_PI/2,(5/6)*M_PI,-M_PI/12,-M_PI/4,-M_PI/2,-(5/6)*M_PI}; // IR sensor angles
    double px = 0;
    double py = 0;
    
    for(int i = 0; i<8; i++){
        prox_value[i] = wb_distance_sensor_get_value(prox_address[i]);
        if(prox_value[i] < PROX_THRESHOLD)
            prox_value[i] = 0;
        px -= prox_value[i] * sin(gamma[i] + robot.heading);
        py -= prox_value[i] * cos(gamma[i] + robot.heading);

    }
    double ph = atan2(py, px);

    local->x = px;
    local->y = py;
    local->heading = ph;
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


void get_hyperparameters_from_supervisor(int* robot_id){
    
}