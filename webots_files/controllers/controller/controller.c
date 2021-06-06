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
#define THRESH_TURN 1.2

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
double turn = 1;

pose_t obstacle_buffer[OBSTACLE_BUFFER_SIZE];
double obstacle_age[OBSTACLE_BUFFER_SIZE];
int obstacle_count = 0;

pose_t prev_pose;

pose_t past_relative_poses[ROBOT_NUMBER];
int started = 0;

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

    double e_x,e_y;
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
                rbbuffer = (char*) wb_receiver_get_data(receiver);
                initialized = 1;
                wb_receiver_next_packet(receiver);
            }
        }
    }

    for(int i = 0; i<ROBOT_NUMBER; i++)
    {
        e_x = ref_poses[i].x - ref_poses[*robot_id].x;
        e_y = ref_poses[i].x - ref_poses[*robot_id].y;
        goal_bearing = atan2(e_y, e_x);
        goal_range = sqrt(e_x*e_x+e_y*e_y);
        goal_pose[i].x = goal_range * cos(goal_bearing);
        goal_pose[i].y = goal_range * sin(goal_bearing);
        goal_pose[i].heading = 0;

        printf("ID_I : %d, ID_J : %d, RANGE %f, BEARING %f\n",*robot_id,i,goal_range,goal_bearing);
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

void reynolds_controller(pose_t *reynold, pose_t robot_pose, double w_cohesion, double w_dispersion, double w_consistency, int robot_id, double rule2radius)
{
    char *rbbuffer;     
    const double *message_direction;
    double message_rssi;

    double bearing[ROBOT_NUMBER];
    double range[ROBOT_NUMBER];

    pose_t relative_poses[ROBOT_NUMBER];
    pose_t relative_speeds[ROBOT_NUMBER];
    pose_t avg_speed, avg_location;
    avg_speed.x = 0; avg_speed.y = 0; avg_location.x = 0; avg_location.y = 0;
    pose_t dispersion;
    dispersion.x = 0; dispersion.y = 0; 


    send_ping();

    while(wb_receiver_get_queue_length(receiver) > 0)
    {
        rbbuffer = (char*) wb_receiver_get_data(receiver);

        int id = (int)(rbbuffer[5]-'0');

        message_direction = wb_receiver_get_emitter_direction(receiver);
        message_rssi = wb_receiver_get_signal_strength(receiver);

        bearing[id] = -atan2(message_direction[2], message_direction[0])+robot_pose.heading;
        range[id] = sqrt((1/message_rssi));
        relative_poses[id].x = range[id] * sin(bearing[id]);
        relative_poses[id].y = range[id] * cos(bearing[id]);

        wb_receiver_next_packet(receiver);
    }

    if(started == 1)
    {
        for(int i = 0; i < ROBOT_NUMBER; i++)
        {
            if(robot_id != i)
            {
                relative_speeds[i].x = relative_poses[i].x-past_relative_poses[i].x;
                relative_speeds[i].y = relative_poses[i].y-past_relative_poses[i].y;
                avg_speed.x += relative_speeds[i].x/ROBOT_NUMBER;
                avg_speed.y += relative_speeds[i].y/ROBOT_NUMBER;
                avg_location.x += relative_poses[i].x/ROBOT_NUMBER;
                avg_location.y += relative_poses[i].y/ROBOT_NUMBER;
            }
            
        }
        /* once the averages are good */
        for(int i = 0; i < ROBOT_NUMBER; i++)
        {
            if(robot_id != i)
            {
                // Rule 2 cohesion
                if(relative_poses[i].x != 0 && relative_poses[i].y != 0)
                    if(pow(range[i],2) < rule2radius)
                    {
                            dispersion.x -= 1/relative_poses[i].x;
                            dispersion.y -= 1/relative_poses[i].y;
                    }
            }
        }

    }


    started = 1;
    for(int i = 0; i < ROBOT_NUMBER; i++)
    {
        past_relative_poses[i].x = relative_poses[i].x;
        past_relative_poses[i].y = relative_poses[i].y;
    }

    // printf("coh:(%f,%f), con:(%f,%f), dis:(%f,%f)\n",avg_location.x,avg_location.y,avg_speed.x,avg_speed.y,dispersion.x,dispersion.y);  

    reynold->x =    w_cohesion *avg_location.x + 
                    w_consistency * avg_speed.x + 
                    w_dispersion * dispersion.x;
    reynold->y =     w_cohesion *avg_location.y + 
                    w_consistency * avg_speed.y + 
                    w_dispersion * dispersion.y;
    reynold->heading = atan2(reynold->y, reynold->x);
}

void unicycle_controller(double *omega, double *v, pose_t robot, pose_t goal, double ka, double kb, double kc)
{   
    /* get the local obstacle avoidance sensors */

    double weights[8] = {1,1,1,0,1,1,1,0}; // IR sensor angles
    double gamma[8] = {-M_PI/12,-M_PI/2,-3*M_PI/2,-(5/6)*M_PI,M_PI/12,M_PI/2,3*M_PI/2,(5/6)*M_PI}; // IR sensor angles
    double ox = 0, oy = 0, angle;
    for(int i = 0; i<8; i++){
        prox_value[i] = wb_distance_sensor_get_value(prox_address[i]);
        if(prox_value[i] < PROX_THRESHOLD)
            prox_value[i] = 0;
        ox += prox_value[i] * cos(gamma[i] + robot.heading) * weights[i];
        oy += prox_value[i] * sin(gamma[i] + robot.heading) * weights[i];
    } 
    angle = atan2(oy,ox);

    /* if we have no sensor readings run a simple unicycle-like controller */
    if( (ox == 0 && oy == 0) || (fabs(angle-atan2(goal.y, goal.x))) >= M_PI )
    {
        double beta = atan2(goal.y, goal.x);
        double alpha = robot.heading - beta;
        double theta =  robot.heading - goal.heading;
        double e = pow( pow(goal.x,2) + pow(goal.y,2), 0.5);

        if(fabs(e) < 0.006) //avoids oscillations
            e = 0;
        if(fabs(alpha) < 0.1) //avoids oscillations
            alpha = 0;
        *v = (ka * cos(alpha)) * e; //sets the unicycle speed
        if(fabs(alpha) > M_PI_2) //we don't want to move backwards
        {
            *omega = alpha;
        }
        else
        {
            if(e == 0) //if we are close to the destination, the alpha value looses meaning so we just account for theta (to prevent oscillation caused by the fact that the controller is discrete)
            {
                    *omega =  ka * kc * theta; //sets the unicycle angular speed
            }
            else
            {
            if(alpha != 0) //sets the unicycle angular speed
                *omega = kb * alpha + ka * (cos(alpha)*sin(alpha))/alpha * (alpha + kc * theta);
            else // when alpha tends to zero the cos*sin/alpha term tends to 1, but c doesn't know that ...
                *omega = kb * alpha + ka * (alpha + kc * theta);
        }
        
        }
    }    /* else run simple wall following controller */
    else
    {

        
        if(angle-robot.heading < THRESH_TURN)
            turn = -1;
        if(angle-robot.heading > THRESH_TURN)
            turn = 1;

        // if( fabs(angle-robot.heading) < 1)
        // {
        //     //just rotate
        //     *omega = robot.heading - angle + M_PI/2*turn + 0.2*turn;
        //     *v = 0;
        // }
        // else{
            //rotate and advance
            //*omega = something
            *omega = robot.heading - angle + M_PI/2*turn +0.2*turn;
            *v = 10*sin(fabs(robot.heading - angle)); //lol
        // }

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
    float *rbbuffer;

    while (wb_receiver_get_queue_length(pso_receiver) > 0)
    {
        rbbuffer = (float*) wb_receiver_get_data(pso_receiver);
        printf("[");
        for(int i = 0; i < BUFFER_SIZE; i++)
        {
            hyperparameters[i] = rbbuffer[i];
            printf("%f ;", hyperparameters[i]);
        }
        printf("]\n");
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