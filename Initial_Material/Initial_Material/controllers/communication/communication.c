#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/supervisor.h>
#include <webots/receiver.h>

#include "communication.h"

void send_sensor_reading(WbDeviceTag emitter_device, int i, int j, float range, float bearing)
{	

	float buffer[255];

	buffer[PACKET_TYPE]= SENSOR;
	buffer[ID_I]= i;
	buffer[ID_J]= j;
	buffer[RANGE] = range;
	buffer[BEARING] = bearing;

    wb_emitter_send(emitter_device,(char *)buffer,BUFFER_SIZE*sizeof(float));        
		
	
}

void send_formation_hyperparameters(WbDeviceTag emitter_device, int robot_id, float hyperparameters[BUFFER_SIZE])
{
    
}

int get_supervisor_packet(WbDeviceTag reciever_device, double range[ROBOT_NUMBER][ROBOT_NUMBER], double bearing[ROBOT_NUMBER][ROBOT_NUMBER], double hyperparameters[BUFFER_SIZE-1])
{

    float *rbbuffer;
    int reset = 0;

    while(wb_receiver_get_queue_length(reciever_device) > 0){

        rbbuffer = (float*) wb_receiver_get_data(reciever_device);

        if(rbbuffer[PACKET_TYPE]==SENSOR)
        {
            int i = (int)rbbuffer[ID_I];
            int j = (int)rbbuffer[ID_J];

            range[i][j] = rbbuffer[RANGE];
            bearing[i][j] = rbbuffer[BEARING];
        }
        if(rbbuffer[PACKET_TYPE]==RESET){
            reset = 1;
            //copy the hyperparameters to the robot's table
            for(int cnt = 0; cnt < BUFFER_SIZE-1; cnt++) 
            {
                hyperparameters[cnt] = rbbuffer[cnt+1];
            }
        }

        wb_receiver_next_packet(reciever_device);
    }

    return reset;
}