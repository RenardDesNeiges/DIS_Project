#ifndef COMMUNICATION
#define COMMUNICATION 

#define ROBOT_NUMBER 4
#define BUFFER_SIZE 12

// type of packet (first element in any buffer)
#define PACKET_TYPE 0
// Packet type can take the following values :
#define RESET 0 // when packet type is RESET the robots should reset all of its variables and set new parameters for the controller according to the hyperparameter vector in the buffer
#define SENSOR 1 // when packet type is SENSOR the supervisors is used to emulate IR sensors

// Hyperparameter vector
#define ALPHA 1
#define BETA_L 2
#define BETA_F 3
#define GAMMA_L 4
#define GAMMA_F 5
#define LAMBDA 6
#define IOTA 7
#define K_A 8
#define K_B 9
#define K_C 10
#define EPSILON_L 11

// For the  sensor packet type
#define ID_I 1
#define ID_J 2
#define RANGE 3
#define BEARING 4

// sends the emulated sensor readings from the supervisor to the robot
void send_sensor_reading(WbDeviceTag emitter_device, int i, int j, float range, float bearing);

void send_formation_hyperparameters(WbDeviceTag emitter_device, int robot_id, float hyperparameters[BUFFER_SIZE]);

int get_supervisor_packet(WbDeviceTag reciever_device, double range[ROBOT_NUMBER][ROBOT_NUMBER], double bearing[ROBOT_NUMBER][ROBOT_NUMBER], double hyperparameters[BUFFER_SIZE-1]);

#endif
