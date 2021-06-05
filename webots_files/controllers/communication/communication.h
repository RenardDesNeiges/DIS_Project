#ifndef COMMUNICATION
#define COMMUNICATION 

#define ROBOT_NUMBER 4
#define BUFFER_SIZE 11

// Hyperparameter vector
#define ALPHA 0
#define BETA_L 1
#define BETA_F 2
#define THETA_L 3
#define THETA_F 4
#define LAMBDA 5
#define IOTA 6
#define K_A 7
#define K_B 8
#define K_C 9
#define EPSILON_L 10

// sends the emulated sensor readings from the supervisor to the robot
void send_sensor_reading(WbDeviceTag emitter_device, int i, int j, float range, float bearing);

void send_formation_hyperparameters(WbDeviceTag emitter_device, int robot_id, float hyperparameters[BUFFER_SIZE]);

int get_supervisor_packet(WbDeviceTag reciever_device, double range[ROBOT_NUMBER][ROBOT_NUMBER], double bearing[ROBOT_NUMBER][ROBOT_NUMBER], double hyperparameters[BUFFER_SIZE-1]);

#endif
