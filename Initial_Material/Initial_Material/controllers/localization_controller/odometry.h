#ifndef ODOMETRY_H
#define ODOMETRY_H 

#define RAD2DEG(X)      X / M_PI * 180.0

typedef struct 
{
  double x;
  double y;
  double heading;
} pose_t;

void odo_compute_acc(pose_t* odo, const double acc[3], const double acc_mean[3]);
void odo_compute_encoders(pose_t* odo, double Aleft_enc, double Aright_enc);
void odo_compute_speed(pose_t* speed, const pose_t ref, double Aleft_enc, double Aright_enc);

void odo_compute_encoders_bonus(pose_t* odo, double Aleft_enc, double Aright_enc);
void odo_reset(int time_step);

pose_t pose_scale(double c, pose_t p);
pose_t pose_add(pose_t a, pose_t b);
pose_t pose_add_3(pose_t a, pose_t b, pose_t c);

#endif
