#ifndef KALMAN_H
#define KALMAN_H 
typedef struct 
{
  double prev_gps[3];
  double gps[3];
  double acc_mean[3];
  double acc[3];
  double prev_left_enc;
  double left_enc;
  double prev_right_enc;
  double right_enc;
} measurement_t;

void kal_init(measurement_t * meas,double  pose[6], double  cov[6][6]);
void kal_predict_acc(measurement_t * meas, double  pose[6], double  cov[6][6]);
void kal_update_enc(measurement_t * meas, double  pose[6], double  cov[6][6]);
void kal_update_gps(measurement_t * meas, double  pose[6], double  cov[6][6]);

#endif
