#ifndef EKF_ATTITUDE_H
#define EKF_ATTITUDE_H

typedef struct {
    float x[4];      /* roll, pitch [rad], bias_gx, bias_gy [rad/s] */
    float P[4][4];
    float q_angle;   /* angle process noise, rad^2 per s    */
    float q_bias;    /* bias random walk, (rad/s)^2 per s   */
    float r_acc;     /* accel angle variance, rad^2         */
} ekf_t;

void ekf_init(ekf_t *e, float roll0, float pitch0, float bx0, float by0);
void ekf_predict(ekf_t *e, float gx, float gy, float gz, float dt);
void ekf_update(ekf_t *e, float ax, float ay, float az);

#endif
