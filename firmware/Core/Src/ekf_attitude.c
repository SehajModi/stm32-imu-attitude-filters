/*
 * ekf_attitude.c : roll/pitch EKF with gyro-bias states (starting point)
 *
 * State  x = [roll, pitch, bias_gx, bias_gy]   (rad, rad, rad/s, rad/s)
 * Predict: Euler-rate kinematics driven by gyro (gx, gy, gz), bias removed
 * Update : accelerometer roll/pitch, R inflated when |a| differs from 1 g
 *
 * Units in: gyro rad/s, accel g, dt seconds.
 * Pass gz with its startup bias already subtracted (no bias state for yaw rate).
 * All values in ekf_init() are starting guesses: tune Q and R on your logs.
 */
#include <math.h>
#include <string.h>
#include "ekf_attitude.h"

#define PITCH_LIM 1.48f   /* ~85 deg: tan(pitch) blows up at 90 */
#define EKF_ACC_GATE_G 0.03f   /* skip accel update when | |a| - 1 g | exceeds this */

void ekf_init(ekf_t *e, float roll0, float pitch0, float bx0, float by0)
{
    memset(e, 0, sizeof *e);
    e->x[0] = roll0;  e->x[1] = pitch0;
    e->x[2] = bx0;    e->x[3] = by0;
    e->P[0][0] = e->P[1][1] = 1e-2f;
    e->P[2][2] = e->P[3][3] = 1e-4f;
    e->q_angle = 1e-5f;
    e->q_bias  = 1e-9f;
    e->r_acc   = 1e-4f;      /* about 0.57 deg std */
}

void ekf_predict(ekf_t *e, float gx, float gy, float gz, float dt)
{
    float roll = e->x[0], pitch = e->x[1];
    float p = gx - e->x[2];
    float q = gy - e->x[3];
    float r = gz;

    float sr = sinf(roll), cr = cosf(roll);
    float tp = tanf(pitch);
    float sec2 = 1.0f + tp * tp;

    /* state propagation */
    e->x[0] += (p + sr * tp * q + cr * tp * r) * dt;
    e->x[1] += (cr * q - sr * r) * dt;
    if (e->x[1] >  PITCH_LIM) e->x[1] =  PITCH_LIM;
    if (e->x[1] < -PITCH_LIM) e->x[1] = -PITCH_LIM;

    /* Jacobian F = I + A*dt, evaluated at the pre-update angles */
    float F[4][4] = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
    F[0][0] += (cr * tp * q - sr * tp * r) * dt;
    F[0][1] += (sr * q + cr * r) * sec2 * dt;
    F[0][2] += -dt;
    F[0][3] += -sr * tp * dt;
    F[1][0] += (-sr * q - cr * r) * dt;
    F[1][3] += -cr * dt;

    /* P = F P F^T + Q*dt */
    float T[4][4], Pn[4][4];
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++) {
            float s = 0;
            for (int k = 0; k < 4; k++) s += F[i][k] * e->P[k][j];
            T[i][j] = s;
        }
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++) {
            float s = 0;
            for (int k = 0; k < 4; k++) s += T[i][k] * F[j][k];
            Pn[i][j] = s;
        }
    memcpy(e->P, Pn, sizeof Pn);
    e->P[0][0] += e->q_angle * dt;
    e->P[1][1] += e->q_angle * dt;
    e->P[2][2] += e->q_bias  * dt;
    e->P[3][3] += e->q_bias  * dt;
}

static float wrap_pi(float a)
{
    const float PI = 3.14159265f;
    while (a >  PI) a -= 2.0f * PI;
    while (a < -PI) a += 2.0f * PI;
    return a;
}

void ekf_update(ekf_t *e, float ax, float ay, float az)
{
    float yz   = sqrtf(ay * ay + az * az);
    float zr   = atan2f(ay, az);
    float zp   = atan2f(-ax, yz);

    /* Board is accelerating (|a| != 1 g): the accel angle is not gravity.
       Skip the update and coast on the gyro, so a bad innovation can never
       drag the angle or the gyro-bias states. Same 0.03 g gate as the
       complementary filter. */
    float dev  = fabsf(sqrtf(ax * ax + ay * ay + az * az) - 1.0f);
    if (dev > EKF_ACC_GATE_G) return;

    /* inside the gate: soften the update as |a| nears the limit */
    float k    = dev / EKF_ACC_GATE_G;
    float R    = e->r_acc * (1.0f + 100.0f * k * k);

    float y0 = wrap_pi(zr - e->x[0]);
    float y1 = zp - e->x[1];

    /* H = [I2 0], so S = P[0:2][0:2] + R*I */
    float s00 = e->P[0][0] + R, s01 = e->P[0][1];
    float s10 = e->P[1][0],     s11 = e->P[1][1] + R;
    float det = s00 * s11 - s01 * s10;
    if (fabsf(det) < 1e-12f) return;
    float i00 =  s11 / det, i01 = -s01 / det;
    float i10 = -s10 / det, i11 =  s00 / det;

    float K[4][2];
    for (int i = 0; i < 4; i++) {
        K[i][0] = e->P[i][0] * i00 + e->P[i][1] * i10;
        K[i][1] = e->P[i][0] * i01 + e->P[i][1] * i11;
    }

    for (int i = 0; i < 4; i++)
        e->x[i] += K[i][0] * y0 + K[i][1] * y1;
    e->x[0] = wrap_pi(e->x[0]);
    if (e->x[1] >  PITCH_LIM) e->x[1] =  PITCH_LIM;
    if (e->x[1] < -PITCH_LIM) e->x[1] = -PITCH_LIM;

    /* P = (I - K H) P ; rows 0 and 1 of P are copied first */
    float r0[4], r1[4];
    for (int j = 0; j < 4; j++) { r0[j] = e->P[0][j]; r1[j] = e->P[1][j]; }
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            e->P[i][j] -= K[i][0] * r0[j] + K[i][1] * r1[j];

    /* keep P symmetric: float rounding slowly breaks it over thousands of steps */
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < i; j++) {
            float m = 0.5f * (e->P[i][j] + e->P[j][i]);
            e->P[i][j] = e->P[j][i] = m;
        }
}
