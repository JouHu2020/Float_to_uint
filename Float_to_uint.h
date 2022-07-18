#include <iostream>
#include <math.h>
#include <stdio.h>
#include <cstring>
#include <iostream>

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -65.0f
#define V_MAX 65.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -18.0f
#define T_MAX 18.0f

/// Joint Soft Stops /
#define A_LIM_P 1.5f //Ab/Ad
#define A_LIM_N -1.5f
#define H_LIM_P 5.0f //hik
#define H_LIM_N -5.0f
#define K_LIM_P 0.2f //kenn
#define K_LIM_N 7.7f
#define KP_SOFTSTOP 100.0f
#define KD_SOFTSTOP 0.4f

#define PI 3.14159265359f

// float uint_to_float(int x_int, float x_min, float x_max, int bits);
struct joint_state
{
    float p, v, t;
};
struct joint_control
{
    float p_des, v_des, kp, kd, t_ff;
};
struct leg_state
{
    joint_state a, h, k;
};
class CANmessage
{
public:
    uint8_t data[8];
};

class rxCANMessage
{
public:
    uint8_t data[6];
};

float fmaxf1(float x, float y);
float fminf1(float x, float y);
float fmaxf3(float x, float y, float z);
float fminf3(float x, float y, float z);
void limit_norm(float *x, float *y, float limit);
int f_to_u(float x, float x_min, float x_max, int bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
// void pack_cmd(CANmessage *msg, joint_control joint);
