#include "Float_to_uint.h"
#include <stdio.h>
using namespace std;

float fmaxf1(float x, float y)
{
    /// Returns maximum of x, y ///
    return (((x) > (y)) ? (x) : (y)); //x若>y返回x的值，否则返回y的值
}

float fminf1(float x, float y)
{
    /// Returns minimum of x, y ///
    return (((x) < (y)) ? (x) : (y));
}

float fmaxf3(float x, float y, float z)
{
    /// Returns maximum of x, y, z ///
    return (x > y ? (x > z ? x : z) : (y > z ? y : z));
}

float fminf3(float x, float y, float z)
{
    /// Returns minimum of x, y, z ///
    return (x < y ? (x < z ? x : z) : (y < z ? y : z));
}

void limit_norm(float *x, float *y, float limit)
{
    /// Scales the lenght of vector (x, y) to be <= limit ///
    float norm = sqrt(*x * *x + *y * *y);
    if (norm > limit)
    {
        *x = *x * limit / norm;
        *y = *y * limit / norm;
    }
}

int f_to_u(float x, float x_min, float x_max, int bits)
{
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

void pack_cmd(CANmessage *msg, joint_control joint)
{ //下发给驱动的指令

    /// limit data to be within bounds ///将数据限制在范围内
    float p_des = fminf1(fmaxf1(P_MIN, joint.p_des), P_MAX);
    float v_des = fminf1(fmaxf1(V_MIN, joint.v_des), V_MAX);
    float kp = fminf1(fmaxf1(KP_MIN, joint.kp), KP_MAX);
    float kd = fminf1(fmaxf1(KD_MIN, joint.kd), KD_MAX);
    float t_ff = fminf1(fmaxf1(T_MIN, joint.t_ff), T_MAX);

    /// convert floats to unsigned ints /// 浮点转无符号整数
    uint16_t p_int = f_to_u(p_des, P_MIN, P_MAX, 16);
    uint16_t v_int = f_to_u(v_des, V_MIN, V_MAX, 12);
    uint16_t kp_int = f_to_u(kp, KP_MIN, KP_MAX, 12);
    uint16_t kd_int = f_to_u(kd, KD_MIN, KD_MAX, 12);
    uint16_t t_int = f_to_u(t_ff, T_MIN, T_MAX, 12);

    msg->data[0] = p_int >> 8;
    msg->data[1] = p_int & 0xFF;
    msg->data[2] = v_int >> 4;
    msg->data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
    msg->data[4] = kp_int & 0xFF;
    msg->data[5] = kd_int >> 4;
    msg->data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
    msg->data[7] = t_int & 0xff;
}

void unpack_reply(rxCANMessage msg, joint_state rx_joint)
{ //驱动返回的信息
    /// unpack ints from can buffer /// 从can的缓冲区得到数据
    uint16_t id = msg.data[0];
    uint16_t p_int = (msg.data[1] << 8) | msg.data[2];
    uint16_t v_int = (msg.data[3] << 4) | (msg.data[4] >> 4);
    uint16_t i_int = ((msg.data[4] & 0xF) << 8) | msg.data[5];
    /// convert uints to floats /// 将uint转换为float
    rx_joint.p = uint_to_float(p_int, P_MIN, P_MAX, 16);
    rx_joint.v = uint_to_float(v_int, V_MIN, V_MAX, 12);
    rx_joint.t = uint_to_float(i_int, -T_MAX, T_MAX, 12);
}

int main()
{
    float a = 120.45;
    uint32_t *data_16 = (uint32_t *)&a;
    printf("data_16:%x\n", *data_16);

    uint8_t data_8[] = {0x85, 0xEB, 0x41, 0x41};
    // uint32_t data_32 = (uint32_t)data_8;
    float *ptr = (float *)data_8;
    printf("float %g\n", *ptr);

    // rxCANMessage rx_can;  //接收到的can数据，6个字节
    // joint_state rx_joint; //根据接收到的can数据转换而成的关节状态，包括位置，速度，力矩
    // FILE *fp = fopen("./data.txt", "r");
    // if (!fp)
    // {
    //     printf("打开失败！\n");
    //     return -1; //返回异常
    // }
    // char data1[10], data2[10]; //用来储存两个字符串数据
    // while (!feof(fp))          //feof（）检测一个文件是否结束，即到达文件尾，若结束，则返回非0值，否则返回0
    // {
    //     fscanf(fp, "%s%s\n", data1, data2);
    //     cout << data1[1] << endl;
    //     rx_can = ; //需要给他赋值
    //     unpack_reply(rx_can, rx_joint);
    // }
    // fclose(fp);

    /* 
        以下部分为浮点数转can数据***************************
     */
    /*     CANmessage tx;
    CANmessage *Msg_des = &tx;
    joint_control joint_des;
    memset(&tx, 0, sizeof(CANmessage));
    memset(&joint_des, 0, sizeof(joint_des));
    joint_des.kd = 0.2;
    joint_des.kp = 5;
    joint_des.p_des = 1.0;
    joint_des.v_des = 0.f;
    joint_des.t_ff = 0.f;
    pack_cmd(Msg_des, joint_des);
    for (int i = 0; i < 8; i++)
        printf("tx[%d]:%x\n", i, tx.data[i]); */

    return 0;
}