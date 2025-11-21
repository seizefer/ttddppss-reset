#ifndef _FILTER_H
#define _FILTER_H
#include "main.h"
#include "math.h"

/*
卡尔曼滤波中
Q值为过程噪声，越小系统越容易收敛，表示对模型预测的值信任度越高；
但是太小则容易发散，如果Q为零，那么我们只相信预测值；
Q值越大表示对于预测的信任度就越低，而对测量值的信任度就变高；
如果Q值无穷大，那么则表示信任测量值。
R值为测量噪声。R太大，滤波的响应（此处响应特指对量测值的响应）会变慢，
因为它对新测量的值的信任度降低；越小系统收敛越快，但过小则容易出现震荡。
*/
typedef struct 
{
    float LastP;//上次估算协方差 初始化值为0.02
    float Now_P;//当前估算协方差 初始化值为0
    float out;//卡尔曼滤波器输出 初始化值为0
    float Kg;//卡尔曼增益 初始化值为0
    float Q;//过程噪声协方差 初始化值为0.001
    float R;//观测噪声协方差 初始化值为0.543
}KFP;//Kalman Filter parameter


float kalmanFilter(KFP *kfp,float input);
float LowpassFilter(float K, float *input);

extern KFP kfp_distance;

extern float filtered_num;
#endif
