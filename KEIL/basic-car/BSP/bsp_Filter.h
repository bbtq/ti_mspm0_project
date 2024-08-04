#ifndef _BSP_FILTER_H_
#define _BSP_FILTER_H_

// 卡尔曼滤波结构体
typedef struct {
    float Q; // 过程噪声协方差
    float R; // 测量噪声协方差
    float x; // 估计值
    float P; // 估计误差协方差
    float K; // 卡尔曼增益
} KalmanFilter;

extern KalmanFilter kf_adc1;
extern KalmanFilter kf_adc2;

//卡尔曼滤波
void kalman_init(KalmanFilter *kf, float process_noise, float measurement_noise, float estimated_error, float initial_value) ;
float kalman_update(KalmanFilter *kf, float measurement);

//中值滤波
int median_filter( int new_sample);

#endif

