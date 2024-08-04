#include "bsp_Filter.h"
#include "stdlib.h"

/**
 * 卡尔曼部分
*/
KalmanFilter kf_adc1;
KalmanFilter kf_adc2;

// 初始化卡尔曼滤波器
void kalman_init(KalmanFilter *kf, float process_noise, float measurement_noise, float estimated_error, float initial_value) {
    kf->Q = process_noise;
    kf->R = measurement_noise;
    kf->P = estimated_error;
    kf->x = initial_value;
}

// 卡尔曼滤波更新
float kalman_update(KalmanFilter *kf, float measurement) {
    // 预测更新
    kf->P = kf->P + kf->Q;

    // 计算卡尔曼增益
    kf->K = kf->P / (kf->P + kf->R);

    // 更新估计值
    kf->x = kf->x + kf->K * (measurement - kf->x);

    // 更新估计误差协方差
    kf->P = (1 - kf->K) * kf->P;

    return kf->x;
}

/**
 * 中值滤波部分
*/

#define WINDOW_SIZE 5

// 辅助函数，用于比较两个整数的大小
int compare(const void *a, const void *b) {
    return (*(int *)a - *(int *)b);
}

// 循迹模块模拟量数据的中值滤波
int median_filter( int new_sample) {
    static int window[WINDOW_SIZE] = {0};
    static unsigned int index = 0;

    window[index] = new_sample;
    index = (index + 1) % WINDOW_SIZE;

    // 创建一个临时数组用于排序
    int temp[WINDOW_SIZE];
    for (int i = 0; i < WINDOW_SIZE; i++) {
        temp[i] = window[i];
    }

    // 对临时数组进行排序
    qsort(temp, WINDOW_SIZE, sizeof(int), compare);

    // 返回排序后数组的中间值
    return temp[WINDOW_SIZE / 2];
}



