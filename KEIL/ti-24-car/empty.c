
#include "board.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "motor.h"
#include "bsp_adc.h"
#include "pid.h"
#include "string.h"
#include "bsp_Filter.h"
#include "bsp_jy901.h"
#include "sw_i2c.h"
#include "gw_grayscale_sensor.h"
#include "trace.h"
#include "i2c_app.h"


int motor_BasePower = 1300; // 1500//1m 2.68s

char cmd_stop[] = "stop\n";
char cmd_start[] = "start\n";
char cmd_power_up[] = "pup\n";
char cmd_power_down[] = "pdown\n";
char cmd_pid_kp_up[] = "kp_up\n";
char cmd_pid_kp_down[] = "kp_down\n";
char cmd_pid_kd_up[] = "kd_up\n";
char cmd_pid_kd_down[] = "kd_down\n";
char cmd_show_adc[] = "adc\n";
char cmd_show_out[] = "out\n";

typedef struct{
    int goStraight_plus ;
    int trace_line_plus ;
}score_increases_peed ;

PIDController pid = {
    // 定距
    .Kp = 0.9,
    .Ki = 0,
    .Kd = 0.5,

    // 输出限制
    .limMax = 3999,
    .limMin = 0, // 只能向前

    // 积分器限制
    .limMaxInt = 0.1 * 4000,
    .limMinInt = -0.1 * 4000 - 1500, // 减去基础正向动力

};

PIDController trace_pid = {
    // 循迹pid
    .Kp = 1.5, // 0.8
    .Ki = 0.1,
    .Kd = 1.7, // 0.5

    // 输出限制
    .limMax = 2000,
    .limMin = -2000 - 1500, // 减去基础正向动力

    // 积分器限制
    .limMaxInt = 0.1 * 4000,
    .limMinInt = -0.1 * 4000 - 1500, // 减去基础正向动力

};

PIDController yaw_pid = {
    // 角度pid
    .Kp = 5,
    .Ki = 0,
    .Kd = 4,

    // 输出限制
    .limMax = 3000,
    .limMin = -3000 - 1500, // 减去基础正向动力

    // 积分器限制
    .limMaxInt = 300,
    .limMinInt = -300, // 减去基础正向动力

};

bool event_lock = 1;    // 事件处理触发
uint8_t event_task = 0; // 任务中的各个小任务段
bool nearingTheEnd = 0; // 最后一项任务的标志
uint8_t ground = 0;     // 圈数
score_increases_peed task4_power_plus = {
    .goStraight_plus = 0,
    .trace_line_plus = 0,
};   //任务4提速部分
int nearget_downpower = 0;
bool break_noout = 1;

bool distance_1m_flag = 0;    // 1m程序执行
bool distance_1m_receive = 0; // 1m完成标志
uint16_t turn_on_first = 0;

bool trace_flag = 0;    // 循迹执行
bool line_no_check = 0; // 无黑线
float yaw_step = 180.0;
int end_power_decelerate = 0;   //最后减速，增加准确度
int start_power_decelerate = 0; //最开始的减速，增加容错（ task4 
uint32_t trace_decelerate_time = 0; //减速时间。

bool yaw_control_flag = 0;
float yaw_expt = 0;

void turn_dirction(void)
{
    base_direction += 180;
    if (base_direction < -180)
        base_direction = base_direction + 360;
    if (base_direction > 180)
        base_direction = base_direction - 360;
}

int main(void)
{

    /**
    **  adc模拟量缓存
    **/
    uint16_t adc_buf[2] = {0, 0};
    unsigned int voltage_value = 0;

    /**
    **  感为科技循迹模块i2c
    **/
    /* 存放扫描到的地址 */
    uint8_t scan_addr[128] = {0};
    volatile uint8_t count;
    uint8_t ping_response;

    // 开发板初始化
    board_init();

    // 感为科技循迹模块初始读取与设置
    /*读取数字量*/
    uint8_t gray_sensor[8];
    uint8_t digital_data = 8;
    /*打开数字量模式，并且读取，后面可以直接读取*/
    digital_data = GW_GRAY_DIGITAL_MODE;
    i2c_app_write(I2C0_INST, 0x4C, &digital_data, 1);
    /*读取数据*/
    i2c_app_read(I2C0_INST, 0x4C, &digital_data, 1);

    // // 卡尔曼参数初始化
    //     kalman_init(&kf_adc1, 10, 30, 10, 0);
    //     kalman_init(&kf_adc2, 10, 30, 10, 0);

    // PID参数初始化
    PIDController_Init(&pid);
    PIDController_Init(&trace_pid);
    PIDController_Init(&yaw_pid);
    int trace_out = 0;
    int base_out = 200; // 要求3的转一定角度的基础差速
    // Total_PID_Init();

    // motor_control(-1500,-1500);
    // motor_control(500,1500);

    // 打印start，初始化完成，程序开始
    printf("start\r\n");

    while (1)
    {

        /*事件处理*/
        if (start_flag)
        {
            // trace_flag = 1;
            if (event_lock)
            {
                event_lock = 0;
                if (task == 1)
                {
                    if (event_task == 0)
                    { // 1m定距离-直线
                        defaultwheel.left.count = 0;
                        distance_1m_flag = 1;
                        yaw_expt = base_direction;
                    }
                    if (event_task == 1)
                    {
                        if (distance_1m_receive)
                        { //
                            sound_flag = 1;
                            distance_1m_flag = 0;
                            motor_control(0, 0);
                        }
                    }
                    event_task++;
                }
                if (task == 2)
                {
                    if (event_task == 0)
                    { // 1m定距离-直线
                        defaultwheel.left.count = 0;
                        distance_1m_flag = 1;
                        yaw_expt = base_direction;
                    }
                    if (event_task == 1)
                    { // 循迹
                        // turn_dirction(); 误差大
                        
                        PIDController_Init(&trace_pid);
                        if (distance_1m_receive)
                        {
                            while(jy901_yaw<yaw_return(base_direction+10))motor_control(1500,700);
                            trace_flag = 1;
                            distance_1m_flag = 0;
                            defaultwheel.left.count = 0;
                            sound_flag = 1;
                            yaw_expt = base_BackDirection;
                        }
                    }
                    if (event_task == 2)
                    {
                        if (line_no_check)
                        { // 角度矫正
                            event_task++;
                            motor_control(300, 500); // 先出去一点，摆正走半圆弧的角度
                            delay_ms(20);
                        }
                    }
                    if (event_task == 3)
                    {
                        if (yaw_control_flag == 0)
                        { // 角度已摆正，走1m返程
                            trace_flag = 0;
                            distance_1m_flag = 1;
                            defaultwheel.left.count = 0;
                            sound_flag = 1;
                            yaw_expt = base_BackDirection; // 返程定向可以略微向右，增加碰线几率
                            yaw_pid.prevError = 0;
                            yaw_pid.prevMeasurement = yaw_expt;
                        }
                    }
                    if (event_task == 4)
                    { // 循迹
                        // turn_dirction(); 误差大

                        PIDController_Init(&trace_pid);
                        if (distance_1m_receive)
                        {
                            while(jy901_yaw<yaw_return(base_BackDirection+10))motor_control(1500,700);
                            sound_flag = 1;
                            trace_flag = 1;
                            distance_1m_flag = 0;
                            defaultwheel.left.count = 0;

                            yaw_expt = base_direction;
                            nearingTheEnd = 1;
                        }
                    }
                    if (event_task == 5)
                    { // 停止
                        if (line_no_check)
                        {
                            trace_flag = 0;
                            distance_1m_flag = 0;
                            defaultwheel.left.count = 0;
                            sound_flag = 1;
                            motor_control(0, 0);
                            nearingTheEnd = 0;
                        }
                    }

                    event_task++;
                }
                if (task == 3)
                {
                    if (event_task == 0)
                    { // 角度矫正 ____ 38.66
                        yaw_expt = yaw_return(base_direction + 39.76);
                        while (fabs(jy901_yaw - yaw_expt) > 3)
                            motor_control(950, -950);
                        motor_control(0, 0);
                        event_task++;
                    }
                    if (event_task == 1)
                    { // 定距离-直线
                        defaultwheel.left.count = 0;
                        distance_1m_flag = 1;
                        sound_flag = 1;
                        yaw_expt = yaw_return(base_direction + 38.70.6);
                        yaw_pid.prevError = 0;
                        yaw_pid.prevMeasurement = yaw_expt;
                    }
                    if (event_task == 2)
                    { // 循迹
                        // turn_dirction(); 误差大
                        sound_flag = 1;
                        yaw_expt = yaw_return(base_direction - 3);
                        while (fabs(jy901_yaw - yaw_expt) > 3)
                            motor_control(-450, 1050);
                        motor_control(500, 500);
                        PIDController_Init(&trace_pid);
                        if (distance_1m_receive)
                        {
                            trace_flag = 1;
                            distance_1m_flag = 0;
                            defaultwheel.left.count = 0;

                            yaw_expt = base_BackDirection;
                        }
                    }
                    if (event_task == 3)
                    {
                        if (line_no_check)
                        { // 角度矫正 ____ 38.66
                            trace_flag = 0;
                            distance_1m_flag = 0;
                            defaultwheel.left.count = 0;
                            sound_flag = 1;
                            yaw_expt = yaw_return(base_BackDirection - 38.76);
                            while (fabs(jy901_yaw - yaw_expt) > 3)
                                motor_control(-950, 950);
                            // printf("angle ready: %d, back_dir : %d\r\n",(int)yaw_expt,(int)base_BackDirection);
                            motor_control(0, 0);
                            event_task = 4;
                        }
                    }
                    if (event_task == 4)
                    { // 定距离-直线
                        defaultwheel.left.count = 0;
                        trace_flag = 0;
                        distance_1m_flag = 1;
                        sound_flag = 1;
                        yaw_expt = yaw_return(base_BackDirection - 38.76);
                        yaw_pid.prevError = 0;
                        yaw_pid.prevMeasurement = yaw_expt;
                        // printf("go: %d\r\n",(int)yaw_expt);
                    }
                    if (event_task == 5)
                    { // 循迹
                        // turn_dirction(); 误差大
                        sound_flag = 1;
                        yaw_expt = yaw_return(base_BackDirection + 3);
                        while (fabs(jy901_yaw - yaw_expt) > 3)
                            motor_control(1050, -950);
                        motor_control(0, 0);
                        PIDController_Init(&trace_pid);
                        if (distance_1m_receive)
                        {
                            trace_flag = 1;
                            distance_1m_flag = 0;
                            defaultwheel.left.count = 0;

                            yaw_expt = base_direction;
                            nearingTheEnd = 1;
                        }
                    }
                    if (event_task == 6)
                    { // 停止
                        if (line_no_check)
                        {
                            trace_flag = 0;
                            distance_1m_flag = 0;
                            defaultwheel.left.count = 0;
                            sound_flag = 1;
                            motor_control(0, 0);
                            nearingTheEnd = 0;
                        }
                    }
                    event_task++;
                }
                if (task == 4)
                {
                    if (event_task == 0)
                    { // 角度矫正 ____ 38.66
                        task4_power_plus.goStraight_plus = 1700;     //17:15  - - - - old : 900
                        task4_power_plus.trace_line_plus = 1000; //17:15  - - - - old : 600
                        trace_pid.Kp += 0.3;
                        trace_pid.Kd += 1.4;

                        trace_flag = 0;
                        distance_1m_flag = 0;
                        defaultwheel.left.count = 0;
                        sound_flag = 1;
                        yaw_expt = yaw_return(base_direction + 38.76);
                        while (fabs(jy901_yaw - yaw_expt) > 3)
                            motor_control(950, -950);
                        motor_control(0, 0);
                        event_task++;
                    }
                    if (event_task == 1)
                    { // 定距离-直线
                        defaultwheel.left.count = 0;
                        distance_1m_flag = 1;
                        sound_flag = 1;
                        yaw_expt = yaw_return(base_direction + 38.76);
                        yaw_pid.prevError = 0;
                        yaw_pid.prevMeasurement = yaw_expt;
                    }
                    if (event_task == 2)
                    { // 循迹
                        // turn_dirction(); 误差大
                        sound_flag = 1;
                        yaw_expt = yaw_return(base_direction-1);
                        while (fabs(jy901_yaw - yaw_expt) > 3)
                            motor_control(600, 1600);

                        // motor_control(0,0);
                        // delay_ms(10);
                        // // i2c_app_read(I2C0_INST, 0x4C, &digital_data, 1);
                        // while(1) {
                        //     if(!i2c_app_read(I2C0_INST, 0x4C, &digital_data, 1)) break;
                        //     delay_ms(20);}
                        // if(trace_check(digital_data)>=0){
                        //     while(break_noout)
                        //     {
                        //         motor_control(650, 1700);
                        //         if(sys_time_1ms%10){
                        //             if(!i2c_app_read(I2C0_INST, 0x4C, &digital_data, 1)){
                        //                 if(trace_check(digital_data)>20)
                        //                     break_noout = 0;
                        //             }
                        //         }
                        //     }
                        // }
                        // break_noout = 1;

                        motor_control(500, 500);
                        PIDController_Init(&trace_pid);
                        if (distance_1m_receive)
                        {
                            trace_flag = 1;
                            distance_1m_flag = 0;
                            defaultwheel.left.count = 0;

                            yaw_expt = base_BackDirection;
                        }
                    }
                    if (event_task == 3)
                    {
                        if (line_no_check)
                        { // 角度矫正 ____ 38.66
                            trace_flag = 0;
                            distance_1m_flag = 0;
                            defaultwheel.left.count = 0;
                            sound_flag = 1;
                            yaw_expt = yaw_return(base_BackDirection -38.76);
                            while (fabs(jy901_yaw - yaw_expt) > 3)
                                motor_control(-950, 950);
                            // printf("angle ready: %d, back_dir : %d\r\n",(int)yaw_expt,(int)base_BackDirection);
                            motor_control(0, 0);
                            event_task = 4;
                        }
                    }
                    if (event_task == 4)
                    { // 定距离-直线
                        defaultwheel.left.count = 0;
                        trace_flag = 0;
                        distance_1m_flag = 1;
                        sound_flag = 1;
                        yaw_expt = yaw_return(base_BackDirection - 38.76);
                        yaw_pid.prevError = 0;
                        yaw_pid.prevMeasurement = yaw_expt;
                        // printf("go: %d\r\n", (int)yaw_expt);
                    }
                    if (event_task == 5)
                    { // 循迹
                        // turn_dirction(); 误差大
                        sound_flag = 1;
                        yaw_expt = yaw_return(base_BackDirection+1);
                        // while (fabs(jy901_yaw - yaw_expt) > 1)
                        //     motor_control(1700, 650);

                        while (fabs(jy901_yaw - yaw_expt) > 3 ){
                            motor_control(1700, 650);
                        }

                        // motor_control(0,0);
                        // delay_ms(10);
                        // while(1) {
                        //     if(!i2c_app_read(I2C0_INST, 0x4C, &digital_data, 1)) break;
                        //     printf("trace\r\n");
                        //     delay_ms(100);
                        //     }
                        // if(trace_check(digital_data)<=0){
                        //     printf("trace\r\n");
                        //     while(break_noout)
                        //     {
                        //         motor_control(1700, 650);
                        //         if(sys_time_1ms%10){
                        //             if(!i2c_app_read(I2C0_INST, 0x4C, &digital_data, 1)){
                        //                 if(trace_check(digital_data)<-20)
                        //                     break_noout = 0;
                        //             }
                        //         }
                        //     }
                        // }
                        // break_noout = 1;

                        // motor_control(0, 0);
                        PIDController_Init(&trace_pid);
                        if (distance_1m_receive)
                        {
                            trace_flag = 1;
                            distance_1m_flag = 0;
                            defaultwheel.left.count = 0;
                            yaw_expt = base_direction;
                            nearingTheEnd = 1;
                        }
                    }
                    if (event_task == 6)
                    { // 跳到0，下一次开始1
                        if (line_no_check)
                        {
                            trace_flag = 0;
                            distance_1m_flag = 0;
                            defaultwheel.left.count = 0;
                            sound_flag = 1;

                            ground++;
                            if (ground == 4)
                            {
                                trace_flag = 0;
                                distance_1m_flag = 0;
                                defaultwheel.left.count = 0;
                                sound_flag = 1;
                                motor_control(0, 0);
                                nearingTheEnd = 1;
                            }
                            else
                            {
                                yaw_expt = yaw_return(base_direction + 38.76);
                                while (fabs(jy901_yaw - yaw_expt) > 3)
                                    motor_control(950, -950);
                                motor_control(0, 0);
                                event_task = 0;
                                event_lock = 1;
                            }
                        }
                    }
                    event_task++;
                }
            if (task == 5)
                {
                    if (event_task == 0)
                    { // 角度矫正 ____ 38.66
                        trace_flag = 0;
                        distance_1m_flag = 0;
                        defaultwheel.left.count = 0;
                        sound_flag = 1;
                        yaw_expt = yaw_return(base_direction + 40.76);
                        while (fabs(jy901_yaw - yaw_expt) > 3)
                            motor_control(950, -950);
                        motor_control(0, 0);
                        event_task++;
                    }
                    if (event_task == 1)
                    { // 定距离-直线
                        defaultwheel.left.count = 0;
                        distance_1m_flag = 1;
                        sound_flag = 1;
                        yaw_expt = yaw_return(base_direction + 40.56);
                        yaw_pid.prevError = 0;
                        yaw_pid.prevMeasurement = yaw_expt;
                    }
                    if (event_task == 2)
                    { // 循迹
                        // turn_dirction(); 误差大
                        sound_flag = 1;
                        yaw_expt = yaw_return(base_direction - 1);
                        while (fabs(jy901_yaw - yaw_expt) > 3)
                            motor_control(-450, 950);
                        motor_control(500, 500);
                        PIDController_Init(&trace_pid);
                        if (distance_1m_receive)
                        {
                            trace_flag = 1;
                            distance_1m_flag = 0;
                            defaultwheel.left.count = 0;

                            yaw_expt = base_BackDirection;
                        }
                    }
                    if (event_task == 3)
                    {
                        if (line_no_check)
                        { // 角度矫正 ____ 38.66
                            trace_flag = 0;
                            distance_1m_flag = 0;
                            defaultwheel.left.count = 0;
                            sound_flag = 1;
                            yaw_expt = yaw_return(base_BackDirection -40.56);
                            while (fabs(jy901_yaw - yaw_expt) > 3)
                                motor_control(-950, 950);
                            // printf("angle ready: %d, back_dir : %d\r\n",(int)yaw_expt,(int)base_BackDirection);
                            motor_control(0, 0);
                            event_task = 4;
                        }
                    }
                    if (event_task == 4)
                    { // 定距离-直线
                        defaultwheel.left.count = 0;
                        trace_flag = 0;
                        distance_1m_flag = 1;
                        sound_flag = 1;
                        yaw_expt = yaw_return(base_BackDirection - 39.16);
                        yaw_pid.prevError = 0;
                        yaw_pid.prevMeasurement = yaw_expt;
                        printf("go: %d\r\n", (int)yaw_expt);
                    }
                    if (event_task == 5)
                    { // 循迹
                        // turn_dirction(); 误差大
                        sound_flag = 1;
                        yaw_expt = yaw_return(base_BackDirection + 1);
                        while (fabs(jy901_yaw - yaw_expt) > 3)
                            motor_control(1050, -650);
                        motor_control(0, 0);
                        PIDController_Init(&trace_pid);
                        if (distance_1m_receive)
                        {
                            trace_flag = 1;
                            distance_1m_flag = 0;
                            defaultwheel.left.count = 0;
                            yaw_expt = base_direction;
                            nearingTheEnd = 1;
                        }
                    }
                    if (event_task == 6)
                    { // 跳到0，下一次开始1
                        if (line_no_check)
                        {
                            trace_flag = 0;
                            distance_1m_flag = 0;
                            defaultwheel.left.count = 0;
                            sound_flag = 1;

                            ground++;
                            if (ground == 4)
                            {
                                trace_flag = 0;
                                distance_1m_flag = 0;
                                defaultwheel.left.count = 0;
                                sound_flag = 1;
                                motor_control(0, 0);
                                nearingTheEnd = 1;
                            }
                            else
                            {
                                yaw_expt = yaw_return(base_direction + 40.76);
                                while (fabs(jy901_yaw - yaw_expt) > 3)
                                    motor_control(950, -950);
                                motor_control(0, 0);
                                event_task = 0;
                                event_lock = 1;
                            }
                        }
                    }
                    event_task++;
                }
            }
        }
        else
            motor_control(0, 0);
        /* - - -- - - - - - - - - - -- - - - -- - - - - 事件处理 - - - - - - - - - - - - - - - -- - - - - - - - - - - - -*/

        /* 循迹  - - - - - - - 读取数字量数据 建议20ms一次*/
        if (trace_flag && !i2c_app_read(I2C0_INST, 0x4C, &digital_data, 1))
        {
            // digital_data |= 0x80;
            SEP_ALL_BIT8(digital_data,
                         gray_sensor[0],
                         gray_sensor[1],
                         gray_sensor[2],
                         gray_sensor[3],
                         gray_sensor[4],
                         gray_sensor[5],
                         gray_sensor[6],
                         gray_sensor[7]);

            //  printf("8: %d \r\n",gray_sensor[7]);
            //   printf("digital : %d\r\n",digital_data);
            int temp = trace_check(digital_data);
            PIDController_Update(&trace_pid, 0.0f, (float)(35 * temp));

            /*陀螺仪辅助*/
            if ((jy901_yaw != 0.00f))
            {
                float temp = jy901_yaw;

                float out = 0; // PIDController_yaw_Update(&yaw_pid,yaw_expt-yaw_step,temp);
            }
            trace_out = (int)(trace_pid.out);
            if(task == 4 ){
                             if(start_power_decelerate == 0 && trace_decelerate_time == 0)
                             {
                                trace_decelerate_time = sys_time_1ms + 500; //17:15  - - - - old : 500
                                start_power_decelerate = task4_power_plus.trace_line_plus;
                             }
                             else{
                                if(sys_time_1ms >= trace_decelerate_time){
                                    start_power_decelerate = 0;
                                }
                             }
                        }
            // printf("trace_out : %d \r\n", trace_out);
            motor_control(motor_BasePower + task4_power_plus.trace_line_plus - start_power_decelerate - trace_out - end_power_decelerate, \
                          motor_BasePower + task4_power_plus.trace_line_plus - start_power_decelerate + trace_out - end_power_decelerate);

            
            if (fabs(yaw_return(jy901_yaw - yaw_expt)) < 20)    //17:15  - - - - old : 20
            {
                end_power_decelerate = 600 + task4_power_plus.trace_line_plus;
                if (fabs(yaw_return(jy901_yaw - yaw_expt)) < 15 && !i2c_app_read(I2C0_INST, 0x4C, &digital_data, 1))
                {
                    if (digital_data == 0xff)
                    {
                        line_no_check = 1; // 已经检测不到黑线
                        event_lock = 1;    // 事件完成，请求指示
                        motor_control(0, 0);
                        end_power_decelerate = 0;
                        start_power_decelerate = 0 ;
                        trace_decelerate_time = 0;
                    }
                }
            }
            if (yaw_step)
                yaw_step -= 0.89f;
            delay_ms(20);
        }

        /*直线行驶*/
        if (distance_1m_flag)
        {

            // printf("1m \r\n");
            /*陀螺仪辅助*/
            if ((jy901_yaw != 0.00f))
            {
                float temp = jy901_yaw;

                float out = PIDController_yaw_Update(&yaw_pid, yaw_expt, temp);
                if (turn_on_first)
                {
                    out = 0;
                    turn_on_first++;
                }
                if (turn_on_first == 4)
                {
                    turn_on_first = 0;
                }

                // printf("yaw: %d",(int)temp);

                /*直行辅助*/
                motor_control(1760 + task4_power_plus.goStraight_plus - nearget_downpower + (int)(10 * out), \
                              1800 + task4_power_plus.goStraight_plus - nearget_downpower - (int)(10 * out));
                // motor_control(500+(int)(10*out), 500-(int)(10*out));

                delay_ms(20);
            }
            // printf("count : %d\r\n",defaultwheel.left.count);
            i2c_app_read(I2C0_INST, 0x4C, &digital_data, 1);
            if(defaultwheel.left.count >= 80 * 44 && task == 4)nearget_downpower = 450+task4_power_plus.goStraight_plus;
            if (defaultwheel.left.count >= 50 * 44 && digital_data != 0xff)
            {
                nearget_downpower = 0;  //重置降速
                distance_1m_receive = 1; // 12.1*440 ——440一圈的脉冲
                // if(task == 4){motor_control(-500,-500);delay_ms(30);}
                motor_control(0, 0);
                turn_on_first = 1;
                event_lock = 1; // 事件完成，请求指示
            }
        }
        /*角度控制*/
        // if (yaw_control_flag&&(jy901_yaw!=0.00f))
        // {
        //     float temp = jy901_yaw;

        //     // float out = PIDController_yaw_Update(&yaw_pid,yaw_expt,temp);

        //     /*直行辅助*/
        //     // motor_control(1460+3*(int)out, 1500-3*(int)out);

        //     /*转*/
        //     // float dif_dir_temp = fabs(temp - yaw_expt) ;
        //     // if(dif_dir_temp< 0.5){motor_control(0,0);base_out = 0;yaw_control_flag = 0;event_lock = 1;yaw_pid.Ki = 0;}
        //     // else if(dif_dir_temp< 5) {yaw_pid.Kp = 5;yaw_pid.Ki = 1;}
        //     // motor_control(-(int)(6*out), +(int)(6*out));
        //     // printf("dif yaw : %d\r\n",(int)dif_dir_temp);

        //     delay_ms(20);
        // }
    }
}

/*死区测试
for(int i = 0; i<900; i+=100){
     motor_control(i,i);
     printf("test : %d\r\n",i);
     delay_ms(500);
     motor_control(0,0);
     delay_ms(500);
}
for(int i = 0; i>-900; i-=100){
     motor_control(i,i);
     printf("test : %d\r\n",i);
     delay_ms(500);
     motor_control(0,0);
     delay_ms(500);
}*/

/*距离控制程序pid
if (distance_1m_flag)
{
    Total_Controller.Location_Y_Control.FeedBack = defaultwheel.left.count;
    float out = PID_Control(&Total_Controller.Location_Y_Control);
    motor_control(0.5 * out, 0.5 * out);
    printf("count: %d , output : %d \r\n", defaultwheel.left.count, (int)out);
    if (defaultwheel.left.count >= (1281 * 5 - 100))
    {
        defaultwheel.left.count = 0;
        distance_1m_flag = 0;
    }
    delay_ms(20);
}*/

// /**
//          * 串口调试部分，（接上hc06即可远程调控）
//          */
//         if (Serial_RxFlag == 1)
//         {
//             Serial_RxFlag = 0;

//             // 小车启停
//             if (memcmp(Serial_RxPacket, cmd_stop, strlen((char *)cmd_stop)) == 0)
//             {
//                 defaultwheel.left.power_multiple = 0.0f;
//                 defaultwheel.right.power_multiple = 0.0f;
//                 printf("stop\r\n");
//             }
//             else if (memcmp(Serial_RxPacket, cmd_start, strlen((char *)cmd_start)) == 0)
//             {
//                 defaultwheel.left.power_multiple = 0.5f;
//                 defaultwheel.right.power_multiple = 0.5f;
//                 printf("start\r\n");
//             }

//             // 小车动力比
//             else if (memcmp(Serial_RxPacket, cmd_power_up, strlen((char *)cmd_power_up)) == 0)
//             {
//                 defaultwheel.left.power_multiple += 0.1f;
//                 defaultwheel.right.power_multiple += 0.1f;
//                 printf("power_up: %f\r\n", defaultwheel.left.power_multiple);
//             }
//             else if (memcmp(Serial_RxPacket, cmd_power_down, strlen((char *)cmd_power_down)) == 0)
//             {
//                 defaultwheel.left.power_multiple -= 0.1f;
//                 defaultwheel.right.power_multiple -= 0.1f;
//                 printf("power_down: %f\r\n", defaultwheel.left.power_multiple);
//             }

//             // PID参数调节部分
//             // else if (memcmp(Serial_RxPacket, cmd_pid_kp_up, strlen((char *)cmd_pid_kp_up)) == 0)
//             // {
//             //     pid.Kp += 0.1f;
//             //     printf("p_up: %f\r\n", pid.Kp);
//             // }
//             // else if (memcmp(Serial_RxPacket, cmd_pid_kp_down, strlen((char *)cmd_pid_kp_down)) == 0)
//             // {
//             //     pid.Kp -= 0.1f;
//             //     printf("p_down: %f\r\n", pid.Kp);
//             // }
//             // else if (memcmp(Serial_RxPacket, cmd_pid_kd_up, strlen((char *)cmd_pid_kd_up)) == 0)
//             // {
//             //     pid.Kd += 0.1f;
//             //     printf("d_up: %f\r\n", pid.Kd);
//             // }
//             // else if (memcmp(Serial_RxPacket, cmd_pid_kd_down, strlen((char *)cmd_pid_kd_down)) == 0)
//             // {
//             //     pid.Kd -= 0.1f;
//             //     printf("d_down: %f\r\n", pid.Kd);
//             // }

//             // // pid结果输出
//             // else if (memcmp(Serial_RxPacket, cmd_show_out, strlen((char *)cmd_show_out)) == 0)
//             // {
//             //     printf("pid out : %d \r\n", (int)pid.out);
//             // }

//             // 两路adc循迹参数读取
//             else if (memcmp(Serial_RxPacket, cmd_show_adc, strlen((char *)cmd_show_adc)) == 0)
//             {
//                 printf("left: %d right: %d\r\n", adc_buf[0], adc_buf[1]);
//             }

//             // 错误调试指令
//             else
//             {
//                 printf("error send data\r\n");
//             }
//         }