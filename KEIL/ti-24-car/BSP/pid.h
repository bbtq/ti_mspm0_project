#ifndef _PID_H_
#define _PID_H_

#include "board.h"

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt))) //限幅函数



typedef struct {

	/* Controller gains */
	float Kp;
	float Ki;
	float Kd;

	/* Derivative low-pass filter time constant */
	float tau;

	/* Output limits */
	float limMin;
	float limMax;
	
	/* Integrator limits */
	float limMinInt;
	float limMaxInt;

	/* Sample time (in seconds) */
	float T;

	/* Controller "memory" */
	float integrator;
	float prevError;			/* Required for integrator */
	float differentiator;
	float prevMeasurement;		/* Required for differentiator */

	/* Controller output */
	float out;

} PIDController;


void  PIDController_Init(PIDController *pid);
float PIDController_Update(PIDController *pid, float setpoint, float measurement);
float PIDController_yaw_Update(PIDController *pid, float setpoint, float measurement);
float yaw_return(float exp);


#endif





// /* 单 PID 控制器参数结构体 */
// typedef struct
// {
//     uint8_t Err_Limit_Flag : 1;            //偏差限幅标志   位定义
//     uint8_t Integrate_Limit_Flag : 1;      //积分限幅标志
//     uint8_t Integrate_Separation_Flag : 1; //积分分离标志
//     float Expect;                          //期望
//     float FeedBack;                        //反馈值
//     float Err;                             //偏差
//     float Last_Err;                        //上次偏差
//     float Err_Max;                         //偏差限幅值
//     float Integrate_Separation_Err;        //积分分离偏差值
//     float Integrate;                       //积分值
//     float Integrate_Max;                   //积分限幅值
//     float Kp;                              //控制参数Kp  12
//     float Ki;                              //控制参数Ki  13
//     float Kd;                              //控制参数Kd  14
//     float Control_OutPut;                  //控制器总输出
//     float Last_Control_OutPut;             //上次控制器总输出
//     float Control_OutPut_Limit;            //输出限幅
//     /***************************************/
//     float Pre_Last_Err;         //上上次偏差
//     float Adaptable_Kd;         //自适应微分参数
//     float Last_FeedBack;        //上次反馈值
//     float Dis_Err;              //微分量
//     float Dis_Error_History[5]; //历史微分量
//     float Err_LPF;
//     float Last_Err_LPF;
//     float Dis_Err_LPF;
//     float Last_Dis_Err_LPF;
//     float Pre_Last_Dis_Err_LPF;
//     float Scale_Kp;
//     float Scale_Ki;
//     float Scale_Kd;
    
// } PID_Controler;

// /* 总 PID 控制器参数结构体 */
// typedef struct
// {
//     PID_Controler Yaw_Angle_Control;    //偏航角度

//     PID_Controler Location_Y_Control;   //Distance 距离控制

//     PID_Controler Trace_Control;   //Trace 循迹控制

// } AllControler;

// extern AllControler Total_Controller; //总控制器PID

// void Total_PID_Init(void);
// float PID_Control(PID_Controler *Controler);
// float PID_Control_Yaw(PID_Controler *Controler);
