#include "PID.h"
#include "math.h"
#include "stdio.h"

void PIDController_Init(PIDController *pid) {

	/* Clear controller variables */
	pid->integrator = 0.0f;
	pid->prevError  = 0.0f;

	pid->differentiator  = 0.0f;
	pid->prevMeasurement = 0.0f;

	pid->out = 0.0f;

}

float PIDController_Update(PIDController *pid, float setpoint, float measurement) {

	/*
	* Error signal
	*/
    float error = setpoint - measurement;


	/*
	* Proportional
	*/
    float proportional = pid->Kp * error;


	/*
	* Integral
	*/
    pid->integrator = pid->integrator + 0.5f * pid->Ki * pid->T * (error + pid->prevError);

	/* Anti-wind-up via integrator clamping */
    if (pid->integrator > pid->limMaxInt) {

        pid->integrator = pid->limMaxInt;

    } else if (pid->integrator < pid->limMinInt) {

        pid->integrator = pid->limMinInt;

    }

	pid->differentiator = -pid->Kd * (measurement - pid->prevMeasurement);

	/*
	* Compute output and apply limits
	*/
    pid->out = proportional + pid->integrator + pid->differentiator;

    if (pid->out > pid->limMax) {

        pid->out = pid->limMax;

    } else if (pid->out < pid->limMin) {

        pid->out = pid->limMin;

    }

	/* Store error and measurement for later use */
    pid->prevError       = error;
    pid->prevMeasurement = measurement;

	/* Return controller output */
    return pid->out;

}


float PIDController_yaw_Update(PIDController *pid, float setpoint, float measurement) {

	/*
	* Error signal
	*/
	// printf("%d\r\n",(int)measurement);
	// static bool temp = 1;
	// if(temp)temp = 0;
	// else{
	// 	if(fabs(pid->prevMeasurement - measurement)>10)measurement = pid->prevMeasurement;
	// }
	// printf("set : %d mea: %d\r\n",(int)setpoint,(int)measurement);

	   /***********************偏航角偏差超过+-180处理*****************************/
    if(setpoint <-180)  setpoint = setpoint + 360;
    if( setpoint > 180)  setpoint = setpoint - 360;


	float error = setpoint - measurement;

   /***********************偏航角偏差超过+-180处理*****************************/
    if(error <-180)  error = error + 360;
    if(error > 180)  error = error - 360;

	/*
	* Proportional
	*/
    float proportional = pid->Kp * error;


	/*
	* Integral
	*/
    pid->integrator = pid->integrator + 0.5f * pid->Ki * pid->T * (error + pid->prevError);

	/* Anti-wind-up via integrator clamping */
    if (pid->integrator > pid->limMaxInt) {

        pid->integrator = pid->limMaxInt;

    } else if (pid->integrator < pid->limMinInt) {

        pid->integrator = pid->limMinInt;

    }

	pid->differentiator = -pid->Kd * (measurement - pid->prevMeasurement);

	/*
	* Compute output and apply limits
	*/
    pid->out = proportional + pid->integrator + pid->differentiator;

    if (pid->out > pid->limMax) {

        pid->out = pid->limMax;

    } else if (pid->out < pid->limMin) {

        pid->out = pid->limMin;

    }

	/* Store error and measurement for later use */
    pid->prevError       = error;
    pid->prevMeasurement = measurement;

	/* Return controller output */
    return pid->out;

}

float yaw_return(float exp)
{
		   /***********************偏航角偏差超过+-180处理*****************************/
    if(exp <-180)  exp = exp + 360;
    if( exp > 180)  exp = exp - 360;
	return exp;
}

// AllControler Total_Controller; //总控制器PID


// /*
// 1偏差限幅标志；  2积分限幅标志；3积分分离标志；   4期望；
// 5反馈            6偏差；        7上次偏差；       8偏差限幅值；
// 9积分分离偏差值；10积分值       11积分限幅值；    12控制参数Kp；
// 13控制参数Ki；   14控制参数Kd； 15控制器总输出；  16上次控制器总输出
// 17总输出限幅度； 18变积分控制时的积分增益
// */
// const float Control_Unit[3][20] = {
//     /*                                     Kp     Ki     Kd                     */
//     /*1 2  3  4  5  6   7  8   9 10   11   12     13     14  15 16   17  18*/
//     {1, 1, 1, 0, 0, 0, 0, 180, 0, 0, 100, 4.00, 1.0000,  2.00, 0, 0,  400, 1, 1, 1}, //Yaw_Angle;偏航角   0

//     /*                                     Kp    Ki   Kd                     */
//     /*1 2  3  4  5  6   7  8   9 10   11   12    13   14  15   16  17  18*/
//     {1, 1, 1, 0, 0, 0, 0, 9000, 0, 0, 100, 1.0,  0.000, 0.2 , 0,   0,  3000, 1, 1, 1}, //Distance;距离控制	1

//     /*                                     Kp    Ki   Kd                     */
//     /*1 2  3  4  5  6   7  8   9 10   11   12    13   14  15   16  17  18*/
//     {1, 1, 1, 0, 0, 0, 0, 2000, 0,  0,  1,  2.0,  0.000, 0.5 , 0,   0,  2000, 1, 1, 1}, //Trace;循迹 2


// };


// /**
//   * @brief  PID_Init(PID参数初始化函数)
//   * @param  *Controler PID控制器
//   * @param  Label PID参数表
//   */
// void PID_Init(PID_Controler *Controler, uint8_t Label)	
// {
//     Controler->Err_Limit_Flag = (uint8_t)(Control_Unit[Label][0]);            //1偏差限幅标志
//     Controler->Integrate_Limit_Flag = (uint8_t)(Control_Unit[Label][1]);      //2积分限幅标志
//     Controler->Integrate_Separation_Flag = (uint8_t)(Control_Unit[Label][2]); //3积分分离标志
//     Controler->Expect = Control_Unit[Label][3];                               //4期望
//     Controler->FeedBack = Control_Unit[Label][4];                             //5反馈值
//     Controler->Err = Control_Unit[Label][5];                                  //6偏差
//     Controler->Last_Err = Control_Unit[Label][6];                             //7上次偏差
//     Controler->Err_Max = Control_Unit[Label][7];                              //8偏差限幅值
//     Controler->Integrate_Separation_Err = Control_Unit[Label][8];             //9积分分离偏差值
//     Controler->Integrate = Control_Unit[Label][9];                            //10积分值
//     Controler->Integrate_Max = Control_Unit[Label][10];                       //11积分限幅值
//     Controler->Kp = Control_Unit[Label][11];                                  //12控制参数Kp
//     Controler->Ki = Control_Unit[Label][12];                                  //13控制参数Ki
//     Controler->Kd = Control_Unit[Label][13];                                  //14控制参数Ki
//     Controler->Control_OutPut = Control_Unit[Label][14];                      //15控制器总输出
//     Controler->Last_Control_OutPut = Control_Unit[Label][15];                 //16上次控制器总输出
//     Controler->Control_OutPut_Limit = Control_Unit[Label][16];                //17上次控制器总输出
//     Controler->Scale_Kp = Control_Unit[Label][17];
//     Controler->Scale_Ki = Control_Unit[Label][18];
//     Controler->Scale_Kd = Control_Unit[Label][19];
// }

// void Total_PID_Init(void){
// 	PID_Init(&Total_Controller.Yaw_Angle_Control,0);
// 	PID_Init(&Total_Controller.Location_Y_Control,1);
//     PID_Init(&Total_Controller.Trace_Control,2);
// }

// /**
//   * @brief  通用 PID 控制器
//   * @param *Controler 对应 PID 控制器
//   */
// float PID_Control(PID_Controler *Controler)
// {
//     /*******偏差计算*********************/
//     Controler->Last_Err = Controler->Err;                     //保存上次偏差
//     Controler->Err = Controler->Expect - Controler->FeedBack; //期望减去反馈得到偏差
//     if (Controler->Err_Limit_Flag == 1)                       //偏差限幅度标志位
//     {
//         Controler->Err = constrain(Controler->Err, -Controler->Err_Max, Controler->Err_Max);
//     }
//     /*******积分计算*********************/
//     if (Controler->Integrate_Separation_Flag == 1) //积分分离标志位
//     {
//         if (fabs(Controler->Err) <= Controler->Integrate_Separation_Err)
//             Controler->Integrate += Controler->Scale_Ki * Controler->Ki * Controler->Err;
//     }
//     else
//     {
//         Controler->Integrate += Controler->Scale_Ki * Controler->Ki * Controler->Err;
//     }
//     /*******积分限幅*********************/
//     if (Controler->Integrate_Limit_Flag == 1) //积分限制幅度标志
//     {
//         Controler->Integrate = constrain(Controler->Integrate, -Controler->Integrate_Max, Controler->Integrate_Max);
//     }
//     /*******总输出计算*********************/
//     Controler->Last_Control_OutPut = Controler->Control_OutPut; //输出值递推
//     Controler->Control_OutPut =
//         Controler->Scale_Kp * Controler->Kp * Controler->Err      //比例
//         + Controler->Integrate                                    //积分
//         + Controler->Kd * (Controler->Err - Controler->Last_Err); //微分
//     /*******总输出限幅*********************/
//     Controler->Control_OutPut = constrain(Controler->Control_OutPut, -Controler->Control_OutPut_Limit, Controler->Control_OutPut_Limit);
//     /*******返回总输出*********************/
//     return Controler->Control_OutPut;
// }

// /**
//   * @brief  角度 PID 控制器
//   * @param *Controler 对应 PID 控制器
//   */
// float PID_Control_Yaw(PID_Controler *Controler)
// {
//     /*******偏差计算*********************/
//     Controler->Last_Err = Controler->Err;                     //保存上次偏差
//     Controler->Err = Controler->Expect - Controler->FeedBack; //期望减去反馈得到偏差  FeedBack
//     /***********************偏航角偏差超过+-180处理*****************************/
//     if(Controler->Err <-180)  Controler->Err = Controler->Err + 360;
//     if(Controler->Err > 180)  Controler->Err = Controler->Err - 360;

//     if (Controler->Err_Limit_Flag == 1) //偏差限幅度标志位
//     {
//         Controler->Err = constrain(Controler->Err, -Controler->Err_Max, Controler->Err_Max);
//     }
//     /*******积分计算*********************/
//     if (Controler->Integrate_Separation_Flag == 1) //积分分离标志位
//     {
//         if (fabs(Controler->Err) <= Controler->Integrate_Separation_Err)
//             Controler->Integrate += Controler->Scale_Ki * Controler->Ki * Controler->Err;
//     }
//     else
//     {
//         Controler->Integrate += Controler->Scale_Ki * Controler->Ki * Controler->Err;
//     }
//     /*******积分限幅*********************/
//     if (Controler->Integrate_Limit_Flag == 1) //积分限制幅度标志
//     {
//         Controler->Integrate = constrain(Controler->Integrate, -Controler->Integrate_Max, Controler->Integrate_Max);
//     }
//     /*******总输出计算*********************/
//     Controler->Last_Control_OutPut = Controler->Control_OutPut; //输出值递推
//     Controler->Control_OutPut =
//         Controler->Scale_Kp * Controler->Kp * Controler->Err      //比例  位置式
//         + Controler->Integrate                                    //积分
//         + Controler->Kd * (Controler->Err - Controler->Last_Err); //微分
//     /*******总输出限幅*********************/
//     Controler->Control_OutPut = constrain(Controler->Control_OutPut, -Controler->Control_OutPut_Limit, Controler->Control_OutPut_Limit);
//     /*******返回总输出*********************/
//     return Controler->Control_OutPut;
// }




