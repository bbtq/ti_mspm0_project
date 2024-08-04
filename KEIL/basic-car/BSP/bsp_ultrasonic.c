/*
 * 立创开发板软硬件资料与相关扩展板软硬件资料官网全部开源
 * 开发板官网：www.lckfb.com
 * 技术支持常驻论坛，任何技术问题欢迎随时交流学习
 * 立创论坛：https://oshwhub.com/forum
 * 关注bilibili账号：【立创开发板】，掌握我们的最新动态！
 * 不靠卖板赚钱，以培养中国工程师为己任
 * Change Logs:
 * Date           Author       Notes
 * 2024-05-27     LCKFB-LP    first version
 */

#include "bsp_ultrasonic.h"


bool SR04_LEN_GET_FLAG = 0;

volatile unsigned int msHcCount = 0;//ms计数
volatile float distance = 0;

/******************************************************************
 * 函 数 名 称：bsp_ultrasonic
 * 函 数 说 明：超声波初始化
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 作       者：LC
 * 备       注：TRIG引脚负责发送超声波脉冲串
******************************************************************/
void Ultrasonic_Init(void)
{
        
    SYSCFG_DL_init();
    //清除定时器中断标志
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    //使能定时器中断
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    //开启超声波的GPIOA端口中断
    NVIC_EnableIRQ(SR04_INT_IRQN);
        
}
/******************************************************************
 * 函 数 名 称：Open_Timer
 * 函 数 说 明：打开定时器
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 作       者：LC
 * 备       注：
******************************************************************/
void Open_Timer(void)
{
        
    DL_TimerG_setTimerCount(TIMER_0_INST, 0);   // 清除定时器计数  
        
    msHcCount = 0;  
        
    DL_TimerG_startCounter(TIMER_0_INST);   // 使能定时器
}

/******************************************************************
 * 函 数 名 称：Get_TIMER_Count
 * 函 数 说 明：获取定时器定时时间
 * 函 数 形 参：无
 * 函 数 返 回：数据
 * 作       者：LC
 * 备       注：
******************************************************************/
uint32_t Get_TIMER_Count(void)
{
    uint32_t time  = 0;  
    time   = msHcCount*100;                         // 得到us 
    time  += DL_TimerG_getTimerCount(TIMER_0_INST);  // 得到ms 
        
    DL_TimerG_setTimerCount(TIMER_0_INST, 0);   // 清除定时器计数  
    delay_ms(10);
    return time ;          
}

/******************************************************************
 * 函 数 名 称：Close_Timer
 * 函 数 说 明：关闭定时器
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 作       者：LC
 * 备       注：
******************************************************************/
void Close_Timer(void)
{
    DL_TimerG_stopCounter(TIMER_0_INST);     // 关闭定时器 
}

/******************************************************************
 * 函 数 名 称：TIMER_0_INST_IRQHandler
 * 函 数 说 明：定时器中断服务函数
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 作       者：LC
 * 备       注：1ms进入一次
******************************************************************/
void TIMER_0_INST_IRQHandler(void)
{
    //如果产生了定时器中断
    switch( DL_TimerG_getPendingInterrupt(TIMER_0_INST) )
    {
        case DL_TIMER_IIDX_ZERO://如果是0溢出中断
                                msHcCount++;
            break;
        
        default://其他的定时器中断
            break;
    }
}


/******************************************************************
 * 函 数 名 Hcsr04StartGet
 * 函 数 说 明：开始测量距离
 * 函 数 形 参：无
 * 函 数 返 回：测量距离
 * 作       者：bbtq
 * 备       注：无
******************************************************************/
void Hcsr04Start(void)
{
    Close_Timer();

    SR04_LEN_GET_FLAG = 0;

    SR04_TRIG(0);//trig拉低信号，发出低电平s   
    delay_1us(10);//持续时间超过5us                        
            
    SR04_TRIG(1);//trig拉高信号，发出高电平
            
    delay_1us(20);//持续时间超过10us
            
    SR04_TRIG(0);//trig拉低信号，发出低电平
    /*Echo发出信号 等待回响信号*/
                /*输入方波后，模块会自动发射8个40KHz的声波，与此同时回波引脚（echo）端的电平会由0变为1；
                （此时应该启动定时器计时）；当超声波返回被模块接收到时，回波引 脚端的电平会由1变为0；
                （此时应该停止定时器计数），定时器记下的这个时间即为
                                                超声波由发射到返回的总时长；*/
}

/******************************************************************
 * 函 数 名 称：Hcsr04GetLength
 * 函 数 说 明：获取测量距离
 * 函 数 形 参：无
 * 函 数 返 回：测量距离
 * 作       者：bbtq
 * 备       注：无
******************************************************************/
volatile float length = 0;
float Hcsr04GetLength(void)
{
    SR04_LEN_GET_FLAG = 0;
    return length;
}

void GROUP1_IRQHandler(void)//Group1的中断服务函数
{
    //读取Group1的中断寄存器并清除中断标志位
    switch( DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1) )
    {
        //检查是否是KEY的GPIOA端口中断，注意是INT_IIDX，不是PIN_18_IIDX 
        case SR04_INT_IIDX:
            //如果按键按下变为高电平
            if( DL_GPIO_readPins(SR04_PORT, SR04_ECHO_PIN) > 0 )
            {
                //设置LED引脚状态翻转
                Open_Timer();   //打开定时器 
                // printf("sr04 interrpt\r\n");
            }
            if( DL_GPIO_readPins(SR04_PORT, SR04_ECHO_PIN) == 0 )
            {
                Close_Timer();   // 关闭定时器 
                length = (float)Get_TIMER_Count() / 58.0f;   // cm  
                SR04_LEN_GET_FLAG = 1;
            }
        break;
    }
}

