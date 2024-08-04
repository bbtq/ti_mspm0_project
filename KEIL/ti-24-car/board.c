/*
 * 立创开发板软硬件资料与相关扩展板软硬件资料官网全部开源
 * 开发板官网：www.lckfb.com
 * 技术支持常驻论坛，任何技术问题欢迎随时交流学习
 * 立创论坛：https://oshwhub.com/forum
 * 关注bilibili账号：【立创开发板】，掌握我们的最新动态！
 * 不靠卖板赚钱，以培养中国工程师为己任
 * Change Logs:
 * Date           Author       Notes
 * 2024-05-23     LCKFB     first version
 */
#include "board.h"
#include "stdio.h"
#include "bsp_jy901.h"
#include "motor.h"

bool sound_flag = 0;
bool start_flag = 0;
uint8_t task = 0;
uint32_t sys_time_1ms = 0;
float base_direction = 0;
float base_BackDirection = 0;
volatile unsigned int delay_times = 0;
volatile unsigned char uart_data = 0;

void board_init(void)
{
	// SYSCFG初始化
	SYSCFG_DL_init();

	// 清除定时器中断标志
	NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
	// 使能定时器中断
	NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

	// 清除串口中断标志
	NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
	// 使能串口中断
	NVIC_EnableIRQ(UART_0_INST_INT_IRQN);

	NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOA_INT_IRQN); // 开启霍尔编码器电机的GPIOA端口中断

	printf("Board Init [[ ** LCKFB ** ]]\r\n");
}

// 搭配滴答定时器实现的精确us延时
void delay_us(unsigned long __us)
{
	uint32_t ticks;
	uint32_t told, tnow, tcnt = 38;

	// 计算需要的时钟数 = 延迟微秒数 * 每微秒的时钟数
	ticks = __us * (32000000 / 1000000);

	// 获取当前的SysTick值
	told = SysTick->VAL;

	while (1)
	{
		// 重复刷新获取当前的SysTick值
		tnow = SysTick->VAL;

		if (tnow != told)
		{
			if (tnow < told)
				tcnt += told - tnow;
			else
				tcnt += SysTick->LOAD - tnow + told;

			told = tnow;

			// 如果达到了需要的时钟数，就退出循环
			if (tcnt >= ticks)
				break;
		}
	}
}
// 搭配滴答定时器实现的精确ms延时
void delay_ms(unsigned long ms)
{
	delay_us(ms * 1000);
}

void delay_1us(unsigned long __us) { delay_us(__us); }
void delay_1ms(unsigned long ms) { delay_ms(ms); }

// 串口发送单个字符
void uart0_send_char(char ch)
{
	// 当串口0忙的时候等待，不忙的时候再发送传进来的字符
	while (DL_UART_isBusy(UART_0_INST) == true)
		;
	// 发送单个字符
	DL_UART_Main_transmitData(UART_0_INST, ch);
}
// 串口发送字符串
void uart0_send_string(char *str)
{
	// 当前字符串地址不在结尾 并且 字符串首地址不为空
	while (*str != 0 && str != 0)
	{
		// 发送字符串首地址中的字符，并且在发送完成之后首地址自增
		uart0_send_char(*str++);
	}
}

#if !defined(__MICROLIB)
// 不使用微库的话就需要添加下面的函数
#if (__ARMCLIB_VERSION <= 6000000)
// 如果编译器是AC5  就定义下面这个结构体
struct __FILE
{
	int handle;
};
#endif

FILE __stdout;

// 定义_sys_exit()以避免使用半主机模式
void _sys_exit(int x)
{
	x = x;
}
#endif

// printf函数重定义
int fputc(int ch, FILE *stream)
{
	// 当串口0忙的时候等待，不忙的时候再发送传进来的字符
	while (DL_UART_isBusy(UART_0_INST) == true)
		;

	DL_UART_Main_transmitData(UART_0_INST, ch);

	return ch;
}

// 定时器的中断服务函数 已配置为1秒的周期
void TIMER_0_INST_IRQHandler(void)
{
	static uint32_t exp_time = 0;
	// 如果产生了定时器中断
	switch (DL_TimerG_getPendingInterrupt(TIMER_0_INST))
	{
	case DL_TIMER_IIDX_ZERO: // 如果是0溢出中断 1ms定时已到
		/*计算速度，并重置计数，供下一次计算*/
		// printf("l count : %d \r\n",defaultwheel.left.count);
		// defaultwheel.left.count = 0;
		if(sound_flag){
			exp_time = sys_time_1ms+300;
			DL_GPIO_setPins(LED1_PORT,LED1_PIN_14_PIN);
			sound_flag = 0;
		}
		if(exp_time == sys_time_1ms){
			DL_GPIO_clearPins(LED1_PORT,LED1_PIN_14_PIN);
		}
		// if(sys_time_1ms%20 == 0) printf("yaw: %d\r\n",(int)jy901_yaw);
		sys_time_1ms++;

		break;

	default: // 其他的定时器中断
		break;
	}
}

void led_flashing(uint8_t time)
{
	for (uint8_t i = 0; i < time; i++)
	{
		DL_GPIO_togglePins(LED1_PORT, LED1_PIN_14_PIN);
		delay_ms(100);
		DL_GPIO_togglePins(LED1_PORT, LED1_PIN_14_PIN);
		delay_ms(100);
	}
}

void GROUP1_IRQHandler(void) // Group1的中断服务函数
{
	static bool key_lock = 0;
	// 读取Group1的中断寄存器并清除中断标志位
	switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1))
	{
	case GPIO_MULTIPLE_GPIOA_INT_IIDX:
		// 检查是否是霍尔编码器的脉冲
		if (DL_GPIO_readPins(motor_count_PORT, motor_count_L_A_PIN) > 0)
		{
			defaultwheel.left.count++;
		}
		if (DL_GPIO_readPins(motor_count_PORT, motor_count_R_A_PIN) > 0)
		{
			defaultwheel.right.count++;
		}
		if (KEY_PIN_18_IIDX)
		{
			if(!key_lock){
				delay_ms(5);
				if(DL_GPIO_readPins(KEY_PORT, KEY_PIN_18_PIN) != 0){
					delay_ms(600);
					if (DL_GPIO_readPins(KEY_PORT, KEY_PIN_18_PIN) != 0)
					{
						if(task == 0){	//记录反向方位
							DL_GPIO_togglePins(LED1_PORT, LED1_PIN_14_PIN);
							
							for (uint8_t i = 0; i < 10; i++)
							{
								
								base_BackDirection += jy901_yaw;
								delay_ms(20);
							}
							base_BackDirection /= 10.0f;
							delay_ms(600);
							DL_GPIO_togglePins(LED1_PORT, LED1_PIN_14_PIN);
						}
						else {		//记录正向方位
							DL_GPIO_togglePins(LED1_PORT, LED1_PIN_14_PIN);
							start_flag = 1;
							for (uint8_t i = 0; i < 10; i++)
							{
								while (jy901_yaw == 0.00)
								; // 等待初始化
								base_direction += jy901_yaw;
								delay_ms(20);
							}
							base_direction /= 10.0f;
							key_lock = 1;
							delay_ms(600);
							DL_GPIO_togglePins(LED1_PORT, LED1_PIN_14_PIN);
							// printf("start task%d\r\n",task);
						}
					}
					else if(DL_GPIO_readPins(KEY_PORT, KEY_PIN_18_PIN)==0){
						task++;
						led_flashing(task);	
					}
				}
			}
		}
		break;
	}
}

// 串口的中断服务函数
char Serial_RxPacket[100]; //"@MSG\r\n"
uint8_t Serial_RxFlag;
void UART_0_INST_IRQHandler(void)
{
	static uint8_t RxState = 0;
	static uint8_t pRxPacket = 0;
	uint8_t RxData = 0;

	// 如果产生了串口中断
	switch (DL_UART_getPendingInterrupt(UART_0_INST))
	{
	case DL_UART_IIDX_RX: // 如果是接收中断

		// 接发送过来的数据保存在变量中
		RxData = DL_UART_Main_receiveData(UART_0_INST);
		copeJY901_data(RxData);

		// 将保存的数据再发送出去
		//			uart0_send_char(RxData);

		if (RxState == 0)
		{
			if (RxData == '@' && Serial_RxFlag == 0)
			{
				RxState = 1;
				pRxPacket = 0;
			}
		}
		else if (RxState == 1)
		{
			if (RxData == '*')
			{
				RxState = 2;
			}
			else
			{
				Serial_RxPacket[pRxPacket] = RxData;
				pRxPacket++;
			}
		}
		else if (RxState == 2)
		{
			if (RxData == '#')
			{
				RxState = 0;
				Serial_RxPacket[pRxPacket] = '\n';
				Serial_RxFlag = 1;
			}
		}
		break;

	default: // 其他的串口中断
		break;
	}
}
