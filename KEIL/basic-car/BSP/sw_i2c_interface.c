
#include "sw_i2c_interface.h"
#include "bsp_mpu6050.h"

/* 定义sda输出函数 bit=0为低电平 bit=1为高电平 */
void sw_sda_out(uint8_t bit, void *user_data)
{   
    SDA_OUT();
    SDA(bit);
//	GPIO_WriteBit(GPIOB, SW_I2C1_PIN_SDA, (BitAction)bit);      //stm32 标准库
	
	/* IIC软件延迟 */
	delay_us(10);
}

/* 定义sda读取函数 bit 为返回的电平值 */
uint8_t sw_sda_in(void *user_data)
{
	uint8_t bit;
    SDA_IN();
    bit = SDA_GET();
//	bit = (uint8_t)GPIO_ReadInputDataBit(GPIOB, SW_I2C1_PIN_SDA);      //stm32 标准库
	
	/* IIC软件延迟 */
	delay_us(10);
	return bit;
}

/* 定义scl时钟输出函数 bit=0为低电平 bit=1为高电平 */
void sw_scl_out(uint8_t bit, void *user_data)
{
    
    SCL(bit);
//	GPIO_WriteBit(GPIOB, SW_I2C1_PIN_SCL, (BitAction)bit);      //stm32 标准库
	
	/* IIC软件延迟 */
	delay_us(10);
}









