
/*
 * 通用软件IIC使用方式
 * 1.首先需要定义三个函数
 * 
 * 根据bit=0 或者bit=1 在SDA阵脚输出对应的电平，然后根据自己需要的IIC频率delay N us
 * void sda_out(uint8_t bit, void *user_data)
 *
 * 读取SDA电平值然后返回低电平0 或者是高电平1，然后根据自己需要的IIC频率delay N us
 * uint8_t sda_in(void *user_data)
 *
 * 根据bit=0 或者bit=1 在SCL阵脚输出对应的电平，然后根据自己需要的IIC频率delay N us
 * void scl_out(uint8_t bit, void *user_data)
 *
 * 2. 把上面定义的函数 塞进 sw_i2c_interface_t
 * 3. 然后就可以把 sw_i2c_interface_t 传给函数使用了
 */

#ifndef _SW_I2C_H_
#define _SW_I2C_H_


#include <stdint.h>

typedef struct {
	void (*sda_out)(uint8_t bit, void *user_data);
	uint8_t (*sda_in)(void *user_data);
	void (*scl_out)(uint8_t bit, void *user_data);
	void *user_data;
} sw_i2c_interface_t;

/**
 * @brief 从IIC总线上的设备读取多个字节
 * @param i2c_interface
 * @param dev_addr 从设备地址
 * @param[out] data 读取到的字节数组
 * @param data_length 读取大小(字节)
 * @return 0:成功, 1:错误
 */
int8_t sw_i2c_read(sw_i2c_interface_t *i2c_interface, uint8_t dev_addr, uint8_t *data, uint8_t data_length);
int8_t sw_i2c_write(sw_i2c_interface_t *i2c_interface, uint8_t dev_addr, const uint8_t *data, uint8_t data_length);

/**
 * @brief 从IIC总线上的设备读取一个字节
 * @param i2c_interface
 * @param dev_addr 从设备地址
 * @param[out] data 读取到的字节
 * @return 0:成功, 1:错误
 */
int8_t sw_i2c_read_byte(sw_i2c_interface_t *i2c_interface, uint8_t dev_addr, uint8_t *data);
int8_t sw_i2c_write_byte(sw_i2c_interface_t *i2c_interface, uint8_t dev_addr, const uint8_t data);

/**
 * @brief 读取IIC总线从设备的寄存器数据. 即先写入寄存器地址,无终止位,再连续读取所需数据
 * @param i2c_interface
 * @param dev_addr 从设备地址
 * @param mem_addr 寄存器地址
 * @param[out] data 读取到的字节数组
 * @param data_length 读取大小(字节),不包括寄存器地址本身
 * @return 0:成功, 1:错误
 */
int8_t sw_i2c_mem_read(sw_i2c_interface_t *i2c_interface, uint8_t dev_addr, uint8_t mem_addr, uint8_t *data, uint8_t data_length);

/**
 * @brief 写入IIC总线从设备的寄存器. 即先写入寄存器地址,再连续写入数组中的数据
 * @param i2c_interface
 * @param dev_addr 从设备地址
 * @param mem_addr 寄存器地址
 * @param[out] data 连续写入的数据
 * @param data_length 所写入的字节大小,不包括寄存器地址本身
 * @return 0:成功, 1:错误
 */
int8_t sw_i2c_mem_write(sw_i2c_interface_t *i2c_interface, uint8_t dev_addr, uint8_t mem_addr, const uint8_t *data, uint8_t data_length);

/**
 * i2c地址扫描
 * @param scan_addr 扫描出来的地址存放,数值不为0的为扫描到的地址，扫到的地址会挨个放在数组的最前面
 * @return 返回扫描到的设备数量, 0为无设备发现
 */
uint8_t i2c_scan(sw_i2c_interface_t *i2c_interface, uint8_t *scan_addr);



#endif
