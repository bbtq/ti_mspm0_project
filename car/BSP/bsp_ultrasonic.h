/*
 * ������������Ӳ�������������չ����Ӳ�����Ϲ���ȫ����Դ
 * �����������www.lckfb.com
 * ����֧�ֳ�פ��̳���κμ������⻶ӭ��ʱ����ѧϰ
 * ������̳��https://oshwhub.com/forum
 * ��עbilibili�˺ţ������������塿���������ǵ����¶�̬��
 * ��������׬Ǯ���������й�����ʦΪ����
 * Change Logs:
 * Date           Author       Notes
 * 2024-05-27     LCKFB-LP    first version
 */
#ifndef _BSP_ULTRASONIC_H_
#define _BSP_ULTRASONIC_H_

#include "board.h"

extern bool SR04_LEN_GET_FLAG;

#define SR04_TRIG(x)  ( x ? DL_GPIO_setPins(SR04_PORT,SR04_TRIG_PIN) : DL_GPIO_clearPins(SR04_PORT,SR04_TRIG_PIN) )
#define SR04_ECHO()   ( ( ( DL_GPIO_readPins(SR04_PORT,SR04_ECHO_PIN) & SR04_ECHO_PIN ) > 0 ) ? 1 : 0 )

void Ultrasonic_Init(void);//��������ʼ�� 
void Hcsr04Start(void);//�������������
float Hcsr04GetLength(void);//��ȡ��������������

#endif
