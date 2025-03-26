#include "Motor.h"
#include "main.h"

int target_angle1=0,target_speed1=0;
int target_angle3=0,target_speed3=0;

void Motor_Set(uint8_t rx_data)
{
	if(rx_data == 0x00)//停止
	{
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, CIN1_Pin, GPIO_PIN_RESET);HAL_GPIO_WritePin(GPIOA, CIN2_Pin, GPIO_PIN_RESET);
		target_speed1=0;
		target_speed3=0;
	}
	
	else if(rx_data == 0x01)//定速前进10
	{
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, CIN1_Pin, GPIO_PIN_RESET);HAL_GPIO_WritePin(GPIOA, CIN2_Pin, GPIO_PIN_SET);
		target_speed1=284;
		target_speed3=-284;
	}else if(rx_data == 0xF1)//定位移前进
	{
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, CIN1_Pin, GPIO_PIN_RESET);HAL_GPIO_WritePin(GPIOA, CIN2_Pin, GPIO_PIN_SET);
		target_angle1=1440;
		target_angle3=-1440;
	}
	
	else if(rx_data == 0x02)//定速后退
	{
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, CIN1_Pin, GPIO_PIN_SET);HAL_GPIO_WritePin(GPIOA, CIN2_Pin, GPIO_PIN_RESET);
		target_speed1=-284;
		target_speed3=-284;
	}else if(rx_data == 0xF2)//定位移后退
	{
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, CIN1_Pin, GPIO_PIN_SET);HAL_GPIO_WritePin(GPIOA, CIN2_Pin, GPIO_PIN_RESET);
		target_angle1=-1440;
		target_angle3=-1440;
	}
	
	else if(rx_data == 0x03)//定速左移
	{
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, CIN1_Pin, GPIO_PIN_RESET);HAL_GPIO_WritePin(GPIOA, CIN2_Pin, GPIO_PIN_SET);
		
	}else if(rx_data == 0x03)//定位移左移
	{
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, CIN1_Pin, GPIO_PIN_RESET);HAL_GPIO_WritePin(GPIOA, CIN2_Pin, GPIO_PIN_SET);
	}
	
	else if(rx_data == 0x04)//右移
	{
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, CIN1_Pin, GPIO_PIN_SET);HAL_GPIO_WritePin(GPIOA, CIN2_Pin, GPIO_PIN_RESET);
	}
	
	else if(rx_data == 0x05)//左转弯
	{
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, CIN1_Pin, GPIO_PIN_SET);HAL_GPIO_WritePin(GPIOA, CIN2_Pin, GPIO_PIN_RESET);
	}
	
	else if(rx_data == 0x06)//右转弯
	{
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, CIN1_Pin, GPIO_PIN_RESET);HAL_GPIO_WritePin(GPIOA, CIN2_Pin, GPIO_PIN_SET);
	}
	
	else if(rx_data == 0x07)//左前
	{
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, CIN1_Pin, GPIO_PIN_RESET);HAL_GPIO_WritePin(GPIOA, CIN2_Pin, GPIO_PIN_SET);
	}
	
	else if(rx_data == 0x08)//左后
	{
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, CIN1_Pin, GPIO_PIN_RESET);HAL_GPIO_WritePin(GPIOA, CIN2_Pin, GPIO_PIN_RESET);
	}
	
	else if(rx_data == 0x09)//右前
	{
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, CIN1_Pin, GPIO_PIN_RESET);HAL_GPIO_WritePin(GPIOA, CIN2_Pin, GPIO_PIN_RESET);
	}
	
	else if(rx_data == 0x10)//右后
	{
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, CIN1_Pin, GPIO_PIN_SET);HAL_GPIO_WritePin(GPIOA, CIN2_Pin, GPIO_PIN_RESET);
	}
	
}







