/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MPU6050.h"
#include "pid.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float pitch=0,roll=0,yaw=0;//用到yaw轴角度
float temp1=0,temp3=0;//中间值，读取电机速度、角度时用到
int ret=0,delta1=0,delta3=0,cnt1=0,cnt3=0;//MPU6050初始化时用到的参数，当前角度与目标角度差值，CNT
int speed1=0,angle1=0,target_angle1=0,real_target_speed1=0,target_speed1=0,pwm_out1=0,sum_angle1=0;
int speed3=0,angle3=0,target_angle3=0,real_target_speed3=0,target_speed3=0,pwm_out3=0,sum_angle3=0;//当前速度、角度，目标角度、速度、PWM占空比
int PID_mode=1,yaw_flag=0;//PID模式，yaw轴闭环模式
const fp32 PID_speed1[3]={0.12,0.01,0};//内环速度环Kp、Ki、Kd参数
const fp32 PID_position1[3]={4.0,0.0024,0};//外环角度环Kp、Ki、Kd参数
const fp32 PID_speed3[3]={0.12,0.01,0};
const fp32 PID_position3[3]={4.0,0.0024,0};
pid_type_def Position1,Speed1;//外环角度环结构体和内环速度环结构体
pid_type_def Position3,Speed3;
volatile uint8_t rx_flag1 = 0;
volatile uint8_t rx_data1;//串口1是否接收到数据的标志位，串口一传输的数据
volatile uint8_t rx_flag3 = 0;
volatile uint8_t rx_data3;
static CAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[1] = {0x00};
uint32_t TxMailbox;//CAN通信配置，用于对yaw轴闭环，当板子一判断符合条件时向板子二发送0x00停止信号
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) 
{	
		
	if(htim->Instance == TIM4)
	{
		cnt1 = __HAL_TIM_GET_COUNTER(&htim2);
		cnt3 = __HAL_TIM_GET_COUNTER(&htim3);	
		
		if(cnt1>=30000)	{cnt1=cnt1-65535;}
		temp1 = ( cnt1 * 360 * 100 ) / (4 * 13 * 34 ) ;
		speed1 = (int)temp1;
		sum_angle1 = sum_angle1 + (int)temp1*0.01;//测出电机1当时速度、角度
				
		if(cnt3>=30000)	{cnt3=cnt3-65535;}
		temp3 = ( cnt3 * 360 * 100 ) / (4 * 13 * 34 ) ;
		speed3 = (int)temp3;
		sum_angle3 = sum_angle3 + (int)temp3*0.01;//测出电机2当时速度、角度
	
		if(delta1<=30 && delta1>=-30 && PID_mode==1)
		{
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,0);//达到条件电机停止
		}else
		{	
			target_speed1=PID_calc(&Position1,0,delta1);
			if(PID_mode==0)	{target_speed1=real_target_speed1;}
			pwm_out1=PID_calc(&Speed1,speed1,target_speed1);//PID计算
			if(pwm_out1>0)//pwm_ou1t>0时电机正转，反之反转
			{
				HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
			}else
			{
				HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
				pwm_out1=(-1)*pwm_out1;
			}
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,pwm_out1);
		}
		
		if(delta3<=30 && delta3>=-30 && PID_mode==1)
		{
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,0);//达到条件电机停止
		}else
		{
			target_speed3=PID_calc(&Position3,0,delta3);
			if(PID_mode==0)	{target_speed3=real_target_speed3;}
			pwm_out3=PID_calc(&Speed3,speed3,target_speed3);//PID计算
			if(pwm_out3>0)//pwm_out3>0时电机正转，反之反转
			{
				HAL_GPIO_WritePin(GPIOA, CIN1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA, CIN2_Pin, GPIO_PIN_RESET);
			}else
			{
				HAL_GPIO_WritePin(GPIOA, CIN1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, CIN2_Pin, GPIO_PIN_SET);
				pwm_out3=(-1)*pwm_out3;
			}
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,pwm_out3);
		}
	
	__HAL_TIM_SetCounter(&htim2,0);
	__HAL_TIM_SetCounter(&htim3,0);
		
//		if(speed3!=0)
//		{
//			char buffer[64];
//			sprintf(buffer, "%d,%d,%d\n",target_speed3,speed3,pwm_out3);
//			HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);	
//		}//用于VOFA+调试输出曲线
		
	}
	
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
			rx_flag1=1;
      HAL_UART_Receive_IT(&huart1, (uint8_t *)&rx_data1, 1);
    }
}//串口1接收蓝牙消息
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USART2_UART_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
	do{
			ret = MPU6050_DMP_init();
		}while(ret);//MPU6050初始化
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);//声明开启定时器1通道一二，产生10KHz的PWM波
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);//声明开启定时器2、3的编码器模式
  HAL_TIM_Base_Start_IT(&htim4);//声明开启定时器4的中断 
	__HAL_TIM_SetCounter(&htim2,0);
	__HAL_TIM_SetCounter(&htim3,0);//程序开始时手动将定时器编码器的CNT值置0
	PID_init(&Position1,PID_POSITION,PID_position1,500.0f,1000.0f);
  PID_init(&Speed1,PID_POSITION,PID_speed1,30.0f,1000.0f);
	PID_init(&Position3,PID_POSITION,PID_position3,500.0f,1000.0f);
  PID_init(&Speed3,PID_POSITION,PID_speed3,30.0f,1000.0f);//PID初始化
	HAL_UART_Receive_IT(&huart1, (uint8_t *)&rx_data1, 1);//声明开启串口1的接收中断，用于接收蓝牙发送的消息
		
	CAN_FilterTypeDef filter;
  filter.FilterIdHigh = 0x0000;
  filter.FilterIdLow = 0x0000;
  filter.FilterMaskIdHigh = 0x0000;
  filter.FilterMaskIdLow = 0x0000;
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter.FilterBank = 0;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14;
  HAL_CAN_ConfigFilter(&hcan, &filter);//CAN通信发送端过滤器配置
	
	HAL_CAN_Start(&hcan);
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
	TxHeader.StdId = 0x123;    // 标准ID
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.DLC = 1;         // 数据长度1字节
  TxHeader.TransmitGlobalTime = DISABLE;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		MPU6050_DMP_Get_Date(&pitch,&roll,&yaw);
		if(yaw_flag)
		{		
			if(yaw>=-2.0 && yaw<=2.0)
			{
				yaw_flag=0;PID_mode=1;
				target_angle1=0;target_angle3=0;
				target_speed1=0;target_speed3=0;
				sum_angle1=0;sum_angle3=0;
				delta1=0;delta3=0;
				HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
			}
		}//实现yaw轴闭环，当板子一判断符合条件时向板子二发送停止信号
		
		if(rx_flag1)//对蓝牙接收到的消息进行运动模式判断
		{
			rx_flag1=0;
			
			if(rx_data1 == 0xFF)
			{
				yaw_flag=1;
				if(yaw<=-2.0)		{rx_data1=0x05;}
				else if(yaw>=2.0)		{rx_data1=0x06;}
				else {rx_data1=0x00;}
			}//实现yaw轴闭环，根据yaw轴此时角度判断需要左转圈还是右转圈
			
			if(rx_data1 == 0x00)		{PID_mode=1;target_angle1=0;target_angle3=0;real_target_speed1=0;real_target_speed3=0;sum_angle1=0;sum_angle3=0;}//停止
			
			else if(rx_data1 == 0x01)	{PID_mode=0;real_target_speed1=500;real_target_speed3=-500;target_angle1=0;target_angle3=0;sum_angle1=0;sum_angle3=0;}//定速前进
			else if(rx_data1 == 0xF1)	{PID_mode=1;target_angle1=1440;target_angle3=-1440;sum_angle1=0;sum_angle3=0;}//定位移前进
			
			else if(rx_data1 == 0x02)	{PID_mode=0;real_target_speed1=-500;real_target_speed3=500;target_angle1=0;target_angle3=0;sum_angle1=0;sum_angle3=0;}//定速后退
			else if(rx_data1 == 0xF2)	{PID_mode=1;target_angle1=-1440;target_angle3=1440;sum_angle1=0;sum_angle3=0;}//定位移后退
			
			else if(rx_data1 == 0x03)	{PID_mode=0;real_target_speed1=500;real_target_speed3=-500;target_angle1=0;target_angle3=0;sum_angle1=0;sum_angle3=0;}//定速左移
			else if(rx_data1 == 0xF3)	{PID_mode=1;target_angle1=1440;target_angle3=-1440;sum_angle1=0;sum_angle3=0;}//定位移左移
			
			else if(rx_data1 == 0x04)	{PID_mode=0;real_target_speed1=-500;real_target_speed3=500;target_angle1=0;target_angle3=0;sum_angle1=0;sum_angle3=0;}//定速右移
			else if(rx_data1 == 0xF4)	{PID_mode=1;target_angle1=-1440;target_angle3=1440;sum_angle1=0;sum_angle3=0;}//定位移右移
			
			else if(rx_data1 == 0x05)	{PID_mode=0;real_target_speed1=500;real_target_speed3=500;target_angle1=0;target_angle3=0;sum_angle1=0;sum_angle3=0;}//定速左转弯
			else if(rx_data1 == 0xF5)	{PID_mode=1;target_angle1=720;target_angle3=720;sum_angle1=0;sum_angle3=0;}//定位移左转弯
			
			else if(rx_data1 == 0x06)	{PID_mode=0;real_target_speed1=-500;real_target_speed3=-500;target_angle1=0;target_angle3=0;sum_angle1=0;sum_angle3=0;}//定速右转弯
			else if(rx_data1 == 0xF6)	{PID_mode=1;target_angle1=-720;target_angle3=-720;sum_angle1=0;sum_angle3=0;}//定位移右转弯
			
			else if(rx_data1 == 0x07)	{PID_mode=0;real_target_speed1=500;real_target_speed3=-500;target_angle1=0;target_angle3=0;sum_angle1=0;sum_angle3=0;}//定速左前
			else if(rx_data1 == 0xF7)	{PID_mode=1;target_angle1=1440;target_angle3=-1440;sum_angle1=0;sum_angle3=0;}//定位移左前
			
			else if(rx_data1 == 0x08)	{PID_mode=1;target_angle1=0;target_angle3=0;real_target_speed1=0;real_target_speed3=0;sum_angle1=0;sum_angle3=0;}//定速左后
			else if(rx_data1 == 0xF8)	{PID_mode=0;target_angle1=0;target_angle3=0;real_target_speed1=0;real_target_speed3=0;sum_angle1=0;sum_angle3=0;}//定位移左后
			
			else if(rx_data1 == 0x09)	{PID_mode=1;target_angle1=0;target_angle3=0;real_target_speed1=0;real_target_speed3=0;sum_angle1=0;sum_angle3=0;}//定速右前
			else if(rx_data1 == 0xF9)	{PID_mode=0;target_angle1=0;target_angle3=0;real_target_speed1=0;real_target_speed3=0;sum_angle1=0;sum_angle3=0;}//定位移右前
			
			else if(rx_data1 == 0x0A)	{PID_mode=0;real_target_speed1=-500;real_target_speed3=500;target_angle1=0;target_angle3=0;sum_angle1=0;sum_angle3=0;}//定速右后
			else if(rx_data1 == 0xFA)	{PID_mode=1;target_angle1=-1440;target_angle3=1440;sum_angle1=0;sum_angle3=0;}//定位移右后
			
			HAL_UART_Transmit(&huart1, (uint8_t *)&rx_data1, 1, HAL_MAX_DELAY);//将接收到的蓝牙消息反传给手机(串口1)
			HAL_UART_Transmit(&huart3, (uint8_t *)&rx_data1, 1, HAL_MAX_DELAY);//将接收到的蓝牙消息传给板子二(串口3)
		}
			
		if(PID_mode)//PID_mode为真(1)时为串级双闭环PID，PID_mode假(0)时为串级双闭环PID
		{
			delta1=target_angle1-sum_angle1;
			delta3=target_angle3-sum_angle3;
		}
		else	{delta1=0;delta3=0;}
		
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
