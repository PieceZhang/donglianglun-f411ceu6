/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_ON() HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET)
#define LED_OFF() HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET)
#define LED_TOGGLE() HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin)

#define ABS(x)		((x>0)? x: -x) 

#define PWM_ZERO 1500
#define PWM_AMPLITUDE 300
#define PWMY_MAX PWM_ZERO + PWM_AMPLITUDE
#define PWMY_MIN PWM_ZERO - PWM_AMPLITUDE
#define PWMX_MAX PWM_ZERO + PWM_AMPLITUDE
#define PWMX_MIN PWM_ZERO - PWM_AMPLITUDE

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
float X_Velocity_Control(int encoder);  // X轴速度控制
float Y_Velocity_Control(int encoder);  // Y轴速度控制
float X_balance_Control(float Angle,float Angle_Zero,float Gyro);  // X轴角度控制
float Y_balance_Control(float Angle,float Angle_Zero,float Gyro);  // Y轴角度控制
void MotorSet(uint16_t mx, uint16_t my);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t MPUflag=0;
uint8_t MPUBuffer[22];
float MPU_gyro[3]={0};
float MPU_angle[3]={0};

uint8_t Start_Flag=0;
int Balance_X,Balance_Y;  // 三轴角度控制量 
int	Velocity_X,Velocity_Y;  // 三轴速度控制量 
float MotorX=1500, MotorY=1500;

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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

	/* USART1: MPU9250 115200(DMA) 100hz, USART2: PC 115200 */
	My_UART_DMA_Init();
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1500);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 1500);
	HAL_Delay(1500);
	HAL_TIM_Base_Start_IT(&htim2);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	float MotorX_last=1500, MotorY_last=1500;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if(MPUflag)
		{
			MPUflag = 0;
			/* PID calculate */
			Balance_X = X_balance_Control(MPU_angle[0],0,MPU_gyro[0]); 
			Balance_Y = Y_balance_Control(MPU_angle[1],0,MPU_gyro[1]);
			Velocity_X = X_Velocity_Control(MotorX_last-1500);
			Velocity_Y = Y_Velocity_Control(MotorY_last-1500);
			MotorX = 1500 + Balance_X + Velocity_X;
			MotorY = 1500 + Balance_Y + Velocity_Y;
			/* limit max and min */
			if(MotorX > PWMX_MAX) MotorX = PWMX_MAX;
			else if(MotorX < PWMX_MIN) MotorX = PWMX_MIN;
			if(MotorY > PWMY_MAX) MotorY = PWMY_MAX;
			else if(MotorY < PWMY_MIN) MotorY = PWMY_MIN;
			/* save last*/
			MotorX_last = MotorX, MotorY_last = MotorY;
			/* motor output */
			MotorSet(MotorX, MotorY);
		}
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

float Velocity_KP=0.15,Velocity_KI=0.085;   //电机速度控制PI参数  100 20
float Balance_KP=39,Balance_KI=0,Balance_KD=48; //平衡控制PID参数  1100 10 80

/**************************************************************************
X轴平衡PID控制
**************************************************************************/
float X_balance_Control(float Angle,float Angle_Zero,float Gyro)
{             
	static float PWM,error,Bias;   
	if(Start_Flag==0) 
		PWM=0,error=0;                                //停止时参数清零
	else
	{
		Bias=Angle-Angle_Zero;                                          //获取偏差
		error+=Bias;                                                    //偏差累积
		if(error>+30)error=+30;                                         //积分限幅
		if(error<-30)error=-30;                                         //积分限幅
		PWM=Balance_KP*Bias + Balance_KI*error + Gyro*Balance_KD/10;    //获取最终数值
	}
	return PWM;
}
/**************************************************************************
Y轴平衡PID控制
**************************************************************************/
float Y_balance_Control(float Angle,float Angle_Zero,float Gyro)
{  
	static float PWM,error,Bias;
	if(Start_Flag==0) 
		PWM=0,error=0;                                //停止时参数清零
	else
	{
		Bias=Angle-Angle_Zero;                                          //获取偏差
		error+=Bias;                                                    //偏差累积
		if(error>+30)error=+30;                                         //积分限幅
		if(error<-30)error=-30;                                         //积分限幅
		PWM=Balance_KP*Bias + Balance_KI*error + Gyro*Balance_KD/10;    //获取最终数值
	}
	return PWM;
}
/**************************************************************************
速度PI控制X 
**************************************************************************/
float X_Velocity_Control(int encoder)
{  
	static float Velocity,Encoder_Least,Encoder,Encoder_Integral;
	if(Start_Flag==0) 
		Encoder_Integral=0,Encoder=0,Velocity=0;    //停止时参数清零
	else
	{
		Encoder_Least=encoder;                                        //速度滤波  
		Encoder *= 0.7f;		                                            //一阶低通滤波器       
		Encoder += Encoder_Least*0.3f;	                                //一阶低通滤波器    
		Encoder_Integral +=Encoder;                                   //积分出位移 
		if(Encoder_Integral>+300) Encoder_Integral=+300;            //积分限幅
		if(Encoder_Integral<-300) Encoder_Integral=-300;            //积分限幅	
		Velocity=Encoder*Velocity_KP+Encoder_Integral*Velocity_KI/100;//获取最终数值
	}
	return Velocity;
}
/**************************************************************************
速度PI控制Y 
**************************************************************************/
float Y_Velocity_Control(int encoder)
{  
	static float Velocity,Encoder_Least,Encoder,Encoder_Integral;
	if(Start_Flag==0) 
		Encoder_Integral=0,Encoder=0,Velocity=0;    //停止时参数清零
	else
	{
		Encoder_Least=encoder;                                        //速度滤波  
		Encoder *= 0.7f;		                                            //一阶低通滤波器       
		Encoder += Encoder_Least*0.3f;	                                //一阶低通滤波器    
		Encoder_Integral +=Encoder;                                   //积分出位移 
		if(Encoder_Integral>+300) Encoder_Integral=+300;            //积分限幅
		if(Encoder_Integral<-300) Encoder_Integral=-300;            //积分限幅	
		Velocity=Encoder*Velocity_KP+Encoder_Integral*Velocity_KI/100;//速度合成
	}
	return Velocity;
}


void MotorSet(uint16_t mx, uint16_t my)
{
	if(my > PWMY_MAX) my=PWMY_MAX;
	else if(my < PWMY_MIN) my=PWMY_MIN;
	if(mx > PWMX_MAX) mx=PWMX_MAX;
	else if(mx < PWMX_MIN) mx=PWMX_MIN;
	
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, my);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, mx);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == (&htim2))  // 20ms
	{
		LED_TOGGLE();
	}
}
	
void USART1_IRQHandler(void)
{
	uint8_t errorcounter;

  HAL_UART_IRQHandler(&huart1);
	My_UART_RESET_RX_CALLBACK(&huart1);
	
	if(MPUBuffer[21]!=0)
	{
		errorcounter=0;
		if(MPUBuffer[10]==(uint8_t)(MPUBuffer[0]+MPUBuffer[1]+MPUBuffer[2]+MPUBuffer[3]+MPUBuffer[4]+
										MPUBuffer[5]+MPUBuffer[6]+MPUBuffer[7]+MPUBuffer[8]+MPUBuffer[9])&&MPUBuffer[0]!=0&&MPUBuffer[1]==0x52)
		{
			MPU_gyro[0] = ((short)(MPUBuffer[3]<<8 | MPUBuffer[2]))/32768.0f*2000;     
			MPU_gyro[1] = ((short)(MPUBuffer[5]<<8 | MPUBuffer[4]))/32768.0f*2000;      
			MPU_gyro[2] = ((short)(MPUBuffer[7]<<8 | MPUBuffer[6]))/32768.0f*2000;  
		}
		else errorcounter++;
		if(MPUBuffer[10+11]==(uint8_t)(MPUBuffer[0+11]+MPUBuffer[1+11]+MPUBuffer[2+11]+MPUBuffer[3+11]+MPUBuffer[4+11]+
										MPUBuffer[5+11]+MPUBuffer[6+11]+MPUBuffer[7+11]+MPUBuffer[8+11]+MPUBuffer[9+11])&&MPUBuffer[0+11]!=0&&MPUBuffer[1+11]==0x53)
		{
			MPU_angle[0] = ((short)(MPUBuffer[3+11]<<8| MPUBuffer[2+11]))/32768.0f*180; 
			MPU_angle[1] = ((short)(MPUBuffer[5+11]<<8| MPUBuffer[4+11]))/32768.0f*180;   	
			MPU_angle[2] = ((short)(MPUBuffer[7+11]<<8| MPUBuffer[6+11]))/32768.0f*180;
			MPU_angle[0] -= 0.5f;  // 修正安装误差
			MPU_angle[1] -= 2.3f;  // 修正安装误差
		}
		else errorcounter++;
		
		if(errorcounter>1) //纠错
		{
			__HAL_DMA_DISABLE(huart1.hdmarx);  //关闭通道
			huart1.hdmarx->Instance->NDTR=sizeof(MPUBuffer); //写寄存器 CNDTR：DMA传输剩余字节数（当通道开启时只读）
			__HAL_DMA_ENABLE(huart1.hdmarx);   //开启通道
		}
		else  // 无错
		{
			if(MPU_angle[0] > 30 || MPU_angle[0] < -30 || MPU_angle[1] > 30 || MPU_angle[1] < -30)
				Start_Flag = 0;
			else
				Start_Flag = 1;
			MPUflag = 1;
		}
	}
}


void HAL_Delay(uint32_t Delay)
{
  uint32_t tickstart = HAL_GetTick();
  uint32_t wait = Delay;

  /* Add a freq to guarantee minimum wait */
//  if (wait < HAL_MAX_DELAY)
//  {
//    wait += (uint32_t)(uwTickFreq);
//  }

  while ((HAL_GetTick() - tickstart) < wait)
  {
  }
}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
