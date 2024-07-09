/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "crc.h"
#include "gpdma.h"
#include "i2c.h"
#include "icache.h"
#include "memorymap.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "app_touchgfx.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LCD.h"
#include "touch.h"
#include "delay.h"
#include "user_app.h"
#include "GUI.h"
#include "DHT11.h"
#include "ili_mpu_cnt.h"
#include "mpu6050.h"
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
RTC_TimeTypeDef  RTC_Time;
RTC_DateTypeDef  RTC_Date;
int flag=0;
int ox=0;
int temp=0;
int bushu1=0;
/******************************************************************************/
int hand=0;
int blueb=0;
int oxflag=0;
int tempflag=0;
int bushuflag=0;
int blueflag=0;
int beepflag=0;
int totaltask=0;
int32_t handflag;
uint8_t rx;
/*****************************************************************************/
extern void touchgfx_signalVSynTimer(void);
extern int32_t n_heart_rate;  
extern  int32_t n_sp02; 
extern unsigned int rec_data[4];
typedef struct _atk_ncr_point
{
    short x;       
    short y;        
}atk_ncr_point;
static atk_ncr_point ncr_input_buf[800];
static uint8_t  ncr_input_buf1[800];
    uint32_t t = 0;
    uint8_t tcnt;
    uint8_t key;
    uint8_t mode = 4;
    uint16_t lastpos[2];
    uint16_t pcnt = 0;
char sbuf[10];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void hand_of_sean();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_GPDMA1_Init();
  MX_ICACHE_Init();
  MX_SPI1_Init();
  MX_TIM16_Init();
  MX_CRC_Init();
  MX_RTC_Init();
  MX_TIM17_Init();
  MX_I2C1_Init();
	MX_I2C2_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_TouchGFX_Init();
  /* USER CODE BEGIN 2 */
	delay_init(160);    							    	//定时器初始化 ，括号内填芯片系统频率（单位：兆）
	LCD_Init();	     								       //显示屏初始化   
	TP_Init();      								      //触摸初始化  ，主要是计算物理坐标和屏幕坐标之间的转换系数  屏幕的物理坐标为读到的电压值
	MPU6050_INIT();
  MX_TIM16_Init();
  MX_CRC_Init();
  MX_TouchGFX_Init();
	HAL_TIM_Base_Start_IT(&htim4);   //开启定时器4，用于计步
  HAL_TIM_Base_Start_IT(&htim16); //开启定时器16开启,系统任务调度开始 
  HAL_TIM_Base_Start_IT(&htim17);//开启定时器17开启,系统任务调度开始  

	/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		 MX_TouchGFX_Process();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_0;
  RCC_OscInitStruct.LSIDiv = RCC_LSI_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLMBOOST = RCC_PLLMBOOST_DIV4;
  RCC_OscInitStruct.PLL.PLLM = 3;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 1;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLLVCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint8_t p_Time16Cnt = 0,		p_Time17Cnt=0;
	/***************************************************************************************/
	//定时器16进行5ms任务中断
	if (htim->Instance == htim16.Instance) 
	{
		p_Time16Cnt++;
		if(!(p_Time16Cnt % 4))  //20ms(50Hz)进行触发刷新
		{
			touchgfx_signalVSynTimer();  //touchgfx用户接口
		}
		
		if(!(p_Time16Cnt % 40))  
		{
			if(ox==1)
			{
				Update_HeartRateInfo();
				LCD_ShowNum(160,120,n_heart_rate/4,3,16);
				LCD_ShowNum(80,120,n_sp02,3,16);
				ox=0;
			}
		}	
			
		if(!(p_Time16Cnt % 60))  
		{
			if(temp==1)
			{
				DHT11_REC_Data();
				LCD_ShowNum(65,160,rec_data[0],3,16);
				LCD_ShowNum(220,160,rec_data[2],3,16);
				temp=0;
			}
   	}
		
		if(!(p_Time16Cnt % 50))
		{
			if(bushu1==1)
				{
          LCD_ShowNum(90,190,step,3,16);
				  bushu1 =0;
		   	}
   	}

  	if(!(p_Time16Cnt % 70))
		{
			if(hand==1)
				{
					LCD_Clear(WHITE);
          hand_of_sean();
				  hand=0;
		   	}
   	}
	
		if(!(p_Time16Cnt % 80))
		{
			if(blueb==1)
				{
	        HAL_UART_Receive(&huart1,&rx,1,HAL_MAX_DELAY);
          if(rx == '1')
           {
             HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_SET);
            }
          if(rx == '2')
           {
             for(uint8_t i = 0; i < 13; i ++)
                {
                 HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_RESET);
                 } 
									blueb=0;
            }
		   	 }
   	}

	 }

	if (htim->Instance == htim4.Instance) 
	{
	   static uint8_t step_time_count = 0;
			detect_step();
			step_time_count ++;
			if(step_time_count == 6)   
			{
				step_time_count = 0;
				if(step_count != 0)
				{
					step_count = 0;
					step ++;
				}
			}
	}
}


void hand_of_sean()
{
	int flag=0;
	int flag2=0,flaga=0,flag1=0,flagh=0;
	int a=0;
	lcddev.width=320;
	lcddev.height=240;
	 POINT_COLOR = RED;
	BACK_COLOR = 0xFFFF;
	
	while(1)
	   {
			 flag1=1;
	 	    tp_dev.scan(0);
        if (tp_dev.sta & TP_PRES_DOWN)                                                   
          {
						flag2=0,flaga=0,flag1=0,flagh=0;flag1=1;
						
            tcnt = 0;
            if (((tp_dev.x  < (lcddev.width - 20 - 2)) && (tp_dev.x  >= (20 + 2))) &&
                ((tp_dev.y  < (lcddev.height - 5 - 2)) && (tp_dev.y  >= (115 + 2))))
              {
                lastpos[0] = tp_dev.x;
                lastpos[1] = tp_dev.y;
                uint16_t POINT_COLOR = RED,BACK_COLOR = WHITE;  
                LCD_DrawLine(lastpos[0], lastpos[1], tp_dev.x, tp_dev.y);
                lastpos[0] = tp_dev.x;
                lastpos[1] = tp_dev.y;
                if (pcnt < 200)
                {
                    if (pcnt != 0)
                    {
                        if ((ncr_input_buf[pcnt - 1].y != tp_dev.y) &&
                            (ncr_input_buf[pcnt - 1].x != tp_dev.x))
                        {
                            ncr_input_buf[pcnt].x = tp_dev.x;
                            ncr_input_buf[pcnt].y = tp_dev.y;
                            pcnt++;
                        }
                    }
                    else
                    {
                        ncr_input_buf[pcnt].x = tp_dev.x;
                        ncr_input_buf[pcnt].y = tp_dev.y;
                        pcnt++;
                    }
                 }a++;
               }
            }else
        {
            lastpos[0] = 0xFFFF;
            tcnt++;
            delay_ms(20);
            t++;
            if (tcnt == 40)
            {
                if (pcnt != 0)
                {
                    pcnt = 0; 
                }
                LCD_Fill(20, 115, lcddev.width - 20 - 1, lcddev.height - 5 - 1, WHITE);
            }
        }
			if(flag1==1&&a==200)
				{
				   LCD_ShowNum(20,40,1,2,16);
					 flag1=0;
			  	 flaga=1;
					a=0;
				}
			if(flaga==1&&a==200)
				{
				   LCD_ShowNum(20,40,1,2,16);
					 flag1=0;
			  	 flaga=1;
					a=0;
				}

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
