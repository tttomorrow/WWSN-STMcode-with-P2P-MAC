/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 CSTX.
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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "string.h"
#include "oled.h"
#include "esp8266.h"
#include "mqtt.h"


//通过阿里云物联网三要素计算你的密码 https://nnhpiot.com/#/Online_tools
//////////////////////////////下面是自行修改参数的地方////////////////////////////////////////
#define ServerIP "192.168.31.121"
#define Port "1883"

#define ClientID "mqttjs_31e8c62ea1"    //需要定义为用户自己的参数
#define Username   "admin"      //需要定义为用户自己的参数
#define Password   "public"  //需要定义为用户自己的参数

#define Topic   "testtopic" //需要定义为用户自己的参数
#define TopicPost   "testtopic" //需要定义为用户自己的参数

#define SSID "Wireless Weak-link Network"
#define WIFIPassword "18239778101"
///////////////////////////////////////////////////////////////

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define SDATA "ToggleLED\r\n"	
unsigned char cscxReg[3]={0xC1,0x00,0x09};	//发送查询数据
unsigned char revCsReg[12]={0xC1 ,0x00 ,0x09 ,0xFF ,0xFF ,0xFF ,0xFF ,0xFF ,0xFF, 0xFF ,0xFF ,0xFF};
unsigned char key=0;
unsigned char disOLED[24];


unsigned char servernotok=1;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
	uint8_t res=1;	 
	int i ;
	char *CSTXREVData[3];
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();	//HAL初始化

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();	

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	USART_Interupt_Enable();	//使能串口接收中断和空闲中断
	//printf 串口1 打印用来和wifi通信了 ，注意不能打印过多的东西否则wifi会死机的
	//printf("\r\n ############ http://www.csgsm.com/ ############\r\n ############("__DATE__ " - " __TIME__ ")############\r\n");
	
	
	CS_OLED_Init();
  OLED_Clear();
	OLED_ShowString(16,0,"LoRa"); OLED_ShowCHinese(48,0,7);OLED_ShowCHinese(64,0,8);OLED_ShowCHinese(80,0,9);

  /* USER CODE END 2 */


	OLED_ShowString(100,0,"[B]");
	cstxInitialize(); 	//初始化LED  闪烁下
	
	if(HAL_GPIO_ReadPin(M1_GPIO_Port, M1_Pin) == 1) //M1 是高电平才开始读取模块的寄存器
	{
		CS_Reg_Send_Data(cscxReg,3);//发送3个字节 	
		HAL_Delay(300);	//等待延迟
		cstx_reg_Receive_Data(revCsReg,&key);

		memset(disOLED,0,24);
		
		OLED_ShowString(0,2,"Addr:");
		for(i=3;i<5;i++) //显示
		{
			sprintf((char *)disOLED,"%02X",revCsReg[i]);
			OLED_ShowString(i*16-4,2,disOLED);
		}
		
		OLED_ShowString(82,2,"Ch:");
		sprintf((char *)disOLED,"%02X",revCsReg[8]);
		OLED_ShowString(104,2,disOLED);
	}
	else
	{
		OLED_ShowString(0,2,"M1, M0 :  0 , 0");
	}

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	HAL_GPIO_WritePin(M1_GPIO_Port, M1_Pin, GPIO_PIN_RESET); //使能模块运行 M1 0
	
	/*
	printf("+++");	//退出透传模式
	//前面发送的所有AT命令都可能给模块反馈ERROR导致没有OK 所以把之前的全部清空串口1 
	//串口1和WiFi模块通信 串口2和LORA模块通信 ，单片机就2个串口全部用掉了*/
	HAL_Delay(1000);
	memset(USART_RX_BUF,0,USART_REC_LEN);
	USART_RX_STA = 0;
	
  while (1)
  {
    /* USER CODE END WHILE */
		
		//WiFi模块连接到路由器或者手机热点
		while(res)
		{
			res=WIFI_Dect((uint8_t *)SSID,(uint8_t *)WIFIPassword);
			HAL_Delay(200);
		}
		HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET); //LED1点亮 表示单片机和wifi连接成功
		
		res=1;
		while(servernotok)
		{
			res=ESP8266_CONNECT_SERVER((uint8_t *)ServerIP,(uint8_t *)Port);
			HAL_Delay(1000);
		}
		HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_RESET);
		OLED_ShowString(0,2,"               ");
		OLED_ShowString(0,2,"CONNE SERVER OK");
		
		//链接到阿里云
		servernotok=1;
		while(servernotok)
		{	
			res=mqtt_Connection((char *)Username,(char *)Password,(char *)ClientID);  //连接服务器
			if(servernotok == 0){					
				USART2_printf("Enter MQTT OK!\r\n");
			}
			else{		
			  USART2_printf("Enter MQTT Error!\r\n");
			}	
		}
		servernotok=1;
		while(servernotok)
		{	
			res=mqtt_subscribe((char *)Topic);  //订阅消息
			if(servernotok == 0){					
				USART2_printf("Subscription succeeded!\r\n");
			}
			else{		
			  USART2_printf("Subscription failed!\r\n");
			}	
		}
		
		OLED_ShowString(0,2,"               ");
		OLED_ShowString(0,2,"REGISTER EMQ OK");
		
		USART2_printf("***CONNECT SERVER SUCCEED***\r\n");		

    /* USER CODE BEGIN 3 */
		while(1) 
		{
				HAL_Delay(2000);
				OLED_ShowString(0,2,"               ");
				OLED_ShowString(0,2,"Send EMQ MQTT .");
				
				if(!servernotok)
				{
					ESP8266_Send_data((char *)TopicPost,"33","44");  //发送数据 调试用

				}
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

/************************ (C) COPYRIGHT CSTX *****END OF FILE****/
