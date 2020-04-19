/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2020 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "string.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
char rx_buffer[100],tx_buffer[150];

uint16_t ProtocolNameLength;
uint16_t ClientIDLength;

uint8_t connect = 0x10,publishCon = 0x30,subscribeCon = 0x82;
char *protocolName = "MQTT";
uint8_t level = 0x04;
uint8_t flag = 0x02;   // 02--> sifresiz
uint16_t keepAlive =60;
uint16_t packetID = 0x01;
uint8_t Qos = 0x00;
char *clientID = "Topuz";


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void Connect_Broker(char *Ip ,char *Port)
{
	HAL_UART_Transmit(&huart1,(uint8_t *)"AT+CIPCLOSE\r\n",strlen("AT+CIPCLOSE\r\n"),1000);			// Close TCP or UDP connection.
	HAL_Delay(100);
	HAL_UART_Transmit(&huart1,(uint8_t *)"AT+CIPMODE=0\r\n",strlen("AT+CIPMODE=0\r\n"),5000);		// TCP Application Mode : 0->Normal Mode ,1->Transparent Mode
	HAL_Delay(100);
	HAL_UART_Transmit(&huart1,(uint8_t *)"AT+CIPMUX=0\r\n",strlen("AT+CIPMUX=0\r\n"),1000);			// Single IP connection
	HAL_Delay(100);
	HAL_UART_Transmit(&huart1,(uint8_t *)"AT+CSTT=\"internet\",\"\",\"\"\r\n",strlen("AT+CSTT=\"internet\",\"\",\"\"\r\n"),5000);	// APN settings
	HAL_Delay(100);
	HAL_UART_Transmit(&huart1,(uint8_t *)"AT+CIICR\r\n",strlen("AT+CIICR\r\n"),5000); // Bring Up Wireless Connection with GPRS or CSD
	HAL_Delay(1000);
	HAL_UART_Transmit(&huart1,(uint8_t *)"AT+CIFSR\r\n",strlen("AT+CIFSR\r\n"),1000); // Get Local IP Address 
	HAL_Delay(100);
	HAL_UART_Transmit(&huart1,(uint8_t *)tx_buffer,sprintf(tx_buffer,"AT+CIPSTART=\"TCP\",\"%s\",\"%s\"\r\n",Ip,Port),5000); // Start up TCP connection
	HAL_Delay(2000);
	HAL_UART_Transmit(&huart1,(uint8_t *)"AT+CIPQSEND=1\r\n",strlen("AT+CIPQSEND=1\r\n"),5000); // Select Data Transmitting Mode. 0->Normal mode ,1->Quick send mode
	HAL_Delay(100);
	HAL_UART_Transmit(&huart1,(uint8_t *)"ATE0\r\n",strlen("ATE0\r\n"),5000);									// Set Command Echo Mode. 0->Echo mode off,1->Echo mode on 
	HAL_Delay(100);
	
	//connect packet
	
	ProtocolNameLength = strlen(protocolName);
	ClientIDLength     = strlen(clientID);
	uint8_t Remainlength;
	Remainlength = 2+ProtocolNameLength+6+ClientIDLength;
	uint16_t length = sprintf(tx_buffer,"%c%c%c%c%s%c%c%c%c%c%c%s",(char)connect,(char)Remainlength,(char)(ProtocolNameLength << 8),(char)ProtocolNameLength,protocolName,(char)level,(char)flag,(char)(keepAlive << 8),(char)keepAlive,(char)(ClientIDLength << 8),(char)ClientIDLength,clientID);
	
	HAL_UART_Transmit(&huart1,(uint8_t *)tx_buffer,sprintf(tx_buffer,"AT+CIPSEND=%d\r\n",length),1000);
	HAL_Delay(100);
	HAL_UART_Transmit(&huart1,(uint8_t *)tx_buffer,sprintf(tx_buffer,"%c%c%c%c%s%c%c%c%c%c%c%s",(char)connect,(char)Remainlength,(char)(ProtocolNameLength << 8),(char)ProtocolNameLength,protocolName,(char)level,(char)flag,(char)(keepAlive << 8),(char)keepAlive,(char)(ClientIDLength << 8),(char)ClientIDLength,clientID),5000);

}
void Subscribe(char *topic)
{
	uint16_t TopicLength = strlen(topic);
	uint8_t RemainLength = 2+2+TopicLength+1; // packetIDlength(2) + topiclengthdata(2)+topiclength+Qos
	uint16_t length = sprintf(tx_buffer,"%c%c%c%c%c%c%s%c",(char)subscribeCon,(char)RemainLength,(char)(packetID << 8),(char)packetID,(char)(TopicLength << 8),(char)TopicLength,topic,(char)Qos);
	HAL_UART_Transmit(&huart1,(uint8_t *)tx_buffer,sprintf(tx_buffer,"AT+CIPSEND=%d\r\n",length),1000);
	HAL_Delay(100);
	HAL_UART_Transmit(&huart1,(uint8_t *)tx_buffer,sprintf(tx_buffer,"%c%c%c%c%c%c%s%c",(char)subscribeCon,(char)RemainLength,(char)(packetID << 8),(char)packetID,(char)(TopicLength << 8),(char)TopicLength,topic,(char)Qos),5000);
	
}
void publish(char *topic, char *message)
{

	uint16_t topiclength = strlen(topic);
	uint8_t remainlength = 2+topiclength+strlen(message);
	int length = sprintf(tx_buffer,"%c%c%c%c%s%s",(char)publishCon,(char)remainlength,(char)(topiclength << 8),(char)topiclength,topic,message);
	HAL_UART_Transmit(&huart1,(uint8_t *)tx_buffer,sprintf(tx_buffer,"AT+CIPSEND=%d\r\n",length),100);
	HAL_Delay(100);
	HAL_UART_Transmit(&huart1,(uint8_t *)tx_buffer,sprintf(tx_buffer,"%c%c%c%c%s%s",(char)publishCon,(char)remainlength,(char)(topiclength << 8),(char)topiclength,topic,message),5000);
	HAL_Delay(100);
	
}
void Read_Message()
{
	int remain_length=0,message_length=0,topic_length=0;
	char message[100];
	HAL_UART_AbortReceive_IT(&huart1);
	for(int i=0;i<sizeof(rx_buffer);i++)
	{
		 if(rx_buffer[i] == 0x30)
		 {
				remain_length = rx_buffer[i+1];
				topic_length  = rx_buffer[i+2]+rx_buffer[i+3];
				message_length = remain_length -(topic_length + 2);
				for(int j=0;j<message_length;j++)
				{
					message[j] = rx_buffer[i+4+topic_length+j];	
				}
				break;
		 }
	}
	if(message[0] == 'O' && message[1] == 'N')
	{
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);			// turn led on (C13)
		publish("LED","LED ON");
	}
	if(message[0] == 'O' && message[1] == 'F' && message[2] == 'F')
	{
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);				// turn led off
		publish("LED","LED OFF");
	}
	memset(rx_buffer,0,sizeof(rx_buffer)); 											// clear buffer
	memset(message,0,sizeof(message));
	HAL_UART_Receive_IT(&huart1,(uint8_t *)rx_buffer,100);
}

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);	
		HAL_UART_Receive_IT(&huart1,(uint8_t *)rx_buffer,100);
		HAL_Delay(1000);
		Connect_Broker("212.125.XX.XXX","1883");	      // Broker IP and port
		HAL_Delay(1000);
		Subscribe("IOT");
		HAL_Delay(100);
  while (1)
  {
	  Read_Message();
		HAL_Delay(500);
		

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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
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
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
