/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "UARTStream.h"
#include "Codec.h"
#include "Frame/Packet.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  Command_Type_Error        = 0xFF,
  Command_Type_Unknown      = 0,
  Command_Type_Led_Set      = 1,
  Command_Type_Led_Get      = 2,
  Command_Type_Ok           = 0x80,
} Command_Type;

typedef struct {
  uint8_t             LED;
  uint8_t             State;
} Command_LedSet;

typedef struct {
  uint8_t             States[2];
} Command_LedGet;

typedef struct {
  Command_Type        Type;
  union {
    Command_LedSet    LedSet;
    Command_LedGet    LedGet;
    Codec_Error       ErrorCode;
  };
} Command;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

static uint8_t	streamRxBuff[64];
static uint8_t	streamTxBuff[64];
static uint8_t  decodeBuff[12];
static Packet   decodePacket;

static UARTStream uartStream1;
static Codec codec;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

uint8_t Command_decode(Command* cmd, StreamBuffer* stream);
uint8_t Command_encode(Command* cmd, StreamBuffer* stream);
void Command_send(Command* cmd);

void Command_onDecode(Codec* codec, Codec_Frame* frame);
void Command_onDecodeError(Codec* codec, Codec_Frame* frame, Codec_LayerImpl* layer, Codec_Error error);

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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  UARTStream_init(&uartStream1, &huart1, streamRxBuff, sizeof(streamRxBuff), streamTxBuff, sizeof(streamTxBuff));
  Codec_init(&codec, Packet_baseLayer());
  Codec_onDecode(&codec, Command_onDecode);
  Codec_onDecodeError(&codec, Command_onDecodeError);
  Codec_setDecodeSync(&codec, Packet_sync);
  Packet_init(&decodePacket, decodeBuff, sizeof(decodeBuff));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  IStream_receive(&uartStream1.Input);
  Codec_beginDecode(&codec, &decodePacket);
  while (1)
  {
    Codec_decode(&codec, &uartStream1.Input);
    
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
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, LED_GREEN_Pin|LED_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_BTN_Pin */
  GPIO_InitStruct.Pin = USER_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_GREEN_Pin LED_RED_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin|LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
uint8_t Command_decode(Command* cmd, StreamBuffer* stream) {
  Stream_setByteOrder(stream, ByteOrder_BigEndian);
  // read type
  cmd->Type = (Command_Type) Stream_readUInt8(stream);
  // check type and parse
  switch (cmd->Type) {
    case Command_Type_Led_Set:
      cmd->LedSet.LED = Stream_readUInt8(stream);
      cmd->LedSet.State = Stream_readUInt8(stream);
      break;
    case Command_Type_Led_Get:
      break;
    case Command_Type_Error:
      cmd->ErrorCode = Stream_readUInt32(stream);
      break;
    case Command_Type_Ok:
      break;
    case Command_Type_Unknown:
    default:
      return 0;
  }
  return 1;
}
uint8_t Command_encode(Command* cmd, StreamBuffer* stream) {
  uint8_t i;
  Stream_setByteOrder(stream, ByteOrder_BigEndian);
  // write type
  Stream_writeUInt8(stream, (uint8_t) cmd->Type);
  // check type and parse
  switch (cmd->Type) {
    case Command_Type_Led_Set:
      Stream_writeUInt8(stream, cmd->LedSet.LED);
      Stream_writeUInt8(stream, cmd->LedSet.State);
      break;
    case Command_Type_Led_Get:
      for (i = 0; i < sizeof(cmd->LedGet.States); i++) {
        Stream_writeUInt8(stream, cmd->LedGet.States[i]);
      }
      break;
    case Command_Type_Error:
      Stream_writeUInt32(stream, cmd->ErrorCode);
      break;
    case Command_Type_Ok:
      break;
    case Command_Type_Unknown:
    default:
      return 0;
  }
  return 1;
}
void Command_send(Command* cmd) {
  StreamBuffer tempStream;
  uint8_t temp[sizeof(Command)];
  Packet encodePacket;
  
  Stream_init(&tempStream, temp, sizeof(temp));
  Command_encode(cmd, &tempStream);
  Packet_init(&encodePacket, temp, Stream_available(&tempStream));
  Codec_encodeFrame(&codec, &encodePacket, &uartStream1.Output, Codec_EncodeMode_FlushLayer);
}
void Command_onDecode(Codec* codec, Codec_Frame* frame) {
  StreamBuffer stream;
  Packet* packet = (Packet*) frame;
  Command cmd;
  Stream_fromBuff(&stream, packet->Data, packet->Size, packet->Len);
  
  if (Command_decode(&cmd, &stream)) {
    switch (cmd.Type) {
      case Command_Type_Led_Set:
        if (cmd.LedSet.State <= 1 && cmd.LedSet.LED <= 1) {
              
          switch (cmd.LedSet.LED) {
            case 0:
              HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, (GPIO_PinState) cmd.LedSet.State);
              break;
            case 1:
              HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, (GPIO_PinState) cmd.LedSet.State);
              break;
          }
        }
        break;
      case Command_Type_Led_Get:
        cmd.Type = Command_Type_Led_Get;
        cmd.LedGet.States[0] = (LED_GREEN_GPIO_Port->ODR & LED_GREEN_Pin) != 0;
        cmd.LedGet.States[1] = (LED_RED_GPIO_Port->ODR & LED_RED_Pin) != 0;
        Command_send(&cmd);
        break;
      case Command_Type_Unknown:
      default:  
        cmd.Type = Command_Type_Error;
        cmd.ErrorCode = 254;
        Command_send(&cmd);
        break;
      
    }
  }
  else {
    cmd.Type = Command_Type_Error;
    cmd.ErrorCode = 255;
    Command_send(&cmd);
  }
  
}
void Command_onDecodeError(Codec* codec, Codec_Frame* frame, Codec_LayerImpl* layer, Codec_Error error) {
  Command cmd;
  cmd.Type = Command_Type_Error;
  cmd.ErrorCode = error;
  Command_send(&cmd);
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	switch ((uint32_t) huart->Instance) {
		case USART1_BASE:
			UARTStream_txHandle(&uartStream1);
			break;
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	switch ((uint32_t) huart->Instance) {
		case USART1_BASE:
			UARTStream_rxHandle(&uartStream1);
			break;
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
