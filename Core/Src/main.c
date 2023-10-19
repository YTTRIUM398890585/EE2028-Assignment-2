  /******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * (c) EE2028 Teaching Team
  ******************************************************************************/


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdio.h"
#include "string.h"

extern void initialise_monitor_handles(void);	// for semi-hosting support (printf)
static void UART1_Init(void);

UART_HandleTypeDef huart1;

int main(void) {
	initialise_monitor_handles();
	HAL_Init();
	UART1_Init();
	int seconds_count = 0;
	while (1) {
		seconds_count++;
		char message1[] = "Welcome to EE2028 !!!\r\n";  // Fixed message
		// Be careful about the buffer size used. Here, we assume that seconds_count does not exceed 6 decimal digits
		char message_print[32]; // UART transmit buffer. See the comment in the line above.
		sprintf(message_print, "%d: %s", seconds_count, message1);
		HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF); //Sending in normal mode
		HAL_Delay(1000);
	}
}

static void UART1_Init(void) {
    /* Pin configuration for UART. BSP_COM_Init() can do this automatically */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Configuring UART1 */
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
      while(1);
    }

}
