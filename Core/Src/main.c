/******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 * (c) EE2028 Teaching Team
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include "stdbool.h"
#include "stdint.h"
#include "stdio.h"
#include "string.h"

// button and led
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01.h"

// Sensors
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_accelero.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_gyro.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_hsensor.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_magneto.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_psensor.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_tsensor.h"

/* Variables -----------------------------------------------------------------*/
/* States
uint8_t to store 2 bits
00 = 0 = standby
01 = 1 = battle without last of ee2028
10 = 2 = battle with last of ee2028
11 = 3 = dead
other values are not used, case default should take care of them and show
something noticeable to aid in debug
*/
uint8_t state = 0;

/* Button
button_flag for EXTI
button_wait_flag for single and double press logic
*/
bool volatile button_flag = BOOL_CLR;
bool single_press = BOOL_CLR;
bool double_press = BOOL_CLR;

// UART Tx buffer, make sure does not excedd 32 - 1 characters when added with
// whatever when printing
char uart_buffer[UART_BUFFER_SIZE];

/* Function Prototype --------------------------------------------------------*/
extern void
initialise_monitor_handles(void); // for semi-hosting support (printf)

static void standby_mode(uint8_t* p_state);
static void battle_no_last_of_ee2028_mode(uint8_t* p_state);
static void battle_last_of_ee2028_mode(uint8_t* p_state);
static void dead_mode(uint8_t* p_state);

static void UART1_Init(void);
static void led_blink(uint32_t period);
static void button_press(void);

UART_HandleTypeDef huart1;

int main(void)
{
    initialise_monitor_handles();
    HAL_Init();
    BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);
    UART1_Init();
    BSP_LED_Init(LED2);

    // print Entering STANDBY MODE when going to STANDBY_MODE
    sprintf(uart_buffer, "Entering STANDBY MODE\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer),
        0xFFFF);

    while (1) {
        // seconds_count++;
        // char message1[] = "Welcome to EE2028 !!!\r\n";  // Fixed message
        // // Be careful about the buffer size used. Here, we assume that
        // seconds_count does not exceed 6 decimal digits char message_print[32]; //
        // UART transmit buffer. See the comment in the line above.
        // sprintf(message_print, "%d: %s", seconds_count, message1);
        // HAL_UART_Transmit(&huart1, (uint8_t*)message_print,
        // strlen(message_print),0xFFFF); //Sending in normal mode HAL_Delay(1000);

        button_press();
        switch (state) {
        case STANDBY_MODE:
            standby_mode(&state);
            break;
        case BATTLE_NO_LAST_OF_EE2028_MODE:
            battle_no_last_of_ee2028_mode(&state);
            break;
        case BATTLE_LAST_OF_EE2028_MODE:
            battle_last_of_ee2028_mode(&state);
            break;
        case DEAD_MODE:
            dead_mode(&state);
            break;
        default:
            // for debugging incase state somehow get here
            led_blink(LED_10HZ);
            break;
        }
    }
}

static void standby_mode(uint8_t* p_state)
{
    // in STANDBY_MODE, LED always on
    led_blink(LED_ALWAYS_ON);

    // in STANDBY_MODE, double press to enter BATTLE_NO_LAST_OF_EE2028_MODE
    if (double_press) {
        // goes to BATTLE_NO_LAST_OF_EE2028_MODE
        *p_state = BATTLE_NO_LAST_OF_EE2028_MODE;

        // print Entering BATTLE MODE when going to BATTLE_NO_LAST_OF_EE2028_MODE
        sprintf(uart_buffer, "Entering BATTLE MODE\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 0xFFFF);

        // clear flag
        double_press = BOOL_CLR;
    }

    // in STANDBY_MODE, single press does nothing
    if (single_press) {
        // does nothing, but still need to reset the flag
        single_press = BOOL_CLR;
    }

    // TODO add in telem code
}

static void battle_no_last_of_ee2028_mode(uint8_t* p_state)
{
    static uint8_t gun_charge = 0;
    if (gun_charge > 10) {
        // incase somehow charge gets more than 10, cap at 10/10
        gun_charge = 10;
    }

    if (gun_charge >= 5) {
        // each shot cost 5 units
        gun_charge -= 5;

        sprintf(uart_buffer, "Gun Shot:    %.*s%.*s %d%%\r\n",
            (gun_charge)*3, "[#][#][#][#][#][#][#][#][#][#]",
            (10 - gun_charge) * 3, "[ ][ ][ ][ ][ ][ ][ ][ ][ ][ ]",
            gun_charge * 10);
        HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 0xFFFF);
    }

    // in BATTLE_NO_LAST_OF_EE2028_MODE, LED blinks at 1 Hz
    led_blink(LED_1HZ);

    // in BATTLE_NO_LAST_OF_EE2028_MODE, double press to enter STANDBY_MODE
    if (double_press) {
        // goes to STANDBY_MODE
        *p_state = STANDBY_MODE;

        // print Entering STANDBY MODE when going to STANDBY_MODE
        sprintf(uart_buffer, "Entering STANDBY MODE\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 0xFFFF);

        // clear flag
        double_press = BOOL_CLR;
    }

    // in BATTLE_NO_LAST_OF_EE2028_MODE, single press charge gun by 3
    if (single_press) {
        gun_charge += 3;

        // print gun status
        sprintf(uart_buffer, "Gun Charged: %.*s%.*s %d%%\r\n",
            (gun_charge)*3, "[#][#][#][#][#][#][#][#][#][#]",
            (10 - gun_charge) * 3, "[ ][ ][ ][ ][ ][ ][ ][ ][ ][ ]",
            gun_charge * 10);
        HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 0xFFFF);

        // clear flag
        single_press = BOOL_CLR;
    }

    // TODO add in telem code
    // TODO add upside down go to BATTLE_LAST_OF_EE2028_MODE
}

static void battle_last_of_ee2028_mode(uint8_t* p_state)
{
    // in BATTLE_LAST_OF_EE2028_MODE, LED blinks at 2 Hz
    led_blink(LED_2HZ);

    // in STANDBY_MODE, double press to enter BATTLE_NO_LAST_OF_EE2028_MODE
    if (double_press) {
        // goes to BATTLE_NO_LAST_OF_EE2028_MODE
        *p_state = BATTLE_NO_LAST_OF_EE2028_MODE;
        double_press = BOOL_CLR;
    }

    // in STANDBY_MODE, single press does nothing
    if (single_press) {
        // does nothing, but still need to reset the flag
        single_press = BOOL_CLR;
    }
}

static void dead_mode(uint8_t* p_state)
{
    // in DEAD_MODE, LED blinks at 2 Hz
    led_blink(LED_2HZ);

    // in DEAD_MODE, double press does nothing
    if (double_press) {
        // does nothing, but still need to reset the flag
        double_press = BOOL_CLR;
    }

    // in DEAD_MODE, single press does nothing
    if (single_press) {
        // does nothing, but still need to reset the flag
        single_press = BOOL_CLR;
    }
}

static void UART1_Init(void)
{
    /* Pin configuration for UART. BSP_COM_Init() can do this automatically */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_6;
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
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        while (1) { }
    }
}

/**
 * @brief ISR for GPIO
 * @param uint16_t GPIO_Pin
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == USER_BUTTON_PIN) {
        button_flag = BOOL_SET;
    }
}

/**
 * @brief Takes in flag for button press and time it to determine single or
 * double press changes the single_press and double_press global variable
 * @param None
 * @retval None
 */
static void button_press(void)
{
    static uint32_t button_last_tick = 0;
    static bool button_wait_flag = BOOL_CLR;

    if (button_flag == BOOL_SET) {
        if (HAL_GetTick() - button_last_tick > 500) {
            // either a single click or the first click of a double click
            button_last_tick = HAL_GetTick();
            button_wait_flag = BOOL_SET; // used to check for single press later
        } else {
            // the last time pressed is within 500ms ==> double press
            double_press = BOOL_SET;
            single_press = BOOL_CLR;
            button_wait_flag = BOOL_CLR;
        }
        button_flag = BOOL_CLR;
    }

    if (button_wait_flag == BOOL_SET && (HAL_GetTick() - button_last_tick > 500)) {
        // wait for 0.5 to decide its single press or not
        double_press = BOOL_CLR;
        single_press = BOOL_SET;
        button_wait_flag = BOOL_CLR;
    }
}

/**
 * @brief Takes in period of blink in ms and blink the LED, period = 0 means
 * always on
 * @param uint32_t period
 * @retval None
 */
static void led_blink(uint32_t period)
{
    static uint32_t last_tick = 0;
    if (period == LED_ALWAYS_ON) {
        BSP_LED_On(LED2);
    } else if ((HAL_GetTick() - last_tick) > (period >> 1)) {
        // toggle every half a period, >> 1  to aproximate half, duty cycle is 50%
        BSP_LED_Toggle(LED2);
        last_tick = HAL_GetTick();
    }
}
