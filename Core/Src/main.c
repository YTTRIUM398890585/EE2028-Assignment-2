/******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 * (c) EE2028 Teaching Team
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include "math.h"
#include "stdbool.h"
#include "stdint.h"
#include "stdio.h"
#include "string.h"

// button and led
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01.h"

// Sensors
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_gyro.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_hsensor.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_magneto.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_psensor.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_tsensor.h"
#include "../../Drivers/BSP/Components/lsm6dsl/lsm6dsl.h"

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
uint32_t last_of_ee2028_tick = 0;

/* Button
button_press_tick for the moment the button is pressed
button_flag for EXTI
button_wait_flag for single and double press logic
*/
volatile uint32_t button_press_tick = 0;
volatile bool button_flag = BOOL_CLR;
bool single_press = BOOL_CLR;
bool double_press = BOOL_CLR;

/* UART
Tx buffer make sure does not excedd 32 - 1 characters when added with whatever when printing
*/
char uart_buffer[UART_BUFFER_SIZE];
UART_HandleTypeDef huart1;

/*
Sensor and Telem
*/
uint32_t last_telem_tick = 0;

// accel gyro and d6d stuff
volatile bool acc_gyro_d6d_ready = BOOL_CLR;
float accel_data[3];
float gyro_data[3];
uint8_t d6d_data = 0;
bool acc_thres_flag = BOOL_CLR;
bool gyro_thres_flag = BOOL_CLR;

// magnetometer
volatile bool mag_ready = BOOL_CLR;
int16_t mag_data[3];
bool mag_thres_flag = BOOL_CLR;

// pressure
volatile bool press_ready = BOOL_CLR;
float pressure_data;
bool pressure_thres_flag = BOOL_CLR;

// hum and temp
volatile bool hum_temp_ready = BOOL_SET; // needed to kick start read
float humidity_data;
float temp_data;
bool humidity_thres_flag = BOOL_CLR;
bool temp_thres_flag = BOOL_CLR;


// for rf

/* Function Prototype --------------------------------------------------------*/
static void standby_mode(uint8_t* p_state);
static void battle_no_last_of_ee2028_mode(uint8_t* p_state);
static void battle_last_of_ee2028_mode(uint8_t* p_state);
static void dead_mode(uint8_t* p_state);

static void UART1_Init(void);
static void led_blink(uint32_t period);
static void button_press(void);

static void read_ready_acc_gyro_d6d(float* p_acc, float* p_gyro, uint8_t* p_d6d, bool* p_acc_thres_flag, bool* p_gyro_thres_flag);
static void read_ready_hum_temp(float* p_hum, float* p_temp, bool* humidity_thres_flag, bool* temp_thres_flag);

static void read_mag(int16_t* p_mag);
static float read_pressure(void);

static void print_threshold_acc(void);
static void print_threshold_gyro(void);
static void print_threshold_mag(void);
static void print_threshold_hum(void);
static void print_threshold_press(void);
static void print_threshold_temp(void);

static void LSM6DSL_AccGyroInit(void);
static void HTS221_HumTempInit(void);

int main(void)
{
    HAL_Init();
    UART1_Init();

    BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);
    BSP_LED_Init(LED2);

    // must init this for I2C to configure the sensors
    SENSOR_IO_Init();

    LSM6DSL_AccGyroInit();
    HTS221_HumTempInit();
    BSP_MAGNETO_Init();
    BSP_PSENSOR_Init();

    // print Entering STANDBY MODE when going to STANDBY_MODE
    sprintf(uart_buffer, "Entering STANDBY MODE\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 0xFFFF);

    while (1) {
        button_press();

        // read data if DRDY triggered
        read_ready_acc_gyro_d6d(accel_data, gyro_data, &d6d_data, &acc_thres_flag, &gyro_thres_flag);
        read_ready_hum_temp(&humidity_data, &temp_data, &humidity_thres_flag, &temp_thres_flag);

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
        // if upside down, go to BATTLE_LAST_OF_EE2028_MODE else BATTLE_NO_LAST_OF_EE2028_MODE
        uint8_t d6d_src = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_D6D_SRC);

        if (d6d_src & D6D_SRC_UPSIDEDOWN) {
            *p_state = BATTLE_LAST_OF_EE2028_MODE;
            last_of_ee2028_tick = HAL_GetTick();
        } else {
            *p_state = BATTLE_NO_LAST_OF_EE2028_MODE;
        }

        // print Entering BATTLE MODE when going to BATTLE_NO_LAST_OF_EE2028_MODE/BATTLE_LAST_OF_EE2028_MODE as thats how it is logically
        sprintf(uart_buffer, "Entering BATTLE MODE\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 0xFFFF);

        last_telem_tick = 0;

        // clear flag
        double_press = BOOL_CLR;
        return;
    }

    // in STANDBY_MODE, single press does nothing
    if (single_press) {
        // does nothing, but still need to reset the flag
        single_press = BOOL_CLR;
    }

    // read GMPH telem and send UART @ 1 Hz
    if (HAL_GetTick() - last_telem_tick >= 1000) {
        read_mag(mag_data);
        pressure_data = read_pressure();

        print_threshold_gyro();
        print_threshold_mag();
        print_threshold_press();
        print_threshold_hum();

        if ((gyro_thres_flag | mag_thres_flag | pressure_thres_flag | humidity_thres_flag) == BOOL_CLR) {
            // no threshold violation, print normal telem
            // split to multiple messages to fit in buffer
            sprintf(uart_buffer, "Gx: %.2f dps, Gy: %.2f dps, Gz: %.2f dps, ", gyro_data[0], gyro_data[1], gyro_data[2]);
            HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 0xFFFF);

            sprintf(uart_buffer, "Mx: %d mG, My: %d mG, Mz: %d mG, ", mag_data[0], mag_data[1], mag_data[2]);
            HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 0xFFFF);

            sprintf(uart_buffer, "P: %.2f kPA, H: %.2f%%\r\n", pressure_data, humidity_data);
            HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 0xFFFF);
        }

        last_telem_tick = HAL_GetTick();
    }
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
        last_telem_tick = 0;

        // clear flag
        double_press = BOOL_CLR;
        return;
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

    // read TPHAGM telem and send UART @ 1 Hz
    if (HAL_GetTick() - last_telem_tick >= 1000) {
        pressure_data = read_pressure();
        read_mag(mag_data);

        print_threshold_acc();
        print_threshold_gyro();
        print_threshold_mag();
        print_threshold_hum();
        print_threshold_press();
        print_threshold_temp();

        if ((acc_thres_flag | gyro_thres_flag | mag_thres_flag | pressure_thres_flag | humidity_thres_flag | temp_thres_flag) == BOOL_CLR) {
            // no threshold violation, print normal telem

            // split to multiple messages to fit in buffer
            sprintf(uart_buffer, "T: %.2f degC, P: %.2f kPA, H: %.2f%%, ", temp_data, pressure_data, humidity_data);
            HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 0xFFFF);

            sprintf(uart_buffer, "Ax: %.2f ms-2, Ay: %.2f ms-2, Az: %.2f ms-2, ", accel_data[0], accel_data[1], accel_data[2]);
            HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 0xFFFF);

            sprintf(uart_buffer, "Gx: %.2f dps, Gy: %.2f dps, Gz: %.2f dps, ", gyro_data[0], gyro_data[1], gyro_data[2]);
            HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 0xFFFF);

            sprintf(uart_buffer, "Mx: %d mG, My: %d mG, Mz: %d mG\r\n", mag_data[0], mag_data[1], mag_data[2]);
            HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 0xFFFF);
        }

        last_telem_tick = HAL_GetTick();
    }

    // if upside down, go to BATTLE_LAST_OF_EE2028_MODE
    if (d6d_data & D6D_SRC_UPSIDEDOWN) {
        *p_state = BATTLE_LAST_OF_EE2028_MODE;
        last_telem_tick = 0;
        last_of_ee2028_tick = HAL_GetTick();
        return;
    }
}

static void battle_last_of_ee2028_mode(uint8_t* p_state)
{
    // in BATTLE_LAST_OF_EE2028_MODE, LED blinks at 2 Hz
    led_blink(LED_2HZ);

    // send help UART @ 1 Hz
    if (HAL_GetTick() - last_telem_tick >= 1000) {
        sprintf(uart_buffer, "Drone Was Attacked! \r\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 0xFFFF);
        last_telem_tick = HAL_GetTick();
    }

    // TODO Disable interupt for telem monitoring

    // in STANDBY_MODE, single press does nothing
    if (single_press) {
        // does nothing, but still need to reset the flag
        single_press = BOOL_CLR;
    }

    // in BATTLE_LAST_OF_EE2028_MODE, double press within 10 s to enter BATTLE_NO_LAST_OF_EE2028_MODE else DEAD_MODE
    if (HAL_GetTick() - last_of_ee2028_tick <= 10000) {
        if (double_press) {
            uint8_t d6d_src = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_D6D_SRC);

            // if not upside down, go to BATTLE_NO_LAST_OF_EE2028_MODE
            if (!(d6d_src & D6D_SRC_UPSIDEDOWN)) {
                *p_state = BATTLE_NO_LAST_OF_EE2028_MODE;
                sprintf(uart_buffer, "Rescued :D \r\n");
                HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 0xFFFF);
            }

            double_press = BOOL_CLR;
            return;
        }
    } else {
        // goes to DEAD_MODE
        *p_state = DEAD_MODE;
        sprintf(uart_buffer, "Dead :( \r\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 0xFFFF);
        return;
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
    // button
    if (GPIO_Pin == USER_BUTTON_PIN) {
        button_press_tick = HAL_GetTick();
        button_flag = BOOL_SET;
    }

    // EXTI from LSM6DSL, flag to read Accel Data, Gyro Data and LSM6DSL_ACC_GYRO_D6D_SRC
    if (GPIO_Pin == GPIO_PIN_11) {
        acc_gyro_d6d_ready = BOOL_SET;
    }

    // EXTI from HTS221, flag to read Humidity and Temperature
    if (GPIO_Pin == GPIO_PIN_15) {
        hum_temp_ready = BOOL_SET;
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
        if (button_press_tick - button_last_tick > 500) {
            // new event: either a single click or the first click of a double click
            button_last_tick = button_press_tick;
            button_wait_flag = BOOL_SET; // used to check for single press later
        } else {
            // forces the next click to be a new event
            // prevents triple click within 500ms to be taken as two double clicks
            button_last_tick = 0;

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
    } else if ((HAL_GetTick() - last_tick) >= (period >> 1)) {
        // toggle every half a period, >> 1  to aproximate half, duty cycle is 50%
        BSP_LED_Toggle(LED2);
        last_tick = HAL_GetTick();
    }
}

/**
 * @brief read acc, gyro, d6d from LSM6DSL when DRDY is flagged
 *        using LSM6DSL.C straight as BSP is bypasssed in init
 * @param p_acc pointer to float array of 3 elements for accel
 * @param p_gyro pointer to float array of 3 elements for gyro
 * @param p_d6d pointer uint8_t for the orientation
 * @param p_acc_thres_flag pointer bool for the threshold monitoring
 * @param p_gyro_thres_flag pointer bool for the threshold monitoring
 * @retval None
 */
static void read_ready_acc_gyro_d6d(float* p_acc, float* p_gyro, uint8_t* p_d6d, bool* p_acc_thres_flag, bool* p_gyro_thres_flag)
{
    // only read when data is ready to reduce I2C overhead and unnecessary reads
    if (acc_gyro_d6d_ready == BOOL_SET) {
        int16_t accel_data_i16[3] = { 0 }; // array to store the x, y and z readings.
        LSM6DSL_AccReadXYZ(accel_data_i16); // read accelerometer
        // the function above returns 16 bit integers which are acceleration in mg (9.8/1000 m/s^2).
        // Converting to float in m/s^2
        for (int i = 0; i < 3; i++) {
            *(p_acc + i) = (float)accel_data_i16[i] * (9.8 / 1000.0f);
        }

        // the function does sensitivity conversion to mdps and returns float in mdps
        LSM6DSL_GyroReadXYZAngRate(p_gyro);
        // Converting to float in dps
        for (int i = 0; i < 3; i++) {
            *(p_gyro + i) = *(p_gyro + i) / 1000.0f;
        }

        // flag threshold if the magnitude exceed
        float magnitude = pow(*(p_acc), 2) + pow(*(p_acc + 1), 2) + pow(*(p_acc + 2), 2);
        *p_acc_thres_flag = (magnitude > ACCEL_SQR_UPPER_THRES) ? BOOL_SET : BOOL_CLR;

        magnitude = pow(*(p_gyro), 2) + pow(*(p_gyro + 1), 2) + pow(*(p_gyro + 2), 2);
        *p_gyro_thres_flag = (magnitude > GYRO_SQR_UPPER_THRES) ? BOOL_SET : BOOL_CLR;

        // read the D6D register to give the orientation
        *p_d6d = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_D6D_SRC);

        // clear the DRDY flag
        acc_gyro_d6d_ready = BOOL_CLR;
    }
}

/**
 * @brief read mag from LIS3MDL
 * @param int16_t* p_mag pointer to int16_t array of 3 elements
 * @retval None
 */
static void read_mag(int16_t* p_mag)
{
    // the function that actually reads the xyz is LIS3MDL_MagReadXYZ in lis3mdl.c
    // the function also does sensitivity conversion to mGauss
    // returns int16_t in mGauss
    LIS3MDL_MagReadXYZ(p_mag);
}

/**
 * @brief read pressure from LPS22HB
 * @param None
 * @retval humidity
 */
static float read_pressure(void)
{
    // the function that actually reads the value is LPS22HB_P_ReadPressure in lps22hb.c
    // returns as float in hPa, the read does the concatenation of 3 bytes, 2's complement and *100/4096 then /100
    // divide 10 to convert to kPa
    return BSP_PSENSOR_ReadPressure() / 10;
}

/**
 * @brief read humidity and temperature from HTS221 when data is ready
 * @param p_hum pointer to float
 * @param p_temp pointer to float
 * @param humidity_thres_flag pointer to bool flag
 * @param temp_thres_flag pointer to bool flag
 * @retval None
 */
static void read_ready_hum_temp(float* p_hum, float* p_temp, bool* humidity_thres_flag, bool* temp_thres_flag)
{
    if (hum_temp_ready == BOOL_SET) {
        // returns as float in %
        *p_hum = HTS221_H_ReadHumidity(HTS221_I2C_ADDRESS);

        // returns as float in deg c
        *p_temp = HTS221_T_ReadTemp(HTS221_I2C_ADDRESS);

        // flag threshold if the magnitude exceed
        *humidity_thres_flag = *p_hum < HUM_LOWER_THRES ? BOOL_SET : BOOL_CLR;
        *temp_thres_flag = *p_temp > TEMP_UPPER_THRES ? BOOL_SET : BOOL_CLR;
        
        // clear the DRDY flag
        hum_temp_ready = BOOL_CLR;
    }
}

/**
 * @brief print warning if over threshold for accel
 * @param None
 * @retval None
 */
static void print_threshold_acc(void){
    if (acc_thres_flag == BOOL_SET) {
        // accel exceed print warning
        sprintf(uart_buffer, "|A|: %.2f ms-2 exceed threshold of %d ms-2\r\n", sqrt(pow(accel_data[0], 2) + pow(accel_data[1], 2) + pow(accel_data[2], 2)), ACCEL_UPPER_THRES);
        HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 0xFFFF);
    }
}

/**
 * @brief print warning if over threshold for gyro
 * @param None
 * @retval None
 */
static void print_threshold_gyro(void){
    if (gyro_thres_flag == BOOL_SET) {
        // gyro exceed print warning
        sprintf(uart_buffer, "|G|: %.2f dps exceed threshold of %d dps\r\n", sqrt(pow(gyro_data[0], 2) + pow(gyro_data[1], 2) + pow(gyro_data[2], 2)), GYRO_UPPER_THRES);
        HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 0xFFFF);
    }
}

/**
 * @brief print warning if over threshold for mag
 * @param None
 * @retval None
 */
static void print_threshold_mag(void){
    if (mag_thres_flag == BOOL_SET) {
        // mag exceed print warning
        sprintf(uart_buffer, "|M|: %.2f mG exceed threshold of %d mG\r\n", sqrt(pow(mag_data[0], 2) + pow(mag_data[1], 2) + pow(mag_data[2], 2)), MAG_UPPER_THRES);
        HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 0xFFFF);
    }
}

/**
 * @brief print warning if over threshold for hum
 * @param None
 * @retval None
 */
static void print_threshold_hum(void){
    if (humidity_thres_flag == BOOL_SET) {
        // hum exceed print warning
        sprintf(uart_buffer, "H: %.2f%% exceed threshold of %d%%\r\n", humidity_data, HUM_LOWER_THRES);
        HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 0xFFFF);
    }
}

/**
 * @brief print warning if over threshold for press
 * @param None
 * @retval None
 */
static void print_threshold_press(void){
    if (pressure_thres_flag == BOOL_SET) {
        // press exceed print warning
        sprintf(uart_buffer, "P: %.2f kPa exceed threshold of %d kPa\r\n", pressure_data, PRESS_UPPER_THRES);
        HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 0xFFFF);
    }
}

/**
 * @brief print warning if over threshold for temp
 * @param None
 * @retval None
 */
static void print_threshold_temp(void){
    if (temp_thres_flag == BOOL_SET) {
        // press exceed print warning
        sprintf(uart_buffer, "T: %.2f degC exceed threshold of %d degC\r\n", temp_data, TEMP_UPPER_THRES);
        HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 0xFFFF);
    }
}

/**
 * @brief modified init for LSM6DSL accelerometer and gyro to support 6D and DRDY interrupt,
 *        also init GPIO PD11 for the EXTI
 * @param None
 * @retval None
 */
static void LSM6DSL_AccGyroInit(void)
{
    /*
    configuring the GPIO for EXTI from LSM6DSL at PD11
    */
    GPIO_InitTypeDef gpio_init_structure;

    __HAL_RCC_GPIOD_CLK_ENABLE();

    // Configure PD11 pin as input with External interrupt
    gpio_init_structure.Pin = GPIO_PIN_11;
    gpio_init_structure.Pull = GPIO_PULLDOWN;
    gpio_init_structure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    gpio_init_structure.Mode = GPIO_MODE_IT_RISING; // interupt is active high

    HAL_GPIO_Init(GPIOD, &gpio_init_structure);

    // Enable and set EXTI Interrupt priority
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, EXTI15_10_IRQn_PREEMPT_PRIO, EXTI15_10_IRQn_SUB_PRIO);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

    //////////////////////////////////////////////////////////////////////////////////////////
    uint8_t ctrl = 0x00;
    uint8_t tmp;

    /*
    configuring the LSM6DSL for 6D EXTI through INT1
    */
    // write 0x60 to 0x10 CTRL1_XL to set ODR_XL = 416 Hz and turn on device, FS_XL = ±2 g
    SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL1_XL, 0x60);

    // Write 0x80 to 0x58 TAP_CFG Enable interrupts; latched mode disabled
    SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_TAP_CFG1, 0x80);

    // thershold at 50 deg seems good
    // Write 0x60 to 0x59 TAP_THS_6D Set 6D threshold (SIXD_THS[1:0] = 11b = 50 degrees), D4D disable
    SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_TAP_THS_6D, 0x60);
    //    // Write 0x40 to 0x59 TAP_THS_6D Set 6D threshold (SIXD_THS[1:0] = 10b = 60 degrees), D4D disable
    //    SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_TAP_THS_6D, 0x40);
    //    // Write 0x20 to 0x59 TAP_THS_6D Set 6D threshold (SIXD_THS[1:0] = 01b = 70 degrees), D4D disable
    //	SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_TAP_THS_6D, 0x20);
    //    // Write 0x00 to 0x59 TAP_THS_6D Set 6D threshold (SIXD_THS[1:0] = 00b = 80 degrees), D4D disable
    //	SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_TAP_THS_6D, 0x00);

    // Write set HPCF_XL[1:0] to 11, INPUT_COMPOSITE = 1, LOW_PASS_ON_6D = 1 in CTRL8_XL to
    // to config LPF2 filter to 6D functionality
    // this LPF has a very low BW, to avoid triggering of 6d when shaking the board
    tmp = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL8_XL);

    ctrl = 0x69;
    tmp &= ~(0x6B);
    tmp |= ctrl;
    SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL8_XL, tmp);

    // Write 04h to 0x5E MD1_CFG 6D interrupt driven to INT1 pin
    SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_MD1_CFG, 0x04);

    /*
    configuring the LSM6DSL for accel
    */

    // all necessary config are done along with the d6d

    /*
    configuring the LSM6DSL for gyro
    */
    tmp = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL2_G);

    // Write value to GYRO MEMS CTRL2_G register: FS = 2000 dps and Data Rate 52 Hz
    ctrl = LSM6DSL_GYRO_FS_2000 | LSM6DSL_ODR_52Hz;
    tmp &= ~(0xFC);
    tmp |= ctrl;
    SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL2_G, tmp);

    /*
    Write value to CTRL3_C register: BDU and Auto-increment and active high int
    the same register to configure for both acc nd gyro
    */
    // Read CTRL3_C
    tmp = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL3_C);

    ctrl = LSM6DSL_BDU_BLOCK_UPDATE | LSM6DSL_ACC_GYRO_IF_INC_ENABLED;
    tmp &= ~(0x64); // clear BDU, IF_INC and H_LACTIVE (for interrupt to be active high)
    tmp |= ctrl;
    SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL3_C, tmp);

    /*
    configuring the LSM6DSL for accel gyro DRDY INT1
    */
    // Write 0x03 to 0x0D INT1_CTRL, DRDY for both accel and gyro interrupt driven to INT1 pin
    SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_INT1_CTRL, 0x03);

    // Write 0x80 to DRDY_PULSE_CFG_G (0Bh) to make DRDY be a pulse and not latched
    SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_DRDY_PULSE_CFG_G, 0x80);

    // set DRDY_MASK to 1 in CTRL4_C (13h) to wait LPF before DRDY
    tmp = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL4_C);

    ctrl = 0x08;
    tmp &= ~(0x09); // clear DRDY_MASK and bit 0 must set to 0
    tmp |= ctrl;
    SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL4_C, tmp);
}

/**
 * @brief modified init for HTS221 humidity and temperature to support DRDY interrupt,
 *        also init GPIO PD15 for the EXTI
 * @param None
 * @retval None
 */
static void HTS221_HumTempInit(void)
{
    /*
    configuring the GPIO for EXTI from LSM6DSL at PD15
    */
    GPIO_InitTypeDef gpio_init_structure;

    __HAL_RCC_GPIOD_CLK_ENABLE();

    // Configure PD15 pin as input with External interrupt
    gpio_init_structure.Pin = GPIO_PIN_15;
    gpio_init_structure.Pull = GPIO_PULLDOWN;
    gpio_init_structure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    gpio_init_structure.Mode = GPIO_MODE_IT_RISING; // interupt is active high

    HAL_GPIO_Init(GPIOD, &gpio_init_structure);

    // Enable and set EXTI Interrupt priority
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, EXTI15_10_IRQn_PREEMPT_PRIO, EXTI15_10_IRQn_SUB_PRIO);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

    //////////////////////////////////////////////////////////////////////////////////////////
    uint8_t tmp;

    /*
    DRDY config
    */
    // DRDY_H_L 0 active high
    // PP_OD 0 push pull
    // DRDY_EN 1 enabled
    // clear 0xC4
    // set 0x04
    tmp = SENSOR_IO_Read(HTS221_I2C_ADDRESS, HTS221_CTRL_REG3);

    tmp &= ~0xC4;
    tmp |= 0x04;

    SENSOR_IO_Write(HTS221_I2C_ADDRESS, HTS221_CTRL_REG3, tmp);

    /*
    Init for H and T
    */
    /* Read CTRL_REG1 */
    tmp = SENSOR_IO_Read(HTS221_I2C_ADDRESS, HTS221_CTRL_REG1);

    /* Enable BDU */
    tmp &= ~HTS221_BDU_MASK;
    tmp |= (1 << HTS221_BDU_BIT);

    /* Set default ODR */
    tmp &= ~HTS221_ODR_MASK;
    tmp |= (uint8_t)0x01; /* Set ODR to 1Hz */

    /* Activate the device */
    tmp |= HTS221_PD_MASK;

    /* Apply settings to CTRL_REG1 */
    SENSOR_IO_Write(HTS221_I2C_ADDRESS, HTS221_CTRL_REG1, tmp);    
}
