/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define M24SR64_Y_RF_DISABLE_Pin GPIO_PIN_2
#define M24SR64_Y_RF_DISABLE_GPIO_Port GPIOE
#define USB_OTG_FS_OVRCR_EXTI3_Pin GPIO_PIN_3
#define USB_OTG_FS_OVRCR_EXTI3_GPIO_Port GPIOE
#define M24SR64_Y_GPO_Pin GPIO_PIN_4
#define M24SR64_Y_GPO_GPIO_Port GPIOE
#define SPSGRF_915_GPIO3_EXTI5_Pin GPIO_PIN_5
#define SPSGRF_915_GPIO3_EXTI5_GPIO_Port GPIOE
#define SPSGRF_915_GPIO3_EXTI5_EXTI_IRQn EXTI9_5_IRQn
#define SPBTLE_RF_IRQ_EXTI6_Pin GPIO_PIN_6
#define SPBTLE_RF_IRQ_EXTI6_GPIO_Port GPIOE
#define SPBTLE_RF_IRQ_EXTI6_EXTI_IRQn EXTI9_5_IRQn
#define BUTTON_EXTI13_Pin GPIO_PIN_13
#define BUTTON_EXTI13_GPIO_Port GPIOC
#define BUTTON_EXTI13_EXTI_IRQn EXTI15_10_IRQn
#define ARD_A5_Pin GPIO_PIN_0
#define ARD_A5_GPIO_Port GPIOC
#define ARD_A4_Pin GPIO_PIN_1
#define ARD_A4_GPIO_Port GPIOC
#define ARD_A3_Pin GPIO_PIN_2
#define ARD_A3_GPIO_Port GPIOC
#define ARD_A2_Pin GPIO_PIN_3
#define ARD_A2_GPIO_Port GPIOC
#define ARD_D1_Pin GPIO_PIN_0
#define ARD_D1_GPIO_Port GPIOA
#define ARD_D0_Pin GPIO_PIN_1
#define ARD_D0_GPIO_Port GPIOA
#define ARD_D10_Pin GPIO_PIN_2
#define ARD_D10_GPIO_Port GPIOA
#define ARD_D4_Pin GPIO_PIN_3
#define ARD_D4_GPIO_Port GPIOA
#define ARD_D7_Pin GPIO_PIN_4
#define ARD_D7_GPIO_Port GPIOA
#define ARD_D13_Pin GPIO_PIN_5
#define ARD_D13_GPIO_Port GPIOA
#define ARD_D12_Pin GPIO_PIN_6
#define ARD_D12_GPIO_Port GPIOA
#define ARD_D11_Pin GPIO_PIN_7
#define ARD_D11_GPIO_Port GPIOA
#define ARD_A1_Pin GPIO_PIN_4
#define ARD_A1_GPIO_Port GPIOC
#define ARD_A0_Pin GPIO_PIN_5
#define ARD_A0_GPIO_Port GPIOC
#define ARD_D3_Pin GPIO_PIN_0
#define ARD_D3_GPIO_Port GPIOB
#define ARD_D6_Pin GPIO_PIN_1
#define ARD_D6_GPIO_Port GPIOB
#define ARD_D8_Pin GPIO_PIN_2
#define ARD_D8_GPIO_Port GPIOB
#define DFSDM1_DATIN2_Pin GPIO_PIN_7
#define DFSDM1_DATIN2_GPIO_Port GPIOE
#define ISM43362_RST_Pin GPIO_PIN_8
#define ISM43362_RST_GPIO_Port GPIOE
#define DFSDM1_CKOUT_Pin GPIO_PIN_9
#define DFSDM1_CKOUT_GPIO_Port GPIOE
#define QUADSPI_CLK_Pin GPIO_PIN_10
#define QUADSPI_CLK_GPIO_Port GPIOE
#define QUADSPI_NCS_Pin GPIO_PIN_11
#define QUADSPI_NCS_GPIO_Port GPIOE
#define OQUADSPI_BK1_IO0_Pin GPIO_PIN_12
#define OQUADSPI_BK1_IO0_GPIO_Port GPIOE
#define QUADSPI_BK1_IO1_Pin GPIO_PIN_13
#define QUADSPI_BK1_IO1_GPIO_Port GPIOE
#define QUAD_SPI_BK1_IO2_Pin GPIO_PIN_14
#define QUAD_SPI_BK1_IO2_GPIO_Port GPIOE
#define QUAD_SPI_BK1_IO3_Pin GPIO_PIN_15
#define QUAD_SPI_BK1_IO3_GPIO_Port GPIOE
#define INTERNAL_I2C2_SCL_Pin GPIO_PIN_10
#define INTERNAL_I2C2_SCL_GPIO_Port GPIOB
#define INTERNAL_I2C2_SDA_Pin GPIO_PIN_11
#define INTERNAL_I2C2_SDA_GPIO_Port GPIOB
#define ISM43362_BOOT0_Pin GPIO_PIN_12
#define ISM43362_BOOT0_GPIO_Port GPIOB
#define ISM43362_WAKEUP_Pin GPIO_PIN_13
#define ISM43362_WAKEUP_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_14
#define LED2_GPIO_Port GPIOB
#define SPSGRF_915_SDN_Pin GPIO_PIN_15
#define SPSGRF_915_SDN_GPIO_Port GPIOB
#define INTERNAL_UART3_TX_Pin GPIO_PIN_8
#define INTERNAL_UART3_TX_GPIO_Port GPIOD
#define INTERNAL_UART3_RX_Pin GPIO_PIN_9
#define INTERNAL_UART3_RX_GPIO_Port GPIOD
#define LPS22HB_INT_DRDY_EXTI0_Pin GPIO_PIN_10
#define LPS22HB_INT_DRDY_EXTI0_GPIO_Port GPIOD
#define LPS22HB_INT_DRDY_EXTI0_EXTI_IRQn EXTI15_10_IRQn
#define LSM6DSL_INT1_EXTI11_Pin GPIO_PIN_11
#define LSM6DSL_INT1_EXTI11_GPIO_Port GPIOD
#define LSM6DSL_INT1_EXTI11_EXTI_IRQn EXTI15_10_IRQn
#define USB_OTG_FS_PWR_EN_Pin GPIO_PIN_12
#define USB_OTG_FS_PWR_EN_GPIO_Port GPIOD
#define SPBTLE_RF_SPI3_CSN_Pin GPIO_PIN_13
#define SPBTLE_RF_SPI3_CSN_GPIO_Port GPIOD
#define ARD_D2_Pin GPIO_PIN_14
#define ARD_D2_GPIO_Port GPIOD
#define ARD_D2_EXTI_IRQn EXTI15_10_IRQn
#define HTS221_DRDY_EXTI15_Pin GPIO_PIN_15
#define HTS221_DRDY_EXTI15_GPIO_Port GPIOD
#define HTS221_DRDY_EXTI15_EXTI_IRQn EXTI15_10_IRQn
#define VL53L0X_XSHUT_Pin GPIO_PIN_6
#define VL53L0X_XSHUT_GPIO_Port GPIOC
#define VL53L0X_GPIO1_EXTI7_Pin GPIO_PIN_7
#define VL53L0X_GPIO1_EXTI7_GPIO_Port GPIOC
#define VL53L0X_GPIO1_EXTI7_EXTI_IRQn EXTI9_5_IRQn
#define LIS3MDL_DRDY_EXTI8_Pin GPIO_PIN_8
#define LIS3MDL_DRDY_EXTI8_GPIO_Port GPIOC
#define LIS3MDL_DRDY_EXTI8_EXTI_IRQn EXTI9_5_IRQn
#define LED3_WIFI__LED4_BLE_Pin GPIO_PIN_9
#define LED3_WIFI__LED4_BLE_GPIO_Port GPIOC
#define SPBTLE_RF_RST_Pin GPIO_PIN_8
#define SPBTLE_RF_RST_GPIO_Port GPIOA
#define USB_OTG_FS_VBUS_Pin GPIO_PIN_9
#define USB_OTG_FS_VBUS_GPIO_Port GPIOA
#define USB_OTG_FS_ID_Pin GPIO_PIN_10
#define USB_OTG_FS_ID_GPIO_Port GPIOA
#define USB_OTG_FS_DM_Pin GPIO_PIN_11
#define USB_OTG_FS_DM_GPIO_Port GPIOA
#define USB_OTG_FS_DP_Pin GPIO_PIN_12
#define USB_OTG_FS_DP_GPIO_Port GPIOA
#define SYS_JTMS_SWDIO_Pin GPIO_PIN_13
#define SYS_JTMS_SWDIO_GPIO_Port GPIOA
#define SYS_JTCK_SWCLK_Pin GPIO_PIN_14
#define SYS_JTCK_SWCLK_GPIO_Port GPIOA
#define ARD_D9_Pin GPIO_PIN_15
#define ARD_D9_GPIO_Port GPIOA
#define INTERNAL_SPI3_SCK_Pin GPIO_PIN_10
#define INTERNAL_SPI3_SCK_GPIO_Port GPIOC
#define INTERNAL_SPI3_MISO_Pin GPIO_PIN_11
#define INTERNAL_SPI3_MISO_GPIO_Port GPIOC
#define INTERNAL_SPI3_MOSI_Pin GPIO_PIN_12
#define INTERNAL_SPI3_MOSI_GPIO_Port GPIOC
#define PMOD_RESET_Pin GPIO_PIN_0
#define PMOD_RESET_GPIO_Port GPIOD
#define PMOD_SPI2_SCK_Pin GPIO_PIN_1
#define PMOD_SPI2_SCK_GPIO_Port GPIOD
#define PMOD_IRQ_EXTI12_Pin GPIO_PIN_2
#define PMOD_IRQ_EXTI12_GPIO_Port GPIOD
#define PMOD_UART2_CTS_Pin GPIO_PIN_3
#define PMOD_UART2_CTS_GPIO_Port GPIOD
#define PMOD_UART2_RTS_Pin GPIO_PIN_4
#define PMOD_UART2_RTS_GPIO_Port GPIOD
#define PMOD_UART2_TX_Pin GPIO_PIN_5
#define PMOD_UART2_TX_GPIO_Port GPIOD
#define PMOD_UART2_RX_Pin GPIO_PIN_6
#define PMOD_UART2_RX_GPIO_Port GPIOD
#define STSAFE_A100_RESET_Pin GPIO_PIN_7
#define STSAFE_A100_RESET_GPIO_Port GPIOD
#define SYS_JTD0_SWO_Pin GPIO_PIN_3
#define SYS_JTD0_SWO_GPIO_Port GPIOB
#define ARD_D5_Pin GPIO_PIN_4
#define ARD_D5_GPIO_Port GPIOB
#define SPSGRF_915_SPI3_CSN_Pin GPIO_PIN_5
#define SPSGRF_915_SPI3_CSN_GPIO_Port GPIOB
#define ST_LINK_UART1_TX_Pin GPIO_PIN_6
#define ST_LINK_UART1_TX_GPIO_Port GPIOB
#define ST_LINK_UART1_RX_Pin GPIO_PIN_7
#define ST_LINK_UART1_RX_GPIO_Port GPIOB
#define ARD_D15_Pin GPIO_PIN_8
#define ARD_D15_GPIO_Port GPIOB
#define ARD_D14_Pin GPIO_PIN_9
#define ARD_D14_GPIO_Port GPIOB
#define ISM43362_SPI3_CSN_Pin GPIO_PIN_0
#define ISM43362_SPI3_CSN_GPIO_Port GPIOE
#define ISM43362_DRDY_EXTI1_Pin GPIO_PIN_1
#define ISM43362_DRDY_EXTI1_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

#define STANDBY_MODE 0
#define BATTLE_NO_LAST_OF_EE2028_MODE 1
#define BATTLE_LAST_OF_EE2028_MODE 2
#define DEAD_MODE 3

#define LED_ALWAYS_ON 0
#define LED_1HZ 1000
#define LED_2HZ 500
#define LED_10HZ 100

#define UART_BUFFER_SIZE 128

#define BOOL_SET 1
#define BOOL_CLR 0

// sensor stuff
#define D6D_SRC_UPSIDEDOWN 0x10

#define ACCEL_UPPER_THRES 11 // 11 m/s^2
#define GYRO_UPPER_THRES 100 // 100 dps
#define ACCEL_SQR_UPPER_THRES ACCEL_UPPER_THRES* ACCEL_UPPER_THRES
#define GYRO_SQR_UPPER_THRES GYRO_UPPER_THRES* GYRO_UPPER_THRES
#define D6D_SRC_UPSIDEDOWN 0x10

#define MAG_UPPER_THRES 4000 // 4000 mG
#define MAG_SQR_UPPER_THRES MAG_UPPER_THRES* MAG_UPPER_THRES

#define PRESS_UPPER_THRES 102 // 102 kPa
#define HUM_LOWER_THRES 80 // 80%
#define TEMP_UPPER_THRES 30 // 40 degC

// SPSGRF Stuffs
// for drone to station
#define SPSGRF_BUFFER_SIZE 64
#define PRESSURE_NBYTES 4
#define TEMP_NBYTES 4
#define HUMIDITY_NBYTES 4
#define ACCEL_NBYTES 12
#define GYRO_NBYTES 12
#define MAG_NBYTES 6
#define STATUS_NBYTES 2
#define TELEM_NBYTES PRESSURE_NBYTES + TEMP_NBYTES + HUMIDITY_NBYTES + ACCEL_NBYTES + GYRO_NBYTES + MAG_NBYTES + STATUS_NBYTES

// STATE_NBYTES[0] [1:0] = drone states
// STATE_NBYTES[0] [5:2] = drone charge
// STATE_NBYTES[1] [5:0] = threshold for pressure, temp, hum, accel, gyro, mag
#define STATE_MSK 0x03
#define CHARGES_MSK 0x3C
#define P_TH_MSK 0x20
#define T_TH_MSK 0x10
#define H_TH_MSK 0x08
#define A_TH_MSK 0x04
#define G_TH_MSK 0x02
#define M_TH_MSK 0x01
#define P_TH_POS 5
#define T_TH_POS 4
#define H_TH_POS 3
#define A_TH_POS 2
#define G_TH_POS 1
#define M_TH_POS 0

// for station to drone
#define CONTORL_NBYTES 1
#define CHARGE_REQ_MSK 0x10
#define STATE_POS 0
#define CHARGE_POS 4


// EXTI priorities
#define EXTI15_10_IRQn_PREEMPT_PRIO 0x01
#define EXTI15_10_IRQn_SUB_PRIO 0x00

#define LSM6DSL_INT1_EXTI11_EXTI_IRQn_PREEMPT_PRIO EXTI15_10_IRQn_PREEMPT_PRIO
#define LSM6DSL_INT1_EXTI11_EXTI_IRQn_SUB_PRIO EXTI15_10_IRQn_SUB_PRIO

#define HTS221_DRDY_EXTI15_EXTI_IRQn_PREEMPT_PRIO EXTI15_10_IRQn_PREEMPT_PRIO
#define HTS221_DRDY_EXTI15_EXTI_IRQn_SUB_PRIO EXTI15_10_IRQn_SUB_PRIO

#define LPS22HB_INT_DRDY_EXTI0_EXTI_IRQn_PREEMPT_PRIO EXTI15_10_IRQn_PREEMPT_PRIO
#define LPS22HB_INT_DRDY_EXTI0_EXTI_IRQn_SUB_PRIO EXTI15_10_IRQn_SUB_PRIO

#define EXTI9_5_IRQn_PREEMPT_PRIO 0x00
#define EXTI9_5_IRQn_SUB_PRIO 0x00

#define LIS3MDL_DRDY_EXTI8_EXTI_IRQn_PREEMPT_PRIO EXTI9_5_IRQn_PREEMPT_PRIO
#define LIS3MDL_DRDY_EXTI8_EXTI_IRQn_SUB_PRIO EXTI9_5_IRQn_SUB_PRIO

#define SPSGRF_915_GPIO3_EXTI5_EXTI_IRQn_PREEMPT_PRIO EXTI9_5_IRQn_PREEMPT_PRIO
#define SPSGRF_915_GPIO3_EXTI5_EXTI_IRQn_SUB_PRIO EXTI9_5_IRQn_SUB_PRIO

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
