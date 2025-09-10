
/*
* This file is part of Betaflight.
*
* Betaflight is free software. You can redistribute this software
* and/or modify this software under the terms of the GNU General
* Public License as published by the Free Software Foundation,
* either version 3 of the License, or (at your option) any later
* version.
*
* Betaflight is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*
* See the GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public
* License along with this software.
*
* If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#define FC_TARGET_MCU       STM32H743
#define BOARD_NAME          KOAFC
#define MANUFACTURER_ID     FALCONSHOP
#define SYSTEM_HSE_MHZ      16

// *************** IMU generic ***********************
#define USE_ACC
#define USE_GYRO
#define USE_GYRO_CLKIN
#define USE_SPI
#define USE_SPI_GYRO
#define USE_ACCGYRO_SPI_BMI088
#define USE_EXTI
#define USE_GYRO_EXTI

// *************** SPI1 IMU0 BMI088 ******************
#define USE_SPI_DEVICE_1
#define SPI1_SDI_PIN         PA6
#define SPI1_SDO_PIN         PA7
#define SPI1_SCK_PIN         PA5

#define GYRO_1_SPI_INSTANCE  SPI1
#define GYRO_1_CS_PIN        PA3
#define GYRO_1_ACC_CS_PIN    PA2
#define GYRO_1_EXTI_PIN      PA1
#define GYRO_1_ALIGN         CW0_DEG_FLIP

// *************** SPI4 IMU1 BMI088 ******************
#define USE_SPI_DEVICE_4
#define SPI4_SCK_PIN         PE2
#define SPI4_SDI_PIN         PE5
#define SPI4_SDO_PIN         PE6

#define GYRO_2_SPI_INSTANCE  SPI4
#define GYRO_2_CS_PIN        PC2
#define GYRO_2_ACC_CS_PIN    PE4
#define GYRO_2_EXTI_PIN      PE3
#define GYRO_2_ALIGN         CW0_DEG_FLIP

// ****** Accelerometer orientation ******
#define DEFAULT_ALIGN_BOARD_YAW         90
#define DEFAULT_ALIGN_BOARD_PITCH       180


// *************** I2C1 Barometer *******************
#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C1_SCL_PIN         PB6
#define I2C1_SDA_PIN         PB7

#define USE_BARO
#define BARO_I2C_INSTANCE    I2CDEV_1
#define USE_BARO_DPS310

// *************** I2C4 Magnetometer & GPS **********
#define USE_I2C_DEVICE_4
#define I2C4_SCL_PIN         PD12
#define I2C4_SDA_PIN         PD13

#define USE_MAG
#define MAG_I2C_INSTANCE     I2CDEV_4
#define USE_MAG_QMC5883

#define USE_GPS
#define GPS_UART             SERIAL_PORT_USART1

// *************** Motor *******************
#define MOTOR1_PIN           PE9
#define MOTOR2_PIN           PE11
#define MOTOR3_PIN           PE13
#define MOTOR4_PIN           PE14
#define MOTOR5_PIN           PB10
#define MOTOR6_PIN           PB11
#define MOTOR7_PIN           PB0
#define MOTOR8_PIN           PB1

// *************** UART ******************
#define UART1_TX_PIN         PA9
#define UART2_TX_PIN         PD5
#define UART3_TX_PIN         PD8
#define UART4_TX_PIN         PB9
#define UART5_TX_PIN         PB13
#define UART6_TX_PIN         PC6
#define UART7_TX_PIN         PE8
#define UART8_TX_PIN         PE1
#define UART1_RX_PIN         PA10
#define UART2_RX_PIN         PD6
#define UART3_RX_PIN         PD9
#define UART4_RX_PIN         PB8
#define UART5_RX_PIN         PB12
#define UART6_RX_PIN         PC7
#define UART7_RX_PIN         PE7
#define UART8_RX_PIN         PE0

// *************** SPI ******************
#define SPI2_SCK_PIN         PB3
#define SPI2_SDI_PIN         PB14
#define SPI2_SDO_PIN         PC3
#define SPI3_SCK_PIN         PB3
#define SPI3_SDI_PIN         PB4
#define SPI3_SDO_PIN         PB2

// *************** SD ******************
#define SDIO_D0_PIN          PC8
#define SDIO_D1_PIN          PC9
#define SDIO_D2_PIN          PC10
#define SDIO_D3_PIN          PC11
#define SDIO_CK_PIN          PC12
#define SDIO_CMD_PIN         PD2
#define DEFAULT_BLACKBOX_DEVICE     BLACKBOX_DEVICE_SDCARD

#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, PE9 , 2,  0) \
    TIMER_PIN_MAP( 1, PE11 , 2,  0) \
    TIMER_PIN_MAP( 2, PE13 , 2,  0) \
    TIMER_PIN_MAP( 3, PE14 , 2,  0) \
    TIMER_PIN_MAP( 4, PB10 , 2,  0) \
    TIMER_PIN_MAP( 5, PB11 , 2,  0) \
    TIMER_PIN_MAP( 6, PB0, 2,  0) \
    TIMER_PIN_MAP( 7, PB1, 2,  0)

#define ADC1_DMA_OPT         8
#define ADC3_DMA_OPT         9
#define TIMUP1_DMA_OPT       0
#define TIMUP2_DMA_OPT       0
#define TIMUP3_DMA_OPT       2
#define TIMUP4_DMA_OPT       1
#define TIMUP5_DMA_OPT       0
#define TIMUP8_DMA_OPT       0

#define DEFAULT_RX_FEATURE   FEATURE_RX_SERIAL
#define SERIALRX_UART        SERIAL_PORT_UART5

// *************** Battery & Voltage *****************
#define ADC_VBAT_PIN         PC4
#define ADC_CURR_PIN         PC5
#define DEFAULT_CURRENT_METER_SOURCE    CURRENT_METER_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE    VOLTAGE_METER_ADC
#define DEFAULT_VOLTAGE_METER_SCALE     102
#define DEFAULT_CURRENT_METER_SCALE     250

// *************** Misc ********************
#define USB_DETECT_PIN       PA9

#define PINIO1_BOX           40
#define PINIO2_BOX           41
#define ENSURE_MPU_DATA_READY_IS_LOW

#define BEEPER_INVERTED
#define BEEPER_PWM_HZ        2500

#define SDCARD_DETECT_PIN    NONE
#define SDIO_DEVICE          SDIODEV_1
#define SDIO_USE_4BIT        1
#define FLASH_CS_PIN         PD4

#define PINIO1_PIN           PD10
#define PINIO2_PIN           PD11

#define SERVO1_PIN           PE5
#define SERVO2_PIN           PE6
#define RX_PPM_PIN           PC7
#define LED_STRIP_PIN        PA8
