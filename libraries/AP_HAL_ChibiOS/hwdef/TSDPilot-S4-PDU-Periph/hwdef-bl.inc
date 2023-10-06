# hw definition for Sierra-F405 node

# Sierra-M9N Bootloader

# MCU class and specific type
MCU STM32F4xx STM32F405xx

# crystal frequency
OSCILLATOR_HZ 8000000

# hw definition file for processing by chibios_pins.py

FLASH_RESERVE_START_KB 0
FLASH_SIZE_KB 1024
FLASH_BOOTLOADER_LOAD_KB 64

# board ID for firmware load
APJ_BOARD_ID 10201

# setup build for a peripheral firmware
env AP_PERIPH 1

# Flash available
FLASH_SIZE_KB 1024

# order of UARTs
SERIAL_ORDER OTG1

PA11 OTG_FS_DM OTG1
PA12 OTG_FS_DP OTG1

# USART4
PC10 UART4_TX UART4
PC11 UART4_RX UART4

PB13 LED_BOOTLOADER OUTPUT LOW GPIO(0)  # blue
PB14 LED_ACTIVITY OUTPUT LOW GPIO(1)  # Green
define HAL_LED_ON 0

# SWD debugging
PA13 JTMS-SWDIO SWD
PA14 JTCK-SWCLK SWD

# enable CAN support
PB8 CAN1_RX CAN1
PB9 CAN1_TX CAN1

define HAL_USE_CAN TRUE
define STM32_CAN_USE_CAN1 TRUE

# use DNA
define HAL_CAN_DEFAULT_NODE_ID 0

STDOUT_SERIAL SD4
STDOUT_BAUDRATE 57600

define HAL_USE_SERIAL TRUE
define STM32_SERIAL_USE_USART4 TRUE

# make bl baudrate match debug baudrate for easier debugging
define BOOTLOADER_BAUDRATE 57600

# use a small bootloader timeout
define HAL_BOOTLOADER_TIMEOUT 1000