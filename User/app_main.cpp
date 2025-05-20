#include "app_main.h"

#include "libxr.hpp"
#include "main.h"
#include "stm32_adc.hpp"
#include "stm32_can.hpp"
#include "stm32_canfd.hpp"
#include "stm32_gpio.hpp"
#include "stm32_i2c.hpp"
#include "stm32_power.hpp"
#include "stm32_pwm.hpp"
#include "stm32_spi.hpp"
#include "stm32_timebase.hpp"
#include "stm32_uart.hpp"
#include "stm32_usb.hpp"
#include "flash_map.hpp"
#include "app_framework.hpp"
#include "xrobot_main.hpp"


using namespace LibXR;

/* User Code Begin 1 */
#include "stm32_flash.hpp"
/* User Code End 1 */
/* External HAL Declarations */
extern ADC_HandleTypeDef hadc3;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c3;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim8;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
extern USBD_HandleTypeDef hUsbDeviceFS;
extern uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
extern uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/* DMA Resources */
static uint16_t adc3_buf[64];
static uint8_t spi1_tx_buf[32];
static uint8_t spi1_rx_buf[32];
static uint8_t usart1_tx_buf[128];
static uint8_t usart1_rx_buf[128];
static uint8_t usart3_rx_buf[128];
static uint8_t usart6_tx_buf[128];
static uint8_t usart6_rx_buf[128];
static uint8_t i2c1_buf[32];
static uint8_t i2c2_buf[32];
static uint8_t i2c3_buf[32];

extern "C" void app_main(void) {
  /* User Code Begin 2 */
  
  /* User Code End 2 */
  STM32TimerTimebase timebase(&htim2);
  PlatformInit(2, 1024);
  STM32PowerManager power_manager;

  /* GPIO Configuration */
  STM32GPIO USER_KEY(USER_KEY_GPIO_Port, USER_KEY_Pin, EXTI0_IRQn);
  STM32GPIO ACCL_CS(ACCL_CS_GPIO_Port, ACCL_CS_Pin);
  STM32GPIO GYRO_CS(GYRO_CS_GPIO_Port, GYRO_CS_Pin);
  STM32GPIO HW0(HW0_GPIO_Port, HW0_Pin);
  STM32GPIO HW1(HW1_GPIO_Port, HW1_Pin);
  STM32GPIO HW2(HW2_GPIO_Port, HW2_Pin);
  STM32GPIO ACCL_INT(ACCL_INT_GPIO_Port, ACCL_INT_Pin, EXTI4_IRQn);
  STM32GPIO GYRO_INT(GYRO_INT_GPIO_Port, GYRO_INT_Pin, EXTI9_5_IRQn);
  STM32GPIO CMPS_INT(CMPS_INT_GPIO_Port, CMPS_INT_Pin, EXTI3_IRQn);
  STM32GPIO CMPS_RST(CMPS_RST_GPIO_Port, CMPS_RST_Pin);
  STM32GPIO LED_B(LED_B_GPIO_Port, LED_B_Pin);
  STM32GPIO LED_G(LED_G_GPIO_Port, LED_G_Pin);
  STM32GPIO LED_R(LED_R_GPIO_Port, LED_R_Pin);

  std::array<uint32_t, 1> adc3_channels = {ADC_CHANNEL_8};
  STM32ADC adc3(&hadc3, adc3_buf, adc3_channels, 3.3);
  auto adc3_adc_channel_8 = adc3.GetChannel(0);
  UNUSED(adc3_adc_channel_8);

  STM32PWM pwm_tim1_ch1(&htim1, TIM_CHANNEL_1);
  STM32PWM pwm_tim1_ch2(&htim1, TIM_CHANNEL_2);
  STM32PWM pwm_tim1_ch3(&htim1, TIM_CHANNEL_3);
  STM32PWM pwm_tim1_ch4(&htim1, TIM_CHANNEL_4);

  STM32PWM pwm_tim10_ch1(&htim10, TIM_CHANNEL_1);

  STM32PWM pwm_tim3_ch3(&htim3, TIM_CHANNEL_3);

  STM32PWM pwm_tim4_ch3(&htim4, TIM_CHANNEL_3);

  STM32PWM pwm_tim8_ch1(&htim8, TIM_CHANNEL_1);
  STM32PWM pwm_tim8_ch2(&htim8, TIM_CHANNEL_2);
  STM32PWM pwm_tim8_ch3(&htim8, TIM_CHANNEL_3);

  STM32SPI spi1(&hspi1, spi1_rx_buf, spi1_tx_buf, 3);

  STM32UART usart1(&huart1,
              usart1_rx_buf, usart1_tx_buf, 5);

  STM32UART usart3(&huart3,
              usart3_rx_buf, {nullptr, 0}, 5);

  STM32UART usart6(&huart6,
              usart6_rx_buf, usart6_tx_buf, 5);

  STM32I2C i2c1(&hi2c1, i2c1_buf, 3);

  STM32I2C i2c2(&hi2c2, i2c2_buf, 3);

  STM32I2C i2c3(&hi2c3, i2c3_buf, 3);

  STM32CAN can1(&hcan1, 5);

  STM32CAN can2(&hcan2, 5);

  STM32VirtualUART uart_cdc(hUsbDeviceFS, UserTxBufferFS, UserRxBufferFS, 15);
  STDIO::read_ = uart_cdc.read_port_;
  STDIO::write_ = uart_cdc.write_port_;
  RamFS ramfs("XRobot");
  Terminal<32, 32, 5, 5> terminal(ramfs);
  LibXR::Thread term_thread;
  term_thread.Create(&terminal, terminal.ThreadFun, "terminal", 4096,
                     static_cast<LibXR::Thread::Priority>(3));


  LibXR::HardwareContainer peripherals{
    LibXR::Entry<LibXR::PowerManager>({power_manager, {"power_manager"}}),
    LibXR::Entry<LibXR::GPIO>({USER_KEY, {"USER_KEY", "wakeup_key"}}),
    LibXR::Entry<LibXR::GPIO>({ACCL_CS, {"bmi088_accl_cs"}}),
    LibXR::Entry<LibXR::GPIO>({GYRO_CS, {"bmi088_gyro_cs"}}),
    LibXR::Entry<LibXR::GPIO>({HW0, {"HW0"}}),
    LibXR::Entry<LibXR::GPIO>({HW1, {"HW1"}}),
    LibXR::Entry<LibXR::GPIO>({HW2, {"HW2"}}),
    LibXR::Entry<LibXR::GPIO>({ACCL_INT, {"bmi088_accl_int"}}),
    LibXR::Entry<LibXR::GPIO>({GYRO_INT, {"bmi088_gyro_int"}}),
    LibXR::Entry<LibXR::SPI>({spi1, {"spi_bmi088"}}),
    LibXR::Entry<LibXR::PWM>({pwm_tim10_ch1, {"pwm_bmi088_heat"}}),
    LibXR::Entry<LibXR::GPIO>({CMPS_INT, {"ist8310_int"}}),
    LibXR::Entry<LibXR::GPIO>({CMPS_RST, {"ist8310_rst"}}),
    LibXR::Entry<LibXR::GPIO>({LED_B, {"LED", "LED_B"}}),
    LibXR::Entry<LibXR::GPIO>({LED_G, {"LED_G"}}),
    LibXR::Entry<LibXR::GPIO>({LED_R, {"LED_R"}}),
    LibXR::Entry<LibXR::PWM>({pwm_tim1_ch1, {"pwm_a", "pwm_launcher_cover_servo"}}),
    LibXR::Entry<LibXR::PWM>({pwm_tim1_ch2, {"pwm_b"}}),
    LibXR::Entry<LibXR::PWM>({pwm_tim1_ch3, {"pwm_c"}}),
    LibXR::Entry<LibXR::PWM>({pwm_tim1_ch4, {"pwm_d"}}),
    LibXR::Entry<LibXR::PWM>({pwm_tim3_ch3, {"pwm_5v"}}),
    LibXR::Entry<LibXR::PWM>({pwm_tim4_ch3, {"pwm_buzzer"}}),
    LibXR::Entry<LibXR::PWM>({pwm_tim8_ch1, {"pwm_e"}}),
    LibXR::Entry<LibXR::PWM>({pwm_tim8_ch2, {"pwm_f"}}),
    LibXR::Entry<LibXR::PWM>({pwm_tim8_ch3, {"pwm_g"}}),
    LibXR::Entry<LibXR::ADC>({adc3_adc_channel_8, {"adc_bat"}}),
    LibXR::Entry<LibXR::UART>({usart1, {"imu_data_uart", "uart_referee"}}),
    LibXR::Entry<LibXR::UART>({usart3, {"uart_dr16"}}),
    LibXR::Entry<LibXR::UART>({usart6, {"uart_ai", "uart_ext_controller"}}),
    LibXR::Entry<LibXR::I2C>({i2c1, {"i2c1"}}),
    LibXR::Entry<LibXR::I2C>({i2c2, {"i2c2"}}),
    LibXR::Entry<LibXR::I2C>({i2c3, {"i2c_ist8310"}}),
    LibXR::Entry<LibXR::CAN>({can1, {"can1", "imu_can"}}),
    LibXR::Entry<LibXR::CAN>({can2, {"can2"}}),
    LibXR::Entry<LibXR::UART>({uart_cdc, {"uart_cdc"}}),
    LibXR::Entry<LibXR::RamFS>({ramfs, {"ramfs"}}),
    LibXR::Entry<LibXR::Terminal<32, 32, 5, 5>>({terminal, {"terminal"}})
  };

  /* User Code Begin 3 */
  STM32Flash<FLASH_SECTOR_NUMBER, FLASH_SECTOR_NUMBER - 1> flash(FLASH_SECTORS);
  LibXR::DatabaseRaw<1> database(flash);

  peripherals.Register(LibXR::Entry<LibXR::Database>{database, {"database"}});
  XRobotMain(peripherals);
  /* User Code End 3 */
}