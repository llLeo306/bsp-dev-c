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
  PlatformInit(2, 512);
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
              usart1_rx_buf, usart1_tx_buf, 5, 5);

  STM32UART usart3(&huart3,
              usart3_rx_buf, {nullptr, 0}, 5, 5);

  STM32UART usart6(&huart6,
              usart6_rx_buf, usart6_tx_buf, 5, 5);

  STM32I2C i2c1(&hi2c1, i2c1_buf, 3);

  STM32I2C i2c2(&hi2c2, i2c2_buf, 3);

  STM32I2C i2c3(&hi2c3, i2c3_buf, 3);

  STM32CAN can1(&hcan1, "can1", 5);

  STM32CAN can2(&hcan2, "can2", 5);

  STM32VirtualUART uart_cdc(hUsbDeviceFS, UserTxBufferFS, UserRxBufferFS, 12, 12);
  STDIO::read_ = &uart_cdc.read_port_;
  STDIO::write_ = &uart_cdc.write_port_;
  RamFS ramfs("XRobot");
  Terminal<32, 32, 5, 5> terminal(ramfs);
  auto terminal_task = Timer::CreateTask(terminal.TaskFun, &terminal, 10);
  Timer::Add(terminal_task);
  Timer::Start(terminal_task);


  LibXR::HardwareContainer<
    LibXR::Entry<LibXR::PowerManager>,
    LibXR::Entry<LibXR::GPIO>,
    LibXR::Entry<LibXR::GPIO>,
    LibXR::Entry<LibXR::GPIO>,
    LibXR::Entry<LibXR::GPIO>,
    LibXR::Entry<LibXR::GPIO>,
    LibXR::Entry<LibXR::GPIO>,
    LibXR::Entry<LibXR::GPIO>,
    LibXR::Entry<LibXR::GPIO>,
    LibXR::Entry<LibXR::SPI>,
    LibXR::Entry<LibXR::PWM>,
    LibXR::Entry<LibXR::GPIO>,
    LibXR::Entry<LibXR::GPIO>,
    LibXR::Entry<LibXR::GPIO>,
    LibXR::Entry<LibXR::GPIO>,
    LibXR::Entry<LibXR::GPIO>,
    LibXR::Entry<LibXR::PWM>,
    LibXR::Entry<LibXR::PWM>,
    LibXR::Entry<LibXR::PWM>,
    LibXR::Entry<LibXR::PWM>,
    LibXR::Entry<LibXR::PWM>,
    LibXR::Entry<LibXR::PWM>,
    LibXR::Entry<LibXR::PWM>,
    LibXR::Entry<LibXR::PWM>,
    LibXR::Entry<LibXR::PWM>,
    LibXR::Entry<LibXR::ADC>,
    LibXR::Entry<LibXR::UART>,
    LibXR::Entry<LibXR::UART>,
    LibXR::Entry<LibXR::UART>,
    LibXR::Entry<LibXR::I2C>,
    LibXR::Entry<LibXR::I2C>,
    LibXR::Entry<LibXR::I2C>,
    LibXR::Entry<LibXR::CAN>,
    LibXR::Entry<LibXR::CAN>,
    LibXR::Entry<LibXR::UART>,
    LibXR::Entry<LibXR::RamFS>,
    LibXR::Entry<LibXR::Terminal<32, 32, 5, 5>>
  > peripherals{
    {power_manager, {"power_manager"}},
    {USER_KEY, {"USER_KEY", "wakeup_key"}},
    {ACCL_CS, {"bmi088_accl_cs"}},
    {GYRO_CS, {"bmi088_gyro_cs"}},
    {HW0, {"HW0"}},
    {HW1, {"HW1"}},
    {HW2, {"HW2"}},
    {ACCL_INT, {"bmi088_accl_int"}},
    {GYRO_INT, {"bmi088_gyro_int"}},
    {spi1, {"spi_bmi088"}},
    {pwm_tim10_ch1, {"pwm_bmi088_heat"}},
    {CMPS_INT, {"ist8310_int"}},
    {CMPS_RST, {"ist8310_rst"}},
    {LED_B, {"LED", "LED_B"}},
    {LED_G, {"LED_G"}},
    {LED_R, {"LED_R"}},
    {pwm_tim1_ch1, {"pwm_a", "pwm_launcher_cover_servo"}},
    {pwm_tim1_ch2, {"pwm_b"}},
    {pwm_tim1_ch3, {"pwm_c"}},
    {pwm_tim1_ch4, {"pwm_d"}},
    {pwm_tim3_ch3, {"pwm_5v"}},
    {pwm_tim4_ch3, {"pwm_buzzer"}},
    {pwm_tim8_ch1, {"pwm_e"}},
    {pwm_tim8_ch2, {"pwm_f"}},
    {pwm_tim8_ch3, {"pwm_g"}},
    {adc3_adc_channel_8, {"adc_bat"}},
    {usart1, {"uart_referee"}},
    {usart3, {"uart_dr16"}},
    {usart6, {"uart_ai", "uart_ext_controller"}},
    {i2c1, {"i2c1"}},
    {i2c2, {"i2c2"}},
    {i2c3, {"i2c_ist8310"}},
    {can1, {"can1"}},
    {can2, {"can2"}},
    {uart_cdc, {"uart_cdc"}},
    {ramfs, {"ramfs"}},
    {terminal, {"terminal"}}
  };

  /* User Code Begin 3 */
  STM32Flash<FLASH_SECTOR_NUMBER, FLASH_SECTOR_NUMBER - 1> flash(FLASH_SECTORS);
  LibXR::DatabaseRaw<1> database(flash);

  peripherals.Register(LibXR::Entry<LibXR::Database>{database, {"database"}});

  XRobotMain(peripherals);
  /* User Code End 3 */
}