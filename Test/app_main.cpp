#include "database.hpp"
#include "libxr.hpp"
#include "main.h"
#include "stm32_adc.hpp"
#include "stm32_flash.hpp"
#include "stm32_gpio.hpp"
#include "stm32_i2c.hpp"
#include "stm32_power.hpp"
#include "stm32_pwm.hpp"
#include "stm32_spi.hpp"
#include "stm32_timebase.hpp"
#include "stm32_uart.hpp"
#include "stm32_usb.hpp"

extern UART_HandleTypeDef huart1, huart3, huart6;
extern TIM_HandleTypeDef htim3, htim4, htim5, htim10;
extern SPI_HandleTypeDef hspi1;
extern I2C_HandleTypeDef hi2c3;
extern ADC_HandleTypeDef hadc1, hadc3;
extern CAN_HandleTypeDef hcan1, hcan2;

static uint8_t bmi088_spi_buff[2][24];
static uint8_t ist8310_i2c_buff[24];
static uint16_t adc1_buff[32][2];
static uint16_t adc3_buff[32][1];
static uint8_t uart_buff[3][2][256];

using namespace LibXR;

extern USBD_HandleTypeDef hUsbDeviceFS;  // NOLINT

extern void libxr_app_main();

extern "C" void app_main(void) {
  LibXR::STM32Timebase stm32_timebase;

  LibXR::PlatformInit();

  LibXR::STM32PowerManager power_manager;

  LibXR::STM32PWM pwm_b(&htim5, TIM_CHANNEL_1);
  LibXR::STM32PWM pwm_g(&htim5, TIM_CHANNEL_2);
  LibXR::STM32PWM pwm_r(&htim5, TIM_CHANNEL_3);
  LibXR::STM32PWM pwm_laser(&htim3, TIM_CHANNEL_3);
  LibXR::STM32PWM pwm_buzzer(&htim4, TIM_CHANNEL_3);
  LibXR::STM32PWM pwm_bmi088_heat(&htim10, TIM_CHANNEL_1);

  LibXR::STM32GPIO user_key(USER_KEY_GPIO_Port, USER_KEY_Pin,
                            USER_KEY_EXTI_IRQn);

  LibXR::STM32GPIO bmi088_gyro_cs(GYRO_CS_GPIO_Port, GYRO_CS_Pin);
  LibXR::STM32GPIO bmi088_accl_cs(ACCL_CS_GPIO_Port, ACCL_CS_Pin);

  LibXR::STM32GPIO bmi088_int_accl(ACCL_INT_GPIO_Port, ACCL_INT_Pin);
  LibXR::STM32GPIO bmi088_int_gyro(GYRO_INT_GPIO_Port, GYRO_INT_Pin);

  LibXR::STM32GPIO ist8310_rst(CMPS_RST_GPIO_Port, CMPS_RST_Pin);
  LibXR::STM32GPIO ist8310_int(CMPS_INT_GPIO_Port, CMPS_INT_Pin,
                               CMPS_INT_EXTI_IRQn);

  LibXR::STM32UART uart1(huart1, uart_buff[0][0], uart_buff[0][1]);
  LibXR::STM32UART uart3(huart3, uart_buff[1][0], uart_buff[1][1]);
  LibXR::STM32UART uart6(huart6, uart_buff[2][0], uart_buff[2][1]);

  LibXR::STM32VirtualUART uart_cdc(hUsbDeviceFS, 10, 10);
  STDIO::read_ = &uart_cdc.read_port_;
  STDIO::write_ = &uart_cdc.write_port_;
  RamFS ramfs("XRobot");
  Terminal terminal(ramfs);

  auto terminal_task = Timer::CreatetTask(terminal.TaskFun, &terminal, 10);
  Timer::Add(terminal_task);
  Timer::Start(terminal_task);

  pwm_b.SetConfig({
      .frequency = 1000,
  });
  pwm_b.Enable();

  void (*led_fun)(LibXR::STM32PWM* pwm) = [](LibXR::STM32PWM* pwm) {
    static bool flag = false;
    static uint32_t counter = 0;

    if (flag && counter == 100) {
      flag = false;
    }

    if (!flag && counter == 00) {
      flag = true;
    }

    if (flag) {
      counter++;
    } else {
      counter--;
    }

    pwm->SetDutyCycle(static_cast<float>(counter) / 100.0f);
  };

  auto led_task = Timer::CreatetTask(led_fun, &pwm_b, 5);
  Timer::Add(led_task);
  Timer::Start(led_task);

  LibXR::STM32I2C ist8310(&hi2c3, ist8310_i2c_buff, 3);

  LibXR::STM32SPI bmi088_spi(&hspi1, bmi088_spi_buff[0], bmi088_spi_buff[1], 3);

  // TODO: CAN1 & CAN2

  std::array<uint32_t, 2> adc1_channels = {ADC_CHANNEL_TEMPSENSOR,
                                           ADC_CHANNEL_VREFINT};

  std::array<uint32_t, 1> adc3_channels = {ADC_CHANNEL_8};

  LibXR::STM32ADC adc3(&hadc3, RawData(adc3_buff), adc3_channels, 3.3f);

  LibXR::STM32ADC adc1(&hadc1, RawData(adc1_buff), adc1_channels, 3.3f);

  LibXR::STM32Flash flash(0xC0000, 128 * 1024, 4);

  LibXR::DatabaseRaw<4> database(flash);

  pwm_buzzer.SetDutyCycle(0.02f);
  pwm_buzzer.SetConfig({.frequency = 262});
  pwm_buzzer.Enable();
  Thread::Sleep(200);
  pwm_buzzer.SetConfig({.frequency = 294});
  Thread::Sleep(200);
  pwm_buzzer.SetConfig({.frequency = 330});
  Thread::Sleep(200);
  pwm_buzzer.Disable();
  pwm_buzzer.SetDutyCycle(0);

  while (true) {
    Thread::Sleep(UINT32_MAX);
  }
}
