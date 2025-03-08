#include "libxr.hpp"
#include "main.h"
#include "stm32_timebase.hpp"
#include "stm32_uart.hpp"

extern UART_HandleTypeDef huart1;

using namespace LibXR;

static uint8_t uart_buff[2][128];

char term_buff = 0;

LibXR::STM32UART *uart;

void _app_main() {}

auto fun = [](void *) { HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8); };

extern "C" void app_main(void) {
  LibXR::STM32Timebase stm32_timebase;

  LibXR::PlatformInit();

  static LibXR::STM32UART uart1(huart1, RawData(uart_buff[0], 128),
                                RawData(uart_buff[1], 128), 10, 10);
  uart = &uart1;
  STDIO::read_ = &uart->read_port_;
  STDIO::write_ = &uart->write_port_;
  RamFS ramfs("XRobot");
  Terminal terminal(ramfs);
  auto led_task = Timer::CreatetTask<void *>(fun, (void *)(0), 500);
  Timer::Add(led_task);
  Timer::Start(led_task);
  auto term_task = Timer::CreatetTask(terminal.TaskFun, &terminal, 10);
  Timer::Add(term_task);
  Timer::Start(term_task);

  while (true) {
    Thread::Sleep(100);
    // uart1.read_port_(RawData(term_buff), ReadOperation(10));
  }
}
