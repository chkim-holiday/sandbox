#include <iomanip>
#include <iostream>

#include "timer_manager.h"

#include <pigpio.h>

#define GPIO_PIN 16

void PrintTime(const std::string& msg) {
  auto now = std::chrono::system_clock::now();
  auto time_t = std::chrono::system_clock::to_time_t(now);
  auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                now.time_since_epoch()) %
            1000000000;

  std::cout << "[" << std::put_time(std::localtime(&time_t), "%H:%M:%S") << "."
            << std::setfill('0') << std::setw(9) << ns.count() << "] " << msg
            << std::endl;
}

int main() {
  TimerManager timer_manager;

  // GPIO 초기화
  if (gpioInitialise() < 0) {
    std::cerr << "Failed to initialize GPIO." << std::endl;
    return -1;
  }

  // GPIO 핀을 출력으로 설정
  gpioSetMode(GPIO_PIN, PI_OUTPUT);
  gpioWrite(GPIO_PIN, 0);

  auto program_start_time = std::chrono::system_clock::now();

  auto future_time_1 = program_start_time + std::chrono::seconds(2);
  auto future_time_2 = program_start_time + std::chrono::seconds(4);
  auto future_time_3 = program_start_time + std::chrono::seconds(6);
  auto future_time_4 = program_start_time + std::chrono::seconds(8);
  ScheduledExecutor scheduler1(timer_manager);
  scheduler1.ScheduleAt(future_time_1, [&]() {
    gpioWrite(GPIO_PIN, 1);
    gpioWrite(GPIO_PIN, 0);
  });
  ScheduledExecutor scheduler2(timer_manager);
  scheduler2.ScheduleAt(future_time_2, [&]() {
    gpioWrite(GPIO_PIN, 1);
    gpioWrite(GPIO_PIN, 0);
  });
  ScheduledExecutor scheduler3(timer_manager);
  scheduler3.ScheduleAt(future_time_3, [&]() {
    gpioWrite(GPIO_PIN, 1);
    gpioWrite(GPIO_PIN, 0);
  });
  ScheduledExecutor scheduler4(timer_manager);
  scheduler4.ScheduleAt(future_time_4, [&]() {
    gpioWrite(GPIO_PIN, 1);
    gpioWrite(GPIO_PIN, 0);
  });

  // 프로그램 실행
  std::cout << "\nActive timers: " << timer_manager.ActiveTimerCount()
            << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(10));
  PrintTime("Program ending");

  gpioWrite(GPIO_PIN, 0);
  gpioTerminate();

  return 0;
}