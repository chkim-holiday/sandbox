#include <iomanip>
#include <iostream>

#include "timer_manager.h"

#include <pigpio.h>

#define GPIO_PIN 16

// 현재 시간 출력 헬퍼
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
  // GPIO 초기화
  if (gpioInitialise() < 0) {
    std::cerr << "Failed to initialize GPIO." << std::endl;
    return -1;
  }

  // GPIO 핀을 출력으로 설정
  gpioSetMode(GPIO_PIN, PI_OUTPUT);

  auto program_start_time = std::chrono::system_clock::now();

  auto future_time_1 = program_start_time + std::chrono::seconds(2);
  auto future_time_2 = program_start_time + std::chrono::seconds(4);
  auto future_time_3 = program_start_time + std::chrono::seconds(6);
  auto future_time_4 = program_start_time + std::chrono::seconds(8);
  ScheduledExecutor scheduler1(timer_system);
  scheduler1.ScheduleAt(future_time_1, []() {
    gpioWrite(GPIO_PIN, true);
    PrintTime("ScheduledExecutor: Execute at absolute time 1.");
    gpioWrite(GPIO_PIN, false);
  });
  ScheduledExecutor scheduler2(timer_system);
  scheduler2.ScheduleAt(future_time_2, []() {
    gpioWrite(GPIO_PIN, true);
    PrintTime("ScheduledExecutor: Execute at absolute time 2.");
    gpioWrite(GPIO_PIN, false);
  });
  ScheduledExecutor scheduler3(timer_system);
  scheduler3.ScheduleAt(future_time_3, []() {
    gpioWrite(GPIO_PIN, true);
    PrintTime("ScheduledExecutor: Execute at absolute time 3.");
    gpioWrite(GPIO_PIN, false);
  });
  ScheduledExecutor scheduler4(timer_system);
  scheduler4.ScheduleAt(future_time_4, []() {
    gpioWrite(GPIO_PIN, true);
    PrintTime("ScheduledExecutor: Execute at absolute time 4.");
    gpioWrite(GPIO_PIN, false);
  });

  // 프로그램 실행
  std::cout << "\nActive timers: " << timer_system.ActiveTimerCount()
            << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(10));
  PrintTime("Program ending");

  gpioWrite(GPIO_PIN, false);
  gpioTerminate();

  return 0;
}