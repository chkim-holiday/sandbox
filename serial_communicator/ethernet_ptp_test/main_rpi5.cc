#include <iomanip>
#include <iostream>

#include "timer_manager.h"

#include <lgpio.h>

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

  int h = lgGpiochipOpen(4);
  if (h < 0) return -1;
  lgGpioClaimOutput(h, 0, GPIO_PIN, 0);
  lgGpioWrite(h, GPIO_PIN, 0);

  auto program_start_time = std::chrono::system_clock::now();
  auto future_time = program_start_time + std::chrono::seconds(3);
  auto future_time_t = std::chrono::system_clock::to_time_t(future_time);
  auto future_reference = std::chrono::system_clock::from_time_t(future_time_t);
  auto future_time_1 = future_reference + std::chrono::milliseconds(100);
  auto future_time_2 = future_reference + std::chrono::milliseconds(200);
  auto future_time_3 = future_reference + std::chrono::milliseconds(300);
  auto future_time_4 = future_reference + std::chrono::milliseconds(400);

  ScheduledExecutor scheduler1(timer_manager);
  scheduler1.ScheduleAt(future_time_1, [&]() {
    lgGpioWrite(h, GPIO_PIN, 1);
    lgGpioWrite(h, GPIO_PIN, 0);
    // PrintTime("ScheduledExecutor: Execute at absolute time 1.");
  });
  ScheduledExecutor scheduler2(timer_manager);
  scheduler2.ScheduleAt(future_time_2, [&]() {
    lgGpioWrite(h, GPIO_PIN, 1);
    lgGpioWrite(h, GPIO_PIN, 0);
    // PrintTime("ScheduledExecutor: Execute at absolute time 2.");
  });
  ScheduledExecutor scheduler3(timer_manager);
  scheduler3.ScheduleAt(future_time_3, [&]() {
    lgGpioWrite(h, GPIO_PIN, 1);
    lgGpioWrite(h, GPIO_PIN, 0);
    // PrintTime("ScheduledExecutor: Execute at absolute time 3.");
  });
  ScheduledExecutor scheduler4(timer_manager);
  scheduler4.ScheduleAt(future_time_4, [&]() {
    lgGpioWrite(h, GPIO_PIN, 1);
    lgGpioWrite(h, GPIO_PIN, 0);
    // PrintTime("ScheduledExecutor: Execute at absolute time 4.");
  });

  // 프로그램 실행
  std::cout << "\nActive timers: " << timer_manager.ActiveTimerCount()
            << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(10));
  PrintTime("Program ending");

  lgGpioWrite(h, GPIO_PIN, 0);
  lgGpiochipClose(h);

  return 0;
}