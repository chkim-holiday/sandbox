#include <iomanip>
#include <iostream>

#include "timer_manager.h"

// 현재 시간 출력 헬퍼
void PrintTime(const std::string& msg) {
  auto now = std::chrono::system_clock::now();
  auto time_t = std::chrono::system_clock::to_time_t(now);
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                now.time_since_epoch()) %
            1000;

  std::cout << "[" << std::put_time(std::localtime(&time_t), "%H:%M:%S") << "."
            << std::setfill('0') << std::setw(3) << ms.count() << "] " << msg
            << std::endl;
}

int main() {
  TimerManager timer_system;

  std::cout << "=== Timer, Timeout, ScheduledExecutor Example ===" << std::endl;
  PrintTime("Program started");

  // 1. Timer: 주기적 실행
  Timer periodic_timer(timer_system);
  periodic_timer.start(std::chrono::milliseconds(500), []() {
    PrintTime("Timer: Periodic tick (every 500ms)");
  });

  // 2. Timeout: 상대 시간 일회성
  Timeout timeout(timer_system);

  timeout.Schedule(std::chrono::milliseconds(1000),
                   []() { PrintTime("Timeout: Fired after 1 second"); });

  timeout.Schedule(std::chrono::milliseconds(2500),
                   []() { PrintTime("Timeout: Fired after 2.5 seconds"); });

  // 3. ScheduledExecutor: 절대 시간 일회성
  ScheduledExecutor scheduler(timer_system);

  auto future_time_1 =
      std::chrono::system_clock::now() + std::chrono::milliseconds(1500);
  scheduler.ScheduleAt(future_time_1, []() {
    PrintTime("ScheduledExecutor: Executed at absolute time (1.5s)");
  });

  auto future_time_2 =
      std::chrono::system_clock::now() + std::chrono::seconds(3);
  scheduler.ScheduleAt(future_time_2, []() {
    PrintTime("ScheduledExecutor: Executed at absolute time (3s)");
  });

  // 4. 취소 가능한 타이머
  auto cancelable_id = timeout.Schedule(std::chrono::seconds(10), []() {
    PrintTime("This should be cancelled!");
  });

  // 2초 후 취소
  std::this_thread::sleep_for(std::chrono::seconds(2));
  if (timer_system.CancelTimer(cancelable_id)) {
    PrintTime("Successfully cancelled timer");
  }

  // 5. 고정밀 타이머 (나노초)
  timer_system.CreateTimeout(std::chrono::nanoseconds(1500000000), []() {
    PrintTime("High precision timeout: 1.5 seconds (in nanoseconds)");
  });

  // 프로그램 실행
  std::cout << "\nActive timers: " << timer_system.ActiveTimerCount()
            << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(4));

  PrintTime("Stopping periodic timer");
  periodic_timer.stop();

  std::this_thread::sleep_for(std::chrono::seconds(1));
  PrintTime("Program ending");

  return 0;
}