#ifndef TIMER_H_
#define TIMER_H_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>

namespace timer {

class Timer {
 public:
  Timer() : is_running_(false) {}

  ~Timer() { Stop(); }

  void Start(int interval_ms, std::function<void()> task) {
    Stop();  // 기존 타이머 중지
    is_running_ = true;
    thread_ = std::thread([this, interval_ms, task]() {
      std::unique_lock<std::mutex> local_lock(mutex_);
      while (is_running_) {
        if (cv_.wait_for(local_lock, std::chrono::milliseconds(interval_ms),
                         [this]() { return !is_running_; })) {
          break;  // running_이 false가 되면 종료
        }
        task();
      }
    });
  }

  void Stop() {
    is_running_ = false;
    cv_.notify_all();
    if (thread_.joinable()) thread_.join();
  }

 private:
  std::atomic<bool> is_running_;
  std::thread thread_;
  std::mutex mutex_;
  std::condition_variable cv_;
};
}  // namespace timer

#endif  // TIMER_H_
