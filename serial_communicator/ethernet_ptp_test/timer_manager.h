#pragma once

#include <sys/epoll.h>
#include <sys/timerfd.h>
#include <unistd.h>
#include <atomic>
#include <chrono>
#include <functional>
#include <iostream>
#include <map>
#include <mutex>
#include <stdexcept>
#include <thread>

using TimerId = int;
using TimePoint = std::chrono::system_clock::time_point;

struct Timestamp {
  uint32_t sec{0};
  uint32_t nsec{0};
};

class TimerManager {
 public:
  using Callback = std::function<void()>;

  TimerManager() {
    // `Epoll` governs all timers
    epoll_fd_ = epoll_create1(0);
    if (epoll_fd_ < 0) throw std::runtime_error("epoll_create1 failed");
    is_thread_running_ = true;
    event_loop_thread_ = std::thread(&TimerManager::RunEventLoop, this);
  }

  ~TimerManager() {
    is_thread_running_ = false;
    if (event_loop_thread_.joinable()) event_loop_thread_.join();

    std::lock_guard<std::mutex> lock(mutex_);
    for (auto& [timer_id, timer_info] : timers_) {
      epoll_ctl(epoll_fd_, EPOLL_CTL_DEL, timer_id, nullptr);
      close(timer_id);
    }
    close(epoll_fd_);
  }

  TimerId CreateTimer(std::chrono::nanoseconds interval, Callback callback) {
    return CreateTimerInternal(interval, callback, true, false, 0);
  }

  TimerId CreateTimeout(std::chrono::nanoseconds delay, Callback callback) {
    return CreateTimerInternal(delay, callback, false, false, 0);
  }

  TimerId ScheduleAt(TimePoint absolute_time, Callback callback) {
    auto duration = absolute_time.time_since_epoch();
    auto nsecs = std::chrono::duration_cast<std::chrono::nanoseconds>(duration);
    return CreateTimerInternal(nsecs, callback, false, true, 0);
  }

  TimerId ScheduleAtPTP(int64_t ptp_nsec, Callback callback) {
    return CreateTimerInternal(std::chrono::nanoseconds(ptp_nsec), callback,
                               false, true, 0);
  }

  // 타이머 취소
  bool CancelTimer(TimerId timer_id) {
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = timers_.find(timer_id);
    if (it == timers_.end()) {
      return false;
    }

    epoll_ctl(epoll_fd_, EPOLL_CTL_DEL, timer_id, nullptr);
    close(timer_id);
    timers_.erase(it);

    return true;
  }

  // 활성 타이머 개수
  size_t ActiveTimerCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return timers_.size();
  }

 private:
  struct TimerInfo {
    Callback callback;
    bool is_periodic;
    bool is_absolute;
  };

  TimerId CreateTimerInternal(std::chrono::nanoseconds time_spec,
                              Callback callback, bool is_periodic,
                              bool is_absolute, int flags) {
    // CLOCK_REALTIME: 절대 시간 (PTP/NTP 동기화 가능)
    // CLOCK_MONOTONIC: 상대 시간 (시스템 시간 변경 영향 없음)
    int clock_type = is_absolute ? CLOCK_REALTIME : CLOCK_MONOTONIC;

    int fd = timerfd_create(clock_type, TFD_NONBLOCK | TFD_CLOEXEC);
    if (fd < 0) {
      throw std::runtime_error("timerfd_create failed");
    }

    struct itimerspec spec;
    auto secs = std::chrono::duration_cast<std::chrono::seconds>(time_spec);
    auto nsecs = time_spec - secs;

    spec.it_value.tv_sec = secs.count();
    spec.it_value.tv_nsec = nsecs.count();

    if (is_periodic) {
      // 주기적 반복
      spec.it_interval.tv_sec = secs.count();
      spec.it_interval.tv_nsec = nsecs.count();
    } else {
      // 일회성
      spec.it_interval.tv_sec = 0;
      spec.it_interval.tv_nsec = 0;
    }

    int timer_flags = is_absolute ? TFD_TIMER_ABSTIME : 0;
    if (timerfd_settime(fd, timer_flags, &spec, nullptr) < 0) {
      close(fd);
      throw std::runtime_error("timerfd_settime failed");
    }

    // epoll에 등록
    struct epoll_event ev;
    ev.events = EPOLLIN | EPOLLET;  // Edge-triggered
    ev.data.fd = fd;

    if (epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, fd, &ev) < 0) {
      close(fd);
      throw std::runtime_error("epoll_ctl failed");
    }

    // 타이머 정보 저장
    {
      std::lock_guard<std::mutex> lock(mutex_);
      timers_[fd] = {callback, is_periodic, is_absolute};
    }

    return fd;
  }

  void RunEventLoop() {
    const int MAX_EVENTS = 64;
    struct epoll_event events[MAX_EVENTS];

    while (is_thread_running_) {
      // 100ms timeout으로 주기적으로 running_ 체크
      int n = epoll_wait(epoll_fd_, events, MAX_EVENTS, 100);

      if (n < 0) {
        if (errno == EINTR) continue;
        std::cerr << "epoll_wait error: "
                  << std::error_code(errno, std::generic_category()).message()
                  << std::endl;
        break;
      }

      for (int i = 0; i < n; i++) {
        int fd = events[i].data.fd;

        // 타이머 만료 횟수 읽기
        uint64_t expirations;
        ssize_t s = read(fd, &expirations, sizeof(expirations));

        if (s != sizeof(expirations)) continue;

        // 콜백 실행
        TimerInfo info;
        bool found = false;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          auto it = timers_.find(fd);
          if (it != timers_.end()) {
            info = it->second;
            found = true;

            // 일회성 타이머는 제거
            if (!info.is_periodic) {
              epoll_ctl(epoll_fd_, EPOLL_CTL_DEL, fd, nullptr);
              close(fd);
              timers_.erase(it);
            }
          }
        }

        if (found && info.callback) {
          try {
            info.callback();
          } catch (const std::exception& e) {
            std::cerr << "Timer callback exception: " << e.what() << std::endl;
          }
        }
      }
    }
  }

  int epoll_fd_;
  std::atomic<bool> is_thread_running_;
  std::thread event_loop_thread_;

  mutable std::mutex mutex_;
  std::map<TimerId, TimerInfo> timers_;
};

// 편의를 위한 래퍼 클래스들
class Timer {
 public:
  Timer(TimerManager& system) : manager_(system), timer_id_(-1) {}

  ~Timer() { stop(); }

  void start(std::chrono::milliseconds interval,
             TimerManager::Callback callback) {
    stop();
    timer_id_ = manager_.CreateTimer(interval, callback);
  }

  void stop() {
    if (timer_id_ >= 0) {
      manager_.CancelTimer(timer_id_);
      timer_id_ = -1;
    }
  }

  bool isActive() const { return timer_id_ >= 0; }

 private:
  TimerManager& manager_;
  TimerId timer_id_;
};

class Timeout {
 public:
  Timeout(TimerManager& manager) : manager_(manager) {}

  TimerId Schedule(std::chrono::milliseconds delay,
                   TimerManager::Callback callback) {
    return manager_.CreateTimeout(delay, callback);
  }

 private:
  TimerManager& manager_;
};

class ScheduledExecutor {
 public:
  ScheduledExecutor(TimerManager& manager) : manager_(manager) {}

  TimerId ScheduleAt(TimePoint absolute_time, TimerManager::Callback callback) {
    return manager_.ScheduleAt(absolute_time, callback);
  }

  // PTP 시간 기준 스케줄링
  TimerId ScheduleAtPTP(int64_t ptp_nsec, TimerManager::Callback callback) {
    return manager_.ScheduleAtPTP(ptp_nsec, callback);
  }

 private:
  TimerManager& manager_;
};