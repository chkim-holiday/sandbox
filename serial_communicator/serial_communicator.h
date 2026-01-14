#ifndef SERIAL_COMMUNICATOR_H_
#define SERIAL_COMMUNICATOR_H_

#include <atomic>
#include <cstdint>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>

namespace serial_communicator {

struct SerialPort {
  std::string port_name;
  uint32_t baud_rate;
  int fd{-1};
};

class SerialCommunicator {
 public:
  using DataCallback = std::function<void(const uint8_t* data, size_t len)>;
  using ErrorCallback = std::function<void(const std::string& error)>;

  SerialCommunicator(const std::string& port_name, int baudrate,
                     DataCallback data_callback, ErrorCallback error_callback);

  ~SerialCommunicator();

  size_t Write(const uint8_t* data, size_t len);

 private:
  speed_t GetBaudRate(int baudrate);
  bool OpenSerialPort(const std::string& port, int baudrate);
  void RunReadThread();

  SerialPort serial_port_;

  const DataCallback data_callback_;  // const로 변경 불가
  const ErrorCallback error_callback_;

  std::atomic<bool> read_thread_activated_{true};
  std::mutex mutex_read_thread_;
  std::thread read_thread_;
};

SerialCommunicator::SerialCommunicator(const std::string& port_name,
                                       int baudrate, DataCallback data_callback,
                                       ErrorCallback error_callback)
    : serial_port_{port_name, static_cast<uint32_t>(baudrate), -1},
      data_callback_(data_callback),
      error_callback_(error_callback) {
  if (!OpenSerialPort(port_name, baudrate))
    throw std::runtime_error("Failed to open serial port: " + port_name);

  if (data_callback_ == nullptr || error_callback_ == nullptr) {
    throw std::runtime_error(
        "Data callback and error callback must be set before starting read "
        "thread.");
  }

  // Start read thread
  read_thread_activated_ = true;
  read_thread_ = std::thread(&SerialCommunicator::RunReadThread, this);
}

SerialCommunicator::~SerialCommunicator() {
  read_thread_activated_ = false;
  if (read_thread_.joinable()) read_thread_.join();
  if (serial_port_.fd != -1) {
    close(serial_port_.fd);
    serial_port_.fd = -1;
  }
}

size_t SerialCommunicator::Write(const uint8_t* data, size_t len) {
  if (serial_port_.fd == -1) return 0;

  auto written_bytes = write(serial_port_.fd, data, len);
  if (written_bytes < 0) {
    if (error_callback_)
      error_callback_("Write error: " + std::string(strerror(errno)));
    return 0;
  }
  return static_cast<size_t>(written_bytes);
}

speed_t SerialCommunicator::GetBaudRate(int baudrate) {
  static std::map<int, speed_t> baudrate_map = {
      {9600, B9600},       {19200, B19200},     {38400, B38400},
      {57600, B57600},     {115200, B115200},   {230400, B230400},
      {460800, B460800},   {500000, B500000},   {576000, B576000},
      {921600, B921600},   {1000000, B1000000}, {1152000, B1152000},
      {1500000, B1500000}, {2000000, B2000000}};

  auto it = baudrate_map.find(baudrate);
  if (it == baudrate_map.end()) {
    std::cerr << "Unsupported baudrate: " << baudrate << ", using 115200"
              << std::endl;
    return B115200;
  }
  return it->second;
}

bool SerialCommunicator::OpenSerialPort(const std::string& port, int baudrate) {
  // 포트 열기 (non-blocking)
  serial_port_.fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (serial_port_.fd < 0) {
    if (error_callback_) error_callback_("Failed to open port: " + port);
    return false;
  }

  // Serial 설정
  struct termios tty;
  if (tcgetattr(serial_port_.fd, &tty) != 0) {
    if (error_callback_) error_callback_("Failed to get terminal attributes");
    ::close(serial_port_.fd);
    serial_port_.fd = -1;
    return false;
  }

  // Baudrate 설정
  speed_t baud = GetBaudRate(baudrate);
  cfsetospeed(&tty, baud);
  cfsetispeed(&tty, baud);

  // 8N1 설정 (8 data bits, No parity, 1 stop bit)
  tty.c_cflag &= ~PARENB;  // No parity
  tty.c_cflag &= ~CSTOPB;  // 1 stop bit
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;             // 8 bits
  tty.c_cflag &= ~CRTSCTS;        // No hardware flow control
  tty.c_cflag |= CREAD | CLOCAL;  // Enable receiver, ignore modem control

  // Raw mode 설정
  tty.c_lflag &= ~ICANON;  // Disable canonical mode
  tty.c_lflag &= ~ECHO;    // Disable echo
  tty.c_lflag &= ~ECHOE;
  tty.c_lflag &= ~ECHONL;
  tty.c_lflag &= ~ISIG;  // Disable signal chars

  tty.c_iflag &= ~(IXON | IXOFF | IXANY);  // Disable software flow control
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

  tty.c_oflag &= ~OPOST;  // Disable output processing
  tty.c_oflag &= ~ONLCR;

  // Timeout 설정 (non-blocking이지만 설정)
  tty.c_cc[VTIME] = 1;  // 0.1초 timeout
  tty.c_cc[VMIN] = 0;   // Minimum bytes to read

  // 설정 적용
  if (tcsetattr(serial_port_.fd, TCSANOW, &tty) != 0) {
    if (error_callback_) error_callback_("Failed to set terminal attributes");
    close(serial_port_.fd);
    serial_port_.fd = -1;
    return false;
  }

  return true;
}

void SerialCommunicator::RunReadThread() {
  constexpr size_t kBufferSize = 1024;
  uint8_t buffer[kBufferSize];

  // poll을 사용한 이벤트 기반 읽기 (CPU 효율 향상)
  struct pollfd pfd;
  pfd.fd = serial_port_.fd;
  pfd.events = POLLIN;

  while (read_thread_activated_) {
    // 100ms timeout으로 poll 대기
    int poll_result = poll(&pfd, 1, 100);

    if (poll_result > 0 && (pfd.revents & POLLIN)) {
      // 데이터 수신 가능
      ssize_t n = ::read(serial_port_.fd, buffer, kBufferSize);

      if (n > 0) {
        // 콜백은 초기화 후 변경되지 않으므로 lock 불필요
        if (data_callback_) data_callback_(buffer, n);

      } else if (n < 0) {
        if (errno == EINTR) {
          // 시그널 인터럽트 - 계속 진행
          continue;
        } else if (errno != EAGAIN && errno != EWOULDBLOCK) {
          // 치명적 오류 - 에러 콜백 호출
          if (error_callback_)
            error_callback_("Read error: " + std::string(strerror(errno)));
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
      }
      // n == 0: EOF (연결 끊김 가능)

    } else if (poll_result < 0) {
      if (errno != EINTR) {
        // poll 오류
        if (error_callback_)
          error_callback_("Poll error: " + std::string(strerror(errno)));
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }
    // poll_result == 0: timeout, read_thread_activated_ 체크하며 계속
  }
}

// void SerialCommunicator::RunReadThread() {
//   constexpr size_t kBufferSize = 1024;
//   uint8_t buffer[kBufferSize];

//   while (read_thread_activated_) {
//     ssize_t n = ::read(serial_port_.fd, buffer, kBufferSize);

//     if (n > 0) {
//       // 데이터 수신 - 콜백 복사 후 lock 해제
//       DataCallback callback;
//       {
//         std::lock_guard<std::mutex> local_lock(callback_mutex_);
//         callback = data_callback_;
//       }
//       // Lock 없이 콜백 호출 (병렬성 향상)
//       if (callback) callback(buffer, n);

//     } else if (n < 0) {
//       if (errno == EAGAIN || errno == EWOULDBLOCK) {
//         // Non-blocking에서 데이터 없음 - 짧은 대기로 CPU 사용률 감소
//         std::this_thread::sleep_for(std::chrono::microseconds(100));
//       } else if (errno == EINTR) {
//         // 시그널 인터럽트 - 계속 진행
//         continue;
//       } else {
//         // 치명적 오류 - 에러 콜백 호출 후 계속 시도
//         ErrorCallback err_callback;
//         {
//           std::lock_guard<std::mutex> local_lock(callback_mutex_);
//           err_callback = error_callback_;
//         }
//         if (err_callback) {
//           err_callback("Read error: " + std::string(strerror(errno)));
//         }
//         // 짧은 대기 후 재시도 (포트 복구 가능성)
//         std::this_thread::sleep_for(std::chrono::milliseconds(100));
//       }
//     }
//     // n == 0: timeout (VTIME), 계속 읽기
//   }
// }

}  // namespace serial_communicator

#endif  // SERIAL_COMMUNICATOR_H_