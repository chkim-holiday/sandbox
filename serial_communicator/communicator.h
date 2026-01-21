#ifndef COMMUNICATOR_H_
#define COMMUNICATOR_H_

#include <atomic>
#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <thread>

// Common
#include <poll.h>

// Serial
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>

// Ethernet
#include <arpa/inet.h>
#include <sys/socket.h>

class Communicator {
 public:
  using DataCallback = std::function<void(const uint8_t* data, size_t len)>;
  using ErrorCallback = std::function<void(const std::string& error)>;

  virtual ~Communicator() {}

  Communicator& SetDataCallback(DataCallback data_callback) {
    data_callback_ = data_callback;
    return *this;
  }

  Communicator& SetErrorCallback(ErrorCallback error_callback) {
    error_callback_ = error_callback;
    return *this;
  }

  bool Start() {
    if (!data_callback_) {
      if (error_callback_)
        error_callback_(
            "Data callback is not set. Cannot start communication.");
      return false;
    }
    if (!OpenCommunication()) {
      if (error_callback_) error_callback_("Failed to open communication.");
      return false;
    }
    StartReadThread();
    return true;
  }

  void Stop() {
    StopReadThread();
    CloseCommunication();
  }

  virtual int Write(const uint8_t* data, size_t len) = 0;

  int Write(const std::string& msg) {
    return Write(reinterpret_cast<const uint8_t*>(msg.data()), msg.size());
  }

 protected:
  virtual int ReadData(uint8_t* buffer, size_t buffer_size) = 0;

  virtual int GetFileDescriptor() const = 0;

  void StartReadThread() {
    read_thread_activated_ = true;
    read_thread_ = std::thread(&Communicator::RunReadThread, this);
  }

  void StopReadThread() {
    read_thread_activated_ = false;
    if (read_thread_.joinable()) read_thread_.join();
  }

  DataCallback data_callback_;
  ErrorCallback error_callback_;

 private:
  virtual bool OpenCommunication() { return false; }

  virtual void CloseCommunication() {
    std::cerr << "CloseCommunication not implemented" << std::endl;
  }

  void RunReadThread() {
    constexpr size_t kBufferSize = 1024;
    uint8_t buffer[kBufferSize];
    struct pollfd pfd;
    pfd.fd = GetFileDescriptor();  // 파일 디스크립터 가져오기
    if (pfd.fd == -1) {
      if (error_callback_) error_callback_("Invalid file descriptor");
      return;
    }
    pfd.events = POLLIN;  // 읽기 이벤트 감시
    while (read_thread_activated_) {
      int poll_result = poll(&pfd, 1, 100);  // 100ms 타임아웃
      if (poll_result > 0) {
        if (pfd.revents & POLLIN) {
          int n = ReadData(buffer, kBufferSize);
          if (n > 0) {
            if (data_callback_) data_callback_(buffer, n);
          } else if (n < 0) {
            if (error_callback_)
              error_callback_("Read error occurred: " +
                              std::string(strerror(errno)));
          }
        }
      } else if (poll_result < 0) {
        if (error_callback_)
          error_callback_("Poll error occurred: " +
                          std::string(strerror(errno)));
      }
    }
  }

  std::atomic<bool> read_thread_activated_{false};
  std::thread read_thread_;
};

class SerialCommunicator : public Communicator {
 public:
  SerialCommunicator(const std::string& port_name, int baudrate)
      : port_name_(port_name), baudrate_(baudrate), fd_(-1) {}

  ~SerialCommunicator() { Stop(); }

  int Write(const uint8_t* data, size_t len) override {
    return write(fd_, data, len);
  }

 protected:
  int ReadData(uint8_t* buffer, size_t buffer_size) override {
    return read(fd_, buffer, buffer_size);
  }

  int GetFileDescriptor() const override { return fd_; }

 private:
  bool OpenCommunication() override {  // 포트 열기 (non-blocking)
    if (!OpenSerialPort(port_name_, baudrate_)) {
      if (error_callback_)
        error_callback_("Failed to open serial port: " + port_name_);
      return false;
    }
    std::cout << "Serial port " << port_name_ << " opened at baudrate "
              << baudrate_ << std::endl;
    return true;
  }

  void CloseCommunication() override {
    std::cerr << "Close SerialCommunication" << std::endl;
    if (fd_ != -1) {
      close(fd_);
      fd_ = -1;
    }
  }

  bool OpenSerialPort(const std::string& port, int baudrate) {
    // 포트 열기 (non-blocking)
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      if (error_callback_) error_callback_("Failed to open port: " + port);
      return false;
    }

    // Serial 설정
    struct termios tty;
    if (tcgetattr(fd_, &tty) != 0) {
      if (error_callback_) error_callback_("Failed to get terminal attributes");
      ::close(fd_);
      fd_ = -1;
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
    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
      if (error_callback_) error_callback_("Failed to set serial attributes");
      close(fd_);
      fd_ = -1;
      return false;
    }

    return true;
  }

  speed_t GetBaudRate(int baudrate) {
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

  std::string port_name_;
  int baudrate_;
  int fd_;
};

// class EthernetCommunicator : public Communicator {
//  public:
//   struct Protocol {
//     static constexpr uint16_t kTCP = 0;
//     static constexpr uint16_t kUDP = 1;
//   };

//   EthernetCommunicator(const std::string& ip, int port,
//                        uint16_t protocol = Protocol::kTCP)
//       : ip_(ip), port_(port), fd_(-1), protocol_(protocol) {}

//   ~EthernetCommunicator() { Stop(); }

//   int Write(const uint8_t* data, size_t len) override {
//     if (protocol_ == Protocol::kTCP) {
//       return send(fd_, data, len, 0);  // TCP 전송
//     } else if (protocol_ == Protocol::kUDP) {
//       sockaddr_in dest_addr{};
//       dest_addr.sin_family = AF_INET;
//       dest_addr.sin_port = htons(port_);
//       inet_pton(AF_INET, ip_.c_str(), &dest_addr.sin_addr);

//       return sendto(fd_, data, len, 0, (struct sockaddr*)&dest_addr,
//                     sizeof(dest_addr));  // UDP 전송
//     } else {
//       return -1;  // unknown protocol
//     }
//   }

//  protected:
//   int ReadData(uint8_t* buffer, size_t buffer_size) override {
//     if (protocol_ == Protocol::kTCP) {
//       int n = recv(fd_, buffer, buffer_size, 0);
//       if (n > 0)
//         return n;
//       else if (n == 0) {
//         if (error_callback_) error_callback_("TCP connection closed by
//         peer."); return -1;
//       } else {
//         if (error_callback_)
//           error_callback_("TCP read error: " + std::string(strerror(errno)));
//         return -1;
//       }
//     } else if (protocol_ == Protocol::kUDP) {
//       sockaddr_in src_addr{};
//       socklen_t addr_len = sizeof(src_addr);
//       return recvfrom(fd_, buffer, buffer_size, 0, (struct
//       sockaddr*)&src_addr,
//                       &addr_len);
//     }
//     return -1;
//   }

//   int GetFileDescriptor() const override { return fd_; }

//  private:
//   bool OpenCommunication() override {
//     if (!OpenEthernetPort(ip_, port_)) {
//       error_callback_("Failed to open Ethernet port: " + ip_ + ":" +
//                       std::to_string(port_));
//       return false;
//     }
//     std::cout << "Ethernet port " << ip_ << ":" << port_
//               << " opened using protocol "
//               << ((protocol_ == Protocol::kTCP) ? "TCP" : "UDP") <<
//               std::endl;
//     return true;
//   }

//   void CloseCommunication() override {
//     std::cerr << "Close EthernetCommunication" << std::endl;
//     if (fd_ != -1) {
//       close(fd_);
//       fd_ = -1;
//     }
//   }

//   bool OpenEthernetPort(const std::string& ip, int port) {
//     fd_ = socket(AF_INET,
//                  (protocol_ == Protocol::kTCP) ? SOCK_STREAM : SOCK_DGRAM,
//                  0);
//     if (fd_ == -1) {
//       if (error_callback_) error_callback_("Failed to create socket");
//       return false;
//     }

//     sockaddr_in server_addr{};
//     server_addr.sin_family = AF_INET;
//     server_addr.sin_port = htons(port);
//     inet_pton(AF_INET, ip.c_str(), &server_addr.sin_addr);

//     if (protocol_ == Protocol::kTCP) {
//       if (connect(fd_, (struct sockaddr*)&server_addr, sizeof(server_addr)) <
//           0) {
//         if (error_callback_)
//           error_callback_("Failed to connect to server. (Errno: " +
//                           std::string(strerror(errno)) + ")");
//         return false;
//       }
//     } else if (protocol_ == Protocol::kUDP) {
//       sockaddr_in local_addr{};
//       local_addr.sin_family = AF_INET;
//       local_addr.sin_port = htons(0);  // 임의의 로컬 포트
//       local_addr.sin_addr.s_addr = INADDR_ANY;

//       if (bind(fd_, (struct sockaddr*)&local_addr, sizeof(local_addr)) < 0) {
//         if (error_callback_)
//           error_callback_("Failed to bind UDP socket. (Errno: " +
//                           std::string(strerror(errno)) + ")");
//         return false;
//       }
//     }

//     return true;
//   }

//  private:
//   std::string ip_;
//   int port_;
//   int fd_;
//   uint16_t protocol_;
// };

class TcpServerCommunicator : public Communicator {
 public:
  TcpServerCommunicator(int port)
      : port_(port), listen_fd_(-1), client_fd_(-1) {}

  ~TcpServerCommunicator() { Stop(); }

  int Write(const uint8_t* data, size_t len) override {
    if (client_fd_ == -1) return -1;
    return send(client_fd_, data, len, 0);
  }

 protected:
  int ReadData(uint8_t* buffer, size_t buffer_size) override {
    if (client_fd_ == -1) return -1;
    int n = recv(client_fd_, buffer, buffer_size, 0);
    if (n == 0) {
      if (error_callback_) error_callback_("Client disconnected");
      close(client_fd_);
      client_fd_ = -1;
      // 새 클라이언트 대기
      AcceptNewClient();
    }
    return n;
  }

  int GetFileDescriptor() const override { return client_fd_; }

 private:
  bool OpenCommunication() override {
    listen_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd_ == -1) {
      if (error_callback_) error_callback_("Failed to create socket");
      return false;
    }

    // SO_REUSEADDR 설정 (빠른 재시작 가능)
    int opt = 1;
    setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port_);
    server_addr.sin_addr.s_addr = INADDR_ANY;  // 모든 인터페이스

    if (bind(listen_fd_, (struct sockaddr*)&server_addr, sizeof(server_addr)) <
        0) {
      if (error_callback_)
        error_callback_("Bind failed: " + std::string(strerror(errno)));
      close(listen_fd_);
      listen_fd_ = -1;
      return false;
    }

    if (listen(listen_fd_, 1) < 0) {  // backlog = 1
      if (error_callback_)
        error_callback_("Listen failed: " + std::string(strerror(errno)));
      close(listen_fd_);
      listen_fd_ = -1;
      return false;
    }

    std::cout << "TCP Server listening on port " << port_ << std::endl;

    // 클라이언트 연결 대기
    if (!AcceptNewClient()) {
      return false;
    }

    return true;
  }

  bool AcceptNewClient() {
    sockaddr_in client_addr{};
    socklen_t addr_len = sizeof(client_addr);

    std::cout << "Waiting for client connection..." << std::endl;
    client_fd_ = accept(listen_fd_, (struct sockaddr*)&client_addr, &addr_len);

    if (client_fd_ < 0) {
      if (error_callback_)
        error_callback_("Accept failed: " + std::string(strerror(errno)));
      return false;
    }

    char client_ip[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &client_addr.sin_addr, client_ip, INET_ADDRSTRLEN);
    std::cout << "Client connected from " << client_ip << ":"
              << ntohs(client_addr.sin_port) << std::endl;

    return true;
  }

  void CloseCommunication() override {
    std::cout << "Closing TCP Server" << std::endl;
    if (client_fd_ != -1) {
      close(client_fd_);
      client_fd_ = -1;
    }
    if (listen_fd_ != -1) {
      close(listen_fd_);
      listen_fd_ = -1;
    }
  }

  int port_;
  int listen_fd_;  // listening socket
  int client_fd_;  // accepted client socket
};

// TCP Client - connect 구조
class TcpClientCommunicator : public Communicator {
 public:
  TcpClientCommunicator(const std::string& ip, int port)
      : ip_(ip), port_(port), fd_(-1) {}

  ~TcpClientCommunicator() { Stop(); }

  int Write(const uint8_t* data, size_t len) override {
    if (fd_ == -1) return -1;
    return send(fd_, data, len, 0);
  }

 protected:
  int ReadData(uint8_t* buffer, size_t buffer_size) override {
    int n = recv(fd_, buffer, buffer_size, 0);
    if (n == 0) {
      if (error_callback_) error_callback_("Server closed connection");
      return -1;
    }
    return n;
  }

  int GetFileDescriptor() const override { return fd_; }

 private:
  bool OpenCommunication() override {
    fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (fd_ == -1) {
      if (error_callback_) error_callback_("Failed to create socket");
      return false;
    }

    sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port_);

    if (inet_pton(AF_INET, ip_.c_str(), &server_addr.sin_addr) <= 0) {
      if (error_callback_) error_callback_("Invalid address: " + ip_);
      close(fd_);
      fd_ = -1;
      return false;
    }

    if (connect(fd_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
      if (error_callback_)
        error_callback_("Connect failed: " + std::string(strerror(errno)));
      close(fd_);
      fd_ = -1;
      return false;
    }

    std::cout << "Connected to TCP server " << ip_ << ":" << port_ << std::endl;
    return true;
  }

  void CloseCommunication() override {
    std::cout << "Closing TCP Client" << std::endl;
    if (fd_ != -1) {
      close(fd_);
      fd_ = -1;
    }
  }

  std::string ip_;
  int port_;
  int fd_;
};

class UdpCommunicator : public Communicator {
 public:
  UdpCommunicator(const std::string& dest_ip, int dest_port, int local_port)
      : dest_ip_(dest_ip),
        dest_port_(dest_port),
        local_port_(local_port),
        fd_(-1) {}

  ~UdpCommunicator() { Stop(); }

  int Write(const uint8_t* data, size_t len) override {
    sockaddr_in dest_addr{};
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(dest_port_);
    inet_pton(AF_INET, dest_ip_.c_str(), &dest_addr.sin_addr);

    return sendto(fd_, data, len, 0, (struct sockaddr*)&dest_addr,
                  sizeof(dest_addr));
  }

 protected:
  int ReadData(uint8_t* buffer, size_t buffer_size) override {
    sockaddr_in src_addr{};
    socklen_t addr_len = sizeof(src_addr);
    return recvfrom(fd_, buffer, buffer_size, 0, (struct sockaddr*)&src_addr,
                    &addr_len);
  }

  int GetFileDescriptor() const override { return fd_; }

 private:
  bool OpenCommunication() override {
    fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ == -1) {
      if (error_callback_) error_callback_("Failed to create UDP socket");
      return false;
    }

    sockaddr_in local_addr{};
    local_addr.sin_family = AF_INET;
    local_addr.sin_port = htons(local_port_);  // 받을 포트
    local_addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(fd_, (struct sockaddr*)&local_addr, sizeof(local_addr)) < 0) {
      if (error_callback_)
        error_callback_("UDP bind failed: " + std::string(strerror(errno)));
      close(fd_);
      fd_ = -1;
      return false;
    }

    std::cout << "UDP socket is opend on local port: [" << local_port_ << "]"
              << std::endl;
    std::cout << "UDP destination set to " << dest_ip_ << ":" << dest_port_
              << std::endl;
    return true;
  }

  void CloseCommunication() override {
    std::cout << "Closing UDP" << std::endl;
    if (fd_ != -1) {
      close(fd_);
      fd_ = -1;
    }
  }

  std::string dest_ip_;
  int dest_port_;
  int local_port_;
  int fd_;
};

#endif  // COMMUNICATOR_H_
