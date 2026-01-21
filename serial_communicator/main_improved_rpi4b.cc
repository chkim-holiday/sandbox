#include <chrono>
#include <iostream>

#include "packet_manager.h"
#include "packet_parser.h"
#include "serial_communicator.h"

#include <pigpio.h>

#define GPIO_PIN 16

#define SERIAL_BAUDRATE 921600

int main() {
  // SerialCommunicator 초기화
  serial_communicator::SerialCommunicator serial_comm("/dev/ttyACM0",
                                                      SERIAL_BAUDRATE);

  // PacketParser 초기화
  packet::PacketParser parser;
  serial_comm.SetDataCallback([&parser](const uint8_t* data, size_t len) {
    parser.AppendRawData(data, len);
  });
  serial_comm.SetErrorCallback([](const std::string& error) {
    std::cerr << "Serial error: " << error << std::endl;
  });
  if (!serial_comm.Start()) {
    std::cerr << "Failed to start serial communication." << std::endl;
    return -1;
  }

  // PacketDispatcher 초기화
  PacketSubscriberManager subscriber_manager(parser);

  // CommandMessageProcessor 초기화 및 등록
  CommandMessageProcessor command_processor(serial_comm);
  command_processor.RegisterCallback([](const std::string& command) {
    std::cout << "[Host PC] <<< Received Command: " << command << std::endl;
    if (command == "SET_SPEED 100") {
      std::cout << "[Host PC] >>> Setting speed to 100" << std::endl;
    }
  });
  subscriber_manager.RegisterSubscriber(packet::MessageType::kCommandMessage,
                                        &command_processor);

  // IMUMessageReceiver 초기화 및 등록
  IMUMessageReceiver imu_receiver;
  imu_receiver.RegisterCallback([](const ImuData& imu_data) {
    std::cout << "[Host PC] <<< Received IMU Data:" << std::endl;
    std::cout << "  Acceleration: ax=" << imu_data.ax << ", ay=" << imu_data.ay
              << ", az=" << imu_data.az << std::endl;
    std::cout << "  Gyroscope: gx=" << imu_data.gx << ", gy=" << imu_data.gy
              << ", gz=" << imu_data.gz << std::endl;
    std::cout << "  Temperature: " << imu_data.temperature << std::endl;
  });
  subscriber_manager.RegisterSubscriber(packet::MessageType::kIMUData,
                                        &imu_receiver);

  // SerialPTPv2Server 초기화 및 등록
  const int sync_period_in_ms = 6000;  // 예: 1000ms (1초)
  timer::Timer sync_timer;
  SerialPTPServer ptp_server(serial_comm, sync_timer, sync_period_in_ms);
  subscriber_manager.RegisterSubscriber(packet::MessageType::kPTPDelayRequest,
                                        &ptp_server);
  subscriber_manager.RegisterSubscriber(packet::MessageType::kPTPReportToMaster,
                                        &ptp_server);

  // DebugMessageReceiver 초기화 및 등록
  DebugMessageReceiver debug_receiver;
  debug_receiver.RegisterCallback([](const std::string& message) {
    std::cout << "[Host PC] <<< Received Debug Message: " << message
              << std::endl;
  });
  subscriber_manager.RegisterSubscriber(packet::MessageType::kDebugEchoMsg,
                                        &debug_receiver);
  subscriber_manager.RegisterSubscriber(packet::MessageType::kDebugHeartBeatMsg,
                                        &debug_receiver);

  // GPIO 초기화
  if (gpioInitialise() < 0) {
    std::cerr << "Failed to initialize GPIO." << std::endl;
    return -1;
  }

  // GPIO 핀을 출력으로 설정
  gpioSetMode(GPIO_PIN, PI_OUTPUT);

  // 메인 루프
  uint64_t current_time = GetLocalTime();
  uint64_t next_trigger_time = (current_time / 20000000ULL) * 20000000ULL;
  if (current_time % 20000000ULL != 0) {
    next_trigger_time += 20000000ULL;
  }

  bool gpio_state = false;
  while (true) {
    auto synchronized_time = GetLocalTime();
    if (synchronized_time >= next_trigger_time) {
      gpio_state = !gpio_state;
      gpioWrite(GPIO_PIN, gpio_state);
      next_trigger_time += 20000000ULL;
    }
    // std::this_thread::sleep_for(std::chrono::microseconds(10));
  }

  return 0;
}
