#include <iostream>

#include "packet_manager.h"
#include "packet_parser.h"
#include "serial_communicator.h"

int main() {
  // SerialCommunicator 초기화
  serial_communicator::SerialCommunicator serial_comm("/dev/ttyACM0", 460800);

  // PacketParser 초기화
  packet::PacketParser parser;
  serial_comm.SetDataCallback([&parser](const uint8_t* data, size_t len) {
    parser.AppendRawData(data, len);
  });
  serial_comm.SetErrorCallback([](const std::string& error) {
    std::cerr << "Serial error: " << error << std::endl;
  });
  if (!serial_comm.StartSerialCommunication()) {
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
  imu_receiver.RegisterCallback(
      [](const IMUMessageReceiver::ImuData& imu_data) {
        std::cout << "[Host PC] <<< Received IMU Data:" << std::endl;
        std::cout << "  Acceleration: ax=" << imu_data.ax
                  << ", ay=" << imu_data.ay << ", az=" << imu_data.az
                  << std::endl;
        std::cout << "  Gyroscope: gx=" << imu_data.gx << ", gy=" << imu_data.gy
                  << ", gz=" << imu_data.gz << std::endl;
        std::cout << "  Temperature: " << imu_data.temperature << std::endl;
      });
  subscriber_manager.RegisterSubscriber(packet::MessageType::kIMUData,
                                        &imu_receiver);

  // SerialPTPv2Server 초기화 및 등록
  const int sync_period_in_ms = 2000;  // 예: 1000ms (1초)
  timer::Timer sync_timer;
  SerialPTPv2Server ptp_server(serial_comm, sync_period_in_ms, sync_timer);
  subscriber_manager.RegisterSubscriber(packet::MessageType::kPTPDelayRequest,
                                        &ptp_server);
  subscriber_manager.RegisterSubscriber(
      packet::MessageType::kPTPReportSlaveToMaster, &ptp_server);

  // 메인 루프
  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  return 0;
}
