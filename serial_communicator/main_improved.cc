#include <iostream>

#include "packet_manager.h"
#include "packet_parser.h"
#include "serial_communicator.h"

#define SERIAL_BAUDRATE 921600

int main() {
  // PacketParser 초기화
  packet::PacketParser parser;

  // SerialCommunicator 초기화
  serial_communicator::SerialCommunicator serial_comm("/dev/ttyACM0",
                                                      SERIAL_BAUDRATE);
  auto serial_data_cb = [&parser](const uint8_t* data, size_t len) {
    parser.AppendRawData(data, len);
  };
  auto serial_error_cb = [](const std::string& error) {
    std::cerr << "Serial error: " << error << std::endl;
  };
  serial_comm.SetDataCallback(serial_data_cb).SetErrorCallback(serial_error_cb);
  if (!serial_comm.Start()) {
    std::cerr << "Failed to start serial communication." << std::endl;
    return -1;
  }

  // PacketSubscriberManager 초기화
  PacketSubscriberManager subscriber_manager(parser);

  // CommandMessageProcessor 초기화 및 등록
  auto command_callback = [](const std::string& command) { (void)command; };
  CommandMessageProcessor command_processor(serial_comm);
  command_processor.RegisterCallback(command_callback);
  subscriber_manager.RegisterSubscriber(packet::MessageType::kCommandMessage,
                                        &command_processor);

  // IMUMessageReceiver 초기화 및 등록
  auto imu_callback = [](const ImuData& imu_data) { (void)imu_data; };
  IMUMessageReceiver imu_receiver;
  imu_receiver.RegisterCallback(imu_callback);
  subscriber_manager.RegisterSubscriber(packet::MessageType::kIMUData,
                                        &imu_receiver);

  // SerialPTPServer 초기화 및 등록
  constexpr int kSyncPeriodInMs{2000};
  timer::Timer sync_timer;
  SerialPTPServer serial_ptp_server(serial_comm, sync_timer, kSyncPeriodInMs);
  subscriber_manager.RegisterSubscriber(packet::MessageType::kPTPDelayRequest,
                                        &serial_ptp_server);
  subscriber_manager.RegisterSubscriber(packet::MessageType::kPTPReportToMaster,
                                        &serial_ptp_server);

  // DebugMessageReceiver 초기화 및 등록
  auto debug_callback = [](const std::string& message) {
    std::cout << message << std::endl;
  };
  DebugMessageReceiver debug_receiver;
  debug_receiver.RegisterCallback(debug_callback);
  subscriber_manager.RegisterSubscriber(packet::MessageType::kDebugEchoMsg,
                                        &debug_receiver);
  subscriber_manager.RegisterSubscriber(packet::MessageType::kDebugHeartBeatMsg,
                                        &debug_receiver);

  // 메인 루프
  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  return 0;
}
