#include <iostream>
#include <memory>

#include "communicator.h"
#include "packet_manager.h"
#include "packet_parser.h"

#define SERIAL_BAUDRATE 921600

int main() {
  try {
    // SerialCommunicator 초기화
    packet::PacketParser serial_parser;
    std::shared_ptr<Communicator> serial_comm_ptr =
        std::make_shared<SerialCommunicator>("/dev/ttyACM0", SERIAL_BAUDRATE);

    auto serial_data_cb = [&serial_parser](const uint8_t* data, size_t len) {
      serial_parser.AppendRawData(data, len);
    };
    auto serial_error_cb = [](const std::string& error) {
      std::cerr << "Serial error: " << error << std::endl;
    };
    serial_comm_ptr->SetDataCallback(serial_data_cb)
        .SetErrorCallback(serial_error_cb);
    if (!serial_comm_ptr->Start()) {
      std::cerr << "Failed to start serial communication." << std::endl;
      return -1;
    }

    // Ethernet Communicator 초기화
    packet::PacketParser eth_parser;
    int local_port = 44444;
    // std::string dest_ip = "127.0.0.1";
    // int dest_port = 45678;
    // std::shared_ptr<Communicator> eth_comm_ptr =
    //     std::make_shared<UdpCommunicator>(dest_ip, dest_port, local_port);
    std::shared_ptr<Communicator> eth_comm_ptr =
        std::make_shared<TcpServerCommunicator>(local_port);
    auto eth_data_cb = [&eth_parser](const uint8_t* data, size_t len) {
      eth_parser.AppendRawData(data, len);
    };
    auto eth_error_cb = [](const std::string& error) {
      std::cerr << "Ethernet error: " << error << std::endl;
    };
    eth_comm_ptr->SetDataCallback(eth_data_cb).SetErrorCallback(eth_error_cb);
    if (!eth_comm_ptr->Start()) {
      std::cerr << "Failed to start Ethernet communication." << std::endl;
      return -1;
    }

    // 메인 루프
    while (true) {
      eth_comm_ptr->Write("Hello via Ethernet");
      serial_comm_ptr->Write("Hello via Serial");
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    serial_comm_ptr->Stop();
    eth_comm_ptr->Stop();
  } catch (const std::exception& e) {
    std::cerr << "Exception: " << e.what() << std::endl;
    return -1;
  }

  return 0;
}
