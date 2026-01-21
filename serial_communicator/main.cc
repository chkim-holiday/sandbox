#include <chrono>
#include <csignal>
#include <cstring>
#include <iomanip>
#include <iostream>

#include "packet.h"
#include "packet_parser.h"
#include "packet_serializer.h"
#include "serial_communicator.h"

#define SERIAL_BAUDRATE 460800

template <typename T>
union ByteConverter {
  T value;
  uint8_t bytes[sizeof(T)];
};

using namespace serial_communicator;

volatile bool is_thread_running = true;

// PTP Sync 관련 변수
struct PTPState {
  uint8_t sync_seq = 0;
  uint64_t t1 = 0;  // SYNC 송신 시각 (master time) (Sync에 담겨 slave로)
  uint64_t t2 = 0;  // SYNC 수신 시각 (slave time) (Req에 담겨 master로)
  uint64_t t3 = 0;  // DELAY_REQ 송신 시각 (slave time) (Req에 담겨 master로)
  uint64_t t4 = 0;  // DELAY_REQ 수신 시각 (master time) (Resp에 담겨 slave로)
  int64_t offset_ns = 0;  // = ((t2 - t1) - (t4 - t3)) / 2
  int64_t delay_ns = 0;   // = ((t2 - t1) + (t4 - t3)) / 2 >= 0
};

PTPState ptp_state;
SerialCommunicator* serial_comm = nullptr;

void SignalHandler(int signal) {
  if (signal == SIGINT) {
    std::cout << "\nReceived SIGINT, shutting down..." << std::endl;
    is_thread_running = false;
  }
}

uint64_t GetLocalTime() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

void SendPacket(SerialCommunicator& comm, uint64_t timestamp_ns,
                const packet::MessageType id, uint8_t seq, const uint8_t* data,
                uint8_t length) {
  const auto serialized_packet = packet::PacketSerializer::SerializePacket(
      timestamp_ns, id, seq, data, length);
  const size_t written =
      comm.Write(reinterpret_cast<const uint8_t*>(serialized_packet.data()),
                 serialized_packet.size());
  (void)written;
}

// PTP SYNC 전송
void SendSync(SerialCommunicator& comm) {
  const uint64_t current_timestamp = GetLocalTime();

  ptp_state.t1 = current_timestamp;
  ByteConverter<uint64_t> bc{ptp_state.t1};
  SendPacket(comm, current_timestamp, packet::MessageType::kPTPSync,
             ptp_state.sync_seq++, bc.bytes, 8);
}

// PTP DELAY_REQ 전송
void SendDelayReq(SerialCommunicator& comm, uint8_t seq) {
  const uint64_t current_timestamp = GetLocalTime();

  std::cout << "[Host PC] >>> Sending DELAY_REQ (T3=" << current_timestamp
            << ")" << std::endl;

  ptp_state.t3 = current_timestamp;
  const ByteConverter<uint64_t> bc{ptp_state.t3};
  SendPacket(comm, current_timestamp, packet::MessageType::kPTPDelayRequest,
             seq, bc.bytes, 8);
}

// PTP DELAY_RESP 전송 (Master 역할)
void SendDelayResp(SerialCommunicator& comm, uint8_t req_seq, uint64_t t2) {
  const uint64_t current_timestamp = GetLocalTime();

  ptp_state.t4 = current_timestamp;

  // Data: req_seq (1 byte) + t4 (8 bytes)
  (void)req_seq;
  ByteConverter<uint64_t> bc{ptp_state.t4};

  std::cout << "[Host PC] >>> Sending DELAY_RESP (T2=" << t2
            << ", T4=" << ptp_state.t4
            << ") diff: " << (int64_t)(ptp_state.t4 - t2) << " ns "
            << std::endl;
  SendPacket(comm, current_timestamp, packet::MessageType::kPTPDelayResponse, 0,
             bc.bytes, 8);
}

int main(int argc, char* argv[]) {
  // 시그널 핸들러 등록
  std::signal(SIGINT, SignalHandler);

  // Serial port 설정
  std::string port_name = "/dev/ttyACM0";  // 기본값
  int baudrate = SERIAL_BAUDRATE;

  if (argc >= 2) port_name = argv[1];
  if (argc >= 3) baudrate = std::atoi(argv[2]);

  std::cout << "Opening serial port: " << port_name << " at " << baudrate
            << " baud" << std::endl;

  // PacketParser 생성
  packet::PacketParser parser;

  // PacketParser 콜백 설정
  auto packet_callback = [](const packet::Packet& packet) {
    std::unordered_map<uint8_t, std::string> message_type_names = {
        {static_cast<uint8_t>(packet::MessageType::kDebugEchoMsg), "ECHO"},
        {static_cast<uint8_t>(packet::MessageType::kDebugHeartBeatMsg),
         "HEARTBEAT"},
        {static_cast<uint8_t>(packet::MessageType::kPTPSync), "PTP SYNC"},
        {static_cast<uint8_t>(packet::MessageType::kPTPDelayRequest),
         "PTP DELAY_REQ"},
        {static_cast<uint8_t>(packet::MessageType::kPTPDelayResponse),
         "PTP DELAY_RESP"},
        {static_cast<uint8_t>(packet::MessageType::kPTPReportToMaster),
         "PTP REPORT_SLAVE_TO_MASTER"},
        {static_cast<uint8_t>(packet::MessageType::kIMUData), "IMU DATA"},
    };
    std::cout << "\n=== Received Packet ===" << std::endl;
    std::cout << "ID Type   : ";
    auto it = message_type_names.find(packet.id);
    if (it != message_type_names.end()) {
      std::cout << it->second << " (0x" << std::hex << std::setw(2)
                << std::setfill('0') << static_cast<int>(packet.id) << std::dec
                << ")" << std::endl;
    } else {
      std::cout << "Unknown (0x" << std::hex << std::setw(2)
                << std::setfill('0') << static_cast<int>(packet.id) << std::dec
                << ")" << std::endl;
    }
    std::cout << "ID        : 0x" << std::hex << std::setw(2)
              << std::setfill('0') << static_cast<int>(packet.id) << std::dec
              << std::endl;

    std::cout << "Timestamp : " << packet.timestamp_ns << " ns" << std::endl;
    std::cout << "Seq       : " << static_cast<int>(packet.seq) << std::endl;
    std::cout << "Length    : " << static_cast<int>(packet.length) << std::endl;

    const uint64_t current_time = GetLocalTime();
    std::cout << "  (current time: " << current_time << " ns)" << std::endl;
    std::cout << "  (MCU local time: " << packet.timestamp_ns << " ns)"
              << std::endl;

    // PTP 프로토콜 처리
    if (packet.id == static_cast<uint8_t>(packet::MessageType::kPTPSync)) {
      // SYNC 수신 (Slave 역할)
      const uint64_t t2 = GetLocalTime();

      std::cout << "<<< Received SYNC: T1(remote)=" << packet.timestamp_ns
                << ", T2(local)=" << t2 << std::endl;

      // DELAY_REQ 전송
      if (serial_comm) {
        SendDelayReq(*serial_comm, packet.seq);
      }

    } else if (packet.id ==
               static_cast<uint8_t>(packet::MessageType::kPTPDelayRequest)) {
      // DELAY_REQ 수신 (Master 역할)
      const uint64_t t2 = GetLocalTime();

      std::cout << "<<< Received DELAY_REQ: seq="
                << static_cast<int>(packet.seq) << ", T2(local)=" << t2
                << std::endl;

      // DELAY_RESP 전송
      if (serial_comm) {
        SendDelayResp(*serial_comm, packet.seq, t2);
      }

    } else if (packet.id ==
               static_cast<uint8_t>(packet::MessageType::kPTPDelayResponse)) {
      // DELAY_RESP 수신 (Slave 역할)
      if (packet.length >= 9) {
        uint8_t req_seq = packet.data[0];
        uint64_t t2;  // Remote가 DELAY_REQ를 받은 시각
        std::memcpy(&t2, packet.data + 1, 8);
        uint64_t t4 = packet.timestamp_ns;  // Remote가 DELAY_RESP를 보낸 시각

        std::cout << "<<< Received DELAY_RESP: req_seq="
                  << static_cast<int>(req_seq) << ", T2(remote)=" << t2
                  << ", T4(remote)=" << t4 << std::endl;

        // 간단한 예제: offset만 계산 (실제로는 T1, T3 저장 필요)
        std::cout << ">>> PTP calculation: T2=" << t2 << ", T3=" << ptp_state.t3
                  << ", T4=" << t4 << std::endl;

        if (ptp_state.t3 > 0) {
          int64_t delay = ((t4 - ptp_state.t3));  // 단방향 지연 근사
          std::cout << ">>> One-way delay: " << delay << " ns" << std::endl;
        }
      }
    } else if (packet.id ==
               static_cast<uint8_t>(packet::MessageType::kDebugEchoMsg)) {
      // ECHO 메시지 수신
      std::cout << "<<< Received ECHO message: ";
      for (int i = 0; i < packet.length; i++) {
        std::cout << packet.data[i];
      }
      std::cout << std::endl;
    } else if (packet.id ==
               static_cast<uint8_t>(packet::MessageType::kPTPReportToMaster)) {
      // PTP Report 메시지 수신
      std::cout << "<<< Received PTP REPORT_SLAVE_TO_MASTER message."
                << std::endl;
      int64_t offset = 0;
      uint64_t delay = 0;
      std::memcpy(&offset, packet.data, 8);
      std::memcpy(&delay, packet.data + 8, 8);
      std::cout << "    Reported offset: " << offset << " ns" << std::endl;
      std::cout << "    Reported path delay: " << delay << " ns" << std::endl;
    } else if (packet.id ==
               static_cast<uint8_t>(packet::MessageType::kDebugHeartBeatMsg)) {
      // HEARTBEAT 메시지 수신
      std::cout << "<<< Received HEARTBEAT message." << std::endl;
    }

    // Raw Packet 출력
    std::cout << "Raw Packet (hex): " << std::endl;
    std::cout << "  ";

    // Header
    printf("%02X ", PKT_HEADER_1);
    printf("%02X ", PKT_HEADER_2);

    // Timestamp (8 bytes)
    const uint8_t* ts_bytes =
        reinterpret_cast<const uint8_t*>(&packet.timestamp_ns);
    for (int i = 0; i < 8; i++) {
      printf("%02X ", ts_bytes[i]);
    }

    // ID
    printf("%02X ", packet.id);
    printf("%02X ", packet.seq);
    printf("%02X ", packet.length);
    for (int i = 0; i < packet.length; i++) {
      // printf("%02X ", packet.data[i]);
      printf("%c ", packet.data[i]);
      if ((i + 1) % 16 == 0 && i < packet.length - 1) {
        std::cout << std::endl << "  ";
      }
    }

    std::cout << std::endl;
    std::cout << "=====================\n" << std::endl;
  };
  parser.SetPacketCallback(packet_callback);

  try {
    // SerialCommunicator 생성
    auto data_callback = [&parser](const uint8_t* data, size_t len) {
      parser.AppendRawData(data, len);
    };
    auto error_callback = [](const std::string& error) {
      std::cerr << "Serial error: " << error << std::endl;
    };
    SerialCommunicator comm(port_name, baudrate);
    comm.SetDataCallback(data_callback);
    comm.SetErrorCallback(error_callback);
    if (!comm.StartSerialCommunication()) {
      throw std::runtime_error("Failed to start serial communication.");
    }

    serial_comm = &comm;  // 전역 포인터 설정

    std::cout << "Serial port opened successfully. Press Ctrl+C to exit."
              << std::endl;
    std::cout << "Sending PTP SYNC packets every 1 second..." << std::endl;

    // PTP SYNC 전송 루프 (Master 역할)
    // uint8_t test_seq = 0;
    while (is_thread_running) {
      // std::cout << "Send echo request..." << std::endl;
      // uint8_t test_data[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE};
      // SendPacket(comm, 12345678ULL, MessageType::kDebugEchoMsg,
      // test_seq++,
      //            test_data, 5);

      // Sync 전송
      std::cout << "[Host PC] Sending SYNC packet..." << std::endl;
      SendSync(comm);

      // 1초 대기
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  std::cout << "Exiting..." << std::endl;
  return 0;
}
