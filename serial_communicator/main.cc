#include <chrono>
#include <csignal>
#include <cstring>
#include <iomanip>
#include <iostream>

#include "packet_parser.h"
#include "serial_communicator.h"

using namespace serial_communicator;

volatile bool g_running = true;

// PTP Sync 관련 변수
struct PTPState {
  uint8_t sync_seq = 0;
  uint64_t t1 = 0;  // SYNC 전송 시각 (Local)
  uint64_t t2 = 0;  // DELAY_REQ 수신 시각 (Remote가 보낸 값)
  uint64_t t3 = 0;  // DELAY_REQ 전송 시각 (Local)
  uint64_t t4 = 0;  // DELAY_RESP 전송 시각 (Remote가 보낸 값)
  int64_t offset_ns = 0;
  int64_t delay_ns = 0;
};

PTPState ptp_state;
SerialCommunicator* g_comm = nullptr;

void SignalHandler(int signal) {
  if (signal == SIGINT) {
    std::cout << "\nReceived SIGINT, shutting down..." << std::endl;
    g_running = false;
  }
}

// 패킷 전송 헬퍼 함수
void SendPacket(SerialCommunicator& comm, uint64_t timestamp_ns,
                const MessageType id, uint8_t seq, const uint8_t* data,
                uint8_t length) {
  uint8_t packet_buffer[512];
  size_t idx = 0;

  // Header
  packet_buffer[idx++] = PKT_HEADER;

  // Timestamp (8 bytes, little-endian)
  std::memcpy(&packet_buffer[idx], &timestamp_ns, sizeof(timestamp_ns));
  idx += sizeof(timestamp_ns);

  // ID
  packet_buffer[idx++] = static_cast<uint8_t>(id);

  // Sequence
  packet_buffer[idx++] = seq;

  // Length
  packet_buffer[idx++] = length;

  // Data
  if (length > 0 && data != nullptr) {
    std::memcpy(&packet_buffer[idx], data, length);
    idx += length;
  }

  // CRC (헤더부터 데이터까지)
  uint8_t crc = ComputeCRC8(packet_buffer, idx);
  packet_buffer[idx++] = crc;

  // 전송
  size_t written = comm.Write(packet_buffer, idx);
  std::cout << "Sent packet: ID=0x" << std::hex << static_cast<int>(id)
            << std::dec << ", seq=" << static_cast<int>(seq)
            << ", len=" << static_cast<int>(length) << ", written=" << written
            << " bytes" << std::endl;
}

// PTP SYNC 전송
void SendSync(SerialCommunicator& comm) {
  auto now = std::chrono::steady_clock::now();
  ptp_state.t1 = std::chrono::duration_cast<std::chrono::nanoseconds>(
                     now.time_since_epoch())
                     .count();

  std::cout << "\n>>> Sending SYNC (T1=" << ptp_state.t1 << ")" << std::endl;
  SendPacket(comm, ptp_state.t1, MessageType::kPTPSync, ptp_state.sync_seq++,
             nullptr, 0);
}

// PTP DELAY_REQ 전송
void SendDelayReq(SerialCommunicator& comm, uint8_t seq) {
  auto now = std::chrono::steady_clock::now();
  ptp_state.t3 = std::chrono::duration_cast<std::chrono::nanoseconds>(
                     now.time_since_epoch())
                     .count();

  std::cout << ">>> Sending DELAY_REQ (T3=" << ptp_state.t3 << ")" << std::endl;
  SendPacket(comm, ptp_state.t3, MessageType::kPTPDelayReq, seq, nullptr, 0);
}

// PTP DELAY_RESP 전송 (Master 역할)
void SendDelayResp(SerialCommunicator& comm, uint8_t req_seq, uint64_t t2) {
  auto now = std::chrono::steady_clock::now();
  uint64_t t4 = std::chrono::duration_cast<std::chrono::nanoseconds>(
                    now.time_since_epoch())
                    .count();

  // Data: req_seq (1 byte) + t2 (8 bytes)
  uint8_t data[9];
  data[0] = req_seq;
  std::memcpy(data + 1, &t2, 8);

  std::cout << ">>> Sending DELAY_RESP (T2=" << t2 << ", T4=" << t4 << ")"
            << std::endl;
  SendPacket(comm, t4, MessageType::kPTPDelayResp, 0, data, 9);
}

int main(int argc, char* argv[]) {
  // 시그널 핸들러 등록
  std::signal(SIGINT, SignalHandler);

  // Serial port 설정
  std::string port_name = "/dev/ttyACM0";  // 기본값
  int baudrate = 115200;

  if (argc >= 2) {
    port_name = argv[1];
  }
  if (argc >= 3) {
    baudrate = std::atoi(argv[2]);
  }

  std::cout << "Opening serial port: " << port_name << " at " << baudrate
            << " baud" << std::endl;

  // PacketParser 생성
  PacketParser parser;

  // PacketParser 콜백 설정
  parser.SetPacketCallback([](const PacketParser::Packet& packet) {
    std::cout << "\n=== Received Packet ===" << std::endl;
    std::cout << "ID        : 0x" << std::hex << std::setw(2)
              << std::setfill('0') << static_cast<int>(packet.id) << std::dec
              << std::endl;
    std::cout << "Timestamp : " << packet.timestamp_ns << " ns" << std::endl;
    std::cout << "Seq       : " << static_cast<int>(packet.seq) << std::endl;
    std::cout << "Length    : " << static_cast<int>(packet.length) << std::endl;

    // PTP 프로토콜 처리
    if (packet.id == static_cast<uint8_t>(MessageType::kPTPSync)) {
      // SYNC 수신 (Slave 역할)
      // T1 = packet.timestamp_ns (Remote가 전송한 시각)
      // T2 = 현재 시각 (수신 시각)
      auto now = std::chrono::steady_clock::now();
      uint64_t t2 = std::chrono::duration_cast<std::chrono::nanoseconds>(
                        now.time_since_epoch())
                        .count();

      std::cout << "<<< Received SYNC: T1(remote)=" << packet.timestamp_ns
                << ", T2(local)=" << t2 << std::endl;

      // DELAY_REQ 전송
      if (g_comm) {
        SendDelayReq(*g_comm, packet.seq);
      }

    } else if (packet.id == static_cast<uint8_t>(MessageType::kPTPDelayReq)) {
      // DELAY_REQ 수신 (Master 역할)
      auto now = std::chrono::steady_clock::now();
      uint64_t t2 = std::chrono::duration_cast<std::chrono::nanoseconds>(
                        now.time_since_epoch())
                        .count();

      std::cout << "<<< Received DELAY_REQ: seq="
                << static_cast<int>(packet.seq) << ", T2(local)=" << t2
                << std::endl;

      // DELAY_RESP 전송
      if (g_comm) {
        SendDelayResp(*g_comm, packet.seq, t2);
      }

    } else if (packet.id == static_cast<uint8_t>(MessageType::kPTPDelayResp)) {
      // DELAY_RESP 수신 (Slave 역할)
      if (packet.length >= 9) {
        uint8_t req_seq = packet.data[0];
        uint64_t t2;  // Remote가 DELAY_REQ를 받은 시각
        std::memcpy(&t2, packet.data + 1, 8);
        uint64_t t4 = packet.timestamp_ns;  // Remote가 DELAY_RESP를 보낸 시각

        std::cout << "<<< Received DELAY_RESP: req_seq="
                  << static_cast<int>(req_seq) << ", T2(remote)=" << t2
                  << ", T4(remote)=" << t4 << std::endl;

        // Offset 및 Delay 계산
        // T1: 우리가 받은 SYNC의 timestamp (remote TX)
        // T2: remote가 DELAY_REQ 받은 시각
        // T3: 우리가 DELAY_REQ 보낸 시각
        // T4: remote가 DELAY_RESP 보낸 시각

        // 간단한 예제: offset만 계산 (실제로는 T1, T3 저장 필요)
        std::cout << ">>> PTP calculation: T2=" << t2 << ", T3=" << ptp_state.t3
                  << ", T4=" << t4 << std::endl;

        if (ptp_state.t3 > 0) {
          int64_t delay = ((t4 - ptp_state.t3));  // 단방향 지연 근사
          std::cout << ">>> One-way delay: " << delay << " ns" << std::endl;
        }
      }
    }

    // Raw Packet 출력
    std::cout << "Raw Packet (hex): " << std::endl;
    std::cout << "  ";

    // Header
    printf("%02X ", PKT_HEADER);

    // Timestamp (8 bytes)
    const uint8_t* ts_bytes =
        reinterpret_cast<const uint8_t*>(&packet.timestamp_ns);
    for (int i = 0; i < 8; i++) {
      printf("%02X ", ts_bytes[i]);
    }

    // ID
    printf("%02X ", packet.id);

    // Seq
    printf("%02X ", packet.seq);

    // Length
    printf("%02X ", packet.length);

    // Data
    for (int i = 0; i < packet.length; i++) {
      // printf("%02X ", packet.data[i]);
      printf("%c ", packet.data[i]);
      if ((i + 1) % 16 == 0 && i < packet.length - 1) {
        std::cout << std::endl << "  ";
      }
    }

    std::cout << std::endl;
    std::cout << "=====================\n" << std::endl;
  });

  try {
    // SerialCommunicator 생성
    SerialCommunicator comm(
        port_name, baudrate,
        // Data callback: 수신한 데이터를 PacketParser에 전달
        [&parser](const uint8_t* data, size_t len) {
          parser.AppendRawData(data, len);
        },
        // Error callback: 에러 출력
        [](const std::string& error) {
          std::cerr << "Serial error: " << error << std::endl;
        });

    g_comm = &comm;  // 전역 포인터 설정

    std::cout << "Serial port opened successfully. Press Ctrl+C to exit."
              << std::endl;
    std::cout << "Sending PTP SYNC packets every 1 second..." << std::endl;

    // PTP SYNC 전송 루프 (Master 역할)
    uint8_t test_seq = 0;
    while (g_running) {
      // 테스트 데이터 생성
      uint8_t test_data[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE};

      std::cout << "\n>>> Sending ECHO with seq=" << static_cast<int>(test_seq)
                << ", len=5" << std::endl;

      SendPacket(comm, 12345678ULL, MessageType::kEcho, test_seq++, test_data,
                 5);

      // 1초 대기
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }

  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  std::cout << "Exiting..." << std::endl;
  return 0;
}
