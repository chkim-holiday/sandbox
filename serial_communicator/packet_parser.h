#ifndef PACKET_PARSER_H_
#define PACKET_PARSER_H_

#include <cstdint>
#include <functional>

#include "crc8_table.h"

#define PKT_HEADER_1 0xAA
#define PKT_HEADER_2 0x55

enum class MessageType : uint8_t {
  kDebugEchoMsg = 0x01,
  kDebugHeartBeatMsg = 0x02,
  kPTPSync = 0x10,
  kPTPDelayReq = 0x11,
  kPTPDelayResp = 0x12,
  kIMUData = 0x20,
};

namespace serial_communicator {

class PacketParser {
 public:
  struct Packet {
    uint64_t timestamp_ns;
    uint8_t id;
    uint8_t seq;
    uint8_t length;
    uint8_t data[256];
  };
  using PacketCallback = std::function<void(const Packet& packet)>;

  void AppendRawData(const uint8_t* data, size_t len) {
    for (size_t i = 0; i < len; i++) ProcessByte(data[i]);
  }

  void SetPacketCallback(PacketCallback callback) {
    packet_callback_ = callback;
  }

 private:
  enum class ProcessState {
    WAIT_HEADER,     // 헤더 대기
    WAIT_TIMESTAMP,  // Timestamp 대기 (8바이트)
    WAIT_ID,         // ID 대기
    WAIT_SEQ,        // Sequence 대기
    WAIT_LEN,        // Length 대기
    WAIT_DATA,       // Data 수신 중
    WAIT_CRC         // CRC 대기
  };

  void ProcessByte(uint8_t byte) {
    switch (state_) {
      case ProcessState::WAIT_HEADER: {
        packet_buffer_[buffer_index_++] = byte;
        if (buffer_index_ == 2) {
          if (packet_buffer_[0] == PKT_HEADER_1 &&
              packet_buffer_[1] == PKT_HEADER_2) {
            state_ = ProcessState::WAIT_TIMESTAMP;
          } else {
            Reset();
          }
        }
      } break;
      case ProcessState::WAIT_TIMESTAMP: {
        packet_buffer_[buffer_index_] = byte;
        ++buffer_index_;
        reinterpret_cast<uint8_t*>(
            &current_packet_.timestamp_ns)[timestamp_bytes_received_++] = byte;
        if (timestamp_bytes_received_ >= 8) state_ = ProcessState::WAIT_ID;
      } break;
      case ProcessState::WAIT_ID: {
        packet_buffer_[buffer_index_] = byte;
        ++buffer_index_;
        current_packet_.id = byte;
        state_ = ProcessState::WAIT_SEQ;
      } break;
      case ProcessState::WAIT_SEQ: {
        packet_buffer_[buffer_index_] = byte;
        ++buffer_index_;
        current_packet_.seq = byte;
        state_ = ProcessState::WAIT_LEN;
      } break;
      case ProcessState::WAIT_LEN: {
        packet_buffer_[buffer_index_] = byte;
        ++buffer_index_;
        current_packet_.length = byte;
        data_received_ = 0;

        if (byte == 0) {
          // 데이터 없음 → 바로 CRC 대기
          state_ = ProcessState::WAIT_CRC;
        } else if (byte > 250) {
          // 비정상적으로 긴 데이터 → 리셋
          Reset();
        } else {
          state_ = ProcessState::WAIT_DATA;
        }
      } break;
      case ProcessState::WAIT_DATA: {
        packet_buffer_[buffer_index_] = byte;
        ++buffer_index_;
        current_packet_.data[data_received_++] = byte;

        if (data_received_ >= current_packet_.length)
          state_ = ProcessState::WAIT_CRC;

        // 버퍼 오버플로우 방지
        if (buffer_index_ >= sizeof(packet_buffer_) - 1) Reset();
      } break;
      case ProcessState::WAIT_CRC: {
        packet_buffer_[buffer_index_] = byte;
        ++buffer_index_;
        // CRC 검증 (헤더부터 데이터까지, CRC 제외)
        uint8_t calculated_crc = ComputeCRC8(packet_buffer_, buffer_index_ - 1);
        if (calculated_crc == byte && packet_callback_) {
          std::cerr << "invoking packet callback." << std::endl;
          packet_callback_(current_packet_);  // 패킷 완성! 콜백 호출
        }
        // CRC 실패해도 무시하고 다음 헤더 탐색
        Reset();
      } break;
    }
  }

  void Reset() {
    state_ = ProcessState::WAIT_HEADER;
    buffer_index_ = 0;
    data_received_ = 0;
    timestamp_bytes_received_ = 0;
  }

  PacketCallback packet_callback_;

  ProcessState state_;
  uint8_t packet_buffer_[512];
  size_t buffer_index_{0};
  size_t data_received_{0};
  size_t timestamp_bytes_received_{0};
  Packet current_packet_;
};

}  // namespace serial_communicator

#endif  // PACKET_PARSER_H_
