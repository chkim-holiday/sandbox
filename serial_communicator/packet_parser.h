#ifndef PACKET_PARSER_H_
#define PACKET_PARSER_H_

#include <cstdint>
#include <functional>

#include "crc8_table.h"
#include "packet.h"

namespace packet {

class PacketParser {
 public:
  using PacketCallback = std::function<void(const Packet& packet)>;

  void AppendRawData(const uint8_t* data, size_t len) {
    for (size_t i = 0; i < len; i++) ProcessByte(data[i]);
  }

  void SetPacketCallback(PacketCallback callback) {
    packet_callback_ = callback;
  }

 private:
  enum class ProcessState {
    kWaitHeader1,     // 헤더 1 대기
    kWaitHeader2,     // 헤더 2 대기
    kWaitTimestamp,   // Timestamp 대기 (8바이트)
    kWaitId,          // ID 대기
    kWaitSeq,         // Sequence 대기
    kWaitDataLength,  // Length 대기
    kWaitDataBlock,   // Data 수신 중
    kWaitCRC          // CRC 대기
  };

  void ProcessByte(uint8_t byte) {
    switch (state_) {
      case ProcessState::kWaitHeader1: {
        if (byte == PKT_HEADER_1) {
          packet_buffer_[0] = byte;
          buffer_index_ = 1;
          state_ = ProcessState::kWaitHeader2;
        }
      } break;
      case ProcessState::kWaitHeader2: {
        if (byte == PKT_HEADER_2) {
          packet_buffer_[buffer_index_++] = byte;
          state_ = ProcessState::kWaitTimestamp;
        } else {
          Reset();
          if (byte == PKT_HEADER_1) {
            packet_buffer_[0] = byte;
            buffer_index_ = 1;
            state_ = ProcessState::kWaitHeader2;
          }
        }
      } break;
      case ProcessState::kWaitTimestamp: {
        packet_buffer_[buffer_index_] = byte;
        ++buffer_index_;
        reinterpret_cast<uint8_t*>(
            &current_packet_.timestamp_ns)[timestamp_bytes_received_++] = byte;
        if (timestamp_bytes_received_ >= 8) state_ = ProcessState::kWaitId;
      } break;
      case ProcessState::kWaitId: {
        packet_buffer_[buffer_index_] = byte;
        ++buffer_index_;
        current_packet_.id = byte;
        state_ = ProcessState::kWaitSeq;
      } break;
      case ProcessState::kWaitSeq: {
        packet_buffer_[buffer_index_] = byte;
        ++buffer_index_;
        current_packet_.seq = byte;
        state_ = ProcessState::kWaitDataLength;
      } break;
      case ProcessState::kWaitDataLength: {
        packet_buffer_[buffer_index_] = byte;
        ++buffer_index_;
        current_packet_.length = byte;
        data_received_ = 0;

        if (byte == 0) {
          // 데이터 없음 → 바로 CRC 대기
          state_ = ProcessState::kWaitCRC;
        } else if (byte > 250) {
          // 비정상적으로 긴 데이터 → 리셋
          Reset();
        } else {
          state_ = ProcessState::kWaitDataBlock;
        }
      } break;
      case ProcessState::kWaitDataBlock: {
        packet_buffer_[buffer_index_] = byte;
        ++buffer_index_;
        current_packet_.data[data_received_++] = byte;

        if (data_received_ >= current_packet_.length)
          state_ = ProcessState::kWaitCRC;

        // 버퍼 오버플로우 방지
        if (buffer_index_ >= sizeof(packet_buffer_) - 1) Reset();
      } break;
      case ProcessState::kWaitCRC: {
        packet_buffer_[buffer_index_] = byte;
        ++buffer_index_;

        // CRC 검증 (헤더부터 데이터까지, CRC 제외)
        uint8_t calculated_crc = ComputeCRC8(packet_buffer_, buffer_index_ - 1);
        // if (calculated_crc == byte)
        //   std::cerr << "CRC OK, invoking packet callback." << std::endl;
        // else
        //   std::cerr << "CRC FAIL. calculated=0x" << std::hex
        //             << (int)calculated_crc << ", received=0x" << (int)byte
        //             << std::dec << std::endl;
        if (calculated_crc == byte && packet_callback_)
          packet_callback_(current_packet_);  // 패킷 완성! 콜백 호출
        // CRC 실패해도 무시하고 다음 헤더 탐색
        Reset();
      } break;
    }
  }

  void Reset() {
    state_ = ProcessState::kWaitHeader1;
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

}  // namespace packet

#endif  // PACKET_PARSER_H_
