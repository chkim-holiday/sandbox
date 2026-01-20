#ifndef PACKET_H_
#define PACKET_H_

#include <cstdint>

#define PKT_HEADER_1 0xAA
#define PKT_HEADER_2 0x55

namespace packet {

enum MessageType : uint8_t {
  kDebugEchoMsg = 0x01,
  kDebugHeartBeatMsg = 0x02,
  kPTPSync = 0x10,
  kPTPDelayRequest = 0x11,
  kPTPDelayResponse = 0x12,
  kPTPReportSlaveToMaster = 0x13,
  kIMUData = 0x20,
};

struct Packet {
  uint64_t timestamp_ns{0};
  uint8_t id{0};
  uint8_t seq{0};
  uint8_t length{0};
  uint8_t data[256]{0};
};

}  // namespace packet

#endif  // PACKET_H_
