#ifndef PACKET_SERIALIZER_H_
#define PACKET_SERIALIZER_H_

#include <cstdint>
#include <cstring>
#include <string>

#include "crc8_table.h"
#include "packet.h"

namespace packet {

class PacketSerializer {
 public:
  static std::string SerializePacket(const uint64_t timestamp_ns,
                                     const MessageType& id, uint8_t seq,
                                     const uint8_t* data, uint8_t data_length) {
    constexpr size_t kPacketOverhead{14};  // 14 bytes overhead
    int total_length = kPacketOverhead + data_length;
    std::string serialized_packet;
    serialized_packet.resize(total_length);
    uint8_t* ptr = reinterpret_cast<uint8_t*>(serialized_packet.data());
    int idx = 0;
    ptr[idx++] = PKT_HEADER_1;
    ptr[idx++] = PKT_HEADER_2;
    std::memcpy(&ptr[idx], &timestamp_ns, sizeof(timestamp_ns));
    idx += sizeof(timestamp_ns);
    ptr[idx++] = static_cast<uint8_t>(id);
    ptr[idx++] = seq;
    ptr[idx++] = data_length;
    if (data_length > 0 && data != nullptr) {
      std::memcpy(&ptr[idx], data, data_length);
      idx += data_length;
    }
    ptr[idx] = ComputeCRC8(ptr, idx);
    return serialized_packet;
  }
};

}  // namespace packet

#endif  // PACKET_SERIALIZER_H_
