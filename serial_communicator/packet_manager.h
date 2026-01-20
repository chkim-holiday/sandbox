#ifndef MESSAGE_DISPATCHER_H_
#define MESSAGE_DISPATCHER_H_

#include "packet_parser.h"
#include "packet_serializer.h"
#include "serial_communicator.h"
#include "timer.h"

uint64_t GetLocalTime() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now().time_since_epoch())
          .count());
}

template <typename T>
union ByteConverter {
  ByteConverter() {}
  ByteConverter(const T& val) : value(val) {}
  ByteConverter(uint8_t* bytes_array) {
    if (sizeof(bytes_array) != sizeof(T))
      throw std::runtime_error("Byte array size does not match type size.");
    if (bytes_array == nullptr)
      throw std::runtime_error("Null byte array provided.");
    std::memcpy(bytes, bytes_array, sizeof(T));
  }

  T value;
  uint8_t bytes[sizeof(T)];
};

class PacketSubscriber {
 public:
  virtual ~PacketSubscriber() = default;
  virtual void HandlePacket(const packet::Packet& packet) = 0;
};

class PacketPublisher {
 public:
  PacketPublisher(serial_communicator::SerialCommunicator& comm)
      : comm_(comm) {}
  virtual ~PacketPublisher() = default;
  virtual void PublishPacket(const packet::MessageType id, const uint8_t seq,
                             const std::string& data) = 0;

 protected:
  serial_communicator::SerialCommunicator& comm_;
};

class CommandMessageProcessor : public PacketSubscriber,
                                public PacketPublisher {
 public:
  using Callback = std::function<void(const std::string& command)>;

  CommandMessageProcessor(serial_communicator::SerialCommunicator& comm)
      : PacketPublisher(comm) {}

  void PublishPacket(const packet::MessageType id, const uint8_t seq,
                     const std::string& data) override {
    const uint8_t* data_ptr = reinterpret_cast<const uint8_t*>(data.data());
    const uint8_t data_len = static_cast<uint8_t>(data.size());
    const uint64_t timestamp_ns = GetLocalTime();
    const auto serialized_packet = packet::PacketSerializer::SerializePacket(
        timestamp_ns, id, seq, data_ptr, data_len);
    const size_t written =
        comm_.Write(reinterpret_cast<const uint8_t*>(serialized_packet.data()),
                    serialized_packet.size());
    (void)written;
  }

  void HandlePacket(const packet::Packet& packet) override {
    // 명령어 메시지 형태: ASCII 문자열
    std::string command(reinterpret_cast<const char*>(packet.data),
                        packet.length);
    if (callback_) callback_(command);
  }

  void RegisterCallback(Callback callback) { callback_ = callback; }

 private:
  Callback callback_;
};

class DebugMessageReceiver : public PacketSubscriber {
 public:
  using Callback = std::function<void(const std::string& message)>;

  void HandlePacket(const packet::Packet& packet) override {
    // 디버그 메시지 형태: ASCII 문자열
    std::string message(reinterpret_cast<const char*>(packet.data),
                        packet.length);
    if (callback_) callback_(message);
  }

  void RegisterCallback(Callback callback) { callback_ = callback; }

 private:
  Callback callback_;
};

class IMUMessageReceiver : public PacketSubscriber {
 public:
  struct ImuData {
    uint64_t timestamp_ns{0};
    int16_t ax{0};
    int16_t ay{0};
    int16_t az{0};
    int16_t gx{0};
    int16_t gy{0};
    int16_t gz{0};
    int16_t temperature{0};
  };

  using Callback = std::function<void(const ImuData& imu_data)>;

  void HandlePacket(const packet::Packet& packet) override {
    // IMU 데이터 형태
    // ax_high, ax_low, ay_high, ay_low, az_high, az_low,
    // gx_high, gx_low, gy_high, gy_low, gz_high, gz_low,
    // temp_high, temp_low (total 14 bytes)
    // Timestamp는 헤더꺼 쓰자.
    constexpr int kExpectedDataLength{14};
    if (packet.length != kExpectedDataLength) {
      std::cerr << "Invalid IMU data length: "
                << static_cast<int>(packet.length)
                << ", expected: " << kExpectedDataLength << std::endl;
      return;
    }
    ImuData imu_data;
    imu_data.timestamp_ns = packet.timestamp_ns;
    imu_data.ax = (packet.data[0] << 8) | packet.data[1];
    imu_data.ay = (packet.data[2] << 8) | packet.data[3];
    imu_data.az = (packet.data[4] << 8) | packet.data[5];
    imu_data.gx = (packet.data[6] << 8) | packet.data[7];
    imu_data.gy = (packet.data[8] << 8) | packet.data[9];
    imu_data.gz = (packet.data[10] << 8) | packet.data[11];
    imu_data.temperature = (packet.data[12] << 8) | packet.data[13];
    if (callback_) callback_(imu_data);
  }

  void RegisterCallback(Callback callback) { callback_ = callback; }

 private:
  Callback callback_;
};

class SerialPTPv2Server : public PacketSubscriber {
 public:
  SerialPTPv2Server(serial_communicator::SerialCommunicator& comm,
                    const int sync_period_in_ms, timer::Timer& sync_timer)
      : comm_(comm),
        sync_period_in_ms_(sync_period_in_ms),
        sync_timer_(sync_timer) {
    sync_timer_.Start(sync_period_in_ms_, [this]() { SendSync(); });
  }

  void HandlePacket(const packet::Packet& packet) override {
    switch (packet.id) {
      case packet::MessageType::kPTPDelayRequest: {
        HandleDelayRequest(packet);
      } break;
      case packet::MessageType::kPTPReportSlaveToMaster: {
        HandlePTPReportSlaveToMaster(packet);
      } break;
      case packet::MessageType::kPTPSync:
      case packet::MessageType::kPTPDelayResponse: {
        std::cerr << "PTP `SYNC` and `DELAY_RESPONSE` requests are not handled "
                     "in server."
                  << std::endl;
      } break;
      default: {
        std::cerr << "Unknown PTP message ID: " << static_cast<int>(packet.id)
                  << std::endl;
      } break;
    }
  }

 private:
  void SendSync() {
    std::cerr << "[Host]->[Slave]: Sending SYNC packet." << std::endl;
    static uint8_t sync_seq = 0;

    ByteConverter<uint64_t> current_timestamp = GetLocalTime();
    const auto serialized_packet = packet::PacketSerializer::SerializePacket(
        current_timestamp.value, packet::MessageType::kPTPSync, sync_seq++,
        current_timestamp.bytes, 8);
    const size_t written =
        comm_.Write(reinterpret_cast<const uint8_t*>(serialized_packet.data()),
                    serialized_packet.size());
    (void)written;
  }

  void HandleDelayRequest(const packet::Packet&) {
    std::cout << "[Slave]->[Host]: Received DELAY_REQ message." << std::endl;
    SendDelayResponse();
  }

  void HandlePTPReportSlaveToMaster(const packet::Packet& packet) {
    std::cout << "[Slave]->[Host]: Received PTP REPORT_SLAVE_TO_MASTER message."
              << std::endl;
    int64_t offset = 0;
    uint64_t delay = 0;
    std::memcpy(&offset, packet.data, 8);
    std::memcpy(&delay, packet.data + 8, 8);
    double offset_in_ms = static_cast<double>(offset) / 1e6;
    double delay_in_ms = static_cast<double>(delay) / 1e6;
    std::cout << "   - Reported offset: " << offset_in_ms << " ms" << std::endl;
    std::cout << "   - Reported path delay: " << delay_in_ms << " ms"
              << std::endl;
  }

  void SendDelayResponse() {
    ByteConverter<uint64_t> current_timestamp = GetLocalTime();
    std::cout << "[Host]->[Slave]: Sending DELAY_RESPONSE T4="
              << current_timestamp.value << " ns " << std::endl;

    const auto serialized_packet = packet::PacketSerializer::SerializePacket(
        current_timestamp.value, packet::MessageType::kPTPDelayResponse, 0,
        current_timestamp.bytes, 8);
    const size_t written =
        comm_.Write(reinterpret_cast<const uint8_t*>(serialized_packet.data()),
                    serialized_packet.size());
    (void)written;
  }

  serial_communicator::SerialCommunicator& comm_;
  int sync_period_in_ms_;
  timer::Timer& sync_timer_;
};

class PacketSubscriberManager {
 public:
  PacketSubscriberManager(packet::PacketParser& parser) : parser_(parser) {
    parser_.SetPacketCallback(
        [this](const packet::Packet& packet) { DispatchPacket(packet); });
  }

  void RegisterSubscriber(packet::MessageType message_id,
                          PacketSubscriber* subscriber) {
    subscribers_[message_id] = subscriber;
  }

 private:
  void DispatchPacket(const packet::Packet& packet) {
    auto it = subscribers_.find(static_cast<packet::MessageType>(packet.id));
    if (it != subscribers_.end()) {
      it->second->HandlePacket(packet);
    } else {
      std::cerr << "No subscriber registered for message ID: "
                << static_cast<int>(packet.id) << std::endl;
    }
  }

  packet::PacketParser& parser_;
  std::unordered_map<packet::MessageType, PacketSubscriber*> subscribers_;
};

#endif  // MESSAGE_DISPATCHER_H_
