// main.cpp - Mbed Studio용 PTP-like Serial Protocol (0xAA 0x55 헤더)

#include "mbed.h"

#include "crc8_table.h"

#define SERIAL_BAUDRATE 921600

// ========== 패킷 정의 ==========
#define PKT_HEADER_1 0xAA
#define PKT_HEADER_2 0x55

enum MessageType {
  kEchoMsg = 0x01,
  kHeartBeatMsg = 0x02,
  kPTPSync = 0x10,
  kPTPDelayReq = 0x11,
  kPTPDelayResp = 0x12,
  kPTPReportSlaveToMaster = 0x13,
  kImuData = 0x20,
};

struct __attribute__((packed)) Packet {
  uint64_t timestamp_ns;        // 패킷에 포함된 타임스탬프
  uint8_t id;                   // MessageType
  uint8_t seq;                  // 시퀀스 번호
  uint8_t len;                  // data 길이 (0-255)
  uint64_t local_timestamp_ns;  // 수신 시점 타임스탬프
};

// ========== RX 상태 머신 ==========
enum RxState {
  WAIT_HEADER1,
  WAIT_HEADER2,
  WAIT_TIMESTAMP,
  WAIT_ID,
  WAIT_SEQ,
  WAIT_LEN,
  WAIT_DATA,
  WAIT_CRC
};

class PacketParser {
 private:
  RxState state;
  uint8_t rx_buffer[269];
  size_t rx_index_;

 public:
  PacketParser() { Reset(); }

  void Reset() {
    state = WAIT_HEADER1;
    rx_index_ = 0;
    packet_ready_ = false;
  }

  void FeedByte(uint8_t byte, uint64_t local_timestamp) {
    switch (state) {
      case WAIT_HEADER1:
        if (byte == PKT_HEADER_1) {
          local_timestamp_at_header1_ = local_timestamp;
          rx_buffer[0] = byte;
          rx_index_ = 1;
          state = WAIT_HEADER2;
        }
        break;
      case WAIT_HEADER2:
        if (byte == PKT_HEADER_2) {
          rx_buffer[rx_index_++] = byte;
          state = WAIT_TIMESTAMP;
        } else {
          Reset();
          if (byte == PKT_HEADER_1) {
            local_timestamp_at_header1_ = local_timestamp;
            rx_buffer[0] = byte;
            rx_index_ = 1;
            state = WAIT_HEADER2;
          }
        }
        break;
      case WAIT_TIMESTAMP:
        rx_buffer[rx_index_++] = byte;
        if (rx_index_ >= 10) {
          state = WAIT_ID;
        }
        break;
      case WAIT_ID:
        rx_buffer[rx_index_++] = byte;
        if (rx_index_ >= 11) {
          state = WAIT_SEQ;
        }
        break;

      case WAIT_SEQ:
        rx_buffer[rx_index_++] = byte;
        if (rx_index_ >= 12) {
          state = WAIT_LEN;
        }
        break;
      case WAIT_LEN:
        rx_buffer[rx_index_++] = byte;
        memcpy(&current_packet_.timestamp_ns, rx_buffer + 2, 8);
        current_packet_.id = rx_buffer[10];
        current_packet_.seq = rx_buffer[11];
        current_packet_.len = byte;
        current_packet_.local_timestamp_ns = local_timestamp_at_header1_;
        if (current_packet_.len == 0)
          state = WAIT_CRC;
        else
          state = WAIT_DATA;
        break;

      case WAIT_DATA:
        packet_data_[rx_index_ - 13] = byte;
        rx_buffer[rx_index_++] = byte;
        if (rx_index_ >= 13 + current_packet_.len) {
          state = WAIT_CRC;
        }
        break;

      case WAIT_CRC: {
        rx_buffer[rx_index_++] = byte;
        uint8_t calc_crc = ComputeCRC8(rx_buffer, rx_index_ - 1);
        if (calc_crc == byte) {
          packet_ready_ = true;
          rx_index_ = 0;
          state = WAIT_HEADER1;
        } else {
          rx_index_ = 0;
          state = WAIT_HEADER1;
        }
      } break;
    }
  }

 public:
  Packet current_packet_;
  uint8_t packet_data_[255];
  bool packet_ready_;
  uint64_t local_timestamp_at_header1_;
};

// ========== 글로벌 변수 ==========
DigitalOut led(LED1);
UnbufferedSerial serial(USBTX, USBRX, SERIAL_BAUDRATE);
Timer timer;
PacketParser parser;
uint8_t seq_counter = 0;

struct RxByte {
  uint8_t data;
  uint64_t timestamp;
};

Mail<RxByte, 512> rx_mail;

volatile uint64_t t1_from_host = 0;
volatile uint64_t t2_at_local = 0;
volatile uint64_t t3_at_local = 0;
volatile uint64_t t4_from_host = 0;
volatile uint64_t last_send_timestamp_for_t3 = 0;
volatile int64_t clock_offset = 0;
volatile uint64_t path_delay = 0;

uint64_t GetLocalTime() { return timer.elapsed_time().count() * 1000ULL; }

uint64_t GetSynchronizedTime() {
  return static_cast<uint64_t>(static_cast<int64_t>(GetLocalTime()) -
                               clock_offset);
}

// ========== 패킷 송신 ==========
void SendPacket(uint8_t msg_id, const uint8_t* data, uint8_t data_len) {
  uint8_t buffer[269];
  size_t offset = 0;

  buffer[offset++] = PKT_HEADER_1;
  buffer[offset++] = PKT_HEADER_2;

  const uint64_t tx_timestamp = GetSynchronizedTime();
  memcpy(buffer + offset, &tx_timestamp, 8);
  offset += 8;
  buffer[offset++] = msg_id;
  uint8_t seq = seq_counter++;
  buffer[offset++] = seq;
  buffer[offset++] = data_len;

  if (data_len > 0) {
    memcpy(buffer + offset, data, data_len);
    offset += data_len;
  }

  uint8_t crc = ComputeCRC8(buffer, offset);
  buffer[offset++] = crc;
  last_send_timestamp_for_t3 = GetLocalTime();
  serial.write(buffer, offset);
}

void SendSync() { SendPacket(kPTPSync, NULL, 0); }

void SendDelayReq() {
  SendPacket(kPTPDelayReq, NULL, 0);
  t3_at_local = last_send_timestamp_for_t3;
}

void SendDelayResp(uint8_t req_seq, uint64_t t2) {
  uint8_t data[9];
  data[0] = req_seq;
  memcpy(data + 1, &t2, 8);
  SendPacket(kPTPDelayResp, data, 9);
}

void SendPTPReportSlaveToMaster(uint64_t t1, uint64_t t2, uint64_t t3,
                                uint64_t t4) {
  uint8_t data[32];
  memcpy(data, &t1, 8);
  memcpy(data + 8, &t2, 8);
  memcpy(data + 16, &t3, 8);
  memcpy(data + 24, &t4, 8);
  SendPacket(kPTPReportSlaveToMaster, data, 32);
}

// ========== RX 인터럽트 ==========
void HandleRxInterrupt() {
  while (serial.readable()) {
    uint8_t byte;
    if (serial.read(&byte, 1) == 1) {
      uint64_t ts = timer.elapsed_time().count() * 1000ULL;

      RxByte* mail = rx_mail.try_alloc();
      if (mail) {
        mail->data = byte;
        mail->timestamp = ts;
        rx_mail.put(mail);
      } else {
        printf("[ERROR] Mail alloc failed!\n");
      }
    }
  }
}

// ========== PTP 핸들러 ==========
void HandleEcho(Packet* pkt) {
  SendPacket(kEchoMsg, parser.packet_data_, pkt->len);
}

void HandleSync(Packet* pkt) {
  t1_from_host = pkt->timestamp_ns;
  t2_at_local = parser.local_timestamp_at_header1_;
  SendDelayReq();
}

void HandleDelayResp(Packet* pkt) {
  // uint8_t request_seq = parser.packet_data_[0];
  uint64_t t4_from_host;
  t4_from_host = pkt->timestamp_ns;

  uint64_t forward_delay = t2_at_local - t1_from_host;
  uint64_t reverse_delay = t4_from_host - t3_at_local;
  clock_offset = (static_cast<int64_t>(forward_delay) -
                  static_cast<int64_t>(reverse_delay)) /
                 2;
  path_delay = (forward_delay + reverse_delay) / 2;
  SendPTPReportSlaveToMaster(t1_from_host, t2_at_local, t3_at_local,
                             t4_from_host);
}

// ========== 메인 ==========
int main() {
  timer.start();
  serial.attach(callback(HandleRxInterrupt), SerialBase::RxIrq);

  ThisThread::sleep_for(200ms);
  printf("\n\n=================================\n");
  printf("PTP Protocol Started (Slave Mode)\n");
  printf("Header: 0xAA 0x55\n");
  printf("=================================\n\n");
  ThisThread::sleep_for(200ms);

  unsigned long loop_count = 0;
  uint64_t last_led_toggle = 0;  // 추가

  uint64_t last_led_set_true = 0;
  uint64_t last_led_set_false = 0;
  while (1) {
    osEvent evt = rx_mail.get(0);

    if (evt.status == osEventMail) {
      RxByte* rx_data = (RxByte*)evt.value.p;
      parser.FeedByte(rx_data->data, rx_data->timestamp);
      if (parser.packet_ready_) {
        switch (parser.current_packet_.id) {
          case kEchoMsg:
            HandleEcho(&parser.current_packet_);
            break;
          case kPTPSync:
            HandleSync(&parser.current_packet_);
            break;
          case kPTPDelayResp:
            HandleDelayResp(&parser.current_packet_);
            break;
          default:
            break;
        }
        parser.packet_ready_ = false;
      }

      rx_mail.free(rx_data);
    }

    loop_count++;
    if (loop_count % 5000 == 0) {
      SendPacket(kHeartBeatMsg, NULL, 0);
    }

    wait_us(100);
  }
}
