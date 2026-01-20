// main.cpp - Mbed Studio용 PTP-like Serial Protocol (0xAA 0x55 헤더)

#include "mbed.h"

#include "crc8_table.h"

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
  Packet current_packet_;
  uint8_t packet_data_[255];
  bool packet_ready_;
  uint64_t local_timestamp_at_header1_;

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
          //   printf("[PARSER] Header1 (0xAA) found\n");
        }
        break;

      case WAIT_HEADER2:
        if (byte == PKT_HEADER_2) {
          rx_buffer[rx_index_++] = byte;
          state = WAIT_TIMESTAMP;
          //   printf("[PARSER] Header2 (0x55) found\n");
        } else {
          //   printf("[PARSER] Invalid Header2: 0x%02X, resetting\n", byte);
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
          //   printf("[PARSER] Timestamp complete\n");
        }
        break;

      case WAIT_ID:
        rx_buffer[rx_index_++] = byte;
        if (rx_index_ >= 11) {
          state = WAIT_SEQ;
          //   printf("[PARSER] ID=0x%02X\n", byte);
        }
        break;

      case WAIT_SEQ:
        rx_buffer[rx_index_++] = byte;
        if (rx_index_ >= 12) {
          state = WAIT_LEN;
          //   printf("[PARSER] SEQ=%d\n", byte);
        }
        break;
      case WAIT_LEN:
        rx_buffer[rx_index_++] = byte;
        memcpy(&current_packet_.timestamp_ns, rx_buffer + 2, 8);
        current_packet_.id = rx_buffer[10];
        current_packet_.seq = rx_buffer[11];
        current_packet_.len = byte;
        current_packet_.local_timestamp_ns = local_timestamp_at_header1_;
        // printf("[PARSER] LEN=%d, ID=0x%02X, SEQ=%d\n", byte,
        // current_packet_.id,
        //    current_packet_.seq);
        if (current_packet_.len == 0)
          state = WAIT_CRC;
        else
          state = WAIT_DATA;
        break;

      case WAIT_DATA:
        packet_data_[rx_index_ - 13] = byte;
        rx_buffer[rx_index_++] = byte;
        // printf("[PARSER] DATA[%zu]=0x%02X\n", rx_index_ - 14, byte);
        if (rx_index_ >= 13 + current_packet_.len) {
          //   printf("[PARSER] Data complete\n");
          state = WAIT_CRC;
        }
        break;

      case WAIT_CRC: {
        rx_buffer[rx_index_++] = byte;
        uint8_t calc_crc = ComputeCRC8(rx_buffer, rx_index_ - 1);
        // printf("[PARSER] CRC: calc=0x%02X, recv=0x%02X\n", calc_crc, byte);
        if (calc_crc == byte) {
          packet_ready_ = true;
          //   printf("Packet ready!");
          //   printf("[PKT] ID=0x%02X, seq=%d, len=%d, CRC OK\n",
          //          current_packet_.id, current_packet_.seq,
          //          current_packet_.len);
          rx_index_ = 0;
          state = WAIT_HEADER1;
        } else {
          //   printf("[PKT] CRC FAIL\n");
          rx_index_ = 0;
          state = WAIT_HEADER1;
        }
      } break;
    }
  }
};

// ========== 글로벌 변수 ==========
DigitalOut led(LED1);
UnbufferedSerial serial(USBTX, USBRX, 460800);
Timer timer;
PacketParser parser;
uint8_t seq_counter = 0;

struct RxByte {
  uint8_t data;
  uint64_t timestamp;
};

Mail<RxByte, 512> rx_mail;

uint64_t t1_from_host = 0;
uint64_t t2_at_local = 0;
uint64_t t3_at_local = 0;
uint64_t t4_from_host = 0;
uint64_t last_send_timestamp = 0;
int64_t clock_offset = 0;
uint64_t path_delay = 0;

// ========== 패킷 송신 ==========
void SendPacket(uint8_t msg_id, const uint8_t* data, uint8_t data_len) {
  uint8_t buffer[269];
  size_t offset = 0;

  buffer[offset++] = PKT_HEADER_1;
  buffer[offset++] = PKT_HEADER_2;

  uint64_t tx_timestamp = timer.elapsed_time().count() * 1000ULL;
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
  last_send_timestamp = timer.elapsed_time().count() * 1000ULL;
  serial.write(buffer, offset);
  //   printf("[TX] Sent %zu bytes, ID=0x%02X, seq=%d\n", offset, msg_id, seq);
}

void SendSync() { SendPacket(kPTPSync, NULL, 0); }

void SendDelayReq() {
  // t3_at_local = timer.elapsed_time().count() * 1000ULL;
  SendPacket(kPTPDelayReq, NULL, 0);
  t3_at_local = last_send_timestamp;
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
  led = !led;
  SendPacket(kEchoMsg, parser.packet_data_, pkt->len);
  led = !led;
}

void HandleSync(Packet* pkt) {
  led = !led;
  t1_from_host = pkt->timestamp_ns;
  t2_at_local = parser.local_timestamp_at_header1_;
  SendDelayReq();
}

void HandleDelayResp(Packet* pkt) {
  // uint8_t request_seq = parser.packet_data_[0];
  led = !led;
  uint64_t t4_from_host;
  t4_from_host = pkt->timestamp_ns;

  uint64_t forward_delay = t2_at_local - t1_from_host;
  uint64_t reverse_delay = t4_from_host - t3_at_local;
  clock_offset = (static_cast<int64_t>(forward_delay) -
                  static_cast<int64_t>(reverse_delay)) /
                 2;
  path_delay = (forward_delay + reverse_delay) / 2;
  // SendPTPReportSlaveToMaster(clock_offset, path_delay);
  SendPTPReportSlaveToMaster(t1_from_host, t2_at_local, t3_at_local,
                             t4_from_host);
  led = !led;
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
  while (1) {
    osEvent evt = rx_mail.get(1);

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
    if (loop_count % 500 == 0) {
      led = !led;
      //   printf("[HEARTBEAT] Alive, loop=%lu\n", loop_count);
      SendPacket(kHeartBeatMsg, NULL, 0);
    }

    ThisThread::sleep_for(1ms);
  }
}
