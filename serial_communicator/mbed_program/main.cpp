// main.cpp - Mbed Studio용 PTP-like Serial Protocol (0xAA 0x55 헤더)

#include "mbed.h"

// ========== 패킷 정의 ==========
#define PKT_HEADER_1 0xAA
#define PKT_HEADER_2 0x55

enum MessageType {
  ECHO_MSG = 0x01,
  HEART_BEAT_MSG = 0x02,
  PTP_SYNC = 0x10,
  PTP_DELAY_REQ = 0x11,
  PTP_DELAY_RESP = 0x12,
  IMU_DATA = 0x20,
};

struct __attribute__((packed)) Packet {
  uint8_t header1;     // 0xAA
  uint8_t header2;     // 0x55
  uint64_t timestamp;  // 나노초
  uint8_t id;          // MessageType
  uint8_t seq;         // 시퀀스 번호
  uint8_t len;         // data 길이 (0-255)
};

// ========== CRC8 Lookup Table ==========
static const uint8_t crc8_table[256] = {
    0x00, 0x31, 0x62, 0x53, 0xC4, 0xF5, 0xA6, 0x97, 0xB9, 0x88, 0xDB, 0xEA,
    0x7D, 0x4C, 0x1F, 0x2E, 0x43, 0x72, 0x21, 0x10, 0x87, 0xB6, 0xE5, 0xD4,
    0xFA, 0xCB, 0x98, 0xA9, 0x3E, 0x0F, 0x5C, 0x6D, 0x86, 0xB7, 0xE4, 0xD5,
    0x42, 0x73, 0x20, 0x11, 0x3F, 0x0E, 0x5D, 0x6C, 0xFB, 0xCA, 0x99, 0xA8,
    0xC5, 0xF4, 0xA7, 0x96, 0x01, 0x30, 0x63, 0x52, 0x7C, 0x4D, 0x1E, 0x2F,
    0xB8, 0x89, 0xDA, 0xEB, 0x3D, 0x0C, 0x5F, 0x6E, 0xF9, 0xC8, 0x9B, 0xAA,
    0x84, 0xB5, 0xE6, 0xD7, 0x40, 0x71, 0x22, 0x13, 0x7E, 0x4F, 0x1C, 0x2D,
    0xBA, 0x8B, 0xD8, 0xE9, 0xC7, 0xF6, 0xA5, 0x94, 0x03, 0x32, 0x61, 0x50,
    0xBB, 0x8A, 0xD9, 0xE8, 0x7F, 0x4E, 0x1D, 0x2C, 0x02, 0x33, 0x60, 0x51,
    0xC6, 0xF7, 0xA4, 0x95, 0xF8, 0xC9, 0x9A, 0xAB, 0x3C, 0x0D, 0x5E, 0x6F,
    0x41, 0x70, 0x23, 0x12, 0x85, 0xB4, 0xE7, 0xD6, 0x7A, 0x4B, 0x18, 0x29,
    0xBE, 0x8F, 0xDC, 0xED, 0xC3, 0xF2, 0xA1, 0x90, 0x07, 0x36, 0x65, 0x54,
    0x39, 0x08, 0x5B, 0x6A, 0xFD, 0xCC, 0x9F, 0xAE, 0x80, 0xB1, 0xE2, 0xD3,
    0x44, 0x75, 0x26, 0x17, 0xFC, 0xCD, 0x9E, 0xAF, 0x38, 0x09, 0x5A, 0x6B,
    0x45, 0x74, 0x27, 0x16, 0x81, 0xB0, 0xE3, 0xD2, 0xBF, 0x8E, 0xDD, 0xEC,
    0x7B, 0x4A, 0x19, 0x28, 0x06, 0x37, 0x64, 0x55, 0xC2, 0xF3, 0xA0, 0x91,
    0x47, 0x76, 0x25, 0x14, 0x83, 0xB2, 0xE1, 0xD0, 0xFE, 0xCF, 0x9C, 0xAD,
    0x3A, 0x0B, 0x58, 0x69, 0x04, 0x35, 0x66, 0x57, 0xC0, 0xF1, 0xA2, 0x93,
    0xBD, 0x8C, 0xDF, 0xEE, 0x79, 0x48, 0x1B, 0x2A, 0xC1, 0xF0, 0xA3, 0x92,
    0x05, 0x34, 0x67, 0x56, 0x78, 0x49, 0x1A, 0x2B, 0xBC, 0x8D, 0xDE, 0xEF,
    0x82, 0xB3, 0xE0, 0xD1, 0x46, 0x77, 0x24, 0x15, 0x3B, 0x0A, 0x59, 0x68,
    0xFF, 0xCE, 0x9D, 0xAC};

uint8_t ComputeCRC8(const uint8_t* data, size_t len) {
  uint8_t crc = 0;
  for (size_t i = 0; i < len; i++) crc = crc8_table[crc ^ data[i]];
  return crc;
}

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
  uint64_t header_timestamp_;

  PacketParser() { Reset(); }

  void Reset() {
    state = WAIT_HEADER1;
    rx_index_ = 0;
    packet_ready_ = false;
  }

  void FeedByte(uint8_t byte, uint64_t timestamp) {
    switch (state) {
      case WAIT_HEADER1:
        if (byte == PKT_HEADER_1) {
          header_timestamp_ = timestamp;
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
            header_timestamp_ = timestamp;
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
        memcpy(&current_packet_.timestamp, rx_buffer + 2, 8);
        current_packet_.id = rx_buffer[10];
        current_packet_.seq = rx_buffer[11];
        current_packet_.len = byte;
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
          current_packet_.timestamp = header_timestamp_;
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

uint64_t last_sync_t1 = 0;
uint64_t last_sync_t2 = 0;
uint64_t last_delay_req_tx_time = 0;
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

  serial.write(buffer, offset);
  //   printf("[TX] Sent %zu bytes, ID=0x%02X, seq=%d\n", offset, msg_id, seq);
}

void SendSync() { SendPacket(PTP_SYNC, NULL, 0); }

void SendDelayReq() {
  last_delay_req_tx_time = timer.elapsed_time().count() * 1000ULL;
  SendPacket(PTP_DELAY_REQ, NULL, 0);
}

void SendDelayResp(uint8_t req_seq, uint64_t t2) {
  uint8_t data[9];
  data[0] = req_seq;
  memcpy(data + 1, &t2, 8);
  SendPacket(PTP_DELAY_RESP, data, 9);
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
  printf("ECHO received: len=%d\n", pkt->len);
  SendPacket(ECHO_MSG, parser.packet_data_, pkt->len);
  led = !led;
}

void HandleSync(Packet* pkt) {
  led = !led;
  last_sync_t1 = pkt->timestamp;
  last_sync_t2 = parser.header_timestamp_;
  printf("\n[SYNC] T1=%llu, T2=%llu\n", last_sync_t1, last_sync_t2);
  SendDelayReq();
}

void HandleDelayResp(Packet* pkt) {
  uint8_t req_seq = parser.packet_data_[0];
  uint64_t t2_master, t4;
  memcpy(&t2_master, parser.packet_data_ + 1, 8);
  t4 = pkt->timestamp;

  int64_t forward_delay = (int64_t)last_sync_t2 - (int64_t)last_sync_t1;
  int64_t reverse_delay = (int64_t)t4 - (int64_t)last_delay_req_tx_time;
  clock_offset = (forward_delay - reverse_delay) / 2;
  path_delay = (forward_delay + reverse_delay) / 2;

  printf("[DELAY_RESP] offset=%lld ns, delay=%llu ns\n", clock_offset,
         path_delay);
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
          case ECHO_MSG:
            HandleEcho(&parser.current_packet_);
            break;
          case PTP_SYNC:
            HandleSync(&parser.current_packet_);
            break;
          case PTP_DELAY_RESP:
            HandleDelayResp(&parser.current_packet_);
            break;
          default:
            printf("Unknown ID: 0x%02X\n", parser.current_packet_.id);
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
      SendPacket(HEART_BEAT_MSG, NULL, 0);
    }

    ThisThread::sleep_for(1ms);
  }
}
