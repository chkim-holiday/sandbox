// main.cpp - Mbed Studio용 PTP-like Serial Protocol (0xAA 0x55 헤더)

#include "mbed.h"

#include "crc8_table.h"

#define SERIAL_BAUDRATE 460800

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
DigitalOut trg(D8);
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

  bool sync_detected = false;
  uint64_t current_time = GetLocalTime();
  uint64_t next_trigger_time = (current_time / 200000000ULL) * 200000000ULL;
  if (current_time % 200000000ULL != 0) {
    next_trigger_time += 200000000ULL;
  }

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

    uint64_t now = GetSynchronizedTime();
    if (!sync_detected && now >= next_trigger_time + 400000000ULL) {
      sync_detected = true;
      next_trigger_time = (now / 50000000ULL) * 50000000ULL;
      if (now % 50000000ULL != 0) next_trigger_time += 50000000ULL;
    }
    now = GetSynchronizedTime();
    if (now >= next_trigger_time) {
      led = !led;                        // GPIO 핀 상태 토글
      trg = !trg;                        // 트리거 핀 상태 토글
      next_trigger_time += 50000000ULL;  // 다음 200ms 배수로 갱신
      loop_count++;
      if (loop_count == 20) {
        loop_count = 0;
        SendPacket(kHeartBeatMsg, NULL, 0);
      }
    }

    wait_us(5);
  }
}

// main_optimized.cpp - PTP-like Serial Protocol 최적화 버전
/*
#include "crc8_table.h"
#include "mbed.h"

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

// ========== Circular Buffer for RX ==========
#define RX_BUF_SIZE 512
struct {
  uint8_t data[RX_BUF_SIZE];
  uint64_t timestamps[RX_BUF_SIZE];
  volatile uint32_t head;
  volatile uint32_t tail;
} rx_circ_buf = {0};

// ========== 글로벌 변수 ==========
DigitalOut led(LED1);
DigitalOut trg(D8);
UnbufferedSerial serial(USBTX, USBRX, SERIAL_BAUDRATE);
Timer timer;
PacketParser parser;
uint8_t seq_counter = 0;

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

// ========== 패킷 송신 (최적화) ==========
void SendPacket(uint8_t msg_id, const uint8_t* data, uint8_t data_len) {
  uint8_t buffer[269];
  size_t offset = 0;

  buffer[offset++] = PKT_HEADER_1;
  buffer[offset++] = PKT_HEADER_2;

  // 송신 직전에 타임스탬프 캡처 - 정확도 향상
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

  // t3 타임스탬프도 송신 직전에 캡처
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

// ========== RX 인터럽트 (최적화) ==========
void HandleRxInterrupt() {
  while (serial.readable()) {
    // 타임스탬프를 먼저 캡처 - 정확도 향상
    uint64_t ts = timer.elapsed_time().count() * 1000ULL;
    uint8_t byte;

    if (serial.read(&byte, 1) == 1) {
      uint32_t next_head = (rx_circ_buf.head + 1) % RX_BUF_SIZE;
      if (next_head != rx_circ_buf.tail) {
        rx_circ_buf.data[rx_circ_buf.head] = byte;
        rx_circ_buf.timestamps[rx_circ_buf.head] = ts;
        rx_circ_buf.head = next_head;
      }
      // Buffer overflow시에도 조용히 무시 (Mail alloc 실패 로그 제거)
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

// ========== 메인 (최적화) ==========
int main() {
  timer.start();
  serial.attach(callback(HandleRxInterrupt), SerialBase::RxIrq);

  ThisThread::sleep_for(200ms);
  printf("\n\n=================================\n");
  printf("PTP Protocol Started (Optimized Slave Mode)\n");
  printf("Header: 0xAA 0x55\n");
  printf("Optimizations:\n");
  printf("  - Circular buffer instead of Mail\n");
  printf("  - Timestamp capture before read\n");
  printf("  - Removed polling delay\n");
  printf("=================================\n\n");
  ThisThread::sleep_for(200ms);

  unsigned long loop_count = 0;

  bool sync_detected = false;
  uint64_t current_time = GetLocalTime();
  uint64_t next_trigger_time = (current_time / 200000000ULL) * 200000000ULL;
  if (current_time % 200000000ULL != 0) {
    next_trigger_time += 200000000ULL;
  }

  while (1) {
    // Circular buffer에서 데이터 처리 - Mail보다 빠름
    while (rx_circ_buf.tail != rx_circ_buf.head) {
      uint8_t byte = rx_circ_buf.data[rx_circ_buf.tail];
      uint64_t ts = rx_circ_buf.timestamps[rx_circ_buf.tail];
      rx_circ_buf.tail = (rx_circ_buf.tail + 1) % RX_BUF_SIZE;

      parser.FeedByte(byte, ts);

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
    }

    uint64_t now = GetSynchronizedTime();
    if (!sync_detected && now >= next_trigger_time + 400000000ULL) {
      sync_detected = true;
      next_trigger_time = (now / 50000000ULL) * 50000000ULL;
      if (now % 50000000ULL != 0) next_trigger_time += 50000000ULL;
    }

    now = GetSynchronizedTime();
    if (now >= next_trigger_time) {
      led = !led;                        // GPIO 핀 상태 토글
      trg = !trg;                        // 트리거 핀 상태 토글
      next_trigger_time += 50000000ULL;  // 다음 50ms 배수로 갱신
      loop_count++;
      if (loop_count == 20) {
        loop_count = 0;
        SendPacket(kHeartBeatMsg, NULL, 0);
      }
    }

    // wait_us(5) 제거 - 불필요한 폴링 지연 제거
    // CPU 양보만 수행
    ThisThread::yield();
  }
}
  */