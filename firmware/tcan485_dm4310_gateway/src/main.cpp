#include <Arduino.h>
#include "driver/twai.h"

// LILYGO T-CAN485 V1.0 official pin map.
static constexpr gpio_num_t PIN_CAN_TX = GPIO_NUM_27;
static constexpr gpio_num_t PIN_CAN_RX = GPIO_NUM_26;
static constexpr uint8_t PIN_CAN_SPEED = 23;
static constexpr uint8_t PIN_BOOST_ENABLE = 16;
static constexpr uint8_t PIN_RS485_TX = 22;
static constexpr uint8_t PIN_RS485_RX = 21;
static constexpr uint8_t PIN_RS485_RE = 17;
static constexpr uint8_t PIN_RS485_ENABLE = 19;

static constexpr uint32_t USB_BAUD = 115200;
static constexpr uint32_t RS485_BAUD = 115200;

// Safety invariant for the first bench phase: this build cannot transmit CAN.
static constexpr bool CAN_TRANSMIT_COMPILED = false;

HardwareSerial RS485(1);

static void print_hex_byte(uint8_t value) {
  if (value < 0x10) Serial.print('0');
  Serial.print(value, HEX);
}

static void initialize_can_listen_only() {
  pinMode(PIN_BOOST_ENABLE, OUTPUT);
  digitalWrite(PIN_BOOST_ENABLE, HIGH);

  pinMode(PIN_CAN_SPEED, OUTPUT);
  digitalWrite(PIN_CAN_SPEED, LOW);  // Official example: high-speed transceiver mode.

  twai_general_config_t general = TWAI_GENERAL_CONFIG_DEFAULT(
      PIN_CAN_TX, PIN_CAN_RX, TWAI_MODE_LISTEN_ONLY);
  general.tx_queue_len = 0;
  general.rx_queue_len = 32;
  twai_timing_config_t timing = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  esp_err_t install_result = twai_driver_install(&general, &timing, &filter);
  Serial.printf("CAN install=%s mode=LISTEN_ONLY bitrate=1000000 tx_compiled=%s\n",
                esp_err_to_name(install_result),
                CAN_TRANSMIT_COMPILED ? "true" : "false");
  if (install_result == ESP_OK) {
    Serial.printf("CAN start=%s\n", esp_err_to_name(twai_start()));
  }
}

static void initialize_rs485_receive() {
  pinMode(PIN_RS485_ENABLE, OUTPUT);
  pinMode(PIN_RS485_RE, OUTPUT);
  digitalWrite(PIN_RS485_ENABLE, HIGH);
  digitalWrite(PIN_RS485_RE, HIGH);
  RS485.begin(RS485_BAUD, SERIAL_8N1, PIN_RS485_RX, PIN_RS485_TX);
  Serial.printf("RS485 receive monitor started: %lu 8N1 RX=%u TX=%u\n",
                static_cast<unsigned long>(RS485_BAUD), PIN_RS485_RX, PIN_RS485_TX);
}

static void poll_can() {
  twai_message_t frame{};
  while (twai_receive(&frame, 0) == ESP_OK) {
    Serial.printf("CAN_RX id=0x%03lX ext=%u rtr=%u dlc=%u data=",
                  static_cast<unsigned long>(frame.identifier), frame.extd,
                  frame.rtr, frame.data_length_code);
    for (uint8_t i = 0; i < frame.data_length_code; ++i) {
      print_hex_byte(frame.data[i]);
      if (i + 1 < frame.data_length_code) Serial.print(' ');
    }
    Serial.println();
  }
}

static void poll_rs485() {
  if (!RS485.available()) return;
  Serial.print("RS485_RX data=");
  uint32_t quiet_deadline = millis() + 5;
  bool first = true;
  while (static_cast<int32_t>(quiet_deadline - millis()) > 0) {
    while (RS485.available()) {
      uint8_t value = static_cast<uint8_t>(RS485.read());
      if (!first) Serial.print(' ');
      print_hex_byte(value);
      first = false;
      quiet_deadline = millis() + 5;
    }
  }
  Serial.println();
}

void setup() {
  Serial.begin(USB_BAUD);
  delay(500);
  Serial.println();
  Serial.println("T-CAN485 DM4310 gateway audit build 0.1");
  initialize_can_listen_only();
  initialize_rs485_receive();
  Serial.println("SAFE_MODE: CAN listen-only; RS485 receive-only application logic");
}

void loop() {
  poll_can();
  poll_rs485();
  delay(1);
}
