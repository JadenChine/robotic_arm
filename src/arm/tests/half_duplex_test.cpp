#include <algorithm>
#include <array>
#include <cassert>
#include <deque>
#include <iostream>
#include <string>
#include <vector>

#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "pico/time.h"
#include "drivers/servo_bus_driver.hpp"
#include "hal/servo_bus.h"
#include "platform/pico_uart_transport.hpp"
#include "platform/servo_platform_config.h"

namespace {
struct Pin {
  bool level = false;
  bool output = false;
  bool pullup = false;
  bool pulldown = false;
};
std::array<Pin, 30> pins;
std::vector<std::string> events;
std::vector<uint8_t> pendingTx;
std::vector<std::vector<uint8_t>> wirePackets;
std::deque<uint8_t> rx;
std::vector<uint8_t> reply;
bool halfDuplex = false;
bool failUartInit = false;
uint32_t irqState = 0;
uint64_t timeMs = 0;
uart_inst_t instances[2] = {{0}, {1}};

size_t eventIndex(const std::string& name) {
  const auto found = std::find(events.begin(), events.end(), name);
  assert(found != events.end());
  return static_cast<size_t>(found - events.begin());
}

void expectListening() {
  assert(!pins[2].level && pins[3].level);
  assert(pendingTx.empty());
  assert(irqState == 0);
}

void testTransport() {
  halfDuplex = true;
  arm::PicoUartTransport transport;
  const uint8_t packet[] = {0x55, 0x55, 1, 3, 28, 0xdf};
  transport.write(packet, sizeof(packet));
  transport.flush();
  assert(events.empty());
  assert(transport.init(0, 115200, 0, 1, 2, 3));
  expectListening();
  assert(pins[2].output && pins[3].output);
  assert(pins[1].pullup && !pins[1].pulldown);
  assert(eventIndex("gpio:2=0") < eventIndex("uart_init"));
  assert(eventIndex("gpio:3=0") < eventIndex("uart_init"));
  assert(eventIndex("uart_format") < eventIndex("gpio:3=1"));

  events.clear();
  transport.write(nullptr, 6);
  transport.write(packet, 0);
  assert(events.empty());

  transport.write(packet, sizeof(packet));
  expectListening();
  assert(eventIndex("gpio:3=0") < eventIndex("gpio:2=1"));
  assert(eventIndex("gpio:2=1") < eventIndex("uart_write"));
  assert(eventIndex("final_stop_bit") < eventIndex("gpio:2=0"));
  assert(eventIndex("gpio:2=0") < eventIndex("gpio:3=1"));
  assert(eventIndex("gpio:3=1") < eventIndex("irq_restore"));
  assert(wirePackets.back() == std::vector<uint8_t>(packet, packet + sizeof(packet)));

  // Preserve the caller's interrupt mask, even if it was already disabled.
  irqState = 1;
  transport.write(packet, sizeof(packet));
  assert(irqState == 1);
  irqState = 0;

  // Invalid mappings must not disturb a working interface.
  events.clear();
  assert(!transport.init(2, 115200, 0, 1, 2, 3));
  assert(!transport.init(1, 115200, 0, 1, 2, 3));
  assert(!transport.init(0, 115200, 1, 0, 2, 3));
  assert(!transport.init(0, 0, 0, 1, 2, 3));
  assert(!transport.init(0, 115200, 0, 1, 2, 2));
  assert(!transport.init(0, 115200, 0, 1, 0, 3));
  assert(!transport.init(0, 115200, 0, 1, 2, 1));
  assert(!transport.init(0, 115200, 0, 1, 23, 3));
  assert(!transport.init(0, 115200, 0, 1, 2));
  assert(!transport.init(0, 115200, 0, 1, arm::PicoUartTransport::kNoEnablePin, 3));
  assert(events.empty());
  expectListening();

  // Switching to the controller backend releases both old OEs.
  assert(transport.init(0, 9600, 0, 1));
  assert(!pins[2].level && !pins[3].level);
  assert(!pins[2].output && !pins[3].output);
  halfDuplex = false;
  events.clear();
  transport.write(packet, sizeof(packet));
  assert(!pendingTx.empty());  // Controller path retains write/flush semantics.
  transport.flush();
  assert(pendingTx.empty());
  assert(events == std::vector<std::string>({"uart_write", "uart_wait", "final_stop_bit"}));

  halfDuplex = true;
  failUartInit = true;
  assert(!transport.init(0, 115200, 0, 1, 2, 3));
  assert(!transport.initialized());
  assert(!pins[2].level && !pins[3].level);
  failUartInit = false;
}

void testHalAndDrivers() {
  servo_bus_config_t config;
  servo_bus_get_default_config(&config);
  assert(config.backend == SERVO_BUS_BACKEND_XARM_CONTROLLER);
  assert(config.baud_rate == 9600);
  assert(config.tx_pin == 0 && config.rx_pin == 1);
  assert(config.tx_enable_pin == 2 && config.rx_enable_pin == 3);
  halfDuplex = false;
  events.clear();
  assert(servo_bus_init(&config) == SERVO_BUS_STATUS_OK);
  assert(std::none_of(events.begin(), events.end(), [](const std::string& event) {
    return event.find("gpio:2=") == 0 || event.find("gpio:3=") == 0;
  }));
  assert(servo_bus_move_raw(1, 500, 700) == SERVO_BUS_STATUS_OK);
  assert(wirePackets.back() == std::vector<uint8_t>({0x55, 0x55, 8, 3, 1, 0xbc, 2, 1, 0xf4, 1}));

  config.backend = SERVO_BUS_BACKEND_DIRECT_SERVO;
  config.baud_rate = 0;  // Resolve to the backend's 115200-baud default.
  halfDuplex = true;
  assert(servo_bus_init(&config) == SERVO_BUS_STATUS_OK);
  assert(servo_bus_get_active_config()->baud_rate == 115200);
  expectListening();

  // Stale bytes are cleared BEFORE sending. A servo reply arriving at the
  // earliest listening opportunity must survive the driver's later flush().
  rx = {0xde, 0xad};
  reply = {0x55, 0x55, 1, 5, 27, 0xe8, 0x1c, 0xda};
  int16_t voltage = 0;
  assert(servo_bus_get_voltage(1, &voltage) == SERVO_BUS_STATUS_OK);
  assert(voltage == 7400);
  assert(wirePackets.back() == std::vector<uint8_t>({0x55, 0x55, 1, 3, 27, 0xe0}));
  assert(rx.empty());
  expectListening();

  // Every no-reply command releases the bus, including each queued move and
  // the final broadcast start used by the application's multi-servo path.
  const size_t beforeMoves = wirePackets.size();
  const uint8_t ids[] = {1, 2};
  const int16_t positions[] = {400, 600};
  assert(servo_bus_move_multiple(ids, positions, 2, 120) == SERVO_BUS_STATUS_OK);
  assert(wirePackets.size() == beforeMoves + 3);
  assert(wirePackets[beforeMoves][4] == arm::kServoMoveTimeWaitWrite);
  assert(wirePackets[beforeMoves + 1][4] == arm::kServoMoveTimeWaitWrite);
  assert(wirePackets.back()[2] == arm::kServoBroadcastId);
  assert(wirePackets.back()[4] == arm::kServoMoveStart);
  expectListening();

  assert(servo_bus_get_voltage(1, &voltage) == SERVO_BUS_STATUS_ERROR_TIMEOUT);
  expectListening();
  reply = {0x55, 0x55, 1, 5, 27, 0xe8, 0x1c, 0};  // Bad checksum.
  assert(servo_bus_get_voltage(1, &voltage) == SERVO_BUS_STATUS_ERROR_TIMEOUT);
  expectListening();

  config.tx_enable_pin = SERVO_BUS_HEARTBEAT_LED_PIN;
  assert(servo_bus_init(&config) == SERVO_BUS_STATUS_ERROR_INVALID_ARGUMENT);
  assert(servo_bus_get_active_config()->tx_enable_pin == 2);
  config.tx_enable_pin = arm::PicoUartTransport::kNoEnablePin;
  assert(servo_bus_init(&config) == SERVO_BUS_STATUS_ERROR_INVALID_ARGUMENT);
  expectListening();

  servo_bus_get_default_config(&config);
  assert(servo_bus_init(&config) == SERVO_BUS_STATUS_OK);
  assert(!pins[2].level && !pins[3].level);
  halfDuplex = false;
  assert(servo_bus_move_raw(1, 500, 700) == SERVO_BUS_STATUS_OK);
}

void testProtocolCorrections() {
  servo_bus_config_t config;
  servo_bus_get_default_config(&config);
  config.backend = SERVO_BUS_BACKEND_DIRECT_SERVO;
  config.baud_rate = 0;
  halfDuplex = true;
  assert(servo_bus_init(&config) == SERVO_BUS_STATUS_OK);

  // PDF LED polarity: 0 = on, 1 = off. Verify entire wire packets, including
  // the checksum, for both addressed and broadcast variants.
  assert(servo_bus_led_on(1) == SERVO_BUS_STATUS_OK);
  assert(wirePackets.back() == std::vector<uint8_t>({0x55, 0x55, 1, 4, 33, 0, 0xd9}));
  assert(servo_bus_led_off(1) == SERVO_BUS_STATUS_OK);
  assert(wirePackets.back() == std::vector<uint8_t>({0x55, 0x55, 1, 4, 33, 1, 0xd8}));
  assert(servo_bus_broadcast_led_on() == SERVO_BUS_STATUS_OK);
  assert(wirePackets.back() == std::vector<uint8_t>({0x55, 0x55, 254, 4, 33, 0, 0xdc}));
  assert(servo_bus_broadcast_led_off() == SERVO_BUS_STATUS_OK);
  assert(wirePackets.back() == std::vector<uint8_t>({0x55, 0x55, 254, 4, 33, 1, 0xdb}));

  // Discovery accepts the single connected servo's real response ID, while
  // addressed reads still reject another servo's otherwise valid packet.
  uint8_t id = 255;
  reply = {0x55, 0x55, 42, 4, 14, 42, 0x99};
  assert(servo_bus_get_id(254, &id) == SERVO_BUS_STATUS_OK && id == 42);
  assert(wirePackets.back() == std::vector<uint8_t>({0x55, 0x55, 254, 3, 14, 0xf0}));
  reply = {0x55, 0x55, 42, 4, 14, 42, 0x99};
  assert(servo_bus_get_id(1, &id) == SERVO_BUS_STATUS_ERROR_TIMEOUT);
  reply = {0x55, 0x55, 42, 4, 14, 42, 0};
  assert(servo_bus_get_id(254, &id) == SERVO_BUS_STATUS_ERROR_TIMEOUT);
  reply = {0x55, 0x55, 1, 4, 14, 1, 0xeb};
  assert(servo_bus_get_id(1, &id) == SERVO_BUS_STATUS_OK && id == 1);

  // All direct movement paths cap time at 30000 ms, keeping the existing
  // position clamping and centered degree mapping.
  assert(servo_bus_move_raw(1, 500, 65535) == SERVO_BUS_STATUS_OK);
  assert(wirePackets.back() == std::vector<uint8_t>({0x55, 0x55, 1, 7, 1, 0xf4, 1, 0x30, 0x75, 0x5c}));
  assert(servo_bus_move_wait_raw(1, 500, 30001) == SERVO_BUS_STATUS_OK);
  assert(wirePackets.back() == std::vector<uint8_t>({0x55, 0x55, 1, 7, 7, 0xf4, 1, 0x30, 0x75, 0x56}));
  assert(servo_bus_broadcast_move_raw(500, 30001) == SERVO_BUS_STATUS_OK);
  assert(wirePackets.back() == std::vector<uint8_t>({0x55, 0x55, 254, 7, 1, 0xf4, 1, 0x30, 0x75, 0x5f}));
  assert(servo_bus_move_raw(1, -10, 0) == SERVO_BUS_STATUS_OK);
  assert(wirePackets.back()[5] == 0 && wirePackets.back()[6] == 0);
  assert(wirePackets.back()[7] == 0 && wirePackets.back()[8] == 0);
  for (const float degrees : {-120.0f, 0.0f, 120.0f}) {
    assert(servo_bus_move_degrees(1, degrees, 120) == SERVO_BUS_STATUS_OK);
    const int position = wirePackets.back()[5] + (wirePackets.back()[6] << 8);
    assert(position == (degrees == -120.0f ? 0 : degrees == 0.0f ? 500 : 1000));
  }

  // Reject invalid requests before putting any bytes on the wire, and report
  // INVALID_ARGUMENT through the HAL instead of success or a later timeout.
  const size_t beforeInvalid = wirePackets.size();
  assert(servo_bus_set_id(1, 254) == SERVO_BUS_STATUS_ERROR_INVALID_ARGUMENT);
  assert(servo_bus_set_id(1, 255) == SERVO_BUS_STATUS_ERROR_INVALID_ARGUMENT);
  assert(servo_bus_set_id(255, 1) == SERVO_BUS_STATUS_ERROR_INVALID_ARGUMENT);
  assert(servo_bus_move_raw(255, 500, 120) == SERVO_BUS_STATUS_ERROR_INVALID_ARGUMENT);
  assert(servo_bus_led_on(255) == SERVO_BUS_STATUS_ERROR_INVALID_ARGUMENT);
  assert(servo_bus_get_id(255, &id) == SERVO_BUS_STATUS_ERROR_INVALID_ARGUMENT);
  int16_t voltage = 0;
  const uint64_t beforeRejectedRead = timeMs;
  assert(servo_bus_get_voltage(254, &voltage) == SERVO_BUS_STATUS_ERROR_INVALID_ARGUMENT);
  assert(timeMs == beforeRejectedRead);  // No 20 ms wait for a forbidden reply.
  assert(servo_bus_set_angle_limits_raw(1, 800, 200) == SERVO_BUS_STATUS_ERROR_INVALID_ARGUMENT);
  assert(servo_bus_set_angle_limits_raw(1, 500, 500) == SERVO_BUS_STATUS_ERROR_INVALID_ARGUMENT);
  assert(servo_bus_set_angle_limits_raw(1, -20, -10) == SERVO_BUS_STATUS_ERROR_INVALID_ARGUMENT);
  assert(servo_bus_set_angle_limits_degrees(1, 90, -90) == SERVO_BUS_STATUS_ERROR_INVALID_ARGUMENT);
  assert(servo_bus_set_voltage_limits(1, 8000, 6000) == SERVO_BUS_STATUS_ERROR_INVALID_ARGUMENT);
  assert(servo_bus_set_voltage_limits(1, 6000, 6000) == SERVO_BUS_STATUS_ERROR_INVALID_ARGUMENT);
  assert(servo_bus_set_voltage_limits(1, 13000, 14000) == SERVO_BUS_STATUS_ERROR_INVALID_ARGUMENT);
  assert(servo_bus_set_led_errors(1, 8) == SERVO_BUS_STATUS_ERROR_INVALID_ARGUMENT);
  assert(servo_bus_set_led_errors(1, 255) == SERVO_BUS_STATUS_ERROR_INVALID_ARGUMENT);
  const uint8_t invalidIds[] = {1, 254};
  const int16_t positions[] = {400, 600};
  assert(servo_bus_move_multiple(invalidIds, positions, 2, 120) == SERVO_BUS_STATUS_ERROR_INVALID_ARGUMENT);
  assert(wirePackets.size() == beforeInvalid);  // No partial queue or START.
  expectListening();

  // Valid boundary values still work and reset any preceding argument error.
  assert(servo_bus_set_id(1, 0) == SERVO_BUS_STATUS_OK);
  assert(wirePackets.back()[5] == 0);
  assert(servo_bus_set_id(1, 253) == SERVO_BUS_STATUS_OK);
  assert(wirePackets.back()[5] == 253);
  assert(servo_bus_set_angle_limits_degrees(1, -120, 120) == SERVO_BUS_STATUS_OK);
  assert(wirePackets.back()[5] == 0 && wirePackets.back()[6] == 0);
  assert(wirePackets.back()[7] == 0xe8 && wirePackets.back()[8] == 3);
  assert(servo_bus_set_voltage_limits(1, 4500, 12000) == SERVO_BUS_STATUS_OK);
  assert(servo_bus_set_led_errors(1, 0) == SERVO_BUS_STATUS_OK);
  assert(servo_bus_set_led_errors(1, 7) == SERVO_BUS_STATUS_OK);
  const uint8_t validIds[] = {0, 253};
  const size_t beforeMoves = wirePackets.size();
  assert(servo_bus_move_multiple(validIds, positions, 2, 65535) == SERVO_BUS_STATUS_OK);
  assert(wirePackets.size() == beforeMoves + 3);
  for (size_t index = beforeMoves; index < beforeMoves + 2; ++index) {
    assert(wirePackets[index][7] == 0x30 && wirePackets[index][8] == 0x75);
  }
  expectListening();

  servo_bus_get_default_config(&config);
  assert(servo_bus_init(&config) == SERVO_BUS_STATUS_OK);
  halfDuplex = false;
  // The direct protocol's time limit must not change the controller protocol.
  assert(servo_bus_move_raw(1, 500, 65535) == SERVO_BUS_STATUS_OK);
  assert(wirePackets.back()[5] == 0xff && wirePackets.back()[6] == 0xff);
}
}  // namespace

uart_inst_t* uart0 = &instances[0];
uart_inst_t* uart1 = &instances[1];

void gpio_init(unsigned pin) { pins.at(pin).output = false; pins.at(pin).level = false; }
void gpio_pull_down(unsigned pin) { pins.at(pin).pulldown = true; pins.at(pin).pullup = false; }
void gpio_pull_up(unsigned pin) { pins.at(pin).pullup = true; pins.at(pin).pulldown = false; }
void gpio_set_dir(unsigned pin, bool output) {
  if (halfDuplex && (pin == 2 || pin == 3) && output) assert(!pins.at(pin).level);
  pins.at(pin).output = output;
}
void gpio_set_function(unsigned, unsigned function) { assert(function == GPIO_FUNC_UART); }
void gpio_put(unsigned pin, bool level) {
  if (halfDuplex && pin == 2 && !level) assert(pendingTx.empty());
  pins.at(pin).level = level;
  events.push_back("gpio:" + std::to_string(pin) + "=" + (level ? "1" : "0"));
  if (halfDuplex) {
    assert(!(pins[2].level && pins[3].level));
    if (pin == 3 && level && !reply.empty()) {
      rx.insert(rx.end(), reply.begin(), reply.end());
      reply.clear();
    }
  }
}
unsigned uart_init(uart_inst_t*, unsigned baud) {
  if (halfDuplex) assert(!pins[2].level && !pins[3].level);
  events.push_back("uart_init");
  return failUartInit ? 0 : baud;
}
void uart_deinit(uart_inst_t*) { assert(pendingTx.empty()); }
void uart_set_hw_flow(uart_inst_t*, bool cts, bool rts) { assert(!cts && !rts); }
void uart_set_format(uart_inst_t*, unsigned bits, unsigned stop, unsigned parity) {
  assert(bits == 8 && stop == 1 && parity == UART_PARITY_NONE);
  events.push_back("uart_format");
}
void uart_set_fifo_enabled(uart_inst_t*, bool enabled) { assert(enabled); }
bool uart_is_readable(uart_inst_t*) { return !rx.empty(); }
char uart_getc(uart_inst_t*) { assert(!rx.empty()); const auto byte = rx.front(); rx.pop_front(); return byte; }
void uart_write_blocking(uart_inst_t*, const uint8_t* data, size_t length) {
  if (halfDuplex) assert(pins[2].level && !pins[3].level && irqState == 1);
  assert(pendingTx.empty());
  pendingTx.assign(data, data + length);  // FIFO accepted bytes, wire still busy.
  events.push_back("uart_write");
}
void uart_tx_wait_blocking(uart_inst_t*) {
  events.push_back("uart_wait");
  if (!pendingTx.empty()) {
    if (halfDuplex) assert(pins[2].level && !pins[3].level && irqState == 1);
    wirePackets.push_back(pendingTx);
    pendingTx.clear();
    events.push_back("final_stop_bit");
  }
}
uint32_t save_and_disable_interrupts() { const auto previous = irqState; irqState = 1; return previous; }
void restore_interrupts(uint32_t state) { irqState = state; events.push_back("irq_restore"); }
absolute_time_t get_absolute_time() { return timeMs++; }
uint32_t to_ms_since_boot(absolute_time_t time) { return static_cast<uint32_t>(time); }
void busy_wait_us_32(uint32_t us) { assert(us == 1); }

int main() {
  testTransport();
  testHalAndDrivers();
  testProtocolCorrections();
  std::cout << "Servo UART transport, HAL and backend regression tests passed\n";
}
