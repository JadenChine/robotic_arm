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
  std::cout << "Servo UART transport, HAL and backend regression tests passed\n";
}
