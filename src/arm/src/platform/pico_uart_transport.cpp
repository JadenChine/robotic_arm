#include "platform/pico_uart_transport.hpp"

#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "pico/time.h"

namespace arm {

namespace {

uart_inst_t* selectUart(uint8_t uartIndex) {
  return uartIndex == 0 ? uart0 : uart1;
}

// GPIOs exposed on the original Pico / Pico W headers. Avoid internal board
// connections (including the Pico W radio) when configuring buffer enables.
bool isHeaderPin(uint8_t pin) {
  return pin <= 22 || (pin >= 26 && pin <= 28);
}

void initDisabledEnablePin(uint8_t pin) {
  gpio_init(pin);
  gpio_pull_down(pin);
  gpio_put(pin, false);
  gpio_set_dir(pin, GPIO_OUT);
}

}  // namespace

PicoUartTransport::PicoUartTransport()
    : uart_(nullptr),
      uart_index_(0),
      baud_rate_(0),
      tx_pin_(0),
      rx_pin_(0),
      tx_enable_pin_(kNoEnablePin),
      rx_enable_pin_(kNoEnablePin),
      initialized_(false) {}

bool PicoUartTransport::init(uint8_t uartIndex, uint32_t baudRate, uint8_t txPin,
                             uint8_t rxPin, uint8_t txEnablePin,
                             uint8_t rxEnablePin) {
  const bool halfDuplex = txEnablePin != kNoEnablePin;
  if (uartIndex > 1 || baudRate == 0 || !isHeaderPin(txPin) ||
      !isHeaderPin(rxPin) || txPin % 4 != 0 || rxPin % 4 != 1 ||
      (txPin / 4) % 2 != uartIndex || (rxPin / 4) % 2 != uartIndex ||
      halfDuplex != (rxEnablePin != kNoEnablePin) ||
      (halfDuplex && (!isHeaderPin(txEnablePin) || !isHeaderPin(rxEnablePin) ||
                      txEnablePin == rxEnablePin || txEnablePin == txPin ||
                      txEnablePin == rxPin || rxEnablePin == txPin ||
                      rxEnablePin == rxPin))) {
    return false;
  }

  if (initialized_) {
    flush();
    if (tx_enable_pin_ != kNoEnablePin) {
      // Also release the old circuit when changing backends or OE pins.
      gpio_put(tx_enable_pin_, false);
      gpio_put(rx_enable_pin_, false);
      gpio_set_dir(tx_enable_pin_, GPIO_IN);
      gpio_set_dir(rx_enable_pin_, GPIO_IN);
    }
    uart_deinit(uart_);
  }
  initialized_ = false;
  uart_ = selectUart(uartIndex);
  uart_index_ = uartIndex;
  baud_rate_ = baudRate;
  tx_pin_ = txPin;
  rx_pin_ = rxPin;
  tx_enable_pin_ = txEnablePin;
  rx_enable_pin_ = rxEnablePin;

  if (halfDuplex) {
    // Keep both buffers off until UART TX is configured and idles HIGH.
    initDisabledEnablePin(txEnablePin);
    initDisabledEnablePin(rxEnablePin);
  }
  if (uart_init(uart_, baudRate) == 0) {
    return false;
  }
  gpio_set_function(txPin, GPIO_FUNC_UART);
  gpio_set_function(rxPin, GPIO_FUNC_UART);
  uart_set_hw_flow(uart_, false, false);
  uart_set_format(uart_, 8, 1, UART_PARITY_NONE);
  uart_set_fifo_enabled(uart_, true);
  if (halfDuplex) {
    // RX buffer Y is high impedance while sending; maintain UART idle HIGH.
    gpio_pull_up(rxPin);
  }
  initialized_ = true;
  clearRx();
  if (halfDuplex) {
    gpio_put(rxEnablePin, true);
  }
  return true;
}

void PicoUartTransport::clearRx() {
  if (!initialized_) {
    return;
  }
  while (uart_is_readable(uart_)) {
    (void)uart_getc(uart_);
  }
}

void PicoUartTransport::write(const uint8_t* data, size_t length) {
  if (!initialized_ || data == nullptr || length == 0) {
    return;
  }
  if (tx_enable_pin_ == kNoEnablePin) {
    uart_write_blocking(uart_, data, length);
    return;
  }

  // Core0 owns this transport. Prevent a timer/USB ISR from delaying bus
  // release after the last stop bit. Current direct packets are <=10 bytes
  // (~0.87 ms at 115200 baud); BLE continues independently on core1.
  const uint32_t irqState = save_and_disable_interrupts();
  gpio_put(rx_enable_pin_, false);
  gpio_put(tx_enable_pin_, true);
  busy_wait_us_32(1);  // Allow OE propagation before the first start bit.
  uart_write_blocking(uart_, data, length);
  // SDK waits on UART BUSY, including the shift register and final stop bit,
  // rather than just waiting for FIFO space or FIFO-empty.
  uart_tx_wait_blocking(uart_);
  gpio_put(tx_enable_pin_, false);
  gpio_put(rx_enable_pin_, true);
  restore_interrupts(irqState);
  // Do not clear RX here: a servo can reply as soon as the bus is released.
}

bool PicoUartTransport::available() const {
  return initialized_ && uart_is_readable(uart_);
}

uint8_t PicoUartTransport::read() {
  return initialized_ ? uart_getc(uart_) : 0;
}

uint32_t PicoUartTransport::millis() const {
  return to_ms_since_boot(get_absolute_time());
}

void PicoUartTransport::flush() {
  if (!initialized_) {
    return;
  }
  uart_tx_wait_blocking(uart_);
}

bool PicoUartTransport::initialized() const {
  return initialized_;
}

uint8_t PicoUartTransport::uartIndex() const {
  return uart_index_;
}

uint32_t PicoUartTransport::baudRate() const {
  return baud_rate_;
}

uint8_t PicoUartTransport::txPin() const {
  return tx_pin_;
}

uint8_t PicoUartTransport::rxPin() const {
  return rx_pin_;
}

}  // namespace arm
