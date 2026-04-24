#include "platform/pico_uart_transport.hpp"

#include "hardware/gpio.h"
#include "pico/time.h"

namespace arm {

namespace {

uart_inst_t* selectUart(uint8_t uartIndex) {
  return uartIndex == 0 ? uart0 : uart1;
}

}  // namespace

PicoUartTransport::PicoUartTransport()
    : uart_(nullptr),
      uart_index_(0),
      baud_rate_(0),
      tx_pin_(0),
      rx_pin_(0),
      initialized_(false) {}

bool PicoUartTransport::init(uint8_t uartIndex, uint32_t baudRate, uint8_t txPin,
                             uint8_t rxPin) {
  uart_ = selectUart(uartIndex);
  uart_index_ = uartIndex;
  baud_rate_ = baudRate;
  tx_pin_ = txPin;
  rx_pin_ = rxPin;

  uart_init(uart_, baudRate);
  gpio_set_function(txPin, GPIO_FUNC_UART);
  gpio_set_function(rxPin, GPIO_FUNC_UART);
  uart_set_format(uart_, 8, 1, UART_PARITY_NONE);
  uart_set_fifo_enabled(uart_, true);
  initialized_ = true;
  clearRx();
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
  uart_write_blocking(uart_, data, length);
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