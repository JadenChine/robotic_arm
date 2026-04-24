#ifndef ARM_PICO_UART_TRANSPORT_HPP
#define ARM_PICO_UART_TRANSPORT_HPP

#include <stddef.h>
#include <stdint.h>

#include "hardware/uart.h"

namespace arm {

class ServoTransport {
public:
  virtual ~ServoTransport() = default;

  virtual void clearRx() = 0;
  virtual void write(const uint8_t* data, size_t length) = 0;
  virtual bool available() const = 0;
  virtual uint8_t read() = 0;
  virtual uint32_t millis() const = 0;
  virtual void flush() = 0;
};

class PicoUartTransport final : public ServoTransport {
public:
  PicoUartTransport();

  bool init(uint8_t uartIndex, uint32_t baudRate, uint8_t txPin, uint8_t rxPin);

  void clearRx() override;
  void write(const uint8_t* data, size_t length) override;
  bool available() const override;
  uint8_t read() override;
  uint32_t millis() const override;
  void flush() override;

  bool initialized() const;
  uint8_t uartIndex() const;
  uint32_t baudRate() const;
  uint8_t txPin() const;
  uint8_t rxPin() const;

private:
  uart_inst_t* uart_;
  uint8_t uart_index_;
  uint32_t baud_rate_;
  uint8_t tx_pin_;
  uint8_t rx_pin_;
  bool initialized_;
};

}  // namespace arm

#endif