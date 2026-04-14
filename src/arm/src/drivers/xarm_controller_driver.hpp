#ifndef ARM_XARM_CONTROLLER_DRIVER_HPP
#define ARM_XARM_CONTROLLER_DRIVER_HPP

#include <stddef.h>
#include <stdint.h>

#include "platform/pico_uart_transport.hpp"

namespace arm {

constexpr uint8_t kControllerCmdServoMove = 3;
constexpr uint8_t kControllerCmdActionGroupRun = 6;
constexpr uint8_t kControllerCmdActionGroupStop = 7;
constexpr uint8_t kControllerCmdActionGroupSpeed = 11;
constexpr uint8_t kControllerCmdGetBatteryVoltage = 15;
constexpr uint8_t kControllerCmdMultiServoUnload = 20;
constexpr uint8_t kControllerCmdMultiServoPosRead = 21;

constexpr int kControllerErrorNone = 0;
constexpr int kControllerErrorTimeout = -1;
constexpr int kControllerErrorOverflow = -2;
constexpr int kControllerErrorProtocol = -3;
constexpr int kControllerErrorInvalidArgument = -4;

class XArmControllerDriver {
public:
  explicit XArmControllerDriver(ServoTransport& transport);

  bool move(uint8_t servoId, int16_t position, uint16_t timeMs);
  bool moveMultiple(const uint8_t* ids, const int16_t* positions, size_t count,
                    uint16_t timeMs);

  bool readPosition(uint8_t servoId, int16_t& position);
  bool readPositions(const uint8_t* ids, size_t count, int16_t* positions);

  bool getBatteryVoltageMv(int16_t& voltageMv);

  bool unload(uint8_t servoId);
  bool unloadMultiple(const uint8_t* ids, size_t count);

  int getLastPacketError() const;

  int16_t degreesToPosition(float degrees) const;
  float positionToDegrees(int16_t position) const;

private:
  static constexpr uint8_t kMaxCommandParams = 253;
  static constexpr uint8_t kMaxMoveServoCount = 83;
  static constexpr uint8_t kMaxReadServoCount = 84;
  static constexpr uint32_t kDefaultResponseTimeoutMs = 200;

  ServoTransport* transport_;
  int last_packet_error_;

  bool sendPacket(uint8_t command, const uint8_t* params, uint8_t paramLen);
  int receivePacket(uint8_t expectedCommand, uint8_t* data, size_t maxDataLen,
                    uint32_t timeoutMs);

  uint8_t clampWritableServoId(uint8_t servoId) const;
  bool isReadableServoId(uint8_t servoId) const;
  int16_t clampServoPosition(int16_t position) const;
};

}  // namespace arm

#endif