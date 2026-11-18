#ifndef ARM_SERVO_BUS_DRIVER_HPP
#define ARM_SERVO_BUS_DRIVER_HPP

#include <stddef.h>
#include <stdint.h>

#include "platform/pico_uart_transport.hpp"

namespace arm {

constexpr uint8_t kServoMoveTimeWrite = 1;
constexpr uint8_t kServoMoveTimeRead = 2;
constexpr uint8_t kServoMoveTimeWaitWrite = 7;
constexpr uint8_t kServoMoveTimeWaitRead = 8;
constexpr uint8_t kServoMoveStart = 11;
constexpr uint8_t kServoMoveStop = 12;
constexpr uint8_t kServoIdWrite = 13;
constexpr uint8_t kServoIdRead = 14;
constexpr uint8_t kServoAngleOffsetAdjust = 17;
constexpr uint8_t kServoAngleOffsetWrite = 18;
constexpr uint8_t kServoAngleOffsetRead = 19;
constexpr uint8_t kServoAngleLimitWrite = 20;
constexpr uint8_t kServoAngleLimitRead = 21;
constexpr uint8_t kServoVinLimitWrite = 22;
constexpr uint8_t kServoVinLimitRead = 23;
constexpr uint8_t kServoTempMaxLimitWrite = 24;
constexpr uint8_t kServoTempMaxLimitRead = 25;
constexpr uint8_t kServoTempRead = 26;
constexpr uint8_t kServoVinRead = 27;
constexpr uint8_t kServoPosRead = 28;
constexpr uint8_t kServoOrMotorModeWrite = 29;
constexpr uint8_t kServoOrMotorModeRead = 30;
constexpr uint8_t kServoLoadOrUnloadWrite = 31;
constexpr uint8_t kServoLoadOrUnloadRead = 32;
constexpr uint8_t kServoLedCtrlWrite = 33;
constexpr uint8_t kServoLedCtrlRead = 34;
constexpr uint8_t kServoLedErrorWrite = 35;
constexpr uint8_t kServoLedErrorRead = 36;

constexpr int kServoErrorTimeout = -1;
constexpr int kServoErrorOverflow = -2;
constexpr int kServoErrorInvalidArgument = -3;
constexpr int16_t kServoPositionInvalidRaw = -32768;
constexpr float kServoPositionInvalidDegrees = -999.0f;
constexpr uint8_t kServoOffsetInvalid = 127;
constexpr uint8_t kServoIdInvalid = 255;
constexpr uint8_t kServoBroadcastId = 254;

class ServoBusDriver {
public:
  explicit ServoBusDriver(ServoTransport& transport);

  void moveTo(uint8_t id, float degrees, uint16_t timeMs = 1000);
  float getPositionInDegrees(uint8_t id);
  void setAngleLimitsInDegrees(uint8_t id, float minDegrees, float maxDegrees);
  bool getAngleLimitsInDegrees(uint8_t id, float& minDegrees, float& maxDegrees);
  bool getMode(uint8_t id, uint8_t& mode, int16_t& speed);
  int8_t getAngleOffset(uint8_t id);
  void move(uint8_t id, int16_t position, uint16_t timeMs = 1000);
  void stop(uint8_t id);
  void setId(uint8_t oldId, uint8_t newId);
  uint8_t getId(uint8_t id = 1);
  void setAngleOffset(uint8_t id, int8_t offset);
  void saveAngleOffset(uint8_t id);
  void setAngleLimits(uint8_t id, int16_t minAngle, int16_t maxAngle);
  bool getAngleLimits(uint8_t id, int16_t& minAngle, int16_t& maxAngle);
  void setVoltageLimits(uint8_t id, int16_t minVoltage, int16_t maxVoltage);
  bool getVoltageLimits(uint8_t id, int16_t& minVoltage, int16_t& maxVoltage);
  void setMaxTemperatureLimit(uint8_t id, uint8_t maxTemp);
  uint8_t getMaxTemperatureLimit(uint8_t id);
  int8_t getTemperature(uint8_t id);
  int16_t getVoltage(uint8_t id);
  int16_t getPosition(uint8_t id);
  void setServoMode(uint8_t id);
  void setMotorMode(uint8_t id, int16_t speed);
  void motorOn(uint8_t id);
  void motorOff(uint8_t id);
  bool isMotorOn(uint8_t id);
  void ledOn(uint8_t id);
  void ledOff(uint8_t id);
  void setLedErrors(uint8_t id, uint8_t error);
  uint8_t getLedErrors(uint8_t id);
  void broadcastMove(int16_t position, uint16_t timeMs);
  void broadcastMoveDegrees(float degrees, uint16_t timeMs);
  void broadcastStop();
  void broadcastLedOn();
  void broadcastLedOff();
  void moveWait(uint8_t id, int16_t position, uint16_t timeMs);
  void moveWaitDegrees(uint8_t id, float degrees, uint16_t timeMs);
  void startMove(uint8_t id = kServoBroadcastId);
  void stopMove(uint8_t id = kServoBroadcastId);
  bool readWaitPosition(uint8_t id, int16_t& position, uint16_t& timeMs);
  bool readWaitPositionInDegrees(uint8_t id, float& degrees, uint16_t& timeMs);
  bool isConnected(uint8_t id);
  bool moveMultiple(const uint8_t* ids, const int16_t* positions, uint16_t timeMs,
                    int count);
  int getLastPacketError() const;

private:
  ServoTransport* transport_;
  int last_packet_error_;

  void sendPacket(uint8_t id, uint8_t command, const uint8_t* params, uint8_t paramLen);
  int receivePacket(uint8_t id, uint8_t command, uint8_t* data, int paramLen);
  uint8_t checksum(const uint8_t* buffer, int length) const;
  int16_t degreesToPosition(float degrees) const;
  float positionToDegrees(int16_t position) const;
  bool isValidServoId(uint8_t id) const;
};

}  // namespace arm

#endif
