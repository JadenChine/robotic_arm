#include "drivers/servo_bus_driver.hpp"

#include <string.h>

namespace arm {

namespace {

template <typename T>
T clampValue(T value, T minimum, T maximum) {
  if (value < minimum) {
    return minimum;
  }
  if (value > maximum) {
    return maximum;
  }
  return value;
}

uint8_t lowByteFrom16(int16_t value) {
  return static_cast<uint8_t>(static_cast<uint16_t>(value) & 0xFFu);
}

uint8_t highByteFrom16(int16_t value) {
  return static_cast<uint8_t>((static_cast<uint16_t>(value) >> 8) & 0xFFu);
}

uint8_t lowByteFromU16(uint16_t value) {
  return static_cast<uint8_t>(value & 0xFFu);
}

uint8_t highByteFromU16(uint16_t value) {
  return static_cast<uint8_t>((value >> 8) & 0xFFu);
}

uint16_t makeWord(uint8_t high, uint8_t low) {
  return static_cast<uint16_t>((static_cast<uint16_t>(high) << 8) | low);
}

}  // namespace

ServoBusDriver::ServoBusDriver(ServoTransport& transport)
    : transport_(&transport), last_packet_error_(0) {}

void ServoBusDriver::moveTo(uint8_t id, float degrees, uint16_t timeMs) {
  move(id, degreesToPosition(degrees), timeMs);
}

float ServoBusDriver::getPositionInDegrees(uint8_t id) {
  const int16_t position = getPosition(id);
  return position == kServoPositionInvalidRaw ? kServoPositionInvalidDegrees
                                              : positionToDegrees(position);
}

void ServoBusDriver::setAngleLimitsInDegrees(uint8_t id, float minDegrees,
                                             float maxDegrees) {
  setAngleLimits(id, degreesToPosition(minDegrees), degreesToPosition(maxDegrees));
}

bool ServoBusDriver::getAngleLimitsInDegrees(uint8_t id, float& minDegrees,
                                             float& maxDegrees) {
  int16_t minAngle = 0;
  int16_t maxAngle = 0;
  if (!getAngleLimits(id, minAngle, maxAngle)) {
    return false;
  }
  minDegrees = positionToDegrees(minAngle);
  maxDegrees = positionToDegrees(maxAngle);
  return true;
}

bool ServoBusDriver::getMode(uint8_t id, uint8_t& mode, int16_t& speed) {
  sendPacket(id, kServoOrMotorModeRead, nullptr, 0);
  uint8_t data[4] = {0};
  if (receivePacket(id, kServoOrMotorModeRead, data, 4) <= 0) {
    return false;
  }
  mode = data[0];
  speed = static_cast<int16_t>(makeWord(data[3], data[2]));
  return true;
}

int8_t ServoBusDriver::getAngleOffset(uint8_t id) {
  sendPacket(id, kServoAngleOffsetRead, nullptr, 0);
  uint8_t data[1] = {0};
  if (receivePacket(id, kServoAngleOffsetRead, data, 1) <= 0) {
    return static_cast<int8_t>(kServoOffsetInvalid);
  }
  return static_cast<int8_t>(data[0]);
}

void ServoBusDriver::move(uint8_t id, int16_t position, uint16_t timeMs) {
  const int16_t clampedPosition = clampValue<int16_t>(position, 0, 1000);
  uint8_t params[4] = {
      lowByteFrom16(clampedPosition), highByteFrom16(clampedPosition),
      lowByteFromU16(timeMs),         highByteFromU16(timeMs),
  };
  sendPacket(id, kServoMoveTimeWrite, params, 4);
}

void ServoBusDriver::stop(uint8_t id) {
  sendPacket(id, kServoMoveStop, nullptr, 0);
}

void ServoBusDriver::setId(uint8_t oldId, uint8_t newId) {
  const uint8_t params[1] = {newId};
  sendPacket(oldId, kServoIdWrite, params, 1);
}

uint8_t ServoBusDriver::getId(uint8_t id) {
  sendPacket(id, kServoIdRead, nullptr, 0);
  uint8_t data[1] = {0};
  return receivePacket(id, kServoIdRead, data, 1) > 0 ? data[0] : kServoIdInvalid;
}

void ServoBusDriver::setAngleOffset(uint8_t id, int8_t offset) {
  const uint8_t params[1] = {
      static_cast<uint8_t>(clampValue<int>(offset, -125, 125)),
  };
  sendPacket(id, kServoAngleOffsetAdjust, params, 1);
}

void ServoBusDriver::saveAngleOffset(uint8_t id) {
  sendPacket(id, kServoAngleOffsetWrite, nullptr, 0);
}

void ServoBusDriver::setAngleLimits(uint8_t id, int16_t minAngle, int16_t maxAngle) {
  const int16_t clampedMin = clampValue<int16_t>(minAngle, 0, 1000);
  const int16_t clampedMax = clampValue<int16_t>(maxAngle, 0, 1000);
  uint8_t params[4] = {lowByteFrom16(clampedMin), highByteFrom16(clampedMin),
                       lowByteFrom16(clampedMax), highByteFrom16(clampedMax)};
  sendPacket(id, kServoAngleLimitWrite, params, 4);
}

bool ServoBusDriver::getAngleLimits(uint8_t id, int16_t& minAngle, int16_t& maxAngle) {
  sendPacket(id, kServoAngleLimitRead, nullptr, 0);
  uint8_t data[4] = {0};
  if (receivePacket(id, kServoAngleLimitRead, data, 4) <= 0) {
    return false;
  }
  minAngle = static_cast<int16_t>(makeWord(data[1], data[0]));
  maxAngle = static_cast<int16_t>(makeWord(data[3], data[2]));
  return true;
}

void ServoBusDriver::setVoltageLimits(uint8_t id, int16_t minVoltage,
                                      int16_t maxVoltage) {
  const int16_t clampedMin = clampValue<int16_t>(minVoltage, 4500, 12000);
  const int16_t clampedMax = clampValue<int16_t>(maxVoltage, 4500, 12000);
  uint8_t params[4] = {lowByteFrom16(clampedMin), highByteFrom16(clampedMin),
                       lowByteFrom16(clampedMax), highByteFrom16(clampedMax)};
  sendPacket(id, kServoVinLimitWrite, params, 4);
}

bool ServoBusDriver::getVoltageLimits(uint8_t id, int16_t& minVoltage,
                                      int16_t& maxVoltage) {
  sendPacket(id, kServoVinLimitRead, nullptr, 0);
  uint8_t data[4] = {0};
  if (receivePacket(id, kServoVinLimitRead, data, 4) <= 0) {
    return false;
  }
  minVoltage = static_cast<int16_t>(makeWord(data[1], data[0]));
  maxVoltage = static_cast<int16_t>(makeWord(data[3], data[2]));
  return true;
}

void ServoBusDriver::setMaxTemperatureLimit(uint8_t id, uint8_t maxTemp) {
  const uint8_t params[1] = {static_cast<uint8_t>(clampValue<int>(maxTemp, 50, 100))};
  sendPacket(id, kServoTempMaxLimitWrite, params, 1);
}

uint8_t ServoBusDriver::getMaxTemperatureLimit(uint8_t id) {
  sendPacket(id, kServoTempMaxLimitRead, nullptr, 0);
  uint8_t data[1] = {0};
  return receivePacket(id, kServoTempMaxLimitRead, data, 1) > 0 ? data[0] : 0;
}

int8_t ServoBusDriver::getTemperature(uint8_t id) {
  sendPacket(id, kServoTempRead, nullptr, 0);
  uint8_t data[1] = {0};
  return receivePacket(id, kServoTempRead, data, 1) > 0
             ? static_cast<int8_t>(data[0])
             : static_cast<int8_t>(kServoErrorTimeout);
}

int16_t ServoBusDriver::getVoltage(uint8_t id) {
  sendPacket(id, kServoVinRead, nullptr, 0);
  uint8_t data[2] = {0};
  return receivePacket(id, kServoVinRead, data, 2) > 0
             ? static_cast<int16_t>(makeWord(data[1], data[0]))
             : static_cast<int16_t>(kServoErrorTimeout);
}

int16_t ServoBusDriver::getPosition(uint8_t id) {
  sendPacket(id, kServoPosRead, nullptr, 0);
  uint8_t data[2] = {0};
  return receivePacket(id, kServoPosRead, data, 2) > 0
             ? static_cast<int16_t>(makeWord(data[1], data[0]))
             : kServoPositionInvalidRaw;
}

void ServoBusDriver::setServoMode(uint8_t id) {
  const uint8_t params[4] = {0, 0, 0, 0};
  sendPacket(id, kServoOrMotorModeWrite, params, 4);
}

void ServoBusDriver::setMotorMode(uint8_t id, int16_t speed) {
  const int16_t clampedSpeed = clampValue<int16_t>(speed, -1000, 1000);
  uint8_t params[4] = {1, 0, lowByteFrom16(clampedSpeed), highByteFrom16(clampedSpeed)};
  sendPacket(id, kServoOrMotorModeWrite, params, 4);
}

void ServoBusDriver::motorOn(uint8_t id) {
  const uint8_t params[1] = {1};
  sendPacket(id, kServoLoadOrUnloadWrite, params, 1);
}

void ServoBusDriver::motorOff(uint8_t id) {
  const uint8_t params[1] = {0};
  sendPacket(id, kServoLoadOrUnloadWrite, params, 1);
}

bool ServoBusDriver::isMotorOn(uint8_t id) {
  sendPacket(id, kServoLoadOrUnloadRead, nullptr, 0);
  uint8_t data[1] = {0};
  return receivePacket(id, kServoLoadOrUnloadRead, data, 1) > 0 && data[0] == 1;
}

void ServoBusDriver::ledOn(uint8_t id) {
  const uint8_t params[1] = {1};
  sendPacket(id, kServoLedCtrlWrite, params, 1);
}

void ServoBusDriver::ledOff(uint8_t id) {
  const uint8_t params[1] = {0};
  sendPacket(id, kServoLedCtrlWrite, params, 1);
}

void ServoBusDriver::setLedErrors(uint8_t id, uint8_t error) {
  const uint8_t params[1] = {error};
  sendPacket(id, kServoLedErrorWrite, params, 1);
}

uint8_t ServoBusDriver::getLedErrors(uint8_t id) {
  sendPacket(id, kServoLedErrorRead, nullptr, 0);
  uint8_t data[1] = {0};
  return receivePacket(id, kServoLedErrorRead, data, 1) > 0 ? data[0] : kServoIdInvalid;
}

void ServoBusDriver::broadcastMove(int16_t position, uint16_t timeMs) {
  const int16_t clampedPosition = clampValue<int16_t>(position, 0, 1000);
  uint8_t params[4] = {lowByteFrom16(clampedPosition), highByteFrom16(clampedPosition),
                       lowByteFromU16(timeMs), highByteFromU16(timeMs)};
  sendPacket(kServoBroadcastId, kServoMoveTimeWrite, params, 4);
}

void ServoBusDriver::broadcastMoveDegrees(float degrees, uint16_t timeMs) {
  broadcastMove(degreesToPosition(degrees), timeMs);
}

void ServoBusDriver::broadcastStop() {
  sendPacket(kServoBroadcastId, kServoMoveStop, nullptr, 0);
}

void ServoBusDriver::broadcastLedOn() {
  const uint8_t params[1] = {1};
  sendPacket(kServoBroadcastId, kServoLedCtrlWrite, params, 1);
}

void ServoBusDriver::broadcastLedOff() {
  const uint8_t params[1] = {0};
  sendPacket(kServoBroadcastId, kServoLedCtrlWrite, params, 1);
}

void ServoBusDriver::moveWait(uint8_t id, int16_t position, uint16_t timeMs) {
  const int16_t clampedPosition = clampValue<int16_t>(position, 0, 1000);
  uint8_t params[4] = {lowByteFrom16(clampedPosition), highByteFrom16(clampedPosition),
                       lowByteFromU16(timeMs), highByteFromU16(timeMs)};
  sendPacket(id, kServoMoveTimeWaitWrite, params, 4);
}

void ServoBusDriver::moveWaitDegrees(uint8_t id, float degrees, uint16_t timeMs) {
  moveWait(id, degreesToPosition(degrees), timeMs);
}

void ServoBusDriver::startMove(uint8_t id) {
  sendPacket(id, kServoMoveStart, nullptr, 0);
}

void ServoBusDriver::stopMove(uint8_t id) {
  sendPacket(id, kServoMoveStop, nullptr, 0);
}

bool ServoBusDriver::readWaitPosition(uint8_t id, int16_t& position, uint16_t& timeMs) {
  sendPacket(id, kServoMoveTimeWaitRead, nullptr, 0);
  uint8_t data[4] = {0};
  if (receivePacket(id, kServoMoveTimeWaitRead, data, 4) <= 0) {
    return false;
  }
  position = static_cast<int16_t>(makeWord(data[1], data[0]));
  timeMs = makeWord(data[3], data[2]);
  return true;
}

bool ServoBusDriver::readWaitPositionInDegrees(uint8_t id, float& degrees,
                                               uint16_t& timeMs) {
  int16_t position = 0;
  if (!readWaitPosition(id, position, timeMs)) {
    return false;
  }
  degrees = positionToDegrees(position);
  return true;
}

bool ServoBusDriver::isConnected(uint8_t id) {
  return getVoltage(id) != kServoErrorTimeout;
}

bool ServoBusDriver::moveMultiple(const uint8_t* ids, const int16_t* positions,
                                  uint16_t timeMs, int count) {
  if (ids == nullptr || positions == nullptr || count <= 0) {
    return false;
  }

  bool allValid = true;
  for (int index = 0; index < count; ++index) {
    if (!isValidServoId(ids[index])) {
      allValid = false;
      continue;
    }
    moveWait(ids[index], positions[index], timeMs);
  }
  startMove(kServoBroadcastId);
  return allValid;
}

int ServoBusDriver::getLastPacketError() const {
  return last_packet_error_;
}

void ServoBusDriver::sendPacket(uint8_t id, uint8_t command, const uint8_t* params,
                                uint8_t paramLen) {
  transport_->clearRx();

  uint8_t txBuffer[32] = {0};
  txBuffer[0] = 0x55;
  txBuffer[1] = 0x55;
  txBuffer[2] = id;
  txBuffer[3] = static_cast<uint8_t>(3 + paramLen);
  txBuffer[4] = command;
  if (params != nullptr && paramLen > 0) {
    memcpy(&txBuffer[5], params, paramLen);
  }
  txBuffer[5 + paramLen] = checksum(txBuffer, 6 + paramLen);
  transport_->write(txBuffer, 6 + paramLen);
  transport_->flush();
}

int ServoBusDriver::receivePacket(uint8_t id, uint8_t command, uint8_t* data,
                                  int paramLen) {
  constexpr uint32_t kResponseTimeoutMs = 20;

  const uint32_t startTime = transport_->millis();
  int receivedBytes = 0;
  int packetLength = 0;
  uint8_t checksumValue = 0;

  while ((transport_->millis() - startTime) < kResponseTimeoutMs) {
    if (!transport_->available()) {
      continue;
    }

    const uint8_t byte = transport_->read();
    switch (receivedBytes) {
      case 0:
      case 1:
        if (byte == 0x55) {
          ++receivedBytes;
        } else {
          receivedBytes = 0;
          checksumValue = 0;
          packetLength = 0;
        }
        break;
      case 2:
        if (byte == id) {
          checksumValue = static_cast<uint8_t>(checksumValue + byte);
          ++receivedBytes;
        } else {
          receivedBytes = 0;
          checksumValue = 0;
          packetLength = 0;
        }
        break;
      case 3:
        packetLength = byte;
        if (packetLength == paramLen + 3) {
          checksumValue = static_cast<uint8_t>(checksumValue + byte);
          ++receivedBytes;
        } else {
          receivedBytes = 0;
          checksumValue = 0;
          packetLength = 0;
        }
        break;
      case 4:
        if (byte == command) {
          checksumValue = static_cast<uint8_t>(checksumValue + byte);
          ++receivedBytes;
        } else {
          receivedBytes = 0;
          checksumValue = 0;
          packetLength = 0;
        }
        break;
      default: {
        if (receivedBytes < packetLength + 2) {
          const int dataIndex = receivedBytes - 5;
          if (dataIndex >= 0 && dataIndex < paramLen) {
            data[dataIndex] = byte;
            checksumValue = static_cast<uint8_t>(checksumValue + byte);
            ++receivedBytes;
          } else {
            last_packet_error_ = kServoErrorOverflow;
            return -1;
          }
        } else {
          if (byte == static_cast<uint8_t>(~checksumValue)) {
            last_packet_error_ = 0;
            return paramLen;
          }
          receivedBytes = 0;
          checksumValue = 0;
          packetLength = 0;
        }
      } break;
    }
  }

  last_packet_error_ = kServoErrorTimeout;
  return -1;
}

uint8_t ServoBusDriver::checksum(const uint8_t* buffer, int length) const {
  uint8_t sum = 0;
  for (int index = 2; index < length - 1; ++index) {
    sum = static_cast<uint8_t>(sum + buffer[index]);
  }
  return static_cast<uint8_t>(~sum);
}

int16_t ServoBusDriver::degreesToPosition(float degrees) const {
  const float clamped = clampValue<float>(degrees, -120.0f, 120.0f);
  return static_cast<int16_t>((clamped * (500.0f / 120.0f)) + 500.0f);
}

float ServoBusDriver::positionToDegrees(int16_t position) const {
  return static_cast<float>((static_cast<float>(position) - 500.0f) * (120.0f / 500.0f));
}

bool ServoBusDriver::isValidServoId(uint8_t id) const {
  return id <= 253;
}

}  // namespace arm