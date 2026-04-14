#include "drivers/xarm_controller_driver.hpp"

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

uint16_t makeWord(uint8_t high, uint8_t low) {
  return static_cast<uint16_t>((static_cast<uint16_t>(high) << 8) | low);
}

}  // namespace

XArmControllerDriver::XArmControllerDriver(ServoTransport& transport)
    : transport_(&transport), last_packet_error_(kControllerErrorNone) {}

bool XArmControllerDriver::move(uint8_t servoId, int16_t position, uint16_t timeMs) {
  const uint8_t id = clampWritableServoId(servoId);
  const int16_t clampedPosition = clampServoPosition(position);

  uint8_t ids[1] = {id};
  int16_t positions[1] = {clampedPosition};
  return moveMultiple(ids, positions, 1, timeMs);
}

bool XArmControllerDriver::moveMultiple(const uint8_t* ids, const int16_t* positions,
                                        size_t count, uint16_t timeMs) {
  if (ids == nullptr || positions == nullptr || count == 0) {
    last_packet_error_ = kControllerErrorInvalidArgument;
    return false;
  }
  if (count > kMaxMoveServoCount) {
    last_packet_error_ = kControllerErrorOverflow;
    return false;
  }

  uint8_t params[kMaxCommandParams] = {0};
  params[0] = static_cast<uint8_t>(count);
  params[1] = static_cast<uint8_t>(timeMs & 0xFFu);
  params[2] = static_cast<uint8_t>((timeMs >> 8) & 0xFFu);

  for (size_t index = 0; index < count; ++index) {
    const size_t base = 3 + (index * 3);
    const int16_t clampedPosition = clampServoPosition(positions[index]);

    params[base] = clampWritableServoId(ids[index]);
    params[base + 1] = static_cast<uint8_t>(clampedPosition & 0xFFu);
    params[base + 2] = static_cast<uint8_t>((clampedPosition >> 8) & 0xFFu);
  }

  return sendPacket(kControllerCmdServoMove, params,
                    static_cast<uint8_t>(3 + (count * 3)));
}

bool XArmControllerDriver::readPosition(uint8_t servoId, int16_t& position) {
  if (!isReadableServoId(servoId)) {
    last_packet_error_ = kControllerErrorInvalidArgument;
    return false;
  }

  const uint8_t ids[1] = {servoId};
  return readPositions(ids, 1, &position);
}

bool XArmControllerDriver::readPositions(const uint8_t* ids, size_t count,
                                         int16_t* positions) {
  if (ids == nullptr || positions == nullptr || count == 0) {
    last_packet_error_ = kControllerErrorInvalidArgument;
    return false;
  }
  if (count > kMaxReadServoCount) {
    last_packet_error_ = kControllerErrorOverflow;
    return false;
  }

  uint8_t params[kMaxCommandParams] = {0};
  params[0] = static_cast<uint8_t>(count);

  for (size_t index = 0; index < count; ++index) {
    if (!isReadableServoId(ids[index])) {
      last_packet_error_ = kControllerErrorInvalidArgument;
      return false;
    }
    params[1 + index] = ids[index];
  }

  if (!sendPacket(kControllerCmdMultiServoPosRead, params,
                  static_cast<uint8_t>(1 + count))) {
    return false;
  }

  uint8_t response[kMaxCommandParams] = {0};
  const int responseLength = receivePacket(kControllerCmdMultiServoPosRead, response,
                                           sizeof(response), kDefaultResponseTimeoutMs);
  if (responseLength <= 0) {
    return false;
  }

  const uint8_t responseCount = response[0];
  const int expectedLength = 1 + (responseCount * 3);
  if (responseCount == 0 || responseLength < expectedLength) {
    last_packet_error_ = kControllerErrorProtocol;
    return false;
  }

  for (size_t requestIndex = 0; requestIndex < count; ++requestIndex) {
    bool found = false;
    for (uint8_t responseIndex = 0; responseIndex < responseCount; ++responseIndex) {
      const int base = 1 + (responseIndex * 3);
      const uint8_t responseId = response[base];
      if (responseId != ids[requestIndex]) {
        continue;
      }

      positions[requestIndex] = static_cast<int16_t>(
          makeWord(response[base + 2], response[base + 1]));
      found = true;
      break;
    }

    if (!found) {
      last_packet_error_ = kControllerErrorProtocol;
      return false;
    }
  }

  last_packet_error_ = kControllerErrorNone;
  return true;
}

bool XArmControllerDriver::getBatteryVoltageMv(int16_t& voltageMv) {
  if (!sendPacket(kControllerCmdGetBatteryVoltage, nullptr, 0)) {
    return false;
  }

  uint8_t response[2] = {0};
  const int responseLength = receivePacket(kControllerCmdGetBatteryVoltage, response,
                                           sizeof(response), kDefaultResponseTimeoutMs);
  if (responseLength < 2) {
    if (responseLength >= 0) {
      last_packet_error_ = kControllerErrorProtocol;
    }
    return false;
  }

  voltageMv = static_cast<int16_t>(makeWord(response[1], response[0]));
  last_packet_error_ = kControllerErrorNone;
  return true;
}

bool XArmControllerDriver::unload(uint8_t servoId) {
  const uint8_t id = clampWritableServoId(servoId);
  return unloadMultiple(&id, 1);
}

bool XArmControllerDriver::unloadMultiple(const uint8_t* ids, size_t count) {
  if (ids == nullptr || count == 0 || count > 252) {
    last_packet_error_ = kControllerErrorInvalidArgument;
    return false;
  }

  uint8_t params[kMaxCommandParams] = {0};
  params[0] = static_cast<uint8_t>(count);

  for (size_t index = 0; index < count; ++index) {
    params[1 + index] = clampWritableServoId(ids[index]);
  }

  return sendPacket(kControllerCmdMultiServoUnload, params,
                    static_cast<uint8_t>(1 + count));
}

int XArmControllerDriver::getLastPacketError() const {
  return last_packet_error_;
}

int16_t XArmControllerDriver::degreesToPosition(float degrees) const {
  const float clampedDegrees = clampValue<float>(degrees, -120.0f, 120.0f);
  return static_cast<int16_t>((clampedDegrees * (500.0f / 120.0f)) + 500.0f);
}

float XArmControllerDriver::positionToDegrees(int16_t position) const {
  const int16_t clampedPosition = clampServoPosition(position);
  return static_cast<float>((static_cast<float>(clampedPosition) - 500.0f) *
                            (120.0f / 500.0f));
}

bool XArmControllerDriver::sendPacket(uint8_t command, const uint8_t* params,
                                      uint8_t paramLen) {
  if (paramLen > kMaxCommandParams) {
    last_packet_error_ = kControllerErrorOverflow;
    return false;
  }

  uint8_t txBuffer[4 + kMaxCommandParams] = {0};
  txBuffer[0] = 0x55;
  txBuffer[1] = 0x55;
  txBuffer[2] = static_cast<uint8_t>(paramLen + 2);
  txBuffer[3] = command;

  if (params != nullptr && paramLen > 0) {
    memcpy(&txBuffer[4], params, paramLen);
  }

  transport_->clearRx();
  transport_->write(txBuffer, 4 + paramLen);
  transport_->flush();

  last_packet_error_ = kControllerErrorNone;
  return true;
}

int XArmControllerDriver::receivePacket(uint8_t expectedCommand, uint8_t* data,
                                        size_t maxDataLen, uint32_t timeoutMs) {
  enum ParseState {
    kWaitHeader1,
    kWaitHeader2,
    kWaitLength,
    kWaitCommand,
    kReadPayload,
    kSkipPayload,
  };

  ParseState state = kWaitHeader1;
  uint8_t payloadLength = 0;
  uint8_t payloadIndex = 0;
  uint8_t skipRemaining = 0;
  uint8_t command = 0;
  uint8_t payload[kMaxCommandParams] = {0};

  const uint32_t startTime = transport_->millis();

  while ((transport_->millis() - startTime) < timeoutMs) {
    if (!transport_->available()) {
      continue;
    }

    const uint8_t byte = transport_->read();
    switch (state) {
      case kWaitHeader1:
        state = (byte == 0x55) ? kWaitHeader2 : kWaitHeader1;
        break;

      case kWaitHeader2:
        state = (byte == 0x55) ? kWaitLength : kWaitHeader1;
        break;

      case kWaitLength:
        if (byte < 2) {
          last_packet_error_ = kControllerErrorProtocol;
          state = kWaitHeader1;
          break;
        }

        payloadLength = static_cast<uint8_t>(byte - 2);
        state = kWaitCommand;
        break;

      case kWaitCommand:
        command = byte;
        payloadIndex = 0;

        if (payloadLength == 0) {
          if (command == expectedCommand) {
            last_packet_error_ = kControllerErrorNone;
            return 0;
          }
          state = kWaitHeader1;
          break;
        }

        if (payloadLength > sizeof(payload)) {
          skipRemaining = payloadLength;
          state = kSkipPayload;
          break;
        }

        state = kReadPayload;
        break;

      case kReadPayload:
        payload[payloadIndex++] = byte;

        if (payloadIndex < payloadLength) {
          break;
        }

        if (command != expectedCommand) {
          state = kWaitHeader1;
          break;
        }

        if (payloadLength > maxDataLen) {
          last_packet_error_ = kControllerErrorOverflow;
          return -1;
        }

        if (payloadLength > 0 && data != nullptr) {
          memcpy(data, payload, payloadLength);
        }

        last_packet_error_ = kControllerErrorNone;
        return payloadLength;

      case kSkipPayload:
        if (skipRemaining > 0) {
          --skipRemaining;
        }
        if (skipRemaining == 0) {
          last_packet_error_ = kControllerErrorOverflow;
          state = kWaitHeader1;
        }
        break;
    }
  }

  last_packet_error_ = kControllerErrorTimeout;
  return -1;
}

uint8_t XArmControllerDriver::clampWritableServoId(uint8_t servoId) const {
  if (servoId < 1 || servoId > 254) {
    return 254;
  }
  return servoId;
}

bool XArmControllerDriver::isReadableServoId(uint8_t servoId) const {
  return servoId >= 1 && servoId <= 253;
}

int16_t XArmControllerDriver::clampServoPosition(int16_t position) const {
  return clampValue<int16_t>(position, 0, 1000);
}

}  // namespace arm