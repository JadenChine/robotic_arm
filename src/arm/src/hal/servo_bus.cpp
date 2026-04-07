#include "hal/servo_bus.h"

#include "drivers/servo_bus_driver.hpp"
#include "platform/pico_uart_transport.hpp"
#include "platform/servo_platform_config.h"

static arm::PicoUartTransport g_transport;
static arm::ServoBusDriver g_driver(g_transport);
static servo_bus_config_t g_active_config = {};
static bool g_initialized = false;

static servo_bus_status_t mapLastDriverError(void) {
  switch (g_driver.getLastPacketError()) {
    case arm::kServoErrorTimeout:
      return SERVO_BUS_STATUS_ERROR_TIMEOUT;
    case arm::kServoErrorOverflow:
      return SERVO_BUS_STATUS_ERROR_OVERFLOW;
    default:
      return SERVO_BUS_STATUS_ERROR_PROTOCOL;
  }
}

static servo_bus_status_t requireInitialized(void) {
  return g_initialized ? SERVO_BUS_STATUS_OK : SERVO_BUS_STATUS_ERROR_NOT_INITIALIZED;
}

extern "C" {

void servo_bus_get_default_config(servo_bus_config_t* config) {
  if (config == nullptr) {
    return;
  }

  config->uart_index = SERVO_BUS_DEFAULT_UART_INDEX;
  config->baud_rate = SERVO_BUS_DEFAULT_BAUD_RATE;
  config->tx_pin = SERVO_BUS_DEFAULT_TX_PIN;
  config->rx_pin = SERVO_BUS_DEFAULT_RX_PIN;
  config->default_servo_id = SERVO_BUS_DEFAULT_SERVO_ID;
  config->default_move_time_ms = SERVO_BUS_DEFAULT_MOVE_TIME_MS;
  config->default_step_degrees = SERVO_BUS_DEFAULT_STEP_DEGREES;
}

const servo_bus_config_t* servo_bus_get_active_config(void) {
  return g_initialized ? &g_active_config : nullptr;
}

bool servo_bus_is_initialized(void) {
  return g_initialized;
}

servo_bus_status_t servo_bus_init(const servo_bus_config_t* config) {
  if (config == nullptr) {
    return SERVO_BUS_STATUS_ERROR_INVALID_ARGUMENT;
  }

  if (!g_transport.init(config->uart_index, config->baud_rate, config->tx_pin,
                        config->rx_pin)) {
    return SERVO_BUS_STATUS_ERROR_PROTOCOL;
  }

  g_active_config = *config;
  g_initialized = true;
  return SERVO_BUS_STATUS_OK;
}

servo_bus_status_t servo_bus_ping(uint8_t id) {
  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }
  return g_driver.isConnected(id) ? SERVO_BUS_STATUS_OK : mapLastDriverError();
}

servo_bus_status_t servo_bus_move_raw(uint8_t id, int16_t position, uint16_t time_ms) {
  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }
  g_driver.move(id, position, time_ms);
  return SERVO_BUS_STATUS_OK;
}

servo_bus_status_t servo_bus_move_degrees(uint8_t id, float degrees, uint16_t time_ms) {
  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }
  g_driver.moveTo(id, degrees, time_ms);
  return SERVO_BUS_STATUS_OK;
}

servo_bus_status_t servo_bus_stop(uint8_t id) {
  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }
  g_driver.stop(id);
  return SERVO_BUS_STATUS_OK;
}

servo_bus_status_t servo_bus_move_wait_raw(uint8_t id, int16_t position,
                                           uint16_t time_ms) {
  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }
  g_driver.moveWait(id, position, time_ms);
  return SERVO_BUS_STATUS_OK;
}

servo_bus_status_t servo_bus_move_wait_degrees(uint8_t id, float degrees,
                                               uint16_t time_ms) {
  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }
  g_driver.moveWaitDegrees(id, degrees, time_ms);
  return SERVO_BUS_STATUS_OK;
}

servo_bus_status_t servo_bus_start_move(uint8_t id) {
  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }
  g_driver.startMove(id);
  return SERVO_BUS_STATUS_OK;
}

servo_bus_status_t servo_bus_stop_move(uint8_t id) {
  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }
  g_driver.stopMove(id);
  return SERVO_BUS_STATUS_OK;
}

servo_bus_status_t servo_bus_read_wait_position(uint8_t id, int16_t* position,
                                                uint16_t* time_ms) {
  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }
  if (position == nullptr || time_ms == nullptr) {
    return SERVO_BUS_STATUS_ERROR_INVALID_ARGUMENT;
  }
  return g_driver.readWaitPosition(id, *position, *time_ms) ? SERVO_BUS_STATUS_OK
                                                            : mapLastDriverError();
}

servo_bus_status_t servo_bus_read_wait_position_degrees(uint8_t id, float* degrees,
                                                        uint16_t* time_ms) {
  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }
  if (degrees == nullptr || time_ms == nullptr) {
    return SERVO_BUS_STATUS_ERROR_INVALID_ARGUMENT;
  }
  return g_driver.readWaitPositionInDegrees(id, *degrees, *time_ms)
             ? SERVO_BUS_STATUS_OK
             : mapLastDriverError();
}

servo_bus_status_t servo_bus_broadcast_move_raw(int16_t position, uint16_t time_ms) {
  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }
  g_driver.broadcastMove(position, time_ms);
  return SERVO_BUS_STATUS_OK;
}

servo_bus_status_t servo_bus_broadcast_move_degrees(float degrees, uint16_t time_ms) {
  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }
  g_driver.broadcastMoveDegrees(degrees, time_ms);
  return SERVO_BUS_STATUS_OK;
}

servo_bus_status_t servo_bus_broadcast_stop(void) {
  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }
  g_driver.broadcastStop();
  return SERVO_BUS_STATUS_OK;
}

servo_bus_status_t servo_bus_broadcast_led_on(void) {
  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }
  g_driver.broadcastLedOn();
  return SERVO_BUS_STATUS_OK;
}

servo_bus_status_t servo_bus_broadcast_led_off(void) {
  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }
  g_driver.broadcastLedOff();
  return SERVO_BUS_STATUS_OK;
}

servo_bus_status_t servo_bus_set_id(uint8_t old_id, uint8_t new_id) {
  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }
  g_driver.setId(old_id, new_id);
  return SERVO_BUS_STATUS_OK;
}

servo_bus_status_t servo_bus_get_id(uint8_t query_id, uint8_t* out_id) {
  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }
  if (out_id == nullptr) {
    return SERVO_BUS_STATUS_ERROR_INVALID_ARGUMENT;
  }
  *out_id = g_driver.getId(query_id);
  return *out_id == arm::kServoIdInvalid ? mapLastDriverError() : SERVO_BUS_STATUS_OK;
}

servo_bus_status_t servo_bus_set_angle_offset(uint8_t id, int8_t offset) {
  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }
  g_driver.setAngleOffset(id, offset);
  return SERVO_BUS_STATUS_OK;
}

servo_bus_status_t servo_bus_save_angle_offset(uint8_t id) {
  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }
  g_driver.saveAngleOffset(id);
  return SERVO_BUS_STATUS_OK;
}

servo_bus_status_t servo_bus_get_angle_offset(uint8_t id, int8_t* offset) {
  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }
  if (offset == nullptr) {
    return SERVO_BUS_STATUS_ERROR_INVALID_ARGUMENT;
  }
  *offset = g_driver.getAngleOffset(id);
  return *offset == static_cast<int8_t>(arm::kServoOffsetInvalid) ? mapLastDriverError()
                                                                  : SERVO_BUS_STATUS_OK;
}

servo_bus_status_t servo_bus_set_angle_limits_raw(uint8_t id, int16_t min_angle,
                                                  int16_t max_angle) {
  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }
  g_driver.setAngleLimits(id, min_angle, max_angle);
  return SERVO_BUS_STATUS_OK;
}

servo_bus_status_t servo_bus_get_angle_limits_raw(uint8_t id, int16_t* min_angle,
                                                  int16_t* max_angle) {
  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }
  if (min_angle == nullptr || max_angle == nullptr) {
    return SERVO_BUS_STATUS_ERROR_INVALID_ARGUMENT;
  }
  return g_driver.getAngleLimits(id, *min_angle, *max_angle) ? SERVO_BUS_STATUS_OK
                                                             : mapLastDriverError();
}

servo_bus_status_t servo_bus_set_angle_limits_degrees(uint8_t id, float min_degrees,
                                                      float max_degrees) {
  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }
  g_driver.setAngleLimitsInDegrees(id, min_degrees, max_degrees);
  return SERVO_BUS_STATUS_OK;
}

servo_bus_status_t servo_bus_get_angle_limits_degrees(uint8_t id, float* min_degrees,
                                                      float* max_degrees) {
  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }
  if (min_degrees == nullptr || max_degrees == nullptr) {
    return SERVO_BUS_STATUS_ERROR_INVALID_ARGUMENT;
  }
  return g_driver.getAngleLimitsInDegrees(id, *min_degrees, *max_degrees)
             ? SERVO_BUS_STATUS_OK
             : mapLastDriverError();
}

servo_bus_status_t servo_bus_set_voltage_limits(uint8_t id, int16_t min_voltage_mv,
                                                int16_t max_voltage_mv) {
  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }
  g_driver.setVoltageLimits(id, min_voltage_mv, max_voltage_mv);
  return SERVO_BUS_STATUS_OK;
}

servo_bus_status_t servo_bus_get_voltage_limits(uint8_t id, int16_t* min_voltage_mv,
                                                int16_t* max_voltage_mv) {
  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }
  if (min_voltage_mv == nullptr || max_voltage_mv == nullptr) {
    return SERVO_BUS_STATUS_ERROR_INVALID_ARGUMENT;
  }
  return g_driver.getVoltageLimits(id, *min_voltage_mv, *max_voltage_mv)
             ? SERVO_BUS_STATUS_OK
             : mapLastDriverError();
}

servo_bus_status_t servo_bus_set_max_temperature_limit(uint8_t id,
                                                       uint8_t max_temp_c) {
  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }
  g_driver.setMaxTemperatureLimit(id, max_temp_c);
  return SERVO_BUS_STATUS_OK;
}

servo_bus_status_t servo_bus_get_max_temperature_limit(uint8_t id,
                                                       uint8_t* max_temp_c) {
  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }
  if (max_temp_c == nullptr) {
    return SERVO_BUS_STATUS_ERROR_INVALID_ARGUMENT;
  }
  *max_temp_c = g_driver.getMaxTemperatureLimit(id);
  return *max_temp_c == 0 ? mapLastDriverError() : SERVO_BUS_STATUS_OK;
}

servo_bus_status_t servo_bus_get_temperature(uint8_t id, int8_t* temperature_c) {
  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }
  if (temperature_c == nullptr) {
    return SERVO_BUS_STATUS_ERROR_INVALID_ARGUMENT;
  }
  *temperature_c = g_driver.getTemperature(id);
  return *temperature_c == arm::kServoErrorTimeout ? mapLastDriverError()
                                                   : SERVO_BUS_STATUS_OK;
}

servo_bus_status_t servo_bus_get_voltage(uint8_t id, int16_t* voltage_mv) {
  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }
  if (voltage_mv == nullptr) {
    return SERVO_BUS_STATUS_ERROR_INVALID_ARGUMENT;
  }
  *voltage_mv = g_driver.getVoltage(id);
  return *voltage_mv == arm::kServoErrorTimeout ? mapLastDriverError()
                                                : SERVO_BUS_STATUS_OK;
}

servo_bus_status_t servo_bus_get_position_raw(uint8_t id, int16_t* position) {
  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }
  if (position == nullptr) {
    return SERVO_BUS_STATUS_ERROR_INVALID_ARGUMENT;
  }
  *position = g_driver.getPosition(id);
  return *position == arm::kServoPositionInvalidRaw ? mapLastDriverError()
                                                    : SERVO_BUS_STATUS_OK;
}

servo_bus_status_t servo_bus_get_position_degrees(uint8_t id, float* degrees) {
  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }
  if (degrees == nullptr) {
    return SERVO_BUS_STATUS_ERROR_INVALID_ARGUMENT;
  }
  *degrees = g_driver.getPositionInDegrees(id);
  return *degrees == arm::kServoPositionInvalidDegrees ? mapLastDriverError()
                                                       : SERVO_BUS_STATUS_OK;
}

servo_bus_status_t servo_bus_set_servo_mode(uint8_t id) {
  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }
  g_driver.setServoMode(id);
  return SERVO_BUS_STATUS_OK;
}

servo_bus_status_t servo_bus_set_motor_mode(uint8_t id, int16_t speed) {
  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }
  g_driver.setMotorMode(id, speed);
  return SERVO_BUS_STATUS_OK;
}

servo_bus_status_t servo_bus_get_mode(uint8_t id, servo_bus_mode_state_t* mode_state) {
  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }
  if (mode_state == nullptr) {
    return SERVO_BUS_STATUS_ERROR_INVALID_ARGUMENT;
  }
  return g_driver.getMode(id, mode_state->mode, mode_state->speed) ? SERVO_BUS_STATUS_OK
                                                                    : mapLastDriverError();
}

servo_bus_status_t servo_bus_motor_on(uint8_t id) {
  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }
  g_driver.motorOn(id);
  return SERVO_BUS_STATUS_OK;
}

servo_bus_status_t servo_bus_motor_off(uint8_t id) {
  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }
  g_driver.motorOff(id);
  return SERVO_BUS_STATUS_OK;
}

servo_bus_status_t servo_bus_is_motor_on(uint8_t id, bool* is_on) {
  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }
  if (is_on == nullptr) {
    return SERVO_BUS_STATUS_ERROR_INVALID_ARGUMENT;
  }
  *is_on = g_driver.isMotorOn(id);
  return g_driver.getLastPacketError() == 0 ? SERVO_BUS_STATUS_OK : mapLastDriverError();
}

servo_bus_status_t servo_bus_led_on(uint8_t id) {
  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }
  g_driver.ledOn(id);
  return SERVO_BUS_STATUS_OK;
}

servo_bus_status_t servo_bus_led_off(uint8_t id) {
  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }
  g_driver.ledOff(id);
  return SERVO_BUS_STATUS_OK;
}

servo_bus_status_t servo_bus_set_led_errors(uint8_t id, uint8_t error_mask) {
  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }
  g_driver.setLedErrors(id, error_mask);
  return SERVO_BUS_STATUS_OK;
}

servo_bus_status_t servo_bus_get_led_errors(uint8_t id, uint8_t* error_mask) {
  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }
  if (error_mask == nullptr) {
    return SERVO_BUS_STATUS_ERROR_INVALID_ARGUMENT;
  }
  *error_mask = g_driver.getLedErrors(id);
  return *error_mask == arm::kServoIdInvalid ? mapLastDriverError()
                                             : SERVO_BUS_STATUS_OK;
}

servo_bus_status_t servo_bus_move_multiple(const uint8_t* ids, const int16_t* positions,
                                           size_t count, uint16_t time_ms) {
  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }
  if (ids == nullptr || positions == nullptr || count == 0) {
    return SERVO_BUS_STATUS_ERROR_INVALID_ARGUMENT;
  }
  return g_driver.moveMultiple(ids, positions, time_ms, static_cast<int>(count))
             ? SERVO_BUS_STATUS_OK
             : SERVO_BUS_STATUS_ERROR_INVALID_ARGUMENT;
}

int servo_bus_get_last_driver_error(void) {
  return g_driver.getLastPacketError();
}

}  // extern "C"