#include "hal/servo_bus.h"

#include "drivers/servo_bus_driver.hpp"
#include "drivers/xarm_controller_driver.hpp"
#include "platform/pico_uart_transport.hpp"
#include "platform/servo_platform_config.h"

static arm::PicoUartTransport g_transport;
static arm::ServoBusDriver g_direct_driver(g_transport);
static arm::XArmControllerDriver g_controller_driver(g_transport);
static servo_bus_config_t g_active_config = {};
static bool g_initialized = false;

static bool usingControllerBackend(void) {
  return g_active_config.backend == SERVO_BUS_BACKEND_XARM_CONTROLLER;
}

static servo_bus_status_t mapDirectDriverError(void) {
  switch (g_direct_driver.getLastPacketError()) {
    case arm::kServoErrorTimeout:
      return SERVO_BUS_STATUS_ERROR_TIMEOUT;
    case arm::kServoErrorOverflow:
      return SERVO_BUS_STATUS_ERROR_OVERFLOW;
    default:
      return SERVO_BUS_STATUS_ERROR_PROTOCOL;
  }
}

static servo_bus_status_t mapControllerDriverError(void) {
  switch (g_controller_driver.getLastPacketError()) {
    case arm::kControllerErrorTimeout:
      return SERVO_BUS_STATUS_ERROR_TIMEOUT;
    case arm::kControllerErrorOverflow:
      return SERVO_BUS_STATUS_ERROR_OVERFLOW;
    case arm::kControllerErrorInvalidArgument:
      return SERVO_BUS_STATUS_ERROR_INVALID_ARGUMENT;
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

  config->backend = static_cast<servo_bus_backend_t>(SERVO_BUS_DEFAULT_BACKEND);
  if (config->backend != SERVO_BUS_BACKEND_DIRECT_SERVO &&
      config->backend != SERVO_BUS_BACKEND_XARM_CONTROLLER) {
    config->backend = SERVO_BUS_BACKEND_XARM_CONTROLLER;
  }

  config->uart_index = SERVO_BUS_DEFAULT_UART_INDEX;
  config->baud_rate =
      config->backend == SERVO_BUS_BACKEND_DIRECT_SERVO
          ? SERVO_BUS_DIRECT_DEFAULT_BAUD_RATE
          : SERVO_BUS_CONTROLLER_DEFAULT_BAUD_RATE;
  config->tx_pin = SERVO_BUS_DEFAULT_TX_PIN;
  config->rx_pin = SERVO_BUS_DEFAULT_RX_PIN;
  config->tx_enable_pin = SERVO_BUS_DIRECT_DEFAULT_TX_ENABLE_PIN;
  config->rx_enable_pin = SERVO_BUS_DIRECT_DEFAULT_RX_ENABLE_PIN;
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

  if (config->backend != SERVO_BUS_BACKEND_DIRECT_SERVO &&
      config->backend != SERVO_BUS_BACKEND_XARM_CONTROLLER) {
    return SERVO_BUS_STATUS_ERROR_INVALID_ARGUMENT;
  }

  uint32_t baudRate = config->baud_rate;
  if (baudRate == 0) {
    baudRate = config->backend == SERVO_BUS_BACKEND_DIRECT_SERVO
                   ? SERVO_BUS_DIRECT_DEFAULT_BAUD_RATE
                   : SERVO_BUS_CONTROLLER_DEFAULT_BAUD_RATE;
  }

  const bool direct = config->backend == SERVO_BUS_BACKEND_DIRECT_SERVO;
  if (direct && (config->tx_enable_pin == SERVO_BUS_HEARTBEAT_LED_PIN ||
                 config->rx_enable_pin == SERVO_BUS_HEARTBEAT_LED_PIN ||
                 config->tx_enable_pin == arm::PicoUartTransport::kNoEnablePin ||
                 config->rx_enable_pin == arm::PicoUartTransport::kNoEnablePin)) {
    return SERVO_BUS_STATUS_ERROR_INVALID_ARGUMENT;
  }

  if (!g_transport.init(config->uart_index, baudRate, config->tx_pin,
                        config->rx_pin,
                        direct ? config->tx_enable_pin : arm::PicoUartTransport::kNoEnablePin,
                        direct ? config->rx_enable_pin : arm::PicoUartTransport::kNoEnablePin)) {
    g_initialized = g_transport.initialized();
    return SERVO_BUS_STATUS_ERROR_PROTOCOL;
  }

  g_active_config = *config;
  g_active_config.baud_rate = baudRate;
  g_initialized = true;
  return SERVO_BUS_STATUS_OK;
}

servo_bus_status_t servo_bus_ping(uint8_t id) {
  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }

  if (usingControllerBackend()) {
    int16_t position = 0;
    return g_controller_driver.readPosition(id, position) ? SERVO_BUS_STATUS_OK
                                                          : mapControllerDriverError();
  }

  return g_direct_driver.isConnected(id) ? SERVO_BUS_STATUS_OK : mapDirectDriverError();
}

servo_bus_status_t servo_bus_move_raw(uint8_t id, int16_t position, uint16_t time_ms) {
  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }

  if (usingControllerBackend()) {
    return g_controller_driver.move(id, position, time_ms) ? SERVO_BUS_STATUS_OK
                                                           : mapControllerDriverError();
  }

  g_direct_driver.move(id, position, time_ms);
  return SERVO_BUS_STATUS_OK;
}

servo_bus_status_t servo_bus_move_degrees(uint8_t id, float degrees, uint16_t time_ms) {
  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }

  if (usingControllerBackend()) {
    const int16_t position = g_controller_driver.degreesToPosition(degrees);
    return g_controller_driver.move(id, position, time_ms) ? SERVO_BUS_STATUS_OK
                                                           : mapControllerDriverError();
  }

  g_direct_driver.moveTo(id, degrees, time_ms);
  return SERVO_BUS_STATUS_OK;
}

servo_bus_status_t servo_bus_stop(uint8_t id) {
  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }

  if (usingControllerBackend()) {
    return g_controller_driver.unload(id) ? SERVO_BUS_STATUS_OK
                                          : mapControllerDriverError();
  }

  g_direct_driver.stop(id);
  return SERVO_BUS_STATUS_OK;
}

servo_bus_status_t servo_bus_move_wait_raw(uint8_t id, int16_t position,
                                           uint16_t time_ms) {
  (void)id;
  (void)position;
  (void)time_ms;

  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }

  if (usingControllerBackend()) {
    return SERVO_BUS_STATUS_ERROR_UNSUPPORTED;
  }

  g_direct_driver.moveWait(id, position, time_ms);
  return SERVO_BUS_STATUS_OK;
}

servo_bus_status_t servo_bus_move_wait_degrees(uint8_t id, float degrees,
                                               uint16_t time_ms) {
  (void)id;
  (void)degrees;
  (void)time_ms;

  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }

  if (usingControllerBackend()) {
    return SERVO_BUS_STATUS_ERROR_UNSUPPORTED;
  }

  g_direct_driver.moveWaitDegrees(id, degrees, time_ms);
  return SERVO_BUS_STATUS_OK;
}

servo_bus_status_t servo_bus_start_move(uint8_t id) {
  (void)id;

  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }

  if (usingControllerBackend()) {
    return SERVO_BUS_STATUS_ERROR_UNSUPPORTED;
  }

  g_direct_driver.startMove(id);
  return SERVO_BUS_STATUS_OK;
}

servo_bus_status_t servo_bus_stop_move(uint8_t id) {
  (void)id;

  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }

  if (usingControllerBackend()) {
    return SERVO_BUS_STATUS_ERROR_UNSUPPORTED;
  }

  g_direct_driver.stopMove(id);
  return SERVO_BUS_STATUS_OK;
}

servo_bus_status_t servo_bus_read_wait_position(uint8_t id, int16_t* position,
                                                uint16_t* time_ms) {
  (void)id;

  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }
  if (position == nullptr || time_ms == nullptr) {
    return SERVO_BUS_STATUS_ERROR_INVALID_ARGUMENT;
  }

  if (usingControllerBackend()) {
    return SERVO_BUS_STATUS_ERROR_UNSUPPORTED;
  }

  return g_direct_driver.readWaitPosition(id, *position, *time_ms)
             ? SERVO_BUS_STATUS_OK
             : mapDirectDriverError();
}

servo_bus_status_t servo_bus_read_wait_position_degrees(uint8_t id, float* degrees,
                                                        uint16_t* time_ms) {
  (void)id;

  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }
  if (degrees == nullptr || time_ms == nullptr) {
    return SERVO_BUS_STATUS_ERROR_INVALID_ARGUMENT;
  }

  if (usingControllerBackend()) {
    return SERVO_BUS_STATUS_ERROR_UNSUPPORTED;
  }

  return g_direct_driver.readWaitPositionInDegrees(id, *degrees, *time_ms)
             ? SERVO_BUS_STATUS_OK
             : mapDirectDriverError();
}

servo_bus_status_t servo_bus_broadcast_move_raw(int16_t position, uint16_t time_ms) {
  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }

  if (usingControllerBackend()) {
    return g_controller_driver.move(254, position, time_ms) ? SERVO_BUS_STATUS_OK
                                                            : mapControllerDriverError();
  }

  g_direct_driver.broadcastMove(position, time_ms);
  return SERVO_BUS_STATUS_OK;
}

servo_bus_status_t servo_bus_broadcast_move_degrees(float degrees, uint16_t time_ms) {
  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }

  if (usingControllerBackend()) {
    const int16_t position = g_controller_driver.degreesToPosition(degrees);
    return g_controller_driver.move(254, position, time_ms) ? SERVO_BUS_STATUS_OK
                                                            : mapControllerDriverError();
  }

  g_direct_driver.broadcastMoveDegrees(degrees, time_ms);
  return SERVO_BUS_STATUS_OK;
}

servo_bus_status_t servo_bus_broadcast_stop(void) {
  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }

  if (usingControllerBackend()) {
    return g_controller_driver.unload(254) ? SERVO_BUS_STATUS_OK
                                           : mapControllerDriverError();
  }

  g_direct_driver.broadcastStop();
  return SERVO_BUS_STATUS_OK;
}

servo_bus_status_t servo_bus_broadcast_led_on(void) {
  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }

  if (usingControllerBackend()) {
    return SERVO_BUS_STATUS_ERROR_UNSUPPORTED;
  }

  g_direct_driver.broadcastLedOn();
  return SERVO_BUS_STATUS_OK;
}

servo_bus_status_t servo_bus_broadcast_led_off(void) {
  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }

  if (usingControllerBackend()) {
    return SERVO_BUS_STATUS_ERROR_UNSUPPORTED;
  }

  g_direct_driver.broadcastLedOff();
  return SERVO_BUS_STATUS_OK;
}

servo_bus_status_t servo_bus_set_id(uint8_t old_id, uint8_t new_id) {
  (void)old_id;
  (void)new_id;

  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }

  if (usingControllerBackend()) {
    return SERVO_BUS_STATUS_ERROR_UNSUPPORTED;
  }

  g_direct_driver.setId(old_id, new_id);
  return SERVO_BUS_STATUS_OK;
}

servo_bus_status_t servo_bus_get_id(uint8_t query_id, uint8_t* out_id) {
  (void)query_id;

  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }
  if (out_id == nullptr) {
    return SERVO_BUS_STATUS_ERROR_INVALID_ARGUMENT;
  }

  if (usingControllerBackend()) {
    return SERVO_BUS_STATUS_ERROR_UNSUPPORTED;
  }

  *out_id = g_direct_driver.getId(query_id);
  return *out_id == arm::kServoIdInvalid ? mapDirectDriverError() : SERVO_BUS_STATUS_OK;
}

servo_bus_status_t servo_bus_set_angle_offset(uint8_t id, int8_t offset) {
  (void)id;
  (void)offset;

  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }

  if (usingControllerBackend()) {
    return SERVO_BUS_STATUS_ERROR_UNSUPPORTED;
  }

  g_direct_driver.setAngleOffset(id, offset);
  return SERVO_BUS_STATUS_OK;
}

servo_bus_status_t servo_bus_save_angle_offset(uint8_t id) {
  (void)id;

  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }

  if (usingControllerBackend()) {
    return SERVO_BUS_STATUS_ERROR_UNSUPPORTED;
  }

  g_direct_driver.saveAngleOffset(id);
  return SERVO_BUS_STATUS_OK;
}

servo_bus_status_t servo_bus_get_angle_offset(uint8_t id, int8_t* offset) {
  (void)id;

  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }
  if (offset == nullptr) {
    return SERVO_BUS_STATUS_ERROR_INVALID_ARGUMENT;
  }

  if (usingControllerBackend()) {
    return SERVO_BUS_STATUS_ERROR_UNSUPPORTED;
  }

  *offset = g_direct_driver.getAngleOffset(id);
  return *offset == static_cast<int8_t>(arm::kServoOffsetInvalid)
             ? mapDirectDriverError()
             : SERVO_BUS_STATUS_OK;
}

servo_bus_status_t servo_bus_set_angle_limits_raw(uint8_t id, int16_t min_angle,
                                                  int16_t max_angle) {
  (void)id;
  (void)min_angle;
  (void)max_angle;

  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }

  if (usingControllerBackend()) {
    return SERVO_BUS_STATUS_ERROR_UNSUPPORTED;
  }

  g_direct_driver.setAngleLimits(id, min_angle, max_angle);
  return SERVO_BUS_STATUS_OK;
}

servo_bus_status_t servo_bus_get_angle_limits_raw(uint8_t id, int16_t* min_angle,
                                                  int16_t* max_angle) {
  (void)id;

  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }
  if (min_angle == nullptr || max_angle == nullptr) {
    return SERVO_BUS_STATUS_ERROR_INVALID_ARGUMENT;
  }

  if (usingControllerBackend()) {
    return SERVO_BUS_STATUS_ERROR_UNSUPPORTED;
  }

  return g_direct_driver.getAngleLimits(id, *min_angle, *max_angle)
             ? SERVO_BUS_STATUS_OK
             : mapDirectDriverError();
}

servo_bus_status_t servo_bus_set_angle_limits_degrees(uint8_t id, float min_degrees,
                                                      float max_degrees) {
  (void)id;
  (void)min_degrees;
  (void)max_degrees;

  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }

  if (usingControllerBackend()) {
    return SERVO_BUS_STATUS_ERROR_UNSUPPORTED;
  }

  g_direct_driver.setAngleLimitsInDegrees(id, min_degrees, max_degrees);
  return SERVO_BUS_STATUS_OK;
}

servo_bus_status_t servo_bus_get_angle_limits_degrees(uint8_t id, float* min_degrees,
                                                      float* max_degrees) {
  (void)id;

  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }
  if (min_degrees == nullptr || max_degrees == nullptr) {
    return SERVO_BUS_STATUS_ERROR_INVALID_ARGUMENT;
  }

  if (usingControllerBackend()) {
    return SERVO_BUS_STATUS_ERROR_UNSUPPORTED;
  }

  return g_direct_driver.getAngleLimitsInDegrees(id, *min_degrees, *max_degrees)
             ? SERVO_BUS_STATUS_OK
             : mapDirectDriverError();
}

servo_bus_status_t servo_bus_set_voltage_limits(uint8_t id, int16_t min_voltage_mv,
                                                int16_t max_voltage_mv) {
  (void)id;
  (void)min_voltage_mv;
  (void)max_voltage_mv;

  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }

  if (usingControllerBackend()) {
    return SERVO_BUS_STATUS_ERROR_UNSUPPORTED;
  }

  g_direct_driver.setVoltageLimits(id, min_voltage_mv, max_voltage_mv);
  return SERVO_BUS_STATUS_OK;
}

servo_bus_status_t servo_bus_get_voltage_limits(uint8_t id, int16_t* min_voltage_mv,
                                                int16_t* max_voltage_mv) {
  (void)id;

  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }
  if (min_voltage_mv == nullptr || max_voltage_mv == nullptr) {
    return SERVO_BUS_STATUS_ERROR_INVALID_ARGUMENT;
  }

  if (usingControllerBackend()) {
    return SERVO_BUS_STATUS_ERROR_UNSUPPORTED;
  }

  return g_direct_driver.getVoltageLimits(id, *min_voltage_mv, *max_voltage_mv)
             ? SERVO_BUS_STATUS_OK
             : mapDirectDriverError();
}

servo_bus_status_t servo_bus_set_max_temperature_limit(uint8_t id,
                                                       uint8_t max_temp_c) {
  (void)id;
  (void)max_temp_c;

  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }

  if (usingControllerBackend()) {
    return SERVO_BUS_STATUS_ERROR_UNSUPPORTED;
  }

  g_direct_driver.setMaxTemperatureLimit(id, max_temp_c);
  return SERVO_BUS_STATUS_OK;
}

servo_bus_status_t servo_bus_get_max_temperature_limit(uint8_t id,
                                                       uint8_t* max_temp_c) {
  (void)id;

  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }
  if (max_temp_c == nullptr) {
    return SERVO_BUS_STATUS_ERROR_INVALID_ARGUMENT;
  }

  if (usingControllerBackend()) {
    return SERVO_BUS_STATUS_ERROR_UNSUPPORTED;
  }

  *max_temp_c = g_direct_driver.getMaxTemperatureLimit(id);
  return *max_temp_c == 0 ? mapDirectDriverError() : SERVO_BUS_STATUS_OK;
}

servo_bus_status_t servo_bus_get_temperature(uint8_t id, int8_t* temperature_c) {
  (void)id;

  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }
  if (temperature_c == nullptr) {
    return SERVO_BUS_STATUS_ERROR_INVALID_ARGUMENT;
  }

  if (usingControllerBackend()) {
    return SERVO_BUS_STATUS_ERROR_UNSUPPORTED;
  }

  *temperature_c = g_direct_driver.getTemperature(id);
  return *temperature_c == arm::kServoErrorTimeout ? mapDirectDriverError()
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

  if (usingControllerBackend()) {
    int16_t controllerVoltageMv = 0;
    if (!g_controller_driver.getBatteryVoltageMv(controllerVoltageMv)) {
      return mapControllerDriverError();
    }
    *voltage_mv = controllerVoltageMv;
    return SERVO_BUS_STATUS_OK;
  }

  *voltage_mv = g_direct_driver.getVoltage(id);
  return *voltage_mv == arm::kServoErrorTimeout ? mapDirectDriverError()
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

  if (usingControllerBackend()) {
    int16_t rawPosition = 0;
    if (!g_controller_driver.readPosition(id, rawPosition)) {
      return mapControllerDriverError();
    }
    *position = rawPosition;
    return SERVO_BUS_STATUS_OK;
  }

  *position = g_direct_driver.getPosition(id);
  return *position == arm::kServoPositionInvalidRaw ? mapDirectDriverError()
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

  if (usingControllerBackend()) {
    int16_t rawPosition = 0;
    if (!g_controller_driver.readPosition(id, rawPosition)) {
      return mapControllerDriverError();
    }
    *degrees = g_controller_driver.positionToDegrees(rawPosition);
    return SERVO_BUS_STATUS_OK;
  }

  *degrees = g_direct_driver.getPositionInDegrees(id);
  return *degrees == arm::kServoPositionInvalidDegrees ? mapDirectDriverError()
                                                       : SERVO_BUS_STATUS_OK;
}

servo_bus_status_t servo_bus_set_servo_mode(uint8_t id) {
  (void)id;

  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }

  if (usingControllerBackend()) {
    return SERVO_BUS_STATUS_ERROR_UNSUPPORTED;
  }

  g_direct_driver.setServoMode(id);
  return SERVO_BUS_STATUS_OK;
}

servo_bus_status_t servo_bus_set_motor_mode(uint8_t id, int16_t speed) {
  (void)id;
  (void)speed;

  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }

  if (usingControllerBackend()) {
    return SERVO_BUS_STATUS_ERROR_UNSUPPORTED;
  }

  g_direct_driver.setMotorMode(id, speed);
  return SERVO_BUS_STATUS_OK;
}

servo_bus_status_t servo_bus_get_mode(uint8_t id, servo_bus_mode_state_t* mode_state) {
  (void)id;

  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }
  if (mode_state == nullptr) {
    return SERVO_BUS_STATUS_ERROR_INVALID_ARGUMENT;
  }

  if (usingControllerBackend()) {
    return SERVO_BUS_STATUS_ERROR_UNSUPPORTED;
  }

  return g_direct_driver.getMode(id, mode_state->mode, mode_state->speed)
             ? SERVO_BUS_STATUS_OK
             : mapDirectDriverError();
}

servo_bus_status_t servo_bus_motor_on(uint8_t id) {
  (void)id;

  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }

  if (usingControllerBackend()) {
    return SERVO_BUS_STATUS_ERROR_UNSUPPORTED;
  }

  g_direct_driver.motorOn(id);
  return SERVO_BUS_STATUS_OK;
}

servo_bus_status_t servo_bus_motor_off(uint8_t id) {
  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }

  if (usingControllerBackend()) {
    return g_controller_driver.unload(id) ? SERVO_BUS_STATUS_OK
                                          : mapControllerDriverError();
  }

  g_direct_driver.motorOff(id);
  return SERVO_BUS_STATUS_OK;
}

servo_bus_status_t servo_bus_is_motor_on(uint8_t id, bool* is_on) {
  (void)id;

  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }
  if (is_on == nullptr) {
    return SERVO_BUS_STATUS_ERROR_INVALID_ARGUMENT;
  }

  if (usingControllerBackend()) {
    return SERVO_BUS_STATUS_ERROR_UNSUPPORTED;
  }

  *is_on = g_direct_driver.isMotorOn(id);
  return g_direct_driver.getLastPacketError() == 0 ? SERVO_BUS_STATUS_OK
                                                    : mapDirectDriverError();
}

servo_bus_status_t servo_bus_led_on(uint8_t id) {
  (void)id;

  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }

  if (usingControllerBackend()) {
    return SERVO_BUS_STATUS_ERROR_UNSUPPORTED;
  }

  g_direct_driver.ledOn(id);
  return SERVO_BUS_STATUS_OK;
}

servo_bus_status_t servo_bus_led_off(uint8_t id) {
  (void)id;

  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }

  if (usingControllerBackend()) {
    return SERVO_BUS_STATUS_ERROR_UNSUPPORTED;
  }

  g_direct_driver.ledOff(id);
  return SERVO_BUS_STATUS_OK;
}

servo_bus_status_t servo_bus_set_led_errors(uint8_t id, uint8_t error_mask) {
  (void)id;
  (void)error_mask;

  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }

  if (usingControllerBackend()) {
    return SERVO_BUS_STATUS_ERROR_UNSUPPORTED;
  }

  g_direct_driver.setLedErrors(id, error_mask);
  return SERVO_BUS_STATUS_OK;
}

servo_bus_status_t servo_bus_get_led_errors(uint8_t id, uint8_t* error_mask) {
  (void)id;

  const servo_bus_status_t status = requireInitialized();
  if (status != SERVO_BUS_STATUS_OK) {
    return status;
  }
  if (error_mask == nullptr) {
    return SERVO_BUS_STATUS_ERROR_INVALID_ARGUMENT;
  }

  if (usingControllerBackend()) {
    return SERVO_BUS_STATUS_ERROR_UNSUPPORTED;
  }

  *error_mask = g_direct_driver.getLedErrors(id);
  return *error_mask == arm::kServoIdInvalid ? mapDirectDriverError()
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

  if (usingControllerBackend()) {
    return g_controller_driver.moveMultiple(ids, positions, count, time_ms)
               ? SERVO_BUS_STATUS_OK
               : mapControllerDriverError();
  }

  return g_direct_driver.moveMultiple(ids, positions, time_ms, static_cast<int>(count))
             ? SERVO_BUS_STATUS_OK
             : SERVO_BUS_STATUS_ERROR_INVALID_ARGUMENT;
}

int servo_bus_get_last_driver_error(void) {
  if (!g_initialized) {
    return 0;
  }

  return usingControllerBackend() ? g_controller_driver.getLastPacketError()
                                  : g_direct_driver.getLastPacketError();
}

}  // extern "C"
