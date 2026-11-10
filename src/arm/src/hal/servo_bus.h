#ifndef ARM_HAL_SERVO_BUS_H
#define ARM_HAL_SERVO_BUS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum servo_bus_status {
  SERVO_BUS_STATUS_OK = 0,
  SERVO_BUS_STATUS_ERROR_NOT_INITIALIZED = -1,
  SERVO_BUS_STATUS_ERROR_INVALID_ARGUMENT = -2,
  SERVO_BUS_STATUS_ERROR_TIMEOUT = -3,
  SERVO_BUS_STATUS_ERROR_OVERFLOW = -4,
  SERVO_BUS_STATUS_ERROR_PROTOCOL = -5,
  SERVO_BUS_STATUS_ERROR_UNSUPPORTED = -6,
} servo_bus_status_t;

typedef enum servo_bus_backend {
  SERVO_BUS_BACKEND_DIRECT_SERVO = 0,
  SERVO_BUS_BACKEND_XARM_CONTROLLER = 1,
} servo_bus_backend_t;

typedef struct servo_bus_config {
  servo_bus_backend_t backend;
  uint8_t uart_index;
  uint32_t baud_rate;
  uint8_t tx_pin;
  uint8_t rx_pin;
  uint8_t default_servo_id;
  uint16_t default_move_time_ms;
  float default_step_degrees;
  // Active-HIGH buffer OEs. Ignored by the controller-board backend.
  uint8_t tx_enable_pin;
  uint8_t rx_enable_pin;
} servo_bus_config_t;

typedef struct servo_bus_mode_state {
  uint8_t mode;
  int16_t speed;
} servo_bus_mode_state_t;

void servo_bus_get_default_config(servo_bus_config_t* config);
const servo_bus_config_t* servo_bus_get_active_config(void);
bool servo_bus_is_initialized(void);
servo_bus_status_t servo_bus_init(const servo_bus_config_t* config);

servo_bus_status_t servo_bus_ping(uint8_t id);
servo_bus_status_t servo_bus_move_raw(uint8_t id, int16_t position, uint16_t time_ms);
servo_bus_status_t servo_bus_move_degrees(uint8_t id, float degrees, uint16_t time_ms);
servo_bus_status_t servo_bus_stop(uint8_t id);

servo_bus_status_t servo_bus_move_wait_raw(uint8_t id, int16_t position, uint16_t time_ms);
servo_bus_status_t servo_bus_move_wait_degrees(uint8_t id, float degrees, uint16_t time_ms);
servo_bus_status_t servo_bus_start_move(uint8_t id);
servo_bus_status_t servo_bus_stop_move(uint8_t id);
servo_bus_status_t servo_bus_read_wait_position(uint8_t id, int16_t* position,
                                                uint16_t* time_ms);
servo_bus_status_t servo_bus_read_wait_position_degrees(uint8_t id, float* degrees,
                                                        uint16_t* time_ms);

servo_bus_status_t servo_bus_broadcast_move_raw(int16_t position, uint16_t time_ms);
servo_bus_status_t servo_bus_broadcast_move_degrees(float degrees, uint16_t time_ms);
servo_bus_status_t servo_bus_broadcast_stop(void);
servo_bus_status_t servo_bus_broadcast_led_on(void);
servo_bus_status_t servo_bus_broadcast_led_off(void);

servo_bus_status_t servo_bus_set_id(uint8_t old_id, uint8_t new_id);
servo_bus_status_t servo_bus_get_id(uint8_t query_id, uint8_t* out_id);
servo_bus_status_t servo_bus_set_angle_offset(uint8_t id, int8_t offset);
servo_bus_status_t servo_bus_save_angle_offset(uint8_t id);
servo_bus_status_t servo_bus_get_angle_offset(uint8_t id, int8_t* offset);

servo_bus_status_t servo_bus_set_angle_limits_raw(uint8_t id, int16_t min_angle,
                                                  int16_t max_angle);
servo_bus_status_t servo_bus_get_angle_limits_raw(uint8_t id, int16_t* min_angle,
                                                  int16_t* max_angle);
servo_bus_status_t servo_bus_set_angle_limits_degrees(uint8_t id, float min_degrees,
                                                      float max_degrees);
servo_bus_status_t servo_bus_get_angle_limits_degrees(uint8_t id, float* min_degrees,
                                                      float* max_degrees);

servo_bus_status_t servo_bus_set_voltage_limits(uint8_t id, int16_t min_voltage_mv,
                                                int16_t max_voltage_mv);
servo_bus_status_t servo_bus_get_voltage_limits(uint8_t id, int16_t* min_voltage_mv,
                                                int16_t* max_voltage_mv);
servo_bus_status_t servo_bus_set_max_temperature_limit(uint8_t id, uint8_t max_temp_c);
servo_bus_status_t servo_bus_get_max_temperature_limit(uint8_t id, uint8_t* max_temp_c);

servo_bus_status_t servo_bus_get_temperature(uint8_t id, int8_t* temperature_c);
servo_bus_status_t servo_bus_get_voltage(uint8_t id, int16_t* voltage_mv);
servo_bus_status_t servo_bus_get_position_raw(uint8_t id, int16_t* position);
servo_bus_status_t servo_bus_get_position_degrees(uint8_t id, float* degrees);

servo_bus_status_t servo_bus_set_servo_mode(uint8_t id);
servo_bus_status_t servo_bus_set_motor_mode(uint8_t id, int16_t speed);
servo_bus_status_t servo_bus_get_mode(uint8_t id, servo_bus_mode_state_t* mode_state);
servo_bus_status_t servo_bus_motor_on(uint8_t id);
servo_bus_status_t servo_bus_motor_off(uint8_t id);
servo_bus_status_t servo_bus_is_motor_on(uint8_t id, bool* is_on);

servo_bus_status_t servo_bus_led_on(uint8_t id);
servo_bus_status_t servo_bus_led_off(uint8_t id);
servo_bus_status_t servo_bus_set_led_errors(uint8_t id, uint8_t error_mask);
servo_bus_status_t servo_bus_get_led_errors(uint8_t id, uint8_t* error_mask);

servo_bus_status_t servo_bus_move_multiple(const uint8_t* ids, const int16_t* positions,
                                           size_t count, uint16_t time_ms);

int servo_bus_get_last_driver_error(void);

#ifdef __cplusplus
}
#endif

#endif
