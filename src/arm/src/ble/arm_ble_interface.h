#ifndef ARM_BLE_INTERFACE_H
#define ARM_BLE_INTERFACE_H

#include <stdbool.h>
#include <stdint.h>

#include "pico/util/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ARM_BLE_JOINT_COUNT 6u
#define ARM_BLE_CARTESIAN_AXIS_COUNT 3u

typedef enum arm_ble_control_mode {
  ARM_BLE_CONTROL_MODE_JOINT = 0,
  ARM_BLE_CONTROL_MODE_CARTESIAN = 1,
} arm_ble_control_mode_t;

typedef enum arm_ble_event_type {
  ARM_BLE_EVENT_SET_TARGETS = 1,
  ARM_BLE_EVENT_SAFE_POSE = 2,
  ARM_BLE_EVENT_SET_CONTROL_MODE = 3,
  ARM_BLE_EVENT_SET_CARTESIAN_TARGET = 4,
} arm_ble_event_type_t;

typedef struct arm_ble_event {
  arm_ble_event_type_t type;
  arm_ble_control_mode_t control_mode;
  int16_t joint_target_deci_deg[ARM_BLE_JOINT_COUNT];
  int16_t cartesian_target_mm[ARM_BLE_CARTESIAN_AXIS_COUNT];
} arm_ble_event_t;

bool arm_ble_prepare(queue_t* event_queue, uint32_t owner_timeout_ms);
void arm_ble_core1_entry(void);

#ifdef __cplusplus
}
#endif

#endif
