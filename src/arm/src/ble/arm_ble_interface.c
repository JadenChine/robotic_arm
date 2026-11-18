#include "ble/arm_ble_interface.h"

#include <stdio.h>
#include <string.h>

#include "arm_control_service.h"
#include "btstack.h"
#include "pico/cyw43_arch.h"
#include "pico/rand.h"
#include "pico/stdlib.h"
#include "platform/servo_platform_config.h"

#define ARM_BLE_MAX_CONNECTIONS 2u
#define ARM_BLE_CONTROL_CMD_TAKE_CONTROL 0x01u
#define ARM_BLE_CONTROL_CMD_RELEASE_CONTROL 0x02u
#define ARM_BLE_CONTROL_CMD_SET_MODE 0x03u
#define ARM_BLE_TARGET_PACKET_SIZE_BYTES (ARM_BLE_JOINT_COUNT * sizeof(int16_t))
#define ARM_BLE_CARTESIAN_PACKET_SIZE_BYTES (ARM_BLE_CARTESIAN_AXIS_COUNT * sizeof(int16_t))
#define BLUETOOTH_LED_SLOW_TOGGLE_MS 500u
#define BLUETOOTH_LED_FAST_TOGGLE_MS 100u
#define BLUETOOTH_LED_TRANSITION_MS 1500u

typedef struct arm_ble_connection_slot {
  bool in_use;
  hci_con_handle_t handle;
} arm_ble_connection_slot_t;

static queue_t* g_event_queue = NULL;

static arm_ble_connection_slot_t g_connections[ARM_BLE_MAX_CONNECTIONS] = {0};

static hci_con_handle_t g_owner_handle = HCI_CON_HANDLE_INVALID;

static btstack_packet_callback_registration_t g_hci_event_callback_registration;
static btstack_timer_source_t g_bluetooth_led_timer;
static uint32_t g_owner_change_ms;
static bool g_bluetooth_led_fast;
static bool g_bluetooth_led_on;

static uint32_t now_ms(void) {
  return to_ms_since_boot(get_absolute_time());
}

// The BLE run loop owns both the timer and the control owner.
static void update_bluetooth_led(btstack_timer_source_t* timer) {
  const bool has_owner = g_owner_handle != HCI_CON_HANDLE_INVALID;
  if (g_bluetooth_led_fast &&
      now_ms() - g_owner_change_ms >= BLUETOOTH_LED_TRANSITION_MS) {
    g_bluetooth_led_fast = false;
    g_bluetooth_led_on = true;
  } else {
    g_bluetooth_led_on = !g_bluetooth_led_on;
  }

  const bool blinking = g_bluetooth_led_fast || !has_owner;
  const bool led_on = !blinking || g_bluetooth_led_on;
  gpio_put(BOARD_BLUETOOTH_LED_PIN,
           led_on ? BOARD_STATUS_LED_ACTIVE_LEVEL : !BOARD_STATUS_LED_ACTIVE_LEVEL);
  if (blinking) {
    const uint32_t toggle_ms = g_bluetooth_led_fast ? BLUETOOTH_LED_FAST_TOGGLE_MS
                                                  : BLUETOOTH_LED_SLOW_TOGGLE_MS;
    btstack_run_loop_set_timer(timer, toggle_ms);
    btstack_run_loop_add_timer(timer);
  }
}

static void start_bluetooth_led_transition(void) {
  btstack_run_loop_remove_timer(&g_bluetooth_led_timer);
  g_owner_change_ms = now_ms();
  g_bluetooth_led_fast = true;
  g_bluetooth_led_on = false;
  update_bluetooth_led(&g_bluetooth_led_timer);
}

static void set_random_static_ble_address(void) {
  bd_addr_t random_addr;
  const uint32_t r0 = get_rand_32();
  const uint32_t r1 = get_rand_32();

  random_addr[0] = (uint8_t)(r0 & 0xffu);
  random_addr[1] = (uint8_t)((r0 >> 8) & 0xffu);
  random_addr[2] = (uint8_t)((r0 >> 16) & 0xffu);
  random_addr[3] = (uint8_t)((r0 >> 24) & 0xffu);
  random_addr[4] = (uint8_t)(r1 & 0xffu);
  random_addr[5] = (uint8_t)((r1 >> 8) & 0xffu);

  // Set two MSBs to 1 to mark this as a *static random* address.
  random_addr[5] = (uint8_t)((random_addr[5] & 0x3fu) | 0xc0u);

  gap_random_address_set(random_addr);
}

static arm_ble_connection_slot_t* find_connection(hci_con_handle_t connection_handle) {
  for (size_t index = 0; index < ARM_BLE_MAX_CONNECTIONS; ++index) {
    if (!g_connections[index].in_use) {
      continue;
    }
    if (g_connections[index].handle == connection_handle) {
      return &g_connections[index];
    }
  }

  return NULL;
}

static arm_ble_connection_slot_t* claim_connection(hci_con_handle_t connection_handle) {
  arm_ble_connection_slot_t* existing = find_connection(connection_handle);
  if (existing != NULL) {
    return existing;
  }

  for (size_t index = 0; index < ARM_BLE_MAX_CONNECTIONS; ++index) {
    if (g_connections[index].in_use) {
      continue;
    }

    g_connections[index].in_use = true;
    g_connections[index].handle = connection_handle;
    return &g_connections[index];
  }

  return NULL;
}

static void release_connection(hci_con_handle_t connection_handle) {
  arm_ble_connection_slot_t* slot = find_connection(connection_handle);
  if (slot == NULL) {
    return;
  }

  slot->in_use = false;
  slot->handle = HCI_CON_HANDLE_INVALID;
}

static bool enqueue_event(const arm_ble_event_t* event) {
  if (g_event_queue == NULL || event == NULL) {
    return false;
  }

  if (queue_try_add(g_event_queue, event)) {
    return true;
  }

  arm_ble_event_t dropped = {0};
  if (!queue_try_remove(g_event_queue, &dropped)) {
    return false;
  }

  return queue_try_add(g_event_queue, event);
}

static void clear_owner(bool enqueue_safe_pose, const char* reason) {
  if (g_owner_handle == HCI_CON_HANDLE_INVALID && !enqueue_safe_pose) {
    return;
  }

  if (g_owner_handle != HCI_CON_HANDLE_INVALID) {
    g_owner_handle = HCI_CON_HANDLE_INVALID;
    start_bluetooth_led_transition();
  }

  if (enqueue_safe_pose) {
    arm_ble_event_t safe_pose_event = {
        .type = ARM_BLE_EVENT_SAFE_POSE,
        .control_mode = ARM_BLE_CONTROL_MODE_JOINT,
        .joint_target_deci_deg = {0},
        .cartesian_target_mm = {0},
    };
    enqueue_event(&safe_pose_event);
  }

  if (reason != NULL) {
    printf("ble ownership cleared: %s\n", reason);
  }
}

static bool grant_owner(hci_con_handle_t connection_handle) {
  if (claim_connection(connection_handle) == NULL) {
    return false;
  }

  if (g_owner_handle != connection_handle) {
    g_owner_handle = connection_handle;
    start_bluetooth_led_transition();
  }

  printf("ble ownership granted to handle=%u\n", connection_handle);
  return true;
}

static int handle_control_point_write(hci_con_handle_t connection_handle,
                                      const uint8_t* buffer,
                                      uint16_t buffer_size) {
  if (buffer_size < 1) {
    return ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LENGTH;
  }

  switch (buffer[0]) {
    case ARM_BLE_CONTROL_CMD_TAKE_CONTROL:
      if (!grant_owner(connection_handle)) {
        return ATT_ERROR_UNLIKELY_ERROR;
      }
      return 0;

    case ARM_BLE_CONTROL_CMD_RELEASE_CONTROL:
      if (g_owner_handle != connection_handle) {
        return ATT_ERROR_INSUFFICIENT_AUTHORIZATION;
      }
      clear_owner(true, "released by owner");
      return 0;

    case ARM_BLE_CONTROL_CMD_SET_MODE: {
      if (g_owner_handle != connection_handle) {
        return ATT_ERROR_INSUFFICIENT_AUTHORIZATION;
      }
      if (buffer_size != 2u) {
        return ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LENGTH;
      }
      if (buffer[1] != ARM_BLE_CONTROL_MODE_JOINT &&
          buffer[1] != ARM_BLE_CONTROL_MODE_CARTESIAN) {
        return ATT_ERROR_VALUE_NOT_ALLOWED;
      }

      arm_ble_event_t mode_event = {
          .type = ARM_BLE_EVENT_SET_CONTROL_MODE,
          .control_mode = (arm_ble_control_mode_t)buffer[1],
          .joint_target_deci_deg = {0},
          .cartesian_target_mm = {0},
      };
      if (!enqueue_event(&mode_event)) {
        return ATT_ERROR_UNLIKELY_ERROR;
      }
      return 0;
    }

    default:
      return ATT_ERROR_VALUE_NOT_ALLOWED;
  }
}

static int handle_joint_targets_write(hci_con_handle_t connection_handle,
                                      const uint8_t* buffer,
                                      uint16_t buffer_size) {
  // Non-owner writes are treated as no-ops (success) to allow a second central to
  // stream targets continuously; only the owner is applied.
  if (g_owner_handle != connection_handle) {
    return 0;
  }

  if (buffer_size != ARM_BLE_TARGET_PACKET_SIZE_BYTES) {
    return ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LENGTH;
  }

  arm_ble_event_t joint_event = {
      .type = ARM_BLE_EVENT_SET_TARGETS,
      .control_mode = ARM_BLE_CONTROL_MODE_JOINT,
      .joint_target_deci_deg = {0},
      .cartesian_target_mm = {0},
  };

  for (size_t joint_index = 0; joint_index < ARM_BLE_JOINT_COUNT; ++joint_index) {
    const uint16_t raw = little_endian_read_16(buffer, (int)(joint_index * 2u));
    joint_event.joint_target_deci_deg[joint_index] = (int16_t)raw;
  }

  if (!enqueue_event(&joint_event)) {
    return ATT_ERROR_UNLIKELY_ERROR;
  }

  return 0;
}

static int handle_cartesian_targets_write(hci_con_handle_t connection_handle,
                                          const uint8_t* buffer,
                                          uint16_t buffer_size) {
  if (g_owner_handle != connection_handle) {
    return 0;
  }

  if (buffer_size != ARM_BLE_CARTESIAN_PACKET_SIZE_BYTES) {
    return ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LENGTH;
  }

  arm_ble_event_t cartesian_event = {
      .type = ARM_BLE_EVENT_SET_CARTESIAN_TARGET,
      .control_mode = ARM_BLE_CONTROL_MODE_CARTESIAN,
      .joint_target_deci_deg = {0},
      .cartesian_target_mm = {0},
  };

  for (size_t axis_index = 0; axis_index < ARM_BLE_CARTESIAN_AXIS_COUNT; ++axis_index) {
    const uint16_t raw = little_endian_read_16(buffer, (int)(axis_index * 2u));
    cartesian_event.cartesian_target_mm[axis_index] = (int16_t)raw;
  }

  if (!enqueue_event(&cartesian_event)) {
    return ATT_ERROR_UNLIKELY_ERROR;
  }

  return 0;
}

static uint16_t att_read_callback(__unused hci_con_handle_t connection_handle,
                                  uint16_t att_handle,
                                  uint16_t offset,
                                  uint8_t* buffer,
                                  uint16_t buffer_size) {
  if (att_handle ==
      ATT_CHARACTERISTIC_0000FFF2_0000_1000_8000_00805F9B34FB_01_VALUE_HANDLE) {
  // Owner status (not a token): 1 if this connection is the owner, else 0.
  const uint8_t owner_status = (connection_handle == g_owner_handle) ? 1u : 0u;
  return att_read_callback_handle_blob(&owner_status, sizeof(owner_status), offset,
                                         buffer, buffer_size);
  }

  return 0;
}

static int att_write_callback(hci_con_handle_t connection_handle,
                              uint16_t att_handle,
                              uint16_t transaction_mode,
                              uint16_t offset,
                              uint8_t* buffer,
                              uint16_t buffer_size) {
  if (transaction_mode != ATT_TRANSACTION_MODE_NONE) {
    return ATT_ERROR_WRITE_NOT_PERMITTED;
  }

  if (offset != 0) {
    return ATT_ERROR_INVALID_OFFSET;
  }

  if (att_handle ==
      ATT_CHARACTERISTIC_0000FFF1_0000_1000_8000_00805F9B34FB_01_VALUE_HANDLE) {
    return handle_control_point_write(connection_handle, buffer, buffer_size);
  }

  if (att_handle ==
      ATT_CHARACTERISTIC_0000FFF3_0000_1000_8000_00805F9B34FB_01_VALUE_HANDLE) {
    return handle_joint_targets_write(connection_handle, buffer, buffer_size);
  }

  if (att_handle ==
      ATT_CHARACTERISTIC_0000FFF4_0000_1000_8000_00805F9B34FB_01_VALUE_HANDLE) {
    return handle_cartesian_targets_write(connection_handle, buffer, buffer_size);
  }

  return 0;
}

static void packet_handler(uint8_t packet_type,
                           __unused uint16_t channel,
                           uint8_t* packet,
                           __unused uint16_t size) {
  if (packet_type != HCI_EVENT_PACKET) {
    return;
  }

  switch (hci_event_packet_get_type(packet)) {
    case HCI_EVENT_DISCONNECTION_COMPLETE: {
      const hci_con_handle_t handle =
          hci_event_disconnection_complete_get_connection_handle(packet);
      const bool was_owner = (handle == g_owner_handle);
      release_connection(handle);
      if (was_owner) {
        clear_owner(true, "owner disconnected");
      }
      printf("ble disconnected: handle=%u\n", handle);
      break;
    }

    case HCI_EVENT_LE_META: {
      const uint8_t subevent_code = hci_event_le_meta_get_subevent_code(packet);
      if (subevent_code != HCI_SUBEVENT_LE_CONNECTION_COMPLETE) {
        break;
      }

      if (hci_subevent_le_connection_complete_get_status(packet) !=
          ERROR_CODE_SUCCESS) {
        break;
      }

      const hci_con_handle_t handle =
          hci_subevent_le_connection_complete_get_connection_handle(packet);

      if (claim_connection(handle) == NULL) {
        printf("ble connection rejected: max %u reached\n",
               (unsigned)ARM_BLE_MAX_CONNECTIONS);
        gap_disconnect(handle);
        break;
      }

      printf("ble connected: handle=%u\n", handle);
      break;
    }

    default:
      break;
  }
}

static void arm_ble_setup(void) {
  l2cap_init();
  sm_init();
  sm_set_io_capabilities(IO_CAPABILITY_NO_INPUT_NO_OUTPUT);

  set_random_static_ble_address();

  att_server_init(profile_data, att_read_callback, att_write_callback);

  g_hci_event_callback_registration.callback = &packet_handler;
  hci_add_event_handler(&g_hci_event_callback_registration);

  static const uint8_t advertisement_data[] = {
      0x02,
      BLUETOOTH_DATA_TYPE_FLAGS,
      0x06,
      0x03,
      BLUETOOTH_DATA_TYPE_INCOMPLETE_LIST_OF_16_BIT_SERVICE_CLASS_UUIDS,
      0xF0,
      0xFF,
      0x08,
      BLUETOOTH_DATA_TYPE_SHORTENED_LOCAL_NAME,
      'A',
      'r',
      'm',
      'C',
      't',
      'r',
      'l',
  };

  const uint16_t adv_interval_min = 0x0030;
  const uint16_t adv_interval_max = 0x0060;
  const uint8_t adv_type = 0;
  bd_addr_t null_addr;
  memset(null_addr, 0, sizeof(null_addr));

  gap_advertisements_set_params(adv_interval_min, adv_interval_max, adv_type, 0,
                                null_addr, 0x07, 0x00);
  gap_advertisements_set_data(sizeof(advertisement_data),
                              (uint8_t*)advertisement_data);
  gap_advertisements_enable(1);
}

bool arm_ble_prepare(queue_t* event_queue, uint32_t owner_timeout_ms) {
  if (event_queue == NULL) {
    return false;
  }

  g_event_queue = event_queue;
  (void)owner_timeout_ms;

  memset(g_connections, 0, sizeof(g_connections));
  g_owner_handle = HCI_CON_HANDLE_INVALID;

  return true;
}

void arm_ble_core1_entry(void) {
  if (g_event_queue == NULL) {
    while (true) {
      tight_loop_contents();
    }
  }

  if (cyw43_arch_init() != 0) {
    printf("ble init failed: cyw43_arch_init\n");
    while (true) {
      sleep_ms(1000);
    }
  }

  printf("ble init ok on core1\n");

  arm_ble_setup();
  btstack_run_loop_set_timer_handler(&g_bluetooth_led_timer, update_bluetooth_led);
  update_bluetooth_led(&g_bluetooth_led_timer);
  hci_power_control(HCI_POWER_ON);

  printf("ble advertising started (service 0xFFF0)\n");

  btstack_run_loop_execute();
}
