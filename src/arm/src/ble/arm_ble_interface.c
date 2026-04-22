#include "ble/arm_ble_interface.h"

#include <stdio.h>
#include <string.h>

#include "arm_control_service.h"
#include "btstack.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"

#define ARM_BLE_MAX_CONNECTIONS 2u
#define ARM_BLE_CONTROL_CMD_TAKE_CONTROL 0x01u
#define ARM_BLE_CONTROL_CMD_RELEASE_CONTROL 0x02u
#define ARM_BLE_OWNER_TIMEOUT_DEFAULT_MS 1500u
#define ARM_BLE_TIMEOUT_CHECK_PERIOD_MS 100u
#define ARM_BLE_TARGET_PACKET_SIZE_BYTES (ARM_BLE_JOINT_COUNT * sizeof(int16_t))

typedef struct arm_ble_connection_slot {
  bool in_use;
  hci_con_handle_t handle;
} arm_ble_connection_slot_t;

static queue_t* g_event_queue = NULL;
static uint32_t g_owner_timeout_ms = ARM_BLE_OWNER_TIMEOUT_DEFAULT_MS;

static arm_ble_connection_slot_t g_connections[ARM_BLE_MAX_CONNECTIONS] = {0};

static hci_con_handle_t g_owner_handle = HCI_CON_HANDLE_INVALID;
static uint32_t g_owner_token = 0;
static uint32_t g_next_owner_token = 1;
static uint32_t g_last_owner_update_ms = 0;

static btstack_packet_callback_registration_t g_hci_event_callback_registration;
static btstack_timer_source_t g_owner_timeout_timer;

static uint32_t now_ms(void) {
  return to_ms_since_boot(get_absolute_time());
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

static void note_owner_activity(void) {
  g_last_owner_update_ms = now_ms();
}

static void clear_owner(bool enqueue_safe_pose, const char* reason) {
  if (g_owner_handle == HCI_CON_HANDLE_INVALID && !enqueue_safe_pose) {
    return;
  }

  g_owner_handle = HCI_CON_HANDLE_INVALID;
  g_owner_token = 0;

  if (enqueue_safe_pose) {
    arm_ble_event_t safe_pose_event = {
        .type = ARM_BLE_EVENT_SAFE_POSE,
        .joint_target_deci_deg = {0},
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

  if (g_next_owner_token == 0) {
    g_next_owner_token = 1;
  }

  g_owner_handle = connection_handle;
  g_owner_token = g_next_owner_token++;
  note_owner_activity();

  printf("ble ownership granted to handle=%u token=%lu\n", connection_handle,
         (unsigned long)g_owner_token);
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
      clear_owner(false, "released by owner");
      return 0;

    default:
      return ATT_ERROR_VALUE_NOT_ALLOWED;
  }
}

static int handle_joint_targets_write(hci_con_handle_t connection_handle,
                                      const uint8_t* buffer,
                                      uint16_t buffer_size) {
  if (g_owner_handle != connection_handle) {
    return ATT_ERROR_INSUFFICIENT_AUTHORIZATION;
  }

  if (buffer_size != ARM_BLE_TARGET_PACKET_SIZE_BYTES) {
    return ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LENGTH;
  }

  arm_ble_event_t joint_event = {
      .type = ARM_BLE_EVENT_SET_TARGETS,
      .joint_target_deci_deg = {0},
  };

  for (size_t joint_index = 0; joint_index < ARM_BLE_JOINT_COUNT; ++joint_index) {
    const uint16_t raw = little_endian_read_16(buffer, (int)(joint_index * 2u));
    joint_event.joint_target_deci_deg[joint_index] = (int16_t)raw;
  }

  note_owner_activity();

  if (!enqueue_event(&joint_event)) {
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
    uint8_t token_bytes[sizeof(uint32_t)] = {0};
    little_endian_store_32(token_bytes, 0, g_owner_token);
    return att_read_callback_handle_blob(token_bytes, sizeof(token_bytes), offset,
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

static void owner_timeout_timer_handler(btstack_timer_source_t* timer_source) {
  if (g_owner_handle != HCI_CON_HANDLE_INVALID) {
    const uint32_t elapsed_ms = now_ms() - g_last_owner_update_ms;
    if (elapsed_ms >= g_owner_timeout_ms) {
      clear_owner(true, "owner timeout");
    }
  }

  btstack_run_loop_set_timer(timer_source, ARM_BLE_TIMEOUT_CHECK_PERIOD_MS);
  btstack_run_loop_add_timer(timer_source);
}

static void arm_ble_setup(void) {
  l2cap_init();
  sm_init();
  sm_set_io_capabilities(IO_CAPABILITY_NO_INPUT_NO_OUTPUT);

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

  g_owner_timeout_timer.process = owner_timeout_timer_handler;
  btstack_run_loop_set_timer(&g_owner_timeout_timer,
                             ARM_BLE_TIMEOUT_CHECK_PERIOD_MS);
  btstack_run_loop_add_timer(&g_owner_timeout_timer);
}

bool arm_ble_prepare(queue_t* event_queue, uint32_t owner_timeout_ms) {
  if (event_queue == NULL) {
    return false;
  }

  g_event_queue = event_queue;
  g_owner_timeout_ms = owner_timeout_ms;
  if (g_owner_timeout_ms == 0) {
    g_owner_timeout_ms = ARM_BLE_OWNER_TIMEOUT_DEFAULT_MS;
  }

  memset(g_connections, 0, sizeof(g_connections));
  g_owner_handle = HCI_CON_HANDLE_INVALID;
  g_owner_token = 0;
  g_next_owner_token = 1;
  g_last_owner_update_ms = now_ms();

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
  hci_power_control(HCI_POWER_ON);

  printf("ble advertising started (service 0xFFF0)\n");

  btstack_run_loop_execute();
}
