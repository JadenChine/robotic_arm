#include <stdbool.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "ble/arm_ble_interface.h"
#include "hal/servo_bus.h"
#include "hardware/sync.h"
#include "kinematics/arm_kinematics.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/util/queue.h"
#include "platform/servo_platform_config.h"

#define ARM_JOINT_COUNT 6u

#define CONTROL_TICK_HZ 20u
#define CONTROL_MOVE_TIME_MS 90u
#define CONTROL_RAW_DEADBAND 2

#define BLE_EVENT_QUEUE_LENGTH 24u

#define JOINT_DEFAULT_MIN_DECI_DEG (-1200)
#define JOINT_DEFAULT_MAX_DECI_DEG (1200)

#define JOINT1_MIN_DECI_DEG JOINT_DEFAULT_MIN_DECI_DEG
#define JOINT1_MAX_DECI_DEG (100)

#define JOINT5_MIN_DECI_DEG (-900)
#define JOINT5_MAX_DECI_DEG (900)


typedef struct joint_config {
	uint8_t servo_id;
	int32_t min_deci_deg;
	int32_t max_deci_deg;
	int8_t direction;
	int32_t offset_deci_deg;
} joint_config_t;

typedef struct pending_ble_targets {
	arm_ble_event_t latest_joint_event;
	arm_ble_event_t latest_cartesian_event;
	bool has_joint_event;
	bool has_cartesian_event;
} pending_ble_targets_t;

// Register-style global targets for each joint in deci-degrees (0.1°).
volatile int32_t g_joint_target_deci_deg[ARM_JOINT_COUNT] = {0, 0, 0, 0, 0, 0};

static const joint_config_t g_joint_configs[ARM_JOINT_COUNT] = {
	{1u, JOINT1_MIN_DECI_DEG,        JOINT1_MAX_DECI_DEG,         1, 0},
	{2u, JOINT_DEFAULT_MIN_DECI_DEG, JOINT_DEFAULT_MAX_DECI_DEG,  1, 0},
	{3u, JOINT_DEFAULT_MIN_DECI_DEG, JOINT_DEFAULT_MAX_DECI_DEG,  1, 0},
	{4u, JOINT_DEFAULT_MIN_DECI_DEG, JOINT_DEFAULT_MAX_DECI_DEG, -1, 0},
	{5u, JOINT5_MIN_DECI_DEG,        JOINT5_MAX_DECI_DEG,         1, 0},
	{6u, JOINT_DEFAULT_MIN_DECI_DEG, JOINT_DEFAULT_MAX_DECI_DEG,  1, 0},
};

static int16_t g_last_sent_raw_by_joint[ARM_JOINT_COUNT] = {0};
static bool g_last_sent_valid_by_joint[ARM_JOINT_COUNT] = {false};
static volatile bool g_control_tick_due = false;
static queue_t g_ble_event_queue;
static arm_ble_control_mode_t g_control_mode = ARM_BLE_CONTROL_MODE_JOINT;
static arm_kinematics_solution_t g_last_valid_cartesian_solution = {0};
static arm_kinematics_cartesian_t g_cartesian_target_mm = {0.0f, 0.0f, 429.0f};
static bool g_cartesian_ik_holding = false;

// Clamps an integer value between a minimum and maximum range.
static int32_t clamp_i32(int32_t value, int32_t minimum, int32_t maximum) {
	if (value < minimum) {
		return minimum;
	}
	if (value > maximum) {
		return maximum;
	}
	return value;
}

// Converts a servo bus status to a human-readable string.
static const char* status_to_string(servo_bus_status_t status) {
	switch (status) {
		case SERVO_BUS_STATUS_OK:
			return "ok";
		case SERVO_BUS_STATUS_ERROR_NOT_INITIALIZED:
			return "not initialized";
		case SERVO_BUS_STATUS_ERROR_INVALID_ARGUMENT:
			return "invalid argument";
		case SERVO_BUS_STATUS_ERROR_TIMEOUT:
			return "timeout";
		case SERVO_BUS_STATUS_ERROR_OVERFLOW:
			return "overflow";
		case SERVO_BUS_STATUS_ERROR_UNSUPPORTED:
			return "unsupported";
		case SERVO_BUS_STATUS_ERROR_PROTOCOL:
		default:
			return "protocol error";
	}
}

// Converts a servo bus backend type to a human-readable string.
static const char* backend_to_string(servo_bus_backend_t backend) {
	switch (backend) {
		case SERVO_BUS_BACKEND_DIRECT_SERVO:
			return "direct servo protocol";
		case SERVO_BUS_BACKEND_XARM_CONTROLLER:
		default:
			return "xArm controller protocol";
	}
}

// Applies joint-specific transformations to a target angle in deci-degrees (0.1°).
static int32_t apply_joint_transform(const joint_config_t* config, int32_t target_deci_deg) {
	int32_t transformed_deci_deg = clamp_i32(target_deci_deg, config->min_deci_deg, config->max_deci_deg);

	if (config->direction < 0) {
		transformed_deci_deg = -transformed_deci_deg;
	}

	transformed_deci_deg += config->offset_deci_deg;
	return clamp_i32(transformed_deci_deg, JOINT_DEFAULT_MIN_DECI_DEG, JOINT_DEFAULT_MAX_DECI_DEG);
}

// Converts an angle in deci-degrees (0.1°) to a raw servo position value.
// Mapping is defined such that -120.0° -> 0, 0.0° -> 500, +120.0° -> 1000.
static int16_t deci_deg_to_raw_position(int32_t deci_deg) {
	const int32_t clamped_deci_deg = clamp_i32(deci_deg, JOINT_DEFAULT_MIN_DECI_DEG, JOINT_DEFAULT_MAX_DECI_DEG);
	const int32_t scaled = (clamped_deci_deg * 500) / JOINT_DEFAULT_MAX_DECI_DEG;
	const int32_t raw_position = clamp_i32(500 + scaled, 0, 1000);
	return (int16_t)raw_position;
}

// Converts degrees to logical deci-degrees for the existing joint target registers.
static int32_t degrees_to_deci_deg(float degrees) {
	return (int32_t)lroundf(degrees * 10.0f);
}

// Builds a sim-space 4-DOF kinematics seed from the logical real-joint target registers.
static void read_kinematics_solution_from_joint_targets(arm_kinematics_solution_t* solution) {
	if (solution == NULL) {
		return;
	}

	solution->joint_deg[0] = ((float)g_joint_target_deci_deg[5]) / 10.0f;
	solution->joint_deg[1] = ((float)g_joint_target_deci_deg[4]) / 10.0f;
	solution->joint_deg[2] = ((float)g_joint_target_deci_deg[3]) / 10.0f;
	solution->joint_deg[3] = ((float)g_joint_target_deci_deg[2]) / 10.0f;
}

// Maps sim-space IK output into the real arm's logical joint registers.
static void write_kinematics_solution_to_joint_targets(const arm_kinematics_solution_t* solution) {
	if (solution == NULL) {
		return;
	}

	g_joint_target_deci_deg[5] = degrees_to_deci_deg(solution->joint_deg[0]);
	g_joint_target_deci_deg[4] = degrees_to_deci_deg(solution->joint_deg[1]);
	g_joint_target_deci_deg[3] = degrees_to_deci_deg(solution->joint_deg[2]);
	g_joint_target_deci_deg[2] = degrees_to_deci_deg(solution->joint_deg[3]);
}

// Seeds Cartesian mode from the current logical joint pose to avoid mode-switch jumps.
static void seed_cartesian_state_from_joint_targets(void) {
	read_kinematics_solution_from_joint_targets(&g_last_valid_cartesian_solution);
	arm_kinematics_forward(&g_last_valid_cartesian_solution, &g_cartesian_target_mm);
	g_cartesian_ik_holding = false;
}

// Timer callback function to signal that a control tick is due.
static bool control_tick_timer_callback(repeating_timer_t* timer) {
	(void)timer;
	g_control_tick_due = true;
	return true;
}

// Consumes the control tick signal and resets its state.
static bool consume_control_tick_due(void) {
	const uint32_t irq_state = save_and_disable_interrupts();
	const bool tick_due = g_control_tick_due;
	g_control_tick_due = false;
	restore_interrupts(irq_state);
	return tick_due;
}

// Initializes joint targets to a safe default pose.
static void initialize_joint_targets_to_safe_pose(void) {
	for (size_t joint_index = 0; joint_index < ARM_JOINT_COUNT; ++joint_index) {
		g_joint_target_deci_deg[joint_index] = 0;
		g_last_sent_valid_by_joint[joint_index] = false;
	}

	g_control_mode = ARM_BLE_CONTROL_MODE_JOINT;
	arm_kinematics_home_solution(&g_last_valid_cartesian_solution);
	arm_kinematics_forward(&g_last_valid_cartesian_solution, &g_cartesian_target_mm);
	g_cartesian_ik_holding = false;
}

// Marks all joints as dirty so the next control tick forces an update.
static void mark_all_joint_outputs_dirty(void) {
	for (size_t joint_index = 0; joint_index < ARM_JOINT_COUNT; ++joint_index) {
		g_last_sent_valid_by_joint[joint_index] = false;
	}
}

// Applies a BLE target update packet to the joint target registers.
static void apply_ble_joint_targets(const arm_ble_event_t* event) {
	const size_t joints_to_apply =
		(g_control_mode == ARM_BLE_CONTROL_MODE_CARTESIAN) ? 2u : ARM_JOINT_COUNT;

	for (size_t joint_index = 0; joint_index < joints_to_apply; ++joint_index) {
		g_joint_target_deci_deg[joint_index] = event->joint_target_deci_deg[joint_index];
	}
}

// Applies an owner-requested control mode switch.
static void apply_ble_control_mode(arm_ble_control_mode_t requested_mode) {
	if (requested_mode != ARM_BLE_CONTROL_MODE_JOINT &&
	    requested_mode != ARM_BLE_CONTROL_MODE_CARTESIAN) {
		return;
	}

	if (requested_mode == g_control_mode) {
		return;
	}

	if (requested_mode == ARM_BLE_CONTROL_MODE_CARTESIAN) {
		seed_cartesian_state_from_joint_targets();
		printf("ble control mode: cartesian xyz\n");
	} else {
		printf("ble control mode: joint targets\n");
	}

	g_control_mode = requested_mode;
}

// Applies a Cartesian target through IK, leaving wrist/gripper joints under manual control.
static void apply_ble_cartesian_target(const arm_ble_event_t* event) {
	if (event == NULL || g_control_mode != ARM_BLE_CONTROL_MODE_CARTESIAN) {
		return;
	}

	const arm_kinematics_cartesian_t requested_target = {
		.x_mm = (float)event->cartesian_target_mm[0],
		.y_mm = (float)event->cartesian_target_mm[1],
		.z_mm = (float)event->cartesian_target_mm[2],
	};
	arm_kinematics_solution_t solved_solution = g_last_valid_cartesian_solution;
	arm_kinematics_cartesian_t clamped_target = requested_target;

	const bool solved = arm_kinematics_inverse(&requested_target,
	                                         &g_last_valid_cartesian_solution,
	                                         &solved_solution,
	                                         &clamped_target);
	if (!solved) {
		if (!g_cartesian_ik_holding) {
			printf("cartesian ik hold: target %.0f %.0f %.0f mm unreachable\n",
			       requested_target.x_mm, requested_target.y_mm, requested_target.z_mm);
		}
		g_cartesian_ik_holding = true;
		return;
	}

	if (g_cartesian_ik_holding) {
		printf("cartesian ik recovered\n");
	}

	g_cartesian_ik_holding = false;
	g_last_valid_cartesian_solution = solved_solution;
	g_cartesian_target_mm = clamped_target;
	write_kinematics_solution_to_joint_targets(&solved_solution);
}

// Applies safe-pose behavior after ownership timeout/disconnect.
static void apply_ble_safe_pose(void) {
	for (size_t joint_index = 0; joint_index < ARM_JOINT_COUNT; ++joint_index) {
		g_joint_target_deci_deg[joint_index] = 0;
	}

	g_control_mode = ARM_BLE_CONTROL_MODE_JOINT;
	arm_kinematics_home_solution(&g_last_valid_cartesian_solution);
	arm_kinematics_forward(&g_last_valid_cartesian_solution, &g_cartesian_target_mm);
	g_cartesian_ik_holding = false;
	mark_all_joint_outputs_dirty();
	printf("ble safety event: commanding safe pose targets\n");
}

static void clear_pending_ble_targets(pending_ble_targets_t* pending_targets) {
	if (pending_targets == NULL) {
		return;
	}

	pending_targets->has_joint_event = false;
	pending_targets->has_cartesian_event = false;
}

static void apply_pending_ble_targets(pending_ble_targets_t* pending_targets) {
	if (pending_targets == NULL) {
		return;
	}

	if (pending_targets->has_joint_event) {
		apply_ble_joint_targets(&pending_targets->latest_joint_event);
	}

	if (pending_targets->has_cartesian_event) {
		apply_ble_cartesian_target(&pending_targets->latest_cartesian_event);
	}

	clear_pending_ble_targets(pending_targets);
}

// Drains queued BLE events from core1 and applies them on core0.
static void process_ble_events(void) {
	arm_ble_event_t event;
	pending_ble_targets_t pending_targets = {0};

	while (queue_try_remove(&g_ble_event_queue, &event)) {
		switch (event.type) {
			case ARM_BLE_EVENT_SET_TARGETS:
				pending_targets.latest_joint_event = event;
				pending_targets.has_joint_event = true;
				break;

			case ARM_BLE_EVENT_SET_CARTESIAN_TARGET:
				pending_targets.latest_cartesian_event = event;
				pending_targets.has_cartesian_event = true;
				break;

			case ARM_BLE_EVENT_SET_CONTROL_MODE:
				apply_pending_ble_targets(&pending_targets);
				apply_ble_control_mode(event.control_mode);
				break;

			case ARM_BLE_EVENT_SAFE_POSE:
				clear_pending_ble_targets(&pending_targets);
				apply_ble_safe_pose();
				break;

			default:
				break;
		}
	}

	apply_pending_ble_targets(&pending_targets);
}

// Sends joint target positions to the servo bus if they have changed significantly.
static servo_bus_status_t send_joint_targets_if_dirty(void) {
	uint8_t ids[ARM_JOINT_COUNT] = {0};
	int16_t positions[ARM_JOINT_COUNT] = {0};
	uint8_t dirty_joint_indices[ARM_JOINT_COUNT] = {0};
	size_t dirty_count = 0;

	for (size_t joint_index = 0; joint_index < ARM_JOINT_COUNT; ++joint_index) {
		const joint_config_t* config = &g_joint_configs[joint_index];
		const int32_t requested_deci_deg = g_joint_target_deci_deg[joint_index];
		const int32_t transformed_deci_deg = apply_joint_transform(config, requested_deci_deg);
		const int16_t raw_position = deci_deg_to_raw_position(transformed_deci_deg);

		int32_t raw_difference = raw_position - g_last_sent_raw_by_joint[joint_index];
		if (raw_difference < 0) {
			raw_difference = -raw_difference;
		}

		if (!g_last_sent_valid_by_joint[joint_index] || raw_difference >= CONTROL_RAW_DEADBAND) {
			ids[dirty_count] = config->servo_id;
			positions[dirty_count] = raw_position;
			dirty_joint_indices[dirty_count] = (uint8_t)joint_index;
			++dirty_count;
		}
	}

	if (dirty_count == 0) {
		return SERVO_BUS_STATUS_OK;
	}

	const servo_bus_status_t status =
		servo_bus_move_multiple(ids, positions, dirty_count, CONTROL_MOVE_TIME_MS);
	if (status != SERVO_BUS_STATUS_OK) {
		return status;
	}

	for (size_t dirty_index = 0; dirty_index < dirty_count; ++dirty_index) {
		const uint8_t joint_index = dirty_joint_indices[dirty_index];
		g_last_sent_raw_by_joint[joint_index] = positions[dirty_index];
		g_last_sent_valid_by_joint[joint_index] = true;
	}

	return SERVO_BUS_STATUS_OK;
}

// Executes a single control tick, sending joint targets and handling errors.
static void run_control_tick(void) {
	static servo_bus_status_t last_status = SERVO_BUS_STATUS_OK;

	const servo_bus_status_t status = send_joint_targets_if_dirty();
	if (status != SERVO_BUS_STATUS_OK && status != last_status) {
		printf("control tick send failed: %s (driver error %d)\n",
		       status_to_string(status), servo_bus_get_last_driver_error());
	}

	if (status == SERVO_BUS_STATUS_OK && last_status != SERVO_BUS_STATUS_OK) {
		printf("control tick send recovered\n");
	}

	last_status = status;
}

// Updates the heartbeat LED state based on control tick frequency.
static void update_heartbeat(bool* led_on, uint32_t* tick_counter) {
	const uint32_t ticks_per_toggle = (CONTROL_TICK_HZ >= 2u) ? (CONTROL_TICK_HZ / 2u) : 1u;

	++(*tick_counter);
	if (*tick_counter < ticks_per_toggle) {
		return;
	}

	*tick_counter = 0;
	*led_on = !(*led_on);
	gpio_put(SERVO_BUS_HEARTBEAT_LED_PIN, *led_on ? 1 : 0);
}

// Prints a startup banner with servo bus configuration details.
static void print_banner(const servo_bus_config_t* config) {
	printf("\nHiWonder Arm Joint Target Controller\n");
	printf("  Backend       : %s\n", backend_to_string(config->backend));
	printf("  UART index    : %u\n", config->uart_index);
	printf("  UART baud     : %lu\n", (unsigned long)config->baud_rate);
	printf("  UART TX pin   : %u\n", config->tx_pin);
	printf("  UART RX pin   : %u\n", config->rx_pin);
	printf("  Control tick  : %u Hz (timer IRQ scheduled)\n", CONTROL_TICK_HZ);
	printf("  Move time ms  : %u\n", CONTROL_MOVE_TIME_MS);
	printf("  Deadband raw  : %d\n", CONTROL_RAW_DEADBAND);
	printf("  Heartbeat LED : %u\n", SERVO_BUS_HEARTBEAT_LED_PIN);
	printf("  Target regs   : g_joint_target_deci_deg[0..5] (deci-degrees, 0.1 deg units)\n");
	printf("  BLE role      : peripheral (core1)\n");
	printf("  BLE service   : 0xFFF0, ctrl=0xFFF1 owner=0xFFF2 joints=0xFFF3 xyz=0xFFF4\n");
	printf("  BLE modes     : joint targets, cartesian xyz\n");
	printf("  BLE timeout   : disabled (release/disconnect only)\n");
}

// Main entry point for the robotic arm firmware.
int main(void) {
	servo_bus_config_t config;
	bool led_on = false;
	uint32_t heartbeat_ticks = 0;
	servo_bus_status_t init_status;
	repeating_timer_t control_tick_timer;

	stdio_init_all();
	sleep_ms(1500);

	gpio_init(SERVO_BUS_HEARTBEAT_LED_PIN);
	gpio_set_dir(SERVO_BUS_HEARTBEAT_LED_PIN, GPIO_OUT);
	gpio_put(SERVO_BUS_HEARTBEAT_LED_PIN, 0);

	servo_bus_get_default_config(&config);
	print_banner(&config);

	printf("initializing servo bus... ");
	init_status = servo_bus_init(&config);
	printf("%s\n", status_to_string(init_status));
	if (init_status != SERVO_BUS_STATUS_OK) {
		printf("servo bus initialization failed, control loop will stay idle\n");
	}

	initialize_joint_targets_to_safe_pose();
	g_control_tick_due = true;

	queue_init(&g_ble_event_queue, sizeof(arm_ble_event_t), BLE_EVENT_QUEUE_LENGTH);
	if (arm_ble_prepare(&g_ble_event_queue, 0u)) {
		multicore_launch_core1(arm_ble_core1_entry);
		printf("ble core launched\n");
	} else {
		printf("ble setup failed: event queue unavailable\n");
	}

	const int64_t control_period_us = -((int64_t)(1000000u / CONTROL_TICK_HZ));
	if (!add_repeating_timer_us(control_period_us, control_tick_timer_callback, NULL,
					   &control_tick_timer)) {
		printf("failed to start control tick timer\n");
	} else {
		printf("control timer started: %u Hz\n", CONTROL_TICK_HZ);
	}

	while (true) {
		process_ble_events();

		if (consume_control_tick_due()) {
			if (init_status == SERVO_BUS_STATUS_OK) {
				run_control_tick();
			}
			update_heartbeat(&led_on, &heartbeat_ticks);
		}

		tight_loop_contents();
	}

	return 0;
}
