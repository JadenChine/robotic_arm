#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "hal/servo_bus.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "platform/servo_platform_config.h"

#define SMOKE_TEST_MIN_SERVO_ID 1u
#define SMOKE_TEST_MAX_SERVO_ID 6u

static uint8_t clamp_active_servo_id(uint8_t servo_id) {
	if (servo_id < SMOKE_TEST_MIN_SERVO_ID || servo_id > SMOKE_TEST_MAX_SERVO_ID) {
		return SMOKE_TEST_MIN_SERVO_ID;
	}
	return servo_id;
}

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

static const char* backend_to_string(servo_bus_backend_t backend) {
	switch (backend) {
		case SERVO_BUS_BACKEND_DIRECT_SERVO:
			return "direct servo protocol";
		case SERVO_BUS_BACKEND_XARM_CONTROLLER:
		default:
			return "xArm controller protocol";
	}
}

static float clamp_demo_degrees(float degrees) {
	if (degrees < -120.0f) {
		return -120.0f;
	}
	if (degrees > 120.0f) {
		return 120.0f;
	}
	return degrees;
}

static void print_help(const servo_bus_config_t* config, uint8_t active_servo_id) {
	printf("\nCommands:\n");
	printf("  h or ? : print help\n");
	printf("  1..6   : select active servo ID (current: %u)\n", active_servo_id);
	printf("  p      : ping active servo (position read probe)\n");
	printf("  c      : move active servo to center (0 deg)\n");
	printf("  a      : step left by configured increment\n");
	printf("  d      : step right by configured increment\n");
	printf("  s      : stop/unload active servo\n");
	printf("  r      : read active servo position\n");
	if (config->backend == SERVO_BUS_BACKEND_XARM_CONTROLLER) {
		printf("  v      : read controller battery voltage\n");
		printf("  t      : read active servo temperature (unsupported in this mode)\n");
	} else {
		printf("  v      : read active servo voltage\n");
		printf("  t      : read active servo temperature\n");
	}
	printf("  e      : print last driver error\n");
}

static void print_banner(const servo_bus_config_t* config, uint8_t active_servo_id) {
	printf("\nHiWonder Servo Smoke Test\n");
	printf("  Backend      : %s\n", backend_to_string(config->backend));
	printf("  Active servo : %u\n", active_servo_id);
	printf("  Select range : %u-%u\n", SMOKE_TEST_MIN_SERVO_ID, SMOKE_TEST_MAX_SERVO_ID);
	printf("  UART index   : %u\n", config->uart_index);
	printf("  UART baud    : %lu\n", (unsigned long)config->baud_rate);
	printf("  UART TX pin  : %u\n", config->tx_pin);
	printf("  UART RX pin  : %u\n", config->rx_pin);
	printf("  Move time ms : %u\n", config->default_move_time_ms);
	printf("  Step degrees : %.1f\n", config->default_step_degrees);
	printf("  Heartbeat LED: %u\n", SERVO_BUS_HEARTBEAT_LED_PIN);
	print_help(config, active_servo_id);
}

static void handle_command(int ch, const servo_bus_config_t* config, uint8_t* active_servo_id,
				   float* target_degrees_by_id) {
	servo_bus_status_t status = SERVO_BUS_STATUS_OK;
	float degrees = 0.0f;
	int16_t raw_position = 0;
	int16_t voltage_mv = 0;
	int8_t temperature_c = 0;
	const uint8_t servo_id = *active_servo_id;
	float* target_degrees = &target_degrees_by_id[servo_id];

	switch (ch) {
		case '1':
		case '2':
		case '3':
		case '4':
		case '5':
		case '6':
			*active_servo_id = (uint8_t)(ch - '0');
			printf("active servo set to ID %u\n", *active_servo_id);
			break;
		case 'h':
		case '?':
			print_help(config, *active_servo_id);
			break;
		case 'p':
			status = servo_bus_ping(servo_id);
			printf("ping servo %u: %s\n", servo_id, status_to_string(status));
			break;
		case 'c':
			*target_degrees = 0.0f;
			status = servo_bus_move_degrees(servo_id, *target_degrees, config->default_move_time_ms);
			printf("servo %u move center: %s\n", servo_id, status_to_string(status));
			break;
		case 'a':
			*target_degrees = clamp_demo_degrees(*target_degrees - config->default_step_degrees);
			status = servo_bus_move_degrees(servo_id, *target_degrees, config->default_move_time_ms);
			printf("servo %u move %.1f deg: %s\n", servo_id, *target_degrees, status_to_string(status));
			break;
		case 'd':
			*target_degrees = clamp_demo_degrees(*target_degrees + config->default_step_degrees);
			status = servo_bus_move_degrees(servo_id, *target_degrees, config->default_move_time_ms);
			printf("servo %u move %.1f deg: %s\n", servo_id, *target_degrees, status_to_string(status));
			break;
		case 's':
			status = servo_bus_stop(servo_id);
			printf("servo %u stop/unload: %s\n", servo_id, status_to_string(status));
			break;
		case 'r':
			status = servo_bus_get_position_degrees(servo_id, &degrees);
			if (status == SERVO_BUS_STATUS_OK) {
				(void)servo_bus_get_position_raw(servo_id, &raw_position);
				*target_degrees = clamp_demo_degrees(degrees);
				printf("servo %u position: %.2f deg (raw %d)\n", servo_id, degrees, raw_position);
			} else {
				printf("servo %u position read: %s\n", servo_id, status_to_string(status));
			}
			break;
		case 'v':
			status = servo_bus_get_voltage(servo_id, &voltage_mv);
			if (status == SERVO_BUS_STATUS_OK) {
				if (config->backend == SERVO_BUS_BACKEND_XARM_CONTROLLER) {
					printf("controller battery voltage: %d mV\n", voltage_mv);
				} else {
					printf("servo %u voltage: %d mV\n", servo_id, voltage_mv);
				}
			} else {
				printf("servo %u voltage read: %s\n", servo_id, status_to_string(status));
			}
			break;
		case 't':
			status = servo_bus_get_temperature(servo_id, &temperature_c);
			if (status == SERVO_BUS_STATUS_OK) {
				printf("servo %u temperature: %d C\n", servo_id, temperature_c);
			} else {
				printf("servo %u temperature read: %s\n", servo_id, status_to_string(status));
			}
			break;
		case 'e':
			printf("last driver error: %d\n", servo_bus_get_last_driver_error());
			break;
		case '\r':
		case '\n':
			break;
		default:
			printf("unknown command '%c'\n", ch);
			print_help(config, *active_servo_id);
			break;
	}
}

int main(void) {
	servo_bus_config_t config;
	uint32_t last_heartbeat_ms;
	bool led_on = false;
	uint8_t active_servo_id = SMOKE_TEST_MIN_SERVO_ID;
	float target_degrees_by_id[SMOKE_TEST_MAX_SERVO_ID + 1] = {0.0f};

	stdio_init_all();
	sleep_ms(1500);

	gpio_init(SERVO_BUS_HEARTBEAT_LED_PIN);
	gpio_set_dir(SERVO_BUS_HEARTBEAT_LED_PIN, GPIO_OUT);
	gpio_put(SERVO_BUS_HEARTBEAT_LED_PIN, 0);

	servo_bus_get_default_config(&config);
	active_servo_id = clamp_active_servo_id(config.default_servo_id);
	if (config.default_servo_id != active_servo_id) {
		printf("default servo ID %u outside smoke-test range, using %u\n",
				config.default_servo_id, active_servo_id);
	}
	print_banner(&config, active_servo_id);

	printf("initializing servo bus... ");
	servo_bus_status_t init_status = servo_bus_init(&config);
	printf("%s\n", status_to_string(init_status));

	last_heartbeat_ms = to_ms_since_boot(get_absolute_time());

	while (true) {
		uint32_t now_ms = to_ms_since_boot(get_absolute_time());
		if (now_ms - last_heartbeat_ms >= SERVO_BUS_HEARTBEAT_INTERVAL_MS) {
			led_on = !led_on;
			gpio_put(SERVO_BUS_HEARTBEAT_LED_PIN, led_on ? 1 : 0);
			last_heartbeat_ms = now_ms;
		}

		int ch = getchar_timeout_us(0);
		if (ch >= 0) {
			handle_command(ch, &config, &active_servo_id, target_degrees_by_id);
		}

		tight_loop_contents();
	}

	return 0;
}
