#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "hal/servo_bus.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "platform/servo_platform_config.h"

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
		case SERVO_BUS_STATUS_ERROR_PROTOCOL:
		default:
			return "protocol error";
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

static void print_help(void) {
	printf("\nCommands:\n");
	printf("  h or ? : print help\n");
	printf("  p      : ping servo 1\n");
	printf("  c      : move servo 1 to center (0 deg)\n");
	printf("  a      : step left by configured increment\n");
	printf("  d      : step right by configured increment\n");
	printf("  s      : stop servo 1\n");
	printf("  r      : read servo 1 position\n");
	printf("  v      : read servo 1 voltage\n");
	printf("  t      : read servo 1 temperature\n");
	printf("  e      : print last driver error\n");
}

static void print_banner(const servo_bus_config_t* config) {
	printf("\nHiWonder Servo Smoke Test\n");
	printf("  Servo ID     : %u\n", config->default_servo_id);
	printf("  UART index   : %u\n", config->uart_index);
	printf("  UART baud    : %lu\n", (unsigned long)config->baud_rate);
	printf("  UART TX pin  : %u\n", config->tx_pin);
	printf("  UART RX pin  : %u\n", config->rx_pin);
	printf("  Move time ms : %u\n", config->default_move_time_ms);
	printf("  Step degrees : %.1f\n", config->default_step_degrees);
	printf("  Heartbeat LED: %u\n", SERVO_BUS_HEARTBEAT_LED_PIN);
	print_help();
}

static void handle_command(int ch, const servo_bus_config_t* config, float* target_degrees) {
	servo_bus_status_t status = SERVO_BUS_STATUS_OK;
	float degrees = 0.0f;
	int16_t raw_position = 0;
	int16_t voltage_mv = 0;
	int8_t temperature_c = 0;

	switch (ch) {
		case 'h':
		case '?':
			print_help();
			break;
		case 'p':
			status = servo_bus_ping(config->default_servo_id);
			printf("ping: %s\n", status_to_string(status));
			break;
		case 'c':
			*target_degrees = 0.0f;
			status = servo_bus_move_degrees(config->default_servo_id, *target_degrees,
			                               config->default_move_time_ms);
			printf("move center: %s\n", status_to_string(status));
			break;
		case 'a':
			*target_degrees = clamp_demo_degrees(*target_degrees - config->default_step_degrees);
			status = servo_bus_move_degrees(config->default_servo_id, *target_degrees,
			                               config->default_move_time_ms);
			printf("move %.1f deg: %s\n", *target_degrees, status_to_string(status));
			break;
		case 'd':
			*target_degrees = clamp_demo_degrees(*target_degrees + config->default_step_degrees);
			status = servo_bus_move_degrees(config->default_servo_id, *target_degrees,
			                               config->default_move_time_ms);
			printf("move %.1f deg: %s\n", *target_degrees, status_to_string(status));
			break;
		case 's':
			status = servo_bus_stop(config->default_servo_id);
			printf("stop: %s\n", status_to_string(status));
			break;
		case 'r':
			status = servo_bus_get_position_degrees(config->default_servo_id, &degrees);
			if (status == SERVO_BUS_STATUS_OK) {
				(void)servo_bus_get_position_raw(config->default_servo_id, &raw_position);
				printf("position: %.2f deg (raw %d)\n", degrees, raw_position);
			} else {
				printf("position read: %s\n", status_to_string(status));
			}
			break;
		case 'v':
			status = servo_bus_get_voltage(config->default_servo_id, &voltage_mv);
			if (status == SERVO_BUS_STATUS_OK) {
				printf("voltage: %d mV\n", voltage_mv);
			} else {
				printf("voltage read: %s\n", status_to_string(status));
			}
			break;
		case 't':
			status = servo_bus_get_temperature(config->default_servo_id, &temperature_c);
			if (status == SERVO_BUS_STATUS_OK) {
				printf("temperature: %d C\n", temperature_c);
			} else {
				printf("temperature read: %s\n", status_to_string(status));
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
			print_help();
			break;
	}
}

int main(void) {
	servo_bus_config_t config;
	uint32_t last_heartbeat_ms;
	bool led_on = false;
	float target_degrees = 0.0f;

	stdio_init_all();
	sleep_ms(1500);

	gpio_init(SERVO_BUS_HEARTBEAT_LED_PIN);
	gpio_set_dir(SERVO_BUS_HEARTBEAT_LED_PIN, GPIO_OUT);
	gpio_put(SERVO_BUS_HEARTBEAT_LED_PIN, 0);

	servo_bus_get_default_config(&config);
	print_banner(&config);

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
		if (ch != PICO_ERROR_TIMEOUT) {
			handle_command(ch, &config, &target_degrees);
		}

		tight_loop_contents();
	}

	return 0;
}
