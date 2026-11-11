#pragma once
#include <stddef.h>
#include <stdint.h>

struct uart_inst_t { unsigned index; };
extern uart_inst_t* uart0;
extern uart_inst_t* uart1;
constexpr unsigned UART_PARITY_NONE = 0;
unsigned uart_init(uart_inst_t*, unsigned);
void uart_deinit(uart_inst_t*);
void uart_set_hw_flow(uart_inst_t*, bool, bool);
void uart_set_format(uart_inst_t*, unsigned, unsigned, unsigned);
void uart_set_fifo_enabled(uart_inst_t*, bool);
bool uart_is_readable(uart_inst_t*);
char uart_getc(uart_inst_t*);
void uart_write_blocking(uart_inst_t*, const uint8_t*, size_t);
void uart_tx_wait_blocking(uart_inst_t*);
