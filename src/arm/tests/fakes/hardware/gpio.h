#pragma once
constexpr unsigned GPIO_FUNC_UART = 2;
constexpr bool GPIO_OUT = true;
constexpr bool GPIO_IN = false;
void gpio_init(unsigned);
void gpio_pull_down(unsigned);
void gpio_pull_up(unsigned);
void gpio_put(unsigned, bool);
void gpio_set_dir(unsigned, bool);
void gpio_set_function(unsigned, unsigned);
