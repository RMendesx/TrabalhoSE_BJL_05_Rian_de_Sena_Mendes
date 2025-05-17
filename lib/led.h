#ifndef LED_H
#define LED_H

#include "pico/stdlib.h"
#include "hardware/gpio.h"

void led_on(uint gpio) // Liga o LED
{
    gpio_init(gpio);
    gpio_set_dir(gpio, GPIO_OUT);
    gpio_put(gpio, 1);
}

void led_off(uint gpio) // Desliga o LED
{
    gpio_init(gpio);
    gpio_set_dir(gpio, GPIO_OUT);
    gpio_put(gpio, 0);
}

#endif