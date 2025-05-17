#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "lib/ssd1306.h"
#include "lib/font.h"
#include "lib/matriz_led.h"
#include "lib/led.h"
#include "lib/buzzer.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#define ADC_JOYSTICK_X 26
#define ADC_JOYSTICK_Y 27

#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define endereco 0x3C

#define LED_GREEN 11
#define LED_RED 13

#define BUZZER_A 10
#define BUZZER_B 21

#define LED_PIN 7

typedef struct
{
    uint16_t x_pos;
    uint16_t y_pos;
} joystick_data_t;

typedef struct
{
    uint16_t x_per;
    uint16_t y_per;
} percentage_data_t;

QueueHandle_t xQueueJoystickData;
QueueHandle_t xQueuePercentageData;

int normalize(int val, int min, int max);

void vJoystickTask(void *params)
{
    adc_gpio_init(ADC_JOYSTICK_Y);
    adc_gpio_init(ADC_JOYSTICK_X);
    adc_init();

    joystick_data_t joydata;

    while (true)
    {
        adc_select_input(0); // GPIO 26 = ADC0
        joydata.y_pos = adc_read();

        adc_select_input(1); // GPIO 27 = ADC1
        joydata.x_pos = adc_read();

        xQueueSend(xQueueJoystickData, &joydata, 0); // Envia o valor do joystick para a fila
        vTaskDelay(pdMS_TO_TICKS(100));              // 10 Hz de leitura
    }
}

void vDisplayTask(void *params)
{
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    ssd1306_t ssd;
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORT);
    ssd1306_config(&ssd);
    ssd1306_send_data(&ssd);

    joystick_data_t joydata;
    percentage_data_t perdata;

    bool cor = true;

    ssd1306_fill(&ssd, !cor);                     // Limpa o display
    ssd1306_rect(&ssd, 3, 3, 122, 60, cor, !cor); // Desenha um retângulo
    ssd1306_draw_string(&ssd, "ENCHENTE", 25, 6); // Desenha uma string
    ssd1306_line(&ssd, 3, 15, 123, 15, cor);      // Desenha uma linha
    ssd1306_send_data(&ssd);                      // Atualiza o display

    while (true)
    {
        if (xQueueReceive(xQueueJoystickData, &joydata, portMAX_DELAY) == pdTRUE)
        {
            int norm_x = normalize(joydata.x_pos, 0, 4080);
            int norm_y = normalize(joydata.y_pos, 0, 4080);

            char str_x[12];
            char str_y[12];

            sprintf(str_x, "%d", norm_x);
            sprintf(str_y, "%d", norm_y);

            ssd1306_fill(&ssd, false); // Limpa a tela antes de desenhar de novo
            ssd1306_rect(&ssd, 3, 3, 122, 60, true, false);
            ssd1306_draw_string(&ssd, "ENCHENTE", 30, 6);
            ssd1306_line(&ssd, 3, 15, 123, 15, true);

            ssd1306_draw_string(&ssd, "CHUVA %", 8, 25);
            ssd1306_draw_string(&ssd, str_x, 65, 25);

            ssd1306_draw_string(&ssd, "NIVEL %", 8, 45);
            ssd1306_draw_string(&ssd, str_y, 65, 45);

            ssd1306_send_data(&ssd);

            perdata.x_per = norm_x;
            perdata.y_per = norm_y;

            xQueueSend(xQueuePercentageData, &perdata, 0);

            if (norm_x >= 80 || norm_y >= 70)
            {
                ssd1306_line(&ssd, 90, 55, 105, 25, true);
                ssd1306_line(&ssd, 90, 55, 120, 55, true);
                ssd1306_line(&ssd, 105, 25, 120, 55, true);
                ssd1306_rect(&ssd, 34, 103, 4, 15, true, true);
                ssd1306_rect(&ssd, 50, 103, 4, 4, true, true);
                ssd1306_send_data(&ssd);
            }
        }
    }
}

void vLedTask(void *params)
{
    joystick_data_t joydata;
    percentage_data_t perdata;

    while (true)
    {
        if (xQueueReceive(xQueueJoystickData, &joydata, portMAX_DELAY) == pdTRUE)
        {
            if (xQueueReceive(xQueuePercentageData, &perdata, portMAX_DELAY) == pdTRUE)
            {
                if (perdata.x_per >= 80 || perdata.y_per >= 70)
                {
                    led_on(LED_RED);
                    led_off(LED_GREEN);
                }
                else
                {
                    led_off(LED_RED);
                    led_on(LED_GREEN);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // Atualiza a cada 50ms
    }
}

void vBuzzerTask(void *params)
{
    joystick_data_t joydata;
    percentage_data_t perdata;

    while (true)
    {
        if (xQueueReceive(xQueueJoystickData, &joydata, portMAX_DELAY) == pdTRUE)
        {
            if (xQueueReceive(xQueuePercentageData, &perdata, portMAX_DELAY) == pdTRUE)
            {
                if (perdata.x_per >= 80 || perdata.y_per >= 70)
                {
                    for (int i = 0; i < 5; i++)
                    {
                        buzzer_on(BUZZER_A, 4000); // Liga o buzzer
                        buzzer_on(BUZZER_B, 4000); // Liga o buzzer
                        vTaskDelay(pdMS_TO_TICKS(100));
                        buzzer_off(BUZZER_A); // Desliga o buzzer
                        buzzer_off(BUZZER_B); // Desliga o buzzer
                        vTaskDelay(pdMS_TO_TICKS(100));
                    }
                }
                else
                {
                    buzzer_off(BUZZER_A); // Desliga o buzzer
                    buzzer_off(BUZZER_B); // Desliga o buzzer
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // Atualiza a cada 50ms
    }
}

void vMatrizTask(void *params)
{
    joystick_data_t joydata;
    percentage_data_t perdata;
    npInit(LED_PIN); // Inicializa o PIO para controle dos LEDs

    while (true)
    {
        if (xQueueReceive(xQueueJoystickData, &joydata, portMAX_DELAY) == pdTRUE)
        {
            if (xQueueReceive(xQueuePercentageData, &perdata, portMAX_DELAY) == pdTRUE)
            {
                if (perdata.x_per >= 80 || perdata.y_per >= 70)
                {
                    printAlert(); // Desenha o sprite de alerta
                }
                else
                {
                    printColor(0, 0, 0); // Limpa a matriz
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20)); // Atualiza a cada 50ms
    }
}

// Modo BOOTSEL com botão B
#include "pico/bootrom.h"
#define botaoB 6
void gpio_irq_handler(uint gpio, uint32_t events)
{
    reset_usb_boot(0, 0);
}

int main()
{
    // Ativa BOOTSEL via botão
    gpio_init(botaoB);
    gpio_set_dir(botaoB, GPIO_IN);
    gpio_pull_up(botaoB);
    gpio_set_irq_enabled_with_callback(botaoB, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

    stdio_init_all();

    // Cria a fila para compartilhamento de valor do joystick
    xQueueJoystickData = xQueueCreate(5, sizeof(joystick_data_t));

    // Cria a fila para compartilhamento de valor percentual
    xQueuePercentageData = xQueueCreate(5, sizeof(percentage_data_t));

    // Criação das tasks
    xTaskCreate(vJoystickTask, "Joystick Task", 256, NULL, 1, NULL);
    xTaskCreate(vDisplayTask, "Display Task", 512, NULL, 1, NULL);
    xTaskCreate(vLedTask, "LED Task", 256, NULL, 1, NULL);
    xTaskCreate(vBuzzerTask, "Buzzer Task", 256, NULL, 1, NULL);
    xTaskCreate(vMatrizTask, "Matriz Task", 256, NULL, 1, NULL);

    // Inicia o agendador
    vTaskStartScheduler();
    panic_unsupported();
}

// Função para normalizar
int normalize(int val, int min, int max)
{
    if (val < min)
        val = min;
    if (val > max)
        val = max;
    return (int)((float)(val - min) * 100.0f / (float)(max - min));
}