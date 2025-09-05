#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include "esp_mac.h"
#include "esp_random.h"
#include "bootloader_random.h"

static const int UART_PORT = UART_NUM_2;
static const int RX_PIN = 32;
static const int TX_PIN = 33;

#define BUF_SIZE 2048
static const size_t UART_QUEUE_SIZE = 10;
static QueueHandle_t uart_queue;

static const char *TAG = "Received UART Message";

static uint8_t rxbuf[BUF_SIZE];
static const size_t MAX_MESSAGE_LENGTH = 128;

static const uint32_t MIN_MESSAGE_DELAY_TIME_MS = 1000;  // 1s
static const uint32_t MAX_MESSAGE_DELAY_TIME_MS = 10000; // 10s

static uint32_t getRandomNumberInRange(const uint32_t MIN, const uint32_t MAX)
{
    uint32_t number;

    bootloader_random_enable();
    number = esp_random();
    bootloader_random_disable();

    return MIN + (number % (MAX - MIN + 1));
}

static void uart_rx_event_task(void *arg)
{
    uart_event_t ev;

    for (;;) {
        // Waits for an event from the queue (does NOT contain the data, just the type and meta)
        if (xQueueReceive(uart_queue, &ev, portMAX_DELAY)) {
            switch (ev.type) {

            case UART_DATA: {
                while (1) {
                    size_t available = 0;
                    uart_get_buffered_data_len(UART_PORT, &available);
                    if (available == 0) break;

                    size_t to_read = available;
                    if (to_read > BUF_SIZE) to_read = BUF_SIZE;

                    int n = uart_read_bytes(UART_PORT, rxbuf, to_read, 0); // timeout 0: nonblocking
                    if (n <= 0) break; // nothing to read

                    ESP_LOGI(TAG, "\"%.*s\"", n, (char *)rxbuf);
                }
            } break;


            case UART_FIFO_OVF:
            case UART_BUFFER_FULL:
                // Hardware FIFO or RX ring-buffer full → clear and reset the queue
                uart_flush_input(UART_PORT);
                xQueueReset(uart_queue);
                break;

            case UART_BREAK:
            case UART_PARITY_ERR:
            case UART_FRAME_ERR:
                uart_flush_input(UART_PORT);
                xQueueReset(uart_queue);
                break;

            default:
                break;
            }
        }
    }

    vTaskDelete(NULL);
}

static void uart_tx_task(void *arg)
{
    TickType_t now;

    uint8_t mac[6];
    ESP_ERROR_CHECK(esp_base_mac_addr_get(mac));  // read base MAC

    char mac_str[18];
    snprintf(mac_str, sizeof(mac_str),
             "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(getRandomNumberInRange(MIN_MESSAGE_DELAY_TIME_MS, MAX_MESSAGE_DELAY_TIME_MS)));
        now = xTaskGetTickCount();

        char message[MAX_MESSAGE_LENGTH];
        int n = snprintf(message, sizeof(message), "Message sent from device with MAC: %s, via uart instance number %d, TX pin number %d, at tick %" PRIu32, mac_str, UART_PORT, TX_PIN, (uint32_t)now);
        if (n > 0){
            uart_write_bytes(UART_PORT, message, n);
        }
    }
}

static void config_uart(void)
{
    uart_config_t cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // used to avoid gibberish if RX_PIN is not connected
    gpio_set_pull_mode(RX_PIN, GPIO_PULLUP_ONLY);

    // Driver + ring-buffers + event queue
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, BUF_SIZE, BUF_SIZE, UART_QUEUE_SIZE, &uart_queue, 0));
}

void app_main(void)
{
    config_uart();

    // Task that processes rx events
    xTaskCreate(uart_rx_event_task, "uart_rx_event_task", 4096, NULL, 12, NULL);
    // Task that processes tx events
    xTaskCreate(uart_tx_task, "uart_tx_task", 4096, NULL, 12, NULL);
}
