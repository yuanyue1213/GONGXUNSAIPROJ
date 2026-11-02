/* ESP32-S3 / ESP-IDF 6.x: N10 UART2 to subscribed UDP viewer. */
#include <assert.h>
#include <fcntl.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "n10_protocol.h"

#define LIDAR_UART UART_NUM_2
#define UDP_PORT 3334
static int udp_fd;
static struct sockaddr_in peer;
static int64_t peer_seen;
static uint32_t sequence, valid_frames, send_errors, uart_errors;

static void send_frame(const uint8_t *frame)
{
    uint8_t packet[8 + N10_FRAME_SIZE];
    memcpy(packet, "N10U", 4);
    uint32_t seq = htonl(sequence++);
    memcpy(packet + 4, &seq, 4);
    memcpy(packet + 8, frame, N10_FRAME_SIZE);
    valid_frames++;
    if (peer_seen && esp_timer_get_time() - peer_seen < 5000000) {
        if (sendto(udp_fd, packet, sizeof(packet), 0,
                   (struct sockaddr *)&peer, sizeof(peer)) != sizeof(packet)) send_errors++;
    }
}

static void bridge_task(void *arg)
{
    QueueHandle_t events = (QueueHandle_t)arg;
    n10_parser_t parser = {0};
    uint8_t bytes[512];
    int64_t last_log = 0;
    while (true) {
        for (int i = 0; i < 8; ++i) {
            char request[32];
            struct sockaddr_in source;
            socklen_t len = sizeof(source);
            int count = recvfrom(udp_fd, request, sizeof(request), 0,
                                 (struct sockaddr *)&source, &len);
            if (count < 0) break;
            if (count == 7 && memcmp(request, "N10_SUB", 7) == 0) {
                peer = source;
                peer_seen = esp_timer_get_time();
            }
        }
        uart_event_t event;
        while (xQueueReceive(events, &event, 0)) {
            if (event.type == UART_FIFO_OVF || event.type == UART_BUFFER_FULL) {
                uart_flush_input(LIDAR_UART);
                xQueueReset(events);
                parser.used = 0;
                uart_errors++;
            } else if (event.type == UART_FRAME_ERR || event.type == UART_PARITY_ERR) {
                parser.used = 0;
                uart_errors++;
            }
        }
        int count = uart_read_bytes(LIDAR_UART, bytes, sizeof(bytes), pdMS_TO_TICKS(10));
        for (int i = 0; i < count; ++i) n10_feed(&parser, bytes[i], send_frame);
        int64_t now = esp_timer_get_time();
        if (now - last_log >= 5000000) {
            ESP_LOGI("n10", "valid=%lu invalid=%lu uart_errors=%lu send_errors=%lu viewer=%s",
                     (unsigned long)valid_frames, (unsigned long)parser.invalid,
                     (unsigned long)uart_errors, (unsigned long)send_errors,
                     peer_seen && now - peer_seen < 5000000 ? "online" : "offline");
            last_log = now;
        }
    }
}

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *ap = esp_netif_create_default_wifi_ap();
    ESP_ERROR_CHECK(ap ? ESP_OK : ESP_FAIL);
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init));
    wifi_config_t config = { .ap = {
        .ssid = "N10-LiDAR", .password = "n10lidar123",
        .channel = 6, .max_connection = 2, .authmode = WIFI_AUTH_WPA2_PSK,
    }};
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &config));
    ESP_ERROR_CHECK(esp_wifi_start());
    udp_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    assert(udp_fd >= 0);
    struct sockaddr_in local = {
        .sin_family = AF_INET, .sin_port = htons(UDP_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    ESP_ERROR_CHECK(bind(udp_fd, (struct sockaddr *)&local, sizeof(local)) == 0 ? ESP_OK : ESP_FAIL);
    ESP_ERROR_CHECK(fcntl(udp_fd, F_SETFL, O_NONBLOCK) >= 0 ? ESP_OK : ESP_FAIL);
    uart_config_t uart_config = {
        .baud_rate = 230400, .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT,
    };
    QueueHandle_t events;
    ESP_ERROR_CHECK(uart_param_config(LIDAR_UART, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(LIDAR_UART, 15, 16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(LIDAR_UART, 8192, 0, 32, &events, 0));
    ESP_LOGI("n10", "N10-LiDAR / n10lidar123; UDP 192.168.4.1:3334; UART2 RX16 TX15");
    BaseType_t created = xTaskCreate(bridge_task, "n10_bridge", 4096, events, 6, NULL);
    assert(created == pdPASS);
}
