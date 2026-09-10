/*
 * N10 lidar UART-to-Wi-Fi bridge for ESP32-C3.
 *
 * Wiring (3.3 V TTL, common GND):
 *   STM32 PB13 / UART5_TX -> XIAO ESP32-C3 D7 / GPIO20 / UART1_RX
 *   STM32 PB12 / UART5_RX <- XIAO ESP32-C3 D6 / GPIO21 / UART1_TX (optional)
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <sys/socket.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"

#define LIDAR_UART               UART_NUM_1
#define LIDAR_UART_RX_GPIO       20  /* XIAO board pin D7 */
#define LIDAR_UART_TX_GPIO       21  /* XIAO board pin D6 */
#define LIDAR_UART_BAUD          230400

#define N10_FRAME_SIZE           58U
#define N10_MAX_SCAN_FRAMES      64U
#define N10_FRAMES_PER_PACKET    20U
#define N10_UDP_PORT             3333
#define N10_UDP_HEADER_SIZE      14U

#define LIDAR_AP_SSID            "N10-LiDAR"
#define LIDAR_AP_PASSWORD        "lidar12345"

static const char *TAG = "n10_bridge";

static uint8_t scan_frames[N10_MAX_SCAN_FRAMES][N10_FRAME_SIZE];
static uint16_t scan_frame_count;
static uint16_t previous_start_angle;
static bool have_previous_angle;
static uint32_t scan_id;

static int udp_socket = -1;
static struct sockaddr_storage subscriber_address;
static socklen_t subscriber_address_length;
static bool subscriber_present;
static SemaphoreHandle_t subscriber_lock;

static void write_le16(uint8_t *destination, uint16_t value)
{
    destination[0] = (uint8_t)value;
    destination[1] = (uint8_t)(value >> 8U);
}

static void write_le32(uint8_t *destination, uint32_t value)
{
    destination[0] = (uint8_t)value;
    destination[1] = (uint8_t)(value >> 8U);
    destination[2] = (uint8_t)(value >> 16U);
    destination[3] = (uint8_t)(value >> 24U);
}

static bool n10_frame_is_valid(const uint8_t *frame)
{
    uint8_t checksum = 0U;

    if (frame[0] != 0xA5U || frame[1] != 0x5AU || frame[2] != N10_FRAME_SIZE) {
        return false;
    }
    for (uint16_t index = 0U; index < N10_FRAME_SIZE - 1U; ++index) {
        checksum = (uint8_t)(checksum + frame[index]);
    }
    return checksum == frame[N10_FRAME_SIZE - 1U];
}

static uint16_t n10_start_angle(const uint8_t *frame)
{
    return ((uint16_t)frame[5] << 8U) | frame[6];
}

static void send_completed_scan(void)
{
    struct sockaddr_storage target_address;
    socklen_t target_address_length;
    uint8_t packet[N10_UDP_HEADER_SIZE + N10_FRAMES_PER_PACKET * N10_FRAME_SIZE];
    uint16_t packet_count;
    bool can_send;

    if (scan_frame_count == 0U) {
        return;
    }

    xSemaphoreTake(subscriber_lock, portMAX_DELAY);
    can_send = subscriber_present && udp_socket >= 0;
    if (can_send) {
        memcpy(&target_address, &subscriber_address, sizeof(target_address));
        target_address_length = subscriber_address_length;
    }
    xSemaphoreGive(subscriber_lock);

    if (!can_send) {
        return;
    }

    packet_count = (scan_frame_count + N10_FRAMES_PER_PACKET - 1U) / N10_FRAMES_PER_PACKET;
    for (uint16_t packet_index = 0U; packet_index < packet_count; ++packet_index) {
        const uint16_t first_frame = packet_index * N10_FRAMES_PER_PACKET;
        uint16_t frames_in_packet = scan_frame_count - first_frame;
        if (frames_in_packet > N10_FRAMES_PER_PACKET) {
            frames_in_packet = N10_FRAMES_PER_PACKET;
        }

        memcpy(packet, "N10S", 4U);
        write_le32(&packet[4], scan_id);
        write_le16(&packet[8], packet_index);
        write_le16(&packet[10], packet_count);
        write_le16(&packet[12], frames_in_packet);
        memcpy(&packet[N10_UDP_HEADER_SIZE], scan_frames[first_frame],
               frames_in_packet * N10_FRAME_SIZE);

        if (sendto(udp_socket, packet, N10_UDP_HEADER_SIZE + frames_in_packet * N10_FRAME_SIZE,
                   0, (const struct sockaddr *)&target_address, target_address_length) < 0) {
            ESP_LOGW(TAG, "UDP scan send failed: errno %d", errno);
            break;
        }
    }
}

static void consume_valid_n10_frame(const uint8_t *frame)
{
    const uint16_t start_angle = n10_start_angle(frame);

    /* A large backwards angle jump terminates the preceding 360-degree scan. */
    if (have_previous_angle && (uint32_t)start_angle + 18000U < previous_start_angle) {
        send_completed_scan();
        scan_frame_count = 0U;
        ++scan_id;
    }

    if (scan_frame_count < N10_MAX_SCAN_FRAMES) {
        memcpy(scan_frames[scan_frame_count], frame, N10_FRAME_SIZE);
        ++scan_frame_count;
    } else {
        ESP_LOGW(TAG, "Scan has more than %u frames; dropping extras", N10_MAX_SCAN_FRAMES);
    }
    previous_start_angle = start_angle;
    have_previous_angle = true;
}

static void lidar_uart_task(void *argument)
{
    uint8_t input[128];
    uint8_t frame[N10_FRAME_SIZE];
    uint16_t frame_index = 0U;

    while (true) {
        const int received = uart_read_bytes(LIDAR_UART, input, sizeof(input), pdMS_TO_TICKS(100));
        for (int input_index = 0; input_index < received; ++input_index) {
            const uint8_t byte = input[input_index];

            if (frame_index == 0U) {
                if (byte == 0xA5U) {
                    frame[frame_index++] = byte;
                }
                continue;
            }
            if (frame_index == 1U) {
                if (byte == 0x5AU) {
                    frame[frame_index++] = byte;
                } else if (byte == 0xA5U) {
                    frame[0] = byte;
                } else {
                    frame_index = 0U;
                }
                continue;
            }

            frame[frame_index++] = byte;
            if (frame_index == 3U && frame[2] != N10_FRAME_SIZE) {
                frame_index = 0U;
            } else if (frame_index == N10_FRAME_SIZE) {
                if (n10_frame_is_valid(frame)) {
                    consume_valid_n10_frame(frame);
                } else {
                    ESP_LOGW(TAG, "Discarded N10 frame with bad checksum");
                }
                frame_index = 0U;
            }
        }
    }
}

static void udp_subscription_task(void *argument)
{
    struct sockaddr_in listen_address = {
        .sin_family = AF_INET,
        .sin_port = htons(N10_UDP_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    uint8_t command[32];

    udp_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (udp_socket < 0 || bind(udp_socket, (struct sockaddr *)&listen_address,
                               sizeof(listen_address)) < 0) {
        ESP_LOGE(TAG, "Cannot open UDP port %u", N10_UDP_PORT);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "UDP subscription port ready: %u", N10_UDP_PORT);
    while (true) {
        struct sockaddr_storage source_address;
        socklen_t source_address_length = sizeof(source_address);
        const int length = recvfrom(udp_socket, command, sizeof(command), 0,
                                    (struct sockaddr *)&source_address, &source_address_length);
        if (length == 13 && memcmp(command, "N10_SUBSCRIBE", 13U) == 0) {
            xSemaphoreTake(subscriber_lock, portMAX_DELAY);
            memcpy(&subscriber_address, &source_address, sizeof(source_address));
            subscriber_address_length = source_address_length;
            subscriber_present = true;
            xSemaphoreGive(subscriber_lock);
            sendto(udp_socket, "N10_OK", 6U, 0, (struct sockaddr *)&source_address,
                   source_address_length);
            ESP_LOGI(TAG, "PC subscriber registered");
        }
    }
}

static void start_wifi_access_point(void)
{
    wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
    wifi_config_t ap_config = {
        .ap = {.channel = 1, .max_connection = 1, .authmode = WIFI_AUTH_WPA2_PSK},
    };

    memcpy(ap_config.ap.ssid, LIDAR_AP_SSID, sizeof(LIDAR_AP_SSID));
    memcpy(ap_config.ap.password, LIDAR_AP_PASSWORD, sizeof(LIDAR_AP_PASSWORD));
    ap_config.ap.ssid_len = strlen(LIDAR_AP_SSID);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_config));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "Wi-Fi AP ready: SSID %s, IP 192.168.4.1", LIDAR_AP_SSID);
}

void app_main(void)
{
    esp_err_t nvs_status = nvs_flash_init();
    if (nvs_status == ESP_ERR_NVS_NO_FREE_PAGES || nvs_status == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_status = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_status);

    const uart_config_t uart_config = {
        .baud_rate = LIDAR_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(LIDAR_UART, 2048, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(LIDAR_UART, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(LIDAR_UART, LIDAR_UART_TX_GPIO, LIDAR_UART_RX_GPIO,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    subscriber_lock = xSemaphoreCreateMutex();
    if (subscriber_lock == NULL) {
        ESP_LOGE(TAG, "Cannot allocate subscriber mutex");
        return;
    }
    start_wifi_access_point();
    xTaskCreate(udp_subscription_task, "n10_udp", 4096, NULL, 5, NULL);
    xTaskCreate(lidar_uart_task, "n10_uart", 4096, NULL, 6, NULL);
}
