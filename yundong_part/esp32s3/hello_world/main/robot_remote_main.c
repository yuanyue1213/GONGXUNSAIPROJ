#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
#include <netinet/in.h>

#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "driver/uart.h"
#include "nvs_flash.h"

#define WIFI_SSID               "RobotCar-ESP32S3"
#define WIFI_PASSWORD           "robotcar123"
#define TCP_PORT                3333
#define COMMAND_TIMEOUT_MS      250
#define SOCKET_TIMEOUT_MS       100
#define MAX_FRAME_LENGTH        63
#define STM32_UART_PORT         UART_NUM_1
/* ESP32-S3 TX -> STM32 PB11, ESP32-S3 RX <- STM32 PB10. */
#define STM32_UART_TX_GPIO      17
#define STM32_UART_RX_GPIO      18
#define STM32_UART_BAUD_RATE    115200
#define STM32_UART_BUFFER_SIZE  1024

static const char *TAG = "robot_remote";

typedef enum {
    MOTION_STOP = 'S',
    MOTION_FORWARD = 'F',
    MOTION_BACKWARD = 'B',
    MOTION_LEFT = 'L',
    MOTION_RIGHT = 'R',
} motion_t;

static motion_t current_motion = MOTION_STOP;

static void set_motion(motion_t motion)
{
    if (current_motion != motion) {
        current_motion = motion;
        ESP_LOGI(TAG, "motion=%c", (char)motion);
    }
}

static void start_stm32_uart(void)
{
    const uart_config_t config = {
        .baud_rate = STM32_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_param_config(STM32_UART_PORT, &config));
    ESP_ERROR_CHECK(uart_set_pin(STM32_UART_PORT, STM32_UART_TX_GPIO,
                                 STM32_UART_RX_GPIO, UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(STM32_UART_PORT,
                                        STM32_UART_BUFFER_SIZE,
                                        STM32_UART_BUFFER_SIZE,
                                        0, NULL, 0));
    ESP_LOGI(TAG, "STM32 UART ready: UART%d TX=%d RX=%d, %d baud",
             STM32_UART_PORT, STM32_UART_TX_GPIO, STM32_UART_RX_GPIO,
             STM32_UART_BAUD_RATE);
}

static bool send_to_stm32(const char *frame)
{
    int expected = (int)strlen(frame);
    int written = uart_write_bytes(STM32_UART_PORT, frame, expected);
    if (written != expected) {
        ESP_LOGE(TAG, "STM32 UART write failed: expected=%d actual=%d",
                 expected, written);
        return false;
    }
    return true;
}

static bool send_all(int socket_fd, const char *data)
{
    size_t remaining = strlen(data);
    while (remaining > 0) {
        ssize_t sent = send(socket_fd, data, remaining, 0);
        if (sent <= 0) {
            return false;
        }
        data += sent;
        remaining -= (size_t)sent;
    }
    return true;
}

static bool parse_command(const char *frame, uint32_t *sequence, motion_t *motion)
{
    if (strncmp(frame, "CMD,", 4) != 0) {
        return false;
    }

    const char *cursor = frame + 4;
    if (*cursor < '0' || *cursor > '9') {
        return false;
    }

    uint32_t value = 0;
    do {
        uint32_t digit = (uint32_t)(*cursor - '0');
        if (value > (UINT32_MAX - digit) / 10U) {
            return false;
        }
        value = value * 10U + digit;
        ++cursor;
    } while (*cursor >= '0' && *cursor <= '9');

    if (cursor[0] != ',' || cursor[1] == '\0' || cursor[2] != '\0') {
        return false;
    }

    switch (cursor[1]) {
    case 'S':
    case 'F':
    case 'B':
    case 'L':
    case 'R':
        *sequence = value;
        *motion = (motion_t)cursor[1];
        return true;
    default:
        return false;
    }
}

/* Forwards complete STM32 lines to the phone as STM,<original line>\n. */
static bool forward_stm32_messages(int socket_fd, char *frame,
                                   size_t *frame_length)
{
    uint8_t incoming[128];
    int received = uart_read_bytes(STM32_UART_PORT, incoming,
                                   sizeof(incoming), 0);

    for (int index = 0; index < received; ++index) {
        char character = (char)incoming[index];
        if (character == '\n') {
            if (*frame_length > 0 && frame[*frame_length - 1] == '\r') {
                --*frame_length;
            }
            frame[*frame_length] = '\0';
            if (*frame_length > 0) {
                char response[MAX_FRAME_LENGTH + 6];
                snprintf(response, sizeof(response), "STM,%s\n", frame);
                if (!send_all(socket_fd, response)) {
                    return false;
                }
            }
            *frame_length = 0;
        } else if (*frame_length < MAX_FRAME_LENGTH) {
            frame[(*frame_length)++] = character;
        } else {
            /* Drop the malformed line and resume on its next newline. */
            *frame_length = 0;
        }
    }
    return true;
}

static bool handle_frame(int socket_fd, const char *frame,
                         int64_t *last_command_us)
{
    uint32_t sequence;
    motion_t motion;
    if (!parse_command(frame, &sequence, &motion)) {
        return send_all(socket_fd, "ERR,0,BAD_FRAME\n");
    }

    char stm32_frame[MAX_FRAME_LENGTH + 2];
    snprintf(stm32_frame, sizeof(stm32_frame), "%s\n", frame);
    if (!send_to_stm32(stm32_frame)) {
        char error[40];
        snprintf(error, sizeof(error), "ERR,%lu,STM_TX\n",
                 (unsigned long)sequence);
        return send_all(socket_fd, error);
    }

    set_motion(motion);
    *last_command_us = esp_timer_get_time();

    char response[40];
    /* ACK confirms that ESP32 accepted and forwarded the command. STM32's
     * execution result arrives later as STM,EXEC,<seq>,<dir>. */
    snprintf(response, sizeof(response), "ACK,%lu,%c\n",
             (unsigned long)sequence, (char)motion);
    return send_all(socket_fd, response);
}

static void handle_client(int socket_fd)
{
    struct timeval timeout = {
        .tv_sec = 0,
        .tv_usec = SOCKET_TIMEOUT_MS * 1000,
    };
    setsockopt(socket_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(socket_fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

    set_motion(MOTION_STOP);
    (void)send_to_stm32("CMD,0,S\n");
    if (!send_all(socket_fd, "HELLO,1\n")) {
        return;
    }

    char frame[MAX_FRAME_LENGTH + 1];
    char stm32_frame[MAX_FRAME_LENGTH + 1];
    size_t frame_length = 0;
    size_t stm32_frame_length = 0;
    bool dropping_oversized_frame = false;
    int64_t last_command_us = esp_timer_get_time();

    while (true) {
        if (!forward_stm32_messages(socket_fd, stm32_frame,
                                    &stm32_frame_length)) {
            break;
        }

        char incoming[128];
        ssize_t received = recv(socket_fd, incoming, sizeof(incoming), 0);
        if (received == 0) {
            break;
        }
        if (received < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != ETIMEDOUT) {
            break;
        }

        if (received > 0) {
            for (ssize_t index = 0; index < received; ++index) {
                char character = incoming[index];
                if (character == '\n') {
                    if (dropping_oversized_frame) {
                        if (!send_all(socket_fd, "ERR,0,TOO_LONG\n")) {
                            goto disconnected;
                        }
                    } else {
                        if (frame_length > 0 && frame[frame_length - 1] == '\r') {
                            --frame_length;
                        }
                        frame[frame_length] = '\0';
                        if (!handle_frame(socket_fd, frame, &last_command_us)) {
                            goto disconnected;
                        }
                    }
                    frame_length = 0;
                    dropping_oversized_frame = false;
                } else if (!dropping_oversized_frame) {
                    if (frame_length < MAX_FRAME_LENGTH) {
                        frame[frame_length++] = character;
                    } else {
                        dropping_oversized_frame = true;
                    }
                }
            }
        }

        /* A deliberate S command already stopped the car.  Only report a
         * timeout when a moving car actually loses its command stream. */
        if (current_motion != MOTION_STOP &&
            esp_timer_get_time() - last_command_us >= COMMAND_TIMEOUT_MS * 1000LL) {
            set_motion(MOTION_STOP);
            (void)send_to_stm32("CMD,0,S\n");
            if (!send_all(socket_fd, "EVENT,STOP,TIMEOUT\n")) {
                break;
            }
        }
    }

disconnected:
    set_motion(MOTION_STOP);
    (void)send_to_stm32("CMD,0,S\n");
    ESP_LOGI(TAG, "client disconnected; motion stopped");
}

static void start_wifi_ap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_config));

    wifi_config_t ap_config = {
        .ap = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
            .ssid_len = sizeof(WIFI_SSID) - 1,
            .channel = 6,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .max_connection = 1,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "Wi-Fi AP ready: SSID=%s, TCP port=%d", WIFI_SSID, TCP_PORT);
}

void app_main(void)
{
    esp_err_t nvs_result = nvs_flash_init();
    if (nvs_result == ESP_ERR_NVS_NO_FREE_PAGES ||
        nvs_result == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_result = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_result);
    start_stm32_uart();
    start_wifi_ap();

    int listen_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (listen_fd < 0) {
        ESP_LOGE(TAG, "socket failed: errno=%d", errno);
        return;
    }
    int reuse_address = 1;
    setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR,
               &reuse_address, sizeof(reuse_address));

    struct sockaddr_in address = {
        .sin_family = AF_INET,
        .sin_port = htons(TCP_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(listen_fd, (struct sockaddr *)&address, sizeof(address)) != 0 ||
        listen(listen_fd, 1) != 0) {
        ESP_LOGE(TAG, "bind/listen failed: errno=%d", errno);
        close(listen_fd);
        return;
    }

    while (true) {
        int client_fd = accept(listen_fd, NULL, NULL);
        if (client_fd < 0) {
            ESP_LOGW(TAG, "accept failed: errno=%d", errno);
            continue;
        }
        ESP_LOGI(TAG, "client connected");
        handle_client(client_fd);
        close(client_fd);
    }
}
