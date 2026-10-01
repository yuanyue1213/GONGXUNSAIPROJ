/*
 * ESP32-S3 小车遥控桥接程序。
 * 数据路径：手机 App --Wi-Fi/TCP--> ESP32 --UART1--> STM32。
 * STM32 负责距离换算后的四轮电机控制、机械臂和舵机执行；
 * ESP32 负责命令校验、单向转发及通信中断时请求停车。
 * ASCII 协议每行以 \n 结束，具体字段见 esp32s3/PROTOCOL.md。
 */
#include <errno.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
#include <netinet/in.h>
#include <netinet/tcp.h>

#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "driver/uart.h"
#include "nvs_flash.h"
#include "../../../shared/robot_distance_protocol.h"
#include "../../../shared/camera_position_protocol.h"

/* 手机连接此热点，再连接 TCP 3333 端口；最多允许一个 Wi-Fi 客户端。 */
#define WIFI_SSID               "RobotCar-ESP32S3"
#define WIFI_PASSWORD           "robotcar123"
#define TCP_PORT                3333
/* 机械臂升降超过 250 ms 未续期时请求停止；底盘和前后定距无心跳超时。
 * socket 最多等待 100 ms，让主循环定期处理机械臂超时检查。
 * 命令上限为 63 字节（不含换行），超长命令丢弃到下一换行。 */
#define COMMAND_TIMEOUT_MS      250
#define SOCKET_TIMEOUT_MS       100
#define MAX_FRAME_LENGTH        63
#define STM32_UART_PORT         UART_NUM_1
/* 交叉接线：GPIO17 TX -> STM32 PB11 RX，GPIO18 RX <- STM32 PB10 TX。
 * 两板必须共地；ESP32 的 UART1 对接 STM32 的 USART3，串口编号无需相同。 */
#define STM32_UART_TX_GPIO      17
#define STM32_UART_RX_GPIO      18
#define STM32_UART_BAUD_RATE    115200
#define STM32_UART_BUFFER_SIZE  1024

static const char *TAG = "robot_remote";

/* 协议字符：S 停底盘，U/D/H 升/降/停，Q 停止前后移动；E/C 只用于 ARM_MOVE 的方向。
 * F/B/L/R 只作为 MOVE 的方向，不再接受旧的连续底盘 CMD 命令。 */
typedef enum {
    MOTION_STOP = 'S',
    MOTION_FORWARD = 'F',
    MOTION_BACKWARD = 'B',
    MOTION_LEFT = 'L',
    MOTION_RIGHT = 'R',
    LIFT_UP = 'U',
    LIFT_DOWN = 'D',
    LIFT_HOLD = 'H',
    ARM_FORE_AFT_HOLD = 'Q',
} motion_t;

/* 仅升降维护续期状态；底盘和前后定距只转发命令，不等待回包。 */
static motion_t current_lift = LIFT_HOLD;

/* 初始化桥接串口：115200 波特率、8 数据位、无校验、1 停止位。
 * 驱动提供发送缓冲；应用不读取或转发 STM32 回包。 */
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
}

/* frame 必须已包含换行。成功仅表示全部字节交给 UART 驱动，
 * 不保证 STM32 已收到或执行；本协议不返回执行结果。 */
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

/* 校验 CMD,<无符号32位序号>,<单字符动作>，不含换行。
 * 检查数字溢出、字段分隔和尾部字符；序号 0 用于自动停车命令。 */
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
    case 'I': /* App 启动原点初始化 */
    case 'O': /* 返回启动原点 */
    case 'A': /* 抓取固定状态 */
    case 'Z': /* 取消自动舵机步骤 */
    case 'S':
    case 'U':
    case 'D':
    case 'H':
    case 'Q':
        *sequence = value;
        *motion = (motion_t)cursor[1];
        return true;
    default:
        return false;
    }
}

/* SERVO,seq,channel,angle：G 夹子、T 转盘、B 基座。
 * G/T 最大 270°，B 最大 360°；角度校验后原样转交 STM32。 */
static bool parse_servo(const char *frame, uint32_t *sequence,
                        char *channel, uint16_t *angle)
{
    const char *cursor;
    char *end;
    unsigned long value;
    uint16_t maximum;

    if (strncmp(frame, "SERVO,", 6) != 0) return false;
    cursor = frame + 6;
    if (*cursor < '0' || *cursor > '9') return false;
    errno = 0;
    value = strtoul(cursor, &end, 10);
    if (errno == ERANGE || value > UINT32_MAX || *end != ',') return false;
    *sequence = (uint32_t)value;
    cursor = end + 1;
    if (cursor[0] == '\0' || cursor[1] != ',') return false;
    *channel = cursor[0];
    if (*channel != 'G' && *channel != 'T' && *channel != 'B') return false;
    maximum = (*channel == 'B') ? 360U : 270U;
    cursor += 2;
    if (*cursor < '0' || *cursor > '9') return false;
    errno = 0;
    value = strtoul(cursor, &end, 10);
    if (errno == ERANGE || *end != '\0' || value > maximum) return false;
    *angle = (uint16_t)value;
    return true;
}

/* 单向命令：非法帧直接丢弃，UART 写失败则结束连接并请求停车。
 * ESP32 不维护底盘 BUSY 状态，由 STM32 在本地判断是否接受新 MOVE。 */
static bool handle_frame(const char *frame, int64_t *last_lift_us)
{
    uint32_t sequence;
    motion_t motion;
    if (strncmp(frame, "ARM_POSE,", 9U) == 0) {
        RobotArmPoseCommand command;
        if (!RobotProtocol_ParseArmPose(frame, &command)) return true;
    } else if (strncmp(frame, "LIFT_MOVE,", 10U) == 0) {
        RobotLiftDistanceCommand command;
        if (!RobotProtocol_ParseLiftDistance(frame, &command)) return true;
    } else if (strncmp(frame, "LIFT_ANGLE,", 11U) == 0) {
        RobotLiftAngleCommand command;
        if (!RobotProtocol_ParseLiftAngle(frame, &command)) return true;
    } else if (strncmp(frame, "ALIGN,", 6U) == 0) {
        CameraAlignCommand command;
        if (!CameraProtocol_ParseAlign(frame, &command)) return true;
    } else if (strncmp(frame, "ARM_MOVE,", 9U) == 0) {
        RobotArmDistanceCommand command;
        if (!RobotProtocol_ParseArmDistance(frame, &command)) return true;
    } else if (strncmp(frame, "MOVE,", 5U) == 0) {
        RobotDistanceCommand command;
        if (!RobotProtocol_ParseMove(frame, &command) ||
            RobotProtocol_MovePulses(&command) == 0U) return true;
    } else if (strncmp(frame, "SERVO,", 6U) == 0) {
        char channel;
        uint16_t angle;
        if (!parse_servo(frame, &sequence, &channel, &angle)) return true;
    } else {
        if (!parse_command(frame, &sequence, &motion)) return true;
        char command[MAX_FRAME_LENGTH + 2];
        snprintf(command, sizeof(command), "%s\n", frame);
        if (!send_to_stm32(command)) return false;
        if (motion == LIFT_UP || motion == LIFT_DOWN || motion == LIFT_HOLD) {
            current_lift = motion;
            *last_lift_us = esp_timer_get_time();
        }
        return true;
    }
    char command[MAX_FRAME_LENGTH + 2];
    snprintf(command, sizeof(command), "%s\n", frame);
    bool sent = send_to_stm32(command);
    /* Autonomous lift angle tests do not use the continuous-lift renewal timer. */
    if (sent && (strncmp(frame, "ARM_POSE,", 9U) == 0 || strncmp(frame, "LIFT_ANGLE,", 11U) == 0 ||
                 strncmp(frame, "LIFT_MOVE,", 10U) == 0)) current_lift = LIFT_HOLD;
    return sent;
}

/* 在同一个循环内处理一个 TCP 客户端：命令组帧和机械臂超时停车。
 * recv 等待超时不是断线；返回 0 或其他网络错误才退出。 */
static void handle_client(int socket_fd)
{
    struct timeval timeout = {
        .tv_sec = 0,
        .tv_usec = SOCKET_TIMEOUT_MS * 1000,
    };
    setsockopt(socket_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    /* 小型命令/STOP 帧立即提交 TCP，减少 Nagle 合包造成的延迟。 */
    int no_delay = 1;
    setsockopt(socket_fd, IPPROTO_TCP, TCP_NODELAY, &no_delay, sizeof(no_delay));

    /* 新连接先请求三轴停止，避免继承上次连接的运动。 */
    current_lift = LIFT_HOLD;
    (void)send_to_stm32("CMD,0,Z\n");
    (void)send_to_stm32("CMD,0,S\n");
    (void)send_to_stm32("CMD,0,H\n");
    (void)send_to_stm32("CMD,0,Q\n");

    /* TCP 是字节流，一次 recv 可以包含半条或多条命令。
     * frame_length 保留半帧；超长时丢弃到换行后再接收下一帧。 */
    char frame[MAX_FRAME_LENGTH + 1];
    size_t frame_length = 0;
    bool dropping_oversized_frame = false;
    int64_t last_lift_us = esp_timer_get_time();

    while (true) {
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
                    if (!dropping_oversized_frame) {
                        /* 同时兼容 LF 和 CRLF 行结束符。 */
                        if (frame_length > 0 && frame[frame_length - 1] == '\r') {
                            --frame_length;
                        }
                        frame[frame_length] = '\0';
                        if (!handle_frame(frame,
                                          &last_lift_us)) {
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

        /* 升降按住时由 App 连续发 U/D 续期，松手 H 或超时都会请求停止。 */
        if (current_lift != LIFT_HOLD &&
            esp_timer_get_time() - last_lift_us >= COMMAND_TIMEOUT_MS * 1000LL) {
            current_lift = LIFT_HOLD;
            (void)send_to_stm32("CMD,0,H\n");
        }

    }

disconnected:
    /* 断线或转发失败时请求三轴停止，STM32 另有任务总时限和电机故障保护。
     * 此处不复位舵机角度，也不将请求停车视为已经收到停止确认。 */
    current_lift = LIFT_HOLD;
    (void)send_to_stm32("CMD,0,Z\n");
    (void)send_to_stm32("CMD,0,S\n");
    (void)send_to_stm32("CMD,0,H\n");
    (void)send_to_stm32("CMD,0,Q\n");
}

/* 初始化网络接口和事件循环，以 WPA2 热点模式提供本地遥控网络。 */
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
}

void app_main(void)
{
    /* Wi-Fi 使用 NVS；分区无空闲页或版本不兼容时擦除并重新初始化。 */
    esp_err_t nvs_result = nvs_flash_init();
    if (nvs_result == ESP_ERR_NVS_NO_FREE_PAGES ||
        nvs_result == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_result = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_result);
    start_stm32_uart();
    start_wifi_ap();

    /* 建立 IPv4 TCP 服务，监听本机所有网络接口的 3333 端口。 */
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

    /* 串行服务客户端：上一连接结束并关闭后，才 accept 下一连接。 */
    while (true) {
        int client_fd = accept(listen_fd, NULL, NULL);
        if (client_fd < 0) {
            ESP_LOGW(TAG, "accept failed: errno=%d", errno);
            continue;
        }
        handle_client(client_fd);
        close(client_fd);
    }
}
