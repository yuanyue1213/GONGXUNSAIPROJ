/* ESP 仅作为启动信号桥：网络侧只接受 START，串口侧只向 App 转发 AUTO/TASK。
 * 路线、视觉决策和运动参数均由 STM32 固件管理，网络断开不会主动停止 STM32 任务。 */
/* Transport only: no route, profile, camera request or autonomous scheduler. */
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include "driver/uart.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "start_protocol.h"

#define BRIDGE_UART UART_NUM_1
#define BRIDGE_TCP_PORT 3333
typedef struct { char data[192]; unsigned length; bool drop; } Frame;
/* TCP/串口都是字节流：以换行划分帧；超长帧整帧丢弃，直到下一换行重新同步。 */
static bool take(Frame *f,char c)
{
    if(c=='\n') {
        bool valid=!f->drop&&f->length>0;
        if(f->length&&f->data[f->length-1]=='\r') --f->length;
        f->data[f->length]=0; f->length=0; f->drop=false; return valid;
    }
    if(!f->drop) {
        if(f->length<sizeof(f->data)-1U) f->data[f->length++]=c;
        else f->drop=true;
    }
    return false;
}
/* 处理 TCP 短写和信号中断；调用方在发送失败后关闭当前客户端。 */
static bool send_all(int fd,const char *line)
{
    size_t size=strlen(line),sent=0;
    while(sent<size) {
        int n=send(fd,line+sent,size-sent,0);
        if(n<0&&errno==EINTR) continue;
        if(n<=0) return false;
        sent+=(size_t)n;
    }
    return true;
}
void app_main(void)
{
    esp_err_t result=nvs_flash_init();
    if(result==ESP_ERR_NVS_NO_FREE_PAGES||result==ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase()); result=nvs_flash_init();
    }
    ESP_ERROR_CHECK(result);
    const uart_config_t uart={.baud_rate=115200,.data_bits=UART_DATA_8_BITS,
        .parity=UART_PARITY_DISABLE,.stop_bits=UART_STOP_BITS_1,
        .flow_ctrl=UART_HW_FLOWCTRL_DISABLE,.source_clk=UART_SCLK_DEFAULT};
    ESP_ERROR_CHECK(uart_param_config(BRIDGE_UART,&uart));
    ESP_ERROR_CHECK(uart_set_pin(BRIDGE_UART,17,18,UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(BRIDGE_UART,2048,0,0,NULL,0));
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t init=WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init));
    wifi_config_t ap={.ap={.ssid="RobotCar-ESP32S3",.password="robotcar123",
        .ssid_len=sizeof("RobotCar-ESP32S3")-1,.channel=6,.authmode=WIFI_AUTH_WPA2_PSK,.max_connection=1}};
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP,&ap));
    ESP_ERROR_CHECK(esp_wifi_start());
    int server=socket(AF_INET,SOCK_STREAM,IPPROTO_IP);
    ESP_ERROR_CHECK(server<0?ESP_FAIL:ESP_OK);
    int reuse=1; setsockopt(server,SOL_SOCKET,SO_REUSEADDR,&reuse,sizeof(reuse));
    struct sockaddr_in addr={.sin_family=AF_INET,.sin_port=htons(BRIDGE_TCP_PORT),.sin_addr.s_addr=htonl(INADDR_ANY)};
    ESP_ERROR_CHECK(bind(server,(struct sockaddr *)&addr,sizeof(addr))<0?ESP_FAIL:ESP_OK);
    ESP_ERROR_CHECK(listen(server,1)<0?ESP_FAIL:ESP_OK);
    ESP_ERROR_CHECK(fcntl(server,F_SETFL,O_NONBLOCK)<0?ESP_FAIL:ESP_OK);
    int client=-1; Frame network={0},serial={0};
    for(;;) {
        if(client<0) {
            client=accept(server,NULL,NULL);
            if(client>=0) {
                struct timeval timeout={.tv_sec=0,.tv_usec=50000};
                setsockopt(client,SOL_SOCKET,SO_SNDTIMEO,&timeout,sizeof(timeout));
                setsockopt(client,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout));
                memset(&network,0,sizeof(network));
            }
        }
        unsigned char bytes[256];
        /* 没有客户端时也持续读取串口，避免旧状态堆积后在重连时误显示。 */
        int n=uart_read_bytes(BRIDGE_UART,bytes,sizeof(bytes),pdMS_TO_TICKS(10));
        for(int i=0;i<n;++i) if(take(&serial,(char)bytes[i])&&client>=0) {
            if(strncmp(serial.data,"AUTO,",5U)!=0&&strncmp(serial.data,"TASK,",5U)!=0) continue;
            char line[194]; snprintf(line,sizeof(line),"%s\n",serial.data);
            if(!send_all(client,line)) { close(client); client=-1; }
        }
        if(client<0) continue;
        fd_set readset; FD_ZERO(&readset); FD_SET(client,&readset);
        struct timeval wait={.tv_sec=0,.tv_usec=10000};
        int ready=select(client+1,&readset,NULL,NULL,&wait);
        if(ready<0) { if(errno!=EINTR) { close(client); client=-1; } continue; }
        if(!ready) continue;
        n=recv(client,bytes,sizeof(bytes),0);
        if(n<=0) { close(client); client=-1; continue; }
        for(int i=0;i<n;++i) if(take(&network,(char)bytes[i])) {
            uint32_t id;
            /* 网络输入必须通过共用 START 解析器，不能转发任意内部运动指令。 */
            if(!StartProtocol_Parse(network.data,&id)) continue;
            char line[32]; int length=snprintf(line,sizeof(line),"START,%lu\n",(unsigned long)id);
            if(uart_write_bytes(BRIDGE_UART,line,length)!=length) {
                send_all(client,"BRIDGE_ERROR,UART_TX\n");
                close(client); client=-1; break;
            }
        }
    }
}
