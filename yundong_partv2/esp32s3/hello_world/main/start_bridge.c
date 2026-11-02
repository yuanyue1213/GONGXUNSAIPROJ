/* 单个ESP32-S3：UART2接N10，UDP3334传点云；TCP3333接App，3335接PC。
 * UART1只在主循环逐帧转发，雷达任务不写STM32串口。 */
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "start_protocol.h"
#include "n10_protocol.h"
#define BRIDGE_UART UART_NUM_1
#define LIDAR_UART UART_NUM_2
static int udp_fd;
static struct sockaddr_in viewer;
static int64_t viewer_seen;
static volatile uint32_t sequence;
typedef struct { char data[192]; unsigned length; bool drop; } Frame;
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
static bool send_all(int fd,const char *line)
{
    size_t left=strlen(line);
    while(left) {
        int n=send(fd,line,left,0);
        if(n<0&&errno==EINTR) continue;
        if(n<=0) return false;
        line+=n; left-=(size_t)n;
    }
    return true;
}
static int listener(unsigned port)
{
    int fd=socket(AF_INET,SOCK_STREAM,IPPROTO_IP),reuse=1;
    ESP_ERROR_CHECK(fd<0?ESP_FAIL:ESP_OK);
    setsockopt(fd,SOL_SOCKET,SO_REUSEADDR,&reuse,sizeof(reuse));
    struct sockaddr_in addr={.sin_family=AF_INET,.sin_port=htons(port),.sin_addr.s_addr=htonl(INADDR_ANY)};
    ESP_ERROR_CHECK(bind(fd,(struct sockaddr *)&addr,sizeof(addr))<0?ESP_FAIL:ESP_OK);
    ESP_ERROR_CHECK(listen(fd,1)<0?ESP_FAIL:ESP_OK);
    ESP_ERROR_CHECK(fcntl(fd,F_SETFL,O_NONBLOCK)<0?ESP_FAIL:ESP_OK);
    return fd;
}
static void send_frame(const uint8_t *frame)
{
    uint8_t packet[8+N10_FRAME_SIZE];
    memcpy(packet,"N10U",4); uint32_t seq=htonl(sequence++);
    memcpy(packet+4,&seq,4); memcpy(packet+8,frame,N10_FRAME_SIZE);
    if(viewer_seen&&esp_timer_get_time()-viewer_seen<5000000)
        (void)sendto(udp_fd,packet,sizeof(packet),0,(struct sockaddr *)&viewer,sizeof(viewer));
}
static void lidar_task(void *arg)
{
    QueueHandle_t events=(QueueHandle_t)arg; n10_parser_t parser={0}; uint8_t bytes[512];
    for(;;) {
        for(unsigned i=0;i<8;++i) {
            char request[32]; struct sockaddr_in source; socklen_t size=sizeof(source);
            int n=recvfrom(udp_fd,request,sizeof(request),0,(struct sockaddr *)&source,&size);
            if(n<0) break;
            if(n==7&&!memcmp(request,"N10_SUB",7)) { viewer=source; viewer_seen=esp_timer_get_time(); }
        }
        uart_event_t event;
        while(xQueueReceive(events,&event,0)) {
            if(event.type==UART_FIFO_OVF||event.type==UART_BUFFER_FULL) {
                uart_flush_input(LIDAR_UART); xQueueReset(events); parser.used=0;
            } else if(event.type==UART_FRAME_ERR||event.type==UART_PARITY_ERR) parser.used=0;
        }
        int n=uart_read_bytes(LIDAR_UART,bytes,sizeof(bytes),pdMS_TO_TICKS(10));
        for(int i=0;i<n;++i) n10_feed(&parser,bytes[i],send_frame);
    }
}
void app_main(void)
{
    esp_err_t result=nvs_flash_init();
    if(result==ESP_ERR_NVS_NO_FREE_PAGES||result==ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase()); result=nvs_flash_init();
    }
    ESP_ERROR_CHECK(result);
    uart_config_t uart={.baud_rate=115200,.data_bits=UART_DATA_8_BITS,.parity=UART_PARITY_DISABLE,
        .stop_bits=UART_STOP_BITS_1,.flow_ctrl=UART_HW_FLOWCTRL_DISABLE,.source_clk=UART_SCLK_DEFAULT};
    ESP_ERROR_CHECK(uart_param_config(BRIDGE_UART,&uart));
    ESP_ERROR_CHECK(uart_set_pin(BRIDGE_UART,17,18,UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(BRIDGE_UART,4096,0,0,NULL,0));
    uart.baud_rate=230400;
    ESP_ERROR_CHECK(uart_param_config(LIDAR_UART,&uart));
    ESP_ERROR_CHECK(uart_set_pin(LIDAR_UART,15,16,UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE));
    QueueHandle_t events;
    ESP_ERROR_CHECK(uart_driver_install(LIDAR_UART,8192,0,32,&events,0));
    ESP_ERROR_CHECK(esp_netif_init()); ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap(); wifi_init_config_t init=WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init));
    wifi_config_t ap={.ap={.ssid="RobotCar-ESP32S3",.password="robotcar123",
        .ssid_len=sizeof("RobotCar-ESP32S3")-1,.channel=6,.authmode=WIFI_AUTH_WPA2_PSK,.max_connection=2}};
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP,&ap)); ESP_ERROR_CHECK(esp_wifi_start());
    udp_fd=socket(AF_INET,SOCK_DGRAM,IPPROTO_IP);
    ESP_ERROR_CHECK(udp_fd<0?ESP_FAIL:ESP_OK);
    struct sockaddr_in local={.sin_family=AF_INET,.sin_port=htons(3334),.sin_addr.s_addr=htonl(INADDR_ANY)};
    ESP_ERROR_CHECK(bind(udp_fd,(struct sockaddr *)&local,sizeof(local))<0?ESP_FAIL:ESP_OK);
    ESP_ERROR_CHECK(fcntl(udp_fd,F_SETFL,O_NONBLOCK)<0?ESP_FAIL:ESP_OK);
    ESP_ERROR_CHECK(xTaskCreate(lidar_task,"n10_bridge",4096,events,6,NULL)==pdPASS?ESP_OK:ESP_FAIL);
    int servers[2]={listener(3333),listener(3335)},clients[2]={-1,-1};
    Frame network[2]={0},serial={0};
    unsigned long scan_run=0; unsigned scan_no=0; uint32_t scan_first=0;
    ESP_LOGI("bridge","RobotCar-ESP32S3; App TCP3333, PC TCP3335, N10 UDP3334; RX16 TX15 / RX18 TX17");
    for(;;) {
        for(unsigned k=0;k<2;++k) if(clients[k]<0) {
            clients[k]=accept(servers[k],NULL,NULL);
            if(clients[k]>=0) {
                struct timeval timeout={.tv_sec=0,.tv_usec=50000};
                setsockopt(clients[k],SOL_SOCKET,SO_SNDTIMEO,&timeout,sizeof(timeout));
                setsockopt(clients[k],SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout));
                memset(&network[k],0,sizeof(Frame));
            }
        }
        uint8_t bytes[256]; int n=uart_read_bytes(BRIDGE_UART,bytes,sizeof(bytes),pdMS_TO_TICKS(5));
        for(int i=0;i<n;++i) if(take(&serial,(char)bytes[i])) {
            char line[224]; unsigned long id; unsigned number;
            if(sscanf(serial.data,"SCAN_READY,%lu,%u",&id,&number)==2) {
                if(id!=scan_run||number!=scan_no) { scan_run=id; scan_no=number; scan_first=sequence; }
                snprintf(line,sizeof(line),"%s,%lu\n",serial.data,(unsigned long)scan_first);
            } else snprintf(line,sizeof(line),"%s\n",serial.data);
            for(unsigned k=0;k<2;++k) if(clients[k]>=0) {
                if(k==0&&strncmp(line,"AUTO,",5)&&strncmp(line,"TASK,",5)) continue;
                if(!send_all(clients[k],line)) { close(clients[k]); clients[k]=-1; }
            }
        }
        for(unsigned k=0;k<2;++k) if(clients[k]>=0) {
            fd_set set; FD_ZERO(&set); FD_SET(clients[k],&set);
            struct timeval wait={0}; int available=select(clients[k]+1,&set,NULL,NULL,&wait);
            if(available<0) { if(errno!=EINTR) { close(clients[k]); clients[k]=-1; } continue; }
            if(!available) continue;
            n=recv(clients[k],bytes,sizeof(bytes),0);
            if(n<=0) { close(clients[k]); clients[k]=-1; continue; }
            for(int i=0;i<n;++i) if(take(&network[k],(char)bytes[i])) {
                const char *frame=network[k].data; uint32_t id;
                bool allowed=k==0?StartProtocol_Parse(frame,&id):
                    (!strncmp(frame,"PC_HELLO,",9)||!strncmp(frame,"SCAN_OK,",8)||!strncmp(frame,"NAV_",4));
                if(!allowed||strlen(frame)>=127U) continue;
                char line[194]; int size=snprintf(line,sizeof(line),"%s\n",frame);
                if(uart_write_bytes(BRIDGE_UART,line,size)!=size) {
                    send_all(clients[k],"BRIDGE_ERROR,UART_TX\n"); close(clients[k]); clients[k]=-1; break;
                }
            }
        }
    }
}
