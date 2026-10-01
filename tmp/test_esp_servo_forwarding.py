from pathlib import Path
root = Path('F:/GONGXUNSAIPROJ')
s = (root / 'yundong_part/esp32s3/hello_world/main/robot_remote_main.c').read_text(encoding='utf-8')
enum = s[s.index('typedef enum {'):s.index('static const char *TAG')] if False else s[s.index('typedef enum {'):s.index('/* 初始化桥接串口')]
a = s.index('static bool parse_command(')
b = s.index('/* 在同一个循环内', a)
code = '''#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include "../yundong_part/shared/robot_distance_protocol.h"
#include "../yundong_part/shared/camera_position_protocol.h"
#define MAX_FRAME_LENGTH 63
static char forwarded[128];
static bool send_to_stm32(const char *frame) { strcpy(forwarded, frame); return true; }
static int64_t esp_timer_get_time(void) { return 0; }
''' + enum + s[a:b] + '''
int main(void) {
    int64_t lift = 0;
    const char *commands[] = {"SERVO,1577625090,G,90", "SERVO,1577625091,T,270", "SERVO,1577625092,B,360", "CMD,1577625093,A", "CMD,1577625094,Z", "ARM_MOVE,1577625095,E,127,10,3200", "ALIGN,1577625096,9889,12000", "LIFT_ANGLE,1577625097,U,90,5,3200", "LIFT_ANGLE,1577625098,D,360,10,6400", "LIFT_MOVE,1577625099,U,10,5,3200", "LIFT_MOVE,1577625100,D,40,10,6400"};
    for (unsigned i = 0; i < sizeof(commands) / sizeof(commands[0]); ++i) {
        char expected[128];
        snprintf(expected, sizeof(expected), "%s\\n", commands[i]);
        assert(handle_frame(commands[i], &lift));
        assert(strcmp(forwarded, expected) == 0);
    }
    forwarded[0] = 0;
    assert(handle_frame("ARM_POSE,1,120,-20,40,5,3200", &lift));
    assert(strcmp(forwarded, "ARM_POSE,1,120,-20,40,5,3200\\n") == 0);
    assert(current_lift == LIFT_HOLD);
    assert(handle_frame("CMD,2,O", &lift));
    assert(handle_frame("CMD,3,I", &lift));
    assert(strcmp(forwarded, "CMD,3,I\\n") == 0);
    assert(handle_frame("CMD,2,O", &lift));
    assert(strcmp(forwarded, "CMD,2,O\\n") == 0);
    forwarded[0] = 0;
    assert(handle_frame("ARM_POSE,3,0,0,401,5,3200", &lift));
    assert(forwarded[0] == 0);
    assert(handle_frame("ALIGN,1,0,9889", &lift));
    assert(forwarded[0] == 0);
    assert(handle_frame("ALIGN,1,9889", &lift));
    assert(forwarded[0] == 0);
    assert(handle_frame("LIFT_ANGLE,1,U,0,5,3200", &lift));
    assert(forwarded[0] == 0);
    current_lift = LIFT_UP;
    assert(handle_frame("LIFT_MOVE,1,U,401,5,3200", &lift));
    assert(forwarded[0] == 0 && current_lift == LIFT_UP);
    assert(handle_frame("LIFT_MOVE,2,U,10,5,3200", &lift));
    assert(current_lift == LIFT_HOLD);
    current_lift = LIFT_UP;
    assert(handle_frame("LIFT_ANGLE,2,U,90,5,3200", &lift));
    assert(current_lift == LIFT_HOLD);
    puts("PASS: ESP32 forwards SERVO/state/ARM/ALIGN/LIFT_ANGLE commands without replies");
}
'''
(root / 'tmp/esp_servo_forwarding_test.c').write_text(code, encoding='utf-8')
