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
#include "../yundong_part/shared/camera_pose_math.h"
#define MAX_FRAME_LENGTH 127
static char forwarded[128];
static bool send_to_stm32(const char *frame) { strcpy(forwarded, frame); return true; }
static int64_t esp_timer_get_time(void) { return 0; }
''' + enum + s[a:b] + '''
int main(void) {
    int64_t lift = 0;
    const char *commands[] = {"ALIGN_POSE,1004,2,9889,9889,10,20,10,300,250,0,0,0", "BASE,1002,360,30", "BASE,1003,0,1", "ORIGIN,1000,251", "PLAN_BEGIN,1001,2", "PLAN_ITEM,1001,0,A,1100,-50,200,-40,20,20,50,60,0,60,0,248,140", "PLAN_ITEM,1001,1,P,200,-40,1300,-110,20,20,50,60,0,60,0,248,140", "PLAN_RUN,1001", "SERVO,1577625090,G,90", "SERVO,1577625091,T,270", "SERVO,1577625092,B,360", "CMD,1577625093,A", "CMD,1577625094,Z", "ARM_MOVE,1577625095,E,127,10,3200", "ALIGN,1577625096,9889,12000", "ALIGN_RING,1577625120,2,9889,12000", "STATE,1577625121,P,200,-40,1300,-110,20,20,50", "STATE,1577625122,P,200,-40,1300,-110,20,20,50,30", "ALIGN_CFG,1577625123,2,9889,12000,12,30,-7", "GRIP,1577625124,0,60,30", "GRIP_STOP,1577625125", "PARALLEL,1577625127,9889,10,3,0", "MOVE,1577625128,C,5,10,9889", "MOVE,1577625129,W,5,10,9889", "STATE,1577625126,P,200,-40,1300,-110,120,120,120,40,30,100,20,250,130", "LIFT_ANGLE,1577625097,U,90,5,3200", "LIFT_ANGLE,1577625098,D,360,10,6400", "LIFT_MOVE,1577625099,U,10,5,3200", "LIFT_MOVE,1577625100,D,40,10,6400"};
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
    assert(handle_frame("CMD,4,P", &lift));
    assert(strcmp(forwarded, "CMD,4,P\\n") == 0);
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
    assert(handle_frame("STATE,1,P,200,-40,1300,-110,20,20,121", &lift));
    assert(handle_frame("ALIGN_CFG,1,2,9889,12000,0,20,10", &lift));
    assert(handle_frame("STATE,1,P,200,-40,1300,-110,20,20,50,0", &lift));
    forwarded[0] = 0;
    assert(handle_frame("ORIGIN,1,361", &lift)); assert(forwarded[0] == 0);
    assert(handle_frame("PLAN_BEGIN,1,17", &lift)); assert(forwarded[0] == 0);
    assert(handle_frame("PLAN_ITEM,1,16,A,0,0,0,0,20,20,50", &lift)); assert(forwarded[0] == 0);
    forwarded[0] = 0;
    assert(handle_frame("BASE,1,-1,60", &lift)); assert(forwarded[0] == 0);
    assert(handle_frame("BASE,1,361,60", &lift)); assert(forwarded[0] == 0);
    assert(handle_frame("BASE,1,90,0", &lift)); assert(forwarded[0] == 0);
    puts("PASS: ESP32 forwards SERVO/state/ARM/ALIGN/LIFT_ANGLE commands without replies");
}
'''
(root / 'tmp/esp_servo_forwarding_test.c').write_text(code, encoding='utf-8')
