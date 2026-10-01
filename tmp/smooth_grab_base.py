from pathlib import Path
root=Path('F:/GONGXUNSAIPROJ/yundong_part')
p=root/'stm32h743/zhukong/Core/Src/robot_control.c'
s=p.read_text(encoding='utf-8')
a=s.index('/* 当前步骤停留时间')
b=s.index('static bool RobotControl_Parse(',a)
s=s[:a]+'''#define ROBOT_GRAB_HOLD_MS       1000U
#define ROBOT_BASE_ROTATE_MS     3000U
#define ROBOT_BASE_UPDATE_MS       20U
static bool s_grab_active;
static bool s_grab_base_rotating;
static uint8_t s_grab_step;
static uint32_t s_grab_tick;
static uint32_t s_base_update_tick;
static uint32_t s_last_grab_sequence;

static bool RobotControl_ApplyGrabStep(void)
{
    const uint16_t *angles = s_grab_angles[s_grab_step];
    /* 第 3、6 步基座以之前姿态为起点，随后 3 秒内渐变到目标。
     * 转盘和夹子在进入步骤时设定；渐变仅影响基座。 */
    s_grab_base_rotating = (s_grab_step == 2U || s_grab_step == 5U);
    s_base_update_tick = HAL_GetTick();
    uint16_t base = s_grab_base_rotating ? s_grab_angles[s_grab_step - 1U][2] : angles[2];
    return ServoControl_SetAngle('T', angles[0]) &&
           ServoControl_SetAngle('G', angles[1]) &&
           ServoControl_SetAngle('B', base);
}

static void RobotControl_TickGrab(void)
{
    if (!s_grab_active) return;
    uint32_t now = HAL_GetTick();
    uint32_t elapsed = (uint32_t)(now - s_grab_tick);
    if (s_grab_base_rotating)
    {
        if (elapsed < ROBOT_BASE_ROTATE_MS &&
            (uint32_t)(now - s_base_update_tick) < ROBOT_BASE_UPDATE_MS) return;
        uint32_t progress = elapsed < ROBOT_BASE_ROTATE_MS ? elapsed : ROBOT_BASE_ROTATE_MS;
        int32_t start = s_grab_angles[s_grab_step - 1U][2];
        int32_t delta = (int32_t)s_grab_angles[s_grab_step][2] - start;
        int32_t numerator = delta * (int32_t)progress;
        numerator += delta < 0 ? -(int32_t)(ROBOT_BASE_ROTATE_MS / 2U) :
                                 (int32_t)(ROBOT_BASE_ROTATE_MS / 2U);
        uint16_t angle = (uint16_t)(start + numerator / (int32_t)ROBOT_BASE_ROTATE_MS);
        if (!ServoControl_SetAngle('B', angle)) { s_grab_active = false; return; }
        s_base_update_tick = now;
        if (elapsed >= ROBOT_BASE_ROTATE_MS)
        {
            s_grab_base_rotating = false;
            s_grab_tick = now; /* 到达目标后，保持 1 秒再推进下一步。 */
        }
        return;
    }
    if (elapsed < ROBOT_GRAB_HOLD_MS) return;
    if (++s_grab_step >= sizeof(s_grab_angles) / sizeof(s_grab_angles[0]))
        s_grab_active = false;
    else
    {
        s_grab_active = RobotControl_ApplyGrabStep();
        s_grab_tick = HAL_GetTick();
    }
}

'''+s[b:]
a=s.index('    if (s_grab_active &&',s.index('void RobotControl_Tick(void)'))
b=s.index('    /* 停车失败期间',a)
s=s[:a]+'    RobotControl_TickGrab();\n'+s[b:]
s=s.replace('    s_grab_active = false;\n    s_grab_step = 0U;', '    s_grab_active = false;\n    s_grab_base_rotating = false;\n    s_grab_step = 0U;')
p.write_text(s,encoding='utf-8')
p=root/'tests/distance_control_test.c'
s=p.read_text(encoding='utf-8')
a=s.index('static void test_grab_state(void)')
b=s.index('int main(void)',a)
s=s[:a]+'''static void test_grab_state(void)
{
    reset(); feed("CMD,20,A\\n");
    assert(servo_commands == 3 && servo_angle == 263);
    feed("CMD,21,A\\n");
    tick = 999; RobotControl_Tick(); assert(servo_commands == 3);
    tick = 1000; RobotControl_Tick();
    assert(servo_commands == 6 && servo_history[4] == 0);
    tick = 2000; RobotControl_Tick(); /* Step 3 starts, no instantaneous jump. */
    assert(servo_commands == 9 && servo_angle == 263);
    tick = 2019; RobotControl_Tick(); assert(servo_commands == 9);
    tick = 2020; RobotControl_Tick(); assert(servo_angle == 262);
    tick = 3500; RobotControl_Tick(); assert(servo_angle == 203);
    tick = 4980; RobotControl_Tick(); assert(servo_angle == 145);
    tick = 5000; RobotControl_Tick(); assert(servo_angle == 144);
    unsigned reached = servo_commands;
    tick = 5999; RobotControl_Tick(); assert(servo_commands == reached);
    tick = 6000; RobotControl_Tick(); /* Step 4: open gripper. */
    assert(servo_history[servo_commands - 2] == 60);
    tick = 7000; RobotControl_Tick();
    assert(servo_history[servo_commands - 3] == 120);
    tick = 8000; RobotControl_Tick(); /* Step 6: turntable 240, base ramps from 144. */
    assert(servo_history[servo_commands - 3] == 240 && servo_angle == 144);
    tick = 9500; RobotControl_Tick(); assert(servo_angle == 204);
    tick = 11000; RobotControl_Tick(); assert(servo_angle == 263);
    tick = 12000; RobotControl_Tick();
    unsigned complete = servo_commands;
    feed("CMD,20,A\\n"); assert(servo_commands == complete);

    reset(); feed("CMD,30,A\\n");
    tick = 1000; RobotControl_Tick(); tick = 2000; RobotControl_Tick();
    tick = 2500; RobotControl_Tick();
    unsigned before_cancel = servo_commands;
    feed("CMD,31,Z\\n"); tick = 5000; RobotControl_Tick();
    assert(servo_commands == before_cancel); /* Cancelling mid-ramp stops updates. */
    feed("CMD,32,A\\nSERVO,33,G,90\\n");
    unsigned manual = servo_commands;
    tick += 1000; RobotControl_Tick();
    assert(servo_commands == manual && servo_channel == 'G' && servo_angle == 90);
    reset(); fail_servo = 1; feed("CMD,40,A\\n");
    tick = 1000; RobotControl_Tick(); assert(servo_commands == 1);
    reset(); feed("CMD,50,A\\n");
    tick = 1000; RobotControl_Tick(); tick = 2000; RobotControl_Tick();
    fail_servo = 1; tick = 2020; RobotControl_Tick();
    unsigned failed = servo_commands;
    tick = 5000; RobotControl_Tick(); assert(servo_commands == failed);
}
'''+s[b:]
p.write_text(s,encoding='utf-8')
p=root/'application/app/src/main/java/com/example/app/MainActivity.kt'
s=p.read_text(encoding='utf-8').replace('6 步，2→3、5→6 间隔 3 秒，其余 1 秒，无到位回包','6 步，第 3、6 步基座平滑旋转 3 秒，无到位回包')
p.write_text(s,encoding='utf-8')
p=root/'esp32s3/PROTOCOL.md'
s=p.read_text(encoding='utf-8').replace('STM32 内写死下列角度，2→3、5→6 命令间隔 3000 ms，其余间隔 1000 ms，不等待实际到位反馈：','STM32 内写死下列角度。第 3 步基座从 263°到 144°、第 6 步从 144°到 263°，\n均在 3000 ms 内按时间线性插值，每隔至少 20 ms 更新目标角度，而非等待 3 秒后跳转。\n转盘和夹子在进入步骤时设定。每步到达目标设定后保持 1000 ms，再进入下一步。\n没有舵机实测反馈，3 秒是目标角度渐变时长，实际运动取决于舵机和负载。')
s=s.replace('角度表和 `s_grab_hold_ms`','角度表、`ROBOT_BASE_ROTATE_MS` 和 `ROBOT_GRAB_HOLD_MS`')
p.write_text(s,encoding='utf-8')
