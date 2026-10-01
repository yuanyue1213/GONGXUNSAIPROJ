from pathlib import Path

root = Path('F:/GONGXUNSAIPROJ')
path = root / 'yundong_part/stm32h743/zhukong/Core/Src/robot_control.c'
text = path.read_text(encoding='utf-8')
start = text.index('/* 抓取固定姿态：')
end = text.index('/* 摄像头持续输出坐标。', start)
text = text[:start] + '''/* Eight grab states. r/z are absolute offsets from the startup origin.
 * A step is held for one second only after both positioning axes reach it. */
typedef struct {
    uint16_t gripper, base;
    int32_t r_mm, z_mm;
    bool position;
} GrabStep;
static const GrabStep s_grab_steps[] = {
    {60U, 263U,   0,   0, true},
    {60U, 263U, 110, -50, true},
    { 0U, 263U,   0,   0, false},
    { 0U, 263U,   0,   0, true},
    { 0U, 144U,   0,   0, false},
    { 0U, 144U,  20, -40, true},
    {60U, 144U,   0,   0, true},
    {60U, 263U,   0,   0, false},
};
#define ROBOT_GRAB_ARM_RPM        10U
#define ROBOT_GRAB_ARM_PULSES     3200U
#define ROBOT_GRAB_HOLD_MS       1000U
#define ROBOT_BASE_ROTATE_MS     3000U
#define ROBOT_BASE_UPDATE_MS       20U
static bool s_grab_active;
static bool s_grab_arm_wait, s_grab_lift_wait;
static bool s_grab_base_rotating;
static uint8_t s_grab_step;
static uint32_t s_grab_tick;
static uint32_t s_base_update_tick;
static uint32_t s_last_grab_sequence;
/* Completed rotation target only advances after the final step succeeds. */
static uint16_t s_grab_turntable;

static void RobotControl_CancelGrab(void)
{
    s_grab_active = false;
    if (s_grab_arm_wait && s_arm_move_active) RobotControl_StopArmMove();
    if (s_grab_lift_wait && s_lift_angle_active) RobotControl_StopLiftAngle();
    s_grab_arm_wait = s_grab_lift_wait = false;
    s_grab_base_rotating = false;
}

static bool RobotControl_ApplyGrabStep(void)
{
    const GrabStep *step = &s_grab_steps[s_grab_step];
    /* Preserve the previous 3-second base ramps on the new steps 5 and 8. */
    s_grab_base_rotating = s_grab_step == 4U || s_grab_step == 7U;
    uint16_t base = s_grab_base_rotating ? s_grab_steps[s_grab_step-1U].base : step->base;
    s_grab_tick = s_base_update_tick = HAL_GetTick();
    if (!ServoControl_SetAngle('T', 0U) || !ServoControl_SetAngle('G', step->gripper) ||
        !ServoControl_SetAngle('B', base)) return false;
    if (step->position) {
        s_grab_arm_wait = RobotControl_StartArmAbsolute(step->r_mm, ROBOT_GRAB_ARM_RPM, ROBOT_GRAB_ARM_PULSES);
        return s_grab_arm_wait;
    }
    return true;
}

static void RobotControl_TickGrab(void)
{
    if (!s_grab_active) return;
    if (s_grab_arm_wait) {
        if (s_arm_move_stop_pending) { RobotControl_CancelGrab(); return; }
        if (s_arm_move_active) return;
        s_grab_arm_wait = false;
        if (!s_arm_move_reached) { RobotControl_CancelGrab(); return; }
        s_grab_lift_wait = RobotControl_StartLiftAbsolute(s_grab_steps[s_grab_step].z_mm,
            ROBOT_GRAB_ARM_RPM, ROBOT_GRAB_ARM_PULSES);
        if (!s_grab_lift_wait) RobotControl_CancelGrab();
        return;
    }
    if (s_grab_lift_wait) {
        if (s_lift_angle_stop_pending) { RobotControl_CancelGrab(); return; }
        if (s_lift_angle_active) return;
        s_grab_lift_wait = false;
        if (!s_lift_angle_reached) { RobotControl_CancelGrab(); return; }
        s_grab_tick = HAL_GetTick();
        return;
    }
    uint32_t now = HAL_GetTick(), elapsed = (uint32_t)(now - s_grab_tick);
    if (s_grab_base_rotating) {
        if (elapsed < ROBOT_BASE_ROTATE_MS && (uint32_t)(now - s_base_update_tick) < ROBOT_BASE_UPDATE_MS) return;
        uint32_t progress = elapsed < ROBOT_BASE_ROTATE_MS ? elapsed : ROBOT_BASE_ROTATE_MS;
        int32_t start = s_grab_steps[s_grab_step-1U].base;
        int32_t delta = (int32_t)s_grab_steps[s_grab_step].base - start;
        int32_t numerator = delta * (int32_t)progress;
        numerator += delta < 0 ? -(int32_t)(ROBOT_BASE_ROTATE_MS/2U) : (int32_t)(ROBOT_BASE_ROTATE_MS/2U);
        if (!ServoControl_SetAngle('B', (uint16_t)(start + numerator/(int32_t)ROBOT_BASE_ROTATE_MS)))
        { RobotControl_CancelGrab(); return; }
        s_base_update_tick = now;
        if (elapsed >= ROBOT_BASE_ROTATE_MS) { s_grab_base_rotating = false; s_grab_tick = now; }
        return;
    }
    if (elapsed < ROBOT_GRAB_HOLD_MS) return;
    if (++s_grab_step >= sizeof(s_grab_steps)/sizeof(s_grab_steps[0])) {
        uint16_t target = s_grab_turntable + 120U;
        if (target <= 270U && ServoControl_SetAngle('T', target)) s_grab_turntable = target;
        RobotControl_CancelGrab();
    } else if (!RobotControl_ApplyGrabStep()) RobotControl_CancelGrab();
}

''' + text[end:]
text = text.replace('s_grab_arm_wait = false;\n    s_grab_base_rotating',
                    's_grab_arm_wait = s_grab_lift_wait = false;\n    s_grab_turntable = 0U;\n    s_grab_base_rotating')
text = text.replace("if (direction == 'U' || direction == 'D' || direction == 'H') {",
                    "if (direction == 'U' || direction == 'D' || direction == 'H') {\n        if (s_grab_active) {\n            if (direction == 'H') RobotControl_CancelGrab();\n            return;\n        }")
text = text.replace('if (s_grab_arm_wait) { RobotControl_CancelGrab(); return; }',
                    'if (s_grab_active) { RobotControl_CancelGrab(); return; }')
path.write_text(text, encoding='utf-8')
