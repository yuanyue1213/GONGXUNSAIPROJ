from pathlib import Path

h = Path('yundong_part/shared/robot_distance_protocol.h')
s = h.read_text(encoding='utf-8')
s = s.replace('    char mode;\n    int32_t r1', '    uint32_t plan_index;\n    char mode;\n    int32_t r1')
s = s.replace('    if (strncmp(frame, "STATE,", 6U) != 0) return false;\n    const char *p = frame + 6U;\n    if (!RobotProtocol_U32(&p, &cmd->sequence, \',\')) return false;', '''    bool item = strncmp(frame, "PLAN_ITEM,", 10U) == 0;
    if (!item && strncmp(frame, "STATE,", 6U) != 0) return false;
    const char *p = frame + (item ? 10U : 6U);
    cmd->plan_index = 0U;
    if (!RobotProtocol_U32(&p, &cmd->sequence, ',')) return false;
    if (item && (!RobotProtocol_U32(&p, &cmd->plan_index, ',') || cmd->plan_index >= 16U)) return false;''')
s = s.replace('\n#endif', '''
#define ROBOT_PLAN_MAX_ITEMS 16U
typedef struct { uint32_t sequence, count; bool run; } RobotPlanCommand;
static inline bool RobotProtocol_ParsePlan(const char *frame, RobotPlanCommand *cmd)
{
    cmd->run = strncmp(frame, "PLAN_RUN,", 9U) == 0;
    if (!cmd->run && strncmp(frame, "PLAN_BEGIN,", 11U) != 0) return false;
    const char *p = frame + (cmd->run ? 9U : 11U);
    cmd->count = 0U;
    if (!RobotProtocol_U32(&p, &cmd->sequence, cmd->run ? '\\0' : ',')) return false;
    if (!cmd->run && (!RobotProtocol_U32(&p, &cmd->count, '\\0') ||
        cmd->count == 0U || cmd->count > ROBOT_PLAN_MAX_ITEMS)) return false;
    return cmd->sequence != 0U;
}
typedef struct { uint32_t sequence, base; } RobotOriginCommand;
static inline bool RobotProtocol_ParseOrigin(const char *frame, RobotOriginCommand *cmd)
{
    if (strncmp(frame, "ORIGIN,", 7U) != 0) return false;
    const char *p = frame + 7U;
    return RobotProtocol_U32(&p, &cmd->sequence, ',') &&
        RobotProtocol_U32(&p, &cmd->base, '\\0') && cmd->sequence != 0U && cmd->base <= 360U;
}
#endif''')
h.write_text(s, encoding='utf-8')

p = Path('yundong_part/esp32s3/hello_world/main/robot_remote_main.c')
s = p.read_text(encoding='utf-8')
s = s.replace('    if (strncmp(frame, "ARM_POSE,", 9U) == 0) {', '''    if (strncmp(frame, "ORIGIN,", 7U) == 0) {
        RobotOriginCommand command;
        if (!RobotProtocol_ParseOrigin(frame, &command)) return true;
    } else if (strncmp(frame, "PLAN_BEGIN,", 11U) == 0 || strncmp(frame, "PLAN_RUN,", 9U) == 0) {
        RobotPlanCommand command;
        if (!RobotProtocol_ParsePlan(frame, &command)) return true;
    } else if (strncmp(frame, "ARM_POSE,", 9U) == 0) {''', 1)
s = s.replace('} else if (strncmp(frame, "STATE,", 6U) == 0) {', '} else if (strncmp(frame, "STATE,", 6U) == 0 || strncmp(frame, "PLAN_ITEM,", 10U) == 0) {')
p.write_text(s, encoding='utf-8')

p = Path('yundong_part/stm32h743/zhukong/Core/Src/robot_control.c')
s = p.read_text(encoding='utf-8').replace('一行最多 63 字节', '一行最多 127 字节')
s = s.replace('static uint32_t s_last_origin_sequence;', 'static uint32_t s_last_origin_sequence;\nstatic uint16_t s_origin_base = 248U;')
s = s.replace("(void)ServoControl_SetAngle('B', 248U);", "(void)ServoControl_SetAngle('B', s_origin_base);")
s = s.replace('static void RobotControl_CancelGrab(void);', '''/* App uploads an entire plan before RUN. No reply or estimated sleep is needed:
 * only a successfully completed card may start its successor. */
static RobotSequenceCommand s_plan_items[ROBOT_PLAN_MAX_ITEMS];
static uint32_t s_plan_sequence, s_last_plan_sequence, s_plan_mask;
static uint8_t s_plan_count, s_plan_index;
static bool s_plan_active;
static bool RobotControl_StartSequence(const RobotSequenceCommand *settings, char direction);
static void RobotControl_CancelGrab(void);''', 1)
s = s.replace('static void RobotControl_CancelGrab(void)\n{\n    s_grab_active = false;', '''static void RobotControl_CancelGrab(void)
{
    s_plan_active = false; s_plan_count = 0U; s_plan_mask = 0U;
    s_grab_active = false;''')
s = s.replace('''            if (ServoControl_SetAngle('T', target)) s_grab_turntable = target;
        }
        RobotControl_CancelGrab();''', '''            if (!ServoControl_SetAngle('T', target)) { RobotControl_CancelGrab(); return; }
            s_grab_turntable = target;
        }
        s_grab_active = false;
        if (s_plan_active && ++s_plan_index < s_plan_count) {
            const RobotSequenceCommand *next = &s_plan_items[s_plan_index];
            if (!RobotControl_StartSequence(next, next->mode)) RobotControl_CancelGrab();
        } else {
            s_plan_active = false; s_plan_count = 0U; s_plan_mask = 0U;
        }''')
# Share the exact existing state configuration and motor state machine with plans.
a = s.index("        s_release_mode = direction == 'P';", s.index('static void RobotControl_HandleFrame(const char *frame)\n{'))
b = s.index('        return;\n    }\n    if (direction == \'U\'', a)
body = s[a:b].replace('if (configured)', 'if (settings != NULL)').replace('settings.', 'settings->')
start_function = '''static bool RobotControl_SequenceIdle(void)
{
    return !s_manual_gripper_active && !s_pose_active && !s_grab_active && !s_align_active &&
        s_arm_origin_valid && s_lift_origin_valid && !s_origin_initializing &&
        s_motion == 'S' && !s_stop_pending && s_lift_motion == 'H' && s_lift_command_ok &&
        !s_arm_move_active && !s_lift_angle_active && !s_arm_move_stop_pending && !s_lift_angle_stop_pending &&
        s_fore_aft_motion == 'Q' && s_fore_aft_command_ok;
}
static bool RobotControl_StartSequence(const RobotSequenceCommand *settings, char direction)
{
    if (!RobotControl_SequenceIdle()) return false;
BODY
    if (!s_grab_active) RobotControl_CancelGrab();
    return s_grab_active;
}

'''.replace('BODY', '\n'.join(line[4:] if line.startswith('    ') else line for line in body.splitlines()))
s = s[:a] + '        (void)RobotControl_StartSequence(configured ? &settings : NULL, direction);\n' + s[b:]
insert = s.index('/* 单向命令：格式错误')
s = s[:insert] + start_function + s[insert:]
s = s.replace("if (s_manual_gripper_active || s_pose_active || !s_arm_origin_valid || !s_lift_origin_valid || s_lift_motion != 'H' || s_motion != 'S') return;", 'if (s_plan_active || !RobotControl_SequenceIdle()) return;')
s = s.replace('    if (RobotControl_Parse(frame, &sequence, &direction) && direction == \'I\') {', '''    if (strncmp(frame, "PLAN_BEGIN,", 11U) == 0 || strncmp(frame, "PLAN_RUN,", 9U) == 0) {
        RobotPlanCommand cmd;
        if (!RobotProtocol_ParsePlan(frame, &cmd) || s_plan_active || !RobotControl_SequenceIdle()) return;
        if (!cmd.run) {
            if (cmd.sequence == s_last_plan_sequence) return;
            s_plan_sequence = cmd.sequence; s_plan_count = (uint8_t)cmd.count; s_plan_mask = 0U;
        } else {
            if (cmd.sequence != s_plan_sequence || cmd.sequence == s_last_plan_sequence || s_plan_count == 0U ||
                s_plan_mask != ((1UL << s_plan_count) - 1U)) return;
            s_last_plan_sequence = cmd.sequence; s_plan_index = 0U; s_plan_active = true;
            if (!RobotControl_StartSequence(&s_plan_items[0], s_plan_items[0].mode)) RobotControl_CancelGrab();
        }
        return;
    }
    if (strncmp(frame, "PLAN_ITEM,", 10U) == 0) {
        RobotSequenceCommand item;
        if (!RobotProtocol_ParseSequence(frame, &item) || s_plan_active || !RobotControl_SequenceIdle() ||
            item.sequence != s_plan_sequence || item.plan_index >= s_plan_count) return;
        s_plan_items[item.plan_index] = item; s_plan_mask |= 1UL << item.plan_index;
        return;
    }
    RobotOriginCommand origin = {.base = 248U};
    bool origin_command = strncmp(frame, "ORIGIN,", 7U) == 0;
    if (origin_command && !RobotProtocol_ParseOrigin(frame, &origin)) return;
    if (origin_command) sequence = origin.sequence;
    if (origin_command || (RobotControl_Parse(frame, &sequence, &direction) && direction == 'I')) {''')
s = s.replace("!ServoControl_SetAngle('B', 248U)) return;\n        s_grab_turntable", "!ServoControl_SetAngle('B', (uint16_t)origin.base)) return;\n        s_origin_base = (uint16_t)origin.base;\n        s_grab_turntable")
s = s.replace('    s_origin_initializing = false; s_last_origin_sequence = 0U;', '''    s_origin_initializing = false; s_last_origin_sequence = 0U; s_origin_base = 248U;
    s_plan_active = false; s_plan_count = 0U; s_plan_mask = 0U;
    s_plan_sequence = s_last_plan_sequence = 0U;''')
p.write_text(s, encoding='utf-8')
