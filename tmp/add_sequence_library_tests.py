from pathlib import Path

p = Path('yundong_part/tests/distance_control_test.c')
s = p.read_text(encoding='utf-8')
index = s.index('int main(void)')
s = s[:index] + '''static void upload_test_plan(uint32_t seq)
{
    char text[128];
    snprintf(text, sizeof(text), "PLAN_BEGIN,%lu,2\\n", (unsigned long)seq); feed(text);
    snprintf(text, sizeof(text), "PLAN_ITEM,%lu,0,A,0,0,0,0,21,22,23,60,10,71,11,250,140\\n", (unsigned long)seq); feed(text);
    snprintf(text, sizeof(text), "PLAN_ITEM,%lu,1,P,50,-5,100,-10,27,28,29,60,20,81,12,260,150\\n", (unsigned long)seq); feed(text);
    snprintf(text, sizeof(text), "PLAN_RUN,%lu\\n", (unsigned long)seq); feed(text);
}
static void test_saved_plan_and_custom_origin(void)
{
    RobotOriginCommand origin;
    RobotPlanCommand plan;
    RobotSequenceCommand item;
    assert(RobotProtocol_ParseOrigin("ORIGIN,9,251", &origin) && origin.base == 251U);
    assert(!RobotProtocol_ParseOrigin("ORIGIN,9,361", &origin));
    assert(!RobotProtocol_ParseOrigin("ORIGIN,0,251", &origin));
    assert(RobotProtocol_ParsePlan("PLAN_BEGIN,9,16", &plan) && plan.count == 16U);
    assert(!RobotProtocol_ParsePlan("PLAN_BEGIN,9,17", &plan));
    assert(!RobotProtocol_ParsePlan("PLAN_BEGIN,9,0", &plan));
    assert(RobotProtocol_ParseSequence("PLAN_ITEM,9,15,A,0,0,0,0,20,20,50", &item) && item.plan_index == 15U);
    assert(!RobotProtocol_ParseSequence("PLAN_ITEM,9,16,A,0,0,0,0,20,20,50", &item));
    reset(); RobotControl_Init(&command_uart, true); servo_commands = 0;
    feed("ORIGIN,9,251\\n"); assert(servo_angle == 251);
    feed("CMD,10,O\\n"); finish_grab_axes(); assert(servo_angle == 251); // Home restores selected base.

    reset(); feed("PLAN_BEGIN,9,2\\nPLAN_ITEM,9,0,A,0,0,0,0,20,20,50\\nPLAN_RUN,9\\n");
    assert(absolute_commands[1] == 0); // No partial plan may run.
    feed("PLAN_ITEM,10,1,P,0,0,0,0,20,20,50\\nPLAN_RUN,9\\n"); assert(absolute_commands[1] == 0);
    reset(); upload_test_plan(20); assert(absolute_commands[1] == 1 && last_arm_angle.speed_rpm == 21);
    feed("PLAN_BEGIN,21,1\\nSTATE,22,P,0,0,0,0,40,40,40\\n"); // Busy upload cannot mutate active plan.
    arm_flags = lift_flags = 3;
    for (unsigned i = 0; i < 1000 && absolute_commands[1] < 6; ++i) { tick += 50; RobotControl_Tick(); }
    assert(absolute_commands[1] == 6 && last_arm_angle.speed_rpm == 27);
    assert(servo_angle == 260 && servo_history[servo_commands-2] == 81); // Second card starts only after first completes.
    for (unsigned i = 0; i < 1000; ++i) { tick += 50; RobotControl_Tick(); }
    unsigned moves = absolute_commands[1], servos = servo_commands;
    feed("PLAN_RUN,20\\n"); assert(absolute_commands[1] == moves && servo_commands == servos);
    feed("STATE,30,A,0,0,0,0,31,32,33\\n"); assert(absolute_commands[1] == moves+1 && last_arm_angle.speed_rpm == 31);

    reset(); upload_test_plan(40); feed("CMD,41,Z\\n");
    arm_flags = lift_flags = 3;
    for (unsigned i = 0; i < 1000; ++i) { tick += 50; RobotControl_Tick(); }
    assert(absolute_commands[1] == 1); // Cancel clears successors and uploaded frames.
    feed("PLAN_RUN,40\\n"); assert(absolute_commands[1] == 1);
    reset(); upload_test_plan(50); arm_flags = 0x0D; tick += 50; RobotControl_Tick();
    arm_flags = lift_flags = 3;
    for (unsigned i = 0; i < 1000; ++i) { tick += 50; RobotControl_Tick(); }
    assert(absolute_commands[1] == 1); // Motor fault must never advance to next card.
}
''' + s[index:]
s = s.replace('    test_app_origin_initialization();', '    test_app_origin_initialization();\n    test_saved_plan_and_custom_origin();')
p.write_text(s, encoding='utf-8')

p = Path('tmp/test_esp_servo_forwarding.py')
s = p.read_text(encoding='utf-8').replace('const char *commands[] = {', 'const char *commands[] = {"ORIGIN,1000,251", "PLAN_BEGIN,1001,2", "PLAN_ITEM,1001,0,A,1100,-50,200,-40,20,20,50,60,0,60,0,248,140", "PLAN_ITEM,1001,1,P,200,-40,1300,-110,20,20,50,60,0,60,0,248,140", "PLAN_RUN,1001", ')
s = s.replace('    puts("PASS: ESP32', '''    forwarded[0] = 0;
    assert(handle_frame("ORIGIN,1,361", &lift)); assert(forwarded[0] == 0);
    assert(handle_frame("PLAN_BEGIN,1,17", &lift)); assert(forwarded[0] == 0);
    assert(handle_frame("PLAN_ITEM,1,16,A,0,0,0,0,20,20,50", &lift)); assert(forwarded[0] == 0);
    puts("PASS: ESP32''')
p.write_text(s, encoding='utf-8')
