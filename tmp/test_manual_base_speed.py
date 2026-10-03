from pathlib import Path

p = Path('yundong_part/tests/distance_control_test.c')
s = p.read_text(encoding='utf-8')
a = s.index('int main(void)')
s = s[:a] + '''static void test_manual_base_speed(void)
{
    RobotBaseCommand cmd;
    assert(RobotProtocol_ParseBase("BASE,1,360,1", &cmd) && cmd.target == 360U && cmd.dps == 1U);
    assert(RobotProtocol_ParseBase("BASE,2,0,360", &cmd));
    const char *invalid[] = {"BASE,0,90,60", "BASE,1,-1,60", "BASE,1,361,60", "BASE,1,90,0",
        "BASE,1,90,361", "BASE,1,90,60,", "BASE,1,90,60x"};
    for (unsigned i = 0; i < sizeof(invalid)/sizeof(invalid[0]); ++i) assert(!RobotProtocol_ParseBase(invalid[i], &cmd));
    reset(); feed("BASE,100,308,30\\n"); assert(servo_commands == 0);
    feed("STATE,101,A,0,0,0,0,20,20,50\\nCMD,102,O\\n"); assert(absolute_commands[1] == 0);
    tick += 1000; RobotControl_Tick(); assert(servo_channel == 'B' && servo_angle == 278);
    feed("BASE,100,308,30\\n"); // Duplicate must not restart ramp.
    tick += 1000; RobotControl_Tick(); assert(servo_angle == 308);
    feed("BASE,103,248,60\\n"); tick += 500; RobotControl_Tick(); assert(servo_angle == 278);
    feed("BASE,104,218,60\\n"); tick += 500; RobotControl_Tick(); assert(servo_angle == 248);
    tick += 500; RobotControl_Tick(); assert(servo_angle == 218);
    unsigned servos = servo_commands; feed("BASE,105,218,60\\n"); RobotControl_Tick(); assert(servo_commands == servos);
    feed("BASE,106,360,1\\n"); tick += 10000; RobotControl_Tick(); assert(servo_angle == 228);
    feed("CMD,107,Z\\n"); servos = servo_commands; tick += 200000; RobotControl_Tick(); assert(servo_commands == servos);
    feed("SERVO,108,B,100\\nBASE,109,160,30\\n"); tick += 1000; RobotControl_Tick(); assert(servo_angle == 130);
    feed("SERVO,110,B,80\\n"); tick += 10000; RobotControl_Tick(); assert(servo_angle == 80);
    feed("BASE,111,140,30\\n"); fail_servo = 1; tick += 1000; RobotControl_Tick();
    fail_servo = 0; servos = servo_commands; tick += 10000; RobotControl_Tick(); assert(servo_commands == servos);
    reset(); RobotControl_Init(&command_uart, true); servo_commands = 0; // Unknown start: first target is direct.
    feed("BASE,112,100,30\\n"); assert(servo_angle == 100 && servo_commands == 1);
    feed("BASE,113,160,30\\n"); tick += 1000; RobotControl_Tick(); assert(servo_angle == 130);
    reset(); feed("CMD,114,A\\nBASE,115,200,30\\n"); assert(arm_stops == 1);
    tick += 1000; RobotControl_Tick(); assert(servo_angle == 218); // Automatic base target tracked at 248.
}
''' + s[a:]
s = s.replace('    test_saved_plan_and_custom_origin();', '    test_saved_plan_and_custom_origin();\n    test_manual_base_speed();')
p.write_text(s, encoding='utf-8')

p = Path('tmp/test_esp_servo_forwarding.py')
s = p.read_text(encoding='utf-8').replace('const char *commands[] = {', 'const char *commands[] = {"BASE,1002,360,30", "BASE,1003,0,1", ')
s = s.replace('    puts("PASS: ESP32', '''    forwarded[0] = 0;
    assert(handle_frame("BASE,1,-1,60", &lift)); assert(forwarded[0] == 0);
    assert(handle_frame("BASE,1,361,60", &lift)); assert(forwarded[0] == 0);
    assert(handle_frame("BASE,1,90,0", &lift)); assert(forwarded[0] == 0);
    puts("PASS: ESP32''')
p.write_text(s, encoding='utf-8')
