from pathlib import Path
p = Path('yundong_part/tests/distance_control_test.c')
s = p.read_text(encoding='utf-8')
a = s.index('static void test_manual_base_speed(void)')
b = s.index('static void test_parallel_pixel_tolerance(void)', a)
section = s[a:b]
section = section.replace("servo_angle == 278", "servo_angle == 274", 1)
section = section.replace('tick += 1000; RobotControl_Tick(); assert(servo_angle == 308);',
    'tick += 1000; RobotControl_Tick(); assert(servo_angle == 304);\n    tick += 250; RobotControl_Tick(); assert(servo_angle == 308);')
section = section.replace('servo_angle == 278', 'servo_angle == 293')
section = section.replace('servo_angle == 248', 'servo_angle == 278')
section = section.replace('tick += 500; RobotControl_Tick(); assert(servo_angle == 218);',
    'tick += 1250; RobotControl_Tick(); assert(servo_angle == 218);')
section = section.replace('servo_angle == 130', 'servo_angle == 126')
section = section.replace('servo_angle == 218); // Automatic', 'servo_angle == 222); // Automatic')
s = s[:a] + section + s[b:]
test = '''static void test_base_acceleration(void)
{
    // Short move never reaches the requested speed: symmetric triangular ramp.
    reset(); feed("SERVO,2100,B,100\\nBASE,2101,160,360\\n");
    const uint16_t expected[] = {102,110,122,137,150,157,160};
    uint16_t previous = 100;
    for (unsigned i = 0; i < 7U; ++i) {
        tick += i == 6U ? 215 : 200; RobotControl_Tick();
        assert(servo_channel == 'B' && servo_angle == expected[i] && servo_angle >= previous);
        previous = servo_angle;
    }
    unsigned commands = servo_commands; tick += 5000; RobotControl_Tick(); assert(servo_commands == commands);
    // Same acceleration on reverse travel and both endpoint directions.
    feed("BASE,2102,100,360\\n"); tick += 200; RobotControl_Tick(); assert(servo_angle == 158);
    tick += 1215; RobotControl_Tick(); assert(servo_angle == 100);
    // Tick rollover still follows elapsed time, with no endpoint overshoot.
    tick = UINT32_MAX - 100U; feed("BASE,2103,0,60\\n");
    tick += 200; RobotControl_Tick(); assert(servo_angle == 98);
    tick += 3000; RobotControl_Tick(); assert(servo_angle == 0);
    feed("BASE,2104,360,360\\n"); tick += 4000; RobotControl_Tick(); assert(servo_angle == 360);
    // Automatic 108-degree move accelerates, cruises, then decelerates in 2 s.
    reset(); feed("CMD,2105,P\\n"); finish_grab_axes(); advance_grab_hold();
    tick += 200; RobotControl_Tick(); assert(servo_angle == 246);
    tick += 800; RobotControl_Tick(); assert(servo_angle == 194);
    tick += 800; RobotControl_Tick(); assert(servo_angle == 142);
    tick += 200; RobotControl_Tick(); assert(servo_angle == 140);
}
'''
s = s.replace('int main(void)', test+'int main(void)').replace('    test_manual_base_speed();', '    test_manual_base_speed();\n    test_base_acceleration();')
p.write_text(s, encoding='utf-8')
