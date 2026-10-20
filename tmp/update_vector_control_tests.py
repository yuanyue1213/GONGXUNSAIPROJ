from pathlib import Path
p = Path('yundong_part/tests/distance_control_test.c')
s = p.read_text(encoding='utf-8')
s = s.replace('#include "../shared/camera_pose_math.h"', '#include "../shared/camera_pose_math.h"\n#include "../shared/arm_vector_math.h"')
s = s.replace("if (id == 1) { *status = lift_flags; return !fail_lift_status; }", "if (id == 1) { *status = lift_flags; if ((lift_flags & 3U) == 3U && !fail_lift_status && absolute_commands[0]) origin_position[0] = absolute_target[0]; return !fail_lift_status; }")
s = s.replace('assert(id == 2); *status = arm_flags; return !fail_arm_status;', 'assert(id == 2); *status = arm_flags; if ((arm_flags & 3U) == 3U && !fail_arm_status && absolute_commands[1]) origin_position[1] = absolute_target[1]; return !fail_arm_status;')
s = s.replace('static void advance_grab_hold(void) { tick += 1000;', 'static void advance_grab_hold(void) { tick += 500;')
# Replace obsolete assertions for the previous sequential automatic flow.
obsolete = ['test_grab_state','test_gripper_open_cancel','test_grab_status_reply_recovery',
            'test_sequence_angles','test_configured_sequences','test_release_sequence',
            'test_saved_plan_and_custom_origin','test_sequence_300_rpm']
for name in obsolete:
    a = s.index('static void '+name+'(void)')
    brace = s.index('{', a); depth = 1; b = brace+1
    while depth:
        if s[b] == '{': depth += 1
        elif s[b] == '}': depth -= 1
        b += 1
    s = s[:a]+s[b:]
    s = s.replace('    '+name+'();\n', '')
# Old automatic fixtures below only need to start/cancel, not compare independent speeds.
s = s.replace('finish_grab_axes(); assert(last_lift_angle.rpm == 300);\n    advance_grab_hold(); assert(last_lift_angle.rpm == 300);', 'finish_grab_axes();\n    advance_grab_hold();')
tests = r'''static void finish_vector_position(void)
{
    arm_flags = lift_flags = 3;
    for (unsigned i = 0; i < 4; ++i) { tick += 50; RobotControl_Tick(); }
    arm_flags = lift_flags = 1;
}
static void finish_base_rotation(void) { tick += 4000; RobotControl_Tick(); }
static void test_vector_protocol_and_speeds(void)
{
    RobotSequenceCommand cmd;
    assert(RobotProtocol_ParseSequence("STATE_V2,10,A,1100,-50,200,-40,600,60,0,60,0,248,140", &cmd));
    assert(cmd.combined_rpm == 600 && cmd.r1 == 1100);
    assert(RobotProtocol_ParseSequence("PLAN_ITEM_V2,10,15,P,200,-40,1300,-110,5,60,0,60,0,248,140", &cmd));
    assert(cmd.plan_index == 15);
    const char *bad[] = {"STATE_V2,0,A,0,0,0,0,20,60,0,60,0,248,140",
        "STATE_V2,10,A,0,0,0,0,601,60,0,60,0,248,140", "STATE_V2,10,A,0,0,0,0,4,60,0,60,0,248,140",
        "STATE_V2,10,A,0,0,0,0,20,60,0,60,0,248,140,junk", "STATE_V2,10,A,0,0,0,0,20,60,0,60,0,248",
        "PLAN_ITEM_V2,10,16,A,0,0,0,0,20,60,0,60,0,248,140", "STATE_V2,10,T,0,0,0,0,20,60,0,60,0,248,140"};
    for (unsigned i=0;i<sizeof(bad)/sizeof(bad[0]);++i) assert(!RobotProtocol_ParseSequence(bad[i],&cmd));
    assert(RobotProtocol_ParseSequence("STATE,10,A,1100,-50,200,-40,25,20,50",&cmd) && cmd.combined_rpm == 25);
    uint16_t r,z;
    assert(ArmVector_Speeds(300,-400,600,&r,&z) && r==360 && z==480);
    assert(ArmVector_Speeds(0,900,600,&r,&z) && r==0 && z==600);
    assert(ArmVector_Speeds(0,0,20,&r,&z) && r==0 && z==0);
    assert(ArmVector_Speeds(1,100000,5,&r,&z) && r==1 && z==5);
    assert(!ArmVector_Speeds(1,1,601,&r,&z));
    assert(ArmVector_DescentStage(0,9900)==9000);
    assert(ArmVector_DescentStage(0,450)==0);
    assert(ArmVector_DescentStage(900,0)==0);
}
static void test_vector_grab(void)
{
    reset(); feed("STATE_V2,2200,A,1100,-50,200,-40,600,60,0,60,0,248,140\n");
    assert(absolute_target[1]==312 && last_arm_angle.speed_rpm==600 && absolute_commands[0]==0);
    finish_vector_position(); advance_grab_hold(); // Step 2: both axes already started.
    assert(absolute_target[1]==3118 && absolute_target[0]==3600);
    assert(last_arm_angle.speed_rpm>300 && last_lift_angle.rpm>300 && last_lift_angle.rpm<600);
    unsigned rc=absolute_commands[1], zc=absolute_commands[0];
    arm_flags=3; tick+=50; RobotControl_Tick(); assert(absolute_commands[0]==zc); // R early does not start Z tail.
    lift_flags=3; tick+=50; RobotControl_Tick();
    assert(absolute_commands[1]==rc && absolute_target[0]==4500 && last_lift_angle.rpm==300);
    lift_flags=1; tick+=50; RobotControl_Tick(); assert(servo_angle==60);
    finish_vector_position(); advance_grab_hold(); assert(servo_channel=='G' && servo_angle==0); // Step 3.
    rc=absolute_commands[1]; tick+=499; RobotControl_Tick(); assert(absolute_commands[1]==rc);
    tick+=1; RobotControl_Tick(); // Step 4: upward pair starts together.
    assert(absolute_target[1]==312 && absolute_target[0]==0 && absolute_commands[1]==rc+1);
    finish_vector_position(); advance_grab_hold(); finish_base_rotation(); assert(servo_angle==140); // Step 5.
    advance_grab_hold(); assert(absolute_target[1]==567 && absolute_target[0]==2700); // Step 6 staged descent.
    finish_vector_position(); assert(absolute_target[0]==3600 && last_lift_angle.rpm==300);
    advance_grab_hold(); rc=absolute_commands[1]; zc=absolute_commands[0]; // Step 7 opens before returning.
    tick+=500; RobotControl_Tick(); assert(servo_angle==30 && absolute_commands[1]==rc && absolute_commands[0]==zc);
    tick+=500; RobotControl_Tick(); assert(absolute_target[1]==312 && absolute_target[0]==0);
    finish_vector_position(); advance_grab_hold(); assert(servo_angle==140); // Step 8 retains storage angle.
    unsigned servos=servo_commands; advance_grab_hold(); assert(servo_commands==servos);
    feed("STATE_V2,2200,A,1100,-50,200,-40,600,60,0,60,0,248,140\n"); assert(servo_commands==servos); // Duplicate.
    feed("BASE,2201,150,30\n"); tick+=1000; RobotControl_Tick(); assert(servo_angle==150); // Completed/idle.
}
static void test_vector_release(void)
{
    reset(); feed("STATE_V2,2300,P,200,-40,1300,-110,200,60,0,60,0,248,140\n");
    finish_vector_position(); advance_grab_hold(); finish_base_rotation(); assert(servo_angle==140);
    advance_grab_hold(); // Step 3: keep open until both axes and the descent tail arrive.
    assert(absolute_target[1]==567 && absolute_target[0]==2700 && servo_angle==60);
    arm_flags=lift_flags=3; tick+=50; RobotControl_Tick(); assert(absolute_target[0]==3600 && servo_angle==60);
    lift_flags=1; tick+=50; RobotControl_Tick(); assert(servo_angle==60);
    finish_vector_position(); assert(servo_angle==0);
    advance_grab_hold(); assert(absolute_target[1]==312 && absolute_target[0]==0); // Step 4 parallel up.
    finish_vector_position(); advance_grab_hold(); finish_base_rotation(); assert(servo_angle==248);
    advance_grab_hold(); assert(absolute_target[1]==3685 && absolute_target[0]==9000); // Step 6.
    finish_vector_position(); assert(absolute_target[0]==9900 && last_lift_angle.rpm==300);
    advance_grab_hold(); tick+=1000; RobotControl_Tick(); assert(servo_angle==60); // Step 7 open.
    advance_grab_hold(); assert(absolute_target[1]==312 && absolute_target[0]==0); // Step 8 return pair.
    finish_vector_position(); advance_grab_hold();
    feed("BASE,2301,250,30\n"); tick+=1000; RobotControl_Tick(); assert(servo_angle==250);
}
static void test_vector_edge_cases_and_faults(void)
{
    // User example from 0,0 to 110,-110: first reach 110,-100, then descend 10 mm.
    reset(); feed("STATE_V2,2400,P,1100,-110,0,0,600,60,0,60,0,248,140\n");
    finish_vector_position(); advance_grab_hold(); finish_base_rotation(); advance_grab_hold();
    assert(absolute_target[1]==3118 && absolute_target[0]==9000);
    finish_vector_position(); assert(absolute_target[0]==9900 && last_lift_angle.rpm==300);
    // <10 mm descent: no upward detour or first Z command; R must arrive first.
    reset(); feed("STATE_V2,2401,A,1100,-5,200,-40,600,60,0,60,0,248,140\n");
    finish_vector_position(); advance_grab_hold(); assert(absolute_commands[0]==0);
    arm_flags=3; tick+=50; RobotControl_Tick(); assert(absolute_target[0]==450 && last_lift_angle.rpm==300);
    feed("CMD,2402,Z\n"); unsigned zc=absolute_commands[0]; tick+=1000; RobotControl_Tick(); assert(absolute_commands[0]==zc && lift_stops>0);
    // Second-axis command failure stops the running first axis and clears plan ownership.
    reset(); feed("CMD,2403,A\n"); finish_vector_position(); fail_lift_angle=1; advance_grab_hold();
    assert(arm_stops>0); fail_lift_angle=0;
    unsigned rc=absolute_commands[1]; tick+=1000; RobotControl_Tick(); assert(absolute_commands[1]==rc);
    // Failed read prevents any new stage.
    reset(); fail_origin_read=1; feed("CMD,2404,A\n"); assert(absolute_commands[1]==0 && absolute_commands[0]==0);
    // Loss of one axis' status stops its peer, with no reserved tail started.
    reset(); feed("CMD,2405,A\n"); finish_vector_position(); advance_grab_hold();
    fail_arm_status=1; zc=absolute_commands[0]; tick+=500; RobotControl_Tick();
    assert(arm_stops>0 && lift_stops>0 && absolute_commands[0]==zc);
    // No-op positions still advance, and all inter-card waiting is 500 ms.
    reset(); feed("PLAN_BEGIN,2500,2\nPLAN_ITEM_V2,2500,0,A,110,0,110,0,20,60,0,60,0,248,140\nPLAN_ITEM,2500,1,T,0,120,60\nPLAN_RUN,2500\n");
    arm_flags=lift_flags=3;
    for (unsigned i=0;i<200;++i) { tick+=50; RobotControl_Tick(); }
    assert(servo_channel=='T' && servo_angle==120);
    unsigned servos=servo_commands; feed("PLAN_RUN,2500\n"); assert(servo_commands==servos);
}
'''
s = s.replace('int main(void)', tests+'\nint main(void)')
s = s.replace('    test_parser();', '    test_vector_protocol_and_speeds(); test_vector_grab(); test_vector_release(); test_vector_edge_cases_and_faults();\n    test_parser();')
p.write_text(s, encoding='utf-8')

p = Path('yundong_part/tests/arm_angle_driver_test.c')
s=p.read_text(encoding='utf-8').replace('312, 301, 3200) == HAL_ERROR', '312, 601, 3200) == HAL_ERROR').replace('900, 301, 3200) == HAL_ERROR', '900, 601, 3200) == HAL_ERROR').replace('1, 0, 301, 3200) == HAL_ERROR', '1, 0, 601, 3200) == HAL_ERROR')
s=s.replace('    puts("PASS:', '''    reset(0x21);
    assert(ArmMotor_MoveAbsolute(2, 312, 600, 3200) == HAL_OK);
    assert(position[3] == 2 && position[4] == 0x58);
    assert(ArmMotor_MoveAbsolute(1, 0, 1, 3200) == HAL_OK && position[4] == 1);
    reset(0x25);
    assert(ArmMotor_MoveAbsolute(1, 900, 600, 3200) == HAL_OK);
    assert(position[7] == 0x17 && position[8] == 0x70);
    puts("PASS:''')
p.write_text(s,encoding='utf-8')
