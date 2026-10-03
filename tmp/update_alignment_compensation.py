from pathlib import Path
p=Path('yundong_part/tests/distance_control_test.c');s=p.read_text(encoding='utf-8')
a='''        camera_sample("RINGS,256,160,256,160,256,160\\n");
        feed("MOVE,2,F,100,45,9889\\n"); assert(position_frames == 2);'''
b='''        camera_sample("RINGS,256,160,256,160,256,160\\n");
        assert(position_frames == 2 && wheel_pulses() == 150U && frame[6] == 1);
        feed("MOVE,2,F,100,45,9889\\n"); assert(position_frames == 2); /* Compensation owns wheels. */
        finish_alignment_move();
        camera_sample("RINGS,256,160,256,160,256,160\\n"); assert(position_frames == 2);
        feed("MOVE,2,F,100,45,9889\\n"); assert(position_frames == 3);'''
assert a in s;s=s.replace(a,b)
a='''    camera_sample("x=256,y=160\\n");
    feed("MOVE,4,F,100,45,9889\\n"); assert(position_frames == 3);'''
b='''    camera_sample("x=256,y=160\\n");
    assert(position_frames == 3 && wheel_pulses() == 150U && frame[6] == 1 && frame[8] == 10);
    feed("MOVE,4,F,100,45,9889\\n"); assert(position_frames == 3);
    finish_alignment_move();
    camera_sample("x=256,y=160\\n"); assert(position_frames == 3); /* Exactly once; no re-centering. */
    feed("MOVE,4,F,100,45,9889\\n"); assert(position_frames == 4);'''
assert a in s;s=s.replace(a,b)
a='static void test_camera_alignment(void)'
b='''static void test_alignment_compensation_stop(void)
{
    for (unsigned mode = 0; mode < 2; ++mode) {
        const char *command = mode ? "ALIGN_RING,1,2,10000,20000\\n" : "ALIGN,1,10000,20000\\n";
        const char *sample = mode ? "RINGS,100,160,256,160,400,160\\n" : "x=256,y=160\\n";
        reset(); camera_ready(); feed(command);
        camera_sample(sample); assert(position_frames == 0);
        camera_sample(sample); assert(position_frames == 1 && wheel_pulses() == 150U);
        camera_sample(sample); assert(position_frames == 1);
        feed("CMD,2,S\\n"); assert(stop_frames == 4);
        camera_sample(sample); assert(position_frames == 1);
        reset(); camera_ready(); feed(command);
        camera_sample(sample); camera_sample(sample);
        fail_status = 1; tick += 60; RobotControl_Tick(); assert(stop_frames == 4);
        camera_sample(sample); assert(position_frames == 1);
    }
}
'''+a
s=s.replace(a,b).replace('    test_ring_alignment();','    test_ring_alignment();\n    test_alignment_compensation_stop();');p.write_text(s,encoding='utf-8')
for path in ['output/目前已有状态与扩展说明.md','yundong_part/esp32s3/PROTOCOL.md','yundong_part/shared/CAMERA_ALIGNMENT_PROTOCOL.md']:
 p=Path(path);s=p.read_text(encoding='utf-8');s+='\n两种位置修正成功对准中心后，均沿图像 x 正方向（现有 B 指令方向）额外移动 15 mm，速度 10 RPM，使用前后距离标定。补偿仅执行一次，等待四轮到位后结束，不再重新居中；停止或失败时不追加补偿，补偿途中也可停止。\n';p.write_text(s,encoding='utf-8')
