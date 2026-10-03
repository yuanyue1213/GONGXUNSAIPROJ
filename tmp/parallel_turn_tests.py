from pathlib import Path
p=Path('yundong_part/tests/distance_control_test.c');s=p.read_text(encoding='utf-8').replace('const char directions[] = "FBLR";','const char directions[] = "FBLRCW";').replace('const uint8_t expected[4][4] = {{0,1,1,0}, {1,0,0,1}, {1,1,0,0}, {0,0,1,1}};','const uint8_t expected[6][4] = {{0,1,1,0}, {1,0,0,1}, {1,1,0,0}, {0,0,1,1}, {1,1,1,1}, {0,0,0,0}};').replace('for (unsigned d = 0; d < 4; ++d)','for (unsigned d = 0; d < 6; ++d)')
mark='static void test_absolute_pose(void)';test='''static void test_parallel_alignment(void)
{
    CameraParallelCommand cmd; int32_t slope;
    const CameraCenter up[] = {{100,100},{200,150},{300,200}};
    const CameraCenter down[] = {{100,200},{200,150},{300,100}};
    const CameraCenter horizontal[] = {{100,150},{200,150},{300,150}};
    const CameraCenter outlier[] = {{100,100},{200,220},{300,200}};
    const CameraCenter narrow[] = {{100,100},{120,110},{140,120}};
    assert(CameraProtocol_RingSlope(up, &slope) && slope == 500);
    assert(CameraProtocol_RingSlope(down, &slope) && slope == -500);
    assert(CameraProtocol_RingSlope(horizontal, &slope) && slope == 0);
    assert(!CameraProtocol_RingSlope(outlier, &slope));
    assert(!CameraProtocol_RingSlope(narrow, &slope));
    assert(CameraProtocol_ParseParallel("PARALLEL,1,10000,10,3,0", &cmd));
    assert(!CameraProtocol_ParseParallel("PARALLEL,1,10000,10,0,0", &cmd));
    assert(!CameraProtocol_ParseParallel("PARALLEL,1,10000,10,3,2", &cmd));
    for (unsigned reverse = 0; reverse < 2U; ++reverse) {
        reset(); camera_ready();
        feed(reverse ? "PARALLEL,901,10000,10,3,1\\n" : "PARALLEL,901,10000,10,3,0\\n");
        camera_sample("x=256,y=160\\n"); assert(position_frames == 0);
        camera_sample("RINGS,100,100,200,150,300,200\\n");
        assert(position_frames == 1 && wheel_pulses() == 30U && frame[8] == 10);
        assert(frame[6] == (reverse ? 0 : 1));
        camera_sample("RINGS,100,150,200,150,300,150\\n"); assert(position_frames == 1);
        finish_alignment_move();
        camera_sample("RINGS,100,150,200,150,300,150\\n");
        camera_sample("RINGS,100,150,200,150,300,150\\n");
        feed("MOVE,902,F,100,45,9889\\n"); assert(position_frames == 1);
        camera_sample("RINGS,100,150,200,150,300,150\\n");
        feed("MOVE,902,F,100,45,9889\\n"); assert(position_frames == 2); /* No positional compensation. */
    }
    reset(); camera_ready(); feed("PARALLEL,903,10000,10,3,0\\n");
    camera_sample("RINGS,100,200,200,150,300,100\\n"); assert(frame[6] == 0);
    feed("CMD,904,S\\n"); assert(stop_frames == 4);
    camera_sample("RINGS,100,200,200,150,300,100\\n"); assert(position_frames == 1);
    reset(); camera_ready(); feed("PARALLEL,905,10000,10,3,0\\n");
    camera_sample("RINGS,100,100,200,220,300,200\\n"); assert(position_frames == 0);
    poll_without_keep(3000);
    camera_sample("RINGS,100,100,200,150,300,200\\n"); assert(position_frames == 0);
}
''';assert mark in s;s=s.replace(mark,test+mark).replace('    test_absolute_pose();','    test_absolute_pose();\n    test_parallel_alignment();');p.write_text(s,encoding='utf-8')
p=Path('tmp/test_esp_servo_forwarding.py');s=p.read_text(encoding='utf-8').replace('"GRIP_STOP,1577625125",','"GRIP_STOP,1577625125", "PARALLEL,1577625127,9889,10,3,0", "MOVE,1577625128,C,5,10,9889", "MOVE,1577625129,W,5,10,9889",');p.write_text(s,encoding='utf-8')
p=Path('yundong_part/application/app/src/test/java/com/example/app/RobotTcpClientTest.kt');s=p.read_text(encoding='utf-8').replace('List(21)','List(22)').replace('                client.stopGripper()','                client.stopGripper()\n                client.sendParallel(9889, 10, 3, true)',1).replace('                assertTrue(frames[20].startsWith("GRIP_STOP,"))','                assertTrue(frames[20].startsWith("GRIP_STOP,"))\n                assertTrue(frames[21].startsWith("PARALLEL,") && frames[21].endsWith(",9889,10,3,1"))');p.write_text(s,encoding='utf-8')
