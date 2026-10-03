from pathlib import Path
p = Path('yundong_part/tests/distance_control_test.c')
s = p.read_text(encoding='utf-8')
s = s.replace('(ring == 2 ? 30U : 150U)', '(ring == 1 ? 1702U : ring == 2 ? 109U : 1571U)')
a=s.index('        finish_alignment_move();', s.index('static void test_ring_alignment'))
b=s.index('\n    }', a)
s=s[:a]+'''        camera_sample("RINGS,100,160,300,180,400,160\\n");
        assert(position_frames == 1); /* Ignore newer coordinates while moving. */
        finish_alignment_move();
        assert(position_frames == 2 && wheel_pulses() == 100U && frame[6] == 1);
        feed("MOVE,2,F,100,45,9889\\n"); assert(position_frames == 2);
        finish_alignment_move();
        camera_sample("RINGS,100,160,300,180,400,160\\n"); assert(position_frames == 2);
        feed("MOVE,2,F,100,45,9889\\n"); assert(position_frames == 3);'''+s[b:]
s=s.replace('wheel_pulses() == 60U); /* Y uses lateral calibration. */', 'wheel_pulses() == 218U); /* Y uses lateral calibration. */')
s=s.replace('camera_sample(sample); assert(position_frames == 0);\n        camera_sample(sample); assert(position_frames == 1 && wheel_pulses() == 100U);', 'camera_sample(sample); assert(position_frames == 1 && wheel_pulses() == 100U);')
a=s.index('    CameraCenter center; CameraAlignCommand cmd; char dir; uint32_t mm;', s.index('static void test_camera_alignment'))
b=s.index('    reset(); camera_ready();\n    camera_feed("x=2");', a)
s=s[:a]+'''    CameraCenter center; CameraAlignCommand cmd;
    int32_t pulses[4]; uint16_t speeds[4]; bool aligned;
    CameraAlignSettings settings = CameraProtocol_DefaultSettings();
    assert(CameraProtocol_ParseCenter("x=256,y=160", &center));
    assert(CameraProtocol_CenterProfile(&center, 10000, 10000, &settings, pulses, speeds, &aligned) && aligned);
    assert(CameraProtocol_ParseAlign("ALIGN,1,9889,9889", &cmd));
    const char *bad[] = {"x=512,y=160", "x=256,y=320", "x=25,y=160", "x=256,y=160x", "x=-01,y=160"};
    for (unsigned i = 0; i < sizeof(bad)/sizeof(bad[0]); ++i) assert(!CameraProtocol_ParseCenter(bad[i], &center));
    const CameraCenter points[] = {{356,160}, {156,160}, {256,260}, {256,60}};
    const int32_t expected[4][4] = {{1091,1091,1091,1091}, {-1091,-1091,-1091,-1091},
        {2182,-2182,2182,-2182}, {-2182,2182,-2182,2182}};
    for (unsigned i = 0; i < 4; ++i) {
        assert(CameraProtocol_CenterProfile(&points[i], 10000, 20000, &settings, pulses, speeds, &aligned));
        assert(!aligned);
        for (unsigned j = 0; j < 4; ++j) assert(pulses[j] == expected[i][j] && speeds[j] == 20);
    }
    center = (CameraCenter){356,210};
    assert(CameraProtocol_CenterProfile(&center, 10000, 10000, &settings, pulses, speeds, &aligned));
    assert(pulses[0] == 1636 && pulses[1] == 546 && pulses[2] == 1636 && pulses[3] == 546);
    assert(speeds[0] == 20 && speeds[1] == 7 && speeds[2] == 20 && speeds[3] == 7);
    center = (CameraCenter){266,170};
    assert(CameraProtocol_CenterProfile(&center, 10000, 10000, &settings, pulses, speeds, &aligned));
    assert(pulses[0] == 218 && pulses[1] == 0 && speeds[0] == 10); /* Diagonal: two wheels stay still. */
    center = (CameraCenter){259,160};
    assert(CameraProtocol_CenterProfile(&center, 1, 1, &settings, pulses, speeds, &aligned) && pulses[0] == 1);
    center = (CameraCenter){258,158};
    assert(CameraProtocol_CenterProfile(&center, 10000, 20000, &settings, pulses, speeds, &aligned) && aligned);
    center = (CameraCenter){512,160};
    assert(!CameraProtocol_CenterProfile(&center, 10000, 20000, &settings, pulses, speeds, &aligned));

    reset(); camera_ready();
    camera_feed("x=400,y=160\\n"); /* Discard stale coordinates at start. */
    feed("ALIGN,1,10000,20000\\n"); RobotControl_Tick(); assert(position_frames == 0);
    camera_sample("x=266,y=160\\n");
    assert(position_frames == 1 && wheel_pulses() == 109 && frame[6] == 1 && frame[8] == 10);
    camera_sample("x=256,y=180\\n"); assert(position_frames == 1);
    feed("MOVE,2,F,100,45,9889\\nCMD,3,A\\n");
    assert(position_frames == 1 && servo_commands == 0);
    finish_alignment_move();
    assert(position_frames == 2 && wheel_pulses() == 100 && frame[8] == 10);
    camera_sample("x=400,y=180\\n"); assert(position_frames == 2);
    finish_alignment_move();
    camera_sample("x=400,y=180\\n"); assert(position_frames == 2); /* No resampling after arrival. */
    feed("ALIGN,1,10000,20000\\n"); camera_sample("x=266,y=160\\n"); assert(position_frames == 2);
    feed("ALIGN,4,10000,20000\\n"); camera_sample("x=266,y=160\\n"); assert(position_frames == 3); /* New sequence can run again. */

    reset(); camera_ready(); feed("ALIGN_CFG,1,0,10000,10000,10,20,0\\n");
    camera_sample("x=356,y=210\\n");
    assert(position_frames == 1 && wheel_pulses() == 1636 && frame[8] == 20);
    assert(frame[21] == 7 && frame[23] == 0 && frame[24] == 0 && frame[25] == 2 && frame[26] == 34);
    finish_alignment_move(); camera_sample("x=400,y=180\\n"); assert(position_frames == 1);
    feed("MOVE,2,F,100,45,9889\\n"); assert(position_frames == 2); /* Zero offset completes immediately on arrival. */
    reset(); camera_ready(); feed("ALIGN,1,9889,9889\\n");
    tick = 3000; RobotControl_Tick(); camera_sample("x=400,y=160\\n"); assert(position_frames == 0);
    feed("MOVE,2,F,100,45,9889\\n"); assert(position_frames == 1);
    reset(); camera_ready(); feed("ALIGN,1,9889,9889\\n"); camera_sample("x=266,y=160\\n");
    feed("CMD,2,S\\n"); assert(stop_frames == 4); camera_sample("x=400,y=160\\n"); assert(position_frames == 1);
    reset(); camera_ready(); feed("ALIGN,1,9889,9889\\n"); camera_sample("x=266,y=160\\n");
    fail_status = 1; tick += 60; RobotControl_Tick(); assert(stop_frames == 4);
    camera_sample("x=400,y=160\\n"); assert(position_frames == 1);
    reset(); camera_ready(); feed("ALIGN,1,9889,9889\\n"); camera_sample("x=266,y=160\\n");
    tick = 120000; RobotControl_Tick(); assert(stop_frames == 4);
    camera_sample("x=400,y=160\\n"); assert(position_frames == 1);
'''+s[b:]
a=s.index('        assert(position_frames == 1', s.index('static void test_configured_alignment'))
b=s.index('        feed("ALIGN,701',a)
s=s[:a]+'''        assert(position_frames == 1 && wheel_pulses() == 436U && frame[8] == 30);
        finish_alignment_move();
        assert(position_frames == 2 && wheel_pulses() == 70U && frame[8] == 12 && frame[6] == 0);
        const char *center = mode ? "RINGS,100,160,266,160,400,160\\n" : "x=266,y=160\\n";
        camera_sample(center); assert(position_frames == 2);
        finish_alignment_move(); camera_sample(center); assert(position_frames == 2);
'''+s[b:]
s=s.replace('position_frames == 4 && frame[8] == 20); /* Legacy resets speed defaults. */', 'position_frames == 3 && wheel_pulses() == 436 && frame[8] == 20); /* Legacy resets speed defaults. */')
p.write_text(s, encoding='utf-8', newline='\r\n')
