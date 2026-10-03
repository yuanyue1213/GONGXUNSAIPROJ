from pathlib import Path
p=Path('yundong_part/tests/distance_control_test.c');s=p.read_text(encoding='utf-8');mark='static void test_camera_alignment(void)'
new='''static void test_ring_alignment(void)
{
    CameraCenter rings[3]; CameraAlignCommand cmd;
    assert(CameraProtocol_ParseRings("RINGS,100,160,266,160,400,160", rings));
    assert(rings[1].x == 266);
    assert(!CameraProtocol_ParseRings("RINGS,100,160,512,160,400,160", rings));
    assert(!CameraProtocol_ParseRings("RINGS,400,160,266,160,100,160", rings));
    assert(!CameraProtocol_ParseRings("RINGS,100,160,266,160", rings));
    assert(!CameraProtocol_ParseRings("RINGS,100,160,266,160,400,160,junk", rings));
    assert(CameraProtocol_ParseAlign("ALIGN_RING,1,2,10000,20000", &cmd) && cmd.ring_index == 2);
    assert(!CameraProtocol_ParseAlign("ALIGN_RING,1,0,10000,20000", &cmd));
    assert(!CameraProtocol_ParseAlign("ALIGN_RING,1,4,10000,20000", &cmd));
    for (unsigned ring = 1; ring <= 3; ++ring) {
        reset(); camera_ready();
        char command[64]; snprintf(command, sizeof(command), "ALIGN_RING,1,%u,10000,20000\\n", ring);
        feed(command);
        camera_sample("x=400,y=160\\n"); assert(position_frames == 0); /* Ignore block frames. */
        camera_sample("RINGS,100,160,266,160,400,160\\n");
        assert(position_frames == 1 && wheel_pulses() == (ring == 2 ? 30U : 150U));
        finish_alignment_move();
        camera_sample("RINGS,256,160,256,160,256,160\\n");
        feed("MOVE,2,F,100,45,9889\\n"); assert(position_frames == 1);
        camera_sample("RINGS,256,160,256,160,256,160\\n");
        feed("MOVE,2,F,100,45,9889\\n"); assert(position_frames == 2);
    }
    reset(); camera_ready(); feed("ALIGN_RING,1,2,10000,20000\\n");
    camera_sample("RINGS,100,160,256,170,400,160\\n");
    assert(position_frames == 1 && wheel_pulses() == 60U); /* Y uses lateral calibration. */
    feed("CMD,2,S\\n"); camera_sample("RINGS,100,160,400,170,450,160\\n");
    assert(position_frames == 1);
    reset(); camera_ready(); feed("ALIGN,1,10000,20000\\n");
    camera_sample("RINGS,100,160,400,170,450,160\\n"); assert(position_frames == 0);
    reset(); camera_ready(); feed("ALIGN_RING,1,2,10000,20000\\n");
    poll_without_keep(3000);
    camera_sample("RINGS,100,160,400,170,450,160\\n"); assert(position_frames == 0);
}
'''
assert mark in s;s=s.replace(mark,new+mark).replace('    test_camera_alignment();','    test_camera_alignment();\n    test_ring_alignment();');p.write_text(s,encoding='utf-8')
p=Path('tmp/test_esp_servo_forwarding.py');s=p.read_text(encoding='utf-8').replace('"ALIGN,1577625096,9889,12000",','"ALIGN,1577625096,9889,12000", "ALIGN_RING,1577625120,2,9889,12000",');p.write_text(s,encoding='utf-8')
p=Path('yundong_part/application/app/src/test/java/com/example/app/RobotTcpClientTest.kt');s=p.read_text(encoding='utf-8').replace('List(14)','List(15)').replace("                client.sendMotion('P')","                client.sendMotion('P')\n                client.sendRingAlignment(0, 9889, 12000)\n                client.sendRingAlignment(2, 9889, 12000)",1).replace('                val frames = result.get','                val frames = result.get',1);s=s.replace('                assertTrue(frames[13].startsWith("CMD,") && frames[13].endsWith(",P"))','                assertTrue(frames[13].startsWith("CMD,") && frames[13].endsWith(",P"))\n                assertTrue(frames[14].startsWith("ALIGN_RING,") && frames[14].endsWith(",2,9889,12000"))');p.write_text(s,encoding='utf-8')
