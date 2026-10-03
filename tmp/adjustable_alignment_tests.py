from pathlib import Path
p=Path('yundong_part/tests/distance_control_test.c');s=p.read_text(encoding='utf-8');mark='static void test_absolute_pose(void)';test='''static void test_configured_alignment(void)
{
    CameraAlignCommand cmd;
    assert(CameraProtocol_ParseAlign("ALIGN_CFG,1,2,10000,20000,12,30,-7", &cmd));
    assert(cmd.ring_index == 2 && cmd.settings.offset_mm == -7);
    assert(!CameraProtocol_ParseAlign("ALIGN_CFG,1,2,10000,20000,4,30,-7", &cmd));
    assert(!CameraProtocol_ParseAlign("ALIGN_CFG,1,4,10000,20000,12,30,-7", &cmd));
    assert(!CameraProtocol_ParseAlign("ALIGN_CFG,1,2,10000,20000,12,30,101", &cmd));
    assert(!CameraProtocol_ParseAlign("ALIGN_CFG,1,2,10000,20000,12,30,-7,junk", &cmd));
    for (unsigned mode = 0; mode < 2U; ++mode) {
        reset(); camera_ready();
        feed(mode ? "ALIGN_CFG,700,2,10000,20000,12,30,-7\\n" :
                    "ALIGN_CFG,700,0,10000,20000,12,30,-7\\n");
        camera_sample(mode ? "RINGS,100,160,296,160,400,160\\n" : "x=296,y=160\\n");
        assert(position_frames == 1 && wheel_pulses() == 150U && frame[8] == 30);
        finish_alignment_move();
        camera_sample(mode ? "RINGS,100,160,266,160,400,160\\n" : "x=266,y=160\\n");
        assert(position_frames == 2 && wheel_pulses() == 30U && frame[8] == 12);
        finish_alignment_move();
        const char *center = mode ? "RINGS,100,160,256,160,400,160\\n" : "x=256,y=160\\n";
        camera_sample(center); assert(position_frames == 2);
        camera_sample(center);
        assert(position_frames == 3 && wheel_pulses() == 70U && frame[8] == 12 && frame[6] == 0);
        finish_alignment_move(); camera_sample(center); assert(position_frames == 3);
        feed("ALIGN,701,10000,20000\\n"); camera_sample("x=296,y=160\\n");
        assert(position_frames == 4 && frame[8] == 20); /* Legacy resets speed defaults. */
    }
    reset(); camera_ready(); feed("ALIGN_CFG,702,0,10000,20000,12,30,0\\n");
    camera_sample("x=256,y=160\\n"); camera_sample("x=256,y=160\\n"); assert(position_frames == 0);
    feed("MOVE,703,F,100,45,9889\\n"); assert(position_frames == 1); /* Zero offset finishes without moving. */
}
''';assert mark in s;s=s.replace(mark,test+mark).replace('    test_absolute_pose();','    test_absolute_pose();\n    test_configured_alignment();');p.write_text(s,encoding='utf-8')
p=Path('tmp/test_esp_servo_forwarding.py');s=p.read_text(encoding='utf-8').replace('#define MAX_FRAME_LENGTH 63','#define MAX_FRAME_LENGTH 127');s=s.replace('"STATE,1577625121,P,200,-40,1300,-110,20,20,50",','"STATE,1577625121,P,200,-40,1300,-110,20,20,50", "STATE,1577625122,P,200,-40,1300,-110,20,20,50,30", "ALIGN_CFG,1577625123,2,9889,12000,12,30,-7",');s=s.replace('    puts("PASS:', '    assert(handle_frame("ALIGN_CFG,1,2,9889,12000,0,20,10", &lift));\n    assert(handle_frame("STATE,1,P,200,-40,1300,-110,20,20,50,0", &lift));\n    puts("PASS:');p.write_text(s,encoding='utf-8')
base=Path('yundong_part/application/app/src/test/java/com/example/app')
(base/'AlignmentSettingsTest.kt').write_text('''package com.example.app

import org.junit.Assert.*
import org.junit.Test

class AlignmentSettingsTest {
    @Test fun configurableSpeedAndOffset() {
        val settings = AlignmentSettings.parse("12", "30", "-7")!!
        assertEquals("ALIGN_CFG,1,2,9889,12000,12,30,-7\\n", settings.frame(1, 2, 9889, 12000))
        assertEquals(0, AlignmentSettings.parse("10", "20", "0")!!.offsetMm)
    }
    @Test fun invalidParameters() {
        assertNull(AlignmentSettings.parse("4", "20", "10"))
        assertNull(AlignmentSettings.parse("10", "61", "10"))
        assertNull(AlignmentSettings.parse("10", "20", "101"))
        assertNull(AlignmentSettings.parse("10", "20", "1.5"))
    }
}
''',encoding='utf-8')
p=base/'RobotTcpClientTest.kt';s=p.read_text(encoding='utf-8').replace('List(17)','List(19)');a="                client.sendSequence('P', SequenceSettings(200, -40, 1300, -110, 25, 15, 55))";assert a in s;s=s.replace(a,a+'''
                client.sendConfiguredAlignment(0, 9889, 12000, AlignmentSettings(12, 30, -7))
                client.sendConfiguredAlignment(2, 9889, 12000, AlignmentSettings(10, 20, 0))''');a='                assertTrue(frames[16].startsWith("STATE,") && frames[16].endsWith(",P,200,-40,1300,-110,25,15,55,60"))';assert a in s;s=s.replace(a,a+'''
                assertTrue(frames[17].startsWith("ALIGN_CFG,") && frames[17].endsWith(",0,9889,12000,12,30,-7"))
                assertTrue(frames[18].startsWith("ALIGN_CFG,") && frames[18].endsWith(",2,9889,12000,10,20,0"))''');p.write_text(s,encoding='utf-8')
