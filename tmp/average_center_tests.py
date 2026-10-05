from pathlib import Path
p=Path('yundong_part/tests/distance_control_test.c'); s=p.read_text(encoding='utf-8')
a=s.index('static void camera_ready(void)')
s=s[:a]+'''static void center_average_sample(const char *line)
{
    for (unsigned i = 0; i < 5U; ++i) camera_sample(line);
}
'''+s[a:]
for name in ['test_ring_alignment', 'test_alignment_compensation_stop', 'test_camera_alignment', 'test_configured_alignment']:
 a=s.index('static void '+name+'('); b=s.index('\nstatic ',a+1)
 s=s[:a]+s[a:b].replace('camera_sample(', 'center_average_sample(')+s[b:]
a=s.index('int main(void)')
s=s[:a]+'''static void test_center_sample_average(void)
{
    for (unsigned ring = 0; ring <= 2; ring += 2) {
        reset(); camera_ready(); char command[80];
        snprintf(command, sizeof(command), "ALIGN_CFG,1500,%u,10000,20000,10,20,0\\n", ring);
        feed(command);
        for (unsigned i = 0; i < 5; ++i) {
            char sample[80];
            if (ring) snprintf(sample, sizeof(sample), "RINGS,100,160,%u,160,400,160\\n", 262+2*i);
            else snprintf(sample, sizeof(sample), "x=%03u,y=160\\n", 262+2*i);
            camera_sample(sample);
            if (i < 4) {
                assert(position_frames == 0);
                RobotControl_Tick(); assert(position_frames == 0); /* Cached frames cannot fill the group. */
            }
        }
        assert(position_frames == 1 && wheel_pulses() == 109); /* mean x=266, not the last x=270 */
        finish_alignment_move();
        camera_sample("x=400,y=160\\n"); assert(position_frames == 1);
    }
    reset(); camera_ready(); feed("ALIGN_CFG,1501,0,10000,20000,10,20,0\\n");
    camera_sample("x=266,y=160\\n"); camera_sample("x=bad,y=160\\n");
    camera_sample("x=266,y=160\\n"); camera_sample("x=266,y=160\\n");
    assert(position_frames == 0);
    feed("CMD,1502,S\\nALIGN_CFG,1503,0,10000,20000,10,20,0\\n");
    for (unsigned i = 0; i < 4; ++i) camera_sample("x=266,y=160\\n");
    assert(position_frames == 0); /* Start/cancel clears partial average. */
    tick += 3000; RobotControl_Tick(); camera_sample("x=266,y=160\\n");
    assert(position_frames == 0); /* Incomplete sample groups time out. */
}
'''+s[a:]
s=s.replace('    test_turn_cards();', '    test_turn_cards();\n    test_center_sample_average();')
p.write_text(s,encoding='utf-8',newline='\r\n')
