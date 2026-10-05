from pathlib import Path
p=Path('yundong_part/tests/distance_control_test.c'); s=p.read_text(encoding='utf-8')
s=s.replace('    assert(servo_history[0] == 30 && servo_history[1] == 100 && servo_angle == 250);\n    finish_grab_axes();', '    assert(servo_commands == 0); /* Return to origin before changing posture. */\n    finish_grab_axes(); assert(servo_history[0] == 100 && servo_angle == 250);')
s=s.replace('    assert(servo_history[servo_commands-3] == 30);', "    for (unsigned i = 0; i < servo_commands; ++i) assert(servo_channels[i] != 'T');")
pos=s.index('int main(void)')
s=s[:pos]+'''static void test_turn_cards(void)
{
    RobotSequenceCommand cmd;
    assert(RobotProtocol_ParseSequence("PLAN_ITEM,100,0,T,0,240,120", &cmd));
    assert(cmd.mode == 'T' && cmd.theta == 0 && cmd.open_angle == 240 && cmd.gripper_dps == 120);
    assert(!RobotProtocol_ParseSequence("STATE,100,T,0,271,120", &cmd));
    assert(!RobotProtocol_ParseSequence("STATE,100,T,0,240,0", &cmd));
    assert(!RobotProtocol_ParseSequence("STATE,100,T,0,240,361", &cmd));
    assert(!RobotProtocol_ParseSequence("PLAN_ITEM,100,16,T,0,240,120", &cmd));
    reset();
    feed("PLAN_BEGIN,1400,2\\nPLAN_ITEM,1400,0,T,0,120,60\\nPLAN_ITEM,1400,1,T,120,0,120\\nPLAN_RUN,1400\\n");
    assert(servo_channel == 'T' && servo_angle == 0 && absolute_commands[1] == 0);
    tick += 1000; RobotControl_Tick(); assert(servo_angle == 60);
    tick += 1000; RobotControl_Tick(); assert(servo_angle == 120);
    tick += 999; RobotControl_Tick(); assert(servo_angle == 120);
    tick += 1; RobotControl_Tick(); assert(servo_angle == 120);
    tick += 500; RobotControl_Tick(); assert(servo_angle == 60);
    feed("CMD,1401,Z\\n"); unsigned servos = servo_commands;
    tick += 5000; RobotControl_Tick(); assert(servo_commands == servos);
    feed("STATE,1402,T,20,20,1\\n"); RobotControl_Tick(); assert(servo_angle == 20);
    feed("STATE,1403,T,20,80,60\\n"); tick += 1000; RobotControl_Tick(); assert(servo_angle == 80);
    feed("STATE,1404,P,0,0,0,0,20,20,50\\n");
    unsigned start = servo_commands;
    arm_flags = lift_flags = 3;
    for (unsigned i = 0; i < 600; ++i) { tick += 50; RobotControl_Tick(); }
    for (unsigned i = start; i < servo_commands; ++i) assert(servo_channels[i] != 'T');
    feed("STATE,1405,T,80,140,60\\n"); tick += 1000; RobotControl_Tick(); assert(servo_angle == 140);
}
'''+s[pos:]
s=s.replace('    test_initial_base_angle_updates();', '    test_initial_base_angle_updates();\n    test_turn_cards();')
p.write_text(s,encoding='utf-8',newline='\r\n')
p=Path('yundong_part/application/app/src/test/java/com/example/app/SequenceCardsTest.kt'); s=p.read_text(encoding='utf-8'); at=s.rfind('}')
s=s[:at]+'''    @Test fun turnCardsPersistAndMixWithArmCards() {
        val turn = SequenceCard("turn-1", "转盘120", 'T', turn = TurnSettings(0,120,60))
        val cards = listOf(turn, card.copy(mode = 'P'), turn.copy(turn = TurnSettings(120,0,120)))
        assertEquals(cards, SequenceCardCodec.decode(SequenceCardCodec.encode(cards)))
        val frames = SequencePlan.frames(8, cards)
        assertEquals("PLAN_ITEM,8,0,T,0,120,60\\n", frames[1])
        assertTrue(frames[2].startsWith("PLAN_ITEM,8,1,P,"))
        assertEquals("PLAN_ITEM,8,2,T,120,0,120\\n", frames[3])
        assertFalse(turn.copy(turn = TurnSettings(0,271,60)).valid())
        assertFalse(turn.copy(turn = TurnSettings(0,120,0)).valid())
        assertTrue(SequenceCardCodec.decode("x|y|T,0,120,0").isEmpty())
    }
'''+s[at:]; p.write_text(s,encoding='utf-8',newline='\r\n')
p=Path('yundong_part/shared/robot_distance_protocol.h'); s=p.read_text(encoding='utf-8'); s=s.replace('    cmd->plan_index = 0U;','    *cmd = (RobotSequenceCommand){0};'); p.write_text(s,encoding='utf-8',newline='\r\n')
