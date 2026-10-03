#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "stm32h7xx_hal.h"
#include "zdt_motor.h"
#include "robot_control.h"
#include "camera_link.h"
#include "../shared/robot_distance_protocol.h"

static UART_HandleTypeDef motor_uart = {1, HAL_UART_STATE_READY};
static UART_HandleTypeDef command_uart = {3, HAL_UART_STATE_READY};
static UART_HandleTypeDef camera_uart = {4, HAL_UART_STATE_READY};
static uint32_t tick;
static uint8_t *rx_byte;
static uint8_t *camera_rx_byte;
static uint8_t reply[4], reply_index;
static uint8_t frame[57];
static unsigned position_frames, stop_frames;
static uint8_t motor_status[4];
static int fail_status, fail_position_ack, fail_stop_ack;
static int fail_rx_start;
static unsigned lift_stops, arm_stops;
static unsigned lift_angle_commands, lift_velocity_commands;
static uint8_t lift_flags;
static int fail_lift_status, fail_lift_stop, fail_lift_angle;
static struct { char direction; uint32_t angle, rpm, pulses; } last_lift_angle;
static unsigned servo_commands;
static char servo_channel;
static uint16_t servo_angle;
static uint16_t servo_history[512];
static char servo_channels[512];
static int fail_servo;
static unsigned arm_angle_commands;
static struct { char direction; uint32_t angle_tenths, speed_rpm, pulses_per_revolution; } last_arm_angle;
static uint8_t arm_flags;
static int fail_arm_status, fail_arm_stop, fail_arm_angle;
static int fail_origin_read;
static int64_t origin_position[2], absolute_target[2];
static unsigned absolute_commands[2];
static unsigned origin_reads;

uint32_t HAL_GetTick(void) { return tick; }
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *uart, uint8_t *data, uint16_t length)
{
    assert(length == 1);
    if (uart == &camera_uart) {
        if (uart->RxState == HAL_UART_STATE_BUSY_RX) return HAL_BUSY;
        uart->RxState = HAL_UART_STATE_BUSY_RX; camera_rx_byte = data; return HAL_OK;
    }
    assert(uart == &command_uart);
    if (fail_rx_start > 0) { --fail_rx_start; return HAL_ERROR; }
    if (uart->RxState == HAL_UART_STATE_BUSY_RX) return HAL_BUSY;
    uart->RxState = HAL_UART_STATE_BUSY_RX;
    rx_byte = data;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *uart, uint8_t *data,
                                    uint16_t length, uint32_t timeout)
{
    (void)timeout;
    assert(uart != &command_uart); /* Application UART must never send a reply. */
    assert(uart == &motor_uart);
    reply_index = 0;
    reply[0] = data[0]; reply[1] = data[1]; reply[2] = 2; reply[3] = 0x6B;
    if (data[1] == 0xAA)
    {
        assert(length == sizeof(frame));
        memcpy(frame, data, length);
        ++position_frames;
        reply[0] = 1; reply[1] = 0xFD;
        if (fail_position_ack) reply_index = 4;
    }
    if (data[1] == 0x3A)
    {
        reply[2] = motor_status[data[0] - 1];
        if (fail_status) reply_index = 4;
    }
    if (data[1] == 0xFE)
    {
        ++stop_frames;
        if (fail_stop_ack) reply_index = 4;
    }
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *uart, uint8_t *data,
                                   uint16_t length, uint32_t timeout)
{
    assert(uart == &motor_uart && length == 1);
    if (reply_index == 4) { tick += timeout; return HAL_TIMEOUT; }
    *data = reply[reply_index++];
    return HAL_OK;
}
HAL_StatusTypeDef LiftMotor_Move(char d) { (void)d; ++lift_velocity_commands; return HAL_OK; }
HAL_StatusTypeDef LiftMotor_Stop(void) { ++lift_stops; return fail_lift_stop ? HAL_ERROR : HAL_OK; }
HAL_StatusTypeDef LiftMotor_MoveAngle(char d, uint32_t angle, uint16_t rpm, uint32_t pulses)
{
    ++lift_angle_commands;
    last_lift_angle.direction = d; last_lift_angle.angle = angle;
    last_lift_angle.rpm = rpm; last_lift_angle.pulses = pulses;
    return fail_lift_angle ? HAL_ERROR : HAL_OK;
}
HAL_StatusTypeDef ArmMotor_MoveForeAft(char d) { (void)d; return HAL_OK; }
HAL_StatusTypeDef ArmMotor_StopForeAft(void) { ++arm_stops; return fail_arm_stop ? HAL_ERROR : HAL_OK; }
HAL_StatusTypeDef ArmMotor_MoveAngle(char d, uint32_t angle, uint16_t rpm, uint32_t pulses)
{
    ++arm_angle_commands;
    last_arm_angle.direction = d; last_arm_angle.angle_tenths = angle;
    last_arm_angle.speed_rpm = rpm; last_arm_angle.pulses_per_revolution = pulses;
    return fail_arm_angle ? HAL_ERROR : HAL_OK;
}
bool LiftMotor_UartReady(void) { return true; }
bool LiftMotor_Probe(uint8_t id) { (void)id; return true; }
char LiftMotor_FirmwareCode(uint8_t id) { (void)id; return 'E'; }
const char *LiftMotor_LastError(uint8_t id) { (void)id; return "NONE"; }
bool LiftMotor_ReadStatus(uint8_t id, uint8_t *status) {
    if (id == 1) { *status = lift_flags; return !fail_lift_status; }
    assert(id == 2); *status = arm_flags; return !fail_arm_status;
}
bool ArmMotor_ReadPosition(uint8_t id, int64_t *angle) {
    ++origin_reads;
    if (fail_origin_read) return false;
    *angle = origin_position[id - 1U]; return true;
}
HAL_StatusTypeDef ArmMotor_MoveAbsolute(uint8_t id, int64_t angle, uint16_t rpm, uint32_t ppr) {
    absolute_target[id-1U] = angle; ++absolute_commands[id-1U];
    return id == 1U ? LiftMotor_MoveAngle(angle < 0 ? 'U' : 'D', (uint32_t)(angle < 0 ? -angle : angle), rpm, ppr) :
        ArmMotor_MoveAngle(angle < 0 ? 'C' : 'E', (uint32_t)(angle < 0 ? -angle : angle), rpm, ppr);
}
bool ServoControl_SetAngle(char c, uint16_t a)
{
    assert(servo_commands < 512);
    servo_history[servo_commands] = a;
    servo_channels[servo_commands] = c;
    ++servo_commands;
    servo_channel = c;
    servo_angle = a;
    return !fail_servo;
}

static void feed(const char *command);
static void reset(void)
{
    tick = 0; position_frames = stop_frames = 0;
    fail_status = fail_position_ack = fail_stop_ack = 0;
    memset(motor_status, 1, sizeof(motor_status));
    lift_stops = arm_stops = 0;
    lift_angle_commands = lift_velocity_commands = 0; lift_flags = 1;
    fail_lift_status = fail_lift_stop = fail_lift_angle = 0;
    servo_commands = 0;
    fail_servo = 0;
    arm_angle_commands = 0; arm_flags = 1;
    fail_arm_status = fail_arm_stop = fail_arm_angle = 0;
    fail_origin_read = 0; origin_position[0] = origin_position[1] = 0;
    command_uart.RxState = HAL_UART_STATE_READY;
    fail_rx_start = 0;
    ZDT_Motor_Init(&motor_uart, true, 20);
    CameraLink_Init(NULL);
    RobotControl_Init(&command_uart, true);
    assert(servo_commands == 0);
    feed("CMD,4294967295,I\n");
    assert(servo_commands == 3 && servo_history[0] == 0 && servo_history[1] == 15 && servo_history[2] == 248);
    servo_commands = 0; /* Other tests count commands after the startup pose. */
    memset(absolute_commands, 0, sizeof(absolute_commands));
}
static void feed(const char *command)
{
    while (*command) {
        *rx_byte = (uint8_t)*command++;
        command_uart.RxState = HAL_UART_STATE_READY;
        HAL_UART_RxCpltCallback(&command_uart);
    }
    RobotControl_Process();
}
static void poll_without_keep(unsigned elapsed)
{
    tick += elapsed;
    RobotControl_Tick();
}
static void test_parser(void)
{
    RobotDistanceCommand command;
    assert(RobotProtocol_ParseMove("MOVE,1,F,100,45,9889", &command));
    assert(RobotProtocol_MovePulses(&command) == 989);
    const char *invalid[] = {
        "MOVE,0,F,100,45,9889", "MOVE,4294967296,F,100,45,9889",
        "MOVE,1,F,-100,45,9889", "MOVE,1,S,100,45,9889",
        "MOVE,1,F,10001,45,9889", "MOVE,1,F,100,301,9889",
        "MOVE,1,F,100,45,1000001", "MOVE,1,F,100,45,9889,",
        "MOVE,1,F,100,45,9889x", "MOVE,1,F,100, 45,9889"
    };
    for (unsigned i = 0; i < sizeof(invalid)/sizeof(invalid[0]); ++i)
        assert(!RobotProtocol_ParseMove(invalid[i], &command));
    assert(RobotProtocol_ParseMove("MOVE,4294967295,R,10000,300,1000000", &command));
    assert(RobotProtocol_MovePulses(&command) == 10000000);
}
static void test_direction_frames(void)
{
    const char directions[] = "FBLRCW";
    const uint8_t expected[6][4] = {{0,1,1,0}, {1,0,0,1}, {1,1,0,0}, {0,0,1,1}, {1,1,1,1}, {0,0,0,0}};
    for (unsigned d = 0; d < 6; ++d)
    {
        char command[64]; reset();
        snprintf(command, sizeof(command), "MOVE,1,%c,100,45,9889\n", directions[d]);
        feed(command);
    assert(position_frames == 1);
        assert(frame[0] == 0 && frame[1] == 0xAA && frame[2] == 0 && frame[3] == 57 && frame[56] == 0x6B);
        for (unsigned wheel = 0; wheel < 4; ++wheel)
        {
            unsigned offset = 4 + wheel * 13;
            assert(frame[offset] == wheel + 1 && frame[offset+1] == 0xFD);
            assert(frame[offset+2] == expected[d][wheel]);
            assert(frame[offset+3] == 0 && frame[offset+4] == 45 && frame[offset+5] == 10);
            assert(frame[offset+6] == 0 && frame[offset+7] == 0 && frame[offset+8] == 3 && frame[offset+9] == 0xDD);
            assert(frame[offset+10] == 2 && frame[offset+11] == 0 && frame[offset+12] == 0x6B);
        }
    }
}
static void test_completion_and_duplicates(void)
{
    reset(); feed("MOVE,1,F,100,45,9889\n");
    feed("MOVE,2,L,100,45,9889\n");
    assert(position_frames == 1);
    motor_status[0] = motor_status[1] = motor_status[2] = 3;
    for (unsigned i = 0; i < 4; ++i) poll_without_keep(60);
    feed("MOVE,4,F,100,45,9889\n");
    assert(position_frames == 1); /* Three reached is insufficient. */
    motor_status[3] = 3;
    for (unsigned i = 0; i < 4; ++i) poll_without_keep(60);
    assert(position_frames == 1);
    feed("MOVE,1,F,100,45,9889\n");
    assert(position_frames == 1);
    feed("MOVE,3,B,100,45,9889\n");
    assert(position_frames == 2);
}
static void test_timeout_stop_fault(void)
{
    reset(); feed("MOVE,1,F,100,45,9889\n");
    tick = 1000; RobotControl_Tick();
    assert(stop_frames == 0);
    tick = 5000; RobotControl_Tick();
    assert(stop_frames == 0);
    tick = 1800000; RobotControl_Tick();
    assert(stop_frames == 4);
    reset(); feed("CMD,10,U\nARM_MOVE,11,E,127,10,3200\n");
    tick = 300; RobotControl_Tick();
    assert(lift_stops == 1);
    assert(arm_stops == 0);
    reset(); feed("MOVE,1,F,100,45,9889\n");
    feed("CMD,8,S\n");
    assert(stop_frames == 4);
    feed("CMD,9,F\n");
    assert(position_frames == 1);
    reset(); feed("MOVE,1,F,100,45,9889\n");
    motor_status[0] = 0x0D; poll_without_keep(60);
    assert(stop_frames == 4);
    reset(); feed("MOVE,1,F,100,45,9889\n");
    fail_status = 1; poll_without_keep(60);
    assert(stop_frames == 4);
    reset(); fail_position_ack = 1; feed("MOVE,1,F,100,45,9889\n");
    assert(stop_frames == 4);
    feed("MOVE,1,F,100,45,9889\n");
    assert(position_frames == 1); /* Lost ACK cannot cause replay. */
    reset(); feed("MOVE,1,F,100,45,9889\n");
    fail_stop_ack = 1; feed("CMD,8,S\n");
    fail_stop_ack = 0; RobotControl_Tick();
    assert(stop_frames == 8);
}
static void test_stream_recovery(void)
{
    reset();
    feed("MOVE,1,F,");
    assert(position_frames == 0);
    feed("100,45,9889\r\n");
    assert(position_frames == 1);
    reset();
    char oversized[82];
    memset(oversized, 'X', 80); oversized[80] = '\n'; oversized[81] = '\0';
    feed(oversized);
    assert(position_frames == 0);
    feed("MOVE,1,F,100,45,9889\n");
    assert(position_frames == 1);
    reset();
    for (unsigned i = 0; i < 600; ++i)
    {
        *rx_byte = 'X';
        command_uart.RxState = HAL_UART_STATE_READY;
        HAL_UART_RxCpltCallback(&command_uart);
    }
    RobotControl_Process();
    feed("\nMOVE,1,F,100,45,9889\n");
    assert(position_frames == 1); /* Discarded overflow cannot become a command. */
}
static void test_receive_restart(void)
{
    reset();
    command_uart.RxState = HAL_UART_STATE_READY;
    fail_rx_start = 1;
    RobotControl_Init(&command_uart, true);
    assert(command_uart.RxState == HAL_UART_STATE_READY);
    RobotControl_Process();
    assert(command_uart.RxState == HAL_UART_STATE_BUSY_RX);
    feed("MOVE,1,F,100,45,9889\n");
    assert(position_frames == 1);

    reset();
    command_uart.RxState = HAL_UART_STATE_READY;
    fail_rx_start = 1;
    HAL_UART_ErrorCallback(&command_uart);
    RobotControl_Process();
    assert(command_uart.RxState == HAL_UART_STATE_BUSY_RX);
    feed("\nMOVE,3,F,100,45,9889\n");
    assert(position_frames == 1);
}
static void test_servo_commands_without_replies(void)
{
    reset();
    feed("SERVO,100,G,90\n");
    assert(servo_commands == 1 && servo_channel == 'G' && servo_angle == 90);
    feed("SERVO,101,T,270\r\nSERVO,102,B,360\n");
    assert(servo_commands == 3 && servo_channel == 'B' && servo_angle == 360);
    feed("SERVO,103,G,271\nSERVO,104,B,361\nSERVO,105,X,90\n");
    assert(servo_commands == 3);
    feed("MOVE,1,F,100,45,9889\nSERVO,106,T,45\n");
    assert(position_frames == 1 && servo_commands == 4);
    assert(servo_channel == 'T' && servo_angle == 45);
}
static void finish_grab_axes(void)
{
    arm_flags = 3; tick += 50; RobotControl_Tick();
    lift_flags = 3; tick += 50; RobotControl_Tick();
    arm_flags = lift_flags = 1;
}
static void advance_grab_hold(void) { tick += 1000; RobotControl_Tick(); }
static void finish_gripper_open(void)
{
    unsigned r_count = absolute_commands[1], z_count = absolute_commands[0];
    tick += 500; RobotControl_Tick();
    assert(servo_channel == 'G' && servo_angle == 30);
    assert(absolute_commands[1] == r_count && absolute_commands[0] == z_count);
    tick += 500; RobotControl_Tick();
    assert(servo_channel == 'G' && servo_angle == 60);
}

static void finish_grab_cycle(unsigned seq, uint16_t expected_turntable)
{
    feed("CMD,999,Z\n"); /* Only cancels; does not reset the completed rotation index. */
    char frame_text[32]; snprintf(frame_text, sizeof(frame_text), "CMD,%u,A\n", seq); feed(frame_text);
    assert(absolute_target[1] == 0 && servo_angle == 248); /* State 1: home. */
    finish_grab_axes(); assert(absolute_target[0] == 0 && last_lift_angle.rpm == 20);
    advance_grab_hold(); assert(absolute_target[0] == 4500 && absolute_target[1] == 0 && last_lift_angle.rpm == 50); /* State 2: z first. */
    tick += 50; RobotControl_Tick(); assert(absolute_target[1] == 0); /* No extension until lift reaches. */
    lift_flags = 3; tick += 50; RobotControl_Tick(); assert(absolute_target[1] == 3118);
    arm_flags = 3; tick += 50; RobotControl_Tick(); arm_flags = lift_flags = 1;
    unsigned r_count = absolute_commands[1], z_count = absolute_commands[0];
    advance_grab_hold(); assert(servo_history[servo_commands-2] == 0 && servo_angle == 248); /* State 3. */
    assert(absolute_commands[1] == r_count && absolute_commands[0] == z_count);
    advance_grab_hold(); assert(absolute_target[1] == 0); /* State 4. */
    finish_grab_axes(); assert(absolute_target[0] == 0 && last_lift_angle.rpm == 20);
    advance_grab_hold(); assert(servo_angle == 248); /* State 5 starts the base ramp. */
    tick += 1000; RobotControl_Tick(); assert(servo_angle == 194);
    tick += 1000; RobotControl_Tick(); assert(servo_angle == 140);
    advance_grab_hold(); assert(absolute_target[1] == 567); /* State 6: r20,z-40. */
    finish_grab_axes(); assert(absolute_target[0] == 3600 && last_lift_angle.rpm == 50);
    advance_grab_hold(); assert(servo_history[servo_commands-2] == 0 && absolute_target[1] == 567); /* State 7 starts slow opening. */
    finish_gripper_open(); assert(absolute_target[1] == 0);
    finish_grab_axes(); assert(absolute_target[0] == 0 && last_lift_angle.rpm == 20);
    advance_grab_hold(); assert(servo_angle == 140); /* State 8. */
    tick += 1000; RobotControl_Tick(); assert(servo_angle == 194);
    tick += 1000; RobotControl_Tick(); assert(servo_angle == 248);
    advance_grab_hold(); assert(servo_channel == 'T' && servo_angle == expected_turntable);
    unsigned before = servo_commands; tick += 1000; RobotControl_Tick(); assert(servo_commands == before);
    feed(frame_text); assert(servo_commands == before); /* Same accepted sequence must not replay. */
}
static void test_gripper_open_cancel(void)
{
    reset(); feed("CMD,200,A\n"); finish_grab_axes();
    advance_grab_hold(); lift_flags = 3; tick += 50; RobotControl_Tick();
    arm_flags = 3; tick += 50; RobotControl_Tick(); arm_flags = lift_flags = 1;
    advance_grab_hold(); advance_grab_hold(); finish_grab_axes();
    advance_grab_hold(); tick += 2000; RobotControl_Tick();
    advance_grab_hold(); finish_grab_axes(); advance_grab_hold();
    unsigned r_count = absolute_commands[1];
    tick += 500; RobotControl_Tick(); assert(servo_channel == 'G' && servo_angle == 30);
    feed("CMD,201,Z\n"); unsigned servos = servo_commands;
    tick += 1500; RobotControl_Tick();
    assert(servo_commands == servos && absolute_commands[1] == r_count);
}
static void test_grab_state(void)
{
    reset(); finish_grab_cycle(20, 120); finish_grab_cycle(21, 240); finish_grab_cycle(22, 0);
    assert(absolute_commands[1] == 15 && absolute_commands[0] == 15); /* Five coordinate states per cycle. */
    feed("CMD,23,O\n"); finish_grab_axes(); finish_grab_cycle(24, 120); /* Home resets the rotation cycle. */
    reset(); feed("CMD,30,A\nCMD,31,A\nARM_POSE,32,0,0,0,5,3200\nARM_MOVE,33,E,10,10,3200\n");
    assert(arm_angle_commands == 1 && servo_commands == 3);
    feed("CMD,34,Z\n"); assert(arm_stops == 1); tick = 5000; RobotControl_Tick(); assert(lift_angle_commands == 0);
    reset(); feed("CMD,35,A\n"); arm_flags = 3; tick = 50; RobotControl_Tick();
    assert(lift_angle_commands == 1); feed("CMD,36,H\n"); assert(lift_stops == 1);
    lift_flags = 3; tick = 5000; RobotControl_Tick(); assert(servo_commands == 3);
    reset(); feed("CMD,37,A\nSERVO,38,G,90\n"); assert(arm_stops == 1 && servo_angle == 90);
    reset(); feed("CMD,39,A\nCMD,40,Q\n"); assert(arm_stops == 1);
    reset(); fail_servo = 1; feed("CMD,41,A\n"); assert(arm_angle_commands == 0);
    reset(); fail_arm_angle = 1; feed("CMD,42,A\n"); assert(arm_stops == 1);
    reset(); feed("CMD,43,A\n"); fail_lift_angle = 1; arm_flags = 3; tick = 50; RobotControl_Tick();
    assert(lift_stops == 1); tick = 5000; RobotControl_Tick(); assert(servo_commands == 3);
    reset(); feed("CMD,44,A\n"); arm_flags = 3; tick = 50; RobotControl_Tick();
    lift_flags = 0x0D; tick = 100; RobotControl_Tick(); assert(lift_stops == 1);
    tick = 5000; RobotControl_Tick(); assert(servo_commands == 3);
    reset(); feed("CMD,45,A\n"); fail_arm_stop = 1; feed("CMD,46,Z\n");
    assert(arm_stops == 1); fail_arm_stop = 0; tick = 50; RobotControl_Tick(); assert(arm_stops == 2);
}
static void test_arm_distance_control(void)
{
    RobotArmDistanceCommand cmd;
    assert(RobotProtocol_ParseArmDistance("ARM_MOVE,1,E,127,10,3200", &cmd));
    assert(RobotProtocol_ArmDistanceAngle(&cmd) == 3600);
    const char *invalid[] = {"ARM_MOVE,0,E,127,10,3200", "ARM_MOVE,1,F,127,10,3200",
        "ARM_MOVE,1,E,1001,10,3200", "ARM_MOVE,1,E,127,121,3200", "ARM_MOVE,1,E,127,10,0",
        "ARM_MOVE,1,E,127,10,3200,", "ARM_MOVE,1,E,127,10,3200x"};
    for (unsigned i = 0; i < sizeof(invalid)/sizeof(invalid[0]); ++i)
        assert(!RobotProtocol_ParseArmDistance(invalid[i], &cmd));
    reset(); feed("CMD,9,E\nCMD,10,C\nARM,11,E,900,10,3200\n");
    assert(arm_angle_commands == 0 && arm_stops == 0);
    feed("ARM_MOVE,1,E,127,10,3200\n");
    assert(arm_angle_commands == 1 && last_arm_angle.angle_tenths == 3600);
    feed("ARM_MOVE,2,C,127,10,3200\n"); assert(arm_angle_commands == 1);
    tick = 1000; RobotControl_Tick(); assert(arm_stops == 0); /* No 300 ms hold timeout. */
    arm_flags = 3; tick += 50; RobotControl_Tick();
    feed("ARM_MOVE,1,E,127,10,3200\n"); assert(arm_angle_commands == 1);
    arm_flags = 1; feed("ARM_MOVE,3,C,10,10,6400\n");
    assert(arm_angle_commands == 2 && last_arm_angle.direction == 'C');
    feed("CMD,4,Q\n"); assert(arm_stops == 1);
    reset(); feed("ARM_MOVE,1,E,127,10,3200\n");
    fail_arm_status = 1; tick = 50; RobotControl_Tick(); assert(arm_stops == 0);
    tick = 499; RobotControl_Tick(); assert(arm_stops == 0);
    tick = 549; RobotControl_Tick(); assert(arm_stops == 1);
    reset(); feed("ARM_MOVE,1,E,127,10,3200\n");
    arm_flags = 0x0D; tick = 50; RobotControl_Tick(); assert(arm_stops == 1);
    reset(); feed("ARM_MOVE,1,E,127,10,3200\n");
    fail_arm_stop = 1; feed("CMD,2,Q\n");
    feed("ARM_MOVE,3,E,127,10,3200\n"); assert(arm_angle_commands == 1);
    fail_arm_stop = 0; RobotControl_Tick(); assert(arm_stops == 2);
    reset(); feed("ARM_MOVE,1,E,127,10,3200\n");
    tick = 180000; RobotControl_Tick(); assert(arm_stops == 1);
    reset(); fail_arm_angle = 1; feed("ARM_MOVE,1,E,127,10,3200\n");
    assert(arm_stops == 1); feed("ARM_MOVE,1,E,127,10,3200\n"); assert(arm_angle_commands == 1);
}
static void test_grab_status_reply_recovery(void)
{
    reset(); feed("CMD,60,A\n");
    fail_arm_status = 1; tick = 50; RobotControl_Tick(); tick = 200; RobotControl_Tick();
    assert(arm_stops == 0 && lift_angle_commands == 0 && servo_commands == 3);
    fail_arm_status = 0; arm_flags = 3; tick = 250; RobotControl_Tick(); assert(lift_angle_commands == 1);
    fail_lift_status = 1; tick = 300; RobotControl_Tick(); tick = 450; RobotControl_Tick();
    assert(lift_stops == 0 && servo_commands == 3);
    fail_lift_status = 0; lift_flags = 3; tick = 500; RobotControl_Tick();
    tick = 1499; RobotControl_Tick(); assert(servo_commands == 3);
    tick = 1500; RobotControl_Tick(); assert(absolute_target[0] == 4500 && absolute_target[1] == 0 && servo_commands == 6);
    reset(); feed("CMD,61,A\n"); fail_arm_status = 1; tick = 500; RobotControl_Tick();
    assert(arm_stops == 1); fail_arm_status = 0; arm_flags = 3; tick = 5000; RobotControl_Tick();
    assert(lift_angle_commands == 0 && servo_commands == 3);
    reset(); feed("CMD,62,A\n"); arm_flags = 3; tick = 50; RobotControl_Tick();
    fail_lift_status = 1; tick = 550; RobotControl_Tick(); assert(lift_stops == 1);
    fail_lift_status = 0; lift_flags = 3; tick = 5000; RobotControl_Tick(); assert(servo_commands == 3);
}
static void test_lift_calibration(void)
{
    RobotLiftAngleCommand command;
    assert(RobotProtocol_ParseLiftAngle("LIFT_ANGLE,1,U,90,5,3200", &command));
    const char *invalid[] = {"LIFT_ANGLE,0,U,90,5,3200", "LIFT_ANGLE,1,E,90,5,3200",
        "LIFT_ANGLE,1,U,0,5,3200", "LIFT_ANGLE,1,U,3601,5,3200",
        "LIFT_ANGLE,1,U,90,4,3200", "LIFT_ANGLE,1,U,90,121,3200",
        "LIFT_ANGLE,1,U,90,5,199", "LIFT_ANGLE,1,U,90,5,51201",
        "LIFT_ANGLE,1,U,90,5,3200,x", "LIFT_ANGLE,1,U,-90,5,3200"};
    for (unsigned i = 0; i < sizeof(invalid)/sizeof(invalid[0]); ++i)
        assert(!RobotProtocol_ParseLiftAngle(invalid[i], &command));
    reset(); feed("LIFT_ANGLE,1,U,90,5,3200\n");
    assert(lift_angle_commands == 1 && last_lift_angle.direction == 'U' &&
        last_lift_angle.angle == 900 && last_lift_angle.rpm == 5 && last_lift_angle.pulses == 3200);
    tick = 1000; RobotControl_Tick(); assert(lift_stops == 0); /* Autonomous: no 300 ms renewal. */
    feed("LIFT_ANGLE,2,D,180,10,6400\nCMD,3,U\nCMD,4,A\n");
    assert(lift_angle_commands == 1 && lift_velocity_commands == 0 && servo_commands == 0);
    lift_flags = 3; tick += 50; RobotControl_Tick();
    feed("LIFT_ANGLE,1,U,90,5,3200\n"); assert(lift_angle_commands == 1);
    lift_flags = 1; feed("LIFT_ANGLE,2,D,180,10,6400\n");
    assert(lift_angle_commands == 2 && last_lift_angle.direction == 'D' && last_lift_angle.angle == 1800);
    feed("CMD,5,H\n"); assert(lift_stops == 1);
    feed("CMD,6,U\n"); assert(lift_velocity_commands == 1);
    feed("LIFT_ANGLE,7,D,90,5,3200\n"); assert(lift_angle_commands == 2); /* Hold movement must stop first. */
    tick += 300; RobotControl_Tick(); assert(lift_stops == 2); /* Hold watchdog still works. */

    reset(); feed("LIFT_ANGLE,1,U,90,5,3200\n");
    fail_lift_status = 1; tick = 50; RobotControl_Tick(); assert(lift_stops == 0);
    fail_lift_status = 0; lift_flags = 3; tick = 100; RobotControl_Tick();
    assert(lift_stops == 0); feed("CMD,2,U\n"); assert(lift_velocity_commands == 1);
    reset(); feed("LIFT_ANGLE,1,U,90,5,3200\n");
    fail_lift_status = 1; tick = 500; RobotControl_Tick(); assert(lift_stops == 1);
    reset(); feed("LIFT_ANGLE,1,U,90,5,3200\n");
    lift_flags = 0x0D; tick = 50; RobotControl_Tick(); assert(lift_stops == 1);
    reset(); feed("LIFT_ANGLE,1,U,90,5,3200\n");
    fail_lift_stop = 1; feed("CMD,2,H\n");
    feed("LIFT_ANGLE,3,D,90,5,3200\n"); assert(lift_angle_commands == 1);
    fail_lift_stop = 0; RobotControl_Tick(); assert(lift_stops == 2);
    feed("LIFT_ANGLE,3,D,90,5,3200\n"); assert(lift_angle_commands == 2);
    reset(); feed("LIFT_ANGLE,1,U,90,5,3200\n");
    tick = 180000; RobotControl_Tick(); assert(lift_stops == 1);
    reset(); fail_lift_angle = 1; feed("LIFT_ANGLE,1,U,90,5,3200\n");
    feed("LIFT_ANGLE,1,U,90,5,3200\n"); assert(lift_angle_commands == 1 && lift_stops == 1);
    reset(); feed("CMD,1,A\nLIFT_ANGLE,2,U,90,5,3200\n"); assert(lift_angle_commands == 0);
}
static void test_lift_distance_control(void)
{
    RobotLiftDistanceCommand command;
    assert(RobotProtocol_ParseLiftDistance("LIFT_MOVE,1,U,40,5,3200", &command));
    assert(RobotProtocol_LiftDistanceAngle(&command) == 3600);
    command.distance_mm = 10; assert(RobotProtocol_LiftDistanceAngle(&command) == 900);
    command.distance_mm = 1; assert(RobotProtocol_LiftDistanceAngle(&command) == 90);
    command.distance_mm = 400; assert(RobotProtocol_LiftDistanceAngle(&command) == 36000);
    const char *invalid[] = {"LIFT_MOVE,0,U,10,5,3200", "LIFT_MOVE,1,E,10,5,3200",
        "LIFT_MOVE,1,U,0,5,3200", "LIFT_MOVE,1,U,401,5,3200",
        "LIFT_MOVE,1,U,10,4,3200", "LIFT_MOVE,1,U,10,121,3200",
        "LIFT_MOVE,1,U,10,5,199", "LIFT_MOVE,1,U,10,5,51201",
        "LIFT_MOVE,1,U,10,5,3200x", "LIFT_MOVE,1,U,-10,5,3200"};
    for (unsigned i = 0; i < sizeof(invalid)/sizeof(invalid[0]); ++i)
        assert(!RobotProtocol_ParseLiftDistance(invalid[i], &command));
    reset(); feed("LIFT_MOVE,1,U,40,5,3200\n");
    assert(lift_angle_commands == 1 && last_lift_angle.angle == 3600 && last_lift_angle.direction == 'U');
    feed("LIFT_MOVE,2,D,10,5,3200\nLIFT_ANGLE,3,D,90,5,3200\n");
    assert(lift_angle_commands == 1); /* Both formats use one axis state. */
    tick = 1000; RobotControl_Tick(); assert(lift_stops == 0);
    lift_flags = 3; tick += 50; RobotControl_Tick();
    feed("LIFT_MOVE,1,U,40,5,3200\n"); assert(lift_angle_commands == 1);
    lift_flags = 1; feed("LIFT_MOVE,2,D,10,5,6400\n");
    assert(lift_angle_commands == 2 && last_lift_angle.angle == 900 &&
        last_lift_angle.direction == 'D' && last_lift_angle.pulses == 6400);
    feed("CMD,4,H\n"); assert(lift_stops == 1);
    reset(); fail_lift_angle = 1; feed("LIFT_MOVE,1,U,10,5,3200\n");
    assert(lift_stops == 1);
    feed("LIFT_MOVE,1,U,10,5,3200\n"); assert(lift_angle_commands == 1);
}
static void camera_feed(const char *line)
{
    while (*line) {
        *camera_rx_byte = (uint8_t)*line++;
        camera_uart.RxState = HAL_UART_STATE_READY;
        HAL_UART_RxCpltCallback(&camera_uart);
    }
    RobotControl_Process();
}
static void camera_sample(const char *line)
{
    camera_feed("\n"); /* Resynchronize after deliberately discarded partial/old frames. */
    camera_feed(line); RobotControl_Tick();
}
static void camera_ready(void)
{
    camera_uart.RxState = HAL_UART_STATE_READY; CameraLink_Init(&camera_uart);
}
static void finish_alignment_move(void)
{
    memset(motor_status, 3, sizeof(motor_status));
    for (unsigned i = 0; i < 4; ++i) poll_without_keep(60);
    poll_without_keep(300);
    memset(motor_status, 1, sizeof(motor_status));
}
static uint32_t wheel_pulses(void)
{
    return ((uint32_t)frame[10] << 24) | ((uint32_t)frame[11] << 16) |
           ((uint32_t)frame[12] << 8) | frame[13];
}
static void test_ring_alignment(void)
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
        char command[64]; snprintf(command, sizeof(command), "ALIGN_RING,1,%u,10000,20000\n", ring);
        feed(command);
        camera_sample("x=400,y=160\n"); assert(position_frames == 0); /* Ignore block frames. */
        camera_sample("RINGS,100,160,266,160,400,160\n");
        assert(position_frames == 1 && wheel_pulses() == (ring == 1 ? 1702U : ring == 2 ? 109U : 1571U));
        camera_sample("RINGS,100,160,300,180,400,160\n");
        assert(position_frames == 1); /* Ignore newer coordinates while moving. */
        finish_alignment_move();
        assert(position_frames == 2 && wheel_pulses() == 100U && frame[6] == 1);
        feed("MOVE,2,F,100,45,9889\n"); assert(position_frames == 2);
        finish_alignment_move();
        camera_sample("RINGS,100,160,300,180,400,160\n"); assert(position_frames == 2);
        feed("MOVE,2,F,100,45,9889\n"); assert(position_frames == 3);
    }
    reset(); camera_ready(); feed("ALIGN_RING,1,2,10000,20000\n");
    camera_sample("RINGS,100,160,256,170,400,160\n");
    assert(position_frames == 1 && wheel_pulses() == 218U); /* Y uses lateral calibration. */
    feed("CMD,2,S\n"); camera_sample("RINGS,100,160,400,170,450,160\n");
    assert(position_frames == 1);
    reset(); camera_ready(); feed("ALIGN,1,10000,20000\n");
    camera_sample("RINGS,100,160,400,170,450,160\n"); assert(position_frames == 0);
    reset(); camera_ready(); feed("ALIGN_RING,1,2,10000,20000\n");
    poll_without_keep(3000);
    camera_sample("RINGS,100,160,400,170,450,160\n"); assert(position_frames == 0);
}
static void test_alignment_compensation_stop(void)
{
    for (unsigned mode = 0; mode < 2; ++mode) {
        const char *command = mode ? "ALIGN_RING,1,2,10000,20000\n" : "ALIGN,1,10000,20000\n";
        const char *sample = mode ? "RINGS,100,160,256,160,400,160\n" : "x=256,y=160\n";
        reset(); camera_ready(); feed(command);
        camera_sample(sample); assert(position_frames == 1 && wheel_pulses() == 100U);
        camera_sample(sample); assert(position_frames == 1);
        feed("CMD,2,S\n"); assert(stop_frames == 4);
        camera_sample(sample); assert(position_frames == 1);
        reset(); camera_ready(); feed(command);
        camera_sample(sample); camera_sample(sample);
        fail_status = 1; tick += 60; RobotControl_Tick(); assert(stop_frames == 4);
        camera_sample(sample); assert(position_frames == 1);
    }
}
static void test_camera_alignment(void)
{
    CameraCenter center; CameraAlignCommand cmd;
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
    camera_feed("x=400,y=160\n"); /* Discard stale coordinates at start. */
    feed("ALIGN,1,10000,20000\n"); RobotControl_Tick(); assert(position_frames == 0);
    camera_sample("x=266,y=160\n");
    assert(position_frames == 1 && wheel_pulses() == 109 && frame[6] == 1 && frame[8] == 10);
    camera_sample("x=256,y=180\n"); assert(position_frames == 1);
    feed("MOVE,2,F,100,45,9889\nCMD,3,A\n");
    assert(position_frames == 1 && servo_commands == 0);
    finish_alignment_move();
    assert(position_frames == 2 && wheel_pulses() == 100 && frame[8] == 10);
    camera_sample("x=400,y=180\n"); assert(position_frames == 2);
    finish_alignment_move();
    camera_sample("x=400,y=180\n"); assert(position_frames == 2); /* No resampling after arrival. */
    feed("ALIGN,1,10000,20000\n"); camera_sample("x=266,y=160\n"); assert(position_frames == 2);
    feed("ALIGN,4,10000,20000\n"); camera_sample("x=266,y=160\n"); assert(position_frames == 3); /* New sequence can run again. */

    reset(); camera_ready(); feed("ALIGN_CFG,1,0,10000,10000,10,20,0\n");
    camera_sample("x=356,y=210\n");
    assert(position_frames == 1 && wheel_pulses() == 1636 && frame[8] == 20);
    assert(frame[21] == 7 && frame[23] == 0 && frame[24] == 0 && frame[25] == 2 && frame[26] == 34);
    finish_alignment_move(); camera_sample("x=400,y=180\n"); assert(position_frames == 1);
    feed("MOVE,2,F,100,45,9889\n"); assert(position_frames == 2); /* Zero offset completes immediately on arrival. */
    reset(); camera_ready(); feed("ALIGN,1,9889,9889\n");
    tick = 3000; RobotControl_Tick(); camera_sample("x=400,y=160\n"); assert(position_frames == 0);
    feed("MOVE,2,F,100,45,9889\n"); assert(position_frames == 1);
    reset(); camera_ready(); feed("ALIGN,1,9889,9889\n"); camera_sample("x=266,y=160\n");
    feed("CMD,2,S\n"); assert(stop_frames == 4); camera_sample("x=400,y=160\n"); assert(position_frames == 1);
    reset(); camera_ready(); feed("ALIGN,1,9889,9889\n"); camera_sample("x=266,y=160\n");
    fail_status = 1; tick += 60; RobotControl_Tick(); assert(stop_frames == 4);
    camera_sample("x=400,y=160\n"); assert(position_frames == 1);
    reset(); camera_ready(); feed("ALIGN,1,9889,9889\n"); camera_sample("x=266,y=160\n");
    tick = 120000; RobotControl_Tick(); assert(stop_frames == 4);
    camera_sample("x=400,y=160\n"); assert(position_frames == 1);
    reset(); camera_ready();
    camera_feed("x=2"); camera_feed("56,y=160\n"); assert(CameraLink_TakeCenter(&center) && center.x == 256);
    camera_feed("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx\n"); assert(!CameraLink_TakeCenter(&center));
    camera_feed("x=511,y=319\n"); assert(CameraLink_TakeCenter(&center) && center.x == 511);
    camera_uart.RxState = HAL_UART_STATE_READY; HAL_UART_ErrorCallback(&camera_uart);
    RobotControl_Process(); camera_sample("x=256,y=160\n");
    assert(CameraLink_TakeCenter(&center) && center.y == 160);
}
static void finish_z_first_axes(void)
{
    lift_flags = 3; tick += 50; RobotControl_Tick();
    arm_flags = 3; tick += 50; RobotControl_Tick();
    lift_flags = arm_flags = 1;
}
static void release_to_pickup(void)
{
    reset(); feed("CMD,80,P\n"); assert(absolute_target[1] == 0 && servo_angle == 248);
    finish_grab_axes(); advance_grab_hold();
    tick += 1000; RobotControl_Tick(); assert(servo_angle == 194);
    tick += 1000; RobotControl_Tick(); assert(servo_angle == 140);
    advance_grab_hold(); assert(absolute_target[0] == 3600 && servo_history[servo_commands-2] == 60 && last_lift_angle.rpm == 50);
}
static void test_release_sequence(void)
{
    release_to_pickup(); unsigned r_count = absolute_commands[1], servos = servo_commands;
    feed("CMD,81,A\nCMD,82,P\nARM_POSE,83,0,0,0,5,3200\n");
    assert(servo_commands == servos && absolute_commands[1] == r_count);
    tick += 50; RobotControl_Tick(); assert(absolute_commands[1] == r_count && servo_commands == servos);
    lift_flags = 3; tick += 50; RobotControl_Tick(); assert(absolute_target[1] == 567 && servo_commands == servos);
    arm_flags = 3; tick += 50; RobotControl_Tick(); assert(servo_channel == 'G' && servo_angle == 0);
    lift_flags = arm_flags = 1;
    advance_grab_hold(); assert(absolute_target[0] == 0 && absolute_target[1] == 567); /* Step 4 z first. */
    finish_z_first_axes(); assert(absolute_target[1] == 0);
    advance_grab_hold(); tick += 1000; RobotControl_Tick(); assert(servo_angle == 194);
    tick += 1000; RobotControl_Tick(); assert(servo_angle == 248);
    advance_grab_hold(); assert(absolute_target[1] == 3685 && absolute_target[0] == 0); /* Step 6: r=130 mm first. */
    tick += 50; RobotControl_Tick(); assert(absolute_target[0] == 0); /* Wait for r arrival before lowering. */
    arm_flags = 3; tick += 50; RobotControl_Tick(); assert(absolute_target[0] == 9900 && last_lift_angle.rpm == 50); /* z=-110 mm. */
    lift_flags = 3; tick += 50; RobotControl_Tick(); arm_flags = lift_flags = 1;
    advance_grab_hold(); assert(servo_history[servo_commands-2] == 0); /* Step 7 starts slow release. */
    finish_gripper_open();
    advance_grab_hold(); assert(absolute_target[1] == 0); /* Step 8 r first. */
    finish_grab_axes(); assert(absolute_target[0] == 0 && last_lift_angle.rpm == 20);
    servos = servo_commands; advance_grab_hold(); assert(servo_commands == servos); /* No final +120. */
    feed("CMD,80,P\n"); assert(servo_commands == servos);
    finish_grab_cycle(84, 120); /* Release does not advance the grab rotation counter. */
    release_to_pickup(); feed("CMD,85,Z\n"); assert(lift_stops == 1);
    servos = servo_commands; lift_flags = 3; tick += 1000; RobotControl_Tick();
    assert(absolute_commands[1] == 1 && servo_commands == servos);
    release_to_pickup(); lift_flags = 0x0D; tick += 50; RobotControl_Tick();
    assert(lift_stops == 1 && absolute_commands[1] == 1); /* Fault cannot extend or close. */
    release_to_pickup(); lift_flags = 3; tick += 50; RobotControl_Tick();
    feed("SERVO,86,G,90\n"); assert(arm_stops == 1 && servo_angle == 90);
}
static void test_configured_sequences(void)
{
    RobotSequenceCommand config;
    assert(RobotProtocol_ParseSequence("STATE,1,A,1155,-55,250,-45,25,15,55", &config));
    assert(config.r1 == 1155 && config.down_rpm == 55 && config.gripper_dps == 60);
    assert(RobotProtocol_ParseSequence("STATE,1,P,250,-45,1350,-115,26,16,56,30", &config));
    assert(config.gripper_dps == 30);
    assert(!RobotProtocol_ParseSequence("STATE,1,P,250,-45,1350,-115,26,16,56,0", &config));
    const char *bad[] = {
        "STATE,0,A,1100,-50,200,-40,20,20,50",
        "STATE,1,X,1100,-50,200,-40,20,20,50",
        "STATE,1,A,10001,-50,200,-40,20,20,50",
        "STATE,1,A,1100,-401,200,-40,20,20,50",
        "STATE,1,A,1100,-50,200,-40,20,20,121",
        "STATE,1,A,1100,-50,200,-40,20,20,50,junk"
    };
    for (unsigned i = 0; i < sizeof(bad)/sizeof(bad[0]); ++i)
        assert(!RobotProtocol_ParseSequence(bad[i], &config));
    reset(); feed("STATE,300,A,1155,-55,250,-45,25,15,55\n");
    assert(last_arm_angle.speed_rpm == 25);
    finish_grab_axes(); assert(last_lift_angle.rpm == 15);
    advance_grab_hold(); assert(absolute_target[0] == 4950 && last_lift_angle.rpm == 55);
    unsigned servos = servo_commands;
    feed("STATE,301,P,100,-10,100,-10,10,10,10\n"); assert(servo_commands == servos); /* Busy cannot change settings. */
    finish_z_first_axes(); assert(absolute_target[1] == 3274 && last_arm_angle.speed_rpm == 25);
    advance_grab_hold(); advance_grab_hold(); finish_grab_axes(); assert(last_lift_angle.rpm == 15);
    advance_grab_hold(); tick += 2000; RobotControl_Tick();
    advance_grab_hold(); assert(absolute_target[1] == 709);
    finish_grab_axes(); assert(absolute_target[0] == 4050 && last_lift_angle.rpm == 55);
    feed("CMD,302,Z\n");
    reset(); feed("STATE,400,P,250,-45,1350,-115,26,16,56,30\n");
    finish_grab_axes(); advance_grab_hold(); tick += 2000; RobotControl_Tick();
    advance_grab_hold(); assert(absolute_target[0] == 4050 && last_lift_angle.rpm == 56);
    finish_z_first_axes(); assert(absolute_target[1] == 709 && last_arm_angle.speed_rpm == 26);
    advance_grab_hold(); assert(last_lift_angle.rpm == 16); finish_z_first_axes();
    advance_grab_hold(); tick += 2000; RobotControl_Tick(); advance_grab_hold();
    assert(absolute_target[1] == 3827 && last_arm_angle.speed_rpm == 26);
    finish_grab_axes(); assert(absolute_target[0] == 10350 && last_lift_angle.rpm == 56);
    advance_grab_hold(); unsigned r_count = absolute_commands[1];
    tick += 1000; RobotControl_Tick(); assert(servo_angle == 30 && absolute_commands[1] == r_count);
    tick += 1000; RobotControl_Tick(); assert(servo_angle == 60 && absolute_commands[1] == r_count);
    advance_grab_hold(); finish_grab_axes(); advance_grab_hold();
    servos = servo_commands; feed("STATE,400,P,250,-45,1350,-115,26,16,56,30\n");
    assert(servo_commands == servos); /* Duplicate cannot replay. */
    feed("CMD,401,P\n"); assert(last_arm_angle.speed_rpm == 20); /* Legacy command uses defaults. */
    reset(); feed("STATE,500,A,1100,-50,200,-40,20,20,121\n"); assert(absolute_commands[1] == 0);
}
static void test_configured_alignment(void)
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
        feed(mode ? "ALIGN_CFG,700,2,10000,20000,12,30,-7\n" :
                    "ALIGN_CFG,700,0,10000,20000,12,30,-7\n");
        camera_sample(mode ? "RINGS,100,160,296,160,400,160\n" : "x=296,y=160\n");
        assert(position_frames == 1 && wheel_pulses() == 436U && frame[8] == 30);
        finish_alignment_move();
        assert(position_frames == 2 && wheel_pulses() == 70U && frame[8] == 12 && frame[6] == 0);
        const char *center = mode ? "RINGS,100,160,266,160,400,160\n" : "x=266,y=160\n";
        camera_sample(center); assert(position_frames == 2);
        finish_alignment_move(); camera_sample(center); assert(position_frames == 2);
        feed("ALIGN,701,10000,20000\n"); camera_sample("x=296,y=160\n");
        assert(position_frames == 3 && wheel_pulses() == 436 && frame[8] == 20); /* Legacy resets speed defaults. */
    }
    reset(); camera_ready(); feed("ALIGN_CFG,702,0,10000,20000,12,30,0\n");
    camera_sample("x=256,y=160\n"); camera_sample("x=256,y=160\n"); assert(position_frames == 0);
    feed("MOVE,703,F,100,45,9889\n"); assert(position_frames == 1); /* Zero offset finishes without moving. */
}
static void test_manual_gripper(void)
{
    RobotGripperCommand cmd;
    assert(RobotProtocol_ParseGripper("GRIP,1,0,60,30", &cmd));
    assert(!RobotProtocol_ParseGripper("GRIP,1,0,271,30", &cmd));
    assert(!RobotProtocol_ParseGripper("GRIP,1,0,60,0", &cmd));
    reset(); feed("GRIP,800,0,60,30\n"); assert(servo_angle == 0);
    tick += 1000; RobotControl_Tick(); assert(servo_angle == 30);
    feed("CMD,801,A\n"); assert(absolute_commands[1] == 0);
    tick += 1000; RobotControl_Tick(); assert(servo_angle == 60);
    unsigned servos = servo_commands; feed("GRIP,800,0,60,30\n"); assert(servo_commands == servos);
    feed("GRIP,802,60,0,60\n"); tick += 500; RobotControl_Tick(); assert(servo_angle == 30);
    feed("GRIP_STOP,803\n"); servos = servo_commands;
    tick += 1000; RobotControl_Tick(); assert(servo_commands == servos && servo_angle == 30);
    feed("GRIP,804,10,10,30\n"); assert(servo_angle == 10);
    servos = servo_commands; tick += 1000; RobotControl_Tick(); assert(servo_commands == servos);
    feed("GRIP,805,0,60,30\nSERVO,806,G,15\n");
    tick += 2000; RobotControl_Tick(); assert(servo_angle == 15);
    feed("GRIP,807,0,60,30\nCMD,808,Z\n");
    servos = servo_commands; tick += 2000; RobotControl_Tick(); assert(servo_commands == servos);
    reset(); feed("ARM_POSE,809,0,10,-10,120,3200\n"); assert(last_arm_angle.speed_rpm == 120);
    finish_grab_axes(); assert(last_lift_angle.rpm == 120);
}
static void test_sequence_angles(void)
{
    RobotSequenceCommand cmd;
    assert(RobotProtocol_ParseSequence("STATE,900,P,200,-40,1300,-110,20,20,50,40,30,100,20,250,130", &cmd));
    assert(cmd.theta == 30 && cmd.base_home == 250 && cmd.open_angle == 100);
    assert(!RobotProtocol_ParseSequence("STATE,900,P,200,-40,1300,-110,20,20,50,40,271,100,20,250,130", &cmd));
    reset(); feed("STATE,900,P,200,-40,1300,-110,20,20,50,40,30,100,20,250,130\n");
    assert(servo_history[0] == 30 && servo_history[1] == 100 && servo_angle == 250);
    finish_grab_axes(); advance_grab_hold(); tick += 1000; RobotControl_Tick(); assert(servo_angle == 190);
    tick += 1000; RobotControl_Tick(); assert(servo_angle == 130);
    advance_grab_hold(); assert(servo_history[servo_commands-2] == 100);
    finish_z_first_axes(); assert(servo_angle == 20 && servo_channel == 'G');
    advance_grab_hold(); finish_z_first_axes(); advance_grab_hold();
    tick += 2000; RobotControl_Tick(); assert(servo_angle == 250);
    advance_grab_hold(); finish_grab_axes(); advance_grab_hold();
    tick += 1000; RobotControl_Tick(); assert(servo_angle == 60);
    tick += 1000; RobotControl_Tick(); assert(servo_angle == 100);
    advance_grab_hold(); finish_grab_axes(); advance_grab_hold();
    assert(servo_history[servo_commands-3] == 30);
}
static void test_parallel_alignment(void)
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
        feed(reverse ? "PARALLEL,901,10000,10,3,1\n" : "PARALLEL,901,10000,10,3,0\n");
        camera_sample("x=256,y=160\n"); assert(position_frames == 0);
        camera_sample("RINGS,100,100,200,150,300,200\n");
        assert(position_frames == 1 && wheel_pulses() == 30U && frame[8] == 10);
        assert(frame[6] == (reverse ? 1 : 0));
        camera_sample("RINGS,100,150,200,150,300,150\n"); assert(position_frames == 1);
        finish_alignment_move();
        camera_sample("RINGS,100,150,200,150,300,150\n");
        camera_sample("RINGS,100,150,200,150,300,150\n");
        feed("MOVE,902,F,100,45,9889\n"); assert(position_frames == 1);
        camera_sample("RINGS,100,150,200,150,300,150\n");
        feed("MOVE,902,F,100,45,9889\n"); assert(position_frames == 2); /* No positional compensation. */
    }
    reset(); camera_ready(); feed("PARALLEL,903,10000,10,3,0\n");
    camera_sample("RINGS,100,200,200,150,300,100\n"); assert(frame[6] == 1);
    feed("CMD,904,S\n"); assert(stop_frames == 4);
    camera_sample("RINGS,100,200,200,150,300,100\n"); assert(position_frames == 1);
    reset(); camera_ready(); feed("PARALLEL,905,10000,10,3,0\n");
    camera_sample("RINGS,100,100,200,220,300,200\n"); assert(position_frames == 0);
    poll_without_keep(3000);
    camera_sample("RINGS,100,100,200,150,300,200\n"); assert(position_frames == 0);
}
static void test_absolute_pose(void)
{
    RobotArmPoseCommand cmd;
    assert(RobotProtocol_ParseArmPose("ARM_POSE,1,120,-20,40,5,3200", &cmd));
    assert(cmd.r_mm == -20 && cmd.z_mm == 40 && cmd.theta == 120);
    const char *bad[] = {"ARM_POSE,0,0,0,0,5,3200", "ARM_POSE,1,271,0,0,5,3200",
        "ARM_POSE,1,0,-1001,0,5,3200", "ARM_POSE,1,0,0,401,5,3200",
        "ARM_POSE,1,0,+20,0,5,3200", "ARM_POSE,1,0,0,0,5,3200junk"};
    for (unsigned i = 0; i < sizeof(bad)/sizeof(bad[0]); ++i) assert(!RobotProtocol_ParseArmPose(bad[i], &cmd));
    reset(); origin_position[1] = 7200; origin_position[0] = -3600;
    RobotControl_Init(&command_uart, true); feed("CMD,4294967295,I\n"); servo_commands = 0;
    feed("ARM_POSE,1,120,127,40,5,3200\n");
    assert(absolute_target[1] == 10800 && absolute_commands[1] == 1);
    feed("ARM_POSE,2,0,0,0,5,3200\nCMD,3,A\n"); assert(absolute_commands[1] == 1);
    arm_flags = 3; tick = 50; RobotControl_Tick();
    assert(absolute_target[0] == -7200 && servo_commands == 0);
    lift_flags = 3; tick = 100; RobotControl_Tick(); assert(servo_commands == 1 && servo_angle == 120);
    /* Same target, new seq: same driver absolute position, not another 127 mm. */
    feed("ARM_POSE,4,120,127,40,5,3200\n"); assert(absolute_target[1] == 10800);
    feed("CMD,5,Z\n"); assert(arm_stops == 1);
    origin_position[1] = 99999; origin_position[0] = 99999; /* Stopping never recaptures origin. */
    feed("CMD,6,O\n"); assert(absolute_target[1] == 7200);
    tick = 150; RobotControl_Tick(); assert(absolute_target[0] == -3600);
    tick = 200; RobotControl_Tick();
    assert(servo_history[servo_commands-3] == 0 && servo_history[servo_commands-2] == 15 && servo_angle == 248);
    unsigned count = absolute_commands[1]; feed("CMD,6,O\n"); assert(absolute_commands[1] == count);
    feed("ARM_POSE,7,60,10,10,5,3200\n"); tick = 250; RobotControl_Tick();
    lift_flags = 0; tick = 300; unsigned servos = servo_commands; RobotControl_Tick();
    assert(servo_commands == servos); /* Lift fault must not execute theta/home pose. */
    reset(); fail_origin_read = 1; RobotControl_Init(&command_uart, true); feed("CMD,4294967295,I\n"); servo_commands = 0;
    feed("ARM_POSE,1,0,10,10,5,3200\nCMD,2,A\nLIFT_MOVE,3,U,10,5,3200\n");
    assert(arm_angle_commands == 0 && lift_angle_commands == 0 && servo_commands == 0);
    fail_origin_read = 0; origin_position[1] = 3600; tick = 500; RobotControl_Tick();
    feed("ARM_POSE,4,0,127,0,5,3200\n"); assert(absolute_target[1] == 7200);
    reset(); feed("ARM_POSE,1,0,10,10,5,3200\nCMD,2,H\n");
    assert(arm_stops == 1); tick = 50; RobotControl_Tick(); assert(lift_angle_commands == 0);
}
static void test_app_origin_initialization(void)
{
    reset(); origin_reads = 0; RobotControl_Init(&command_uart, true);
    tick = 1000; RobotControl_Tick();
    assert(servo_commands == 0 && origin_reads == 0); /* No startup pose or implicit retry. */
    feed("CMD,1,A\nARM_POSE,2,0,10,10,5,3200\nCMD,3,O\n");
    assert(arm_angle_commands == 0 && lift_angle_commands == 0 && servo_commands == 0);
    origin_position[1] = 7200; origin_position[0] = -3600;
    feed("CMD,4,I\n"); assert(origin_reads == 2 && servo_commands == 3 && servo_angle == 248);
    origin_position[1] = 123456; feed("CMD,5,I\n");
    assert(origin_reads == 2 && servo_commands == 3); /* Re-initialization cannot silently shift zero. */
    feed("ARM_POSE,6,0,127,40,5,3200\n"); assert(absolute_target[1] == 10800);
    reset(); origin_reads = 0; RobotControl_Init(&command_uart, true); fail_origin_read = 1;
    feed("CMD,1,I\n"); unsigned reads = origin_reads;
    feed("CMD,2,Z\n"); fail_origin_read = 0; tick = 1000; RobotControl_Tick(); assert(origin_reads == reads);
    feed("CMD,3,I\n"); assert(origin_reads == reads+2);
}
static void upload_test_plan(uint32_t seq)
{
    char text[128];
    snprintf(text, sizeof(text), "PLAN_BEGIN,%lu,2\n", (unsigned long)seq); feed(text);
    snprintf(text, sizeof(text), "PLAN_ITEM,%lu,0,A,0,0,0,0,21,22,23,60,10,71,11,250,140\n", (unsigned long)seq); feed(text);
    snprintf(text, sizeof(text), "PLAN_ITEM,%lu,1,P,50,-5,100,-10,27,28,29,60,20,81,12,260,150\n", (unsigned long)seq); feed(text);
    snprintf(text, sizeof(text), "PLAN_RUN,%lu\n", (unsigned long)seq); feed(text);
}
static void test_saved_plan_and_custom_origin(void)
{
    RobotOriginCommand origin;
    RobotPlanCommand plan;
    RobotSequenceCommand item;
    assert(RobotProtocol_ParseOrigin("ORIGIN,9,251", &origin) && origin.base == 251U);
    assert(!RobotProtocol_ParseOrigin("ORIGIN,9,361", &origin));
    assert(!RobotProtocol_ParseOrigin("ORIGIN,0,251", &origin));
    assert(RobotProtocol_ParsePlan("PLAN_BEGIN,9,16", &plan) && plan.count == 16U);
    assert(!RobotProtocol_ParsePlan("PLAN_BEGIN,9,17", &plan));
    assert(!RobotProtocol_ParsePlan("PLAN_BEGIN,9,0", &plan));
    assert(RobotProtocol_ParseSequence("PLAN_ITEM,9,15,A,0,0,0,0,20,20,50", &item) && item.plan_index == 15U);
    assert(!RobotProtocol_ParseSequence("PLAN_ITEM,9,16,A,0,0,0,0,20,20,50", &item));
    reset(); RobotControl_Init(&command_uart, true); servo_commands = 0;
    feed("ORIGIN,9,251\n"); assert(servo_angle == 251);
    feed("CMD,10,O\n"); finish_grab_axes(); assert(servo_angle == 251); // Home restores selected base.

    reset(); feed("PLAN_BEGIN,9,2\nPLAN_ITEM,9,0,A,0,0,0,0,20,20,50\nPLAN_RUN,9\n");
    assert(absolute_commands[1] == 0); // No partial plan may run.
    feed("PLAN_ITEM,10,1,P,0,0,0,0,20,20,50\nPLAN_RUN,9\n"); assert(absolute_commands[1] == 0);
    reset(); upload_test_plan(20); assert(absolute_commands[1] == 1 && last_arm_angle.speed_rpm == 21);
    feed("PLAN_BEGIN,21,1\nSTATE,22,P,0,0,0,0,40,40,40\n"); // Busy upload cannot mutate active plan.
    arm_flags = lift_flags = 3;
    for (unsigned i = 0; i < 1000 && !(servo_channel == 'T' && servo_angle == 120); ++i) { tick += 50; RobotControl_Tick(); }
    assert(absolute_commands[1] == 5 && servo_channel == 'T' && servo_angle == 120);
    tick += 999; RobotControl_Tick(); assert(absolute_commands[1] == 5);
    tick += 1; RobotControl_Tick();
    assert(absolute_commands[1] == 6 && last_arm_angle.speed_rpm == 27);
    assert(servo_angle == 260 && servo_history[servo_commands-2] == 81); // Second card starts only after first completes.
    for (unsigned i = 0; i < 1000; ++i) { tick += 50; RobotControl_Tick(); }
    unsigned moves = absolute_commands[1], servos = servo_commands;
    feed("PLAN_RUN,20\n"); assert(absolute_commands[1] == moves && servo_commands == servos);
    feed("STATE,30,A,0,0,0,0,31,32,33\n"); assert(absolute_commands[1] == moves+1 && last_arm_angle.speed_rpm == 31);

    reset(); upload_test_plan(40); feed("CMD,41,Z\n");
    arm_flags = lift_flags = 3;
    for (unsigned i = 0; i < 1000; ++i) { tick += 50; RobotControl_Tick(); }
    assert(absolute_commands[1] == 1); // Cancel clears successors and uploaded frames.
    feed("PLAN_RUN,40\n"); assert(absolute_commands[1] == 1);
    reset(); upload_test_plan(50); arm_flags = 0x0D; tick += 50; RobotControl_Tick();
    arm_flags = lift_flags = 3;
    for (unsigned i = 0; i < 1000; ++i) { tick += 50; RobotControl_Tick(); }
    assert(absolute_commands[1] == 1); // Motor fault must never advance to next card.
}
static void test_manual_base_speed(void)
{
    RobotBaseCommand cmd;
    assert(RobotProtocol_ParseBase("BASE,1,360,1", &cmd) && cmd.target == 360U && cmd.dps == 1U);
    assert(RobotProtocol_ParseBase("BASE,2,0,360", &cmd));
    const char *invalid[] = {"BASE,0,90,60", "BASE,1,-1,60", "BASE,1,361,60", "BASE,1,90,0",
        "BASE,1,90,361", "BASE,1,90,60,", "BASE,1,90,60x"};
    for (unsigned i = 0; i < sizeof(invalid)/sizeof(invalid[0]); ++i) assert(!RobotProtocol_ParseBase(invalid[i], &cmd));
    reset(); feed("BASE,100,308,30\n"); assert(servo_commands == 0);
    feed("STATE,101,A,0,0,0,0,20,20,50\nCMD,102,O\n"); assert(absolute_commands[1] == 0);
    tick += 1000; RobotControl_Tick(); assert(servo_channel == 'B' && servo_angle == 278);
    feed("BASE,100,308,30\n"); // Duplicate must not restart ramp.
    tick += 1000; RobotControl_Tick(); assert(servo_angle == 308);
    feed("BASE,103,248,60\n"); tick += 500; RobotControl_Tick(); assert(servo_angle == 278);
    feed("BASE,104,218,60\n"); tick += 500; RobotControl_Tick(); assert(servo_angle == 248);
    tick += 500; RobotControl_Tick(); assert(servo_angle == 218);
    unsigned servos = servo_commands; feed("BASE,105,218,60\n"); RobotControl_Tick(); assert(servo_commands == servos);
    feed("BASE,106,360,1\n"); tick += 10000; RobotControl_Tick(); assert(servo_angle == 228);
    feed("CMD,107,Z\n"); servos = servo_commands; tick += 200000; RobotControl_Tick(); assert(servo_commands == servos);
    feed("SERVO,108,B,100\nBASE,109,160,30\n"); tick += 1000; RobotControl_Tick(); assert(servo_angle == 130);
    feed("SERVO,110,B,80\n"); tick += 10000; RobotControl_Tick(); assert(servo_angle == 80);
    feed("BASE,111,140,30\n"); fail_servo = 1; tick += 1000; RobotControl_Tick();
    fail_servo = 0; servos = servo_commands; tick += 10000; RobotControl_Tick(); assert(servo_commands == servos);
    reset(); RobotControl_Init(&command_uart, true); servo_commands = 0; // Unknown start: first target is direct.
    feed("BASE,112,100,30\n"); assert(servo_angle == 100 && servo_commands == 1);
    feed("BASE,113,160,30\n"); tick += 1000; RobotControl_Tick(); assert(servo_angle == 130);
    reset(); feed("CMD,114,A\nBASE,115,200,30\n"); assert(arm_stops == 1);
    tick += 1000; RobotControl_Tick(); assert(servo_angle == 218); // Automatic base target tracked at 248.
}
static void test_parallel_pixel_tolerance(void)
{
    const CameraCenter near[] = {{100,150},{200,151},{300,153}};
    int32_t slope;
    assert(CameraProtocol_RingSlope(near, &slope) && slope == 15);
    assert(CameraProtocol_RingYSpread(near) == 3U);
    for (unsigned reverse = 0; reverse < 2U; ++reverse) {
        reset(); camera_ready();
        feed(reverse ? "PARALLEL,1100,10000,20,3,1\n" : "PARALLEL,1100,10000,20,3,0\n");
        camera_sample("RINGS,100,150,200,151,300,153\n");
        assert(position_frames == 1 && wheel_pulses() == 2U && frame[8] == 5U);
        assert(frame[6] == (reverse ? 1U : 0U));
        finish_alignment_move();
        camera_sample("RINGS,100,150,200,151,300,151\n");
        camera_sample("RINGS,100,150,200,151,300,151\n");
        camera_sample("RINGS,100,153,200,151,300,150\n"); // Non-horizontal resets completion count.
        assert(position_frames == 2 && wheel_pulses() == 2U && frame[6] == (reverse ? 0U : 1U));
        finish_alignment_move();
        camera_sample("RINGS,100,150,200,151,300,151\n");
        camera_sample("RINGS,100,150,200,151,300,151\n");
        feed("MOVE,1101,F,100,45,9889\n"); assert(position_frames == 2);
        camera_sample("RINGS,100,150,200,151,300,151\n");
        feed("MOVE,1101,F,100,45,9889\n"); assert(position_frames == 3);
    }
    reset(); camera_ready(); feed("PARALLEL,1102,9889,10,3,0\n");
    camera_sample("RINGS,100,150,200,153,300,150\n"); assert(position_frames == 0); // Zero slope with noise must not turn.
    feed("MOVE,1103,F,100,45,9889\n"); assert(position_frames == 0); // Nor may it complete immediately.
    for (unsigned i = 0; i < 3U; ++i) camera_sample("RINGS,100,150,200,150,300,150\n");
    feed("MOVE,1103,F,100,45,9889\n"); assert(position_frames == 1);
    reset(); camera_ready(); feed("PARALLEL,1104,1,5,1,0\n");
    camera_sample("RINGS,100,150,200,151,300,153\n"); assert(wheel_pulses() == 1U); // Never round fine correction down to zero.
}
static void test_initial_base_angle_updates(void)
{
    reset(); origin_position[1] = 7200; origin_position[0] = -3600;
    RobotControl_Init(&command_uart, true); origin_reads = 0;
    feed("ORIGIN,1200,251\n"); assert(origin_reads == 2);
    feed("SERVO,1201,T,120\nSERVO,1202,G,42\n");
    origin_position[0] = origin_position[1] = 99999;
    unsigned servos = servo_commands;
    feed("ORIGIN,1203,200\n");
    assert(origin_reads == 2 && servo_commands == servos+1 && servo_channel == 'B' && servo_angle == 200);
    feed("ORIGIN,1203,100\n"); assert(servo_commands == servos+1); // Duplicate cannot change angle.
    feed("CMD,1204,O\n"); assert(absolute_target[1] == 7200);
    finish_grab_axes(); assert(absolute_target[0] == -3600 && servo_angle == 200);
    feed("CMD,1205,A\nORIGIN,1206,180\n"); assert(arm_stops == 1 && servo_angle == 180);
    assert(origin_reads == 2); // Updating during a sequence cancels it without recapturing zero.
    tick += 10000; RobotControl_Tick();
    feed("CMD,1207,O\n"); finish_grab_axes(); assert(servo_angle == 180 && absolute_target[1] == 7200);
    fail_servo = 1; feed("ORIGIN,1208,100\n"); fail_servo = 0;
    feed("CMD,1209,O\n"); finish_grab_axes(); assert(servo_angle == 180); // Failed PWM update must not replace home angle.
    feed("ORIGIN,1208,100\n"); assert(servo_angle == 100 && origin_reads == 2);
    reset(); RobotControl_Init(&command_uart, true); fail_origin_read = 1;
    feed("ORIGIN,1210,240\n");
    unsigned reads = origin_reads;
    feed("ORIGIN,1211,220\n"); assert(servo_angle == 220 && origin_reads == reads);
    fail_origin_read = 0; tick += 500; RobotControl_Tick();
    feed("CMD,1212,O\n"); finish_grab_axes(); assert(servo_angle == 220);
}
int main(void)
{
    test_parser(); test_direction_frames(); test_completion_and_duplicates(); test_timeout_stop_fault();
    test_stream_recovery();
    test_receive_restart();
    test_servo_commands_without_replies();
    test_grab_state();
    test_gripper_open_cancel();
    test_arm_distance_control();
    test_grab_status_reply_recovery();
    test_lift_calibration();
    test_lift_distance_control();
    test_camera_alignment();
    test_ring_alignment();
    test_alignment_compensation_stop();
    test_absolute_pose();
    test_parallel_alignment();
    test_parallel_pixel_tolerance();
    test_sequence_angles();
    test_manual_gripper();
    test_configured_alignment();
    test_configured_sequences();
    test_release_sequence();
    test_app_origin_initialization();
    test_saved_plan_and_custom_origin();
    test_manual_base_speed();
    test_initial_base_angle_updates();
    puts("PASS: parsing, four-wheel FD frames, completion, autonomous move, grab/arm, camera alignment limits, duplicate/busy, fault and stop retry");
    return 0;
}
