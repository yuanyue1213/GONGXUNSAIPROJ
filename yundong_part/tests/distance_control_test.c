#include <assert.h>
#include "imu_link.h"
#include "gyro_turn.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include "stm32h7xx_hal.h"
#include "zdt_motor.h"
#include "lift_motor.h"
#include "robot_control.h"
#include "camera_link.h"
#include "../shared/robot_distance_protocol.h"
#include "../shared/camera_pose_math.h"
#include "../shared/arm_vector_math.h"
#include "../shared/motor_status_guard.h"

static UART_HandleTypeDef motor_uart = {1, HAL_UART_STATE_READY};
static UART_HandleTypeDef command_uart = {3, HAL_UART_STATE_READY};
static UART_HandleTypeDef camera_uart = {4, HAL_UART_STATE_READY};
static uint32_t tick;
static uint8_t *rx_byte;
static uint8_t *camera_rx_byte;
static uint8_t reply[8], reply_index,reply_length=4;
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
static char application_reply[2048], camera_request[128];
static UART_HandleTypeDef imu_uart={7,HAL_UART_STATE_READY};
static uint8_t *imu_rx_byte;
static unsigned velocity_frames;
static uint8_t velocity_frame[37];
static int simulated_rpm;
static double wheel_encoder[4];
static int wheel_velocity[4],fail_encoder;

uint32_t HAL_GetTick(void) { return tick; }
HAL_StatusTypeDef ImuLink_SetBaud(UART_HandleTypeDef *uart,uint32_t baud)
{
    (void)baud; uart->RxState=HAL_UART_STATE_READY; return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *uart, uint8_t *data, uint16_t length)
{
    assert(length == 1);
    if (uart == &imu_uart) {
        if (uart->RxState == HAL_UART_STATE_BUSY_RX) return HAL_BUSY;
        uart->RxState = HAL_UART_STATE_BUSY_RX; imu_rx_byte=data; return HAL_OK;
    }
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
    if(uart==&command_uart) {
        size_t n=strlen(application_reply);
        assert(n+length<sizeof(application_reply));
        memcpy(application_reply+n,data,length); application_reply[n+length]=0; return HAL_OK;
    }
    if(uart==&camera_uart) {
        assert(length<sizeof(camera_request)); memcpy(camera_request,data,length); camera_request[length]=0; return HAL_OK;
    }
    assert(uart == &motor_uart);
    reply_index = 0; reply_length=4;
    reply[0] = data[0]; reply[1] = data[1]; reply[2] = 2; reply[3] = 0x6B;
    if (data[1] == 0xAA)
    {
        if(length==37U) {
            memcpy(velocity_frame,data,length); ++velocity_frames;
            simulated_rpm=(data[6]==0?1:-1)*(int)((unsigned)data[7]*256U+data[8]);
            for(unsigned i=0;i<4U;++i) {
                unsigned o=4+i*8;
                int polarity=(i==0||i==3)?1:0;
                int magnitude=(int)((unsigned)data[o+3]*256U+data[o+4]);
                wheel_velocity[i]=data[o+2]==polarity?magnitude:-magnitude;
            }
            reply[0]=1; reply[1]=0xF6;
            if (fail_position_ack) reply_index=4;
            return HAL_OK;
        }
        assert(length == sizeof(frame));
        memcpy(frame, data, length);
        ++position_frames;
        reply[0] = 1; reply[1] = 0xFD;
        if (fail_position_ack) reply_index = 4;
    }
    if(data[1]==0x36) {
        unsigned i=data[0]-1U; assert(i<4&&length==3);
        int64_t signed_counts=(int64_t)llround(wheel_encoder[i]);
        if(i==0||i==3) signed_counts=-signed_counts;
        uint32_t magnitude=(uint32_t)llabs(signed_counts);
        reply_length=8; reply[2]=signed_counts<0;
        reply[3]=(uint8_t)(magnitude>>24); reply[4]=(uint8_t)(magnitude>>16);
        reply[5]=(uint8_t)(magnitude>>8); reply[6]=(uint8_t)magnitude; reply[7]=0x6B;
        if(fail_encoder) reply_index=8;
    }
    if (data[1] == 0x3A)
    {
        reply[2] = motor_status[data[0] - 1];
        if (fail_status) reply_index = 4;
    }
    if (data[1] == 0xFE)
    {
        simulated_rpm=0;
        wheel_velocity[data[0]-1U]=0;
        ++stop_frames;
        if (fail_stop_ack) reply_index = 4;
    }
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *uart, uint8_t *data,
                                   uint16_t length, uint32_t timeout)
{
    assert(uart == &motor_uart && length == 1);
    if (reply_index == reply_length) { tick += timeout; return HAL_TIMEOUT; }
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
    if (id == 1) { *status = lift_flags; if ((lift_flags & 3U) == 3U && !fail_lift_status && absolute_commands[0]) origin_position[0] = absolute_target[0]; return !fail_lift_status; }
    assert(id == 2); *status = arm_flags; if ((arm_flags & 3U) == 3U && !fail_arm_status && absolute_commands[1]) origin_position[1] = absolute_target[1]; return !fail_arm_status;
}
bool ArmMotor_ReadPosition(uint8_t id, int64_t *angle) {
    ++origin_reads;
    if (fail_origin_read) { LiftMotor_Fault(id,"POSITION_READ"); return false; }
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
    application_reply[0]=camera_request[0]=0;
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
    ImuLink_Init(NULL); imu_uart.RxState=HAL_UART_STATE_READY; velocity_frames=0; simulated_rpm=0;
    memset(wheel_encoder,0,sizeof(wheel_encoder)); memset(wheel_velocity,0,sizeof(wheel_velocity)); fail_encoder=0;
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
    tick += 50; RobotControl_Tick(); /* Complete the second axis for either ordering. */
    arm_flags = lift_flags = 1;
}
/* Generic progression helper; timing boundaries have dedicated assertions. */
static void advance_grab_hold(void) { tick += 1000; RobotControl_Tick(); }





static void test_arm_distance_control(void)
{
    RobotArmDistanceCommand cmd;
    assert(RobotProtocol_ParseArmDistance("ARM_MOVE,1,E,127,10,3200", &cmd));
    assert(RobotProtocol_ArmDistanceAngle(&cmd) == 3600);
    const char *invalid[] = {"ARM_MOVE,0,E,127,10,3200", "ARM_MOVE,1,F,127,10,3200",
        "ARM_MOVE,1,E,1001,10,3200", "ARM_MOVE,1,E,127,301,3200", "ARM_MOVE,1,E,127,10,0",
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

static void test_lift_calibration(void)
{
    RobotLiftAngleCommand command;
    assert(RobotProtocol_ParseLiftAngle("LIFT_ANGLE,1,U,90,5,3200", &command));
    const char *invalid[] = {"LIFT_ANGLE,0,U,90,5,3200", "LIFT_ANGLE,1,E,90,5,3200",
        "LIFT_ANGLE,1,U,0,5,3200", "LIFT_ANGLE,1,U,3601,5,3200",
        "LIFT_ANGLE,1,U,90,4,3200", "LIFT_ANGLE,1,U,90,301,3200",
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
        "LIFT_MOVE,1,U,10,4,3200", "LIFT_MOVE,1,U,10,301,3200",
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
static void center_average_sample(const char *line)
{
    camera_sample(line);
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
        center_average_sample("x=400,y=160\n"); assert(position_frames == 0); /* Ignore block frames. */
        center_average_sample("RINGS,100,160,266,160,400,160\n");
        assert(position_frames == 1 && wheel_pulses() == (ring == 1 ? 1702U : ring == 2 ? 109U : 1571U));
        center_average_sample("RINGS,100,160,300,180,400,160\n");
        assert(position_frames == 1); /* Ignore newer coordinates while moving. */
        finish_alignment_move();
        assert(position_frames == 2 && wheel_pulses() == 100U && frame[6] == 1);
        feed("MOVE,2,F,100,45,9889\n"); assert(position_frames == 2);
        finish_alignment_move();
        center_average_sample("RINGS,100,160,300,180,400,160\n"); assert(position_frames == 2);
        feed("MOVE,2,F,100,45,9889\n"); assert(position_frames == 3);
    }
    reset(); camera_ready(); feed("ALIGN_RING,1,2,10000,20000\n");
    center_average_sample("RINGS,100,160,256,170,400,160\n");
    assert(position_frames == 1 && wheel_pulses() == 218U); /* Y uses lateral calibration. */
    feed("CMD,2,S\n"); center_average_sample("RINGS,100,160,400,170,450,160\n");
    assert(position_frames == 1);
    reset(); camera_ready(); feed("ALIGN,1,10000,20000\n");
    center_average_sample("RINGS,100,160,400,170,450,160\n"); assert(position_frames == 0);
    reset(); camera_ready(); feed("ALIGN_RING,1,2,10000,20000\n");
    poll_without_keep(30000);
    center_average_sample("RINGS,100,160,400,170,450,160\n"); assert(position_frames == 0);
}
static void test_alignment_compensation_stop(void)
{
    for (unsigned mode = 0; mode < 2; ++mode) {
        const char *command = mode ? "ALIGN_RING,1,2,10000,20000\n" : "ALIGN,1,10000,20000\n";
        const char *sample = mode ? "RINGS,100,160,256,160,400,160\n" : "x=256,y=160\n";
        reset(); camera_ready(); feed(command);
        center_average_sample(sample); assert(position_frames == 1 && wheel_pulses() == 100U);
        center_average_sample(sample); assert(position_frames == 1);
        feed("CMD,2,S\n"); assert(stop_frames == 4);
        center_average_sample(sample); assert(position_frames == 1);
        reset(); camera_ready(); feed(command);
        center_average_sample(sample); center_average_sample(sample);
        fail_status = 1; tick += 60; RobotControl_Tick(); assert(stop_frames == 4);
        center_average_sample(sample); assert(position_frames == 1);
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
    assert(pulses[0] == 1636 && pulses[1] == 545 && pulses[2] == 1636 && pulses[3] == 545);
    assert(speeds[0] == 20 && speeds[1] == 7 && speeds[2] == 20 && speeds[3] == 7);
    center = (CameraCenter){266,170};
    assert(CameraProtocol_CenterProfile(&center, 10000, 10000, &settings, pulses, speeds, &aligned));
    assert(pulses[0] == 218 && pulses[1] == 0 && speeds[0] == 10); /* Diagonal: two wheels stay still. */
    center = (CameraCenter){259,160};
    assert(CameraProtocol_CenterProfile(&center, 1, 1, &settings, pulses, speeds, &aligned) && pulses[0] == 1);
    center = (CameraCenter){257,159};
    assert(CameraProtocol_CenterProfile(&center, 10000, 20000, &settings, pulses, speeds, &aligned) && !aligned);
    center = (CameraCenter){258,158};
    assert(CameraProtocol_CenterProfile(&center, 10000, 20000, &settings, pulses, speeds, &aligned) && !aligned);
    assert(pulses[0] == -22 && pulses[1] == 65); /* +/-2 px must now produce correction. */
    center = (CameraCenter){512,160};
    assert(!CameraProtocol_CenterProfile(&center, 10000, 20000, &settings, pulses, speeds, &aligned));

    reset(); camera_ready();
    camera_feed("x=400,y=160\n"); /* Discard stale coordinates at start. */
    feed("ALIGN,1,10000,20000\n"); RobotControl_Tick(); assert(position_frames == 0);
    center_average_sample("x=266,y=160\n");
    assert(position_frames == 1 && wheel_pulses() == 109 && frame[6] == 1 && frame[8] == 10);
    center_average_sample("x=256,y=180\n"); assert(position_frames == 1);
    feed("MOVE,2,F,100,45,9889\nCMD,3,A\n");
    assert(position_frames == 1 && servo_commands == 0);
    finish_alignment_move();
    assert(position_frames == 2 && wheel_pulses() == 100 && frame[8] == 10);
    center_average_sample("x=400,y=180\n"); assert(position_frames == 2);
    finish_alignment_move();
    center_average_sample("x=400,y=180\n"); assert(position_frames == 2); /* No resampling after arrival. */
    feed("ALIGN,1,10000,20000\n"); center_average_sample("x=266,y=160\n"); assert(position_frames == 2);
    feed("ALIGN,4,10000,20000\n"); center_average_sample("x=266,y=160\n"); assert(position_frames == 3); /* New sequence can run again. */

    reset(); camera_ready(); feed("ALIGN_CFG,1,0,10000,10000,10,20,0\n");
    center_average_sample("x=356,y=210\n");
    assert(position_frames == 1 && wheel_pulses() == 1091 && frame[8] == 20 && frame[6] == 1); /* X first. */
    center_average_sample("x=400,y=180\n"); assert(position_frames == 1); /* No Y before X arrival. */
    finish_alignment_move();
    assert(position_frames == 2 && wheel_pulses() == 545 && frame[8] == 20 && frame[19] == 1); /* Original averaged Y, L. */
    finish_alignment_move(); center_average_sample("x=400,y=180\n"); assert(position_frames == 2);
    feed("MOVE,2,F,100,45,9889\n"); assert(position_frames == 3); /* Zero offset completes on Y arrival. */
    reset(); camera_ready(); feed("ALIGN,1,9889,9889\n");
    tick = 30000; RobotControl_Tick(); center_average_sample("x=400,y=160\n"); assert(position_frames == 0);
    feed("MOVE,2,F,100,45,9889\n"); assert(position_frames == 1);
    reset(); camera_ready(); feed("ALIGN,1,9889,9889\n"); center_average_sample("x=266,y=160\n");
    feed("CMD,2,S\n"); assert(stop_frames == 4); center_average_sample("x=400,y=160\n"); assert(position_frames == 1);
    reset(); camera_ready(); feed("ALIGN,1,9889,9889\n"); center_average_sample("x=266,y=160\n");
    fail_status = 1; tick += 60; RobotControl_Tick(); assert(stop_frames == 4);
    center_average_sample("x=400,y=160\n"); assert(position_frames == 1);
    reset(); camera_ready(); feed("ALIGN,1,9889,9889\n"); center_average_sample("x=266,y=160\n");
    tick = 120000; RobotControl_Tick(); assert(stop_frames == 4);
    center_average_sample("x=400,y=160\n"); assert(position_frames == 1);
    reset(); camera_ready();
    camera_feed("x=2"); camera_feed("56,y=160\n"); assert(CameraLink_TakeCenter(&center) && center.x == 256);
    camera_feed("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx\n"); assert(!CameraLink_TakeCenter(&center));
    camera_feed("x=511,y=319\n"); assert(CameraLink_TakeCenter(&center) && center.x == 511);
    camera_uart.RxState = HAL_UART_STATE_READY; HAL_UART_ErrorCallback(&camera_uart);
    RobotControl_Process(); center_average_sample("x=256,y=160\n");
    assert(CameraLink_TakeCenter(&center) && center.y == 160);
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
        center_average_sample(mode ? "RINGS,100,160,296,160,400,160\n" : "x=296,y=160\n");
        assert(position_frames == 1 && wheel_pulses() == 436U && frame[8] == 30);
        finish_alignment_move();
        assert(position_frames == 2 && wheel_pulses() == 70U && frame[8] == 12 && frame[6] == 0);
        const char *center = mode ? "RINGS,100,160,266,160,400,160\n" : "x=266,y=160\n";
        center_average_sample(center); assert(position_frames == 2);
        finish_alignment_move(); center_average_sample(center); assert(position_frames == 2);
        feed("ALIGN,701,10000,20000\n"); center_average_sample("x=296,y=160\n");
        assert(position_frames == 3 && wheel_pulses() == 436 && frame[8] == 20); /* Legacy resets speed defaults. */
    }
    reset(); camera_ready(); feed("ALIGN_CFG,702,0,10000,20000,12,30,0\n");
    center_average_sample("x=256,y=160\n"); center_average_sample("x=256,y=160\n"); assert(position_frames == 0);
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

static void test_parallel_alignment(void)
{
    CameraParallelCommand cmd; float slope;
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
    tick += 1000; RobotControl_Tick(); assert(servo_channel == 'B' && servo_angle == 274);
    feed("BASE,100,308,30\n"); // Duplicate must not restart ramp.
    tick += 1000; RobotControl_Tick(); assert(servo_angle == 304);
    tick += 250; RobotControl_Tick(); assert(servo_angle == 308);
    feed("BASE,103,248,60\n"); tick += 500; RobotControl_Tick(); assert(servo_angle == 293);
    feed("BASE,104,218,60\n"); tick += 500; RobotControl_Tick(); assert(servo_angle == 278);
    tick += 1250; RobotControl_Tick(); assert(servo_angle == 218);
    unsigned servos = servo_commands; feed("BASE,105,218,60\n"); RobotControl_Tick(); assert(servo_commands == servos);
    feed("BASE,106,360,1\n"); tick += 10000; RobotControl_Tick(); assert(servo_angle == 228);
    feed("CMD,107,Z\n"); servos = servo_commands; tick += 200000; RobotControl_Tick(); assert(servo_commands == servos);
    feed("SERVO,108,B,100\nBASE,109,160,30\n"); tick += 1000; RobotControl_Tick(); assert(servo_angle == 126);
    feed("SERVO,110,B,80\n"); tick += 10000; RobotControl_Tick(); assert(servo_angle == 80);
    feed("BASE,111,140,30\n"); fail_servo = 1; tick += 1000; RobotControl_Tick();
    fail_servo = 0; servos = servo_commands; tick += 10000; RobotControl_Tick(); assert(servo_commands == servos);
    reset(); RobotControl_Init(&command_uart, true); servo_commands = 0; // Unknown start: first target is direct.
    feed("BASE,112,100,30\n"); assert(servo_angle == 100 && servo_commands == 1);
    feed("BASE,113,160,30\n"); tick += 1000; RobotControl_Tick(); assert(servo_angle == 126);
    reset(); feed("CMD,114,A\nBASE,115,200,30\n"); assert(arm_stops == 1);
    tick += 1000; RobotControl_Tick(); assert(servo_angle == 222); // Automatic base target tracked at 248.
}
static void test_parallel_pixel_tolerance(void)
{
    const CameraCenter near[] = {{100,150},{200,151},{300,153}};
    float slope;
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
    feed("CMD,1205,A\nORIGIN,1206,180\n"); assert(arm_stops == 0 && servo_angle == 180);
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
static void test_turn_cards(void)
{
    RobotSequenceCommand cmd;
    assert(RobotProtocol_ParseSequence("PLAN_ITEM,100,0,T,0,240,120", &cmd));
    assert(cmd.mode == 'T' && cmd.theta == 0 && cmd.open_angle == 240 && cmd.gripper_dps == 120);
    assert(!RobotProtocol_ParseSequence("STATE,100,T,0,271,120", &cmd));
    assert(!RobotProtocol_ParseSequence("STATE,100,T,0,240,0", &cmd));
    assert(!RobotProtocol_ParseSequence("STATE,100,T,0,240,361", &cmd));
    assert(!RobotProtocol_ParseSequence("PLAN_ITEM,100,16,T,0,240,120", &cmd));
    reset();
    feed("PLAN_BEGIN,1400,2\nPLAN_ITEM,1400,0,T,0,120,60\nPLAN_ITEM,1400,1,T,120,0,120\nPLAN_RUN,1400\n");
    assert(servo_channel == 'T' && servo_angle == 0 && absolute_commands[1] == 0);
    tick += 1000; RobotControl_Tick(); assert(servo_angle == 60);
    tick += 1000; RobotControl_Tick(); assert(servo_angle == 120);
    tick += 999; RobotControl_Tick(); assert(servo_angle == 120);
    tick += 1; RobotControl_Tick(); assert(servo_angle == 120);
    tick += 500; RobotControl_Tick(); assert(servo_angle == 60);
    feed("CMD,1401,Z\n"); unsigned servos = servo_commands;
    tick += 5000; RobotControl_Tick(); assert(servo_commands == servos);
    feed("STATE,1402,T,20,20,1\n"); RobotControl_Tick(); assert(servo_angle == 20);
    feed("STATE,1403,T,20,80,60\n"); tick += 1000; RobotControl_Tick(); assert(servo_angle == 80);
    feed("STATE,1404,P,0,0,0,0,20,20,50\n");
    unsigned start = servo_commands;
    arm_flags = lift_flags = 3;
    for (unsigned i = 0; i < 600; ++i) { tick += 50; RobotControl_Tick(); }
    for (unsigned i = start; i < servo_commands; ++i) assert(servo_channels[i] != 'T');
    feed("STATE,1405,T,80,140,60\n"); tick += 1000; RobotControl_Tick(); assert(servo_angle == 140);
}
static void test_center_sample_average(void)
{
    for (unsigned ring = 0; ring <= 2; ring += 2) {
        reset(); camera_ready(); char command[80];
        snprintf(command, sizeof(command), "ALIGN_CFG,1500,%u,10000,20000,10,20,0\n", ring);
        feed(command); RobotControl_Tick(); assert(position_frames == 0);
        camera_sample("x=bad,y=160\n"); assert(position_frames == 0);
        camera_sample(ring ? "RINGS,100,160,266,160,400,160\n" : "x=266,y=160\n");
        assert(position_frames == 1 && wheel_pulses() == 109);
        finish_alignment_move();
        camera_sample("x=400,y=160\n"); assert(position_frames == 1);
    }
    reset(); camera_ready(); feed("ALIGN_CFG,1501,0,10000,20000,10,20,0\n");
    tick += 30000; RobotControl_Tick(); camera_sample("x=266,y=160\n");
    assert(position_frames == 0); /* No valid new coordinate before timeout. */
}
static void test_center_axis_stages(void)
{
    reset(); camera_ready(); feed("ALIGN_RING,1600,2,10000,20000\n");
    center_average_sample("RINGS,100,160,246,170,400,160\n");
    assert(position_frames == 1 && wheel_pulses() == 109 && frame[6] == 0); /* Negative X -> F. */
    finish_alignment_move();
    assert(position_frames == 2 && wheel_pulses() == 218 && frame[19] == 1); /* Positive Y -> L. */
    finish_alignment_move();
    assert(position_frames == 3 && wheel_pulses() == 100 && frame[6] == 1); /* Final X compensation. */
    finish_alignment_move();
    feed("MOVE,1601,F,100,45,9889\n"); assert(position_frames == 4);

    reset(); camera_ready(); feed("ALIGN_CFG,1602,0,10000,20000,10,20,0\n");
    center_average_sample("x=266,y=150\n"); assert(position_frames == 1);
    feed("CMD,1603,S\n"); finish_alignment_move();
    assert(position_frames == 1); /* Cancel clears pending Y and skips compensation. */
}
static void test_fractional_center_mean(void)
{
    CameraAlignSettings settings = CameraProtocol_DefaultSettings();
    int32_t pulses[2]; uint16_t speeds[2]; bool aligned;
    assert(CameraProtocol_CenterMeanAxes(256, 160, 1, 10000, 20000, &settings, pulses, speeds, &aligned));
    assert(aligned && pulses[0] == 0 && pulses[1] == 0);
    assert(CameraProtocol_CenterMeanAxes(257, 159, 1, 10000, 20000, &settings, pulses, speeds, &aligned));
    assert(!aligned && pulses[0] == 11 && pulses[1] == -22);
    assert(CameraProtocol_CenterMeanAxes(255, 161, 1, 10000, 20000, &settings, pulses, speeds, &aligned));
    assert(!aligned && pulses[0] == -11 && pulses[1] == 22);

    assert(CameraProtocol_CenterMeanAxes(12821, 8000, 50, 10000, 20000, &settings, pulses, speeds, &aligned));
    assert(!aligned && pulses[0] == 5); /* Any nonzero error is corrected. */
    assert(CameraProtocol_CenterMeanAxes(12851, 7949, 50, 10000, 20000, &settings, pulses, speeds, &aligned));
    assert(!aligned && pulses[0] == 11 && pulses[1] == -22); /* x=257.02, y=158.98. */
    assert(CameraProtocol_CenterMeanAxes(12749, 8000, 50, 10000, 20000, &settings, pulses, speeds, &aligned));
    assert(!aligned && pulses[0] == -11); /* x=254.98, symmetric negative rounding. */
    assert(CameraProtocol_CenterMeanAxes(13801, 8000, 50, 10000, 20000, &settings, pulses, speeds, &aligned));
    assert(pulses[0] == 218 && speeds[0] == 20); /* x=276.02: actual error >20 px. */
    assert(!CameraProtocol_CenterMeanAxes(0, 0, 0, 10000, 20000, &settings, pulses, speeds, &aligned));
    assert(!CameraProtocol_CenterMeanAxes(25600, 8000, 50, 10000, 20000, &settings, pulses, speeds, &aligned));

}
static void test_grab_initial_base_ramp(void)
{
    reset(); feed("SERVO,2100,B,140\n");
    unsigned r = absolute_commands[1], z = absolute_commands[0];
    feed("STATE,2101,A,1100,-50,200,-40,20,20,50,60,0,60,0,248,140\n");
    assert(servo_angle == 140 && absolute_commands[1] == r);
    tick += 200; RobotControl_Tick(); assert(servo_angle == 142);
    assert(absolute_commands[1] == r && absolute_commands[0] == z);
    tick += 800; RobotControl_Tick(); assert(servo_angle == 194);
    assert(absolute_commands[1] == r && absolute_commands[0] == z);
    tick += 800; RobotControl_Tick(); assert(servo_angle == 246);
    assert(absolute_commands[1] == r && absolute_commands[0] == z);
    tick += 200; RobotControl_Tick(); assert(servo_angle == 248);
    assert(absolute_commands[1] == r + 1);
    feed("CMD,2102,Z\nSERVO,2103,B,300\nSTATE,2104,A,1100,-50,200,-40,20,20,50,60,0,60,0,200,140\n");
    assert(servo_angle == 300);
    tick += 200; RobotControl_Tick(); assert(servo_angle == 298);
    tick += 800; RobotControl_Tick(); assert(servo_angle == 250);
    feed("CMD,2105,Z\n"); unsigned servos = servo_commands;
    tick += 3000; RobotControl_Tick(); assert(servo_commands == servos);
}
static void test_decimal_camera_chain(void)
{
    CameraCenter rings[3]; int32_t pulses[4]; uint16_t speeds[4]; bool aligned;
    CameraAlignSettings settings = {10U,20U,0}; float slope;
    assert(CameraProtocol_ParseRings("RINGS,100.1,159.9,256.4,160.3,400.8,160.2", rings));
    assert(fabsf(rings[1].x-256.4f)<0.0001f && fabsf(rings[1].y-160.3f)<0.0001f);
    assert(CameraProtocol_ParseRings("RINGS,0.0,0.0,256,160,511.9,319.9", rings));
    const char *invalid[] = {
        "RINGS,100.12,160,256,160,400,160", "RINGS,100.,160,256,160,400,160",
        "RINGS,.1,160,256,160,400,160", "RINGS,-0.1,160,256,160,400,160",
        "RINGS,nan,160,256,160,400,160", "RINGS,1e2,160,256,160,400,160",
        "RINGS,100,160,256,160,512.0,160", "RINGS,100,160,256,160,400,320.0",
        "RINGS,100.2,160,100.1,160,400,160", "RINGS,100,160,256,160,400,160.1junk",
        "RINGS,99999999999999999999,160,256,160,400,160"
    };
    for (unsigned i=0;i<sizeof(invalid)/sizeof(invalid[0]);++i)
        assert(!CameraProtocol_ParseRings(invalid[i],rings));
    assert(CameraProtocol_CenterMeanAxes(256.4,160.3,1,10000,20000,&settings,pulses,speeds,&aligned));
    assert(!aligned && pulses[0]==4 && pulses[1]==7);
    assert(CameraProtocol_CenterMeanAxes(12800.1,8000.0,50,1000000,10000,&settings,pulses,speeds,&aligned));
    assert(!aligned && pulses[0]==2 && pulses[1]==0); /* Mean x=256.002, never truncated. */
    assert(CameraProtocol_CenterMeanAxes(255.9,159.9,1,10000,20000,&settings,pulses,speeds,&aligned));
    assert(pulses[0]==-1 && pulses[1]==-2);
    assert(!CameraProtocol_CenterMeanAxes(NAN,160,1,10000,20000,&settings,pulses,speeds,&aligned));
    CameraCenter center={256.1f,160.1f};
    assert(CameraProtocol_CenterProfile(&center,10000,10000,&settings,pulses,speeds,&aligned));
    assert(pulses[0]==2 && pulses[1]==0); /* Mix fractional axes before rounding wheel pulses. */
    CameraCenter line[3]={{100.0f,160.0f},{256.0f,160.1f},{400.0f,160.2f}};
    assert(CameraProtocol_RingSlope(line,&slope) && slope>0.6f && slope<0.7f);
    assert(CameraProtocol_RingYSpread(line)>0.19f && CameraProtocol_RingYSpread(line)<0.21f);
    CameraPoseAlignCommand joint={.ring_index=2,.forward_ppm=10000,.lateral_ppm=20000,
        .wheelbase_mm=190,.track_mm=251,.settings={10,20,0}};
    CameraCenter pose[3]={{100.4f,160.3f},{256.4f,160.3f},{400.4f,160.3f}};
    assert(CameraProtocol_PoseProfile(&joint,pose,pulses,speeds,&aligned) && !aligned);
    assert(pulses[0]==11 && pulses[1]==-2); /* Fractional translation reaches wheel profile. */
    reset(); camera_ready(); feed("ALIGN_RING,9900,2,10000,20000\n");
    camera_sample("RINGS,100.1,159.9,256.4,160.3,400.8,160.2\n");
    assert(position_frames==1 && wheel_pulses()==4);
    finish_alignment_move(); assert(position_frames==2 && wheel_pulses()==7);
}
static void test_arm_speed_300_rpm(void)
{
    RobotArmDistanceCommand arm; RobotLiftDistanceCommand lift;
    RobotLiftAngleCommand angle; RobotArmPoseCommand pose;
    assert(RobotProtocol_ParseArmDistance("ARM_MOVE,9000,E,127,300,3200", &arm));
    assert(RobotProtocol_ParseLiftDistance("LIFT_MOVE,9000,U,40,300,3200", &lift));
    assert(RobotProtocol_ParseLiftAngle("LIFT_ANGLE,9000,U,90,300,3200", &angle));
    assert(RobotProtocol_ParseArmPose("ARM_POSE,9000,0,10,-10,300,3200", &pose));
    assert(!RobotProtocol_ParseArmPose("ARM_POSE,9000,0,10,-10,301,3200", &pose));
    reset(); feed("ARM_MOVE,9000,E,127,300,3200\n");
    assert(last_arm_angle.speed_rpm == 300);
    reset(); feed("LIFT_MOVE,9000,U,40,300,3200\n");
    assert(last_lift_angle.rpm == 300);
    reset(); feed("ARM_POSE,9000,0,10,-10,300,3200\n");
    assert(last_arm_angle.speed_rpm == 300);
    finish_grab_axes(); assert(last_lift_angle.rpm == 300);
}

static void test_joint_alignment(void)
{
    CameraPoseAlignCommand cmd;
    assert(CameraProtocol_ParsePoseAlign("ALIGN_POSE,1900,2,10000,20000,10,20,10,300,250,0,0,0", &cmd));
    CameraPoseAlignCommand measured;
    assert(CameraProtocol_ParsePoseAlign("ALIGN_POSE,1900,2,10000,20000,10,20,10,190,251.4,-7,-291.27,0", &measured));
    assert(measured.camera_forward_mm == -7.0f);
    assert(fabsf(measured.track_mm - 251.4f) < 0.001f && fabsf(measured.camera_left_mm + 291.27f) < 0.001f);
    assert(!CameraProtocol_ParsePoseAlign("ALIGN_POSE,1900,2,10000,20000,10,20,10,190,251.444,0,-291.27,0", &measured));
    CameraCenter rings[3] = {{156,160},{256,160},{356,160}};
    int32_t pulses[4]; uint16_t speeds[4]; bool aligned;
    assert(CameraProtocol_PoseProfile(&cmd, rings, pulses, speeds, &aligned) && aligned);
    rings[1].x = 257;
    assert(CameraProtocol_PoseProfile(&cmd, rings, pulses, speeds, &aligned) && !aligned);
    assert(pulses[0] == 11 && pulses[1] == 11 && pulses[2] == 11 && pulses[3] == 11);
    rings[0] = (CameraCenter){156,150}; rings[1] = (CameraCenter){266,161}; rings[2] = (CameraCenter){356,170};
    assert(CameraProtocol_PoseProfile(&cmd, rings, pulses, speeds, &aligned) && !aligned);
    assert(pulses[0] != pulses[1]); /* Translation and yaw share one wheel profile. */
    int32_t normal[4]; memcpy(normal, pulses, sizeof(normal)); cmd.reverse = 1;
    assert(CameraProtocol_PoseProfile(&cmd, rings, pulses, speeds, &aligned) && !aligned);
    assert(memcmp(normal, pulses, sizeof(normal)) != 0);
    reset(); camera_ready(); feed("ALIGN_POSE,1901,2,10000,20000,10,20,10,300,250,0,0,0\n");
    camera_sample("RINGS,156,150,266,161,356,170\n"); assert(position_frames == 1);
    finish_alignment_move();
    for (unsigned i = 0; i < 3; ++i) camera_sample("RINGS,156,160,256,160,356,160\n");
    assert(position_frames == 2 && wheel_pulses() == 100); /* One final X compensation. */
    finish_alignment_move();
    feed("MOVE,1902,F,100,45,9889\n"); assert(position_frames == 3);
}
static void test_custom_center(void)
{
    CameraAlignCommand cmd;
    assert(CameraProtocol_ParseAlign("ALIGN_CFG,2000,2,10000,20000,10,20,0,300.5,180.5", &cmd));
    int32_t axes[2]; uint16_t speeds[2]; bool aligned;
    assert(CameraProtocol_CenterMeanAxes(300.5,180.5,1,10000,20000,&cmd.settings,axes,speeds,&aligned) && aligned);
    assert(CameraProtocol_CenterMeanAxes(301.5,181.5,1,10000,20000,&cmd.settings,axes,speeds,&aligned) && !aligned);
    assert(axes[0] == 11 && axes[1] == 22);
    const char *invalid[] = {"512,160", "256,320", "-1,160", "256.01,160", "256", "", "256,160,0", "NaN,160"};
    char command[160];
    for (unsigned i = 0; i < sizeof(invalid)/sizeof(invalid[0]); ++i) {
        snprintf(command,sizeof(command),"ALIGN_CFG,2000,2,10000,20000,10,20,0,%s",invalid[i]);
        assert(!CameraProtocol_ParseAlign(command,&cmd));
    }
    assert(CameraProtocol_ParseAlign("ALIGN_CFG,2000,0,10000,20000,10,20,0,511.9,319.9", &cmd));
    assert(CameraProtocol_ParseAlign("ALIGN,2000,10000,20000", &cmd));
    assert(CameraProtocol_TargetX(&cmd.settings) == 256 && CameraProtocol_TargetY(&cmd.settings) == 160);
    for (unsigned ring = 0; ring < 2; ++ring) {
        reset(); camera_ready();
        snprintf(command,sizeof(command),"ALIGN_CFG,2001,%u,10000,20000,10,20,0,296,160\n",ring ? 2 : 0);
        feed(command);
        center_average_sample(ring ? "RINGS,100,160,296,160,400,160\n" : "x=296,y=160\n");
        assert(position_frames == 0);
        feed("MOVE,2002,F,100,45,9889\n"); assert(position_frames == 1);
    }
    CameraPoseAlignCommand pose;
    assert(CameraProtocol_ParsePoseAlign("ALIGN_POSE,2003,2,10000,20000,10,20,0,190,251.4,-7,-291.27,0,300,180",&pose));
    CameraCenter rings[3] = {{100,180},{300,180},{400,180}};
    int32_t pulses[4]; uint16_t wheel_speeds[4];
    assert(CameraProtocol_PoseProfile(&pose,rings,pulses,wheel_speeds,&aligned) && aligned);
    rings[1].x = 301;
    assert(CameraProtocol_PoseProfile(&pose,rings,pulses,wheel_speeds,&aligned) && !aligned);
    for (unsigned i = 0; i < 4; ++i) assert(pulses[i] == 11);
    assert(!CameraProtocol_ParsePoseAlign("ALIGN_POSE,2003,2,10000,20000,10,20,0,190,251.4,-7,-291.27,0,",&pose));
}
static void test_base_acceleration(void)
{
    // Short move never reaches the requested speed: symmetric triangular ramp.
    reset(); feed("SERVO,2100,B,100\nBASE,2101,160,360\n");
    const uint16_t expected[] = {102,110,122,137,150,157,160};
    uint16_t previous = 100;
    for (unsigned i = 0; i < 7U; ++i) {
        tick += i == 6U ? 215 : 200; RobotControl_Tick();
        assert(servo_channel == 'B' && servo_angle == expected[i] && servo_angle >= previous);
        previous = servo_angle;
    }
    unsigned commands = servo_commands; tick += 5000; RobotControl_Tick(); assert(servo_commands == commands);
    // Same acceleration on reverse travel and both endpoint directions.
    feed("BASE,2102,100,360\n"); tick += 200; RobotControl_Tick(); assert(servo_angle == 158);
    tick += 1215; RobotControl_Tick(); assert(servo_angle == 100);
    // Tick rollover still follows elapsed time, with no endpoint overshoot.
    tick = UINT32_MAX - 100U; feed("BASE,2103,0,60\n");
    tick += 200; RobotControl_Tick(); assert(servo_angle == 98);
    tick += 3000; RobotControl_Tick(); assert(servo_angle == 0);
    feed("BASE,2104,360,360\n"); tick += 4000; RobotControl_Tick(); assert(servo_angle == 360);
    // Automatic 108-degree move accelerates, cruises, then decelerates in 2 s.
    reset(); feed("CMD,2105,P\n"); finish_grab_axes(); advance_grab_hold();
    tick += 200; RobotControl_Tick(); assert(servo_angle == 246);
    tick += 800; RobotControl_Tick(); assert(servo_angle == 194);
    tick += 800; RobotControl_Tick(); assert(servo_angle == 142);
    tick += 200; RobotControl_Tick(); assert(servo_angle == 140);
    // Large automatic move extends beyond 2 s rather than increasing acceleration.
    reset(); feed("SERVO,2106,B,0\nCMD,2107,A\n");
    unsigned arm_commands = absolute_commands[1];
    tick += 2000; RobotControl_Tick();
    assert(servo_angle > 0 && servo_angle < 248 && absolute_commands[1] == arm_commands);
    tick += 1000; RobotControl_Tick();
    assert(servo_angle == 248 && absolute_commands[1] == arm_commands + 1);
    // Cancel during acceleration holds the last PWM target.
    reset(); feed("BASE,2108,308,60\n"); tick += 200; RobotControl_Tick();
    uint16_t held = servo_angle; feed("CMD,2109,Z\n");
    commands = servo_commands; tick += 5000; RobotControl_Tick();
    assert(servo_commands == commands && servo_angle == held);
}
static uint16_t last_servo_angle(char channel)
{
    for (unsigned i=servo_commands; i>0; --i) if (servo_channels[i-1]==channel) return servo_history[i-1];
    assert(false); return 0;
}
static void finish_vector_position(void)
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
    lift_flags=1; tick+=50; RobotControl_Tick(); assert(last_servo_angle('G')==60);
    finish_vector_position(); advance_grab_hold(); assert(last_servo_angle('G')==0); // Step 3.
    rc=absolute_commands[1]; tick+=999; RobotControl_Tick(); assert(absolute_commands[1]==rc);
    tick+=1; RobotControl_Tick(); // Step 4: upward pair starts together.
    assert(absolute_target[1]==312 && absolute_target[0]==0 && absolute_commands[1]==rc+1);
    finish_vector_position(); advance_grab_hold(); finish_base_rotation(); assert(servo_angle==140); // Step 5.
    advance_grab_hold(); assert(absolute_target[1]==567 && absolute_target[0]==2700); // Step 6 staged descent.
    finish_vector_position(); assert(absolute_target[0]==3600 && last_lift_angle.rpm==300);
    advance_grab_hold(); rc=absolute_commands[1]; zc=absolute_commands[0]; // Step 7 opens before returning.
    tick+=500; RobotControl_Tick(); assert(last_servo_angle('G')==30 && absolute_commands[1]==rc && absolute_commands[0]==zc);
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
    tick+=500; RobotControl_Tick(); // 60 degrees at 60 deg/s needs a full second.
    advance_grab_hold(); assert(absolute_target[1]==312 && absolute_target[0]==0); // Step 4 parallel up.
    finish_vector_position(); advance_grab_hold(); finish_base_rotation(); assert(servo_angle==248);
    advance_grab_hold(); assert(absolute_target[1]==3685 && absolute_target[0]==9000); // Step 6.
    finish_vector_position(); assert(absolute_target[0]==9900 && last_lift_angle.rpm==300);
    advance_grab_hold(); tick+=1000; RobotControl_Tick(); assert(servo_angle==60); // Step 7 open.
    advance_grab_hold(); assert(absolute_target[1]==312 && absolute_target[0]==0); // Step 8 return pair.
    finish_vector_position(); advance_grab_hold();
    feed("BASE,2301,250,30\n"); tick+=1000; RobotControl_Tick(); assert(servo_angle==250);
}
static void test_sequence_optimized_timing(void)
{
    /* Ordinary positioning settles for 200 ms, not the old fixed 500 ms.
     * Slow closing still reserves its full 60/6 = 10 seconds before retracting. */
    reset(); feed("SERVO,2600,G,60\nSTATE_V2,2601,A,1100,-50,200,-40,20,6,0,60,0,248,140\n");
    finish_vector_position();
    unsigned rc=absolute_commands[1];
    tick+=49; RobotControl_Tick(); assert(absolute_commands[1]==rc);
    tick+=1; RobotControl_Tick(); assert(absolute_commands[1]==rc+1);
    finish_vector_position();
    tick+=99; RobotControl_Tick(); assert(last_servo_angle('G')==60);
    tick+=1; RobotControl_Tick(); assert(last_servo_angle('G')==0);
    rc=absolute_commands[1];
    tick+=9999; RobotControl_Tick(); assert(absolute_commands[1]==rc);
    tick+=1; RobotControl_Tick(); assert(absolute_commands[1]==rc+1);
    feed("CMD,2602,Z\n");

    /* Initial opening overlaps positioning, but the next descent must wait
     * for 45/60 = 750 ms from the opening command, even if R arrives early. */
    reset(); feed("STATE_V2,2603,A,1100,-50,200,-40,20,60,0,60,0,248,140\n");
    finish_vector_position(); rc=absolute_commands[1];
    tick=749; RobotControl_Tick(); assert(absolute_commands[1]==rc);
    tick=750; RobotControl_Tick(); assert(absolute_commands[1]==rc+1);
    feed("CMD,2604,Z\n");

    /* Release opens only at the origin; its opening budget starts there,
     * rather than when the positioning command was sent. */
    reset(); feed("STATE_V2,2605,P,200,-40,1100,-50,20,60,0,60,0,248,140\n");
    finish_vector_position();
    unsigned servos=servo_commands;
    tick=799; RobotControl_Tick(); assert(servo_commands==servos);
    tick+=20; RobotControl_Tick();
    tick=850; RobotControl_Tick(); tick+=100; RobotControl_Tick();
    assert(last_servo_angle('B')<248);
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
    // Z at its intermediate target also waits for R; neither early arrival bypasses the barrier.
    reset(); feed("CMD,2406,A\n"); finish_vector_position(); advance_grab_hold();
    zc=absolute_commands[0]; lift_flags=3; tick+=50; RobotControl_Tick(); assert(absolute_commands[0]==zc);
    arm_flags=3; tick+=50; RobotControl_Tick(); assert(absolute_commands[0]==zc+1 && last_lift_angle.rpm==300);
    // No-op positions still advance, and all inter-card waiting is 500 ms.
    reset(); feed("PLAN_BEGIN,2500,2\nPLAN_ITEM_V2,2500,0,A,110,0,110,0,20,60,0,60,0,248,140\nPLAN_ITEM,2500,1,T,0,120,60\nPLAN_RUN,2500\n");
    arm_flags=lift_flags=3;
    for (unsigned i=0;i<200;++i) { tick+=50; RobotControl_Tick(); }
    assert(servo_channel=='T' && servo_angle==120);
    unsigned servos=servo_commands; feed("PLAN_RUN,2500\n"); assert(servo_commands==servos);
    // Inter-card hold is exactly 500 ms, with T cards retaining their original wire format.
    reset(); feed("PLAN_BEGIN,2501,2\nPLAN_ITEM,2501,0,T,0,10,10\nPLAN_ITEM,2501,1,T,10,20,10\nPLAN_RUN,2501\n");
    tick+=1000; RobotControl_Tick(); assert(servo_angle==10);
    servos=servo_commands; tick+=499; RobotControl_Tick(); assert(servo_commands==servos);
    tick+=1; RobotControl_Tick(); assert(servo_commands==servos+1 && servo_angle==10);
    tick+=1000; RobotControl_Tick(); assert(servo_angle==20);
}

static void test_transient_stall_status(void)
{
    MotorStatusGuard guard={0};
    assert(!MotorStatusGuard_Fault(&guard,0x05,UINT32_MAX-100U));
    assert(!MotorStatusGuard_Fault(&guard,0x05,298U));
    assert(MotorStatusGuard_Fault(&guard,0x05,299U)); // 400 ms across tick rollover.
    assert(!MotorStatusGuard_Fault(&guard,0x01,300U));
    assert(!MotorStatusGuard_Fault(&guard,0x05,301U)); // Clearing the flag resets persistence.
    assert(MotorStatusGuard_Fault(&guard,0x0B,302U)); // Confirmed protection beats arrival.
    assert(MotorStatusGuard_Fault(&guard,0x00,303U)); // Disable still stops immediately.
    reset(); feed("ARM_MOVE,3000,E,127,600,3200\n"); // Manual still rejects 600.
    assert(arm_angle_commands == 0);
    feed("ARM_MOVE,3001,E,127,100,3200\n");
    arm_flags=0x05; tick+=50; RobotControl_Tick(); assert(arm_stops==0);
    arm_flags=1; tick+=50; RobotControl_Tick(); assert(arm_stops==0);
    arm_flags=3; tick+=50; RobotControl_Tick(); assert(arm_stops==0);
    reset(); feed("LIFT_MOVE,3002,D,40,100,3200\n");
    lift_flags=0x05; tick+=50; RobotControl_Tick(); assert(lift_stops==0);
    tick+=399; RobotControl_Tick(); assert(lift_stops==0);
    tick+=50; RobotControl_Tick(); assert(lift_stops==1); // Persistent Cgi still stops.
    reset(); feed("LIFT_MOVE,3003,D,40,100,3200\n");
    lift_flags=0x09; tick+=50; RobotControl_Tick(); assert(lift_stops==1); // Cgp immediately stops.
    reset(); feed("ARM_MOVE,3004,E,127,100,3200\n");
    arm_flags=0x07; tick+=50; RobotControl_Tick(); assert(arm_stops==0); // Arrived + holding current.
    reset(); feed("CMD,3005,A\n"); finish_vector_position(); advance_grab_hold();
    arm_flags=lift_flags=0x05; unsigned r=arm_stops,z=lift_stops;
    tick+=50; RobotControl_Tick(); assert(arm_stops==r && lift_stops==z);
    arm_flags=lift_flags=1; tick+=50; RobotControl_Tick(); assert(arm_stops==r && lift_stops==z);
    lift_flags=0x09; tick+=50; RobotControl_Tick(); assert(arm_stops>r && lift_stops>z);
    reset(); feed("MOVE,3006,F,100,45,9889\n");
    motor_status[0]=0x05; tick+=50; RobotControl_Tick(); assert(stop_frames==0);
    motor_status[0]=1; tick+=200; RobotControl_Tick(); assert(stop_frames==0);
    motor_status[2]=0x09;
    for(unsigned i=0;i<4;++i) { tick+=50; RobotControl_Tick(); }
    assert(stop_frames==4);
}
static void test_autonomous_feedback(void)
{
    reset(); feed("PING,89\r\n"); assert(strcmp(application_reply,"PONG,89,STM32\n")==0);
    application_reply[0]=0; feed("PING,0\nPING,89,junk\n"); assert(application_reply[0]==0);
    reset(); feed("MOVE,80,F,100,45,9889\n");
    assert(application_reply[0]==0); /* Legacy manual commands still have no replies. */
    reset(); feed("QUERY,90\n"); assert(strstr(application_reply,"STATUS,90,IDLE,1,0\n"));
    application_reply[0]=0;
    feed("EXEC,91,MOVE,F,100,45,9889\n");
    assert(strstr(application_reply,"ACTION,91,ACCEPTED,0\n"));
    feed("PING,88\n"); assert(strstr(application_reply,"PONG,88,STM32\n")); // Read-only even while moving.
    application_reply[0]=0; feed("QUERY,91\n");
    assert(strcmp(application_reply,"ACTION,91,ACCEPTED,0\n")==0);
    unsigned before=position_frames;
    feed("EXEC,91,MOVE,F,100,45,9889\n"); assert(position_frames==before);
    feed("EXEC,92,MOVE,F,100,45,9889\n"); assert(strstr(application_reply,"ACTION,92,BUSY,TASK_ACTIVE\n"));
    memset(motor_status,3,sizeof(motor_status));
    for(unsigned i=0;i<4;++i) poll_without_keep(50);
    assert(strstr(application_reply,"ACTION,91,DONE,0\n"));
    application_reply[0]=0; feed("QUERY,91\n"); assert(strcmp(application_reply,"ACTION,91,DONE,0\n")==0);
    feed("EXEC,91,MOVE,F,100,45,9889\n"); assert(position_frames==before);
    feed("EXEC,91,MOVE,F,200,45,9889\n"); assert(strstr(application_reply,"ERROR,REUSED"));
    reset(); feed("EXEC,93,MOVE,F,100,45,9889\n"); motor_status[0]=0x09;
    poll_without_keep(50); assert(strstr(application_reply,"ACTION,93,ERROR,FAULT"));
    reset(); feed("EXEC,1000,STATE_V2,A,1100,-50,200,-40,20,60,0,60,0,248,140\n");
    arm_flags=lift_flags=3;
    for(unsigned i=0;i<150U&&!strstr(application_reply,"ACTION,1000,DONE");++i) poll_without_keep(500);
    assert(strstr(application_reply,"ACTION,1000,DONE,0"));
    reset(); feed("EXEC,1001,STATE_V2,P,200,-40,1100,-50,20,60,0,60,0,248,140\n");
    arm_flags=lift_flags=3;
    for(unsigned i=0;i<150U&&!strstr(application_reply,"ACTION,1001,DONE");++i) poll_without_keep(500);
    assert(strstr(application_reply,"ACTION,1001,DONE,0"));
    reset(); feed("EXEC,1002,PULSE,B,43,10\n");
    assert(strstr(application_reply,"ACTION,1002,ACCEPTED"));
    memset(motor_status,3,sizeof(motor_status));
    for(unsigned i=0;i<4;++i) poll_without_keep(50);
    assert(strstr(application_reply,"ACTION,1002,DONE,0"));
    reset(); feed("EXEC,94,STATE_V2,A,1100,-50,200,-40,20,60,0,60,0,248,140\n");
    assert(strstr(application_reply,"ACTION,94,ACCEPTED"));
    fail_arm_status=1; tick=700; RobotControl_Tick(); tick=1400; RobotControl_Tick();
    for(unsigned i=0;i<10;++i) poll_without_keep(100);
    assert(strstr(application_reply,"ACTION,94,ERROR"));
    reset(); feed("EXEC,95,MOVE,F,100,45,9889\n"); feed("CMD,96,Z\n");
    assert(strstr(application_reply,"ACTION,95,CANCELLED,STOP"));
    reset(); CameraLink_Init(&camera_uart); feed("CAM,101,QR\n");
    assert(strcmp(camera_request,"CAM,101,QR\n")==0);
    feed("CAM,102,IDLE\n"); assert(strcmp(camera_request,"CAM,102,IDLE\n")==0);
    camera_sample("VISION,101,QR,156+123+516+231\n");
    assert(strstr(application_reply,"VISION,101,QR,156+123+516+231\n"));
    application_reply[0]=0; camera_sample("VISION,101,MAT,5,111.2,160.7\n");
    assert(strstr(application_reply,"VISION,101,MAT,5,111.2,160.7\n"));
    application_reply[0]=0; camera_sample("VISION,101,MAT,7,111.2,160.7\n"); assert(application_reply[0]==0);
}
static void imu_bytes(const uint8_t *bytes,unsigned length)
{
    for(unsigned i=0;i<length;++i) {
        *imu_rx_byte=bytes[i]; imu_uart.RxState=HAL_UART_STATE_READY; HAL_UART_RxCpltCallback(&imu_uart);
    }
}
static void imu_packet(uint8_t kind,float value,bool good)
{
    uint8_t packet[11]={0x55,kind,0,0,0,0,0,0,0,0,0};
    int16_t raw=(int16_t)lroundf(value*32768.0f/(kind==0x53?180.0f:2000.0f));
    packet[6]=(uint8_t)raw; packet[7]=(uint8_t)((uint16_t)raw>>8);
    for(unsigned i=0;i<10;++i) packet[10]=(uint8_t)(packet[10]+packet[i]);
    if(!good) ++packet[10];
    imu_bytes(packet,4); ImuLink_Process(); imu_bytes(packet+4,7); ImuLink_Process();
}
static void imu_sample(float yaw,float rate)
{
    while(yaw>=180) yaw-=360;
    while(yaw< -180) yaw+=360;
    /* Simulated controller coordinates; installed sensor wire signs are reversed. */
    imu_packet(0x52,-rate,true); imu_packet(0x53,-yaw,true);
}
static void imu_ready(float yaw)
{
    imu_uart.RxState=HAL_UART_STATE_READY; ImuLink_Init(&imu_uart);
    for(unsigned i=0;i<10U;++i) { tick+=20; imu_sample(yaw,0); }
}
static void test_gyro_link_and_controller(void)
{
    GyroCommand c;
    assert(GyroProtocol_ParseExec("EXEC,1,IMU_ZERO",&c)&&c.kind==GYRO_REFERENCE);
    assert(GyroProtocol_ParseExec("EXEC,2,TURN,W,360.0,20",&c)&&c.degrees==360);
    assert(GyroProtocol_ParseExec("EXEC,3,HEADING,270.0,20",&c)&&c.kind==GYRO_HEADING);
    const char *bad[]={"EXEC,0,IMU_ZERO","EXEC,1,TURN,X,90.0,20","EXEC,1,TURN,W,0.0,20",
        "EXEC,1,TURN,W,90.01,20","EXEC,1,TURN,W,90.0,61","EXEC,1,HEADING,360.0,20","EXEC,1,IMU_ZERO,junk"};
    for(unsigned i=0;i<sizeof(bad)/sizeof(bad[0]);++i) assert(!GyroProtocol_ParseExec(bad[i],&c));
    reset(); feed("EXEC,4000,TURN,W,90.0,20\n"); assert(strstr(application_reply,"ERROR,IMU_NOT_READY"));
    feed("QUERY_IMU,4000\n"); assert(strstr(application_reply,"IMU,4000,NOT_READY"));
    reset(); imu_ready(170); ImuSample sample;
    assert(ImuLink_Read(&sample));
    imu_packet(0x53,10,false); assert(ImuLink_Read(&sample)&&fabsf(sample.yaw-170)<0.01f);
    imu_sample(179,0); imu_sample(-171,0); assert(ImuLink_Read(&sample)&&fabsf(sample.yaw-189)<0.02f);
    tick+=201; assert(!ImuLink_Read(&sample));
    reset(); imu_ready(0);
    /* Fixed protocol vectors independent of the encoder helpers above:
     * 0x4000 -> +90 degrees/+1000 degrees/s; 0xC000 -> negatives. */
    static const uint8_t angle_positive[]={0x55,0x53,0,0,0,0,0,0x40,0,0,0xE8};
    static const uint8_t rate_positive[]={0x55,0x52,0,0,0,0,0,0x40,0,0,0xE7};
    static const uint8_t angle_negative[]={0x55,0x53,0,0,0,0,0,0xC0,0,0,0x68};
    static const uint8_t rate_negative[]={0x55,0x52,0,0,0,0,0,0xC0,0,0,0x67};
    imu_bytes(rate_positive,11); imu_bytes(angle_positive,11); ImuLink_Process();
    assert(ImuLink_Read(&sample)&&fabsf(sample.yaw+90)<0.001f&&fabsf(sample.rate+1000)<0.001f);
    reset(); imu_ready(0);
    imu_bytes(rate_negative,11); imu_bytes(angle_negative,11); ImuLink_Process();
    assert(ImuLink_Read(&sample)&&fabsf(sample.yaw-90)<0.001f&&fabsf(sample.rate-1000)<0.001f);
    reset(); imu_ready(0); feed("EXEC,4001,IMU_ZERO\n"); assert(strstr(application_reply,"ACTION,4001,DONE,0"));
    feed("QUERY_IMU,4001\n");
    const char *zero_reply=strstr(application_reply,"IMU,4001,READY,");
    float zero_yaw=1,zero_rate=1;
    assert(zero_reply&&sscanf(zero_reply,"IMU,4001,READY,%f,%f",&zero_yaw,&zero_rate)==2);
    assert(zero_yaw==0&&zero_rate==0);
    feed("EXEC,4002,TURN,W,90.0,20\n"); assert(GyroTurn_State()==GYRO_RUNNING);
    unsigned start_commands=velocity_frames;
    /* A simple velocity plant verifies convergence, slew limit and closed-loop
     * completion; this is not a claim about real chassis tuning. */
    float yaw=0,rate=0;
    for(unsigned i=0;i<1500U&&GyroTurn_State()==GYRO_RUNNING;++i) {
        rate+=(simulated_rpm*1.4f-rate)*0.25f; yaw+=rate*0.02f;
        tick+=20; imu_sample(yaw,rate); RobotControl_Tick();
        assert(abs(simulated_rpm)<=20);
    }
    assert(GyroTurn_State()==GYRO_DONE&&fabsf(yaw-90)<=1.0f&&velocity_frames>start_commands);
    assert(strstr(application_reply,"ACTION,4002,DONE,0"));
    /* Four equal magnitudes and the expected CW/CCW grouping, synchronized F6. */
    assert(velocity_frame[0]==0&&velocity_frame[1]==0xAA&&velocity_frame[3]==37);
    for(unsigned i=0;i<4;++i) {
        unsigned offset=4+i*8; assert(velocity_frame[offset]==i+1&&velocity_frame[offset+1]==0xF6);
        assert(velocity_frame[offset+3]==velocity_frame[7]&&velocity_frame[offset+4]==velocity_frame[8]);
        assert(velocity_frame[offset+6]==0&&velocity_frame[offset+7]==0x6B);
    }
    feed("EXEC,4003,HEADING,0.0,20\n");
    for(unsigned i=0;i<10U;++i) { tick+=20; imu_sample(yaw,0); RobotControl_Tick(); }
    assert(simulated_rpm<0); // Return toward start, using left rotation.
    feed("CMD,4004,S\n"); assert(GyroTurn_State()==GYRO_ERROR&&!GyroTurn_Busy());
    assert(strstr(application_reply,"ACTION,4003,CANCELLED,STOP"));
    reset(); imu_ready(0); feed("EXEC,4010,TURN,W,90.0,20\n");
    tick+=201; RobotControl_Tick(); assert(GyroTurn_State()==GYRO_ERROR);
    assert(strstr(application_reply,"ERROR,IMU_TIMEOUT")&&stop_frames>=4);
    reset(); imu_ready(0); feed("EXEC,4011,TURN,W,90.0,20\n");
    motor_status[0]=9; tick+=60; imu_sample(0,0); RobotControl_Tick();
    assert(strstr(application_reply,"ERROR,MOTOR_FAULT"));
    reset(); imu_ready(0); feed("EXEC,4012,TURN,W,90.0,20\n");
    fail_position_ack=1; tick+=20; imu_sample(0,0); RobotControl_Tick();
    assert(strstr(application_reply,"ERROR,MOTOR_COMMAND"));
    reset(); imu_ready(0); feed("EXEC,4013,TURN,W,90.0,20\n");
    fail_stop_ack=1; tick+=201; RobotControl_Tick(); assert(GyroTurn_Busy());
    unsigned before=position_frames; feed("MOVE,4014,F,100,20,9889\n"); assert(position_frames==before);
    fail_stop_ack=0; RobotControl_Tick(); assert(!GyroTurn_Busy());
    reset(); imu_ready(0); feed("EXEC,4015,TURN,W,90.0,20\n");
    tick+=20; imu_sample(-25,0); RobotControl_Tick(); assert(strstr(application_reply,"ERROR,IMU_DIRECTION"));
    reset(); imu_ready(0); feed("EXEC,4020,TURN,C,360.0,20\n");
    yaw=rate=0;
    for(unsigned i=0;i<2500U&&GyroTurn_State()==GYRO_RUNNING;++i) {
        rate+=(simulated_rpm*1.4f-rate)*0.25f; yaw+=rate*0.02f;
        tick+=20; imu_sample(yaw,rate); RobotControl_Tick();
    }
    assert(GyroTurn_State()==GYRO_DONE&&fabsf(yaw+360)<=1.0f);
    reset(); imu_ready(0); feed("EXEC,4021,TURN,W,90.0,20\n");
    for(unsigned i=0;i<300U&&GyroTurn_State()==GYRO_RUNNING;++i) { tick+=20; imu_sample(0,0); RobotControl_Tick(); }
    assert(strstr(application_reply,"ERROR,TURN_NO_PROGRESS"));
    reset(); imu_ready(0); feed("EXEC,4022,IMU_ZERO\n");
    tick+=201;
    for(unsigned i=0;i<12U;++i) { tick+=20; imu_sample(0,0); }
    feed("EXEC,4023,HEADING,90.0,20\n"); assert(strstr(application_reply,"ERROR,IMU_NO_REFERENCE"));
}

static void test_gyro_straight(void)
{
    GyroCommand c;
    assert(GyroProtocol_ParseExec("EXEC,4100,DRIVE,F,1600,30,3200",&c)&&c.kind==GYRO_DRIVE&&c.pulses==1600);
    const char *bad[]={"EXEC,1,DRIVE,L,10,20,3200","EXEC,1,DRIVE,F,0,20,3200",
        "EXEC,1,DRIVE,F,1,301,3200","EXEC,1,DRIVE,F,1,20,0",
        "EXEC,1,DRIVE,F,1,20,51201","EXEC,1,DRIVE,B,1,20,3200,junk"};
    for(unsigned i=0;i<sizeof(bad)/sizeof(bad[0]);++i) assert(!GyroProtocol_ParseExec(bad[i],&c));
    for(unsigned backward=0;backward<2;++backward) {
        reset(); imu_ready(179);
        for(unsigned i=0;i<4;++i) wheel_encoder[i]=-100000;
        int64_t position;
        for(unsigned i=1;i<=4;++i) assert(ZDT_Motor_ReadWheelPosition(i,&position)==HAL_OK&&position==-100000);
        feed(backward?"EXEC,4100,DRIVE,B,3200,30,6400\n":"EXEC,4100,DRIVE,F,1600,30,3200\n");
        assert(GyroTurn_State()==GYRO_RUNNING);
        float yaw=179,rate=0; bool corrected=false; double distance=0;
        for(unsigned step=0;step<1500&&GyroTurn_State()==GYRO_RUNNING;++step) {
            double forward=0;
            for(unsigned i=0;i<4;++i) {
                assert(abs(wheel_velocity[i])<=30);
                wheel_encoder[i]+=wheel_velocity[i]*65536.0*0.10/60.0;
                forward-=wheel_velocity[i]/4.0;
            }
            double clockwise=(-wheel_velocity[0]+wheel_velocity[1]+wheel_velocity[2]-wheel_velocity[3])/4.0;
            if(clockwise< -0.1) corrected=true;
            rate+=(float)((clockwise*1.4+(fabs(forward)>0.1?6.0:0))-rate)*0.4f;
            yaw+=rate*0.10f;
            tick+=100; imu_sample(yaw,rate); RobotControl_Tick();
        }
        for(unsigned i=0;i<4;++i) distance-=(wheel_encoder[i]+100000)*(backward?-1:1)/4.0;
        assert(GyroTurn_State()==GYRO_DONE&&corrected&&fabsf(yaw-179)<=1.05f);
        assert(distance>=32767&&distance<33000); // measured 0.5 revolution, yaw cancels
        for(unsigned i=0;i<4;++i) assert(wheel_velocity[i]==0);
        assert(strstr(application_reply,"ACTION,4100,DONE,0"));
    }
    reset(); imu_ready(0); feed("EXEC,4101,DRIVE,F,1600,30,3200\n");
    for(unsigned i=0;i<300&&GyroTurn_State()==GYRO_RUNNING;++i) {
        tick+=20; imu_sample(0,0); RobotControl_Tick();
    }
    assert(strstr(application_reply,"ERROR,DRIVE_NO_PROGRESS")); // Commanded RPM/time cannot complete distance.
    reset(); imu_ready(0); fail_encoder=1;
    feed("EXEC,4102,DRIVE,F,1600,30,3200\n"); assert(strstr(application_reply,"ERROR,ODOM_READ")&&velocity_frames==0);
    reset(); imu_ready(0); feed("EXEC,4103,DRIVE,B,1600,30,3200\n");
    fail_encoder=1; tick+=20; imu_sample(0,0); RobotControl_Tick();
    assert(strstr(application_reply,"ERROR,ODOM_READ")&&stop_frames>=4);
    reset(); imu_ready(0); feed("EXEC,4104,DRIVE,F,1600,30,3200\n");
    tick+=201; RobotControl_Tick(); assert(strstr(application_reply,"ERROR,IMU_TIMEOUT"));
    reset(); imu_ready(0); feed("EXEC,4105,DRIVE,F,1600,30,3200\n");
    feed("CMD,4106,S\n"); assert(!GyroTurn_Busy()&&strstr(application_reply,"ACTION,4105,CANCELLED,STOP"));
    reset(); imu_ready(0); feed("EXEC,4107,DRIVE,F,1600,30,3200\n");
    for(unsigned i=0;i<4;++i) wheel_encoder[i]=1000;
    tick+=20; imu_sample(0,0); RobotControl_Tick(); assert(strstr(application_reply,"ERROR,ODOM_DIRECTION"));
}

static void test_startup_stop_busy_reason(void)
{
    reset();
    fail_stop_ack=1;
    memset(motor_status,0,sizeof(motor_status)); // Disabled drivers may reject STOP.
    motor_status[1]=1; // Driver 2 remains enabled and cannot confirm a stop.
    GyroTurn_SafeBoot();
    assert(GyroTurn_Busy()&&strcmp(GyroTurn_Reason(),"STARTUP_STOP")==0);
    unsigned before=velocity_frames;
    feed("EXEC,4900,TURN,W,90.0,10\n");
    assert(strstr(application_reply,"ACTION,4900,BUSY,STARTUP_STOP\n"));
    assert(velocity_frames==before);
    fail_stop_ack=0; RobotControl_Tick(); // Retry confirms the remaining driver's stop.
    assert(!GyroTurn_Busy());
    feed("QUERY,4901\n");
    assert(strstr(application_reply,"STATUS,4901,IDLE,1,0\n"));
}
static void test_motor_fault_feedback(void)
{
    /* Real chassis driver failures must identify each queried address. */
    reset(); fail_encoder=1;
    int64_t position;
    for(uint8_t id=1;id<=4;++id) assert(ZDT_Motor_ReadWheelPosition(id,&position)!=HAL_OK);
    RobotControl_Tick();
    for(unsigned id=1;id<=4;++id) {
        char expected[64]; snprintf(expected,sizeof(expected),"MOTOR_DIAG,CHASSIS,%u,POSITION_READ",id);
        assert(strstr(application_reply,expected));
    }
    /* A combined command must not falsely accuse wheel 1. */
    reset(); fail_position_ack=1;
    int16_t speeds[4]={10,10,10,10};
    assert(ZDT_Motor_SetWheelSpeeds(speeds,10)!=HAL_OK); RobotControl_Tick();
    assert(strstr(application_reply,"MOTOR_DIAG,CHASSIS,0,SPEED_COMMAND"));
    /* Both arm axes are checked even when the first origin read fails. */
    reset(); RobotControl_Init(&command_uart,true); fail_origin_read=1; origin_reads=0;
    feed("CMD,123,I\n"); assert(origin_reads==2); RobotControl_Tick();
    assert(strstr(application_reply,"MOTOR_DIAG,ARM,1,POSITION_READ"));
    assert(strstr(application_reply,"MOTOR_DIAG,ARM,2,POSITION_READ"));
    assert(!strstr(application_reply,"ORIGIN_READ"));
    application_reply[0]=0; tick+=500; RobotControl_Tick();
    assert(!strstr(application_reply,"MOTOR_DIAG,")); // repeated failures throttled
    tick+=2000; RobotControl_Tick();
    assert(strstr(application_reply,"MOTOR_DIAG,ARM,1,POSITION_READ"));
    /* Motor namespaces remain distinct even with identical numeric addresses. */
    reset(); LiftMotor_Fault(1,"STOP"); ZDT_Motor_Fault(1,"STATUS_READ"); RobotControl_Tick();
    assert(strstr(application_reply,"MOTOR_DIAG,ARM,1,STOP"));
    assert(strstr(application_reply,"MOTOR_DIAG,CHASSIS,1,STATUS_READ"));
}
int main(void)
{
    test_startup_stop_busy_reason();
    test_motor_fault_feedback();
    test_gyro_straight();
    test_gyro_link_and_controller();
#ifdef _WIN32
    _set_error_mode(_OUT_TO_STDERR);
    signal(SIGABRT, exit);
#endif
    test_transient_stall_status();
    test_vector_protocol_and_speeds(); test_vector_grab(); test_vector_release(); test_sequence_optimized_timing(); test_vector_edge_cases_and_faults();
    test_parser(); test_direction_frames(); test_completion_and_duplicates(); test_timeout_stop_fault();
    test_stream_recovery();
    test_receive_restart();
    test_servo_commands_without_replies();
    test_arm_distance_control();
    test_lift_calibration();
    test_lift_distance_control();
    test_camera_alignment();
    test_ring_alignment();
    test_alignment_compensation_stop();
    test_absolute_pose();
    test_parallel_alignment();
    test_parallel_pixel_tolerance();
    test_manual_gripper();
    test_configured_alignment();
    test_app_origin_initialization();
    test_manual_base_speed();
    test_base_acceleration();
    test_initial_base_angle_updates();
    test_turn_cards();
    test_center_sample_average();
    test_center_axis_stages();
    test_fractional_center_mean();
    test_joint_alignment();
    test_custom_center();
    test_arm_speed_300_rpm();
    test_decimal_camera_chain();
    test_grab_initial_base_ramp();
    test_autonomous_feedback();
    puts("PASS: parsing, four-wheel FD frames, completion, autonomous move, grab/arm, camera alignment limits, duplicate/busy, fault and stop retry");
    return 0;
}
