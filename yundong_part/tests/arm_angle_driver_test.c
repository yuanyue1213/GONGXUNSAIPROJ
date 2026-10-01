#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "lift_motor.h"

static UART_HandleTypeDef uart = {2, HAL_UART_STATE_READY};
static uint32_t tick;
static unsigned firmware_length, response_length, response_index, position_count;
static uint8_t response[37], position[16];
static unsigned position_length;
static int fail_enable, fail_position;
static uint32_t real_position;
static uint8_t real_sign;
static int fail_read;
uint32_t HAL_GetTick(void) { return tick; }
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *u, uint8_t *data, uint16_t len, uint32_t timeout)
{
    (void)timeout; assert(u == &uart && (data[0] == 1 || data[0] == 2));
    memset(response, 0, sizeof(response)); response_index = 0;
    response[0] = data[0]; response[1] = data[1];
    if (data[1] == 0x42) {
        assert(len == 4); response_length = firmware_length;
        response[2] = (uint8_t)firmware_length;
        response[firmware_length - 1] = 0x6B;
    } else if (data[1] == 0x36) {
        assert(len == 3); response_length = fail_read ? 0 : 8;
        response[2] = real_sign;
        response[3] = (uint8_t)(real_position >> 24); response[4] = (uint8_t)(real_position >> 16);
        response[5] = (uint8_t)(real_position >> 8); response[6] = (uint8_t)real_position; response[7] = 0x6B;
    } else {
        response_length = 4; response[2] = 2; response[3] = 0x6B;
        if (data[1] == 0xF3 && fail_enable) response_length = 0;
        if (data[1] == 0xFD) {
            ++position_count; assert(len <= sizeof(position));
            memcpy(position, data, len); position_length = len;
            if (fail_position) response_length = 0;
        }
    }
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *u, uint8_t *data, uint16_t len, uint32_t timeout)
{
    assert(u == &uart);
    if (response_index + len > response_length) { tick += timeout; return HAL_TIMEOUT; }
    memcpy(data, response + response_index, len); response_index += len; return HAL_OK;
}
static void reset(unsigned length)
{
    tick = position_count = 0; firmware_length = length;
    fail_enable = fail_position = 0; LiftMotor_Init(&uart);
    fail_read = 0; real_position = 0; real_sign = 0;
}
int main(void)
{
    int64_t angle;
    reset(0x21); real_position = 131072;
    assert(ArmMotor_ReadPosition(2, &angle) && angle == 7200);
    real_sign = 1; assert(ArmMotor_ReadPosition(1, &angle) && angle == -7200);
    assert(ArmMotor_MoveAbsolute(1, -3600, 5, 3200) == HAL_OK);
    assert(position[2] == 1 && position[10] == 1 && position[8] == 0x0C && position[9] == 0x80);
    assert(ArmMotor_MoveAbsolute(2, 0, 5, 3200) == HAL_OK);
    assert(position[2] == 0 && position[10] == 1 && position[9] == 0);
    assert(ArmMotor_MoveAbsolute(2, (int64_t)UINT32_MAX, 5, 51200) != HAL_OK);
    reset(0x25); real_position = 12345; real_sign = 1;
    assert(ArmMotor_ReadPosition(1, &angle) && angle == -12345);
    assert(ArmMotor_MoveAbsolute(2, -12345, 10, 3200) == HAL_OK);
    assert(position[2] == 1 && position[13] == 1 && position[11] == 0x30 && position[12] == 0x39);
    real_sign = 2; assert(!ArmMotor_ReadPosition(1, &angle));
    real_sign = 0; fail_read = 1; assert(!ArmMotor_ReadPosition(1, &angle));
    assert(!ArmMotor_ReadPosition(3, &angle));
    reset(0x21);
    assert(ArmMotor_MoveAngle('E', 900, 10, 3200) == HAL_OK);
    const uint8_t emm[] = {2,0xFD,0,0,10,10,0,0,3,0x20,2,0,0x6B};
    assert(position_length == sizeof(emm) && memcmp(position, emm, sizeof(emm)) == 0);
    assert(ArmMotor_MoveAngle('C', 900, 10, 6400) == HAL_OK);
    assert(position[2] == 1 && position[8] == 6 && position[9] == 0x40);
    reset(0x25);
    assert(ArmMotor_MoveAngle('C', 900, 10, 3200) == HAL_OK);
    const uint8_t x[] = {2,0xFD,1,0,0xC8,0,0xC8,0,100,0,0,3,0x84,2,0,0x6B};
    assert(position_length == sizeof(x) && memcmp(position, x, sizeof(x)) == 0);
    reset(0x21); fail_enable = 1;
    assert(ArmMotor_MoveAngle('E', 900, 10, 3200) != HAL_OK && position_count == 0);
    reset(0x21); fail_position = 1;
    assert(ArmMotor_MoveAngle('E', 900, 10, 3200) != HAL_OK && position_count == 1);
    reset(0x21);
    assert(ArmMotor_MoveAngle('E', 1, 10, 200) != HAL_OK && position_count == 0);
    assert(ArmMotor_MoveAngle('E', 36001, 10, 3200) != HAL_OK);
    reset(0x21);
    assert(LiftMotor_MoveAngle('U', 900, 5, 3200) == HAL_OK);
    const uint8_t lift_emm[] = {1,0xFD,1,0,5,10,0,0,3,0x20,2,0,0x6B};
    assert(position_length == sizeof(lift_emm) && memcmp(position, lift_emm, sizeof(lift_emm)) == 0);
    assert(LiftMotor_Stop() == HAL_OK && position_count == 1);
    assert(LiftMotor_MoveAngle('D', 1800, 10, 6400) == HAL_OK);
    assert(position[0] == 1 && position[2] == 0 && position[8] == 0x0C && position[9] == 0x80);
    reset(0x25);
    assert(LiftMotor_MoveAngle('D', 900, 5, 3200) == HAL_OK);
    const uint8_t lift_x[] = {1,0xFD,0,0,0xC8,0,0xC8,0,50,0,0,3,0x84,2,0,0x6B};
    assert(position_length == sizeof(lift_x) && memcmp(position, lift_x, sizeof(lift_x)) == 0);
    reset(0x21); fail_enable = 1;
    assert(LiftMotor_MoveAngle('U', 900, 5, 3200) != HAL_OK && position_count == 0);
    reset(0x21); fail_position = 1;
    assert(LiftMotor_MoveAngle('U', 900, 5, 3200) != HAL_OK && position_count == 1);
    assert(LiftMotor_Stop() == HAL_OK);
    reset(0x21);
    assert(LiftMotor_MoveAngle('E', 900, 5, 3200) != HAL_OK && position_count == 0);
    puts("PASS: ID1/ID2 Emm/X FD frames, angle conversion, directions and ACK failures");
    return 0;
}
