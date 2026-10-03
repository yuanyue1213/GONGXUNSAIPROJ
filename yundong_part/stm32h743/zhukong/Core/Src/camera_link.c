#include "camera_link.h"
/* UART4 interrupt only buffers bytes. Parse in main loop, independently of ESP32 USART3. */
#define CAMERA_RX_SIZE 256U
static UART_HandleTypeDef *s_uart;
static uint8_t s_byte;
static volatile uint8_t s_buffer[CAMERA_RX_SIZE];
static volatile uint16_t s_head, s_tail;
static volatile bool s_fault, s_restart;
static char s_frame[64];
static uint32_t s_ring_index;
static unsigned s_length;
static bool s_drop, s_valid;
static CameraCenter s_center, s_rings[3];
static void arm_receive(void)
{
    if (s_uart == NULL) return;
    HAL_StatusTypeDef result = HAL_UART_Receive_IT(s_uart, &s_byte, 1U);
    s_restart = result != HAL_OK && result != HAL_BUSY;
}
void CameraLink_Init(UART_HandleTypeDef *uart)
{
    s_uart = uart; s_head = s_tail = 0U; s_fault = false;
    s_length = 0U; s_drop = s_valid = false; s_restart = false; s_ring_index = 0U;
    if (uart != NULL) {
        HAL_NVIC_SetPriority(UART4_IRQn, 5U, 0U);
        HAL_NVIC_EnableIRQ(UART4_IRQn);
        arm_receive();
    }
}
bool CameraLink_Ready(void) { return s_uart != NULL; }
void CameraLink_SelectTarget(uint32_t ring_index)
{
    s_ring_index = ring_index;
    CameraLink_Discard();
}
void CameraLink_Discard(void)
{
    uint32_t interrupts = __get_PRIMASK(); __disable_irq();
    s_tail = s_head; s_fault = false; __set_PRIMASK(interrupts);
    s_length = 0U; s_drop = true; s_valid = false;
}
bool CameraLink_RxCallback(UART_HandleTypeDef *uart)
{
    if (s_uart == NULL || uart != s_uart) return false;
    uint16_t next = (uint16_t)((s_head + 1U) % CAMERA_RX_SIZE);
    if (next == s_tail) s_fault = true;
    else { s_buffer[s_head] = s_byte; s_head = next; }
    arm_receive(); return true;
}
bool CameraLink_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (s_uart == NULL || uart != s_uart) return false;
    s_fault = true;
    __HAL_UART_CLEAR_OREFLAG(uart); __HAL_UART_CLEAR_NEFLAG(uart); __HAL_UART_CLEAR_FEFLAG(uart);
    arm_receive(); return true;
}
void CameraLink_Process(void)
{
    if (s_uart == NULL) return;
    if (s_restart || s_uart->RxState == HAL_UART_STATE_READY) arm_receive();
    if (s_fault) { CameraLink_Discard(); return; }
    while (s_tail != s_head) {
        if (s_fault) { CameraLink_Discard(); return; }
        char c = (char)s_buffer[s_tail]; s_tail = (uint16_t)((s_tail + 1U) % CAMERA_RX_SIZE);
        if (c == '\n') {
            if (!s_drop) {
                if (s_length > 0U && s_frame[s_length - 1U] == '\r') --s_length;
                s_frame[s_length] = '\0';
                CameraCenter center;
                CameraCenter rings[3];
                if (s_ring_index == 0U && CameraProtocol_ParseCenter(s_frame, &center)) {
                    s_center = center; s_valid = true;
                } else if (s_ring_index >= 1U && s_ring_index <= 4U &&
                           CameraProtocol_ParseRings(s_frame, rings)) {
                    memcpy(s_rings, rings, sizeof(s_rings));
                    if (s_ring_index <= 3U) s_center = rings[s_ring_index - 1U];
                    s_valid = true;
                }
            }
            s_length = 0U; s_drop = false;
        } else if (!s_drop) {
            if (s_length < sizeof(s_frame) - 1U) s_frame[s_length++] = c;
            else s_drop = true;
        }
    }
}
bool CameraLink_TakeCenter(CameraCenter *center)
{
    if (!s_valid || s_ring_index == 4U) return false;
    *center = s_center; s_valid = false; return true;
}

bool CameraLink_TakeRings(CameraCenter rings[3])
{
    if (!s_valid || s_ring_index != 4U) return false;
    memcpy(rings, s_rings, sizeof(s_rings)); s_valid = false; return true;
}
