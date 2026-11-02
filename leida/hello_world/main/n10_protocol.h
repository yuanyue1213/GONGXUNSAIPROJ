#ifndef N10_PROTOCOL_H
#define N10_PROTOCOL_H
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#define N10_FRAME_SIZE 58
typedef struct {
    uint8_t data[N10_FRAME_SIZE];
    size_t used;
    uint32_t invalid;
} n10_parser_t;
/* Bytewise sliding resync handles noise, partial frames and overlapping headers. */
static inline void n10_feed(n10_parser_t *p, uint8_t byte,
                            void (*on_frame)(const uint8_t *))
{
    p->data[p->used++] = byte;
    while (p->used) {
        if (p->data[0] != 0xA5 ||
            (p->used >= 2 && p->data[1] != 0x5A) ||
            (p->used >= 3 && p->data[2] != N10_FRAME_SIZE)) {
            memmove(p->data, p->data + 1, --p->used);
            continue;
        }
        if (p->used < N10_FRAME_SIZE) break;
        uint8_t sum = 0;
        for (int i = 0; i < N10_FRAME_SIZE - 1; ++i) sum += p->data[i];
        unsigned start = ((unsigned)p->data[5] << 8) | p->data[6];
        unsigned end = ((unsigned)p->data[55] << 8) | p->data[56];
        if (sum == p->data[57] && start < 36000 && end < 36000) {
            on_frame(p->data);
            p->used = 0;
        } else {
            p->invalid++;
            memmove(p->data, p->data + 1, --p->used);
        }
    }
}
#endif
