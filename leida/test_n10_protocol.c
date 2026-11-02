/* Host test of the exact UART stream parser compiled into the ESP firmware. */
#include <assert.h>
#include <stdio.h>
#include "hello_world/main/n10_protocol.h"
static int received;
static void accept(const uint8_t *frame) { assert(frame[7] == 3); received++; }
static void feed(n10_parser_t *p, const uint8_t *data, size_t n)
{ for (size_t i = 0; i < n; i++) n10_feed(p, data[i], accept); }
int main(void)
{
    n10_parser_t p = {0};
    uint8_t frame[58] = {0xA5, 0x5A, 0x3A};
    frame[7] = 3;
    for (int i = 0; i < 57; i++) frame[57] += frame[i];
    uint8_t noise[] = {0, 0xA5, 0xA5};
    feed(&p, noise, sizeof(noise));
    feed(&p, frame, 13);
    assert(received == 0);
    feed(&p, frame + 13, 45);
    assert(received == 1);
    uint8_t bad[58]; memcpy(bad, frame, 58); bad[20] ^= 1;
    feed(&p, bad, 58);
    feed(&p, frame, 58);
    assert(received == 2 && p.invalid == 1);
    /* Truncation followed immediately by a new frame must resynchronize. */
    feed(&p, frame, 21);
    feed(&p, frame, 58);
    assert(received == 3);
    for (int i = 0; i < 1000; i++) feed(&p, frame, 58);
    assert(received == 1003 && p.used == 0);
    puts("PASS: noise, overlap, split, checksum, truncation and back-to-back frames");
    return 0;
}
