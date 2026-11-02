/* PC/STM32共用的路径校验约定：CRC32覆盖BEGIN元数据和全部STEP行（含LF）。 */
#ifndef NAV_PROTOCOL_H
#define NAV_PROTOCOL_H
#include <stddef.h>
#include <stdint.h>
static inline uint32_t Nav_Crc32(uint32_t crc,const char *data,size_t count)
{
    while(count--) {
        crc^=(uint8_t)*data++;
        for(unsigned bit=0;bit<8;++bit) crc=(crc>>1)^((crc&1U)?0xEDB88320U:0U);
    }
    return crc;
}
#endif
