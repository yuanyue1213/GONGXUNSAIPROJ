/* 三端共用的一键启动帧校验：只接受规定格式及编号范围，避免三端规则不一致。 */
#ifndef START_PROTOCOL_H
#define START_PROTOCOL_H
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
/* Network and UART boundary: exactly START,<nonzero uint32>, without newline. */
static inline bool StartProtocol_Parse(const char *line, uint32_t *id)
{
    if(strncmp(line,"START,",6U)!=0) return false;
    const char *p=line+6U; uint32_t value=0;
    if(*p<'0'||*p>'9') return false;
    while(*p>='0'&&*p<='9') {
        unsigned digit=(unsigned)(*p++-'0');
        if(value>(UINT32_MAX-digit)/10U) return false;
        value=value*10U+digit;
    }
    if(*p||!value) return false;
    *id=value; return true;
}
#endif
