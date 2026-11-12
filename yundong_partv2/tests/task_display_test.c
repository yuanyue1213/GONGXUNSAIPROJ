#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "task_display.h"
#include "lcd_bus.h"
#include "robot_config.h"
static uint16_t frame[240][320];
static unsigned reg,x0,x1,y0,y1,pixels,expected,commands,delay_ms;
static bool light;
void LcdBus_Init(void) {}
void LcdBus_Delay(uint32_t ms) { delay_ms+=ms; }
void LcdBus_Backlight(void) { assert(delay_ms>=220); light=true; }
void LcdBus_Write(bool data,const uint8_t *bytes,size_t count)
{
    if(!data) {
        assert(count==1);
        if(reg==0x2c) assert(pixels==expected);
        reg=*bytes; ++commands;
        if(reg==0x2c) { pixels=0; expected=(x1-x0+1)*(y1-y0+1); }
    } else if(reg==0x2a||reg==0x2b) {
        assert(count==4);
        unsigned first=bytes[0]*256U+bytes[1],last=bytes[2]*256U+bytes[3];
        assert(first<=last);
        if(reg==0x2a) { assert(last<320); x0=first; x1=last; }
        else { assert(last<240); y0=first; y1=last; }
    } else if(reg==0x2c) {
        assert(count%2==0&&pixels+count/2<=expected);
        for(unsigned i=0;i<count;i+=2,++pixels)
            frame[y0+pixels/(x1-x0+1)][x0+pixels%(x1-x0+1)]=(uint16_t)(bytes[i]*256U+bytes[i+1]);
    } else if(reg==0x36) assert(count==1&&*bytes==LCD_MADCTL);
    else if(reg==0x3a) assert(count==1&&*bytes==0x55);
}
static void preview(const char *path)
{
    FILE *f=fopen(path,"wb"); assert(f);
    fprintf(f,"P6\n320 240\n255\n");
    for(unsigned y=0;y<240;++y) for(unsigned x=0;x<320;++x) {
        unsigned p=frame[y][x];
        uint8_t rgb[]={(p>>11)*255/31,((p>>5)&63)*255/63,(p&31)*255/31};
        assert(fwrite(rgb,1,3,f)==3);
    }
    fclose(f);
}
int main(int argc,char **argv)
{
    TaskDisplay_Progress(0,0,0); assert(!commands);
    TaskDisplay_Init(); assert(light&&frame[0][0]==0x001f);
    TaskCode code; assert(TaskProtocol_Code(MISSION_FIXED_QR_TEXT,&code));
    TaskDisplay_Task(&code);
    for(unsigned b=0;b<2;++b) for(unsigned group=0;group<2;++group) for(unsigned item=0;item<3;++item) {
        unsigned x=16+group*164+item*42,y=12+b*116,top=80,bottom=0;
        for(unsigned dy=0;dy<80;++dy) for(unsigned dx=0;dx<36;++dx)
            if(frame[y+dy][x+dx]==0xffff) { if(dy<top) top=dy; if(dy>bottom) bottom=dy; }
        assert(top==0&&bottom==79); /* 每一位任务数字实际笔画高80px。 */
    }
    if(argc==2) preview(argv[1]);
    TaskDisplay_State(DISPLAY_RUNNING); assert(frame[0][0]==0xfd20);
    for(unsigned stage=0;stage<4;++stage) {
        TaskDisplay_Progress(0,0,stage);
        assert(frame[96][16+stage*9]==0x07e0);
        assert(frame[96][180+stage*9]==0x07e0);
        assert(frame[230][16+stage*36]==0x07e0);
    }
    assert(frame[12][45]==0x07e0); /* 第一批第一件全部完成，数字1变绿。 */
    unsigned before=commands;
    TaskDisplay_Progress(0,0,3); TaskDisplay_Progress(2,0,0); TaskDisplay_Progress(0,3,0); TaskDisplay_Progress(0,0,4);
    assert(commands==before);
    TaskDisplay_State(DISPLAY_DONE); assert(frame[0][0]==0x07e0);
    TaskDisplay_State(DISPLAY_ERROR); assert(frame[0][0]==0xf800);
    TaskDisplay_Task(NULL); before=commands; TaskDisplay_Progress(0,0,0); assert(commands==before);
    for(unsigned y=230;y<234;++y) assert(frame[y][16]==0x3186);
    /* 覆盖全部六色数字的绘制和两批进度。 */
    assert(TaskProtocol_Code("456+123+654+321",&code)); TaskDisplay_Task(&code);
    for(unsigned b=0;b<2;++b) for(unsigned i=0;i<3;++i) for(unsigned s=0;s<4;++s) TaskDisplay_Progress(b,i,s);
    assert(pixels==expected);
    puts("PASS: ILI9341 initialization, bounded RGB565 windows, all 12 digits at 80px, per-item four-stage progress, duplicate/invalid guards and reset");
}
