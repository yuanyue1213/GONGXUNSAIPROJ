/* ILI9341 320x240横屏：四组任务码同时显示，80像素高七段数字。
 * 上排第一批，下排第二批；每排左颜色、右工位。无字库、无整屏缓冲。
 * 初始化参数取自厂商2.4/2.8寸通用例程，采用0x01软件复位。 */
#include "task_display.h"
#include "lcd_bus.h"
#include "robot_config.h"
#include <string.h>
enum { BLACK=0x0000,WHITE=0xffff,GREEN=0x07e0,GRAY=0x3186,BLUE=0x001f,AMBER=0xfd20,RED=0xf800 };
static bool ready,has_code;
static TaskCode task;
static uint8_t progress[2][3];
static void command(uint8_t reg,const uint8_t *data,size_t count)
{
    LcdBus_Write(false,&reg,1);
    if(count) LcdBus_Write(true,data,count);
}
static void fill(unsigned x,unsigned y,unsigned width,unsigned height,uint16_t color)
{
    if(!width||!height||x+width>320||y+height>240) return;
    uint8_t cols[]={x>>8,x,(x+width-1)>>8,x+width-1};
    uint8_t rows[]={y>>8,y,(y+height-1)>>8,y+height-1};
    command(0x2a,cols,4); command(0x2b,rows,4); command(0x2c,NULL,0);
    uint8_t line[640]; /* 一行RGB565，不占用整屏RAM。 */
    for(unsigned i=0;i<width;++i) { line[2*i]=color>>8; line[2*i+1]=color; }
    for(unsigned i=0;i<height;++i) LcdBus_Write(true,line,2*width);
}
static void digit(unsigned x,unsigned y,unsigned value,uint16_t color)
{
    /* a,b,c,d,e,f,g；每个有效数字的最上/最下笔画覆盖完整80像素。 */
    static const uint8_t bits[]={0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d};
    static const uint8_t rects[7][4]={
        {0,0,36,7},{29,0,7,40},{29,40,7,40},{0,73,36,7},
        {0,40,7,40},{0,0,7,40},{0,37,36,7}};
    uint8_t mask=value<=6?bits[value]:0x40;
    fill(x,y,36,80,BLACK);
    for(unsigned i=0;i<7;++i) if(mask&(1U<<i))
        fill(x+rects[i][0],y+rects[i][1],rects[i][2],rects[i][3],color);
}
static void item_draw(unsigned batch,unsigned item,bool redraw_digit)
{
    unsigned y=12+batch*116;
    for(unsigned group=0;group<2;++group) {
        unsigned x=16+group*164+item*42;
        if(redraw_digit) digit(x,y,has_code?(group?task.places[batch][item]:task.colors[batch][item]):7,
                              progress[batch][item]==15?GREEN:WHITE);
        for(unsigned stage=0;stage<4;++stage)
            fill(x+stage*9,y+84,7,8,(progress[batch][item]&(1U<<stage))?GREEN:GRAY);
    }
}
void TaskDisplay_State(DisplayState state)
{
    if(!ready) return;
    uint16_t color=state==DISPLAY_DONE?GREEN:state==DISPLAY_ERROR?RED:state==DISPLAY_RUNNING?AMBER:BLUE;
    fill(0,0,320,4,color); fill(0,236,320,4,color);
    fill(0,4,4,232,color); fill(316,4,4,232,color);
}
void TaskDisplay_Task(const TaskCode *code)
{
    if(!ready) return;
    has_code=code!=NULL;
    if(code) task=*code;
    memset(progress,0,sizeof(progress));
    for(unsigned b=0;b<2;++b) for(unsigned i=0;i<3;++i) item_draw(b,i,true);
    for(unsigned i=0;i<24;++i) fill(16+i*12,230,10,4,GRAY);
}
void TaskDisplay_Progress(unsigned batch,unsigned item,unsigned stage)
{
    if(!ready||!has_code||batch>=2||item>=3||stage>=4) return;
    uint8_t bit=1U<<stage;
    if(progress[batch][item]&bit) return;
    progress[batch][item]|=bit;
    item_draw(batch,item,progress[batch][item]==15);
    fill(16+(batch*12+stage*3+item)*12,230,10,4,GREEN);
}
void TaskDisplay_Init(void)
{
    LcdBus_Init();
    command(0x01,NULL,0); LcdBus_Delay(100);
    /* 每条记录为长度、寄存器、数据；末尾0结束。 */
    static const uint8_t init[]={
        3,0xcf,0x00,0xd9,0x30, 4,0xed,0x64,0x03,0x12,0x81,
        3,0xe8,0x85,0x10,0x78, 5,0xcb,0x39,0x2c,0x00,0x34,0x02,
        1,0xf7,0x20, 2,0xea,0x00,0x00, 1,0xc0,0x21, 1,0xc1,0x12,
        2,0xc5,0x32,0x3c, 1,0xc7,0xc1, 1,0x36,LCD_MADCTL,
        1,0x3a,0x55, 2,0xb1,0x00,0x18, 2,0xb6,0x0a,0xa2,
        1,0xf2,0x00, 1,0x26,0x01,
        15,0xe0,0x0f,0x20,0x1e,0x09,0x12,0x0b,0x50,0xba,0x44,0x09,0x14,0x05,0x23,0x21,0x00,
        15,0xe1,0x00,0x19,0x19,0x00,0x12,0x07,0x2d,0x28,0x3f,0x02,0x0a,0x08,0x25,0x2d,0x0f,0};
    for(unsigned i=0;init[i];) {
        unsigned n=init[i++]; uint8_t reg=init[i++];
        command(reg,&init[i],n); i+=n;
    }
    command(0x11,NULL,0); LcdBus_Delay(120); command(0x29,NULL,0);
    fill(0,0,320,240,BLACK);
    fill(158,4,2,224,GRAY); fill(4,118,312,2,GRAY);
    ready=true; TaskDisplay_Task(NULL); TaskDisplay_State(DISPLAY_WAIT);
    LcdBus_Backlight();
}
