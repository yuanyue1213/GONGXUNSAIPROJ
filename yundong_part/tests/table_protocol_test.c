#include <assert.h>
#include <stdio.h>
#include "../shared/task_protocol.h"
#include "../shared/raw_pick_protocol.h"
int main(void)
{
    TaskCameraRequest c; TaskVision v;
    assert(TaskProtocol_Camera("CAM,77,TABLE,156,256.0,160.0",&c));
    assert(c.kind==TASK_TABLE&&c.target==156&&c.has_anchor&&c.anchor.x==256&&c.anchor.y==160);
    assert(TaskProtocol_Camera("CAM,78,TABLE,516",&c)&&!c.has_anchor);
    assert(TaskProtocol_Vision("VISION,77,TABLE,156,255.4,159.8",&v));
    assert(v.kind==TASK_TABLE&&v.target==156&&v.request==77);
    assert(TaskProtocol_Camera("CAM,79,OCCUPIED,516,256.0,160.0",&c));
    assert(c.kind==TASK_OCCUPIED&&c.target==516&&c.has_anchor);
    assert(TaskProtocol_Vision("VISION,79,OCCUPIED,516,255.4,159.8",&v)&&v.kind==TASK_OCCUPIED);
    assert(!TaskProtocol_Camera("CAM,79,OCCUPIED,111",&c));
    assert(!TaskProtocol_Vision("VISION,79,OCCUPIED,516,512.0,159.8",&v));
    for(unsigned a=1;a<=6;++a) for(unsigned b=1;b<=6;++b) for(unsigned d=1;d<=6;++d)
        assert(TaskProtocol_TableColors(a*100+b*10+d)==(a!=b&&a!=d&&b!=d));
    const char *bad[]={"CAM,77,TABLE,155","CAM,77,TABLE,15","CAM,77,TABLE,157",
        "CAM,0,TABLE,156","CAM,77,TABLE,156,512.0,160.0","CAM,77,TABLE,156,256.0,320.0",
        "CAM,77,TABLE,156,256.0,160.0,junk"};
    for(unsigned i=0;i<sizeof(bad)/sizeof(bad[0]);++i) assert(!TaskProtocol_Camera(bad[i],&c));
    assert(!TaskProtocol_Vision("VISION,77,TABLE,111,256.0,160.0",&v));
    assert(!TaskProtocol_Vision("VISION,77,TABLE,156,256.0,160.0,junk",&v));
    assert(TaskProtocol_Camera("CAM,1,MAT,5,256.0,160.0",&c)&&c.kind==TASK_MAT);
    assert(TaskProtocol_Camera("CAM,2,RING,3",&c)&&c.kind==TASK_RING);
    assert(TaskProtocol_Vision("VISION,3,NONE",&v)&&v.kind==TASK_NONE);
    assert(TaskProtocol_Camera("CAM,80,PICK,6",&c)&&c.kind==TASK_PICK&&c.target==6);
    assert(TaskProtocol_Vision("VISION,80,PICK,6,3",&v)&&v.kind==TASK_PICK&&v.position==3);
    assert(!TaskProtocol_Camera("CAM,80,PICK,7",&c));
    assert(!TaskProtocol_Camera("CAM,80,PICK,1,256.0,160.0",&c));
    assert(!TaskProtocol_Vision("VISION,80,PICK,6,0",&v));
    assert(!TaskProtocol_Vision("VISION,80,PICK,6,4",&v));
    assert(!TaskProtocol_Vision("VISION,80,PICK,6,3,junk",&v));
    assert(TaskProtocol_Camera("CAM,81,NEAR,5",&c)&&c.kind==TASK_NEAR&&c.target==5);
    assert(TaskProtocol_Vision("VISION,81,NEAR,5,2",&v)&&v.kind==TASK_NEAR);
    assert(!TaskProtocol_Vision("VISION,81,NEAR,5,3",&v));
    assert(!TaskProtocol_Camera("CAM,81,NEAR,7",&c));
    RobotSequenceCommand raw;
    assert(RawPick_Ready("RAW_READY,1,A,1100,-50,200,-40,20,60,0,60,0,248,140",&raw));
    assert(!RawPick_Ready("RAW_READY,1,P,1100,-50,200,-40,20,60,0,60,0,248,140",&raw));
    assert(RawPick_Take("RAW_TAKE,2"));
    assert(!RawPick_Take("RAW_TAKE,0")&&!RawPick_Take("RAW_TAKE,2,junk"));
    puts("PASS: TABLE requests/responses, three distinct colors, pixel bounds, legacy MAT/RING/NONE");
}
