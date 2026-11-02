#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../stm32/yundongv2/Tests/navigation_only.c"

static unsigned legs,prepared,references,stops,fail_leg;
static bool fail_reference;
static char status[128];
void StartLink_Publish(const char *line)
{
    snprintf(status,sizeof(status),"%s",line);
    assert(MissionRunner_Running());
    MissionRunner_Run(999); /* 状态回调期间不能重入另一轮。 */
}
bool Motion_Reference(void) { ++references; return !fail_reference; }
bool RoutePlan_Prepare(uint32_t id) { assert(id==42); ++prepared; return true; }
bool RoutePlan_Run(unsigned leg)
{
    assert(leg==++legs&&leg<=8);
    return leg!=fail_leg;
}
void Motion_Stop(void) { ++stops; }
int main(void)
{
    MissionRunner_Run(42);
    assert(!MissionRunner_Running()&&legs==8&&prepared==1&&references==1&&!stops);
    assert(strstr(status,"AUTO,42,DONE,8,0,0,0,NAV_ONLY"));
    legs=0; fail_leg=3;
    MissionRunner_Run(42);
    assert(!MissionRunner_Running()&&legs==3&&prepared==2&&stops==1);
    assert(strstr(status,"STOPPED,3,0,0,0,COMMAND_FAILED"));
    legs=0; fail_reference=true;
    MissionRunner_Run(42);
    assert(!MissionRunner_Running()&&!legs&&prepared==2&&stops==2);
    puts("PASS: navigation-only runner, all eight legs, no camera/arm dependency, failure exit and no re-entry");
}
