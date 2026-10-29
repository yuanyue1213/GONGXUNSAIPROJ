/* 与硬件无关的单通道舵机轨迹状态：记录起点、目标和时间，计算当前输出角。
 * 完成表示软件轨迹到期，不代表通过位置传感器确认了舵机实际到位。 */
#include "servo_trajectory.h"
#include "servo_motion.h"
void ServoTrajectory_Set(ServoTrajectory *job,uint16_t angle)
{
    *job=(ServoTrajectory){.start=angle,.target=angle,.angle=angle,.known=true};
}
/* 重新规划时先算出旧轨迹当前输出，避免新轨迹从旧目标开始而发生跳变。 */
void ServoTrajectory_Start(ServoTrajectory *job,uint16_t target,uint32_t duration,uint32_t now)
{
    /* A replaced trajectory starts from its current output, never a stale target. */
    if(job->active) ServoTrajectory_Update(job,now);
    if(!job->known||!duration||job->angle==target) { ServoTrajectory_Set(job,target); return; }
    job->start=job->angle; job->target=target; job->started=now;
    job->duration=duration; job->active=true;
}
uint16_t ServoTrajectory_Update(ServoTrajectory *job,uint32_t now)
{
    if(job->active) {
        /* 使用无符号差值计算经过时间，正常短时运动可跨越毫秒计数器回绕。 */
        uint32_t elapsed=now-job->started;
        job->angle=ServoMotion_Angle(job->start,job->target,elapsed,job->duration);
        if(elapsed>=job->duration) job->active=false;
    }
    return job->angle;
}
