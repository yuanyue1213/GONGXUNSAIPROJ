from pathlib import Path

old = '三圆心最小二乘拟合斜率 m=(3Σxy-ΣxΣy)/(3Σx²-(Σx)²)。要求 x 严格递增、总跨度至少80px、各点对拟合线残差≤5px，避免退化/错检。每次选车轮步长 min(maxStepMm, floor(|m|*1000/25)+1)，四轮到位后静置300ms，丢弃旧帧再测。连续三个新帧 |m|≤0.017（约1°）则结束，不执行位置修正的x补偿。'
new = '三圆心最小二乘拟合斜率 m=(3Σxy-ΣxΣy)/(3Σx²-(Σx)²)。要求 x 严格递增、总跨度至少80px、各点对拟合线残差≤5px，避免退化/错检。连续三个新帧中，三个圆心的最大y与最小y之差均≤1px才结束；不再使用原来的|m|≤0.017阈值（左右跨度200px时，原阈值会放过3px高度差）。斜率绝对值≤0.05时以车轮0.2mm、5RPM微调，直接换算标定脉冲、最少1脉冲；偏差更大时使用 min(maxStepMm, floor(|m|*1000/25)+1) mm与App转向速度。若拟合斜率为0但高度差>1px，清除连续完成计数、等下一帧，不猜测转向。四轮到位后静置300ms，丢弃旧帧再测。不执行位置修正的x补偿。图像水平仍受摄像头安装角度及透视影响，不能保证实物绝对平行。'
for name in ['yundong_part/esp32s3/PROTOCOL.md', 'yundong_part/shared/CAMERA_ALIGNMENT_PROTOCOL.md']:
    p = Path(name)
    s = p.read_text(encoding='utf-8')
    assert old in s
    p.write_text(s.replace(old, new), encoding='utf-8')
p = Path('yundong_part/tests/distance_control_test.c')
s = p.read_text(encoding='utf-8')
idx = s.index('int main(void)')
s = s[:idx] + '''static void test_parallel_pixel_tolerance(void)
{
    const CameraCenter near[] = {{100,150},{200,151},{300,153}};
    int32_t slope;
    assert(CameraProtocol_RingSlope(near, &slope) && slope == 15);
    assert(CameraProtocol_RingYSpread(near) == 3U);
    for (unsigned reverse = 0; reverse < 2U; ++reverse) {
        reset(); camera_ready();
        feed(reverse ? "PARALLEL,1100,10000,20,3,1\\n" : "PARALLEL,1100,10000,20,3,0\\n");
        camera_sample("RINGS,100,150,200,151,300,153\\n");
        assert(position_frames == 1 && wheel_pulses() == 2U && frame[8] == 5U);
        assert(frame[6] == (reverse ? 1U : 0U));
        finish_alignment_move();
        camera_sample("RINGS,100,150,200,151,300,151\\n");
        camera_sample("RINGS,100,150,200,151,300,151\\n");
        camera_sample("RINGS,100,153,200,151,300,150\\n"); // Non-horizontal resets completion count.
        assert(position_frames == 2 && wheel_pulses() == 2U && frame[6] == (reverse ? 0U : 1U));
        finish_alignment_move();
        camera_sample("RINGS,100,150,200,151,300,151\\n");
        camera_sample("RINGS,100,150,200,151,300,151\\n");
        feed("MOVE,1101,F,100,45,9889\\n"); assert(position_frames == 2);
        camera_sample("RINGS,100,150,200,151,300,151\\n");
        feed("MOVE,1101,F,100,45,9889\\n"); assert(position_frames == 3);
    }
    reset(); camera_ready(); feed("PARALLEL,1102,9889,10,3,0\\n");
    camera_sample("RINGS,100,150,200,153,300,150\\n"); assert(position_frames == 0); // Zero slope with noise must not turn.
    feed("MOVE,1103,F,100,45,9889\\n"); assert(position_frames == 0); // Nor may it complete immediately.
    for (unsigned i = 0; i < 3U; ++i) camera_sample("RINGS,100,150,200,150,300,150\\n");
    feed("MOVE,1103,F,100,45,9889\\n"); assert(position_frames == 1);
    reset(); camera_ready(); feed("PARALLEL,1104,1,5,1,0\\n");
    camera_sample("RINGS,100,150,200,151,300,153\\n"); assert(wheel_pulses() == 1U); // Never round fine correction down to zero.
}
''' + s[idx:]
s = s.replace('    test_parallel_alignment();', '    test_parallel_alignment();\n    test_parallel_pixel_tolerance();')
p.write_text(s, encoding='utf-8')
