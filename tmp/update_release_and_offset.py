from pathlib import Path
p=Path('yundong_part/stm32h743/zhukong/Core/Src/robot_control.c');s=p.read_text(encoding='utf-8');s=s.replace('{ 0U, 263U, 110, -115, true}', '{ 0U, 263U, 110, -110, true}').replace('one 15 mm move along image +X','one 10 mm move along image +X').replace('direction = CAMERA_IMAGE_RIGHT_DIRECTION; mm = 15U;', 'direction = CAMERA_IMAGE_RIGHT_DIRECTION; mm = 10U;');p.write_text(s,encoding='utf-8')
p=Path('yundong_part/tests/distance_control_test.c');s=p.read_text(encoding='utf-8');s=s.replace('absolute_target[0] == 10350); /* z=-115 mm. */','absolute_target[0] == 9900); /* z=-110 mm. */')
for old,new in [('position_frames == 2 && wheel_pulses() == 150U && frame[6] == 1','position_frames == 2 && wheel_pulses() == 100U && frame[6] == 1'),('camera_sample(sample); assert(position_frames == 1 && wheel_pulses() == 150U);','camera_sample(sample); assert(position_frames == 1 && wheel_pulses() == 100U);'),('position_frames == 3 && wheel_pulses() == 150U && frame[6] == 1 && frame[8] == 10','position_frames == 3 && wheel_pulses() == 100U && frame[6] == 1 && frame[8] == 10')]:
 assert old in s;s=s.replace(old,new)
p.write_text(s,encoding='utf-8')
for path in ['output/目前已有状态与扩展说明.md','yundong_part/esp32s3/PROTOCOL.md','yundong_part/shared/CAMERA_ALIGNMENT_PROTOCOL.md']:
 p=Path(path);s=p.read_text(encoding='utf-8').replace('z=-115 mm','z=-110 mm').replace('110/-115','110/-110').replace('额外移动 15 mm','额外移动 10 mm');p.write_text(s,encoding='utf-8')
