from pathlib import Path
p=Path('yundong_part/tests/distance_control_test.c'); s=p.read_text(encoding='utf-8')
s=s.replace('i < 5U; ++i) camera_sample(line)', 'i < 50U; ++i) camera_sample(line)')
for name in ['test_ring_alignment','test_camera_alignment','test_center_sample_average']:
 a=s.index('static void '+name+'('); b=s.index('\nstatic ',a+1)
 part=s[a:b].replace('poll_without_keep(3000)', 'poll_without_keep(30000)').replace('tick = 3000;', 'tick = 30000;').replace('tick += 3000;', 'tick += 30000;')
 if name=='test_center_sample_average':
  part=part.replace('i < 5;', 'i < 50;').replace('i < 4)', 'i < 49)').replace('i < 4;', 'i < 49;').replace('262+2*i', '262+2*(i % 5)')
  part=part.replace('            camera_sample(sample);', '            tick += 100; camera_sample(sample); /* 5 seconds total: more than the former 3-second limit. */')
 s=s[:a]+part+s[b:]
p.write_text(s,encoding='utf-8',newline='\r\n')
for filename in ['yundong_part/shared/CAMERA_ALIGNMENT_PROTOCOL.md','yundong_part/esp32s3/PROTOCOL.md','output/目前已有状态与扩展说明.md']:
 p=Path(filename); s=p.read_text(encoding='utf-8')
 s=s.replace('5个有效新坐标','50个有效新坐标').replace('5个有效色环坐标','50个有效色环坐标').replace('5个新的有效目标坐标','50个新的有效目标坐标').replace('5个新坐标','50个新坐标').replace('5个所选色环','50个所选色环').replace('5帧','50帧')
 s=s.replace('整组采样超时3秒','整组采样超时30秒').replace('收集整组坐标超过3秒','收集整组坐标超过30秒').replace('整组须在3秒内','整组须在30秒内').replace('整组坐标超过3秒','整组坐标超过30秒')
 p.write_text(s,encoding='utf-8',newline='\r\n')
