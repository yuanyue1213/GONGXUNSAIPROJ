from pathlib import Path
p=Path('yundong_part/tests/distance_control_test.c');s=p.read_text(encoding='utf-8')
for old,new in [('absolute_target[0] == 4500 && absolute_target[1] == 0); /* State 2: z first. */','absolute_target[0] == 4500 && absolute_target[1] == 0 && last_lift_angle.rpm == 30); /* State 2: z first. */'),('finish_grab_axes(); assert(absolute_target[0] == 3600);','finish_grab_axes(); assert(absolute_target[0] == 3600 && last_lift_angle.rpm == 30);'),('finish_grab_axes(); assert(absolute_target[0] == 0);','finish_grab_axes(); assert(absolute_target[0] == 0 && last_lift_angle.rpm == 20);'),('absolute_target[0] == 3600 && servo_history[servo_commands-2] == 60);','absolute_target[0] == 3600 && servo_history[servo_commands-2] == 60 && last_lift_angle.rpm == 30);'),('absolute_target[0] == 9000); /* z=-100 mm. */','absolute_target[0] == 9000 && last_lift_angle.rpm == 30); /* z=-100 mm. */')]:
 assert old in s;s=s.replace(old,new)
p.write_text(s,encoding='utf-8')
for path in ['output/目前已有状态与扩展说明.md','yundong_part/esp32s3/PROTOCOL.md']:
 p=Path(path);s=p.read_text(encoding='utf-8').replace('抓取伸缩、升降及返回原点速度为 20 RPM','抓取和放下流程下降速度为 30 RPM，上升、伸缩及返回原点速度为 20 RPM').replace('两轴固定 20 RPM、3200 脉冲/圈','流程下降 30 RPM，上升及伸缩 20 RPM，3200 脉冲/圈');s+='\n抓取和放下状态的下降动作使用 30 RPM；上升及前后伸缩仍为 20 RPM。App 手动绝对定位速度仍按输入值执行。\n';p.write_text(s,encoding='utf-8')
