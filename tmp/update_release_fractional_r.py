from pathlib import Path
p=Path('yundong_part/tests/distance_control_test.c');s=p.read_text(encoding='utf-8');a='advance_grab_hold(); assert(absolute_target[1] == 3118 && absolute_target[0] == 0); /* Step 6 r first. */';b='advance_grab_hold(); assert(absolute_target[1] == 3161 && absolute_target[0] == 0); /* Step 6: r=111.5 mm first. */';assert a in s;s=s.replace(a,b);p.write_text(s,encoding='utf-8')
p=Path('output/目前已有状态与扩展说明.md');s=p.read_text(encoding='utf-8').replace('r=110 mm、z=-110 mm','r=111.5 mm、z=-110 mm');p.write_text(s,encoding='utf-8')
p=Path('yundong_part/esp32s3/PROTOCOL.md');s=p.read_text(encoding='utf-8').replace('110/-110','111.5/-110');p.write_text(s,encoding='utf-8')
