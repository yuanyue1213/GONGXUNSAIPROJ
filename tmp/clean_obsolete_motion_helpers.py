from pathlib import Path
p=Path('yundong_part/tests/distance_control_test.c')
s=p.read_text(encoding='utf-8')
for name in ['finish_grab_cycle', 'finish_gripper_open', 'finish_z_first_axes', 'upload_test_plan']:
    a=s.index('static void '+name+'(')
    b=s.index('{',a)+1
    depth=1
    while depth:
        if s[b]=='{': depth+=1
        elif s[b]=='}': depth-=1
        b+=1
    s=s[:a]+s[b:]
p.write_text(s,encoding='utf-8')
