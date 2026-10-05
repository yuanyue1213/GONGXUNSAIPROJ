from pathlib import Path
for filename in ['yundong_part/shared/CAMERA_ALIGNMENT_PROTOCOL.md','yundong_part/esp32s3/PROTOCOL.md','output/目前已有状态与扩展说明.md']:
 p=Path(filename); s=p.read_text(encoding='utf-8')
 s=s.replace('将完整 X/Y 误差合并为一条四轮定距运动','分别计算完整X/Y误差，先执行X四轮定距运动，四轮到位并停稳300ms后再执行Y运动')
 s=s.replace('完整X/Y误差合并为一次四轮移动','完整X/Y误差分两段，先X到位再Y到位')
 s=s.replace('两个方向在同一条AA帧中执行。','此处四轮合成值仅表示数学分解；实际先执行X脉冲，再执行Y脉冲，各发一条AA帧。')
 s=s.replace('按上述公式一次下发完整X/Y四轮目标','按上述公式分别下发X、Y四轮目标，X到位停稳300ms后才执行Y')
 s=s.replace('均值按±1px容差和1.090819mm/px计算一次完整移动','均值按±1px容差和1.090819mm/px分别计算X/Y完整移动，先X后Y')
 s += '\n中心校准分轴执行：同一组5帧均值计算X/Y目标，先X、后Y，各段独立选粗调/精调速度；X到位静置300ms才启动Y，不重新采样。每轴±1px内跳过。全部有效修正段到位后才执行设置的X补偿；补偿为0时直接结束。取消、故障时不执行待运行的Y或补偿。\n'
 p.write_text(s,encoding='utf-8',newline='\r\n')
