from pathlib import Path
p=Path('output/目前已有状态与扩展说明.md'); s=p.read_text(encoding='utf-8')
s=s.replace('到位后再设置初始夹子、基座姿态。', '到位后仅设置夹子姿态，保持基座当前角度。基座只执行“当前角度→翻转位→初始位”，不在开始时先跳回初始位。')
s=s.replace('| 1 | 8 | 保持 | 60° | 248° |', '| 1 | 8 | 保持 | 60° | 保持当前角度 |')
s=s.replace('| 2 | 7 | 保持 | 60° | 248°→140° |', '| 2 | 7 | 保持 | 60° | 当前角度→140° |')
p.write_text(s,encoding='utf-8',newline='\r\n')
p=Path('yundong_part/esp32s3/PROTOCOL.md'); s=p.read_text(encoding='utf-8')
s=s.replace('确认两轴到位后才建立初始G/B姿态', '确认两轴到位后只设置G，保持B当前角度')
s += '\n放下基座（2026-10-04）：取消开始时设置base_home；所有保持步骤不重复设置B。两次2秒渐变均从最后成功下发的B角度出发，目标依次为base_tilt、base_home。协议及App参数不变。\n'
p.write_text(s,encoding='utf-8',newline='\r\n')
