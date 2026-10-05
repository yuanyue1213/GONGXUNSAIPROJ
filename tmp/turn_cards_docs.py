from pathlib import Path
p=Path('output/目前已有状态与扩展说明.md'); s=p.read_text(encoding='utf-8')
a=s.index('## 放下'); b=s.index('## 位置修正',a)
block=s[a:b].replace('| 0° |','| 保持 |').replace('转盘 0°、爪子','转盘保持进入流程时的角度、爪子')
block=block.replace('按抓取的', '开始时先将r、z回到已记录的原点（r=0、z=0），到位后再设置初始夹子、基座姿态。放下全程不发送转盘角度指令，旧卡片的转盘参数忽略。\n\n按抓取的')
s=s[:a]+block+s[b:]
s += '''
## 转盘卡片

App“流程→转盘”可设置起始角度、结束角度（均0～270°）和速度（1～360°/秒），命名保存为卡片。执行时先直接设置起始角度，再按角度差与速度计算时长，每20ms平滑更新到结束角度。没有编码器，完成依据PWM渐变时长。可和抓取、放下混合编排，卡片间保持1秒，停止流程取消后续卡片。
'''
p.write_text(s,encoding='utf-8',newline='\r\n')
p=Path('yundong_part/esp32s3/PROTOCOL.md'); s=p.read_text(encoding='utf-8')
s += '''
## 转盘卡片与放下回零（2026-10-03）

转盘卡片：`STATE,seq,T,startAngle,endAngle,degreesPerSecond\\n`。
编排：`PLAN_ITEM,planSeq,index,T,startAngle,endAngle,degreesPerSecond\\n`。
角度0～270°，速度1～360°/秒。沿用PLAN_BEGIN/PLAN_RUN及16张上限，ESP校验后转发。STM先直接设置起始角度，再每20ms按线性角速度更新至结束角度；完成后按已有规则推进下一卡片，Z取消。没有实际舵机到位反馈。

放下P先r/z回零、确认两轴到位后才建立初始G/B姿态；全程不再设置T，旧STATE/卡片theta字段仍兼容解析但忽略。抓取转盘行为不变。
'''
p.write_text(s,encoding='utf-8',newline='\r\n')
