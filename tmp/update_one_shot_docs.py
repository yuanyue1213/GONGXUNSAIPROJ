from pathlib import Path
p=Path('yundong_part/shared/camera_position_protocol.h')
s=p.read_text(encoding='utf-8')
s=s.replace('2 px tolerance, choose the larger error axis first.', '2 px tolerance on each axis; combine full X/Y errors in one move.')
for line in ['#define CAMERA_CORRECTION_FINE_GAIN    30U\n', '#define CAMERA_CORRECTION_COARSE_GAIN  50U\n', '#define CAMERA_CORRECTION_MAX_MM       15U\n']:
    s=s.replace(line,'')
s=s.replace('Match speed to the same larger-axis error used for the distance calculation.', 'Select maximum wheel speed from the larger image error.')
a=s.index('static inline bool CameraProtocol_Correction(')
b=s.index('typedef struct { uint32_t sequence, ppm, rpm, step_mm, reverse; }',a)
s=s[:a]+s[b:]
p.write_text(s,encoding='utf-8',newline='\r\n')

common='两种中心位置修正均只取一次新坐标，将完整 X/Y 误差合并为一条四轮定距运动，不再循环采样。每轴 ±2 px 内不修正；超过容差的距离按 1.090819 mm/px 完整换算，不乘修正比例、不限制为15 mm。四轮到位后静置300 ms，再执行一次配置的 x 补偿（默认 +10 mm、10 RPM）；补偿为0时到位即结束。停止或失败不追加补偿。该方式不复测最终图像误差，实际精度取决于标定、安装和轮胎滑移。'
p=Path('yundong_part/shared/CAMERA_ALIGNMENT_PROTOCOL.md'); s=p.read_text(encoding='utf-8')
a=s.index('修正系数 ='); b=s.index('\n```',a)
s=s[:a]+'''X距离mm = dx × 1.090819
Y距离mm = dy × 1.090819
X脉冲 = round(X距离mm × forward_ppm / 1000)
Y脉冲 = round(Y距离mm × lateral_ppm / 1000)
四轮有符号脉冲[左前,右前,右后,左后] = [X+Y,X-Y,X+Y,X-Y]
最大轮速RPM = max(abs(dx),abs(dy)) > 20 ? 粗调速度 : 精调速度
各轮速度 = 最大轮速 × abs(该轮脉冲) / 最大轮脉冲（取整，最小1RPM）'''+s[b:]
a=s.index('每次处理绝对偏差更大的轴'); b=s.index('ppm 的来源',a)
s=s[:a]+'''使用64位整数直接从像素换算脉冲，避免先取整到毫米的误差。每轴±2px范围内置零；非零轴换算不足1脉冲时最少1脉冲。轮速按脉冲比例配置，让各轮尽量同时结束；0脉冲轮保持不动。方向映射保持上表。

例如 (356,210)：dx=100、dy=50，对应X=109.0819mm、Y=54.54095mm。两轴ppm均为10000时，X=1091脉冲、Y=545脉冲，四轮目标为[1636,546,1636,546]。粗调速度20RPM时各轮为[20,7,20,7]RPM。两个方向在同一条AA帧中执行。

'''+s[b:]
a=s.index('1. 启动时'); b=s.index('主要代码：',a)
s=s[:a]+'''1. 启动时丢弃旧缓存及第一条残帧，等待一个新的有效目标坐标。
2. 按上述公式一次下发完整X/Y四轮目标，运动期间忽略后续坐标；已在±2px内则跳过主移动。
3. 等四轮到位后静置300ms，执行配置的x补偿。补偿为0则直接结束。
4. 补偿到位后结束，不再重新识别；新的序号可以重新执行一次校准。

等待首个坐标超过3秒、整个流程超过120秒或电机通信/状态故障时取消；运动中取消会请求四轮停止，失败则重试停车。无目标不输出坐标，因此会走采样超时。修正期间拒绝新MOVE/ARM_MOVE/A，不排队，S/Z可取消。无应用回包，App无法获知真实到位状态；一次计算不保证消除标定或滑移造成的最终误差。

'''+s[b:]
s=s.replace('- 误差 >20 px：50% 修正、20 RPM；否则 30%、10 RPM；优先较大误差轴，单步 1..15 mm。','- 一个有效色环坐标同时计算完整X/Y位移；最大偏差>20px使用粗调速度，否则使用精调速度。')
s=s.replace('- 等待四轮到位，停稳 300 ms 后丢弃旧坐标并读取新帧；两个新坐标都在中心 ±2 px 内结束。','- 等待四轮到位及一次补偿完成后结束，不再次读取坐标。')
s=s.replace('总流程最多 120 秒/60 次移动','总流程最多 120 秒')
old='两种位置修正成功对准中心后，均沿图像 x 正方向（现有 B 指令方向）额外移动 10 mm，速度 10 RPM，使用前后距离标定。补偿仅执行一次，等待四轮到位后结束，不再重新居中；停止或失败时不追加补偿，补偿途中也可停止。'
s=s.replace(old,common).replace('中心、比例、修正比例、容差及单步上限均保持原值。','中心、像素比例和容差保持原值；完整修正不再使用距离增益及单步上限。')
p.write_text(s,encoding='utf-8',newline='\r\n')
p=Path('yundong_part/esp32s3/PROTOCOL.md'); s=p.read_text(encoding='utf-8')
a=s.index('每次处理较大偏差轴：'); b=s.index('详见 [摄像头位置修正文档]',a)
s=s[:a]+common+' 首帧超时3秒、总时限120秒；S/Z可取消。执行中拒绝新的MOVE/ARM_MOVE/A，无应用回包。'+s[b:]
s=s.replace('- 误差 >20 px：50% 修正、20 RPM；否则 30%、10 RPM；优先较大误差轴，单步 1..15 mm。','- 一帧同时计算完整X/Y位移；偏差>20px选粗调速度，否则精调速度，四轮速度按目标脉冲比例配置。')
s=s.replace('- 等待四轮到位，停稳 300 ms 后丢弃旧坐标并读取新帧；两个新坐标都在中心 ±2 px 内结束。','- 一次主移动及配置补偿到位后结束，不再重新识别。')
s=s.replace('总流程最多 120 秒/60 次移动','总流程最多 120 秒')
s=s.replace(old,common).replace('中心、比例、修正比例、容差及单步上限均保持原值。','中心、像素比例和容差保持原值；完整修正不再使用距离增益及单步上限。')
p.write_text(s,encoding='utf-8',newline='\r\n')
p=Path('output/目前已有状态与扩展说明.md'); s=p.read_text(encoding='utf-8')
a=s.index('| 1. 获取位置'); b=s.index('\n## 色环位置修正',a)
s=s[:a]+'''| 1. 获取位置 | 读取一次新坐标，中心(256,160)，1px=1.090819mm | 有效坐标 |
| 2. 修正移动 | 完整X/Y误差合并为一次四轮移动，各轴±2px内跳过 | 四轮到位 |
| 3. 补偿 | 静置300ms，再执行设置的x补偿；补偿为0时跳过 | 四轮到位 |
| 4. 完成 | 结束，不复测；可再次点击重新执行 | 结束 |
'''+s[b:]
s=s.replace('分步移动、停稳后重新识别，连续两次在 ±2 px 内结束','读取一次色环坐标，完整X/Y误差一次移动，再执行设定的x补偿后结束')
s=s.replace(old,common)
p.write_text(s,encoding='utf-8',newline='\r\n')
