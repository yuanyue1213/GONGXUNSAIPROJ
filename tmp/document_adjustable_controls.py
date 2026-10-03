from pathlib import Path
text='''
## 可调夹子速度及位置修正

抓取、放下各自新增夹子张开速度（6–300 度/秒，默认 60）。仅用于流程的 0°→60° 张开，时长为 ceil(60000/速度) ms；夹紧及手动角度保持现有行为。STATE 帧最后新增可选字段夹子速度：
`STATE,seq,A/P,r1,z1,r2,z2,rRPM,upRPM,downRPM,gripperDps\\n`。
旧 STATE 无该字段时仍为 60 度/秒（1 秒张开）。回零/后续步骤等待张开完成。

两种位置修正共用 App 设置的粗调速度、精调及补偿速度（5–60 RPM）和结束 x 补偿距离（整数 -100..100 mm）。默认分别为 20 RPM、10 RPM、+10 mm；补偿为 0 时关闭，负值使用图像 x 反方向。中心、比例、修正比例、容差及单步上限均保持原值。

新帧：`ALIGN_CFG,seq,target,forwardPpm,lateralPpm,fineRPM,coarseRPM,xOffsetMm\\n`。
target=0 为绿色物块，1/2/3 为左/中/右色环。例：`ALIGN_CFG,1,2,9889,9889,10,20,10\\n`。
ESP32/STM32 校验后一次性启动，忙碌/重复/非法帧忽略。原 ALIGN、ALIGN_RING 使用原默认参数。
协议接收帧上限扩展为 127 字节（不含换行）；相机坐标帧不变。没有新增回包。
'''
for path in ['yundong_part/esp32s3/PROTOCOL.md','yundong_part/shared/CAMERA_ALIGNMENT_PROTOCOL.md']:
 p=Path(path);p.write_text(p.read_text(encoding='utf-8')+text,encoding='utf-8')
p=Path('output/目前已有状态与扩展说明.md');p.write_text(p.read_text(encoding='utf-8')+'''
App 可分别设置抓取、放下的夹子张开速度（默认 60 度/秒）。两种位置修正共用可调粗调速度、精调/补偿速度及结束 x 补偿距离；补偿为 0 时关闭，负数反向。
''',encoding='utf-8')
