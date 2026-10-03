from pathlib import Path
text='''
## 色环位置修正（circle.py）

App 独立按钮“执行色环位置修正”，选择左/中/右色环（1/2/3，默认 2）。原绿色物块位置修正按钮保留。

- 摄像头运行 `camera/MaixVision/circle.py`，使用 `/dev/ttyS2`、115200 baud，TX 接 STM32 UART4 RX/PA1，共地；与原物块识别共用这条串口，分别运行对应脚本。
- 摄像头 → STM32：`RINGS,x1,y1,x2,y2,x3,y3\\n`，如 `RINGS,100,160,266,160,400,160\\n`。只在识别到三个圆时输出，按 x 从小到大排列，坐标范围 x=0..511、y=0..319；无目标不发送旧坐标。
- App → ESP32 → STM32：`ALIGN_RING,序号,色环编号,前后每米脉冲,左右每米脉冲\\n`，如 `ALIGN_RING,1,2,9889,9889\\n`。ESP 校验后原样转发，无回包。
- 对准中心 (256,160)，比例 1.090819 mm/px；方向沿用已标定映射：图像右侧 B、左侧 F、下侧 L、上侧 R。
- 误差 >20 px：50% 修正、20 RPM；否则 30%、10 RPM；优先较大误差轴，单步 1..15 mm。
- 等待四轮到位，停稳 300 ms 后丢弃旧坐标并读取新帧；两个新坐标都在中心 ±2 px 内结束。
- 单次等图像最多 3 秒，总流程最多 120 秒/60 次移动；“停止底盘”取消。色环修正不会使用绿色物块坐标，原修正不会使用色环坐标。
- 编号表示当前图像中的左右排序，不表示颜色或固定物块身份；三圆应持续在视野中。缩放、安装改变后需重新标定。
'''
for name in ['yundong_part/esp32s3/PROTOCOL.md','yundong_part/shared/CAMERA_ALIGNMENT_PROTOCOL.md']:
 p=Path(name);p.write_text(p.read_text(encoding='utf-8')+text,encoding='utf-8')
p=Path('output/目前已有状态与扩展说明.md');p.write_text(p.read_text(encoding='utf-8')+'''
## 色环位置修正

App 选择左、中、右色环（默认中间），点击“执行色环位置修正”。摄像头运行 circle.py，目标为所选色环中心到达 (256,160)。沿用物块修正的距离比例与移动方向，分步移动、停稳后重新识别，连续两次在 ±2 px 内结束；停止底盘可取消。
''',encoding='utf-8')
import ast
ast.parse(Path('camera/MaixVision/circle.py').read_text(encoding='utf-8'))
print('PASS: camera Python syntax')
