from pathlib import Path
text='''
## 左右转与圆心连线平行校准

App 左转/右转为原地小步转向，输入车轮行进距离 mm 与转向速度；该距离不是车身角度。按当前前进映射，左转 C 轮序符号为[+,-,-,+]，右转 W 为[-,+,+,-]（轮序左前/右前/右后/左后）。帧沿用 `MOVE,seq,C或W,wheelTravelMm,rpm,ppm\\n`。App 转向速度限制5–60 RPM。

摄像头运行现有 `camera/MaixVision/circle.py`，UART4 接收 `RINGS,x1,y1,x2,y2,x3,y3\\n`；无需新增摄像头帧。
新增 `PARALLEL,seq,ppm,rpm,maxStepMm,reverse\\n`。ppm 为当前车轮距离标定；速度5–60 RPM，单步1–10mm，reverse为0/1（反向切换）。默认10 RPM、3mm，正斜率发C、负斜率发W，反向设置交换。

三圆心最小二乘拟合斜率 m=(3Σxy-ΣxΣy)/(3Σx²-(Σx)²)。要求 x 严格递增、总跨度至少80px、各点对拟合线残差≤5px，避免退化/错检。每次选车轮步长 min(maxStepMm, floor(|m|*1000/25)+1)，四轮到位后静置300ms，丢弃旧帧再测。连续三个新帧 |m|≤0.017（约1°）则结束，不执行位置修正的x补偿。

新帧等候最多3秒，总流程120秒或60次转动；停止底盘可取消。与位置修正/抓取/放下互斥，不排队。斜率是图像水平关系，不能替代透视标定下的真实车身角度。首次使用检查转向是否减小倾斜；反了则切换“校准转向：反向”。
'''
for name in ['yundong_part/esp32s3/PROTOCOL.md','yundong_part/shared/CAMERA_ALIGNMENT_PROTOCOL.md']:
 p=Path(name);p.write_text(p.read_text(encoding='utf-8')+text,encoding='utf-8')
p=Path('output/目前已有状态与扩展说明.md');p.write_text(p.read_text(encoding='utf-8')+'''
## 圆心连线平行校准

摄像头运行 circle.py，App 点击“执行平行校准”。根据三个圆心拟合连线斜率，左右小步转向、停稳后重新识别，连续三帧在约±1°内结束。速度、单步上限及方向可调；停止底盘可取消。左右转按钮可单独转动。
''',encoding='utf-8')
