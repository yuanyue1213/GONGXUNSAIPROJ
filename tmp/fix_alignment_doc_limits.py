from pathlib import Path
p=Path('yundong_part/esp32s3/PROTOCOL.md');s=p.read_text(encoding='utf-8').replace('两种位置修正共用 App 设置的粗调速度、精调及补偿速度（5–120 RPM）','两种位置修正共用 App 设置的粗调速度、精调及补偿速度（5–60 RPM）');p.write_text(s,encoding='utf-8')
