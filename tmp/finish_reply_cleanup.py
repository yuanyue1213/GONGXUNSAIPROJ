from pathlib import Path
root = Path('F:/GONGXUNSAIPROJ')
s = (root / 'tmp/update_reply_tests.py').read_text(encoding='utf-8')
exec(s[s.index("p = root / 'yundong_part/esp32s3/hello_world/main/robot_remote_main.c'"):])
p = root / 'yundong_part/stm32h743/zhukong/Core/Src/robot_control.c'
s = p.read_text(encoding='utf-8').replace('ESP32 <-> STM32', 'ESP32 -> STM32')
s = s.replace('            if (s_dropping_frame)\n            {\n\n            }\n            else', '            if (!s_dropping_frame)')
s = s.replace('\n\n        }', '\n        }').replace('\n\n    }', '\n    }')
p.write_text(s, encoding='utf-8')
