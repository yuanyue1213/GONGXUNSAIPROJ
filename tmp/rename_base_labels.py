from pathlib import Path
for filename in ['output/目前已有状态与扩展说明.md', 'yundong_part/esp32s3/PROTOCOL.md']:
    p=Path(filename); s=p.read_text(encoding='utf-8')
    s=s.replace('初始基座角度','启动基座角度')
    s=s.replace('当前角度→翻转位→初始位','当前角度→转盘取物角度→放置角度')
    s=s.replace('不在开始时先跳回初始位','不在开始时先跳到放置角度')
    s=s.replace('基座初始/翻转位','基座用途角度（抓取：抓取角度/转盘存放角度；放下：放置角度/转盘取物角度）')
    s += '''
## 基座角度名称

| 场景 | 名称 | 原字段 | 用途 |
| --- | --- | --- | --- |
| 启动初始化 | 启动基座角度 | ORIGIN.baseAngle | 启动姿态及返回原点时的基座角度 |
| 抓取 | 抓取角度 | base_home | 从外部抓取物品 |
| 抓取 | 转盘存放角度 | base_tilt | 将物品存放到转盘 |
| 放下 | 转盘取物角度 | base_tilt | 从转盘拿起物品 |
| 放下 | 放置角度 | base_home | 向外放下物品 |

放下基座顺序：当前角度→转盘取物角度→放置角度。仅修改显示名称，协议字段、已保存卡片和角度值不变。
'''
    p.write_text(s,encoding='utf-8',newline='\r\n')
