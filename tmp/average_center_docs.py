from pathlib import Path
for filename in ['yundong_part/shared/CAMERA_ALIGNMENT_PROTOCOL.md', 'yundong_part/esp32s3/PROTOCOL.md', 'output/目前已有状态与扩展说明.md']:
    p=Path(filename); s=p.read_text(encoding='utf-8')
    for old,new in [
        ('只取一次新坐标','收集5个有效新坐标，分别取x/y平均值并四舍五入到整数像素'),
        ('等待一个新的有效目标坐标','等待5个新的有效目标坐标并取平均值'),
        ('读取一次新坐标','读取5个新坐标并取平均值'),
        ('读取一次色环坐标','读取5个所选色环的新坐标并取平均值'),
        ('一个有效色环坐标同时计算完整X/Y位移','5个有效色环坐标的平均值同时计算完整X/Y位移'),
        ('一帧同时计算完整X/Y位移','5帧坐标平均值同时计算完整X/Y位移'),
        ('首帧超时3秒','整组采样超时3秒'),
        ('等待首个坐标超过3秒','收集整组坐标超过3秒'),
    ]: s=s.replace(old,new)
    s += '\n中心校准采样：每次启动清空累计值，收集5个有效新坐标分别求x/y平均，四舍五入到整数像素；同一缓存帧不重复计数，非法帧不计数。整组须在3秒内取得，否则取消。均值按±1px容差和1.090819mm/px计算一次完整移动，随后按设置执行补偿，不循环重测。物块和选中色环共用此逻辑。\n'
    p.write_text(s,encoding='utf-8',newline='\r\n')
