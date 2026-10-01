from pathlib import Path
import math
from reportlab.pdfgen import canvas
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, PageBreak
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.cidfonts import UnicodeCIDFont
import pypdfium2 as pdfium

ROOT = Path('F:/GONGXUNSAIPROJ')
OUT = ROOT / 'output/pdf/麦氏轮定距移动与张大头电机转角计算说明.pdf'
OUT.parent.mkdir(parents=True, exist_ok=True)
pdfmetrics.registerFont(UnicodeCIDFont('STSong-Light'))
styles = getSampleStyleSheet()
for key in ['Normal', 'Title', 'Heading1', 'Heading2']:
    styles[key].fontName = 'STSong-Light'
    styles[key].textColor = colors.black
styles['Normal'].fontSize = 10.5
styles['Normal'].leading = 17
styles['Normal'].spaceAfter = 6
styles['Title'].fontSize = 21
styles['Title'].leading = 29
styles['Title'].spaceAfter = 14
styles['Heading1'].fontSize = 16
styles['Heading1'].leading = 23
styles['Heading1'].spaceAfter = 12
styles['Heading2'].fontSize = 12.5
styles['Heading2'].leading = 19
styles['Heading2'].spaceBefore = 9
styles['Heading2'].spaceAfter = 7
styles.add(ParagraphStyle(name='Formula', fontName='STSong-Light', fontSize=12,
                          leading=20, spaceAfter=8, leftIndent=12))
styles.add(ParagraphStyle(name='Cell', fontName='STSong-Light', fontSize=10,
                          leading=15, spaceAfter=0))
styles.add(ParagraphStyle(name='SmallNote', fontName='STSong-Light', fontSize=9,
                          leading=14, spaceAfter=7))
story=[]
def p(text, style='Normal'): story.append(Paragraph(text, styles[style]))
def h(text): p(text, 'Heading2')
def formula(text): p(text, 'Formula')
def table(rows, widths):
    cells=[[Paragraph(str(x), styles['Cell']) for x in row] for row in rows]
    t=Table(cells,colWidths=widths,repeatRows=1,hAlign='LEFT')
    t.setStyle(TableStyle([
        ('BACKGROUND',(0,0),(-1,0),colors.HexColor('#E9EFF5')),
        ('GRID',(0,0),(-1,-1),.5,colors.HexColor('#D9D9D9')),
        ('VALIGN',(0,0),(-1,-1),'MIDDLE'),
        ('LEFTPADDING',(0,0),(-1,-1),8),('RIGHTPADDING',(0,0),(-1,-1),8),
        ('TOPPADDING',(0,0),(-1,-1),6),('BOTTOMPADDING',(0,0),(-1,-1),6)]))
    story.extend([t,Spacer(1,10)])

p('麦氏轮定距移动与张大头电机转角计算说明','Title')
p('适用底盘 103 mm 轮径 1比1直驱 四轮麦克纳姆轮','Heading2')
p('本说明把设定的移动米数，依次换算成车轮圈数、电机转角和张大头 ZDT X42S 驱动器的位置脉冲，并对应当前 App 与 STM32 的实现。纯前后或左右平移时，四个电机的转角大小相同，转动方向按轮位分配。')
formula('总公式：θm = 360 × L × i × k ÷ (π × D)')
p('其中 L、D 都使用米，θm 的单位为度。按当前参数且 k=1：每移动 1 米，每个电机理论转 1112.5394°，约 3.090387 圈。')
h('1 参数含义与取得方法')
table([
['符号','参数及单位','本车数值与计算依据'],
['L','设定移动距离 m','由用户输入。100 mm = 0.1 m；1000 mm = 1 m。'],
['D 与 r','有效轮径与半径 m','D = 103 ÷ 1000 = 0.103 m；r = D ÷ 2 = 0.0515 m。'],
['i','电机圈数 ÷ 车轮圈数','1:1 直驱，因此 i=1。若电机转 5 圈车轮转 1 圈，i=5。'],
['α','电机整步步距角 °','示例取 1.8°；应按实际电机步距角设置。'],
['μ','驱动器细分倍数','示例取 16；必须与驱动器实际细分一致。'],
['k','距离修正系数','无量纲，初始为 1。前后与左右分别标定。'],
['v','电机速度 RPM','当前默认 45 RPM；决定运动速度，不直接决定目标转角。']
],[49,147,303])
h('2 麦氏轮比例的适用条件')
p('以下按常见的 45° 滚子、X 型安装、车体不旋转、四轮有效轮径一致的理想运动学计算。纯前后和左右平移的轮缘等效位移大小均为 L，因此可使用同一周长换算，不应统一乘或除 √2，也不应因为有四个轮子就把单轮转角除以 4。')
p('左右运动更容易受到滚子、地面摩擦和打滑影响，实车误差用独立的 k 修正。其他滚子角度、其他安装方式、斜向运动或旋转，需重新建立运动学关系。')
story.append(PageBreak())

p('逐步计算 以移动 1 米为例','Heading1')
h('3 从距离得到车轮和电机转角')
p('第 1 步：统一单位。设定 L=1 m，轮径 D=0.103 m。圆周率 π 取 3.141592653589793，计算过程中保留足够精度，最后再取整。')
formula('车轮周长 C = π × D = 0.3235840433 m/圈')
p('第 2 步：移动距离除以一圈对应的行程，得到车轮转数。')
formula('车轮圈数 nw = L ÷ C = 3.0903872445 圈')
p('第 3 步：车轮圈数乘减速比，再乘所选方向的修正系数，得到要求电机转动的圈数。本例 i=1、k=1。')
formula('电机圈数 nm = nw × i × k = 3.0903872445 圈')
p('第 4 步：每圈 360°，因此电机角度为：')
formula('θm = nm × 360 = 1112.539408°')
p('这里的角度是一次任务的相对转角，可以大于 360°。不能对 360° 取余，否则 3 圈多会被错误地变成不足 1 圈。')
h('4 从电机转角得到驱动脉冲')
p('驱动器 FD 位置命令接受“微步脉冲数”。先用整步角与细分设置求每圈脉冲及每脉冲角度。细分改变脉冲数量，不改变同一距离需要的理论机械转角。')
formula('电机每圈脉冲 Sm = (360 ÷ α) × μ')
formula('本例 Sm = (360 ÷ 1.8) × 16 = 3200 脉冲/圈')
formula('每脉冲角度 δθ = α ÷ μ = 0.1125°/脉冲')
formula('目标脉冲 P = round(θm ÷ δθ)')
formula('等价公式 P = round[L × Sm × i × k ÷ (π × D)]')
p('本例未取整的脉冲为 9889.239182，四舍五入得到 P=9889。驱动实际要求角度为 9889 × 0.1125 = 1112.5125°，与理论角度相差约 0.0269°。这个取整误差对应理论行程约 0.024 mm，不足以解释明显的距离偏短。')
p('App 的“车轮每圈脉冲”填写 Sw=Sm×i。本车 Sw=3200；若 i=5、Sm=3200，则 Sw=16000。已填写 Sw 后，不要在每米脉冲计算中再次乘 i。')
story.append(PageBreak())

p('距离查表与四轮转动方向','Heading1')
h('5 常用米数的计算结果')
p('条件为 D=0.103 m、i=1、α=1.8°、μ=16、k=1。当前 App 先将每米脉冲取整，再由 STM32 计算目标脉冲。')
rows=[['距离 m','理论角度 °','当前目标脉冲','脉冲对应角度 °']]
for L in [.1,.2,.5,1,2]:
    P=math.floor(L*9889+.5)
    rows.append([f'{L:g}',f'{360*L/(math.pi*.103):.4f}',str(P),f'{P*.1125:.4f}'])
table(rows,[75,134,143,147])
p('例如移动 0.5 m：理论角度 556.2697°；当前程序计算 round(500×9889÷1000)=4945 脉冲，对应 556.3125°。整数脉冲存在微小量化差异。')
h('6 当前代码的四轮方向组合')
p('P 代表上一节计算出的正脉冲大小。轮序为左前 ID1、右前 ID2、右后 ID3、左后 ID4。下表是当前工程已有的实车方向映射。')
table([['移动方向','左前 1','右前 2','右后 3','左后 4'],
       ['F 前进','−P','−P','−P','−P'],['B 后退','+P','+P','+P','+P'],
       ['L 左移','+P','−P','+P','−P'],['R 右移','−P','+P','−P','+P']],
      [115,96,96,96,96])
p('正负号是软件中的方向符号，不直接等于所有电机的 CW/CCW。当前驱动将正号映射为左侧 CCW、右侧 CW，负号则反向。安装方向或轮位变更后，应重新核对映射。')
h('7 为什么纯平移不用额外乘 √2')
p('可用常规几何坐标理解：车体向前为 x、向左为 y，忽略旋转。在一种标准 X 型轮系的几何符号约定下，四轮等效位移为 x−y、x+y、x−y、x+y，轮序为左前、右前、右后、左后。纯前进令 y=0；纯左移令 x=0，单轮位移绝对值都是 L。')
p('如果斜向移动，才先分解 x=L cosβ、y=L sinβ，再代入四轮关系。其中 β 是行进方向与车体前向的夹角，不是滚子角。45° 斜向时两轮理想位移为 0，另两轮为 √2L。当前 App 只实现 F/B/L/R，未实现这一扩展。')
p('几何模型的符号约定与当前电机安装映射不同，不能直接把几何公式的正负号替换到现有驱动中。','SmallNote')
story.append(PageBreak())

p('对应驱动协议与实车标定','Heading1')
h('8 当前程序如何把米数变成位置帧')
formula('每米脉冲 q = round[1000 × Sw × k ÷ (π × Dmm)]')
formula('目标脉冲 P = round(distance_mm × q ÷ 1000)')
p('Dmm=103，Sw=3200，k=1，所以 q=9889。App 距离输入当前是整数毫米：想移动 1 m，应填 1000。当前两次取整与直接使用理论公式的结果可能相差少量脉冲。')
p('后退 1 米、45 RPM 的 App 命令示例：')
formula('MOVE,23,B,1000,45,9889')
p('23 是任务序号，不参与转角计算。STM32 得到 P=9889=0x000026A1，并为四个电机生成 FD 位置子帧。ID1 在本例使用 CCW，其子帧为：')
formula('01 FD 01 00 2D 0A 00 00 26 A1 02 00 6B')
p('依次表示：地址 01；功能 FD；方向 01=CCW；速度 00 2D=45 RPM；加速度档位 0A=10；四字节目标脉冲 00 00 26 A1；模式 02=相对当前实际位置；AA 内同步字段 00；校验 6B。方向 00 表示 CW，数值字段采用高字节在前。')
p('四个 13 字节子帧组成 AA 多电机命令：00 AA 00 39 + 四轮子帧 + 6B，共 57 字节。当前通过查询四轮 3A 状态判断完成。RPM 和加速度控制执行过程，目标脉冲决定请求转角；时间估算不能替代到位状态。')
h('9 修正系数怎样计算')
formula('新 k = 原 k × 设定距离 ÷ 实测距离')
p('例如正常完成前进 1 m，反复实测平均为 0.95 m，原 k=1，则新 k=1÷0.95=1.052632。理论请求角度变为 1112.539408×1.052632=1171.094114°；App 每米脉冲变为 round(9889.239182×1.052632)=10410。左右方向应单独测量并计算。')
p('只有收到正常 DONE、方向正确且多次结果稳定后才标定。当前出现 STM_CONFIRM_TIMEOUT 时，App 会约 2 秒后请求停车，不能把被中途停止的实测距离用于标定。公式给出电机目标转角，无法消除缺少回包、提前停车或随机打滑造成的误差。')
h('10 计算依据')
p('电机协议：ZDT_X42S第二代闭环步进电机用户手册V1.0.5_260527，第 49–50 页 AA 多电机命令，第 57 页 Emm FD 位置模式，第 73–74 页电机状态。参数：用户提供 103 mm 轮径、1:1 减速比；1.8° 与 16 细分为示例设定，需与驱动器实值一致。','SmallNote')
p('工程对应：DistanceMove.kt 的每米脉冲计算、robot_distance_protocol.h 的目标脉冲计算、robot_control.c 与 zdt_motor.c 的四轮方向及帧构造。麦氏轮运动学参考 WPILib 官方说明：<link href="https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/mecanum-drive-kinematics.html" color="#254E74">Mecanum Drive Kinematics</link>。','SmallNote')

def footer(c, doc):
    c.setFont('STSong-Light',9)
    c.setFillColor(colors.HexColor('#606060'))
    c.drawString(48,28,'103 mm 轮径  |  1:1 直驱  |  定距移动计算')
    c.drawRightString(547,28,f'{doc.page}')
doc=SimpleDocTemplate(str(OUT),pagesize=(595.28,841.89),rightMargin=48,leftMargin=48,
                      topMargin=42,bottomMargin=45,title='麦氏轮定距移动与张大头电机转角计算说明',author='')
doc.build(story,onFirstPage=footer,onLaterPages=footer)
qa=ROOT/'tmp/pdfs/distance_note_qa'
qa.mkdir(parents=True,exist_ok=True)
pdf=pdfium.PdfDocument(str(OUT))
for n,page in enumerate(pdf):
    page.render(scale=1.3).to_pil().save(qa/f'page-{n+1}.png')
print(f'PDF: {OUT}\nPages: {len(pdf)}')
