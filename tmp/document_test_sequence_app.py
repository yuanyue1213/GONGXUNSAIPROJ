from pathlib import Path
base=Path('yundong_part/application/app/src/test/java/com/example/app')
(base/'SequenceSettingsTest.kt').write_text('''package com.example.app

import org.junit.Assert.*
import org.junit.Test

class SequenceSettingsTest {
    @Test fun decimalRadiusAndSignedHeight() {
        val settings = SequenceSettings.parse("111.5", "-50", "130", "-110", "20", "20", "50")!!
        assertEquals(1115, settings.r1)
        assertEquals("STATE,7,P,1115,-50,1300,-110,20,20,50\\n", settings.frame(7, 'P'))
    }
    @Test fun invalidInputIsRejected() {
        for (r in listOf("1000.1", "-1000.1", "1.23", "NaN", ""))
            assertNull(SequenceSettings.parse(r, "-50", "20", "-40", "20", "20", "50"))
        assertNull(SequenceSettings.parse("110", "-401", "20", "-40", "20", "20", "50"))
        assertNull(SequenceSettings.parse("110", "-50.5", "20", "-40", "20", "20", "50"))
        assertNull(SequenceSettings.parse("110", "-50", "20", "-40", "4", "20", "50"))
        assertNull(SequenceSettings.parse("110", "-50", "20", "-40", "20", "20", "61"))
    }
}
''',encoding='utf-8')
p=base/'RobotTcpClientTest.kt';s=p.read_text(encoding='utf-8').replace('List(15)','List(17)');a='                client.sendRingAlignment(2, 9889, 12000)';assert a in s;s=s.replace(a,a+'''
                client.sendSequence('X', SequenceSettings(1100, -50, 200, -40, 20, 20, 50))
                client.sendSequence('A', SequenceSettings(1100, -50, 200, -40, 20, 20, 50))
                client.sendSequence('P', SequenceSettings(200, -40, 1300, -110, 25, 15, 55))''');a='                assertTrue(frames[14].startsWith("ALIGN_RING,") && frames[14].endsWith(",2,9889,12000"))';assert a in s;s=s.replace(a,a+'''
                assertTrue(frames[15].startsWith("STATE,") && frames[15].endsWith(",A,1100,-50,200,-40,20,20,50"))
                assertTrue(frames[16].startsWith("STATE,") && frames[16].endsWith(",P,200,-40,1300,-110,25,15,55"))''');p.write_text(s,encoding='utf-8')
text='''
## App 可编辑抓取、放下参数

App 的抓取与放下各有两个目标位置：抓取第 2/6 步、放下第 3/6 步。r 可输入 0.1 mm 精度（±1000 mm），z 为整数 mm（±400 mm）；分别设置伸缩、上升、下降速度（5–60 RPM）。保持位置步骤沿用上一位置，回零步骤固定 r=z=0。原有执行顺序、舵机角度、渐变与停顿保持。

点击执行时一次发送 `STATE,seq,A或P,r1_十分之一毫米,z1_mm,r2_十分之一毫米,z2_mm,伸缩RPM,上升RPM,下降RPM\\n`。
放下默认示例：`STATE,1,P,200,-40,1300,-110,20,20,50\\n`。
ESP32 校验后原样转发。STM32 校验全部字段、原点及空闲状态后复制整套参数并启动；非法、重复或忙碌指令忽略，不会改变正在执行的流程。没有应用回包。每次执行都附带参数，因此重启固件后也无需单独同步。App 使用 rememberSaveable 保存界面输入；未额外实现跨冷启动的永久配置存储。
传统 `CMD,seq,A/P` 仍使用固件默认位置与速度。
'''
p=Path('yundong_part/esp32s3/PROTOCOL.md');p.write_text(p.read_text(encoding='utf-8')+text,encoding='utf-8')
p=Path('output/目前已有状态与扩展说明.md');p.write_text(p.read_text(encoding='utf-8')+'''
抓取第 2、6 步及放下第 3、6 步的 r、z 可以在 App 修改；保持步骤沿用上一位置，回零步骤固定为零。每个流程可独立设置伸缩、上升、下降速度。点击执行时使用当前输入参数。
''',encoding='utf-8')
