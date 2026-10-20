package com.example.app
import org.junit.Assert.*
import org.junit.Test
class ImuFeedbackTest {
    @Test fun explainsFailureStageAndShowsCountersWithoutDeclaringMotionReady() {
        val text = ImuFeedback.text("IMU,1,NOT_READY,INIT_NO_REPLY,115200,0,0,0,0,0,0,0,0,0,0,0")
        assertTrue(text.contains("累计接收0字节")); assertTrue(text.contains("寄存器回应0"))
        assertTrue(text.contains("未获得可确认波特率"))
        assertTrue(ImuFeedback.text("IMU,1,NOT_READY,INIT_PARAM_MISMATCH").contains("不一致"))
        assertTrue(ImuFeedback.text("IMU,1,NOT_READY,INIT_STREAM_TIMEOUT").contains("角速度数据不足"))
        assertEquals("JY901S：展开角度 90.0°，Z角速度 0.0°/s", ImuFeedback.text("IMU,1,READY,90.0,0.0"))
    }
}
