package com.example.app

import org.junit.Assert.*
import org.junit.Test

class MotorFeedbackTest {
    @Test fun identifiesMotorsAndOperations() {
        assertEquals("底盘3号·右后轮：位置读取失败",
            MotorFeedback.text("MOTOR_DIAG,CHASSIS,3,POSITION_READ")?.second)
        assertEquals("机械臂1号·升降轴：原点位置读取失败",
            MotorFeedback.text("MOTOR_DIAG,ARM,1,ORIGIN_READ")?.second)
        assertEquals("机械臂2号·伸缩轴：停车指令应答失败",
            MotorFeedback.text("MOTOR_DIAG,ARM,2,STOP")?.second)
        assertNotEquals(MotorFeedback.text("MOTOR_DIAG,ARM,1,STOP")?.first,
            MotorFeedback.text("MOTOR_DIAG,CHASSIS,1,STOP")?.first)
        assertTrue(MotorFeedback.text("MOTOR_DIAG,CHASSIS,0,SPEED_COMMAND")!!.second.contains("无法确定"))
    }
    @Test fun rejectsMalformedFeedback() {
        listOf("MOTOR_DIAG,ARM,3,STOP", "MOTOR_DIAG,CHASSIS,5,STOP",
            "MOTOR_DIAG,ARM,0,STOP", "MOTOR_DIAG,OTHER,1,STOP",
            "MOTOR_DIAG,ARM,1,UNKNOWN", "MOTOR_DIAG,ARM,x,STOP",
            "MOTOR_DIAG,ARM,1,STOP,extra").forEach { assertNull(MotorFeedback.text(it)) }
    }
}
