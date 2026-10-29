package com.example.app
import org.junit.Assert.*
import org.junit.Test

class AutonomousTaskTest {
    @Test fun routeSpeedSupports1200ButRetainsFineAndTurnLimits() {
        assertTrue(settings.copy(rpm=1200).valid())
        assertTrue(settings.copy(rpm=1200).frames(10,11,parameters).first().contains(",1200,10,"))
        assertFalse(settings.copy(rpm=1201).valid())
        assertFalse(settings.copy(fineRpm=301).valid())
        assertTrue(settings.copy(turnRpm=120).valid())
        assertFalse(settings.copy(turnRpm=121).valid())
    }
    private val settings = AutonomousSettings(9889, 10000, 45, 10, 256.4f, 160.3f, "321", "123", listOf(0, 90, 180), true, true)
    private val parameters = "RGPTK".flatMap { role -> (1..3).map { slot ->
        AutonomousArmParameters(role,slot,SequenceSettings(1100,-50,200,-40,20))
    } }
    @Test fun defaultCodeSkipsQrWithoutDisablingPositionVision() {
        val preset = settings.copy(useDefaultTaskCode = true, manualCode = "unused")
        val frames = preset.frames(10, 11, parameters)
        assertEquals("AUTO_OPTIONS,10,1,1,156+123+516+231\n", frames[1])
        assertEquals("AUTO_PATH,10,BODY_NEAR_DEFAULT_V1\n", frames[3])
        assertEquals(20, frames.size)
        val normal = preset.copy(useDefaultTaskCode = false).frames(10, 11, parameters)
        assertEquals("AUTO_OPTIONS,10,1,1,-\n", normal[1])
        assertEquals("AUTO_PATH,10,BODY_NEAR_V3\n", normal[3])
        assertEquals("AUTO_OPTIONS,10,0,1,156+123+516+231\n", preset.copy(visionEnabled = false).frames(10,11,parameters)[1])
        val visualOnly = preset.copy(armEnabled = false).frames(10,11,emptyList())
        assertEquals("AUTO_OPTIONS,10,1,0,156+123+516+231\n", visualOnly[1])
        assertTrue(validAutonomousCode(DEFAULT_TASK_CODE))
    }
    @Test fun uploadIndependentParametersAndRouteBeforeStart() {
        val frames = settings.frames(10, 11, parameters)
        assertEquals(20, frames.size)
        assertEquals("AUTO_CFG,10,9889,10000,45,10,256.4,160.3,321,123,0,90,180\n", frames.first())
        assertEquals("AUTO_OPTIONS,10,1,1,-\n",frames[1])
        assertEquals("AUTO_DRIVE,10,3291,40,3200\n",frames[2])
        assertEquals("AUTO_PATH,10,BODY_NEAR_V3\n",frames[3])
        assertTrue(frames[4].startsWith("AUTO_ARM,10,R,1,STATE_V2,10,A,"))
        assertTrue(frames[10].startsWith("AUTO_ARM,10,P,1,STATE_V2,10,P,"))
        assertTrue(frames[18].startsWith("AUTO_ARM,10,K,3,"))
        assertFalse(frames.any { it.startsWith("AUTO_ROUTE,") })
        assertEquals(25,AutonomousRoute.names.size)
        assertTrue(AutonomousRoute.names[1].contains("后退1000"))
        assertTrue(AutonomousRoute.names[16].contains("放置后抓回"))
        assertEquals("AUTO_START,11,10\n", frames.last())
        assertTrue(frames.all { it.length <= 192 && it.endsWith('\n') })
    }
    @Test fun rejectMissingOrDuplicateParametersAndInvalidMapping() {
        assertTrue(settings.frames(10, 11, parameters.dropLast(1)).isEmpty())
        assertTrue(settings.frames(10, 11, List(15) { parameters[0] }).isEmpty())
        assertFalse(settings.copy(processMap = "113").valid())
        assertFalse(settings.copy(slots = listOf(0, 0, 180)).valid())
        assertFalse(settings.copy(targetX = Float.NaN).valid())
        assertFalse(settings.copy(targetX = 511.99f).valid())
    }
    @Test fun chassisOnlyNeedsNoArmParametersOrVisualSettings() {
        val mode = settings.copy(visionEnabled = false, armEnabled = false, slots = emptyList(), targetX = Float.NaN, processMap = "bad")
        val frames = mode.frames(10,11,emptyList())
        assertEquals(5,frames.size)
        assertEquals("AUTO_OPTIONS,10,0,0,-\n",frames[1])
        assertFalse(frames.any { it.startsWith("AUTO_ARM,") })
    }
    @Test fun armWithoutVisionUsesFixedProfilesAndDefaultTaskCode() {
        val mode = settings.copy(visionEnabled = false, manualCode = "156+123+516+231")
        assertEquals("AUTO_OPTIONS,10,0,1,156+123+516+231\n", mode.frames(10,11,parameters)[1])
        assertEquals("AUTO_PATH,10,BODY_NEAR_DEFAULT_V1\n", mode.copy(manualCode = "").frames(10,11,parameters)[3])
        assertFalse(validAutonomousCode("156+123+234+231"))
        assertFalse(validAutonomousCode("116+123+611+231"))
        assertFalse(validAutonomousCode("123+321"))
    }
    @Test fun visualOnlyNeedsNoArmParameters() {
        val frames=settings.copy(armEnabled=false).frames(10,11,emptyList())
        assertEquals(5,frames.size)
        assertEquals("AUTO_OPTIONS,10,1,0,-\n",frames[1])
    }
    @Test fun rawWaitStatusIsReadable() {
        assertTrue(AutonomousStatus.text("AUTO,11,WORK,4,0,0,0,WAIT_RAW_TARGET").contains("等待目标颜色"))
        assertTrue(AutonomousStatus.text("AUTO,11,STOPPED,4,0,0,0,RAW_TARGET_TIMEOUT").contains("近车位超时"))
    }
    @Test fun stoppedStageAndOriginalErrorAreVisible() {
        val text = AutonomousStatus.text("AUTO,11,STOPPED,4,1,0,0,ARM:ODOM_READ")
        assertTrue(text.contains("停止环节：机械臂取放"))
        assertTrue(text.contains("第2件"))
        assertTrue(text.contains("编码器失败"))
        assertTrue(AutonomousStatus.text("AUTO,11,WORK,4,0,0,0,ALIGN:WAIT_VISION").contains("工位中心识别"))
    }
    @Test fun cameraCommunicationErrorsAreDistinct() {
        assertTrue(AutonomousStatus.text("AUTO,11,STOPPED,1,0,0,0,QR:CAMERA_TX_FAILED").contains("发送请求失败"))
        assertTrue(AutonomousStatus.text("AUTO,11,STOPPED,1,0,0,0,QR:CAMERA_NO_RESPONSE").contains("未收到相机回包"))
        assertTrue(AutonomousStatus.text("AUTO,11,WORK,1,0,0,0,QR:CAMERA_RESPONDING").contains("双向通信已收到回包"))
    }
    @Test fun taskStatusIsReadable() {
        assertTrue(AutonomousStatus.text("AUTO,11,STOPPED,2,0,0,0,VISION_TIMEOUT").contains("视觉识别超时"))
        assertEquals("任务码：156+123+516+231", AutonomousStatus.text("TASK,11,156+123+516+231"))
    }
    @Test fun rotationUsesGeometryAndRejectsUnsafeCalibration() {
        assertEquals(3291, settings.turnPulses())
        assertEquals(6857, settings.copy(turnCorrection=2.0).turnPulses())
        assertFalse(settings.copy(wheelbaseMm=Double.NaN).valid())
        assertFalse(settings.copy(trackMm=0.0).valid())
        assertFalse(settings.copy(turnCorrection=0.0).valid())
        assertFalse(settings.copy(turnRpm=121).valid())
        assertFalse(settings.copy(forwardPpm=1000000,wheelbaseMm=2000.0,trackMm=2000.0).valid())
        assertTrue(AutonomousStatus.text("AUTO,11,TURNING,2,0,0,0,0").contains("原地转向"))
    }
}
