package com.example.app
import org.junit.Assert.*
import org.junit.Test

class AutonomousTaskTest {
    private val settings = AutonomousSettings(9889, 10000, 45, 10, 256.4f, 160.3f, "321", "123", listOf(0, 90, 180), true, true)
    private val parameters = "RGPTK".flatMap { role -> (1..3).map { slot ->
        AutonomousArmParameters(role,slot,SequenceSettings(1100,-50,200,-40,20))
    } }
    @Test fun uploadIndependentParametersAndRouteBeforeStart() {
        val frames = settings.frames(10, 11, parameters)
        assertEquals(39, frames.size)
        assertEquals("AUTO_CFG,10,9889,10000,45,10,256.4,160.3,321,123,0,90,180\n", frames.first())
        assertEquals("AUTO_OPTIONS,10,1,1,-\n",frames[1])
        assertEquals("AUTO_DRIVE,10,3291,20,3200\n",frames[2])
        assertEquals("AUTO_ROUTE,10,1,2200,2200\n",frames[4])
        assertTrue(frames[23].startsWith("AUTO_ARM,10,R,1,STATE_V2,10,A,"))
        assertTrue(frames[29].startsWith("AUTO_ARM,10,P,1,STATE_V2,10,P,"))
        assertTrue(frames[37].startsWith("AUTO_ARM,10,K,3,"))
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
        assertFalse(settings.copy(route = settings.route.dropLast(1)).valid())
        assertFalse(settings.copy(route = settings.route.dropLast(1)+AutonomousRoutePoint(2200,150)).valid())
    }
    @Test fun chassisOnlyNeedsNoArmParametersOrVisualSettings() {
        val mode = settings.copy(visionEnabled = false, armEnabled = false, slots = emptyList(), targetX = Float.NaN, processMap = "bad")
        val frames = mode.frames(10,11,emptyList())
        assertEquals(24,frames.size)
        assertEquals("AUTO_OPTIONS,10,0,0,-\n",frames[1])
        assertFalse(frames.any { it.startsWith("AUTO_ARM,") })
    }
    @Test fun manualTaskCodeForArmWithoutVision() {
        val mode = settings.copy(visionEnabled = false, manualCode = "156+123+516+231")
        assertTrue(mode.frames(10,11,parameters).contains("AUTO_OPTIONS,10,0,1,156+123+516+231\n"))
        assertTrue(mode.copy(manualCode = "").frames(10,11,parameters).isEmpty())
        assertFalse(validAutonomousCode("156+123+234+231"))
        assertFalse(validAutonomousCode("116+123+611+231"))
        assertFalse(validAutonomousCode("123+321"))
    }
    @Test fun visualOnlyNeedsNoArmParameters() {
        val frames=settings.copy(armEnabled=false).frames(10,11,emptyList())
        assertEquals(24,frames.size)
        assertEquals("AUTO_OPTIONS,10,1,0,-\n",frames[1])
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
        assertFalse(settings.copy(turnRpm=61).valid())
        assertFalse(settings.copy(forwardPpm=1000000,wheelbaseMm=2000.0,trackMm=2000.0).valid())
        assertTrue(AutonomousStatus.text("AUTO,11,TURNING,2,0,0,0,0").contains("原地转向"))
    }
}
