package com.example.app

import org.junit.Assert.*
import org.junit.Test

class DistanceMoveTest {
    private fun rotation(direction: Char = 'C', angle: String = "90", rpm: String = "10",
        base: String = "190", track: String = "251.4", factor: String = "1") =
        DistanceMove.rotationFromInputs(direction, angle, rpm, "103", "3200", factor, base, track)

    @Test fun fixedAngleRotationEncodesRoundedWheelPulses() {
        val left = rotation()!!
        val expected = (kotlin.math.PI * 441.4 * 90 / 360 * 9889 / 1000).let {
            kotlin.math.round(it).toInt()
        }
        assertEquals("EXEC,7,TURN,C,90.0,10\n", left.frame(7))
        assertEquals(90.0, left.rotationDegrees!!, 0.0)
        assertEquals(left.pulsesPerMetre, rotation('W')!!.pulsesPerMetre)
        assertTrue(kotlin.math.abs(expected * 4 - rotation(angle = "360")!!.pulsesPerMetre) <= 2)
        assertTrue(rotation(angle = "0.1")!!.pulsesPerMetre > 0)
        assertTrue(kotlin.math.abs(expected * 2 - rotation(factor = "2")!!.pulsesPerMetre) <= 1)
    }

    @Test fun invalidRotationInputsAreRejected() {
        for (angle in listOf("0", "-90", "0.01", "360.1", "NaN", "Infinity", "abc"))
            assertNull(rotation(angle = angle))
        for (base in listOf("49", "2001", "NaN", "Infinity")) assertNull(rotation(base = base))
        for (track in listOf("49", "2001", "NaN", "Infinity")) assertNull(rotation(track = track))
        assertNull(rotation('F'))
        assertNotNull(rotation(rpm = "120"))
        assertNull(rotation(rpm = "121"))
        assertNull(rotation(factor = "0"))
        assertNull(DistanceMove.rotationFromInputs('C', "360", "10", "20", "62831", "0.1", "2000", "2000"))
    }
    @Test fun measuredRotationCalibrationMatchesAutonomousDefault() {
        assertEquals(3291, rotation(factor=DEFAULT_TURN_CORRECTION.toString())!!.pulsesPerMetre)
    }
    @Test fun gyroTurnNeedsOnlyAngleAndSpeed() {
        assertEquals("EXEC,42,TURN,W,90.0,20\n",DistanceMove.gyroRotationFromInputs('W',"90","20")!!.frame(42))
        assertNull(DistanceMove.gyroRotationFromInputs('W',"90.01","20"))
        assertNull(DistanceMove.gyroRotationFromInputs('F',"90","20"))
        assertNull(DistanceMove.gyroRotationFromInputs('C',"0","20"))
        assertNotNull(DistanceMove.gyroRotationFromInputs('C',"90","120"))
        assertNull(DistanceMove.gyroRotationFromInputs('C',"90","121"))
    }

    @Test fun wheel103DirectDriveConversion() {
        val move = DistanceMove.fromInputs('F', "1000", "45", "103", "3200", "1")!!
        assertEquals(9889, move.pulsesPerMetre)
        assertEquals("EXEC,7,ROUTE_MOVE,F,9889,45,3200\n", move.frame(7))
    }
    @Test fun rawPulseTurnsUseSynchronousPositionMode() {
        assertEquals("EXEC,12,ROUTE_MOVE,C,989,20,3200\n",
            DistanceMove.fromInputs('C',"100","20","103","3200","1")!!.frame(12))
        assertEquals("EXEC,13,ROUTE_MOVE,W,989,20,3200\n",
            DistanceMove.fromInputs('W',"100","20","103","3200","1")!!.frame(13))
    }

    @Test fun changedMicrostepsAndMeasuredCorrection() {
        val adjusted = DistanceMove.fromInputs('L', "100", "45", "103", "6400", "1.25")!!
        assertEquals(24723, adjusted.pulsesPerMetre)
        val back = DistanceMove.fromInputs('B', "100", "45", "103", "6400", "1.25")!!
        assertEquals("EXEC,8,ROUTE_MOVE,B,2472,45,6400\n",back.frame(8))
        assertEquals("EXEC,9,ROUTE_MOVE,L,2472,45,6400\n",adjusted.frame(9))
        assertNull(DistanceMove.fromInputs('F',"100","20","103","51201","1"))
    }

    @Test fun invalidInputsCannotProduceMovement() {
        for (distance in listOf("0", "-1", "10001", "1.5", "abc", "2147483648"))
            assertNull(DistanceMove.fromInputs('F', distance, "45", "103", "3200", "1"))
        for (speed in listOf("0", "4", "301", "NaN"))
            assertNull(DistanceMove.fromInputs('F', "100", speed, "103", "3200", "1"))
        for (diameter in listOf("0", "NaN", "Infinity", "19", "501"))
            assertNull(DistanceMove.fromInputs('F', "100", "45", diameter, "3200", "1"))
        for (factor in listOf("0", "NaN", "Infinity", "-1", "11"))
            assertNull(DistanceMove.fromInputs('F', "100", "45", "103", "3200", factor))
        assertNull(DistanceMove.fromInputs('S', "100", "45", "103", "3200", "1"))
    }
}
