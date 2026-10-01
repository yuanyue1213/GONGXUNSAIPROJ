package com.example.app

import org.junit.Assert.*
import org.junit.Test

class DistanceMoveTest {
    @Test fun wheel103DirectDriveConversion() {
        val move = DistanceMove.fromInputs('F', "1000", "45", "103", "3200", "1")!!
        assertEquals(9889, move.pulsesPerMetre)
        assertEquals("MOVE,7,F,1000,45,9889\n", move.frame(7))
    }

    @Test fun changedMicrostepsAndMeasuredCorrection() {
        val adjusted = DistanceMove.fromInputs('L', "100", "45", "103", "6400", "1.25")!!
        assertEquals(24723, adjusted.pulsesPerMetre)
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
