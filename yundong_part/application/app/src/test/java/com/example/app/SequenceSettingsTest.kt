package com.example.app

import org.junit.Assert.*
import org.junit.Test

class SequenceSettingsTest {
    @Test fun decimalRadiusAndSignedHeight() {
        val settings = SequenceSettings.parse("111.5", "-50", "130", "-110", "20", "20", "50")!!
        assertEquals(1115, settings.r1)
        assertEquals("STATE,7,P,1115,-50,1300,-110,20,20,50,60,0,60,0,248,140\n", settings.frame(7, 'P'))
    }
    @Test fun adjustableGripperSpeed() {
        assertEquals(30, SequenceSettings.parse("110", "-50", "20", "-40", "20", "20", "50", "30")!!.gripperDps)
        assertNull(SequenceSettings.parse("110", "-50", "20", "-40", "20", "20", "50", "0"))
    }
    @Test fun maximumSequenceSpeed() {
        assertNotNull(SequenceSettings.parse("110", "-50", "20", "-40", "160", "160", "160"))
        assertNull(SequenceSettings.parse("110", "-50", "20", "-40", "161", "160", "160"))
        assertNull(SequenceSettings.parse("110", "-50", "20", "-40", "160", "161", "160"))
    }
    @Test fun invalidInputIsRejected() {
        for (r in listOf("1000.1", "-1000.1", "1.23", "NaN", ""))
            assertNull(SequenceSettings.parse(r, "-50", "20", "-40", "20", "20", "50"))
        assertNull(SequenceSettings.parse("110", "-401", "20", "-40", "20", "20", "50"))
        assertNull(SequenceSettings.parse("110", "-50.5", "20", "-40", "20", "20", "50"))
        assertNull(SequenceSettings.parse("110", "-50", "20", "-40", "4", "20", "50"))
        assertNull(SequenceSettings.parse("110", "-50", "20", "-40", "20", "20", "161"))
    }
}
