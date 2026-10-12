package com.example.app

import org.junit.Assert.*
import org.junit.Test

class AlignmentSettingsTest {
    @Test fun customCenter() {
        val settings = AlignmentSettings.parse("10", "20", "0", "511.9", "319.9")!!
        assertEquals("ALIGN_CFG,2,0,10000,10000,10,20,0,511.9,319.9\n", settings.frame(2, 0, 10000, 10000))
        assertNotNull(AlignmentSettings.parse("10", "20", "0", "0", "0"))
        for ((x, y) in listOf("512" to "160", "256" to "320", "-0.1" to "160",
            "256.01" to "160", "NaN" to "160", "256" to "Infinity")) {
            assertNull(AlignmentSettings.parse("10", "20", "0", x, y))
        }
    }
    @Test fun configurableSpeedAndOffset() {
        val settings = AlignmentSettings.parse("12", "30", "-7")!!
        assertEquals("ALIGN_CFG,1,2,9889,12000,12,30,-7\n", settings.frame(1, 2, 9889, 12000))
        assertEquals(0, AlignmentSettings.parse("10", "20", "0")!!.offsetMm)
    }
    @Test fun invalidParameters() {
        assertNull(AlignmentSettings.parse("4", "20", "10"))
        assertNull(AlignmentSettings.parse("10", "61", "10"))
        assertNull(AlignmentSettings.parse("10", "20", "101"))
        assertNull(AlignmentSettings.parse("10", "20", "1.5"))
    }
}
