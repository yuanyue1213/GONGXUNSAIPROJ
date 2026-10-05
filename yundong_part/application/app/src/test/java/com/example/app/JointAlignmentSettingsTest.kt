package com.example.app

import org.junit.Assert.*
import org.junit.Test

class JointAlignmentSettingsTest {
    @Test fun geometryAndFrame() {
        val geometry = JointAlignmentSettings(190.0, 251.4, -7.0, -291.27, true)
        assertTrue(geometry.valid())
        assertEquals("ALIGN_POSE,42,2,9889,12000,10,20,10,190,251.4,-7,-291.27,1\n",
            geometry.frame(42, 2, 9889, 12000, AlignmentSettings()))
        assertFalse(geometry.copy(wheelbaseMm = 49.0).valid())
        assertFalse(geometry.copy(trackMm = 2001.0).valid())
        assertFalse(geometry.copy(cameraForwardMm = -2001.0).valid())
    }
}
