package com.example.app
import org.junit.Assert.*
import org.junit.Test
class SequenceSettingsTest {
    @Test fun decimalRadiusAndCombinedSpeedFrame() {
        val settings = SequenceSettings.parse("111.5", "-50", "130", "-110", "600")!!
        assertEquals(1115, settings.r1)
        assertEquals(600, settings.combinedRpm)
        assertEquals("STATE_V2,7,P,1115,-50,1300,-110,600,60,0,60,0,248,140\n", settings.frame(7, 'P'))
    }
    @Test fun invalidParameters() {
        for (speed in listOf("4", "601", "NaN", "20.5", ""))
            assertNull(SequenceSettings.parse("110", "-50", "20", "-40", speed))
        for (r in listOf("1000.1", "-1000.1", "1.23", "NaN", ""))
            assertNull(SequenceSettings.parse(r, "-50", "20", "-40", "20"))
        assertNull(SequenceSettings.parse("110", "-401", "20", "-40", "20"))
        assertNull(SequenceSettings.parse("110", "-50.5", "20", "-40", "20"))
        assertNull(SequenceSettings.parse("110", "-50", "20", "-40", "20", "0"))
        assertEquals(30, SequenceSettings.parse("110", "-50", "20", "-40", "20", "30")!!.gripperDps)
    }
    @Test fun legacyCardMigratesRadialSpeedAndRetainsAngles() {
        val card = SequenceCardCodec.decode("old|saved|A,1105,-50,200,-40,25,20,50,30,0,65,5,249,141").single()
        assertEquals(25, card.settings.combinedRpm)
        assertEquals(1105, card.settings.r1)
        assertEquals(141, card.settings.baseTilt)
        assertEquals(listOf(card), SequenceCardCodec.decode(SequenceCardCodec.encode(listOf(card))))
    }
}
