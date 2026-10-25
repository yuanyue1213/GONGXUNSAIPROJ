package com.example.app

import org.junit.Assert.*
import org.junit.Test

class ScreenshotArmDefaultsTest {
    @Test fun screenshotDistancesKeepTenthsOfMillimetresAndGrabReleaseDifferences() {
        val grabs = listOf(1540, 1190, 1850)
        val puts = listOf(1520, 1200, 1850)
        val heights = listOf(-128, -126, -120)
        val bases = listOf(209, 170, 134)
        for (position in 1..3) {
            val i = position - 1
            val grab = ScreenshotArmDefaults.settings('G', position)
            assertEquals(SequenceSettings(grabs[i], heights[i], 110, -36, 300, 60,
                openAngle = 30, baseHome = bases[i], baseTilt = 57), grab)
            for (role in "PT") assertEquals(SequenceSettings(110, -36, puts[i], heights[i], 300, 10,
                openAngle = 30, baseHome = bases[i], baseTilt = 57), ScreenshotArmDefaults.settings(role, position))
        }
    }
    @Test fun nearPoseUsesOnlyGrabTwoAndFieldsRoundTripInMillimetres() {
        for (position in 1..3) assertEquals(ScreenshotArmDefaults.settings('G', 2), ScreenshotArmDefaults.settings('R', position))
        val fields = ScreenshotArmDefaults.fields('R', 2)
        assertEquals("119.0", fields["r1"])
        assertEquals("11.0", fields["r2"])
        val parsed = SequenceSettings.parse(fields.getValue("r1"), fields.getValue("z1"), fields.getValue("r2"),
            fields.getValue("z2"), fields.getValue("rpm"), fields.getValue("gripper"), open = fields.getValue("open"),
            close = fields.getValue("close"), baseHome = fields.getValue("home"), baseTilt = fields.getValue("tilt"))
        assertEquals(ScreenshotArmDefaults.settings('G', 2), parsed)
        for (position in 1..3) {
            val placed = ScreenshotArmDefaults.settings('P', position)
            assertEquals(placed.copy(z2 = placed.z2 + 100), ScreenshotArmDefaults.settings('K', position))
        }
        assertEquals(listOf(-28, -26, -20), (1..3).map { ScreenshotArmDefaults.settings('K', it).z2 })
    }
    @Test fun factoryLibraryContainsValidGrabReleaseAndVisibleTurnCards() {
        val cards = ScreenshotArmDefaults.cards()
        assertEquals(8, cards.size)
        assertTrue(cards.all { it.valid() })
        assertEquals(cards, SequenceCardCodec.decode(SequenceCardCodec.encode(cards)))
        assertEquals(TurnSettings(0, 120, 60), cards.last().turn)
        assertTrue(cards.first().frame(1).contains(",1540,-128,110,-36,300,60,"))
    }
}
