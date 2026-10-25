package com.example.app

import java.util.Locale

/** User-provided physical positions 1/2/3 = left/middle/right. R uses only near pose 2. */
internal object ScreenshotArmDefaults {
    const val SLOT_ANGLES = "0,120,180" // Third angle was not visible: retain previous 180 default.
    private val pickR = listOf(1540, 1190, 1850)
    private val putR = listOf(1520, 1200, 1850)
    private val heights = listOf(-128, -126, -120)
    private val angles = listOf(209, 170, 134)

    fun settings(role: Char, position: Int): SequenceSettings {
        require(role in "RGPTK" && position in 1..3)
        val i = (if (role == 'R') 2 else position) - 1
        return if (role in "RG") SequenceSettings(pickR[i], heights[i], 110, -36, 300,
            gripperDps = 60, openAngle = 30, closeAngle = 0, baseHome = angles[i], baseTilt = 57)
        else SequenceSettings(110, -36, putR[i], heights[i] + if (role == 'K') 100 else 0, 300,
            gripperDps = 10, openAngle = 30, closeAngle = 0, baseHome = angles[i], baseTilt = 57)
    }

    fun fields(role: Char, position: Int): Map<String, String> {
        val s = settings(role, position)
        fun mm(v: Int) = String.format(Locale.US, "%.1f", v / 10.0)
        return mapOf("r1" to mm(s.r1), "z1" to s.z1.toString(), "r2" to mm(s.r2), "z2" to s.z2.toString(),
            "rpm" to s.combinedRpm.toString(), "gripper" to s.gripperDps.toString(),
            "open" to s.openAngle.toString(), "close" to s.closeAngle.toString(),
            "home" to s.baseHome.toString(), "tilt" to s.baseTilt.toString())
    }

    fun cards(): List<SequenceCard> = (1..3).map {
        SequenceCard("default-pick-$it", "抓取$it", 'A', settings('G', it))
    } + (1..3).map {
        SequenceCard("default-put-$it", "放下$it", 'P', settings('P', it))
    } + listOf(SequenceCard("default-turn-0", "转盘0", 'T', turn = TurnSettings(0, 0, 60)),
        SequenceCard("default-turn-120", "转盘120", 'T', turn = TurnSettings(0, 120, 60)))
}
