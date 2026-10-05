package com.example.app

import java.math.BigDecimal

internal data class SequenceSettings(
    val r1: Int, val z1: Int, val r2: Int, val z2: Int,
    val radialRpm: Int, val upRpm: Int, val downRpm: Int, val gripperDps: Int = 60,
    val theta: Int = 0, val openAngle: Int = 60, val closeAngle: Int = 0,
    val baseHome: Int = 248, val baseTilt: Int = 140,
) {
    fun valid() = r1 in -10000..10000 && r2 in -10000..10000 &&
        z1 in -400..400 && z2 in -400..400 &&
        radialRpm in 5..160 && upRpm in 5..160 && downRpm in 5..160 && gripperDps in 6..300 && theta in 0..270 && openAngle in 0..270 && closeAngle in 0..270 && baseHome in 0..360 && baseTilt in 0..360
    fun frame(sequence: Long, mode: Char) =
        "STATE,$sequence,$mode,$r1,$z1,$r2,$z2,$radialRpm,$upRpm,$downRpm,$gripperDps,$theta,$openAngle,$closeAngle,$baseHome,$baseTilt\n"
    companion object {
        fun parse(r1: String, z1: String, r2: String, z2: String,
                  radial: String, up: String, down: String, gripper: String = "60", theta: String = "0", open: String = "60", close: String = "0",
                  baseHome: String = "248", baseTilt: String = "140"): SequenceSettings? = try {
            SequenceSettings(BigDecimal(r1.trim()).movePointRight(1).intValueExact(),
                z1.trim().toInt(), BigDecimal(r2.trim()).movePointRight(1).intValueExact(),
                z2.trim().toInt(), radial.trim().toInt(), up.trim().toInt(), down.trim().toInt(), gripper.trim().toInt(), theta.trim().toInt(), open.trim().toInt(),
                close.trim().toInt(), baseHome.trim().toInt(), baseTilt.trim().toInt())
                .takeIf { it.valid() }
        } catch (_: IllegalArgumentException) { null }
          catch (_: ArithmeticException) { null }
    }
}
