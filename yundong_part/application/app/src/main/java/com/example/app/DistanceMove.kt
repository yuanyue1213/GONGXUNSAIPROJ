package com.example.app

import kotlin.math.PI
import kotlin.math.roundToInt

/** The motor protocol counts microstep pulses, not encoder edges. */
internal data class DistanceMove(
    val direction: Char,
    val distanceMm: Int,
    val speedRpm: Int,
    val pulsesPerMetre: Int,
) {
    fun frame(sequence: Long): String =
        "MOVE,$sequence,$direction,$distanceMm,$speedRpm,$pulsesPerMetre\n"

    companion object {
        fun fromInputs(
            direction: Char, distance: String, speed: String,
            diameter: String, wheelPulses: String, correction: String,
        ): DistanceMove? {
            val mm = distance.toIntOrNull() ?: return null
            val rpm = speed.toIntOrNull() ?: return null
            val diameterMm = diameter.toDoubleOrNull() ?: return null
            val pulses = wheelPulses.toIntOrNull() ?: return null
            val factor = correction.toDoubleOrNull() ?: return null
            if (direction !in "FBLR" || mm !in 1..10000 || rpm !in 5..300 ||
                !diameterMm.isFinite() || diameterMm !in 20.0..500.0 ||
                pulses !in 1..1000000 || !factor.isFinite() || factor !in 0.1..10.0) return null
            val perMetre = (1000.0 * pulses * factor / (PI * diameterMm)).roundToInt()
            if (perMetre !in 1..1000000 || mm.toLong() * perMetre < 500L) return null
            return DistanceMove(direction, mm, rpm, perMetre)
        }
    }
}
