package com.example.app

import kotlin.math.PI
import kotlin.math.roundToInt

/** The motor protocol counts microstep pulses, not encoder edges. */
internal data class DistanceMove(
    val direction: Char,
    val distanceMm: Int,
    val speedRpm: Int,
    val pulsesPerMetre: Int,
    val rotationDegrees: Double? = null,
    val wheelPulses: Int = 3200,
) {
    fun frame(sequence: Long): String = if (rotationDegrees != null)
        "EXEC,$sequence,TURN,$direction,${java.lang.String.format(java.util.Locale.US, "%.1f", rotationDegrees)},$speedRpm\n"
        else {
            val pulses = (distanceMm.toLong() * pulsesPerMetre + 500L) / 1000L
            "EXEC,$sequence,ROUTE_MOVE,$direction,$pulses,$speedRpm,$wheelPulses\n"
        }

    companion object {
        fun gyroRotationFromInputs(direction: Char, angle: String, speed: String): DistanceMove? {
            val tenths = try { java.math.BigDecimal(angle.trim()).movePointRight(1).intValueExact() }
                catch (_: IllegalArgumentException) { return null } catch (_: ArithmeticException) { return null }
            val rpm = speed.toIntOrNull() ?: return null
            if (direction !in "CW" || tenths !in 1..3600 || rpm !in 5..120) return null
            return DistanceMove(direction,0,rpm,0,tenths/10.0)
        }
        fun rotationFromInputs(
            direction: Char, angle: String, speed: String,
            diameter: String, wheelPulses: String, correction: String,
            wheelbase: String, track: String,
        ): DistanceMove? {
            val degrees = angle.toDoubleOrNull() ?: return null
            val baseMm = wheelbase.toDoubleOrNull() ?: return null
            val trackMm = track.toDoubleOrNull() ?: return null
            val calibration = fromInputs(direction, "1000", speed, diameter, wheelPulses, correction)
                ?: return null
            if (direction !in "CW" || !degrees.isFinite() || degrees !in 0.1..360.0 ||
                calibration.speedRpm !in 5..120 || !baseMm.isFinite() || baseMm !in 50.0..2000.0 ||
                !trackMm.isFinite() || trackMm !in 50.0..2000.0) return null
            val wheelTravelMm = PI * (baseMm + trackMm) * degrees / 360.0
            val pulses = (wheelTravelMm * calibration.pulsesPerMetre / 1000.0).roundToInt()
            if (pulses !in 1..1000000) return null
            // MOVE computes round(distance * ppm / 1000). Encode the exact rounded
            // pulse target using distance=1000, without rounding wheel travel to mm.
            return DistanceMove(direction, 1000, calibration.speedRpm, pulses, degrees)
        }

        fun fromInputs(
            direction: Char, distance: String, speed: String,
            diameter: String, wheelPulses: String, correction: String,
        ): DistanceMove? {
            val mm = distance.toIntOrNull() ?: return null
            val rpm = speed.toIntOrNull() ?: return null
            val diameterMm = diameter.toDoubleOrNull() ?: return null
            val pulses = wheelPulses.toIntOrNull() ?: return null
            val factor = correction.toDoubleOrNull() ?: return null
            if (direction !in "FBLRCW" || mm !in 1..10000 || rpm !in 5..300 ||
                !diameterMm.isFinite() || diameterMm !in 20.0..500.0 ||
                pulses !in 200..51200 || !factor.isFinite() || factor !in 0.1..10.0) return null
            val perMetre = (1000.0 * pulses * factor / (PI * diameterMm)).roundToInt()
            if (perMetre !in 1..1000000 || mm.toLong() * perMetre < 500L) return null
            return DistanceMove(direction, mm, rpm, perMetre, wheelPulses = pulses)
        }
    }
}
