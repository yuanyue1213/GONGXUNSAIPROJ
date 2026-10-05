package com.example.app

internal data class JointAlignmentSettings(
    val wheelbaseMm: Double, val trackMm: Double,
    val cameraForwardMm: Double, val cameraLeftMm: Double, val reverse: Boolean,
) {
    fun valid() = wheelbaseMm in 50.0..2000.0 && trackMm in 50.0..2000.0 &&
        cameraForwardMm in -2000.0..2000.0 && cameraLeftMm in -2000.0..2000.0
    private fun mm(value: Double) = java.math.BigDecimal.valueOf(value).setScale(2, java.math.RoundingMode.HALF_UP).stripTrailingZeros().toPlainString()
    internal fun frame(sequence: Long, ring: Int, forward: Int, lateral: Int, settings: AlignmentSettings) =
        "ALIGN_POSE,$sequence,$ring,$forward,$lateral,${settings.fineRpm},${settings.coarseRpm},${settings.offsetMm},${mm(wheelbaseMm)},${mm(trackMm)},${mm(cameraForwardMm)},${mm(cameraLeftMm)},${if (reverse) 1 else 0}\n"
}


