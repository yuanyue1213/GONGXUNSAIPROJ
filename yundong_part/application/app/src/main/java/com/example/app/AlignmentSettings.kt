package com.example.app

internal data class AlignmentSettings(
    val fineRpm: Int = 10, val coarseRpm: Int = 20, val offsetMm: Int = 10,
    val centerX: Double = 256.0, val centerY: Double = 160.0,
) {
    fun valid() = fineRpm in 5..60 && coarseRpm in 5..60 && offsetMm in -100..100 &&
        validPixel(centerX, 512.0) && validPixel(centerY, 320.0)
    fun frame(sequence: Long, ring: Int, forward: Int, lateral: Int) =
        "ALIGN_CFG,$sequence,$ring,$forward,$lateral,$fineRpm,$coarseRpm,$offsetMm${centerSuffix()}\n"
    internal fun centerSuffix() = if (centerX == 256.0 && centerY == 160.0) "" else
        ",${pixel(centerX)},${pixel(centerY)}"
    private fun pixel(value: Double) = java.math.BigDecimal.valueOf(value).stripTrailingZeros().toPlainString()
    companion object {
        private fun validPixel(value: Double, limit: Double) = value.isFinite() && value >= 0 && value < limit &&
            java.math.BigDecimal.valueOf(value).stripTrailingZeros().scale() <= 1
        fun parse(fineRpm: String, coarseRpm: String, offsetMm: String, centerX: String = "256", centerY: String = "160"): AlignmentSettings? = try {
            AlignmentSettings(fineRpm.trim().toInt(), coarseRpm.trim().toInt(), offsetMm.trim().toInt(),
                centerX.trim().toDouble(), centerY.trim().toDouble())
                .takeIf { it.valid() }
        } catch (_: IllegalArgumentException) { null }
    }
}
