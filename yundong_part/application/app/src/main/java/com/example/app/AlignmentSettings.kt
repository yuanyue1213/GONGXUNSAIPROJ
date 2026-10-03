package com.example.app

internal data class AlignmentSettings(
    val fineRpm: Int = 10, val coarseRpm: Int = 20, val offsetMm: Int = 10,
) {
    fun valid() = fineRpm in 5..60 && coarseRpm in 5..60 && offsetMm in -100..100
    fun frame(sequence: Long, ring: Int, forward: Int, lateral: Int) =
        "ALIGN_CFG,$sequence,$ring,$forward,$lateral,$fineRpm,$coarseRpm,$offsetMm\n"
    companion object {
        fun parse(fineRpm: String, coarseRpm: String, offsetMm: String): AlignmentSettings? = try {
            AlignmentSettings(fineRpm.trim().toInt(), coarseRpm.trim().toInt(), offsetMm.trim().toInt())
                .takeIf { it.valid() }
        } catch (_: IllegalArgumentException) { null }
    }
}
