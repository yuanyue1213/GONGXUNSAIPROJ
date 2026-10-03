package com.example.app

import java.net.URLDecoder
import java.net.URLEncoder

/** A saved card is an immutable snapshot; later form edits never change it. */
internal data class SequenceCard(
    val id: String,
    val name: String,
    val mode: Char,
    val settings: SequenceSettings,
) {
    fun valid() = id.isNotBlank() && name.isNotBlank() && name.length <= 40 &&
        mode in "AP" && settings.valid()
}

internal object SequenceCardCodec {
    fun encode(cards: List<SequenceCard>): String = cards.filter { it.valid() }.joinToString("\n") {
        "${URLEncoder.encode(it.id, "UTF-8")}|${URLEncoder.encode(it.name, "UTF-8")}|" +
            it.settings.frame(1, it.mode).trim().removePrefix("STATE,1,")
    }

    fun decode(value: String): List<SequenceCard> = value.lineSequence().mapNotNull { line ->
        try {
            val parts = line.split('|')
            if (parts.size != 3) return@mapNotNull null
            val fields = parts[2].split(',')
            if (fields.size != 14 || fields[0].length != 1) return@mapNotNull null
            val v = fields.drop(1).map(String::toInt)
            SequenceCard(URLDecoder.decode(parts[0], "UTF-8"), URLDecoder.decode(parts[1], "UTF-8"),
                fields[0][0], SequenceSettings(v[0], v[1], v[2], v[3], v[4], v[5], v[6],
                    v[7], v[8], v[9], v[10], v[11], v[12])).takeIf { it.valid() }
        } catch (_: IllegalArgumentException) { null }
    }.toList()
}

internal object SequencePlan {
    const val MAX_ITEMS = 16
    fun frames(sequence: Long, cards: List<SequenceCard>): List<String> {
        if (sequence !in 1..0xFFFFFFFFL || cards.isEmpty() || cards.size > MAX_ITEMS || cards.any { !it.valid() }) return emptyList()
        return listOf("PLAN_BEGIN,$sequence,${cards.size}\n") + cards.mapIndexed { index, card ->
            card.settings.frame(sequence, card.mode).replaceFirst("STATE,$sequence,", "PLAN_ITEM,$sequence,$index,")
        } + "PLAN_RUN,$sequence\n"
    }
}
