package com.example.app

import java.net.URLDecoder
import java.net.URLEncoder

internal data class TurnSettings(val start: Int, val end: Int, val dps: Int) {
    fun valid() = start in 0..270 && end in 0..270 && dps in 1..360
}

/** A saved card is an immutable snapshot; later form edits never change it. */
internal data class SequenceCard(
    val id: String,
    val name: String,
    val mode: Char,
    val settings: SequenceSettings = SequenceSettings(0,0,0,0,20,20,50),
    val turn: TurnSettings? = null,
) {
    fun valid() = id.isNotBlank() && name.isNotBlank() && name.length <= 40 &&
        (if (mode == 'T') turn?.valid() == true else mode in "AP" && settings.valid())
    fun frame(sequence: Long) = if (mode == 'T') "STATE,$sequence,T,${turn!!.start},${turn.end},${turn.dps}\n" else settings.frame(sequence, mode)
}

internal object SequenceCardCodec {
    fun encode(cards: List<SequenceCard>): String = cards.filter { it.valid() }.joinToString("\n") {
        "${URLEncoder.encode(it.id, "UTF-8")}|${URLEncoder.encode(it.name, "UTF-8")}|" +
            it.frame(1).trim().removePrefix("STATE,1,")
    }

    fun decode(value: String): List<SequenceCard> = value.lineSequence().mapNotNull { line ->
        try {
            val parts = line.split('|')
            if (parts.size != 3) return@mapNotNull null
            val fields = parts[2].split(',')
            if (fields.size == 4 && fields[0] == "T") {
                return@mapNotNull SequenceCard(URLDecoder.decode(parts[0], "UTF-8"),
                    URLDecoder.decode(parts[1], "UTF-8"), 'T',
                    turn = TurnSettings(fields[1].toInt(), fields[2].toInt(), fields[3].toInt())).takeIf { it.valid() }
            }
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
            card.frame(sequence).replaceFirst("STATE,$sequence,", "PLAN_ITEM,$sequence,$index,")
        } + "PLAN_RUN,$sequence\n"
    }
}
