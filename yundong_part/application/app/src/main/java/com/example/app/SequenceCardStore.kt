package com.example.app

import android.content.Context

internal class SequenceCardStore(context: Context) {
    private val preferences = context.applicationContext.getSharedPreferences("sequence_cards", Context.MODE_PRIVATE)
    fun cards() = SequenceCardCodec.decode(preferences.getString("cards", "") ?: "")
    fun plan() = SequenceCardCodec.decode(preferences.getString("plan", "") ?: "").take(SequencePlan.MAX_ITEMS)
    fun saveCards(cards: List<SequenceCard>) { preferences.edit().putString("cards", SequenceCardCodec.encode(cards)).apply() }
    fun savePlan(plan: List<SequenceCard>) { preferences.edit().putString("plan", SequenceCardCodec.encode(plan)).apply() }
}
