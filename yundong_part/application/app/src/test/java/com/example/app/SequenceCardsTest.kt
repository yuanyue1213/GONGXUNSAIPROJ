package com.example.app

import org.junit.Assert.*
import org.junit.Test

class SequenceCardsTest {
    private val settings = SequenceSettings(1105, -50, 200, -40, 20, 25, 60, 30, 10, 65, 5, 249, 141)
    private val card = SequenceCard("card-1", "抓取 | 物块\n一", 'A', settings)

    @Test fun savedSnapshotsRetainNamesAnglesAndFractionalDistances() {
        val release = card.copy(id = "card-2", name = "放下", mode = 'P')
        assertEquals(listOf(card, release, card), SequenceCardCodec.decode(SequenceCardCodec.encode(listOf(card, release, card))))
        assertEquals(listOf(card), SequenceCardCodec.decode("bad-record\n" + SequenceCardCodec.encode(listOf(card))))
        assertTrue(SequenceCardCodec.decode("x|y|A,0,0").isEmpty())
    }

    @Test fun planFramesPreserveOrderAndDuplicateCards() {
        val cards = listOf(card, card.copy(mode = 'P'), card)
        val frames = SequencePlan.frames(7, cards)
        assertEquals(5, frames.size)
        assertEquals("PLAN_BEGIN,7,3\n", frames.first())
        assertTrue(frames[1].startsWith("PLAN_ITEM,7,0,A,1105,-50,"))
        assertTrue(frames[2].startsWith("PLAN_ITEM,7,1,P,"))
        assertTrue(frames[3].startsWith("PLAN_ITEM,7,2,A,"))
        assertEquals("PLAN_RUN,7\n", frames.last())
        assertTrue(SequencePlan.frames(0, cards).isEmpty())
        assertTrue(SequencePlan.frames(1, emptyList()).isEmpty())
        assertTrue(SequencePlan.frames(1, List(17) { card }).isEmpty())
        assertTrue(SequencePlan.frames(1, listOf(card.copy(mode = 'X'))).isEmpty())
        val longest = card.copy(settings = SequenceSettings(-10000, -400, -10000, -400, 120, 120, 120, 300, 270, 270, 270, 360, 360))
        assertTrue(SequencePlan.frames(0xFFFFFFFFL, List(16) { longest }).all { it.trimEnd().length <= 127 })
    }
    @Test fun turnCardsPersistAndMixWithArmCards() {
        val turn = SequenceCard("turn-1", "转盘120", 'T', turn = TurnSettings(0,120,60))
        val cards = listOf(turn, card.copy(mode = 'P'), turn.copy(turn = TurnSettings(120,0,120)))
        assertEquals(cards, SequenceCardCodec.decode(SequenceCardCodec.encode(cards)))
        val frames = SequencePlan.frames(8, cards)
        assertEquals("PLAN_ITEM,8,0,T,0,120,60\n", frames[1])
        assertTrue(frames[2].startsWith("PLAN_ITEM,8,1,P,"))
        assertEquals("PLAN_ITEM,8,2,T,120,0,120\n", frames[3])
        assertFalse(turn.copy(turn = TurnSettings(0,271,60)).valid())
        assertFalse(turn.copy(turn = TurnSettings(0,120,0)).valid())
        assertTrue(SequenceCardCodec.decode("x|y|T,0,120,0").isEmpty())
    }
}
