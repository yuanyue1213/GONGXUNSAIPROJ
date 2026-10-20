package com.example.app

import org.junit.Assert.*
import org.junit.Test

class LinkProbeTest {
    @Test fun espAcknowledgementDoesNotClaimStmOrMotionSuccess() {
        val probe = LinkProbe(); probe.start(20)
        assertTrue(probe.receive("LINK,20,ESP_OK")!!.contains("等待STM32"))
        assertTrue(probe.timeout(20)!!.contains("未收到STM32"))
        assertNull(probe.receive("ACTION,20,DONE,0"))
        assertTrue(probe.receive("PONG,20,STM32")!!.contains("不代表动作完成"))
        assertNull(probe.timeout(20))
    }
    @Test fun ignoresOldIdsAndHandlesStmBeforeEsp() {
        val probe = LinkProbe(); probe.start(21)
        assertNull(probe.receive("PONG,20,STM32"))
        assertNull(probe.receive("LINK,22,ESP_OK"))
        assertTrue(probe.timeout(21)!!.contains("未收到ESP32"))
        assertNotNull(probe.receive("PONG,21,STM32"))
        assertNull(probe.receive("LINK,21,ESP_OK"))
        probe.start(22); assertNull(probe.timeout(21))
        probe.clear(); assertNull(probe.receive("PONG,22,STM32"))
    }
}
