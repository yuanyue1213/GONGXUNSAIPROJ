package com.example.myapplication
import org.junit.Assert.*
import org.junit.Test

class StartProtocolTest {
    @Test fun onlyStartFrameIsProduced() {
        assertEquals("START,4294967295\n", StartProtocol.frame(0xFFFFFFFFL))
        assertThrows(IllegalArgumentException::class.java) { StartProtocol.frame(0) }
        assertThrows(IllegalArgumentException::class.java) { StartProtocol.frame(0x100000000L) }
    }
    @Test fun rejectsStaleStatusAndFollowsExistingRunOnReconnect() {
        assertNull(StartProtocol.status("AUTO,1,DONE,25,0,24,24,S2", 2))
        val existing = StartProtocol.status("AUTO,1,BUSY,0,0,0,0,RUNNING", 2)!!
        assertEquals(1L, existing.runId)
        assertFalse(existing.finished)
        assertTrue(StartProtocol.status("AUTO,1,DONE,25,0,24,24,S2", existing.runId!!)!!.finished)
        assertTrue(StartProtocol.status("AUTO,2,STOPPED,1,0,0,0,IMU_TIMEOUT", 2)!!.finished)
        assertNull(StartProtocol.status("EXEC,1,MOVE,F,50", 2))
    }
}
