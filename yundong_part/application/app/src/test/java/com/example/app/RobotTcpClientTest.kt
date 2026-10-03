package com.example.app

import java.net.ServerSocket
import java.net.SocketTimeoutException
import java.util.concurrent.CountDownLatch
import java.util.concurrent.Executors
import java.util.concurrent.TimeUnit
import org.junit.Assert.*
import org.junit.Test

class RobotTcpClientTest {
    @Test fun uploadsOriginAndCompletePlanWithoutReplies() {
        ServerSocket(0).use { server ->
            val connected = CountDownLatch(1)
            val executor = Executors.newSingleThreadExecutor()
            val result = executor.submit<List<String>> {
                server.accept().use { socket ->
                    socket.soTimeout = 3000
                    val reader = socket.getInputStream().bufferedReader()
                    List(7) { reader.readLine() }
                }
            }
            val client = RobotTcpClient({ ok, _ -> if (ok) connected.countDown() }, {})
            try {
                client.connect("127.0.0.1", server.localPort)
                assertTrue(connected.await(3, TimeUnit.SECONDS))
                client.initializeOrigin(361) // invalid requests produce no frame
                client.initializeOrigin(251)
                val card = SequenceCard("a", "抓取", 'A', SequenceSettings(1100, -50, 200, -40, 20, 20, 50))
                client.sendPlan(listOf(card, card.copy(mode = 'P')))
                client.sendBase(-1, 60)
                client.sendBase(361, 60)
                client.sendBase(90, 0)
                client.sendBase(360, 30)
                client.sendBase(0, 1)
                val frames = result.get(5, TimeUnit.SECONDS)
                assertTrue(frames[0].startsWith("ORIGIN,") && frames[0].endsWith(",251"))
                val id = frames[1].split(',')[1]
                assertEquals("PLAN_BEGIN,$id,2", frames[1])
                assertTrue(frames[2].startsWith("PLAN_ITEM,$id,0,A,"))
                assertTrue(frames[3].startsWith("PLAN_ITEM,$id,1,P,"))
                assertEquals("PLAN_RUN,$id", frames[4])
                assertTrue(frames[5].startsWith("BASE,") && frames[5].endsWith(",360,30"))
                assertTrue(frames[6].startsWith("BASE,") && frames[6].endsWith(",0,1"))
            } finally { client.close(); executor.shutdownNow() }
        }
    }

    @Test fun sendsAllThreeServoChannelsWithoutReplies() {
        ServerSocket(0).use { server ->
            val connected = CountDownLatch(1)
            val executor = Executors.newSingleThreadExecutor()
            val result = executor.submit<List<String>> {
                server.accept().use { socket ->
                    socket.soTimeout = 3000
                    val reader = socket.getInputStream().bufferedReader()
                    List(22) { reader.readLine() }
                }
            }
            val client = RobotTcpClient(
                onConnectionChanged = { ok, _ -> if (ok) connected.countDown() },
                onMessage = {},
            )
            try {
                client.connect("127.0.0.1", server.localPort)
                assertTrue(connected.await(3, TimeUnit.SECONDS))
                client.sendServo('G', 90)
                client.sendServo('T', 270)
                client.sendServo('B', 360)
                client.sendMotion('A')
                client.sendMotion('Z')
                client.sendArmDistance('E', 127, 10, 3200)
                client.sendArmDistance('C', 10, 5, 6400)
                client.sendAlignment(9889, 12000)
                client.sendLiftDistance('E', 10, 5, 3200) // Invalid direction sends nothing.
                client.sendLiftDistance('U', 0, 5, 3200)
                client.sendLiftDistance('U', 401, 5, 3200)
                client.sendLiftDistance('U', 10, 5, 3200)
                client.sendLiftDistance('D', 40, 10, 6400)
                client.sendArmPose(271, 0, 0, 5, 3200) // Invalid targets do not send.
                client.sendArmPose(0, -1001, 0, 5, 3200)
                client.sendArmPose(0, 0, 401, 5, 3200)
                client.sendArmPose(120, -20, 40, 5, 3200)
                client.sendMotion('O')
                client.sendMotion('I')
                client.sendMotion('P')
                client.sendRingAlignment(0, 9889, 12000)
                client.sendRingAlignment(2, 9889, 12000)
                client.sendSequence('X', SequenceSettings(1100, -50, 200, -40, 20, 20, 50))
                client.sendSequence('A', SequenceSettings(1100, -50, 200, -40, 20, 20, 50))
                client.sendSequence('P', SequenceSettings(200, -40, 1300, -110, 25, 15, 55))
                client.sendConfiguredAlignment(0, 9889, 12000, AlignmentSettings(12, 30, -7))
                client.sendConfiguredAlignment(2, 9889, 12000, AlignmentSettings(10, 20, 0))
                client.sendGripper(0, 271, 30)
                client.sendGripper(0, 60, 30)
                client.stopGripper()
                client.sendParallel(9889, 10, 3, true)
                val frames = result.get(3, TimeUnit.SECONDS)
                assertEquals(listOf("G,90", "T,270", "B,360"),
                    frames.take(3).map { it.split(',').drop(2).joinToString(",") })
                assertTrue(frames.take(3).all { it.startsWith("SERVO,") })
                assertTrue(frames[3].startsWith("CMD,") && frames[3].endsWith(",A"))
                assertTrue(frames[4].startsWith("CMD,") && frames[4].endsWith(",Z"))
                assertTrue(frames[5].startsWith("ARM_MOVE,") && frames[5].endsWith(",E,127,10,3200"))
                assertTrue(frames[6].startsWith("ARM_MOVE,") && frames[6].endsWith(",C,10,5,6400"))
                assertTrue(frames[7].startsWith("ALIGN,") && frames[7].endsWith(",9889,12000"))
                assertTrue(frames[8].startsWith("LIFT_MOVE,") && frames[8].endsWith(",U,10,5,3200"))
                assertTrue(frames[9].startsWith("LIFT_MOVE,") && frames[9].endsWith(",D,40,10,6400"))
                assertTrue(frames[10].startsWith("ARM_POSE,") && frames[10].endsWith(",120,-20,40,5,3200"))
                assertTrue(frames[11].startsWith("CMD,") && frames[11].endsWith(",O"))
                assertTrue(frames[12].startsWith("CMD,") && frames[12].endsWith(",I"))
                assertTrue(frames[13].startsWith("CMD,") && frames[13].endsWith(",P"))
                assertTrue(frames[14].startsWith("ALIGN_RING,") && frames[14].endsWith(",2,9889,12000"))
                assertTrue(frames[15].startsWith("STATE,") && frames[15].endsWith(",A,1100,-50,200,-40,20,20,50,60,0,60,0,248,140"))
                assertTrue(frames[16].startsWith("STATE,") && frames[16].endsWith(",P,200,-40,1300,-110,25,15,55,60,0,60,0,248,140"))
                assertTrue(frames[17].startsWith("ALIGN_CFG,") && frames[17].endsWith(",0,9889,12000,12,30,-7"))
                assertTrue(frames[18].startsWith("ALIGN_CFG,") && frames[18].endsWith(",2,9889,12000,10,20,0"))
                assertTrue(frames[19].startsWith("GRIP,") && frames[19].endsWith(",0,60,30"))
                assertTrue(frames[20].startsWith("GRIP_STOP,"))
                assertTrue(frames[21].startsWith("PARALLEL,") && frames[21].endsWith(",9889,10,3,1"))
            } finally {
                client.close()
                executor.shutdownNow()
            }
        }
    }
    @Test fun noRepliesDoNotTriggerAutomaticStopAndNextMoveCanBeSent() {
        ServerSocket(0).use { server ->
            val connected = CountDownLatch(1)
            val silenceChecked = CountDownLatch(1)
            val executor = Executors.newSingleThreadExecutor()
            val result = executor.submit<List<String>> {
                server.accept().use { socket ->
                    socket.soTimeout = 3000
                    val reader = socket.getInputStream().bufferedReader()
                    val first = reader.readLine()
                    assertTrue(first.startsWith("MOVE,"))
                    // Never send ACK/EXEC: missing replies must not produce an automatic S.
                    socket.soTimeout = 2500
                    try {
                        fail("Unexpected automatic command: ${reader.readLine()}")
                    } catch (_: SocketTimeoutException) { }
                    silenceChecked.countDown()
                    socket.soTimeout = 3000
                    listOf(first, reader.readLine(), reader.readLine())
                }
            }
            val client = RobotTcpClient(
                onConnectionChanged = { ok, _ -> if (ok) connected.countDown() },
                onMessage = { assertTrue(it.startsWith("TX,")) },
            )
            try {
                client.connect("127.0.0.1", server.localPort)
                assertTrue(connected.await(3, TimeUnit.SECONDS))
                val move = DistanceMove('F', 1000, 45, 9889)
                val firstId = client.sendMove(move)
                assertTrue(silenceChecked.await(4, TimeUnit.SECONDS))
                client.sendMotion('S')
                val secondId = client.sendMove(move)
                val trace = result.get(3, TimeUnit.SECONDS)
                assertEquals("MOVE,$firstId,F,1000,45,9889", trace[0])
                assertTrue(trace[1].startsWith("CMD,") && trace[1].endsWith(",S"))
                assertEquals("MOVE,$secondId,F,1000,45,9889", trace[2])
                assertNotEquals(firstId, secondId)
            } finally {
                client.close()
                executor.shutdownNow()
            }
        }
    }
}
