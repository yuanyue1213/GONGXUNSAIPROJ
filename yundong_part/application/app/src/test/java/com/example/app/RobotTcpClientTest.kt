package com.example.app

import java.net.ServerSocket
import java.net.SocketTimeoutException
import java.util.concurrent.CountDownLatch
import java.util.concurrent.Executors
import java.util.concurrent.TimeUnit
import org.junit.Assert.*
import org.junit.Test

class RobotTcpClientTest {
    @Test fun sendsAllThreeServoChannelsWithoutReplies() {
        ServerSocket(0).use { server ->
            val connected = CountDownLatch(1)
            val executor = Executors.newSingleThreadExecutor()
            val result = executor.submit<List<String>> {
                server.accept().use { socket ->
                    socket.soTimeout = 3000
                    val reader = socket.getInputStream().bufferedReader()
                    List(13) { reader.readLine() }
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
