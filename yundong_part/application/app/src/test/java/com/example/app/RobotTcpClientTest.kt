package com.example.app

import java.net.ServerSocket
import java.net.SocketTimeoutException
import java.util.concurrent.CountDownLatch
import java.util.concurrent.Executors
import java.util.concurrent.TimeUnit
import org.junit.Assert.*
import org.junit.Test

class RobotTcpClientTest {
    @Test fun diagnosticPingAndStatusReceiveRealSocketReplies() {
        ServerSocket(0).use { server ->
            val connected = CountDownLatch(1)
            val replies = CountDownLatch(3)
            val received = java.util.concurrent.CopyOnWriteArrayList<String>()
            val executor = Executors.newSingleThreadExecutor()
            val probe = LinkProbe()
            val result = executor.submit<List<String>> {
                server.accept().use { socket ->
                    socket.soTimeout = 3000
                    val reader = socket.getInputStream().bufferedReader()
                    val ping = reader.readLine(); val query = reader.readLine()
                    val id = ping.split(',')[1]
                    val qid = query.split(',')[1]
                    val output = socket.getOutputStream()
                    output.write("LINK,$id,ESP_OK\nPON".toByteArray()); output.flush()
                    output.write("G,$id,STM32\r\nSTATUS,$qid,IDLE,0,0\n".toByteArray()); output.flush()
                    replies.await(3,TimeUnit.SECONDS)
                    listOf(ping,query)
                }
            }
            val client = RobotTcpClient({ ok,_ -> if (ok) connected.countDown() }, {
                if (!it.startsWith("TX,")) { received.add(it); probe.receive(it); replies.countDown() }
            })
            try {
                client.connect("127.0.0.1",server.localPort)
                assertTrue(connected.await(3,TimeUnit.SECONDS))
                val id = client.checkLink(); probe.start(id); client.requestStatus()
                assertTrue(replies.await(4,TimeUnit.SECONDS))
                assertNull(probe.timeout(id))
                assertEquals("LINK,$id,ESP_OK",received[0])
                assertEquals("PONG,$id,STM32",received[1])
                assertTrue(received[2].startsWith("STATUS,"))
                val lines = result.get(3,TimeUnit.SECONDS)
                assertEquals("PING,$id",lines[0]); assertTrue(lines[1].startsWith("QUERY,"))
            } finally { client.close(); executor.shutdownNow() }
        }
    }
    @Test fun sendsGyroTurnsAndReadsImuFeedback() {
        ServerSocket(0).use { server ->
            val connected=CountDownLatch(1)
            val feedback=CountDownLatch(1)
            val executor=Executors.newSingleThreadExecutor()
            val result=executor.submit<List<String>> {
                server.accept().use { socket ->
                    socket.soTimeout=3000
                    val reader=socket.getInputStream().bufferedReader()
                    val lines=List(3) { reader.readLine() }
                    socket.getOutputStream().write("IMU,10,READY,90.0,0.0\n".toByteArray())
                    socket.getOutputStream().flush()
                    feedback.await(3,TimeUnit.SECONDS)
                    lines
                }
            }
            val client=RobotTcpClient({ ok,_ -> if(ok) connected.countDown() }, {
                if(it=="IMU,10,READY,90.0,0.0") feedback.countDown()
            })
            try {
                client.connect("127.0.0.1",server.localPort)
                assertTrue(connected.await(3,TimeUnit.SECONDS))
                val right=client.sendMove(DistanceMove.gyroRotationFromInputs('W',"90","20")!!)
                val left=client.sendMove(DistanceMove.gyroRotationFromInputs('C',"360","10")!!)
                client.requestImu()
                assertTrue(feedback.await(4,TimeUnit.SECONDS))
                val lines=result.get(4,TimeUnit.SECONDS)
                assertEquals("EXEC,$right,TURN,W,90.0,20",lines[0])
                assertEquals("EXEC,$left,TURN,C,360.0,10",lines[1])
                assertTrue(lines[2].startsWith("QUERY_IMU,"))
            } finally { client.close(); executor.shutdownNow() }
        }
    }
    @Test fun receivesFragmentedTaskFeedbackAndRecoversAfterOversize() {
        ServerSocket(0).use { server ->
            val received = java.util.concurrent.CopyOnWriteArrayList<String>()
            val complete = CountDownLatch(2)
            val executor = Executors.newSingleThreadExecutor()
            val sent = executor.submit {
                server.accept().use { socket ->
                    val output = socket.getOutputStream()
                    output.write("AUTO,11,RUN".toByteArray())
                    output.flush()
                    output.write("NING,0,0,0,0,0\r\n".toByteArray())
                    output.write(("x".repeat(300) + "\nTASK,11,156+123+516+231\n").toByteArray())
                    output.flush()
                    complete.await(3, TimeUnit.SECONDS)
                }
            }
            val client = RobotTcpClient({ _, _ -> }, { line -> received.add(line); complete.countDown() })
            try {
                client.connect("127.0.0.1", server.localPort)
                assertTrue(complete.await(4, TimeUnit.SECONDS))
                assertEquals(listOf("AUTO,11,RUNNING,0,0,0,0,0", "TASK,11,156+123+516+231"), received.toList())
                sent.get(4, TimeUnit.SECONDS)
            } finally { client.close(); executor.shutdownNow() }
        }
    }
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
                val card = SequenceCard("a", "抓取", 'A', SequenceSettings(1100, -50, 200, -40, 20))
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
                assertTrue(frames[2].startsWith("PLAN_ITEM_V2,$id,0,A,"))
                assertTrue(frames[3].startsWith("PLAN_ITEM_V2,$id,1,P,"))
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
                client.sendSequence('X', SequenceSettings(1100, -50, 200, -40, 20))
                client.sendSequence('A', SequenceSettings(1100, -50, 200, -40, 20))
                client.sendSequence('P', SequenceSettings(200, -40, 1300, -110, 25))
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
                assertTrue(frames[15].startsWith("STATE_V2,") && frames[15].endsWith(",A,1100,-50,200,-40,20,60,0,60,0,248,140"))
                assertTrue(frames[16].startsWith("STATE_V2,") && frames[16].endsWith(",P,200,-40,1300,-110,25,60,0,60,0,248,140"))
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
                    assertTrue(first.startsWith("EXEC,") && first.contains(",DRIVE,F,"))
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
                assertEquals("EXEC,$firstId,DRIVE,F,9889,45,3200", trace[0])
                assertTrue(trace[1].startsWith("CMD,") && trace[1].endsWith(",S"))
                assertEquals("EXEC,$secondId,DRIVE,F,9889,45,3200", trace[2])
                assertNotEquals(firstId, secondId)
            } finally {
                client.close()
                executor.shutdownNow()
            }
        }
    }
}
