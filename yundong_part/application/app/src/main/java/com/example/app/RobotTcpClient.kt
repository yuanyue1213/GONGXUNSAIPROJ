package com.example.app

import java.io.BufferedWriter
import java.io.OutputStreamWriter
import java.net.InetSocketAddress
import java.net.Socket
import java.util.concurrent.Executors
import java.util.concurrent.atomic.AtomicLong

/** One-way commands; no reply, heartbeat, or execution-confirmation timer. */
internal class RobotTcpClient(
    private val onConnectionChanged: (Boolean, String) -> Unit,
    private val onMessage: (String) -> Unit,
) : AutoCloseable {
    private val connectionExecutor = Executors.newSingleThreadExecutor()
    private val sendExecutor = Executors.newSingleThreadExecutor()
    private val sequence = AtomicLong(System.currentTimeMillis() and 0x7FFFFFFF)
    private val connectionGeneration = AtomicLong(0)
    private val lock = Any()
    private var socket: Socket? = null
    private var writer: BufferedWriter? = null
    fun connect(host: String, port: Int) {
        disconnect()
        val generation = connectionGeneration.get()
        onConnectionChanged(false, "正在连接 $host:$port…")
        connectionExecutor.execute {
            val newSocket = Socket()
            try {
                // Small command/STOP packets must not wait for Nagle/delayed ACK batching.
                newSocket.tcpNoDelay = true
                newSocket.connect(InetSocketAddress(host, port), 3_000)
                val newWriter = BufferedWriter(OutputStreamWriter(newSocket.getOutputStream(), Charsets.US_ASCII))
                val input = newSocket.getInputStream()
                synchronized(lock) {
                    if (generation != connectionGeneration.get()) return@execute
                    socket = newSocket
                    writer = newWriter
                }
                onConnectionChanged(true, "已连接 $host:$port")
                while (true) {
                    // Observe EOF/disconnection only; legacy replies are discarded.
                    if (input.read() == -1) break
                }
            } catch (error: Exception) {
                if (generation == connectionGeneration.get())
                    onConnectionChanged(false, "连接错误：${error.message ?: error.javaClass.simpleName}")
            } finally {
                synchronized(lock) {
                    if (socket === newSocket) {
                        socket = null
                        writer = null
                    }
                }
                newSocket.close()
                if (generation == connectionGeneration.get()) onConnectionChanged(false, "已断开")
            }
        }
    }

    fun sendMove(move: DistanceMove): Long {
        val id = nextSequence()
        sendFrame(move.frame(id))
        return id
    }

    fun sendAlignment(forwardPpm: Int, lateralPpm: Int) {
        if (forwardPpm !in 1..1000000 || lateralPpm !in 1..1000000) return
        sendFrame("ALIGN,${nextSequence()},$forwardPpm,$lateralPpm\n")
    }

    fun sendConfiguredAlignment(ring: Int, forwardPpm: Int, lateralPpm: Int, settings: AlignmentSettings) {
        if (ring !in 0..3 || forwardPpm !in 1..1000000 || lateralPpm !in 1..1000000 || !settings.valid()) return
        sendFrame(settings.frame(nextSequence(), ring, forwardPpm, lateralPpm))
    }

    fun sendJointAlignment(ring: Int, forward: Int, lateral: Int, settings: AlignmentSettings, geometry: JointAlignmentSettings) {
        if (ring !in 1..3 || forward !in 1..1000000 || lateral !in 1..1000000 || !settings.valid() || !geometry.valid()) return
        sendFrame(geometry.frame(nextSequence(), ring, forward, lateral, settings))
    }

    fun sendRingAlignment(ring: Int, forwardPpm: Int, lateralPpm: Int) {
        if (ring !in 1..3 || forwardPpm !in 1..1000000 || lateralPpm !in 1..1000000) return
        sendFrame("ALIGN_RING,${nextSequence()},$ring,$forwardPpm,$lateralPpm\n")
    }

    fun sendArmDistance(direction: Char, distanceMm: Int, rpm: Int, pulsesPerRev: Int) {
        if (direction !in "EC" || distanceMm !in 1..1000 || rpm !in 5..120 ||
            pulsesPerRev !in 200..51200) return
        sendFrame("ARM_MOVE,${nextSequence()},$direction,$distanceMm,$rpm,$pulsesPerRev\n")
    }

    fun sendLiftDistance(direction: Char, distanceMm: Int, rpm: Int, pulsesPerRev: Int) {
        if (direction !in "UD" || distanceMm !in 1..400 || rpm !in 5..120 ||
            pulsesPerRev !in 200..51200) return
        sendFrame("LIFT_MOVE,${nextSequence()},$direction,$distanceMm,$rpm,$pulsesPerRev\n")
    }

    fun sendSequence(mode: Char, settings: SequenceSettings) {
        if (mode !in "AP" || !settings.valid()) return
        sendFrame(settings.frame(nextSequence(), mode))
    }

    fun initializeOrigin(baseAngle: Int) {
        if (baseAngle !in 0..360) return
        sendFrame("ORIGIN,${nextSequence()},$baseAngle\n")
    }

    fun sendPlan(cards: List<SequenceCard>) {
        val frames = SequencePlan.frames(nextSequence(), cards.toList())
        if (frames.isEmpty()) return
        val generation = connectionGeneration.get()
        // One executor task keeps the upload contiguous, even if other controls are tapped.
        sendExecutor.execute { frames.forEach { writeFrame(it, generation) } }
    }

    fun sendMotion(direction: Char) {
        if (direction !in "SUDHQAZOIP") return
        sendFrame("CMD,${nextSequence()},$direction\n")
    }

    fun sendArmPose(theta: Int, rMm: Int, zMm: Int, rpm: Int, pulsesPerRev: Int) {
        if (theta !in 0..270 || rMm !in -1000..1000 || zMm !in -400..400 ||
            rpm !in 5..120 || pulsesPerRev !in 200..51200) return
        sendFrame("ARM_POSE,${nextSequence()},$theta,$rMm,$zMm,$rpm,$pulsesPerRev\n")
    }

    fun sendParallel(ppm: Int, rpm: Int, stepMm: Int, reverse: Boolean) {
        if (ppm !in 1..1000000 || rpm !in 5..60 || stepMm !in 1..10) return
        sendFrame("PARALLEL,${nextSequence()},$ppm,$rpm,$stepMm,${if (reverse) 1 else 0}\n")
    }

    fun sendGripper(start: Int, end: Int, dps: Int) {
        if (start !in 0..270 || end !in 0..270 || dps !in 6..300) return
        sendFrame("GRIP,${nextSequence()},$start,$end,$dps\n")
    }
    fun stopGripper() { sendFrame("GRIP_STOP,${nextSequence()}\n") }

    fun sendBase(target: Int, dps: Int) {
        if (target !in 0..360 || dps !in 1..360) return
        sendFrame("BASE,${nextSequence()},$target,$dps\n")
    }

    fun sendServo(channel: Char, angle: Int) {
        val maximum = if (channel == 'B') 360 else 270
        if (channel !in "GTB" || angle !in 0..maximum) return
        sendFrame("SERVO,${nextSequence()},$channel,$angle\n")
    }

    private fun nextSequence(): Long = sequence.updateAndGet { if (it >= 0xFFFFFFFFL) 1L else it + 1L }

    private fun sendFrame(command: String) {
        val generation = connectionGeneration.get()
        sendExecutor.execute { writeFrame(command, generation) }
    }

    private fun writeFrame(command: String, generation: Long) {
        val activeWriter = synchronized(lock) {
            if (generation != connectionGeneration.get()) return
            writer
        } ?: return
        try {
            synchronized(activeWriter) {
                activeWriter.write(command)
                activeWriter.flush()
            }
            onMessage("TX,${command.trimEnd()}")
        } catch (_: Exception) {
            disconnect()
        }
    }

    fun disconnect() {
        val activeSocket = synchronized(lock) {
            connectionGeneration.incrementAndGet()
            writer = null
            socket.also { socket = null }
        }
        activeSocket?.close()
    }

    override fun close() {
        disconnect()
        connectionExecutor.shutdownNow()
        sendExecutor.shutdownNow()
    }
}
