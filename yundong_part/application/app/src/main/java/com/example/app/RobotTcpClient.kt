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

    fun sendArmDistance(direction: Char, distanceMm: Int, rpm: Int, pulsesPerRev: Int) {
        if (direction !in "EC" || distanceMm !in 1..1000 || rpm !in 5..60 ||
            pulsesPerRev !in 200..51200) return
        sendFrame("ARM_MOVE,${nextSequence()},$direction,$distanceMm,$rpm,$pulsesPerRev\n")
    }

    fun sendLiftDistance(direction: Char, distanceMm: Int, rpm: Int, pulsesPerRev: Int) {
        if (direction !in "UD" || distanceMm !in 1..400 || rpm !in 5..60 ||
            pulsesPerRev !in 200..51200) return
        sendFrame("LIFT_MOVE,${nextSequence()},$direction,$distanceMm,$rpm,$pulsesPerRev\n")
    }

    fun sendMotion(direction: Char) {
        if (direction !in "SUDHQAZOI") return
        sendFrame("CMD,${nextSequence()},$direction\n")
    }

    fun sendArmPose(theta: Int, rMm: Int, zMm: Int, rpm: Int, pulsesPerRev: Int) {
        if (theta !in 0..270 || rMm !in -1000..1000 || zMm !in -400..400 ||
            rpm !in 5..60 || pulsesPerRev !in 200..51200) return
        sendFrame("ARM_POSE,${nextSequence()},$theta,$rMm,$zMm,$rpm,$pulsesPerRev\n")
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
