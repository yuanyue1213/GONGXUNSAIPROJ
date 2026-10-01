from pathlib import Path
root = Path('F:/GONGXUNSAIPROJ')
p = root / 'yundong_part/tests/distance_control_test.c'
s = p.read_text()
s = s.replace('#include "../shared/robot_line_decoder.h"\n', '')
s = s.replace('static char messages[8192];', 'static unsigned lift_stops, arm_stops;')
a = s.index('    if (uart == &command_uart)')
b = s.index('    assert(uart == &motor_uart);', a)
s = s[:a] + '    assert(uart != &command_uart); /* Application UART must never send a reply. */\n' + s[b:]
s = s.replace('LiftMotor_Stop(void) { return HAL_OK; }', 'LiftMotor_Stop(void) { ++lift_stops; return HAL_OK; }')
s = s.replace('ArmMotor_StopForeAft(void) { return HAL_OK; }', 'ArmMotor_StopForeAft(void) { ++arm_stops; return HAL_OK; }')
s = s.replace("    messages[0] = '\\0';", '    lift_stops = arm_stops = 0;')
lines = []
for line in s.splitlines():
    if 'assert(' in line and 'messages' in line:
        if 'Three reached' in line:
            line = '    feed("MOVE,4,F,100,45,9889\\n");\n    assert(position_frames == 1); /* Three reached is insufficient. */'
        elif 'EVENT,LIFT' in line: line = '    assert(lift_stops == 1);'
        elif 'EVENT,ARM' in line: line = '    assert(arm_stops == 1);'
        elif 'position_frames' in line: line = '    assert(position_frames == %s);' % ('0' if 'position_frames == 0' in line else '1')
        elif 'stop_frames' in line: line = '    assert(stop_frames == %s);' % ('8' if 'stop_frames == 8' in line else '0' if 'stop_frames == 0' in line else '4')
    lines.append(line)
s = '\n'.join(lines) + '\n'
s = s.replace('test_receive_restart_and_reply_decoder', 'test_receive_restart')
a = s.index('    feed("\\nDIAG,12\\n");')
b = s.index('\n}\nint main', a)
s = s[:a] + '    feed("\\nMOVE,3,F,100,45,9889\\n");\n    assert(position_frames == 1);' + s[b:]
assert 'messages' not in s
p.write_text(s)
p = root / 'yundong_part/application/app/src/test/java/com/example/app/RobotTcpClientTest.kt'
p.write_text('''package com.example.app

import java.net.ServerSocket
import java.net.SocketTimeoutException
import java.util.concurrent.CountDownLatch
import java.util.concurrent.Executors
import java.util.concurrent.TimeUnit
import org.junit.Assert.*
import org.junit.Test

class RobotTcpClientTest {
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
''')
p = root / 'yundong_part/esp32s3/hello_world/main/robot_remote_main.c'
s = p.read_text().replace('双向转发', '单向转发').replace('让主循环定期处理串口回包和超时检查', '让主循环定期处理机械臂超时检查').replace('STM32 回包上限由共享解码器定义', '超长命令丢弃到下一换行').replace('驱动提供收发缓冲，本程序通过 uart_read_bytes 轮询读取回包', '驱动提供发送缓冲；应用不读取或转发 STM32 回包').replace('不保证 STM32 已收到或执行；执行结果需等待 STM32 回包', '不保证 STM32 已收到或执行；本协议不返回执行结果')
s = s.replace('    setsockopt(socket_fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));\n', '')
p.write_text(s)
decoder = root / 'yundong_part/shared/robot_line_decoder.h'
if decoder.exists(): decoder.unlink()
