/* App 页面仅提供一键启动和状态显示；连接与任务状态由 ViewModel 管理。 */
package com.example.myapplication

import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.compose.foundation.layout.*
import androidx.compose.material3.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.unit.dp
import androidx.lifecycle.ViewModelProvider
import com.example.myapplication.ui.theme.MyApplicationTheme

class MainActivity : ComponentActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        val model = ViewModelProvider(this)[StartViewModel::class.java]
        setContent {
            MyApplicationTheme {
                Surface(Modifier.fillMaxSize()) {
                    Column(Modifier.fillMaxSize().padding(24.dp),
                        verticalArrangement = Arrangement.spacedBy(24.dp, Alignment.CenterVertically),
                        horizontalAlignment = Alignment.CenterHorizontally) {
                        Text("运动控制 · 全流程", style = MaterialTheme.typography.headlineMedium)
                        Text("先连接 RobotCar-ESP32S3 热点，再启动任务。")
                        Button(onClick = model::start, enabled = !model.busy,
                            modifier = Modifier.fillMaxWidth().height(64.dp)) {
                            Text(if (model.busy) "全流程执行中" else "一键启动全流程")
                        }
                        Text(model.status)
                        Text("路线、视觉识别和机械臂取放均由 STM32 执行。",
                            style = MaterialTheme.typography.bodySmall)
                    }
                }
            }
        }
    }
}
