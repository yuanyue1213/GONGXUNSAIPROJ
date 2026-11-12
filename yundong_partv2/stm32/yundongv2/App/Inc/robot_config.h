/* STM32 可调参数统一入口。保持原值；动作顺序在两个 flow.h 中编写。 */
#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H
#include <stdint.h>

/* 仅 FixedQr 固件使用；仍到二维码区，再加载此任务码。 */
#define MISSION_FIXED_QR_TEXT "123+321+132+312"

/* ILI9341任务码屏；PE1实际连接BLK，不是复位。 */
#define LCD_MOSI_PORT GPIOB
#define LCD_MOSI_PIN GPIO_PIN_2
#define LCD_SCK_PORT GPIOC
#define LCD_SCK_PIN GPIO_PIN_10
#define LCD_CS_PORT GPIOE
#define LCD_CS_PIN GPIO_PIN_2
#define LCD_DC_PORT GPIOD
#define LCD_DC_PIN GPIO_PIN_11
#define LCD_BLK_PORT GPIOE
#define LCD_BLK_PIN GPIO_PIN_1
#define LCD_SPI_DELAY_LOOPS 12U
#define LCD_MADCTL 0x28U /* MV+BGR，横屏320x240；反向横屏用0xE8。 */

/* Software ceiling in motor-shaft RPM, shared by protocol parsers/controllers
 * and Emm/X drivers. This does not set the operating speed or servo degrees/s. */
#ifndef STEPPER_MAX_SPEED_RPM
#define STEPPER_MAX_SPEED_RPM 2000U
#endif
#if STEPPER_MAX_SPEED_RPM < 5U || STEPPER_MAX_SPEED_RPM > 3000U
#error "Stepper software limit must be 5..3000 RPM (documented Emm/X command range)"
#endif

/* Software trajectory-speed limits in degrees/second, not hardware ratings. */
#define SERVO_GRIPPER_MAX_DPS 300U
#define SERVO_BASE_MAX_DPS 360U
#define SERVO_TURNTABLE_MAX_DPS 360U

/* 步进速度直接填写电机轴RPM；舵机独立使用角度/秒。 */
#define MOTION_ROUTE_RPM          300U
#define MOTION_FINE_RPM           200U
#define MOTION_TURN_RPM           150U
#define MOTION_ARM_RPM            2000U
#define MOTION_STOW_RPM           1500U
#define MOTION_GRIPPER_DPS   SERVO_GRIPPER_MAX_DPS
#define MOTION_BASE_ALIGN_DPS     SERVO_BASE_MAX_DPS
#define MOTION_BASE_TRANSFER_DPS  SERVO_BASE_MAX_DPS
#define MOTION_TURNTABLE_DPS      SERVO_TURNTABLE_MAX_DPS

/* 配置速度不能超过驱动的软件上限。 */
_Static_assert(MOTION_ROUTE_RPM<=STEPPER_MAX_SPEED_RPM&&MOTION_FINE_RPM<=STEPPER_MAX_SPEED_RPM,"Chassis speed exceeds software limit");
_Static_assert(MOTION_TURN_RPM<=STEPPER_MAX_SPEED_RPM,"Turn speed exceeds software limit");
_Static_assert(MOTION_ARM_RPM<=STEPPER_MAX_SPEED_RPM&&MOTION_STOW_RPM<=STEPPER_MAX_SPEED_RPM,"Arm speed exceeds software limit");
_Static_assert(MOTION_GRIPPER_DPS<=SERVO_GRIPPER_MAX_DPS&&
               MOTION_BASE_ALIGN_DPS<=SERVO_BASE_MAX_DPS&&MOTION_BASE_TRANSFER_DPS<=SERVO_BASE_MAX_DPS&&
               MOTION_TURNTABLE_DPS<=SERVO_TURNTABLE_MAX_DPS,"Servo speed exceeds protocol limit");

/* 距离单位为毫米，PPM 为每米脉冲；下方机械臂姿态 r=0.1 mm、z=mm。 */
#define MISSION_FORWARD_PPM 13064U /* 实测标定：原12585 × 300/289，供全流程与测试共用。 */
#define MISSION_LATERAL_PPM 13228U
#define MISSION_ROUTE_RPM MOTION_ROUTE_RPM
#define MISSION_FINE_RPM MOTION_FINE_RPM
#define MISSION_TURN_RPM MOTION_TURN_RPM
#define MISSION_WHEEL_PPR 3200U
#define MISSION_TARGET_X 256.0f
#define MISSION_TARGET_Y 160.0f
#define MISSION_IMAGE_RIGHT_DIRECTION 'B'
#define MISSION_IMAGE_DOWN_DIRECTION 'L'
/* 粗调和精调共用本车当前相机标定。 */
#define MISSION_ALIGN_MM_PER_PIXEL 1.090819f
#define MISSION_ALIGN_COARSE_RPM MOTION_FINE_RPM
#define MISSION_ALIGN_FINE_RPM MOTION_FINE_RPM
#define MISSION_ALIGN_FINE_TOLERANCE_PX 1.0f
#define MISSION_ALIGN_FINE_MAX_MOVES 3U
#define MISSION_UNALIGN_RPM MOTION_FINE_RPM
/* 仅用于原料区识别成功后的粗调，方向由图像位置决定，单位mm。 */
#define RAW_COARSE_FORWARD_MM 100.0f
#define RAW_COARSE_LATERAL_MM 5.0f
/* 相对搜索起点的目标坐标幅值：前后 ±100 mm、左右 ±50 mm。 */
#define MISSION_ALIGN_SEARCH_FORWARD_MM 100.0f
#define MISSION_ALIGN_SEARCH_LATERAL_MM 50.0f
/* NONE 也可能是相机正在积累稳定帧，先留出识别窗口再触发移动。 */
#define MISSION_ALIGN_NONE_SETTLE_MS 1500U
/* Emm加速度为0..255档位；底盘和机械臂估时均换算RPM/s，0为无加减速。 */
#define MISSION_WHEEL_ACCEL 20U /* 约84.75 RPM/s；正常路线移动使用。 */
_Static_assert(MISSION_WHEEL_ACCEL<=255U,"Emm acceleration level must be 0..255");
#define MISSION_ALIGN_ACCEL 30U /* 约88.50 RPM/s；校准、搜索及回原位使用。 */
_Static_assert(MISSION_ALIGN_ACCEL<=255U,"Alignment Emm acceleration level must be 0..255");
#define MISSION_ARM_EMM_ACCEL 150U /* 约188.68 RPM/s；伸缩、升降及收臂的估时与下发共用。 */
_Static_assert(MISSION_ARM_EMM_ACCEL<=255U,"Arm Emm acceleration level must be 0..255");
#define MISSION_MOTION_SETTLE_MS 50U
/* 各区按左、中、右填写 {抓取伸缩,放置伸缩,高度,底座角度}。
 * 先沿用已使用的姿态，可把对应区域宏替换成独立的三组参数。 */
#define ARM_STATION_POSES { {1540,1520,-128,209}, {1190,1200,-126,170}, {1850,1850,-120,134} }
#define ARM_RAW_POSES { {1540,1520,-128,209}, {1100,1100,-40,180}, {1850,1850,-120,134} }
#define ARM_PROCESS_POSES ARM_STATION_POSES
#define ARM_STORE_POSES ARM_STATION_POSES
#define ARM_STACK_POSES ARM_STATION_POSES
#define ARM_STACK_HEIGHT 60 /* 初赛图5物料高60 mm，第二层相对普通放置位抬高一件高度。 */
#define ARM_READY_R 110
#define ARM_SLOT_ANGLES {0,120,240}
#define ARM_ALIGN_BASE 180U


/* 夹爪、车载槽位及行走姿态。伸缩 0.1 mm，升降 mm，角度为度。 */
#define ARM_INITIAL_GRIP 15U
#define ARM_INITIAL_BASE 90U
#define ARM_INITIAL_TURNTABLE 0U
#define TURNTABLE_TEST_ANGLE 0U /* 仅TurntableTest独立上电固件使用。 */
#define BASE_TEST_ANGLE 0U /* 仅BaseTest独立上电固件使用。 */
#define ARM_GRIP_OPEN 30U
#define ARM_GRIP_CLOSED 0U
#define ARM_SLOT_R 150 /* 15 mm；原料放入和各区从车上取料共用。 */
#define ARM_SLOT_Z (-10)
#define ARM_SLOT_BASE 60U
#define ARM_RAW_RETURN_BASE 180U
#define ARM_STOW_BASE 180U
#define ARM_GRIP_SETTLE_MS 100U
#define ARM_RELEASE_SETTLE_MS 50U
#define ARM_GRAB_TEST_STARTUP_MS 1500U /* 独立抓取固件：初始舵机姿态稳定窗口。 */
#define ARM_GRAB_TEST_INTERVAL_MS 1000U /* 每轮抓取并存入0度槽位后的等待。 */
#define ARM_PLACE_TEST_STARTUP_MS 1500U /* 独立放置固件：初始舵机姿态稳定窗口。 */
#define ARM_PLACE_TEST_INTERVAL_MS 1000U /* 每轮从0度槽位取料并放置后的等待。 */
#define MISSION_STEP_SETTLE_MS 20U

/* 传动及转向控制。 */
#define ARM_RADIAL_MM_PER_REV 127.0
#define ARM_LIFT_MM_PER_REV 40.0
#define ARM_PULSES_PER_REV 3200U
#define ARM_X_ACCEL_RPM_S 400U
#define CHASSIS_FORWARD_POLARITY {1,0,0,1}
#define TURN_KP 0.8f
#define TURN_KD 0.18f
#define TURN_ANGLE_TOLERANCE 1.0f
#define TURN_RATE_TOLERANCE 3.0f
#define TURN_STABLE_MS 200U
#define TURN_LOOP_MS 20U
#define TURN_WHEEL_ACCEL 240U
/* 视觉平行：由图像斜率计算角度，IMU闭环修正；正斜率默认右转。 */
#define PARALLEL_MIN_SPAN_PX 80.0f
#define PARALLEL_MAX_RESIDUAL_PX 5.0f
#define PARALLEL_Y_TOLERANCE_PX 1.0f
#define PARALLEL_FINE_SLOPE 0.05f
#define PARALLEL_ANGLE_GAIN 1.0f /* 图像倾角到车体转角的安装标定比例。 */
#define PARALLEL_MAX_ANGLE_DEG 15.0f
#define PARALLEL_TURN_TOLERANCE_DEG 0.05f
#define PARALLEL_RPM 10U
#define PARALLEL_FINE_RPM 5U
#define PARALLEL_SETTLE_MS 300U
#define PARALLEL_MAX_MOVES 3U /* 最多修正3次，结束后直接执行动作，不追加复核。 */
#define PARALLEL_REVERSE 0U

/* 串口参数；引脚和时钟拓扑仍由 CubeMX IOC 管理。 */
#define MOTOR_UART_BAUD 115200U
#define CAMERA_UART_BAUD 115200U
#define CAMERA_REQUEST_RETRY_MS 1000U /* 等待期间补发同一视觉请求，不重置稳定窗口。 */
#define NAV_MAX_STEPS 128U
#define NAV_PC_FRESH_MS 5000U
#define NAV_ENTRY_LEFT_MM 125U
#define NAV_ENTRY_FORWARD_MM 165U
#define FORWARD_TEST_DISTANCE_MM 3000U /* 独立直行测试：App启动后向前300cm。 */
#define FORWARD_TEST_PPM MISSION_FORWARD_PPM
#define FORWARD_TEST_RPM 200U /* 仅300cm测试固件使用。 */
#define FORWARD_TEST_ACCEL 20U /* 约84.75RPM/s，仅300cm测试的主直行使用。 */
#define FORWARD_TEST_FRAME_GAP_MS 3U
/* 红线导航区2250×2170，左下角为(0,0)，与PC坐标一致。 */
#define NAV_GRID_WIDTH_MM 2250U
#define NAV_GRID_HEIGHT_MM 2170U
#define NAV_SCAN_CAR_X_MM 1975U
#define NAV_SCAN_TOP_CAR_Y_MM 1935U
#define NAV_SCAN_BOTTOM_CAR_Y_MM 200U
#define NAV_RADAR_FORWARD_MM 130U /* 雷达在车体旋转中心正前方13cm。 */
#define NAV_SCAN_SETTLE_MS 300U
#define START_UART_BAUD 115200U
#define IMU_UART_BAUD_RATE 9600U
#define IMU_OUTPUT_RATE_REG 0x06U /* 普通固件沿用10Hz；纯直行测试不初始化IMU。 */
#define UART_TX_TIMEOUT_MS 20U
#define MOTOR_ACK_TIMEOUT_MS 20U
#define IMU_YAW_SIGN (-1.0f)
#define IMU_RATE_SIGN (-1.0f)
#define IMU_FRESH_MS 200U
#define IMU_QUEUE 512U
#define CAMERA_RX_SIZE 256U
#define START_RX_SIZE 256U
#define MOTOR_RX_SIZE 512U

/* PWM：1 us 计数；修改周期时同时同步 IOC 的 TIM1/TIM3 ARR。 */
#define SERVO_PERIOD_US 20000U
#define SERVO_MIN_US 500U
#define SERVO_MAX_US 2500U
#define SERVO_UPDATE_MS (SERVO_PERIOD_US/1000U)
#define SERVO_GRIPPER_MAX_ANGLE 270U
#define SERVO_TURNTABLE_MAX_ANGLE 270U
#define SERVO_BASE_MAX_ANGLE 360U

#endif
