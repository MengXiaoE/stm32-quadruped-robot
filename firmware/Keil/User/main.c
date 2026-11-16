#include "stm32f10x.h"
#include "Delay.h"
#include "LED.h"
#include "PWM.h"
#include "BOO.h"
#include "Input.h"
#include "Servo.h"
#include "Serial.h"
#include "WS2812B.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

/* ================================================================
 *      >>>>>>  机身  偏  左  /  偏  右   修  偏  速  查  表  <<<<<<
 *
 *  跑偏怎么改？只改下面四个 TRIM 即可（前进/后退各一对），
 *  其它步态参数（DELAY / DELTA / LIFT）一般不用动。
 *
 *  ┌──────────────────────────────────────────────────────────┐
 *  │  *_LEFT_TRIM  > 0  →  削弱左侧推力  →  纠正 "机身偏右"  │
 *  │  *_LEFT_TRIM  < 0  →  加强左侧推力  →  纠正 "机身偏左"  │
 *  │  *_RIGHT_TRIM > 0  →  削弱右侧推力  →  纠正 "机身偏左"  │
 *  │  *_RIGHT_TRIM < 0  →  加强右侧推力  →  纠正 "机身偏右"  │
 *  └──────────────────────────────────────────────────────────┘
 *  侧别归属：左侧 = A2 + A4 (含 A6/A8)、右侧 = A1 + A3 (含 A5/A7)
 *  调机要点：一次只动 1~2°；左右 TRIM 通常只调一边；幅度别超 ±8,
 *           否则步态会明显跛。
 *
 *  >> 参数实际定义位置 <<
 *      前进:  FORWARD_LEFT_TRIM / FORWARD_RIGHT_TRIM  ( 见 "前进步态可调参数区" )
 *      后退:  BACK_LEFT_TRIM    / BACK_RIGHT_TRIM     ( 见 "后退步态可调参数区" )
 *      左/右转: 单侧推进型步态, 不设 TRIM; 要修走偏改 *_LIFT / *_STEP_DELTA
 *
 *  ★ 当前生效: FORWARD_LEFT_TRIM = +5.0  (修正"机身稍稍右偏")
 * ================================================================ */


/* ################################################################
 * ##                                                            ##
 * ##   >>>>>>   左 转 / 右 转   步  态   速  度   参  数    <<<<<<
 * ##                                                            ##
 * ##   ★ 调整"原地左转 / 原地右转"快慢, 所有参数集中在此处 ★    ##
 * ##   ★ 改完直接编译烧录即可生效; 不要去文件中段重复定义。  ★  ##
 * ##                                                            ##
 * ################################################################
 *
 *  ┌──────────────────────────────────────────────────────────┐
 *  │  *_DELAY_TIME → 单步耗时(节拍, 1拍=25ms); 越小越快        │
 *  │       4=100ms/步, 400ms/周期 (★快, 当前值)                │
 *  │       5=125ms/步, 500ms/周期 (适中)                      │
 *  │       6=150ms/步, 600ms/周期 (稳, 原默认)                 │
 *  │       < 4 时 MG90 跟不上, 会丢步/抽舵机, 不建议           │
 *  │  *_STEP_DELTA → 度/tick; 必须 DELTA × DELAY ≥ 30°(推程差) │
 *  │  *_LIFT       → 抬腿幅度(度); 太小擦地, 太大点头/晃身     │
 *  └──────────────────────────────────────────────────────────┘
 *  调速窍门: 要更快只把 DELAY 往下调; 太快擦地时把 LIFT +5
 *  适用范围: 仅控制"原地左转 / 原地右转"两套步态
 *           (前进/后退/TRIM 参数另有章节, 不在此处)
 * ================================================================ */
/* ----------- ★ 左转 (Left) 速度参数 ★ ----------- */
const uint16_t LEFT_DELAY_TIME   = 4;       // 左转 单步耗时: 4 节拍 = 100ms/步, 400ms/周期 (原值 6, 现已加快)
const float    LEFT_STEP_DELTA   = 12.0f;   // 左转 度/tick: 12×4 = 48° 已覆盖最大 30° 推程
const float    LEFT_LIFT         = 30.0f;   // 左转 抬腿幅度(度)

/* ----------- ★ 右转 (Right) 速度参数 ★ ----------- */
const uint16_t RIGHT_DELAY_TIME  = 4;       // 右转 单步耗时: 4 节拍 = 100ms/步, 400ms/周期 (原值 6, 现已加快)
const float    RIGHT_STEP_DELTA  = 12.0f;   // 右转 度/tick (与左转对称)
const float    RIGHT_LIFT        = 30.0f;   // 右转 抬腿幅度(度)
/* ================================================================ */


//基于STM32四足机器人(更新版)
//行走步态:前进，后退，原地左转，原地右转
//固定动作:招手，抬腿，展开，复位，摇摆，坐下
//外设:红外线，电磁炮，舵机云台，WS2812B彩色灯光，蜂鸣器，指示灯
//功能:离地检测，锂电池充放电
//实时状态机 + 舵机时序控制 + 非阻塞延时 + 标志位管理

//原项目作者：我的STM 32又烧了
//本开源版本在原项目基础上进行优化与开源整理。
//原创内容按仓库根目录 MIT License 发布；第三方代码按各自条款处理。

//!!!!!!注意程序默认串口波特率9600,因为新买的HC05蓝牙模块就是9600，无需配置就能与手机app直接连接。
//!!!!!!如果你之前使用了遥控器版本，或者设置了蓝牙模块波特率，那么在Serial.c程序里修改为对应波特率!


/* ================================================================
 *   ★★★  新  手  阅  读  指  南  （建 议 先 看 这 段）  ★★★
 * ================================================================
 *
 *  一、这个程序的骨架只有两句话
 *      先初始化各种外设 → 然后进入 while(1) 死循环，永远不退出。
 *      没有操作系统、没有多线程。机器狗所有"智能行为"，都是主循环
 *      一圈一圈转出来、靠"状态机 + 计数器"一点点推进的。
 *
 *  二、什么是"一个 tick（节拍）"？
 *      主循环最后有一句 Delay_ms(25)，所以正常情况下主循环约 25ms
 *      转一圈。我们把"转一圈"叫做一个 tick。
 *      代码里所有 *_DelayCnt 都在数 tick，例如 FORWARD_DELAY_TIME = 4
 *      表示"数满 4 个 tick（约 100ms）才换下一步"。
 *
 *  三、主循环每圈固定做 6 件事（顺序很重要）
 *      ① 输出：把 Angle1~Angle9 刷成 PWM，驱动 9 个舵机
 *      ② 输入：检查蓝牙有没有新指令，有就改状态/改目标角度
 *      ③ 云台：让 9 号舵机平滑地转向 ReceivedAngle9
 *      ④ 动作：摇摆/前进/后退/左转/右转 各自的状态机往前推一小步
 *      ⑤ 收尾：状态灯闪烁、蜂鸣器计时关断、串口超时保护
 *      ⑥ 节拍：Delay_ms(25)，然后回到 ①
 *
 *  四、命名规律（看懂这个，800 行能少读一半）
 *      Xxx_State     0 = 这个动作没在跑；非 0 = 正在跑（常表示状态机第几步）
 *      Xxx_Step      当前走到 4 步步态里的第几步，1→2→3→4→1 循环
 *      Xxx_DelayCnt  已经等了几个 tick；数够 DELAY_TIME 才换下一步
 *      Angle1~4      四条腿的"胯部"舵机：决定腿往前伸 / 往后蹬
 *      Angle5~8      四条腿的"膝部"舵机：决定脚抬起 / 落地
 *      Angle9        云台舵机
 *
 *  五、"非阻塞"是什么意思？
 *      新手写法常常是"舵机转 90°，然后 Delay_ms(500) 干等"。这样整机
 *      500ms 内什么都干不了。本程序改成：每 25ms 只转 15°，转 4 次
 *      自然到位。等待期间主循环照样能收蓝牙指令、闪灯、看传感器。
 *      真正会卡住主循环的只有 WS2812 灯效（内部有 1~2s 的阻塞延时），
 *      所以那条分支特意先把串口标志清掉再执行，防止丢指令。
 *
 *  六、volatile 是什么？
 *      volatile 告诉编译器"这个变量随时可能被别处改变，每次都老实去内存
 *      读写，不要缓存到寄存器里"。通常用于可能被中断服务函数改动的变量。
 *      本文件大量使用它属于保守写法，不影响理解，先知道含义即可。
 * ================================================================ */


/* Serial_RxPacket / Serial_RxFlag 的 extern 声明统一由 Serial.h 提供
 * (含 volatile 与 SERIAL_RX_PACKET_SIZE 宏), 本文件不再重复 extern,
 * 防止与 Serial.h 的声明出现 size/volatile 不一致. */

volatile uint8_t MAGE_State = 0;               // 模式状态标志

/* ----------------------------------------------------------------
 *  9 个舵机分别装在哪？（按本工程步态表里的腿号对应关系）
 *
 *      腿号          胯部(大腿根)   膝部(小腿)
 *      1 右前腿      Angle1         Angle5
 *      2 左前腿      Angle2         Angle6
 *      3 右后腿      Angle3         Angle7
 *      4 左后腿      Angle4         Angle8
 *      云台                          Angle9
 *
 *  胯部角度：腿"往前伸"还是"往后蹬"，范围 0~180°。
 *  膝部角度：脚"抬起"还是"落地"，A5/A7 直立为 0°，A6/A8 直立为 180°。
 *  硬件接线（见 PWM.c）：
 *      Angle1~4 → PA0/PA1/PA2/PA3，TIM2 四通道
 *      Angle5~8 → PB0/PB1/PA6/PA7，TIM3 四通道
 *      Angle9   → PA8，TIM1 通道1
 *  每个 SERVO 的 PWM 都是 50Hz（20ms 周期），0.5~2.5ms 脉宽对应 0~180°。
 * ---------------------------------------------------------------- */

/* 9 路舵机角度变量 —— 初值与 ResetAll() 内的目标姿态一致。
 * 这样从上电到主循环首帧 Servo_SetAngleN(Angle*) 之间, PWM 不会经过
 * 一个 90° 中间态再跳到 ResetAll 的 100/80/.../180, 避免开机瞬间舵机
 * "甩头". */
volatile float Angle1 = 100.0f;
volatile float Angle2 = 80.0f;
volatile float Angle3 = 90.0f;
volatile float Angle4 = 90.0f;
volatile float Angle5 = 0.0f;
volatile float Angle6 = 180.0f;
volatile float Angle7 = 0.0f;
volatile float Angle8 = 180.0f;
volatile float Angle9 = 100.0f;	// 云台舵机

// 云台舵机控制参数
volatile float ReceivedAngle9 = 100.0f;  // 目标角度
const float MinServoStep = 0.5f;         // 最小调节步长
const float MaxServoStep = 8.0f;         // 最大调节步长
const float FastThreshold = 20.0f;       // 快速调节阈值
const float SlowThreshold = 2.0f;        // 慢速调节阈值

// 功能动作状态标志
volatile uint8_t A4_State = 0;
volatile uint8_t A5_State = 0;
volatile uint8_t XNLED_State = 0;
volatile uint8_t ZMLED_State = 0;
volatile uint8_t HW_State = 0;

// 左右摇摆动作参数
volatile uint8_t ActionState = 0;          // 动作状态机
volatile uint16_t ActionDelayCnt = 0;       // 动作延时计数器
const uint16_t ACTION_DELAY_TIME = 12;     // 动作延时时间
const float ServoSwingStep = 2.0f;         // 舵机摆动步长

// 前进动作状态变量
volatile uint8_t Forward_State = 0;
volatile uint16_t Forward_DelayCnt = 0;
volatile uint8_t Forward_Step = 0;

/* ============================================================
 *           >>>  前进步态  可  调  参  数  区  <<<
 *  改完编译烧录即可生效。注释里写清楚每个参数的「类型 / 取值范围 /
 *  正负方向含义 / 经验起点」, 调坏了就照着回退。
 * ============================================================
 *
 * ① FORWARD_DELAY_TIME  —— 单步耗时(节拍数)
 *    类型: uint16_t, 单位: 主循环 tick (1 tick = 25ms)
 *    含义: 每一步(4 步一周期)持续多少 tick; 越小越快。
 *    范围: 建议 4~10; < 4 时 MG90 跟不上会丢步、抽舵机。
 *    经验值: 6=600ms周期(稳)、5=500ms周期(快)、4=400ms周期(极限, 当前)。
 *
 * ② FORWARD_STEP_DELTA  —— 单 tick 角度增量
 *    类型: float, 单位: 度/tick
 *    含义: 每个 tick 舵机最多前进多少度。必须保证
 *          FORWARD_STEP_DELTA × FORWARD_DELAY_TIME ≥ 60
 *          (60° = 胯前位↔后位差), 否则一步走不完整个推程。
 *    范围: 6.0 ~ 15.0; > 15 接近 MG90 极限 (15°/25ms), 会卡顿。
 *    经验值: DELAY=6→DELTA=10、DELAY=5→DELTA=12、DELAY=4→DELTA=15 (当前)。
 *
 * ③ FORWARD_LIFT  —— 抬腿幅度(小腿离地高度)
 *    类型: float, 单位: 度
 *    含义: 摆动相小腿从直立位(A5/A7=0、A6/A8=180)抬起多少度。
 *    范围: 20 ~ 60; 过小→擦地, 过大→重心起伏剧烈、整机点头。
 *    经验值: 45 (抗擦地+不晃的折中点)。地面光滑可降到 30。
 *    当前: 50 (比 45 略抬高, 更不容易擦地, 但机身起伏会稍大一点)。
 *
 * ④ FORWARD_LEFT_TRIM  —— 左侧推程修正
 *    类型: float, 单位: 度
 *    正负:「正值」= 把左侧(A2/A4)的「后位」往中间收 → 左推变弱 → 修「偏右」
 *         「负值」= 把左侧后位往外推 → 左推变强 → 修「偏左」
 *    范围: -8 ~ +8。值越大修偏越猛, > 8 时步态会明显跛。
 *    经验值: 0 起步; 偏右就 +; 偏左就 -。一次改 1~2°。
 *
 * ⑤ FORWARD_RIGHT_TRIM —— 右侧推程修正
 *    类型: float, 单位: 度
 *    正负:「正值」= 把右侧(A1/A3)后位往中间收 → 右推变弱 → 修「偏左」
 *         「负值」= 把右侧后位往外推 → 右推变强 → 修「偏右」
 *    范围: -8 ~ +8。同 ④ 用法, 一般 ④ 和 ⑤ 只调一个就够。
 *    经验值: 0 起步。
 * ============================================================ */
const uint16_t FORWARD_DELAY_TIME  = 4;       // ① 节拍数 (4×25ms = 100ms/步, 400ms/周期)
const float    FORWARD_STEP_DELTA  = 15.0f;   // ② 度/tick (15×4 = 60° 刚好走完推程)
const float    FORWARD_LIFT        = 50.0f;   // ③ 抬腿幅度 (45→50, 抬得更离地)
const float    FORWARD_LEFT_TRIM   = 5.0f;    // ④ ★ 当前生效: +3° 修正"机身稍稍右偏" ★
const float    FORWARD_RIGHT_TRIM  = 0.0f;    // ⑤ 右侧修正 (未启用)

/* —— 前进步态目标角度表 (由上方调参区自动算出, 不要手改下面这段) ——
 * 对角小跑 trot: (腿1右前,腿4左后) 与 (腿2左前,腿3右后) 互为相反相位。
 * 胯位定义: A1 前=165/后=105、A4 前=105/后=165、A2 前=15/后=75、A3 前=75/后=15。
 * 「后位」收紧(- LEFT_TRIM / - RIGHT_TRIM) = 减少该侧推程 → 把机身往同侧拉。
 * 膝位: A5/A7 直立=0、抬起=+LIFT; A6/A8 直立=180、抬起=180-LIFT。
 *
 * 步态拓扑: 抬腿/摆胯必须同步, 避免"已落地的脚还在摆胯 → 拖脚"。
 *   Step 1 → 2  推进相 A: 腿1+4 在地推后位、腿2+3 抬起摆前位
 *   Step 2 → 3  切换相: 胯位不动, 仅切换抬腿(腿1+4 抬起、腿2+3 落地)
 *   Step 3 → 4  推进相 B: 腿2+3 在地推后位、腿1+4 抬起摆前位
 *   Step 4 → 1  切换相: 胯位不动, 仅切换抬腿(腿1+4 落地、腿2+3 抬起)
 */
const float Forward_Targets[4][8] = {
    //            A1                          A2                          A3                          A4                          A5            A6                      A7            A8
    { 165.0f,                      75.0f - FORWARD_LEFT_TRIM,  15.0f + FORWARD_RIGHT_TRIM, 105.0f,                     0.0f,         180.0f - FORWARD_LIFT,  FORWARD_LIFT, 180.0f                }, // 1: 腿1+4 落地在前位, 腿2+3 抬起在后位
    { 105.0f + FORWARD_RIGHT_TRIM, 15.0f,                      75.0f,                      165.0f - FORWARD_LEFT_TRIM, 0.0f,         180.0f - FORWARD_LIFT,  FORWARD_LIFT, 180.0f                }, // 2: 腿1+4 推到后位(推进), 腿2+3 抬起摆到前位(同步)
    { 105.0f + FORWARD_RIGHT_TRIM, 15.0f,                      75.0f,                      165.0f - FORWARD_LEFT_TRIM, FORWARD_LIFT, 180.0f,                 0.0f,         180.0f - FORWARD_LIFT }, // 3: 胯位不变, 抬腿切换(腿1+4 抬, 腿2+3 落地于前位)
    { 165.0f,                      75.0f - FORWARD_LEFT_TRIM,  15.0f + FORWARD_RIGHT_TRIM, 105.0f,                     FORWARD_LIFT, 180.0f,                 0.0f,         180.0f - FORWARD_LIFT }, // 4: 腿2+3 推到后位(推进), 腿1+4 抬起摆到前位(同步)
};

// 后退动作状态变量
volatile uint8_t Back_State = 0;
volatile uint16_t Back_DelayCnt = 0;
volatile uint8_t Back_Step = 0;

/* ============================================================
 *           >>>  后退步态  可  调  参  数  区  <<<
 *  含义与前进对称: 把"推程方向"从「往后推」改成「往前推」即可。
 *  TRIM 在后退里作用于「前位」(因为后退时前位才是推程末端)。
 * ============================================================
 *  ① BACK_DELAY_TIME   节拍数 (tick=25ms), 建议 4~10
 *  ② BACK_STEP_DELTA   度/tick, 需满足 ②×① ≥ 60° (推程角度差)
 *  ③ BACK_LIFT         抬腿幅度, 20~60
 *  ④ BACK_LEFT_TRIM    左侧(A2/A4)前位修正: 偏右↑(+) / 偏左↓(-)
 *  ⑤ BACK_RIGHT_TRIM   右侧(A1/A3)前位修正: 偏左↑(+) / 偏右↓(-)
 *  (TRIM 正负规则与前进保持一致, 只是作用在前位, 不再是后位)
 * ============================================================ */
const uint16_t BACK_DELAY_TIME = 5;       // ① 5×25ms = 125ms/步, 500ms/周期
const float    BACK_STEP_DELTA = 12.0f;   // ② 度/tick (12×5 = 60° 刚好走完推程)
const float    BACK_LIFT       = 45.0f;   // ③ 抬腿幅度
const float    BACK_LEFT_TRIM  = 0.0f;    // ④ 左侧前位修正 (后退暂未观察到偏移, 留 0)
const float    BACK_RIGHT_TRIM = 0.0f;    // ⑤ 右侧前位修正

/* —— 后退步态目标角度表 ——
 * 对角小跑 trot, 相位与前进完全相同(腿1+4 与 腿2+3 交替推进 / 摆动);
 * 唯一区别: 胯位的「前位 / 后位」互换 —— 落地腿从「前→后」改成「从后→前」。
 *
 *   Step 1 → 2  推进相 A: 腿1+4 在地推前位、腿2+3 抬起摆后位
 *   Step 2 → 3  切换相: 胯位不动, 仅切换抬腿(腿1+4 抬起、腿2+3 落地)
 *   Step 3 → 4  推进相 B: 腿2+3 在地推前位、腿1+4 抬起摆后位
 *   Step 4 → 1  切换相: 胯位不动, 仅切换抬腿(腿1+4 落地、腿2+3 抬起)
 */
const float Back_Targets[4][8] = {
    //            A1                          A2                          A3                          A4                          A5         A6                   A7         A8
    { 105.0f,                    15.0f + BACK_LEFT_TRIM,  75.0f - BACK_RIGHT_TRIM, 165.0f,                  0.0f,      180.0f - BACK_LIFT,  BACK_LIFT, 180.0f             }, // 1: 腿1+4 落地于后位, 腿2+3 抬起在前位
    { 165.0f - BACK_RIGHT_TRIM,  75.0f,                   15.0f,                   105.0f + BACK_LEFT_TRIM, 0.0f,      180.0f - BACK_LIFT,  BACK_LIFT, 180.0f             }, // 2: 腿1+4 从后推到前位(后退推进), 腿2+3 抬起摆到后位(同步)
    { 165.0f - BACK_RIGHT_TRIM,  75.0f,                   15.0f,                   105.0f + BACK_LEFT_TRIM, BACK_LIFT, 180.0f,              0.0f,      180.0f - BACK_LIFT }, // 3: 胯位不变, 抬腿切换(腿1+4 抬, 腿2+3 落地于后位)
    { 105.0f,                    15.0f + BACK_LEFT_TRIM,  75.0f - BACK_RIGHT_TRIM, 165.0f,                  BACK_LIFT, 180.0f,              0.0f,      180.0f - BACK_LIFT }, // 4: 腿2+3 从后推到前位(后退推进), 腿1+4 抬起摆到后位(同步)
};

// 左转动作状态变量 (速度参数在文件顶部 "★ 左转 (Left) 速度参数 ★" 区)
volatile uint8_t Left_State = 0;
volatile uint16_t Left_DelayCnt = 0;
volatile uint8_t Left_Step = 0;

// 对角小跑左转目标表: 腿1+腿4 与 腿2+腿3 交替抬起摆胯
const float Left_Targets[4][8] = {
    //  A1      A2     A3     A4      A5         A6                  A7         A8
    { 150.0f, 30.0f, 30.0f, 150.0f, LEFT_LIFT, 180.0f,             0.0f,      180.0f - LEFT_LIFT }, // 1: 腿1+腿4 抬起, 腿2+腿3 落地后位
    { 165.0f, 15.0f, 15.0f, 165.0f, 0.0f,      180.0f,             0.0f,      180.0f             }, // 2: 全部落地, 腿1/4推到极前、腿2/3推到极后
    { 165.0f, 15.0f, 15.0f, 165.0f, 0.0f,      180.0f - LEFT_LIFT, LEFT_LIFT, 180.0f             }, // 3: 腿2+腿3 抬起, 腿1/4 落地保持
    { 135.0f, 45.0f, 45.0f, 135.0f, 0.0f,      180.0f,             0.0f,      180.0f             }, // 4: 回到标准站立过渡
};

// 右转动作状态变量 (速度参数在文件顶部 "★ 右转 (Right) 速度参数 ★" 区)
volatile uint8_t Right_State = 0;
volatile uint16_t Right_DelayCnt = 0;
volatile uint8_t Right_Step = 0;

// 对角小跑右转目标表: 相对左转, 腿1+腿4 与 腿2+腿3 的抬起顺序对调
const float Right_Targets[4][8] = {
    //  A1      A2     A3     A4      A5          A6                  A7          A8
    { 150.0f, 30.0f, 30.0f, 150.0f, 0.0f,       180.0f - RIGHT_LIFT, RIGHT_LIFT, 180.0f              }, // 1: 腿2+腿3 抬起
    { 165.0f, 15.0f, 15.0f, 165.0f, 0.0f,       180.0f,              0.0f,       180.0f              }, // 2: 全部落地
    { 165.0f, 15.0f, 15.0f, 165.0f, RIGHT_LIFT, 180.0f,              0.0f,       180.0f - RIGHT_LIFT }, // 3: 腿1+腿4 抬起
    { 135.0f, 45.0f, 45.0f, 135.0f, 0.0f,       180.0f,              0.0f,       180.0f              }, // 4: 回到标准站立过渡
};

// LED闪烁参数
volatile uint8_t LED_State = 0;            // LED状态
volatile uint16_t LED_DelayCnt = 0;         // LED延时计数器
const uint16_t LED_DELAY_TIME = 12;        // LED闪烁间隔

// 滑动动作参数 (当前主循环里被注释停用, 仅 Huadong_Smooth() 函数保留)
volatile uint8_t Huadong_State = 0;
volatile uint8_t Huadong_Step = 0;
const uint8_t MAX_STEP = 15;
const uint8_t STEP_INTERVAL = 1;
volatile uint8_t step_delay = 0;
volatile uint8_t huadong_release_flag = 0;

uint16_t booOffCnt = 0;

/* 一次性清掉所有"会持续运动"的状态机/步骤/计数器。
 * 切换步态、收到 REST/ZK/ZX 等指令前调一次, 避免两套步态同时跑导致舵机互抢。
 * 注意: 不复位舵机角度本身(角度交给调用方/插值器接管), 也不动云台、LED、灯效。 */
static void StopAllGaits(void)
{
    ActionState = 0;     ActionDelayCnt = 0;
    Forward_State = 0;   Forward_Step = 0;   Forward_DelayCnt = 0;
    Back_State = 0;      Back_Step = 0;      Back_DelayCnt = 0;
    Left_State = 0;      Left_Step = 0;      Left_DelayCnt = 0;
    Right_State = 0;     Right_Step = 0;     Right_DelayCnt = 0;
}

void ResetAll(void)
{
    // 舵机角度复位
    Angle1 = 100.0f; Angle2 = 80.0f; Angle3 = 90.0f; Angle4 = 90.0f;
    Angle5 = 0.0f;  Angle6 = 180.0f; Angle7 = 0.0f; Angle8 = 180.0f;
    Angle9 = 100.0f;
    ReceivedAngle9 = 100.0f;

    // 所有动作状态复位
    StopAllGaits();
    A4_State = 0;
    A5_State = 0;
    XNLED_State = 0;
    MAGE_State = 0;

}

void Huadong_Smooth(void)
{
    // 上部舵机保持固定姿态
    Angle1 = 135.0f; Angle2 = 45.0f; Angle3 = 45.0f; Angle4 = 135.0f;
    Servo_SetAngle1(Angle1); Servo_SetAngle2(Angle2);
    Servo_SetAngle3(Angle3); Servo_SetAngle4(Angle4);

    // 滑动动作延时控制
    step_delay++;
    if (step_delay < STEP_INTERVAL) return;
    step_delay = 0;

    // 滑动状态机：正滑/反滑切换
    switch(Huadong_State)
    {
        case 1:
            Huadong_Step++;
            if (Huadong_Step >= MAX_STEP)
            {
                Huadong_Step = MAX_STEP;
                Huadong_State = 2;
            }
            break;
        case 2:
            Huadong_Step--;
            if (Huadong_Step <= 0)
            {
                Huadong_Step = 0;
                Huadong_State = 1;
            }
            break;
        default:
            return;
    }

    // 计算下部舵机角度
    Angle5 = Huadong_Step * 2;
    Angle7 = Huadong_Step * 2;
    Angle6 = 180 - (Huadong_Step * 2);
    Angle8 = 180 - (Huadong_Step * 2);

    // 更新下部舵机
    Servo_SetAngle5(Angle5); Servo_SetAngle6(Angle6);
    Servo_SetAngle7(Angle7); Servo_SetAngle8(Angle8);
}

/* ----------------------------------------------------------------
 *  插值小工具：把 *current 朝 target 挪一小步，每拍最多挪 step 度。
 *
 *  例：current = 100，target = 165，step = 12
 *      100 → 112 → 124 → ... → 165（大约 6 拍走完）
 *  如果差距已经不超过 step，就直接等于 target，避免在目标附近来回抖。
 *
 *  为什么需要它？
 *  舵机不是"设一个角度就瞬间到"，直接让 100° 跳到 165° 会造成
 *  电流猛增 → 电池电压被拉低 → 单片机复位 / 舵机抽搐。
 *  拆成每拍十几度的小步，机械和电源都吃得消。
 *
 *  参数用 volatile float *（指针）而不是返回值，是为了直接修改
 *  main.c 里的全局角度变量 AngleX。
 * ---------------------------------------------------------------- */
// 把 *current 向 target 推进, 单步最多移动 step 度
static void InterpolateTo(volatile float *current, float target, float step)
{
    float diff = target - *current;
    if (diff > step)       *current += step;
    else if (diff < -step) *current -= step;
    else                   *current  = target;
}

/* ================================================================
 *  main()：整个固件的唯一入口
 *  ----------------------------------------------------------------
 *  【第一阶段】上电初始化（只执行一次，在 while 之前）
 *      下面 7 行把机器狗身上所有硬件"点亮/配置好"，顺序可以理解为：
 *      先让舵机能动 → 让耳朵(蓝牙)能听 → 让按钮/传感器能读
 *      → 让灯和嘴(蜂鸣器)能响 → 最后摆一个标准站姿。
 *
 *  【第二阶段】while(1) 主循环（约 25ms 一圈，永不退出）
 *      ① 输出   Angle1~9 → PWM，真正驱动舵机
 *      ② 输入   解析蓝牙指令，修改状态机 / 目标角度
 *      ③ 云台   9 号舵机平滑转向
 *      ④ 动作   各个步态状态机各自推进一步（核心）
 *      ⑤ 收尾   LED、蜂鸣器、串口超时
 *      ⑥ 节拍   Delay_ms(25)
 *
 *  注意：②里每收到一条新指令都会先 StopAllGaits()，把其它步态清零，
 *  所以④里虽然并列写了 5 个 if，同一时刻实际上只有一个步态在跑，
 *  不会出现"前进和左转同时抢舵机"的情况。
 * ================================================================ */
int main(void)
{
    // 外设初始化
    Servo_Init();        // 配置 TIM2/TIM3/TIM1 → 9 路 50Hz 舵机 PWM

    Serial_Init();       // USART1 9600 8N1（PA9 发 / PA10 收），接 HC-05 蓝牙
    Input_Init();        // 输入引脚（按键 / 离地检测等）
    WS2812B_Init();      // 彩色灯带
    BOO_Init();          // 蜂鸣器、电磁炮等开关量输出
    LED_Init();          // 两颗状态指示灯（PA12 / PA15）
    LED_StartBlink();    // 让状态灯从第一圈就开始闪（主机活着的"心跳"）

    // 系统初始复位：舵机回到标准站姿，清空所有动作状态机
    ResetAll();

    /* ============================================================
     *  主循环开始。下面每一段都对应上方注释里的 ①~⑥。
     *  记住：这一圈算出来的新角度，会在下一圈开头才输出给舵机，
     *  所以动作上会有约 25ms 的延迟，肉眼完全看不出来。
     * ============================================================ */
    while (1)
    {
        /* ---------- ① 舵机输出：把"想要的角度"变成 PWM 波形 ----------
         * Angle1~9 只是内存里的 float 变量，舵机并不认识；
         * Servo_SetAngleN() 会把它换算成 0.5~2.5ms 脉宽（50Hz）。
         * 每圈都无脑刷一遍：不管前面谁改了 AngleX，下一圈自动生效。
         * 这也是"舵机只能通过改 Angle 变量来控制"的简单约定。 */
        // 实时刷新所有舵机角度
        Servo_SetAngle1(Angle1);
        Servo_SetAngle2(Angle2);
        Servo_SetAngle3(Angle3);
        Servo_SetAngle4(Angle4);
        Servo_SetAngle5(Angle5);
        Servo_SetAngle6(Angle6);
        Servo_SetAngle7(Angle7);
        Servo_SetAngle8(Angle8);
        Servo_SetAngle9(Angle9);

        /* ---------- ② 输入：蓝牙指令解析 ----------
         * 一帧数据的格式：'@' + 正文 + '#' + '*'
         *   例如  @UP#*     前进
         *         @BACK#*   后退
         *         @LEFT#*   原地左转
         *         @RIGHT#*  原地右转
         *         @TJ#*     抬腿 / 放下
         *         @REST#*   停止并回到标准站姿
         *         @100#*    云台转到 100°
         * '@' 和 '#*' 已经被 Serial.c 的中断服务函数剥掉，正文存进
         * Serial_RxPacket，并置 Serial_RxFlag = 1。
         * 主循环看到 Flag == 1，就知道"有一条新指令等着处理"。
         *
         * 下面这一长串 if / else if 本质上是"字符串查表"：
         * strncasecmp(cmd, "UP", 2) == 0 表示
         * "cmd 的前 2 个字符忽略大小写后等于 UP"，命中就走对应分支。
         */
        /************************ 串口指令解析处理 ************************/
        if (Serial_RxFlag == 1)
        {
            /* 把待解析包先拷到栈上局部 cmd[], 然后立刻清 Serial_RxPacket
             * 并把 Serial_RxFlag 置 0, 释放给 ISR。
             * 这样后面即便走到 WS2812 那种阻塞 1~2s 的灯效分支, ISR 也能在
             * 那段时间内接收下一条蓝牙指令; 否则 ISR 因 Flag==1 一直拒收 '@',
             * 整个 WS2812 期间用户所有指令(包括 REST) 都会丢失。 */
            char cmd[SERIAL_RX_PACKET_SIZE];
            memcpy(cmd, (const char *)Serial_RxPacket, SERIAL_RX_PACKET_SIZE);
            cmd[SERIAL_RX_PACKET_SIZE - 1] = '\0';   // 兜底, 防止后续 str* 跑飞

            memset(Serial_RxPacket, 0, SERIAL_RX_PACKET_SIZE);
            Serial_RxFlag = 0;   // 释放 ISR; 后续解析全部针对 cmd, 不再读共享缓冲

            /* strtof 尝试把整串文本解析成小数：
             * 成功时 endptr 指向字符串末尾、且数值在 0~180 之内，
             * 就把它当成云台目标角度（例如手机发送 @100#*）。 */
            char *endptr = NULL;
            float num = strtof(cmd, &endptr);

            // 云台角度指令(0-180 数字)
            if (endptr != cmd && *endptr == '\0' && num >= 0.0f && num <= 180.0f)
            {
                ReceivedAngle9 = num;
            }
            // 复位指令
            else if (strncasecmp(cmd, "REST", 4) == 0)
            {
                ResetAll();   // ResetAll 内部已调 StopAllGaits()
                Serial_SendString("OK\r\n");
            }
            // 展开指令
            else if (strncasecmp(cmd, "ZK", 2) == 0)
            {
                StopAllGaits();
                Angle1 = 135.0f; Angle2 = 45.0f; Angle3 = 45.0f; Angle4 = 135.0f;
                Angle5 = 0.0f;   Angle6 = 180.0f; Angle7 = 0.0f; Angle8 = 180.0f;
                A4_State = 0; A5_State = 0;
            }
            // 坐下指令
            else if (strncasecmp(cmd, "ZX", 2) == 0)
            {
                StopAllGaits();
                Angle1 = 135.0f; Angle2 = 45.0f; Angle3 = 45.0f; Angle4 = 135.0f;
                Angle5 = 90.0f;  Angle6 = 90.0f;  Angle7 = 90.0f; Angle8 = 90.0f;
                A4_State = 0; A5_State = 0;
            }
            // 左右摇摆指令
            else if (strncasecmp(cmd, "YB", 2) == 0)
            {
                StopAllGaits();
                ActionState = 1;   // 启动摇摆状态机
                A4_State = 0; A5_State = 0;
            }
            // 招手动作切换
            else if (strncasecmp(cmd, "ZS", 2) == 0)
            {
                StopAllGaits();
                if (A4_State == 0)
                {
                    Angle1 = 160.0f; Angle2 = 45.0f; Angle3 = 110.0f; Angle4 = 70.0f;
                    Angle5 = 20.0f;  Angle6 = 35.0f;  Angle7 = 20.0f; Angle8 = 160.0f;
                    A4_State = 1;
                }
                else
                {
                    Angle1 = 135.0f; Angle2 = 45.0f; Angle3 = 45.0f; Angle4 = 135.0f;
                    Angle5 = 0.0f;   Angle6 = 180.0f; Angle7 = 0.0f; Angle8 = 180.0f;
                    A4_State = 0;
                }
            }
            // 抬脚动作切换
            else if (strncasecmp(cmd, "TJ", 2) == 0)
            {
                StopAllGaits();
                if (A5_State == 0)
                {
                    Angle1 = 90.0f; Angle2 = 90.0f; Angle3 = 45.0f; Angle4 = 135.0f;
                    Angle5 = 0.0f;  Angle6 = 180.0f; Angle7 = 135.0f; Angle8 = 180.0f;
                    A5_State = 1;
                }
                else
                {
                    Angle1 = 135.0f; Angle2 = 45.0f; Angle3 = 45.0f; Angle4 = 135.0f;
                    Angle5 = 0.0f;   Angle6 = 180.0f; Angle7 = 0.0f; Angle8 = 180.0f;
                    A5_State = 0;
                }
            }
            /* ↓ 从"前进"开始是运动类指令。
             * 它们并不直接去拧舵机，只做两件事：
             *   1) StopAllGaits()：把正在跑的动作全部停掉（防止抢舵机）；
             *   2) 把某个 Xxx_State 置 1、Xxx_Step 置 1，相当于"点火"。
             * 真正一步步迈腿的动作，交给下面第 ④ 段每圈推一次。 */
            // 前进指令
            else if (strncasecmp(cmd, "UP", 2) == 0)
            {
                StopAllGaits();
                Forward_State = 1;
                Forward_Step = 1;
                A4_State = 0;
                A5_State = 0;
            }
            // 后退指令
            else if (strncasecmp(cmd, "BACK", 4) == 0)
            {
                StopAllGaits();
                Back_State = 1;
                Back_Step = 1;
                A4_State = 0;
                A5_State = 0;
            }
            // 左转指令
            else if (strncasecmp(cmd, "LEFT", 4) == 0)
            {
                StopAllGaits();
                Left_State = 1;
                Left_Step = 1;
                A4_State = 0;
                A5_State = 0;
            }
            // 右转指令
            else if (strncasecmp(cmd, "RIGHT", 5) == 0)
            {
                StopAllGaits();
                Right_State = 1;
                Right_Step = 1;
                A4_State = 0;
                A5_State = 0;
            }
            // 红外控制
            else if (strncasecmp(cmd, "HW", 2) == 0)
            {
                if (HW_State == 0)
                {
                    HW_ON();
                    HW_State = 1;
                }
                else
                {
                    HW_State = 0;
                    HW_OFF();
                }
            }
            // 炫彩灯模式控制 (注意: 内部 Delay_us(60000~80000) 累计阻塞 ~1.5s)
            else if (strncasecmp(cmd, "XNLED", 5) == 0)
            {
                if (XNLED_State == 0)
                {
                    FlowFrom10To9_AllOn(255, 20, 0, 60000);
                    WS2812_Update();
                    XNLED_State = 1;
                }
                else
                {
                    ColorFlowFrom10To9_LargeChange(80000);
                    WS2812_Update();
                    XNLED_State = 0;
                }
            }
            // 照明灯控制
            else if (strncasecmp(cmd, "ZMLED", 5) == 0)
            {
                if (ZMLED_State == 0)
                {
                    All_LED_On(200, 200, 200);
                    WS2812_Update();
                    ZMLED_State = 1;
                }
                else
                {
                    All_LED_On(0, 0, 0);
                    WS2812_Update();
                    ZMLED_State = 0;
                }
            }
            else if (strncasecmp(cmd, "BOO", 3) == 0)
            {
                Color_Wipe(255, 20, 0, 50000);
                WS2812_Update();
                BOO_ON();
                booOffCnt = 16;
            }
        }

        /* ---------- ③ 云台舵机平滑调节 ----------
         * 这里是一段"比例调速"：
         *   离目标越远 → servoStep 越大（最多 8°/拍），转得快；
         *   离目标越近 → servoStep 越小（最少 0.5°/拍），慢慢靠拢；
         *   差距小于 0.1° → 直接吸附到目标，防止永远差一点点。
         * 效果就是"起步快、到位轻"，不会猛地一顿。
         */
        /************************ 云台舵机平滑调节 ************************/
        float diff = ReceivedAngle9 - Angle9;
        float absDiff = fabs(diff);
        float servoStep = MinServoStep;

        if (absDiff >= FastThreshold) servoStep = MaxServoStep;
        else if (absDiff > SlowThreshold)
        {
            servoStep = MinServoStep + (MaxServoStep - MinServoStep) *
                       (absDiff - SlowThreshold) / (FastThreshold - SlowThreshold);
        }

        if (diff > servoStep)
        {
            Angle9 += servoStep;
            if (Angle9 > 180.0f) Angle9 = 180.0f;
        }
        else if (diff < -servoStep)
        {
            Angle9 -= servoStep;
            if (Angle9 < 0.0f) Angle9 = 0.0f;
        }
        else if (absDiff > 0.1f)
        {
            Angle9 = ReceivedAngle9;
        }

        /* 摇摆：4 个小状态轮流执行（右侧膝部弯→直，左侧膝部弯→直），
         * 靠左右两侧交替弯曲/伸直让机身左右晃动。
         * 每个状态持续 ACTION_DELAY_TIME 个 tick。收到 "YB" 后一直循环，
         * 直到收到 REST 或其它运动指令把它停掉。 */
        /************************ 左右摇摆动作执行 ************************/
        if (ActionState != 0)
        {
            ActionDelayCnt++;
            switch(ActionState)
            {
                case 1:
                    if (Angle5 > 0.0f) Angle5 -= ServoSwingStep;
                    if (Angle7 > 0.0f) Angle7 -= ServoSwingStep;
                    if (Angle6 < 180.0f) Angle6 += ServoSwingStep;
                    if (Angle8 < 180.0f) Angle8 += ServoSwingStep;
                    if (ActionDelayCnt >= ACTION_DELAY_TIME)
                    {
                        ActionState = 2; ActionDelayCnt = 0;
                    }
                    break;
                case 2:
                    if (Angle5 < 40.0f) Angle5 += ServoSwingStep;
                    if (Angle7 < 40.0f) Angle7 += ServoSwingStep;
                    if (ActionDelayCnt >= ACTION_DELAY_TIME)
                    {
                        ActionState = 3; ActionDelayCnt = 0;
                    }
                    break;
                case 3:
                    if (Angle5 > 0.0f) Angle5 -= ServoSwingStep;
                    if (Angle7 > 0.0f) Angle7 -= ServoSwingStep;
                    if (ActionDelayCnt >= ACTION_DELAY_TIME)
                    {
                        ActionState = 4; ActionDelayCnt = 0;
                    }
                    break;
                case 4:
                    if (Angle6 > 140.0f) Angle6 -= ServoSwingStep;
                    if (Angle8 > 140.0f) Angle8 -= ServoSwingStep;
                    if (ActionDelayCnt >= ACTION_DELAY_TIME)
                    {
                        ActionState = 1; ActionDelayCnt = 0;
                    }
                    break;
                default:
                    ActionState = 0; ActionDelayCnt = 0;
                    break;
            }
        }

        /* ============================================================
         *  ④ 动作状态机（整份程序的核心，看懂这一段就懂机器狗怎么走）
         * ============================================================
         *  下面 5 个 if（摇摆、前进、后退、左转、右转）写法几乎一样，
         *  都是"状态机 + 插值"。以【前进】为例：
         *
         *      每圈：Forward_DelayCnt++
         *              ↓ 数够 FORWARD_DELAY_TIME(=4) 个 tick 了吗？
         *          是 → Forward_Step 切下一步（1→2→3→4→1 循环），
         *               计数器归零
         *          否 → 步数不变
         *              ↓
         *      每圈：对 8 个角度各调用一次 InterpolateTo()，
         *            朝"当前这一步的目标角度"最多挪 15°。
         *
         *  一句话分工：
         *      Step 决定"去哪"  —— 查 Forward_Targets 表，4 个离散姿势
         *      InterpolateTo 决定"怎么去" —— 每拍挪一点，舵机不瞬移
         *
         *  ▼ 前进一个完整周期 = 4 步 × 4 tick × 25ms = 400ms：
         *
         *   第1步  腿1(右前)+腿4(左后) 落地在前位，      ┐ 起始姿势
         *          腿2(左前)+腿3(右后) 抬起在后位       ┘
         *   第2步  腿1+腿4 踩住地面，胯部从前位蹬到后位，  ┐ 推进相 A
         *          同时腿2+腿3 抬在空中，从后摆到前位      ┘
         *   第3步  胯部角度不变，只交换抬腿：              ┐ 换腿相
         *          腿1+腿4 抬起，腿2+腿3 落地              ┘
         *   第4步  腿2+腿3 踩住地面，从前往后蹬，          ┐ 推进相 B
         *          同时腿1+腿4 抬在空中，从后摆到前位      ┘
         *   回到第1步：再交换一次抬腿，如此循环 → 机器狗就走起来了。
         *
         *  为什么能往前走？
         *      落地腿的脚掌和地面有摩擦，胯部往后蹬时脚相对地面不动，
         *      反作用力把机身往前推；抬起的腿趁机收回来，准备下一次蹬地。
         *
         *  为什么对角腿一组（1+4、2+3）？
         *      这叫"对角小跑(trot)"，两条对角腿同相，机身受力对称、不易翻。
         *      如果同侧两条腿一起抬，机身就会往那一侧倒。
         *
         *  ▼ 后退（Back_Targets）：结构完全对称，只是落地腿改成从后位
         *     往前蹬，于是机身往反方向移动。
         *  ▼ 左转/右转：两组对角腿交替"抬起换位 / 落地撑住"，用左右不
         *     对称的推力产生原地旋转；速度参数在文件顶部单独配置。
         *
         *  小提示：这些 if 是并列写的，看起来能同时跑，但每条运动指令都会
         *  先 StopAllGaits() 清零，所以同一时刻只有一个 State 非 0。
         * ============================================================ */
        /************************ 前进动作执行（插值版） ************************/
        if (Forward_State != 0)
        {
            Forward_DelayCnt++;
            if (Forward_DelayCnt >= FORWARD_DELAY_TIME)
            {
                Forward_DelayCnt = 0;
                Forward_Step++;
                if (Forward_Step > 4) Forward_Step = 1;
            }

            /* C 数组下标从 0 开始，而 Step 是给人看的 1~4，所以减 1 得到
             * 0~3 再去查表；问号表达式是"万一越界就用第 0 行"的保险。 */
            // 1..4 → 0..3, 防越界
            uint8_t idx = (Forward_Step >= 1 && Forward_Step <= 4) ? (Forward_Step - 1) : 0;
            InterpolateTo(&Angle1, Forward_Targets[idx][0], FORWARD_STEP_DELTA);
            InterpolateTo(&Angle2, Forward_Targets[idx][1], FORWARD_STEP_DELTA);
            InterpolateTo(&Angle3, Forward_Targets[idx][2], FORWARD_STEP_DELTA);
            InterpolateTo(&Angle4, Forward_Targets[idx][3], FORWARD_STEP_DELTA);
            InterpolateTo(&Angle5, Forward_Targets[idx][4], FORWARD_STEP_DELTA);
            InterpolateTo(&Angle6, Forward_Targets[idx][5], FORWARD_STEP_DELTA);
            InterpolateTo(&Angle7, Forward_Targets[idx][6], FORWARD_STEP_DELTA);
            InterpolateTo(&Angle8, Forward_Targets[idx][7], FORWARD_STEP_DELTA);
        }

        /* 后退：和前进同一套逻辑，只是查 Back_Targets 表、
         * 用 BACK_* 参数，推程方向反过来。看懂前进即可照葫芦画瓢。 */
        /************************ 后退动作执行（插值版） ************************/
        if (Back_State != 0)
        {
            Back_DelayCnt++;
            if (Back_DelayCnt >= BACK_DELAY_TIME)
            {
                Back_DelayCnt = 0;
                Back_Step++;
                if (Back_Step > 4) Back_Step = 1;
            }

            uint8_t idx = (Back_Step >= 1 && Back_Step <= 4) ? (Back_Step - 1) : 0;
            InterpolateTo(&Angle1, Back_Targets[idx][0], BACK_STEP_DELTA);
            InterpolateTo(&Angle2, Back_Targets[idx][1], BACK_STEP_DELTA);
            InterpolateTo(&Angle3, Back_Targets[idx][2], BACK_STEP_DELTA);
            InterpolateTo(&Angle4, Back_Targets[idx][3], BACK_STEP_DELTA);
            InterpolateTo(&Angle5, Back_Targets[idx][4], BACK_STEP_DELTA);
            InterpolateTo(&Angle6, Back_Targets[idx][5], BACK_STEP_DELTA);
            InterpolateTo(&Angle7, Back_Targets[idx][6], BACK_STEP_DELTA);
            InterpolateTo(&Angle8, Back_Targets[idx][7], BACK_STEP_DELTA);
        }

        /* 左转/右转：同样是"计够 tick → 换 Step → 插值过去"，
         * 只是目标表 Left_Targets / Right_Targets 描述的是原地转身姿势。 */
        /************************ 左转动作执行（插值版）************************/
        if (Left_State != 0)
        {
            Left_DelayCnt++;
            if (Left_DelayCnt >= LEFT_DELAY_TIME)
            {
                Left_DelayCnt = 0;
                Left_Step++;
                if (Left_Step > 4) Left_Step = 1;
            }

            uint8_t idx = (Left_Step >= 1 && Left_Step <= 4) ? (Left_Step - 1) : 0;
            InterpolateTo(&Angle1, Left_Targets[idx][0], LEFT_STEP_DELTA);
            InterpolateTo(&Angle2, Left_Targets[idx][1], LEFT_STEP_DELTA);
            InterpolateTo(&Angle3, Left_Targets[idx][2], LEFT_STEP_DELTA);
            InterpolateTo(&Angle4, Left_Targets[idx][3], LEFT_STEP_DELTA);
            InterpolateTo(&Angle5, Left_Targets[idx][4], LEFT_STEP_DELTA);
            InterpolateTo(&Angle6, Left_Targets[idx][5], LEFT_STEP_DELTA);
            InterpolateTo(&Angle7, Left_Targets[idx][6], LEFT_STEP_DELTA);
            InterpolateTo(&Angle8, Left_Targets[idx][7], LEFT_STEP_DELTA);
        }

        /* 右转：与左转对称，两组对角腿的抬起顺序对调。 */
        /************************ 右转动作执行（插值版）************************/
        if (Right_State != 0)
        {
            Right_DelayCnt++;
            if (Right_DelayCnt >= RIGHT_DELAY_TIME)
            {
                Right_DelayCnt = 0;
                Right_Step++;
                if (Right_Step > 4) Right_Step = 1;
            }

            uint8_t idx = (Right_Step >= 1 && Right_Step <= 4) ? (Right_Step - 1) : 0;
            InterpolateTo(&Angle1, Right_Targets[idx][0], RIGHT_STEP_DELTA);
            InterpolateTo(&Angle2, Right_Targets[idx][1], RIGHT_STEP_DELTA);
            InterpolateTo(&Angle3, Right_Targets[idx][2], RIGHT_STEP_DELTA);
            InterpolateTo(&Angle4, Right_Targets[idx][3], RIGHT_STEP_DELTA);
            InterpolateTo(&Angle5, Right_Targets[idx][4], RIGHT_STEP_DELTA);
            InterpolateTo(&Angle6, Right_Targets[idx][5], RIGHT_STEP_DELTA);
            InterpolateTo(&Angle7, Right_Targets[idx][6], RIGHT_STEP_DELTA);
            InterpolateTo(&Angle8, Right_Targets[idx][7], RIGHT_STEP_DELTA);
        }

        /* ---------- ⑤-1 状态灯"心跳" ----------
         * LED_State 在 1 和 2 之间来回切，每 12 个 tick（约 300ms）
         * 交换一次 PA12 / PA15 的亮灭 → 两颗灯交替闪，表示程序还活着。
         * 如果上电后灯不闪，先怀疑程序卡死或没跑起来，而不是舵机。
         */
        /************************ LED闪烁控制 ************************/
        if (LED_State != 0)
        {
            LED_DelayCnt++;
            if (LED_DelayCnt >= LED_DELAY_TIME)
            {
                LED_DelayCnt = 0;
                if (LED_State == 1)
                {
                    GPIO_ResetBits(GPIOA, GPIO_Pin_12);
                    GPIO_SetBits(GPIOA, GPIO_Pin_15);
                    LED_State = 2;
                }
                else
                {
                    GPIO_SetBits(GPIOA, GPIO_Pin_12);
                    GPIO_ResetBits(GPIOA, GPIO_Pin_15);
                    LED_State = 1;
                }
            }
        }

        /************************ (离地检测)PA5滑动动作控制 ************************/
//         if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5) == 1)
//         {
//             if (Huadong_State == 0)
//             {
//                 Forward_State = 0;
//                 Back_State = 0;
//                 Left_State = 0;
//                 Right_State = 0;
//                 Huadong_State = 1;
//                 Huadong_Step = 0;
//                 step_delay = 0;
//             }
//             huadong_release_flag = 1;
//         }
//         else
//         {
//             if (huadong_release_flag == 1)
//             {
//                 // 恢复站立姿态
//                 Angle1 = 135.0f; Angle2 = 45.0f; Angle3 = 45.0f; Angle4 = 135.0f;
//                 Angle5 = 0.0f;   Angle6 = 180.0f; Angle7 = 0.0f; Angle8 = 180.0f;
//
//                 Servo_SetAngle1(Angle1); Servo_SetAngle2(Angle2);
//                 Servo_SetAngle3(Angle3); Servo_SetAngle4(Angle4);
//                 Servo_SetAngle5(Angle5); Servo_SetAngle6(Angle6);
//                 Servo_SetAngle7(Angle7); Servo_SetAngle8(Angle8);
//
//                 huadong_release_flag = 0;
//             }
//             Huadong_State = 0;
//         }
//
//         // 执行滑动动作
//         if (Huadong_State != 0)
//         {
//             Huadong_Smooth();
//         }

        /* ---------- ⑤-2 蜂鸣器自动关闭 ----------
         * 收到 "BOO" 指令时会执行 BOO_ON() 并令 booOffCnt = 16。
         * 之后每圈减 1，减到 0 就 BOO_OFF()。
         * 16 个 tick × 25ms ≈ 400ms，所以蜂鸣器只会"滴"一声。
         * 这也是一种非阻塞定时：没有用 Delay 阻塞主循环。 */
        if (booOffCnt > 0)
        {
            booOffCnt--;
            if (booOffCnt == 0) BOO_OFF();
        }

        // 串口接收状态机超时复位: '@' 收到后 200ms 内没等到 '#*' 就强制清零,
        // 避免蓝牙断包/噪声把整机锁在"接收中"状态导致后续指令全部失效。
        Serial_TickReset();

        /* ---------- ⑥ 节拍控制：主循环的"心跳间隔" ----------
         * 所有 *_DelayCnt 都以这一句为节拍。整个循环（含前面的计算和
         * 舵机刷新）实际略多于 25ms，所以真实节拍约为 25ms 多一点，
         * 步态速度是"大概值"，不需要精确到毫秒。
         *
         * 千万别删这一句：没有它，循环会以微秒级速度空转，
         * Step 几乎瞬间走完 4 步，舵机会被命令瞬间跳角度 →
         * 峰值电流暴涨 → 电池电压塌陷 → 整机抽搐/复位。 */
        // 主循环延时25ms: 所有 *_DelayCnt 都以此为节拍, 缺了这一句会导致步态以微秒级切换,
        // 舵机被命令瞬移 → 峰值电流暴涨 → 电压塌陷 → 抽搐
        Delay_ms(25);
    }
}
