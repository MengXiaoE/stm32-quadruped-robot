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


/* Serial_RxPacket / Serial_RxFlag 的 extern 声明统一由 Serial.h 提供
 * (含 volatile 与 SERIAL_RX_PACKET_SIZE 宏), 本文件不再重复 extern,
 * 防止与 Serial.h 的声明出现 size/volatile 不一致. */

volatile uint8_t MAGE_State = 0;               // 模式状态标志

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
 *    经验值: 6=600ms周期(稳)、5=500ms周期(快, 当前)、4=400ms周期(极限)。
 *
 * ② FORWARD_STEP_DELTA  —— 单 tick 角度增量
 *    类型: float, 单位: 度/tick
 *    含义: 每个 tick 舵机最多前进多少度。必须保证
 *          FORWARD_STEP_DELTA × FORWARD_DELAY_TIME ≥ 60
 *          (60° = 胯前位↔后位差), 否则一步走不完整个推程。
 *    范围: 6.0 ~ 15.0; > 15 接近 MG90 极限 (15°/25ms), 会卡顿。
 *    经验值: DELAY=6→DELTA=10、DELAY=5→DELTA=12 (当前)、DELAY=4→DELTA=15。
 *
 * ③ FORWARD_LIFT  —— 抬腿幅度(小腿离地高度)
 *    类型: float, 单位: 度
 *    含义: 摆动相小腿从直立位(A5/A7=0、A6/A8=180)抬起多少度。
 *    范围: 20 ~ 60; 过小→擦地, 过大→重心起伏剧烈、整机点头。
 *    经验值: 45 (抗擦地+不晃的折中点)。地面光滑可降到 30。
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
const uint16_t FORWARD_DELAY_TIME  = 5;       // ① 节拍数 (5×25ms = 125ms/步, 500ms/周期)
const float    FORWARD_STEP_DELTA  = 12.0f;   // ② 度/tick (12×5 = 60° 刚好走完推程)
const float    FORWARD_LIFT        = 45.0f;   // ③ 抬腿幅度
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

// 把 *current 向 target 推进, 单步最多移动 step 度
static void InterpolateTo(volatile float *current, float target, float step)
{
    float diff = target - *current;
    if (diff > step)       *current += step;
    else if (diff < -step) *current -= step;
    else                   *current  = target;
}

int main(void)
{
    // 外设初始化
    Servo_Init();
    Serial_Init();
    Input_Init();
    WS2812B_Init();
    BOO_Init();
    LED_Init();
    LED_StartBlink();

    // 系统初始复位
    ResetAll();

    while (1)
    {
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

        if (booOffCnt > 0)
        {
            booOffCnt--;
            if (booOffCnt == 0) BOO_OFF();
        }

        // 串口接收状态机超时复位: '@' 收到后 200ms 内没等到 '#*' 就强制清零,
        // 避免蓝牙断包/噪声把整机锁在"接收中"状态导致后续指令全部失效。
        Serial_TickReset();

        // 主循环延时25ms: 所有 *_DelayCnt 都以此为节拍, 缺了这一句会导致步态以微秒级切换,
        // 舵机被命令瞬移 → 峰值电流暴涨 → 电压塌陷 → 抽搐
        Delay_ms(25);
    }
}
