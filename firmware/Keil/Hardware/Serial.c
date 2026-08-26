#include "stm32f10x.h"                  // Device header
#include "Serial.h"
#include <stdio.h>
#include <stdarg.h>

char Serial_RxPacket[SERIAL_RX_PACKET_SIZE];
volatile uint8_t Serial_RxFlag;

/* 接收状态机的内部状态。
 * 以前是 USART1_IRQHandler 里的 static 局部变量；提到文件作用域是为了
 * 让 Serial_TickReset() 也能在超时后强制复位它们。
 *   RxState     0: 等 '@'；1: 收正文（直到 '#'）；2: 收到 '#'，等 '*'
 *   pRxPacket   下一个写入下标（同时也是当前已收字节数）
 *   rxStaleTicks 主循环 tick 数计数器，仅 RxState != 0 时累加，超阈值就复位 */
static volatile uint8_t  RxState       = 0;
static volatile uint8_t  pRxPacket     = 0;
static volatile uint16_t rxStaleTicks  = 0;

/* 200ms / 25ms-per-tick = 8 ticks 后认为接收态卡死 */
#define RX_STALE_TIMEOUT_TICKS  8

void Serial_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 9600;//这里的波特率需要与你的蓝牙模块一致!
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART1, &USART_InitStructure);

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);

	USART_Cmd(USART1, ENABLE);
}

void Serial_SendByte(uint8_t Byte)
{
	USART_SendData(USART1, Byte);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
}

void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
	uint16_t i;
	for (i = 0; i < Length; i ++)
	{
		Serial_SendByte(Array[i]);
	}
}

void Serial_SendString(char *String)
{
	/* 计数器从 uint8_t 提到 uint16_t: 避开 255 字符隐性上限。
	 * 老实现传入超过 255 字符的字符串会在 i 回绕到 0 之后死循环重发开头,
	 * 直到末尾 '\0' 凑巧落在前 256 字节内才退出。当前调用都很短(<32 字符),
	 * 但 Serial_Printf 内部缓冲是 100B, 一旦改大就会立刻踩, 提前堵。 */
	uint16_t i;
	for (i = 0; String[i] != '\0'; i ++)
	{
		Serial_SendByte(String[i]);
	}
}

uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y --)
	{
		Result *= X;
	}
	return Result;
}

void Serial_SendNumber(uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i ++)
	{
		Serial_SendByte(Number / Serial_Pow(10, Length - i - 1) % 10 + '0');
	}
}

int fputc(int ch, FILE *f)
{
	Serial_SendByte(ch);
	return ch;
}

void Serial_Printf(char *format, ...)
{
	char String[100];
	va_list arg;
	va_start(arg, format);
	/* 用 vsnprintf 而非 vsprintf：上限 sizeof(String)-1, 保留末尾 '\0'。
	 * vsprintf 无界, 一旦格式化结果超 100B 就会越界覆盖栈上返回地址 →
	 * HardFault → 整机重启。当前调用都不长但属于"等一次踩一次"的雷, 提前堵。*/
	vsnprintf(String, sizeof(String), format, arg);
	String[sizeof(String) - 1] = '\0';   // 显式兜底, 即便实现不规范也确保终止
	va_end(arg);
	Serial_SendString(String);
}

void Serial_TickReset(void)
{
	/* 把"检查 + 计数 + 复位"整段放在关中断区:
	 * 老实现先判 RxState != 0 再 disable_irq, 中间 ISR 可能把 RxState 改成 1
	 * (新的 '@' 刚到), 然后我们紧接着把它清回 0, 丢掉一个本来有效的包。
	 * 整段都关 IRQ 后, ISR 要么在我们开始前完成, 要么等我们结束再跑, 不再有竞态。
	 * 关中断时长仅几条指令, 在 72MHz 下 < 1us, 不会影响 9600 波特的接收。 */
	__disable_irq();
	if (RxState != 0)
	{
		if (++rxStaleTicks >= RX_STALE_TIMEOUT_TICKS)
		{
			RxState = 0;
			pRxPacket = 0;
			rxStaleTicks = 0;
		}
	}
	else
	{
		rxStaleTicks = 0;
	}
	__enable_irq();
}

void USART1_IRQHandler(void)
{
	/* 先处理 RX 错误标志 (ORE / FE / NE / PE):
	 *   这四个错误位与 RXNE 共享同一中断源, 一旦置位且不清除, 后续行为是:
	 *     1) ISR 持续触发(本函数不断进入), 但走不进 RXNE 分支什么也不做
	 *        → CPU 被 IRQ 死循环吃满
	 *     2) RXNE 永远不再被硬件置位 → 蓝牙整体失效, 只能重启 MCU
	 *   触发场景: WS2812 那串 Delay_us(60000~80000) 累计阻塞, 期间 9600 波特的
	 *   蓝牙仍在收数, RDR 没人读就会 ORE; 电气噪声也会触发 FE/NE。
	 *   STM32F1 清除流程: 读 SR 再读 DR (GetFlagStatus 完成读 SR, ReceiveData 读 DR)。 */
	if (USART_GetFlagStatus(USART1, USART_FLAG_ORE) == SET ||
		USART_GetFlagStatus(USART1, USART_FLAG_FE)  == SET ||
		USART_GetFlagStatus(USART1, USART_FLAG_NE)  == SET ||
		USART_GetFlagStatus(USART1, USART_FLAG_PE)  == SET)
	{
		(void)USART_ReceiveData(USART1);   // 读 DR, 完成 SR→DR 清除序列
		/* 既然发生过收包错误, 进行中的半截包必然已损坏, 整段丢弃,
		 * 把状态机带回 idle, 等下一个 '@' 重新启动. */
		RxState = 0;
		pRxPacket = 0;
		rxStaleTicks = 0;
		return;
	}

	if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
	{
		uint8_t RxData = USART_ReceiveData(USART1);

		if (RxState == 0)
		{
			if (RxData == '@' && Serial_RxFlag == 0)
			{
				RxState = 1;
				pRxPacket = 0;
				rxStaleTicks = 0;
			}
		}
		else if (RxState == 1)
		{
			if (RxData == '#')
			{
				RxState = 2;
			}
			else
			{
				/* 边界保护：留一格给末尾 '\0'。多余字节直接丢弃，
				 * 状态机继续等 '#*'，不会卡死也不会冲爆 RxPacket。 */
				if (pRxPacket < SERIAL_RX_PACKET_SIZE - 1)
				{
					Serial_RxPacket[pRxPacket] = RxData;
					pRxPacket ++;
				}
			}
			rxStaleTicks = 0;
		}
		else if (RxState == 2)
		{
			if (RxData == '*')
			{
				RxState = 0;
				Serial_RxPacket[pRxPacket] = '\0';
				Serial_RxFlag = 1;
				rxStaleTicks = 0;
			}
			else
			{
				/* 收到 '#' 后没拿到 '*': 这不是合法尾, 协议层视为错包。
				 * 老实现会在此处沉默地停在 state 2, 等下一个 '*' 或 200ms 超时,
				 * 期间任何字节都被丢, 浪费一个完整 tick。
				 * 改成立即回到 idle, 把这次接收丢弃, 等待下一个 '@' 起头。*/
				RxState = 0;
				pRxPacket = 0;
				rxStaleTicks = 0;
			}
		}

		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}
}
