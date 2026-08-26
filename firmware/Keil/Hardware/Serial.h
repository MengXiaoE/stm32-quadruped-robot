#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdio.h>
#include <stdint.h>

/* 串口接收缓冲区容量（含末尾 '\0'）。修改这里，main.c 和 Serial.c 同时跟上。*/
#define SERIAL_RX_PACKET_SIZE  100

/* RxPacket 本身不加 volatile：
 *   ISR 仅在 Serial_RxFlag==0 时写入；main 仅在 Serial_RxFlag==1 时读取。
 *   Serial_RxFlag 是 volatile，已构成同步点，buffer 不存在并发访问。
 *   若给 RxPacket 加 volatile，调用 strtof/strncasecmp/memset 会丢限定符报错。*/
extern char Serial_RxPacket[];
extern volatile uint8_t Serial_RxFlag;

void Serial_Init(void);
void Serial_SendByte(uint8_t Byte);
void Serial_SendArray(uint8_t *Array, uint16_t Length);
void Serial_SendString(char *String);
void Serial_SendNumber(uint32_t Number, uint8_t Length);
void Serial_Printf(char *format, ...);

/* 主循环每个 tick(25ms) 调一次。
 * 若接收状态机收到 '@' 后 200ms 内没等到 '#*' 结束符，强制复位接收状态机，
 * 避免蓝牙断包/噪声把整机锁在"接收中"导致所有指令失效。 */
void Serial_TickReset(void);

#endif
