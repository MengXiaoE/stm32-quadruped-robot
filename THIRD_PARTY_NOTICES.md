# 第三方内容声明

本文件用于区分作者原创内容与随工程保留的第三方代码。除明确写明的作者原创文件外，不应把第三方代码、字库或应用理解为由作者重新授予 MIT 许可。

## 原项目署名

本仓库基于“我的STM 32又烧了”的四足机器人项目进行优化与开源整理。原项目贡献归原作者所有，后续优化归相应贡献者所有；MIT 仅适用于相应权利人有权授权的内容。

## STMicroelectronics STM32F10x Standard Peripheral Library

`firmware/Keil/Library/` 中的 `stm32f10x_*.c/.h` 和 `misc.*` 基于 STMicroelectronics STM32F10x Standard Peripheral Library V3.5.0。文件内保留 ST 的原始版权和免责声明。使用、再分发和修改应遵循随源码提供的 ST 许可/免责声明及适用的 ST 许可条款。

## ARM CMSIS Cortex-M3

`firmware/Keil/Start/core_cm3.c` 与 `core_cm3.h` 含 ARM Limited 的版权声明和原始分发条件。请保留原文件版权头，并遵守 ARM/CMSIS 的适用条款。

## 启动文件与 STM32 设备头文件

`firmware/Keil/Start/` 中的 STM32 设备头文件、系统文件和启动汇编文件保留原始供应商版权头。它们不是作者原创内容；本仓库仅将其作为构建 STM32 工程所需的第三方依赖一并提供。

## 未随本包分发的 OLED 模板

原始资料中包含带有第三方版权标识的 OLED 模板和头像/平台图标。本次清洁发布包不提交 `OLED.c/.h`、`OLED_Data.c/.h`，工程也不引用它们。若后续重新加入，必须先核实其可再分发许可并补充相应声明。

## 未随本包分发的手机 APK

`Arduino bluetooth controller` 是第三方 Android 应用，不属于本项目；本仓库不托管 APK，不对其版权、签名或再分发作任何授权或保证。
