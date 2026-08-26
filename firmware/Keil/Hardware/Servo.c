#include "stm32f10x.h"                  // Device header
#include "PWM.h"


/* 角度硬钳位：所有 Servo_SetAngleN 都走这里再算 PWM。
 * 防止上游传入越界（负数 / >180）时 (Angle/180*2000+500) 被截成 uint16_t
 * 后产生数万微秒的脉宽 → 舵机堵转 → 电流冲击 → 锂电压塌陷 → 整机重启。
 * MG90 机械可走 0~180°，超出范围一律收回边界。 */
static float clamp180(float a)
{
	if (a < 0.0f)   return 0.0f;
	if (a > 180.0f) return 180.0f;
	return a;
}


void Servo_Init(void)
{
	PWM_Init();					//TIM2
	PWM_Init_TIM3(); //TIM3
	PWM4_Init();	//TIM4
}


void Servo_SetAngle1(float Angle1)
{
	PWM_SetCompare1(clamp180(Angle1) / 180 * 2000 + 500);	//设置占空比

}

void Servo_SetAngle2(float Angle2)
{

	PWM_SetCompare2(clamp180(Angle2) / 180 * 2000 + 500);

}
void Servo_SetAngle3(float Angle3)
{
	PWM_SetCompare3(clamp180(Angle3) / 180 * 2000 + 500);
}

void Servo_SetAngle4(float Angle4)
{
	PWM_SetCompare4(clamp180(Angle4) / 180 * 2000 + 500);
}

void Servo_SetAngle5(float Angle5)
{
	PWM_SetCompare1_TIM3(clamp180(Angle5) / 180 * 2000 + 500);
}
void Servo_SetAngle6(float Angle6)
{
	PWM_SetCompare2_TIM3(clamp180(Angle6) / 180 * 2000 + 500);
}
void Servo_SetAngle7(float Angle7)
{
	PWM_SetCompare3_TIM3(clamp180(Angle7) / 180 * 2000 + 500);
}
void Servo_SetAngle8(float Angle8)
{
	PWM_SetCompare4_TIM3(clamp180(Angle8) / 180 * 2000 + 500);
}
void Servo_SetAngle9(float Angle9)
{
	PWM_SetCompare1_TIM1(clamp180(Angle9) / 180 * 2000 + 500);
}
