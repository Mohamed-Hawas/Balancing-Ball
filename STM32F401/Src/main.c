/*
 * @Author : Mohamed Hawas
 * @date : 7/10/2024
 */

#include "LIB/STD_TYPES.h"
#include "LIB/BIT_MATH.h"
#include "math.h"

#include "../Src/App/Balancing_Ball.h"
#include "../Src/HAL/HSERVO/HSERVO_Int.h"
#include "../Src/HAL/HULTRA_SONIC/HULTRA_SONIC_Int.h"
#include "MCAL/MRCC/MRCC_Int.h"
#include "MCAL/MFPU/MFPU_Int.h"
#include "MCAL/MSTK/MSYSTICK_Int.h"
#include "MCAL/MGPIO/MGPIO_int.h"
#include "MCAL/MTIMER/MTIMER_Int.h"
#include "MCAL/MNVIC/MNVIC_int.h"
#include "MCAL/MEXTI/MEXTI_Int.h"
#include "MCAL/MIWDT/MIWDT_int.h"
#include "MCAL/MSPI/MSPI_int.h"
#include "MCAL/MI2C/MI2C_int.h"
#include "MCAL/MUART/MUSART_Interface.h"


void System_Clock_Init();
void Interrupt_Init();
void HAL_Init();
int main(void)
{
	MFPU_Enable();
	System_Clock_Init();
	Interrupt_Init();
	HAL_Init();
	//*************************test*******************************
	MGPIO_vSetPinMode(PORTC, PIN13, OUTPUT);
	MGPIO_vSetPinValue(PORTC, PIN13, LOW);
	//*************************run key******************************************
	MGPIO_vSetPinMode(PORTA, PIN0, INPUT);
	MGPIO_vSetPinInPutType(PORTA, PIN0, PULLUP);

	do{
	}while( MGPIO_u8GetPinValue(PORTA, PIN0) ) ;

	set_pid_parameters(3, 1, 1.5, 50);

	while(1){
		Balance_Ball();
	}
}


void System_Clock_Init(){
	MRCC_vInit();
	MRCC_vEnableClock(GPIOA_EN);
	MRCC_vEnableClock(GPIOB_EN);
	MRCC_vEnableClock(GPIOC_EN);
	MRCC_vEnableClock(SYSCFG_EN);
	MRCC_vEnableClock(USART1_EN);
	MRCC_vEnableClock(USART2_EN);
	MRCC_vEnableClock(USART6_EN);
	MRCC_vEnableClock(TIM1_EN);
	MRCC_vEnableClock(TIM2_EN);
	MRCC_vEnableClock(TIM3_EN);
	MRCC_vEnableClock(TIM4_EN);
	MRCC_vEnableClock(TIM5_EN);
	MRCC_vEnableClock(TIM9_EN);
	MRCC_vEnableClock(TIM10_EN);
	MRCC_vEnableClock(TIM11_EN);
	MRCC_vEnableClock(I2C1_EN);
}
void Interrupt_Init(){
	MNVIC_vEnableInterrupt(NVIC_TIM1_CC);
	MNVIC_vEnableInterrupt(NVIC_TIM1_UP_TIM10);
	MNVIC_vEnableInterrupt(NVIC_TIM2);
	MNVIC_vEnableInterrupt(NVIC_TIM3);
	MNVIC_vEnableInterrupt(NVIC_TIM4);
	MNVIC_vEnableInterrupt(NVIC_TIM1_BRK_TIM9);
	MNVIC_vEnableInterrupt(NVIC_TIM5);
	MNVIC_vInitGrouping(GP16SUB0);
	//MNVIC_vSetIntPriority(NVIC_TIM1_UP_TIM10, GP15, SUB0);
	//MNVIC_vSetIntPriority(NVIC_TIM3, GP1, SUB0);
	//MNVIC_vSetIntPriority(NVIC_EXTI9_5, GP9, SUB0);
	MNVIC_vEnableInterrupt(NVIC_EXTI1);
	MEXTI_vEnableInterrupt(EXTI1);

}

void HAL_Init(){

	HSERVO_vServoInit(SERVO1, TIMER3, CH1); //  TIMER3, CH1 -> Port A, Pin 6
	HULTRA_vInitialize(ULTRA_SONIC1, TIMER1, CH1); // TIMER1, CH1 -> Port A, Pin 8

}
