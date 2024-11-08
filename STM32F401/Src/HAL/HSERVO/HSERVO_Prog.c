/*******************************************************************************************************/
/* Author            : Amr ElMaghraby                                                                  */
/* Version           : V1.0.3                                                                          */
/* Data              : 28 Dec 2023                                                                     */
/* Description       : HSERVO_Prog.c --> implementations                                          */
/* Module  Features  :                                                                                 */
/*      01- HSERVO_vServoInit                                                                          */
/*      02- HSERVO_vServoDeg                                                                           */
/*******************************************************************************************************/
// Include necessary MCU Abstraction Layer (MCAL) headers
#include "../../../Src/HAL/HSERVO/HSERVO_Config.h"
#include "../../../Src/HAL/HSERVO/HSERVO_Int.h"
#include "../../MCAL/MRCC/MRCC_Int.h"    // RCC (Reset and Clock Control)
#include "../../MCAL/MGPIO/MGPIO_int.h"  // GPIO (General Purpose Input/Output)
#include "../../MCAL/MTIMER/MTIMER_Int.h" // TIMER (Timer Control)

// Include HSERVO configuration and interface headers
// Define a structure to store servo information
typedef struct {
	Enum_TIMER_NUM TIMER;    // Timer number associated with the servo
	Enum_TIMER_CHs CHANNEL;  // Timer channel associated with the servo
} LOC_SERVO_NUM;

// Declare an array to store servo information for multiple servos
LOC_SERVO_NUM SERVO_STRUCT[TOT_SERVO_NUM];


void HSERVO_vServoInit(Enum_SERVO_NUM Copy_u8ServoNum,Enum_TIMER_NUM Copy_u8TimerNum,u8 Copy_u8ChannelNum){
	// Update the TIMER and CHANNEL configuration in the servo structure
	SERVO_STRUCT[Copy_u8ServoNum].TIMER = Copy_u8TimerNum;
	SERVO_STRUCT[Copy_u8ServoNum].CHANNEL = Copy_u8ChannelNum;

}

/**
 * @brief Sets the angle of the servo motor specified by Copy_u8ServoNum to the desired degree.
 *
 * @param Copy_u8ServoNum: Enum_SERVO_NUM representing the servo number.
 * @param Copy_s8Deg: Desired angle in degrees (from -45 to 45).
 */
void HSERVO_vServoDeg(Enum_SERVO_NUM Copy_u8ServoNum, s16 Copy_s8Deg) {
	// Ensure the angle is within the valid angle range (-45 to 45)
	if (Copy_s8Deg > 60) {
		Copy_s8Deg = 60 ;
	}else if (Copy_s8Deg < -45) {
		Copy_s8Deg = -45 ;
	}
	// Add Zero Position to the angle
	Copy_s8Deg += ZERO_POS;

	// Calculate the positive duty cycle corresponding to the desired angle
	u16 Positive_Duty = ((Copy_s8Deg * 2000) / 180) + 500;

	// Set the PWM signal to control the servo motor
	MTIMER_vPWM(SERVO_STRUCT[Copy_u8ServoNum].TIMER, SERVO_STRUCT[Copy_u8ServoNum].CHANNEL,
				SERVO_FullPeriod , Positive_Duty );

}




