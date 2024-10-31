#include "../HAL/HSERVO/HSERVO_Int.h"
#include "../../Src/App/Balancing_Ball.h"

#include "../../Src/HAL/HSERVO/HSERVO_Int.h"
#include "../../Src/HAL/HULTRA_SONIC/HULTRA_SONIC_Int.h"

#include "../MCAL/MTIMER/MTIMER_Int.h"
f64 distance, prevDistance, servoAngle, prevServoAngle, error, prevError, dError, prev_dError, P, I, D, Kp, Ki, Kd, dt ;
u8 	callBalance = 0 ;

f64 low_pass_filter(f64 input, f64 prev_output, f64 alpha) {
	// y[n]=α⋅x[n]+(1−α)⋅y[n−1]
	float outPut =  alpha * input + (1 - alpha) * prev_output;
	return outPut ;
}

void balance (void){

	HULTRA_vSendTrigger(PORTB, PIN12);
	HULTRA_vGetDistance(ULTRA_SONIC1, &distance);

	// To handle the error of the ultra_sonic
	if ( distance > 32 || distance < 0 ) {
		distance = prevDistance ;
	}

	// Get the error
	error = distance - REFRENCE_POS ;
	dError = error - prevError ;

	P = error ;
	I += error * dt ;
	dError = low_pass_filter(dError, prev_dError, 0.2) ;
	D = (dError) / dt ;

	servoAngle = Kp * P + Ki * I + Kd * D;
	servoAngle = low_pass_filter(servoAngle, prevServoAngle, 0.3);

	HSERVO_vServoDeg(SERVO1 , servoAngle);

	// Clamping saturation error
	// Check if the output saturated and the error and servo angle is the same sign
	if (((u8)prevServoAngle == (u8)servoAngle) && (servoAngle/distance > 0) && (servoAngle > 60 ||servoAngle<-45) )
		I = 0 ;  // Disable the integrator

	prevError = error ;
	prev_dError = dError ;
	prevDistance = distance ;
	prevServoAngle = servoAngle ;
}

void set_call_balance_flag(){
	callBalance = 1 ;
}

void set_pid_parameters (f64 copy_Kp, f64 copy_Ki, f64 copy_Kd, f64 copy_dt_ms ){
	Kp = copy_Kp ;
	Ki = copy_Ki ;
	Kd = copy_Kd ;
	dt = copy_dt_ms/1000 ;

	MTIMER_vClearCNT(TIMER2);
	MTIMER_CallBack(TIMER2,set_call_balance_flag );
	MTIMER_vPeriodicMS(TIMER2,copy_dt_ms);
}


void Balance_Ball(){
	if(callBalance)
		balance();

	callBalance = 0 ;
}
