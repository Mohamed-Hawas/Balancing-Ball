#include <ESP32Servo.h>
#include <stdint.h>

Servo myServo;  // Create a Servo object

#define TRIG_PIN 12 
#define ECHO_PIN 14 
#define SERVO_PIN 19 
#define REFRENCE_POS 15 
#define SERVO_ZERO_REF 75 

float distance, prevDistance, servoAngle, prevServoAngle, error, prevError, dError, prev_dError, P, I, D, Kp, Ki, Kd, dt ;

void set_pid_parameters (float copy_Kp, float copy_Ki, float copy_Kd, float copy_dt_ms ){
	Kp = copy_Kp ;
	Ki = copy_Ki ;
	Kd = copy_Kd ;
	dt = copy_dt_ms/1000 ;
}

float low_pass_filter(float input, float prev_output, float alpha) {
	// y[n]=α⋅x[n]+(1−α)⋅y[n−1]
	float outPut =  alpha * input + (1 - alpha) * prev_output;
  return outPut ;
}


void servo(int angle){  
   
  if (angle > 60) {
		angle = 60 ;
	}else if (angle < -45) {
		angle = -45 ;
	}
  angle += SERVO_ZERO_REF ;
  myServo.write(angle);
}

float get_distance() {
    float duration, distance;

    // Send a 10us pulse to trigger the sensor
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    // Measure the time for the echo to be received
    duration = pulseIn(ECHO_PIN, HIGH);

    // Calculate distance in cm
    distance = duration * 0.034 / 2;

    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
  return distance ;
}

void balance (void){

  distance = get_distance();

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
	servo(servoAngle);

	// Clamping saturation error
	// Check if the output saturated & the error and servo angle is the same sign
	if (((uint8_t)prevServoAngle == (uint8_t)servoAngle) && (servoAngle/distance > 0) && (servoAngle > 60 ||servoAngle<-45) )
		I = 0 ;  // Disable the integrator

	prevError = error ;
  prev_dError = dError ;
	prevDistance = distance ;
	prevServoAngle = servoAngle ;
}

void setup() {
  myServo.attach(SERVO_PIN);  
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  Serial.begin(115200); 
}

void loop() {
  set_pid_parameters(3, 1, 1.5, 50);
  balance();
  delay(dt*1000);  

}

