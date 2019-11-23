#include <PID_v1.h>


// MOTOR pins
int enA = 10;  // Motor enable, A
int in1 = 9;  // Motor A PWM 1
int in2 = 8;  // Motor A PWM 2

int enB = 5;  // Motor enable, B
int in3 = 4;  // Motor B PWM 1
int in4 = 3;  // Motor B PWM 2

// PID variables
double Setpoint, Input, Output;
double k_P = 0.5;
double k_I = 0;
double k_D = 0.095;  // double Kp=0.845, Ki=0, Kd=0.143;
//PID mrPID(&Input, &Output, &Setpoint, k_P, k_I, k_D, DIRECT);

// PHOTODIODE pins
int sensor_pin0 = 0;
int sensor_pin1 = 1;
int sensor_pin2 = 3;
int sensor_pin3 = 2;




void setup() {
  
  // MOTORS
  set_up_motors(0);

  // PID
  Setpoint = 0;
//  myPID.SetMode(AUTOMATIC);  // turn PID on
//  myPID.SetOutputLimits(-1, 1);

Serial.begin(115200);
Serial.println("sensor_pin0, sensor_pin1, sensor_pin2, sensor_pin3");
  
}

// pid output drives a ratio of two motor speeds



void loop() {

  // poll offset of photodiodes
  float offset_photodiodes12 = get_photodiode_difference(sensor_pin1, sensor_pin2);  // diff = pin1 - pin2

  Input = offset_photodiodes12;

  Serial.print(analogRead(sensor_pin0)); Serial.print(",");
  Serial.print(analogRead(sensor_pin1)); Serial.print(",");
  Serial.print(analogRead(sensor_pin2)); Serial.print(",");
  Serial.print(analogRead(sensor_pin3)); Serial.print(",");
  Serial.println("");
  
  // feed offset to PID input
  // compute new PID value
  // feed PID output to motors (motor Left/Right ratio)
  // apply motor ratio to both motors (0-255 for L and R)
}



float get_photodiode_difference(int sensor_pin1, int sensor_pin2) {
  float val_sensor1 = analogRead(sensor_pin1);
  float val_sensor2 = analogRead(sensor_pin2);
  float diff = val_sensor1 - val_sensor2;
  
  return diff;
}




void set_up_motors(int pwm_speed) {

  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

    // LEFT
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(enA, pwm_speed);  // 0 - 255 PWM

    // LEFT
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enB, pwm_speed);  // 0 - 255 PWM
  
}



void set_up_photodiodes() {

  pinMode(sensor_pin0, INPUT);
  pinMode(sensor_pin1, INPUT);
  pinMode(sensor_pin2, INPUT);
  pinMode(sensor_pin3, INPUT);

}
