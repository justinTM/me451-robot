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
PID myPID(&Input, &Output, &Setpoint, k_P, k_I, k_D, DIRECT);

int limit = 255;
float gain = 50.0;

// PHOTODIODE pins
int sensor_pin0 = 0;
int sensor_pin1 = 1;
int sensor_pin2 = 2;
int sensor_pin3 = 3;

// calibrating pins
int i = 0;  // count current number of measurements
bool isCalibrated = false;  // stop counting after calibration

// moving average window filter stuff
float averages[4];  // each element is the average of a sensor array
unsigned long int iMovingAverage = 0;
const float windowSize = 10.0;
const int intWindowSize = (int) windowSize;
int values_s0[intWindowSize];
int values_s1[intWindowSize];
int values_s2[intWindowSize];
int values_s3[intWindowSize];

float avg_s0 = 0;
float avg_s1 = 0;
float avg_s2 = 0;
float avg_s3 = 0;

float sum_s0 = 0;
float sum_s1 = 0;
float sum_s2 = 0;
float sum_s3 = 0;


void setup() {

  // PID
  Setpoint = 0;
  myPID.SetMode(AUTOMATIC);  // turn PID on
  myPID.SetOutputLimits(-limit, limit);

  Serial.begin(9600);

  for (int i=0; i < intWindowSize; i++) {
    values_s0[i] = 0;
    values_s1[i] = 0;
    values_s2[i] = 0;
    values_s3[i] = 0;
  }
  
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

//  Serial.println("sensor_pin0, sensor_pin1, sensor_pin2, sensor_pin3, PID_IN, PID_OUT");
  Serial.println("PID_OUT");
}

// pid output drives a ratio of two motor speeds



void loop() {
  delay(10);
  Input = (avg_s0 + avg_s1) - (avg_s2 + avg_s3);
//  Serial.println(avg_s0, avg_s1, avg_s2, avg_s3, Input);

  myPID.Compute();

//  run_motors(Output);
  run_motors(200);

  sum_s0 = sum_s0 - values_s0[iMovingAverage];
  sum_s1 = sum_s1 - values_s1[iMovingAverage];
  sum_s2 = sum_s2 - values_s2[iMovingAverage];
  sum_s3 = sum_s3 - values_s3[iMovingAverage];

  values_s0[iMovingAverage] = analogRead(sensor_pin0);
  values_s1[iMovingAverage] = analogRead(sensor_pin1);
  values_s2[iMovingAverage] = analogRead(sensor_pin2);
  values_s3[iMovingAverage] = analogRead(sensor_pin3);

  sum_s0 = sum_s0 + values_s0[iMovingAverage];
  sum_s1 = sum_s1 + values_s1[iMovingAverage];
  sum_s2 = sum_s2 + values_s2[iMovingAverage];
  sum_s3 = sum_s3 + values_s3[iMovingAverage];


  iMovingAverage++;

  if (iMovingAverage >= intWindowSize) {
    iMovingAverage = 0; // reset the moving average back to beginning
  }

  avg_s0 = gain*(sum_s0 / windowSize);
  avg_s1 = gain*(sum_s1 / windowSize);
  avg_s2 = gain*(sum_s2 / windowSize);
  avg_s3 = gain*(sum_s3 / windowSize);

//      Serial.print(sum_s0); Serial.print(", ");
//      Serial.print(sum_s1); Serial.print(", ");
//      Serial.print(sum_s2); Serial.print(", ");
//      Serial.print(sum_s3); Serial.print(", ");
//      Serial.println("");

      
//  Serial.print(avg_s0); Serial.print(", ");
//  Serial.print(avg_s1); Serial.print(", ");
//  Serial.print(avg_s2); Serial.print(", ");
//  Serial.print(avg_s3); Serial.print(", ");
//  Serial.print(Input); Serial.print(", ");
  Serial.print(Output); Serial.println("");
}



int get_calibrated_sensor_value(int sensor_pin, int average) {
  return analogRead(sensor_pin) - average;
}




void run_motors(int pwm_speed) {

//  Serial.println(pwm_speed);
  int threshold = 100;
  
  if (pwm_speed > -threshold && pwm_speed < threshold) {
    
    return; // ignore PID outputs below threshold
    
  } else if (pwm_speed > threshold) {
     
     turn_right(pwm_speed);
     
  } else if (pwm_speed < -threshold) {
    
    turn_left(pwm_speed);
    
  }
  
//  if (pwm_speed < -100){
//    digitalWrite(in1, LOW);
//    digitalWrite(in2, HIGH);
//    analogWrite(enA, -pwm_speed);  // RIGHT MOTOR
//    digitalWrite(in3, LOW);
//    digitalWrite(in4, HIGH);
//    analogWrite(enB, 100);  // LEFT MOTOR
//  }
//  if (pwm_speed > 100){
//    digitalWrite(in1, LOW);
//    digitalWrite(in2, HIGH);
//    analogWrite(enA, 100);  // RIGHT MOTOR
//    digitalWrite(in3, LOW);
//    digitalWrite(in4, HIGH);
//    analogWrite(enB, pwm_speed);  // LEFT MOTOR
//  }
}


void turn_right(int pwm_speed) {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(enA, -pwm_speed);  // RIGHT MOTOR
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enB, 100);  // LEFT MOTOR
}

void turn_left(int pwm_speed) {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(enA, 100);  // RIGHT MOTOR
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enB, pwm_speed);  // LEFT MOTOR
}


float average_of_array(int this_array[], int num_elements) {
  float sum = 0;
  for (int i=0; i < num_elements; i++) {
    sum = sum + this_array[i];
  }
  float average = sum / num_elements;

  return average;
}


void set_up_photodiodes() {

  pinMode(sensor_pin0, INPUT);
  pinMode(sensor_pin1, INPUT);
  pinMode(sensor_pin2, INPUT);
  pinMode(sensor_pin3, INPUT);

}
