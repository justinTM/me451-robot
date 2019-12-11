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

int ms_start = 0;
int isTurnTime = true;

bool isDone = false;

bool isEvadeMode = true;



void setup() {

  // set up PID
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

  set_up_photodiodes();

  Serial.println("sensor_pin0, sensor_pin1, sensor_pin2, sensor_pin3, PID_IN, PID_OUT");
  //  Serial.println("PID_IN, PID_OUT");

  ms_start = millis();
}




void loop() {
  
  set_up_motors_straight(); 
  
  Input = (5*avg_s0 + avg_s1) - (avg_s2 + 5*avg_s3);
  myPID.Compute();
  run_motors(Output);

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
      
  Serial.print(avg_s0); Serial.print(", ");
  Serial.print(avg_s1); Serial.print(", ");
  Serial.print(avg_s2); Serial.print(", ");
  Serial.print(avg_s3); Serial.println("");
//  Serial.print(Input); Serial.print(", ");
//  Serial.print(Output); Serial.println("");
}




int get_calibrated_sensor_value(int sensor_pin, int average) {
  return analogRead(sensor_pin) - average;
}




void run_motors(int pwm_speed) {

//  Serial.println(pwm_speed);
  int threshold = 100;
  
  // right wheel is going faster than the left, so make left faster
  if (pwm_speed > -threshold && pwm_speed < threshold) {
    go_straight(255);
  } else if (pwm_speed > threshold) {
    turn_left(pwm_speed);
  } else if (pwm_speed < -threshold) {
    // pwm_speed is negative
    turn_right(pwm_speed);
  }
}




void go_straight(int pwm_speed) {
  analogWrite(enA, pwm_speed);  // RIGHT MOTOR
  analogWrite(enB, pwm_speed*1.5);  // LEFT MOTOR
}


void turn_right(int pwm_speed) {
  analogWrite(enA, 120);  // RIGHT MOTOR
  analogWrite(enB, -pwm_speed);  // LEFT MOTOR
}


void turn_left(int pwm_speed) {
  analogWrite(enA, pwm_speed);  // RIGHT MOTOR
  analogWrite(enB, 120);  // LEFT MOTOR
}


void turn_180_counterclockwise(int pwm_speed) {
  digitalWrite(in1, HIGH);  // reverse one of the motors
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);  // forward one of the motors
  digitalWrite(in4, HIGH);

  analogWrite(enA, pwm_speed);  // RIGHT MOTOR
  analogWrite(enB, pwm_speed * 1.1);  // LEFT MOTOR
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




void set_up_motors_straight() {
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}
