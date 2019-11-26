#include <PID_v1.h>

#define NUM_MEASURES 100


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
int sensor_pin2 = 2;
int sensor_pin3 = 3;

// calibrating pins
int i = 0;  // count current number of measurements
bool isCalibrated = false;  // stop counting after calibration

// make array of values for each sensor
int array_s0[NUM_MEASURES];
int array_s1[NUM_MEASURES];
int array_s2[NUM_MEASURES];
int array_s3[NUM_MEASURES];

// moving average window filter stuff
float averages[NUM_MEASURES];  // each element is the average of a sensor array
unsigned long int iMovingAverage = 0;
const float windowSize = 50.0;
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

  // MOTORS
  set_up_motors(250);

  // PID
  Setpoint = 0;
//  myPID.SetMode(AUTOMATIC);  // turn PID on
//  myPID.SetOutputLimits(-1, 1);

  Serial.begin(115200);
  Serial.println("sensor_pin0, sensor_pin1, sensor_pin2, sensor_pin3");

  for (int i=0; i < intWindowSize; i++) {
    values_s0[i] = 0;
    values_s1[i] = 0;
    values_s2[i] = 0;
    values_s3[i] = 0;
  }
}

// pid output drives a ratio of two motor speeds



void loop() {


  // poll offset of photodiodes
  float offset_photodiodes12 = get_photodiode_difference(sensor_pin1, sensor_pin2);  // diff = pin1 - pin2

  Input = offset_photodiodes12;

  // feed offset to PID input
  // compute new PID value
  // feed PID output to motors (motor Left/Right ratio)
  // apply motor ratio to both motors (0-255 for L and R)

  // loop a few times, storing sensor values in 4 arrays
  // take average of values for each sensor

  // get a few values, to calibrate with average of each array
  if (i < NUM_MEASURES) {
    array_s0[i] = analogRead(sensor_pin0);
    array_s1[i] = analogRead(sensor_pin1);
    array_s2[i] = analogRead(sensor_pin2);
    array_s3[i] = analogRead(sensor_pin3);

    Serial.print(i); Serial.print(", ");
    i++;
  } else if (i == NUM_MEASURES && !isCalibrated) {
    Serial.println("");

    isCalibrated  = calibrate_sensors(averages, array_s0, array_s1, array_s2, array_s3);

    Serial.println("Calibrated values:");
    Serial.print(averages[0]); Serial.print(", ");
    Serial.print(averages[1]); Serial.print(", ");
    Serial.print(averages[2]); Serial.print(", ");
    Serial.println(averages[3]);

    Serial.println("Calibrated! Begin garbage:");
  } else {

      sum_s0 = sum_s0 - values_s0[iMovingAverage];
      sum_s1 = sum_s1 - values_s1[iMovingAverage];
      sum_s2 = sum_s2 - values_s2[iMovingAverage];
      sum_s3 = sum_s3 - values_s3[iMovingAverage];

      values_s0[iMovingAverage] = get_calibrated_sensor_value(sensor_pin0, averages[0]);
      values_s1[iMovingAverage] = get_calibrated_sensor_value(sensor_pin1, averages[1]);
      values_s2[iMovingAverage] = get_calibrated_sensor_value(sensor_pin2, averages[2]);
      values_s3[iMovingAverage] = get_calibrated_sensor_value(sensor_pin3, averages[3]);

      sum_s0 = sum_s0 + values_s0[iMovingAverage];
      sum_s1 = sum_s1 + values_s1[iMovingAverage];
      sum_s2 = sum_s2 + values_s2[iMovingAverage];
      sum_s3 = sum_s3 + values_s3[iMovingAverage];


      iMovingAverage++;

      if (iMovingAverage >= intWindowSize) {
        iMovingAverage = 0; // reset the moving average back to beginning
      }

      avg_s0 = sum_s0 / windowSize;
      avg_s1 = sum_s1 / windowSize;
      avg_s2 = sum_s2 / windowSize;
      avg_s3 = sum_s3 / windowSize;

//      Serial.print(sum_s0); Serial.print(", ");
//      Serial.print(sum_s1); Serial.print(", ");
//      Serial.print(sum_s2); Serial.print(", ");
//      Serial.print(sum_s3); Serial.print(", ");
//      Serial.println("");

      Serial.print(avg_s0); Serial.print(", ");
      Serial.print(avg_s1); Serial.print(", ");
      Serial.print(avg_s2); Serial.print(", ");
      Serial.print(avg_s3); Serial.print(", ");
      Serial.println("");


  }






}


int get_calibrated_sensor_value(int sensor_pin, int average) {
  return analogRead(sensor_pin) - average;
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
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(enA, pwm_speed);  // 0 - 255 PWM

    // LEFT
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enB, pwm_speed);  // 0 - 255 PWM

}


bool calibrate_sensors(float averages[], int array_s0[], int array_s1[], int array_s2[], int array_s3[]) {
  // average of each array
  averages[0] = average_of_array(array_s0, NUM_MEASURES);
  averages[1] = average_of_array(array_s1, NUM_MEASURES);
  averages[2] = average_of_array(array_s2, NUM_MEASURES);
  averages[3] = average_of_array(array_s3, NUM_MEASURES);




  return true;
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
