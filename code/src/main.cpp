#include <Arduino.h>
#include <DRV8833.h>
#include <PID_v1.h>
#include <DRV8835.h>
#include <L298.h>

// l298n
L298 driver(8, 3, 5, 4, 7, 6); // m1,m2,enA,m3,m4,enB

// pid
const double KP = 0.8; // you should edit this with your experiments....start with same values then increase or decrease as you like
const double KD = 0;
double lastError = 0;
const int GOAL = 3500;
const unsigned char MAX_SPEED = 100;

//                    PID
// Define Variables we'll be connecting to
double Setpoint, Input, Output;
// Specify the links and initial tuning parameters
double Kp = 2, Ki = 5, Kd = 1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

//                    Motor driver DRV8833:
// DRV8833 driver = DRV8833();
const int inputA1 = 5, inputA2 = 6, inputB1 = 9, inputB2 = 10;
const int motorSpeed = 128; // Half speed (255 / 2).

// QTR
#include <QTRSensors.h>
#define NUM_SENSORS 8                  // number of sensors used
#define NUM_SAMPLES_PER_SENSOR 4       // average 4 analog samples per sensor reading
#define EMITTER_PIN QTR_NO_EMITTER_PIN // emitter is controlled by digital pin 2. in this case no emitter pin

// sensors 0 through 5 are connected to analog inputs 0 through 5, respectively
QTRSensorsAnalog qtra((unsigned char[]){A0, A1, A2, A3, A4, A5, A6, A7}, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

void followLine()
{
  // Get line position
  unsigned int position = qtra.readLine(sensorValues);
  Serial.println(position);

  // Compute error from line
  int error = GOAL - position;
  //
  // Compute motor adjustment
  int adjustment = KP * error + KD * (error - lastError);
  //
  // Store error for next increment
  lastError = error;
  //
  // Adjust motors
  driver.setMotorAPower(constrain(MAX_SPEED - adjustment, 0, MAX_SPEED));
  driver.setMotorBPower(constrain(MAX_SPEED + adjustment, 0, MAX_SPEED));
}

void setup()
{
  // Start the serial port:
  Serial.begin(9600);

  // l298n driver init
  driver.init();
  // // Attach the motors to the input pins:
  // driver.attachMotorA(inputA1, inputA2);
  // driver.attachMotorB(inputB1, inputB2);

  delay(500);
  pinMode(13, OUTPUT);
  Serial.print("calibrating");
  // QTR calibration
  digitalWrite(13, HIGH);       // turn on Arduino's LED to indicate we are in calibration mode
  for (int i = 0; i < 100; i++) // make the calibration take about 10 seconds
  {
    qtra.calibrate(); // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)
  }
  digitalWrite(13, LOW); // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtra.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtra.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);

  // calibration done

  Serial.println("Ready!");
}

void loop()
{
  // put your main code here, to run repeatedly:
  // read calibrated sensor values and obtain a measure of the line position from 0 to 5000
  // To get raw sensor values, call:
  // qtra.read(sensorValues); instead of unsigned int position = qtra.readLine(sensorValues);
  // unsigned int position = qtra.readLine(sensorValues);

  // // print the sensor values as numbers from 0 to 1000, where 0 means maximum reflectance and
  // // 1000 means minimum reflectance, followed by the line position
  // for (unsigned char i = 0; i < NUM_SENSORS; i++)
  // {
  //   Serial.print(sensorValues[i]);
  //   Serial.print('\t');
  // }
  // // Serial.println(); // uncomment this line if you are using raw values
  // Serial.println(position); // comment this line out if you are using raw values

  // delay(250);

  followLine(); 
}
