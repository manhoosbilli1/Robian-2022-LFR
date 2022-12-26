#include <Arduino.h>
#include "TB67H420FTG.h"

// double KP = .02; // you should edit this with your experiments....start with same values then increase or decrease as you like
// double KD = 0;
double KP = .075; // you should edit this with your experiments....start with same values then increase or decrease as you like
double KD = 0.008;

double KI = 0;
double lastError = 0;
const int GOAL = 2500;
unsigned int position;
unsigned int onLine;
unsigned char MAX_SPEED = 40;
const int ir = 8;
const int buz = 12;

unsigned int checkpoint = 0;
bool Stop = false;
//                    Motor driver DRV8833:
TB67H420FTG driver(3, 5, 9, 6); // in1,in2, in3, in4
// const int motorSpeed = 100; // Half speed (255 / 2).

// QTR
#include <QTRSensors.h>
#define NUM_SENSORS 6                  // number of sensors used
#define NUM_SAMPLES_PER_SENSOR 4       // average 4 analog samples per sensor reading
#define EMITTER_PIN QTR_NO_EMITTER_PIN // emitter is controlled by digital pin 2. in this case no emitter pin

// sensors 0 through 5 are connected to analog inputs 0 through 5, respectively
QTRSensorsAnalog qtra((unsigned char[]){A0, A1, A2, A3, A4, A5}, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];
double errSum;
unsigned long lastTime;
void followLine()
{
  // Get line position
  unsigned int position = qtra.readLine(sensorValues);
  //  Serial.println(position);

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

  // // Attach the motors to the input pins:
  driver.init();

  delay(500);
  pinMode(13, OUTPUT);
  pinMode(buz, OUTPUT);
  pinMode(ir, INPUT);

  Serial.print("calibrating");
  // QTR calibration
  digitalWrite(13, HIGH);       // turn on Arduino's LED to indicate we are in calibration mode
  for (int i = 0; i < 100; i++) // make the calibration take about 10 seconds
  {
    driver.setMotorAPower(40);
    driver.setMotorBPower(-40);

    qtra.calibrate(); // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)
  }
  driver.setMotorAPower(0);
  driver.setMotorBPower(0);
  digitalWrite(13, LOW); // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  // for (int i = 0; i < NUM_SENSORS; i++)
  // {
  //   Serial.print(qtra.calibratedMinimumOn[i]);
  //   Serial.print(' ');
  // }
  // Serial.println();

  // print the calibration maximum values measured when emitters were on
  // for (int i = 0; i < NUM_SENSORS; i++)
  // {
  //   Serial.print(qtra.calibratedMaximumOn[i]);
  //   Serial.print(' ');
  // }
  // Serial.println();
  // Serial.println();
  delay(3000);

  // calibration done
}

unsigned long int now = 0;
bool firstTime = true;
void loop()
{
  if (millis() - now >= 1000)
  {
    firstTime = true;
    digitalWrite(buz, LOW);
  }
  if (digitalRead(ir) == HIGH && firstTime == true)
  {
    now = millis();
    firstTime = false;
    Serial.println(checkpoint);
    digitalWrite(buz, HIGH);
    checkpoint++;
    if (checkpoint == 18)
    {
      Stop = true;
      checkpoint = 0;
      driver.setMotorAPower(0);
      driver.setMotorBPower(0);
      while (1)
        ;
    }
  }

  followLine();
  // if(onLine == 6)
  // {
  //   MAX_SPEED = 20;
  // }
  // else 
  // {
  //   MAX_SPEED = 45; 
  // }
  // if(digitalRead(ir) == HIGH)
  // {
  //   digitalWrite(buz, HIGH);
  // }
  // else {
  //   digitalWrite(buz, LOW);
  // }
  // }
  // check what position it gives when all on white..
  // check what position it gives when all on black.
  // on half black.
  //  when sensor on black line go straight with low speed.
  //  when sensor on goal and then T junction comes go striaght.

  /*DATA
  -> ALL 6 ON LINE = 6 SENSOR HIGH AT T INTERSECTION and pos 2500... COMMAND GO FORWARD UNTIL NO LONGER 6 ON LINE.


  */
}