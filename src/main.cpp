#include <Arduino.h>

int fsrRearPin = 1;
int fsrPin = 0;
int LEDpin = 11;
int rearLEDpin = 10;

int maxPressure = 0;
int refPressure = 0;

void setup() {
  Serial.begin(9600);
  pinMode(LEDpin, OUTPUT);
  pinMode(rearLEDpin, OUTPUT);
}

void driveForward(int power){
  int ledBrightness = map(power, 0, 100, 0, 255);
  analogWrite(LEDpin, ledBrightness);
}

void driveBackward(int power){
  int ledBrightness = map(power, 0, 100, 0, 255);
  analogWrite(rearLEDpin, ledBrightness);
}

void loop() {
  int fsrFrontReading = analogRead(fsrPin);
  int fsrRearReading = analogRead(fsrRearPin);

  int powerForward = 0;
  int powerBackward = 0;
  if(fsrFrontReading > 0 && fsrRearReading > 0){
    Serial.print("fsrFrontReading = ");
    Serial.println(fsrFrontReading);
    Serial.print("fsrRearReading = ");
    Serial.println(fsrRearReading);

    if(fsrFrontReading > maxPressure){
      maxPressure = fsrFrontReading;
    }

    if(fsrRearReading > maxPressure){
      maxPressure = fsrRearReading;
    }

    int equtyThreshold = 50;
    if(abs(fsrFrontReading - fsrRearReading) < equtyThreshold){
      // The pressure on both sensors is +- the same
      refPressure = (fsrFrontReading + fsrRearReading) / 2; // take the middle as a ref. pressure
    }

    if(fsrFrontReading > refPressure || fsrRearReading > refPressure){
      int frontPressure = map(fsrFrontReading, refPressure, maxPressure, 0, 100);
      int backPressure = map(fsrRearReading, refPressure, maxPressure, 0, 100);

      // Serial.print("maxPressure = ");
      // Serial.println(maxPressure);
      // Serial.print("refPressure = ");
      // Serial.println(refPressure);
      // Serial.print("frontPressure = ");
      // Serial.println(frontPressure);
      // Serial.print("backPressure = ");
      // Serial.println(backPressure);

      int pressureThreshold = 15; // diff to move forward
      int backPressureThreshold = 40; // diff to apply breaks
      int maxPressure = 70; // the diff above means full throttle

      if(frontPressure - backPressure > pressureThreshold){
        if(frontPressure > maxPressure){
          powerForward = 100;
        } else {
          powerForward = map(frontPressure, pressureThreshold, 100, 20, 100);
        }

        driveForward(powerForward);
        driveBackward(0);
      } else if(backPressure - frontPressure > backPressureThreshold){
        powerBackward = map(backPressure, backPressureThreshold, 100, 20, 100);

        driveForward(0);
        driveBackward(powerBackward);
      } else {
        driveForward(0);
        driveBackward(0);
      }
    }
  } else{
    driveForward(0);
    driveBackward(0);
  }

  Serial.print("Power Forward = ");
  Serial.println(powerForward);
  Serial.print("Power Backward = ");
  Serial.println(powerBackward);
  Serial.println("------------------");

  delay(200);
}
