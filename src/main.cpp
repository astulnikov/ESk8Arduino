#include <Arduino.h>
#include <VescUart.h>
#include <SoftwareSerial.h>
#include <Servo.h>

const int LEFT_MOTOR_ID = 42;

int fsrRearPin = 1;
int fsrPin = 0;
int LEDpin = 11;
int rearLEDpin = 10;

int maxPressure = 0;
int refPressure = 0;

int currentPowerForward = 0;
int currentPowerBackward = 0;

VescUart vesc;
SoftwareSerial comSerial(2, 3); // RX, TX

void setup() {
  comSerial.begin(9600);

  pinMode(LEDpin, OUTPUT);
  pinMode(rearLEDpin, OUTPUT);

  Serial.begin(115200);
  vesc.setSerialPort(&Serial);
}

void releaseMotors(){
  analogWrite(rearLEDpin, 0);
  analogWrite(LEDpin, 0);

  vesc.setCurrent(0);
  vesc.setCurrent(0, LEFT_MOTOR_ID);
}

void driveForward(int power){
  comSerial.print("Power Forward = ");
  comSerial.println(power);

  int ledBrightness = map(power, 0, 100, 0, 255);
  analogWrite(LEDpin, ledBrightness);
  analogWrite(rearLEDpin, 0);

  power = power / 4; // Reduce the maximum power
  vesc.setDuty(power / 100.0);
  vesc.setDuty(power / 100.0, LEFT_MOTOR_ID);
}

void driveBackward(int power){ // Currently brakes
  comSerial.print("Power Backward = ");
  comSerial.println(power);

  int ledBrightness = map(power, 0, 100, 0, 255);
  analogWrite(rearLEDpin, ledBrightness);
  analogWrite(LEDpin, 0);

  vesc.setBrakeCurrent(power / 5.0);
  vesc.setBrakeCurrent(power / 5.0, LEFT_MOTOR_ID);
}

void brake(){
  comSerial.println("Brake");

  analogWrite(rearLEDpin, 255);
  analogWrite(LEDpin, 0);

  vesc.setDuty(0);
  vesc.setDuty(0, LEFT_MOTOR_ID);
}

void readVESC(){
  if (vesc.getVescValues() ) {
    comSerial.println(vesc.data.rpm);
    comSerial.println(vesc.data.inpVoltage);
    comSerial.println(vesc.data.ampHours);
    comSerial.println(vesc.data.tachometerAbs);
  }
}

void printPressure(int fsrFrontReading, int fsrRearReading){
  comSerial.print("fsrFrontReading = ");
  comSerial.println(fsrFrontReading);
  comSerial.print("fsrRearReading = ");
  comSerial.println(fsrRearReading);
}

int filterPowerForward(int power) {
  int resultingPower = 0;
  if(currentPowerBackward == 0){
    resultingPower = (currentPowerForward + power) / 2;
  }
  return resultingPower;
}

int filterPowerBackward(int power) {
  int resultingPower = 0;
  if(currentPowerForward == 0){
    resultingPower = power;
  }
  return resultingPower;
}

void checkPressure(){
  int fsrFrontReading = analogRead(fsrPin);
  int fsrRearReading = analogRead(fsrRearPin);

  int powerForward = 0;
  int powerBackward = 0;

  if(fsrFrontReading > 50){
    if(fsrRearReading > 50){
      // printPressure(fsrFrontReading, fsrRearReading);

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

        frontPressure = max(0, frontPressure);
        backPressure = max(0, backPressure);

        int pressureThreshold = 20; // diff to move forward
        int backPressureThreshold = 30; // diff to apply breaks
        int maxPressure = 70; // the diff above means full throttle

        if(frontPressure - backPressure > pressureThreshold){
          if(frontPressure > maxPressure){
            powerForward = 100;
          } else {
            powerForward = map(frontPressure, pressureThreshold, 100, 20, 100);
          }

          powerForward = filterPowerForward(powerForward);
          driveForward(powerForward);
        } else if(backPressure - frontPressure > backPressureThreshold){
          powerBackward = map(backPressure, backPressureThreshold, 100, 20, 100);

          powerBackward = filterPowerBackward(powerBackward);
          if(powerBackward = 0){
            powerForward = filterPowerForward(0);
          }
          brake();
        } else {
          releaseMotors();
        }
      } else {
        releaseMotors();
      }
    } else {
      releaseMotors();
    }
  } else{
    driveForward(0);
  }
  currentPowerForward = powerForward;
  currentPowerBackward = powerBackward;
}

void loop() {
  readVESC();
  checkPressure();

  delay(50);
}
