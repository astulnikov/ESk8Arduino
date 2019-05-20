#include <Arduino.h>
#include <VescUart.h>
#include <SoftwareSerial.h>
#include <Servo.h>

const int STOP_ESC_VALUE = 1500;
const int BACK_ESC_VALUE = 1200;
const int RUN_ESC_VALUE = 1700;

int fsrRearPin = 1;
int fsrPin = 0;
int LEDpin = 11;
int rearLEDpin = 10;

int maxPressure = 0;
int refPressure = 0;

VescUart vesc;
SoftwareSerial comSerial(2, 3); // RX, TX

void setup() {
  comSerial.begin(9600);

  pinMode(LEDpin, OUTPUT);
  pinMode(rearLEDpin, OUTPUT);

  Serial.begin(115200);
  vesc.setSerialPort(&Serial);
}

void driveForward(int power){
  comSerial.print("Power Forward = ");
  comSerial.println(power);

  int ledBrightness = map(power, 0, 100, 0, 255);
  analogWrite(LEDpin, ledBrightness);
  analogWrite(rearLEDpin, 0);

  vesc.setRPM(100 * power);
}

void driveBackward(int power){ // Currently brakes
  comSerial.print("Power Backward = ");
  comSerial.println(power);

  int ledBrightness = map(power, 0, 100, 0, 255);
  analogWrite(rearLEDpin, ledBrightness);
  analogWrite(LEDpin, 0);

  vesc.setBrakeCurrent(power / 10);
}

void readVESC(){
  if (vesc.getVescValues() ) {
    comSerial.println(vesc.data.rpm);
    comSerial.println(vesc.data.inpVoltage);
    comSerial.println(vesc.data.ampHours);
    comSerial.println(vesc.data.tachometerAbs);
  }
}

void loop() {
  readVESC();
  int fsrFrontReading = analogRead(fsrPin);
  int fsrRearReading = analogRead(fsrRearPin);

  int powerForward = 0;
  int powerBackward = 0;
  if(fsrFrontReading > 0 && fsrRearReading > 0){
    comSerial.print("fsrFrontReading = ");
    comSerial.println(fsrFrontReading);
    comSerial.print("fsrRearReading = ");
    comSerial.println(fsrRearReading);

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

      int pressureThreshold = 25; // diff to move forward
      int backPressureThreshold = 40; // diff to apply breaks
      int maxPressure = 70; // the diff above means full throttle

      if(frontPressure - backPressure > pressureThreshold){
        if(frontPressure > maxPressure){
          powerForward = 100;
        } else {
          powerForward = map(frontPressure, pressureThreshold, 100, 20, 100);
        }

        driveForward(powerForward);
      } else if(backPressure - frontPressure > backPressureThreshold){
        powerBackward = map(backPressure, backPressureThreshold, 100, 20, 100);

        driveBackward(powerBackward);
      } else {
        driveForward(0);
      }
    }
  } else{
    driveForward(0);
  }

  delay(200);
}
