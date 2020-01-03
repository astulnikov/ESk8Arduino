#include <Arduino.h>
#include <VescUart.h>
#include <SoftwareSerial.h>
#include <Servo.h>

const float WHEEL_SIZE = 0.09;

const byte CHECK_CODE = 'c';
const byte END_LINE_CODE = 'l';

const byte RUN_STOP_CODE = 'S';
const byte RUN_FORWARD_CODE = 'F';
const byte RUN_BACKWARD_CODE = 'B';
const byte TURN_TO_ANGLE_CODE = 'T';

const byte SPEED_METRIC_CODE = 'S';
const byte COMPASS_METRIC_CODE = 'C';
const byte FRONT_SCANNER_METRIC_CODE = 'F';
const byte REAR_DISTANCE_METRIC_CODE = 'R';

const int LEFT_MOTOR_ID = 42;
const int MAX_RPM = 20000;

int fsrRearPin = 1;
int fsrPin = 0;
int LEDpin = 11;
int rearLEDpin = 10;

int currentForward = 0;
int currentBackward = 0;

int currentRpm = 0;

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

  if(power == 0){
    analogWrite(rearLEDpin, 255);
    analogWrite(LEDpin, 255);
  } else {
    int ledBrightness = map(power, 0, 100, 0, 255);
    analogWrite(LEDpin, ledBrightness);
    analogWrite(rearLEDpin, 0);
  }

  // int rpmRate = currentRpm / MAX_RPM * 100;
  // if(rpmRate < power){
  //   power = rpmRate + (power - rpmRate) / 4;
  // }

  power = min(power, 50); // Reduce the maximum power

  vesc.setDuty(power / 100.0);
  vesc.setDuty(power / 100.0, LEFT_MOTOR_ID);
}

void driveBackward(int power){ // Currently brakes

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

void sendFloatAsByteArray(float value){
  typedef union{
    float number;
    uint8_t bytes[sizeof value];
  } FLOATUNION_t;

  FLOATUNION_t floatUnion;
  floatUnion.number = value;
  comSerial.write(floatUnion.bytes, sizeof(floatUnion.bytes));
}

void readVESC(){
  if (vesc.getVescValues() ) {
    int rpm = vesc.data.rpm;
    if(rpm != currentRpm){
      float speed = rpm * WHEEL_SIZE * 3.14159265359 / 60;
      comSerial.write(SPEED_METRIC_CODE);
      sendFloatAsByteArray(speed);
      comSerial.write("\n");
    }
    currentRpm = rpm;
    // comSerial.println(vesc.data.rpm);
    // comSerial.println(vesc.data.inpVoltage);
    // comSerial.println(vesc.data.ampHours);
    // comSerial.println(vesc.data.tachometerAbs);
  }
}

void printPressure(int fsrFrontReading, int fsrRearReading){
  comSerial.print("fsrFrontReading = ");
  comSerial.println(fsrFrontReading);
  comSerial.print("fsrRearReading = ");
  comSerial.println(fsrRearReading);
}

int filterValue(int current, int previous) {
  int result = 0;
  result = (current + previous) / 2;
  return result;
}

void checkPressure(){
  int fsrFrontReading = analogRead(fsrPin);
  int fsrRearReading = analogRead(fsrRearPin);

  if(fsrFrontReading > 20){
    if(fsrRearReading > 20){
      // printPressure(fsrFrontReading, fsrRearReading);

      fsrFrontReading = filterValue(fsrFrontReading, currentForward);
      fsrRearReading = filterValue(fsrRearReading, currentBackward);

      currentForward = fsrFrontReading;
      currentBackward = fsrRearReading;

      int minDiff = (fsrFrontReading + fsrRearReading) / 4;
      int diff = abs(fsrFrontReading - fsrRearReading);
      int startThreshold = 50;
      if(diff > startThreshold){
        if(fsrFrontReading > fsrRearReading){
          int power = map(diff, startThreshold, 1023 - minDiff, 5, 100);
          power =  min(power, 100);
          driveForward(power);
        } else {
          brake();
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
}

union ArrayToInteger {
  byte array[4];
  uint32_t integer;
};

void chooseAction(byte data[]) {
    if(data[0] != CHECK_CODE){
      return;
    }
    
    byte command = data[1];
    switch (command) {
      case RUN_FORWARD_CODE:
      {
        ArrayToInteger converter = {data[5], data[4], data[3], data[2]};
        int speed = converter.integer;
        driveForward(speed);
      } break;
      case RUN_BACKWARD_CODE:
      {
        driveBackward(80);
      }  break;
      case RUN_STOP_CODE:
      {
        releaseMotors();
      }  break;
      case TURN_TO_ANGLE_CODE: 
        ArrayToInteger converter = {data[5], data[4], data[3], data[2]};
        int angle = converter.integer;
        break;
    }
}

void readMessage() {
  byte data[256];
  comSerial.readBytesUntil(END_LINE_CODE, data, 256);  
  chooseAction(data);
}

void loop() {
  readVESC();
  // checkPressure();

  if (comSerial.available() > 0) {
    readMessage();
  } 
  
  delay(50);
}
