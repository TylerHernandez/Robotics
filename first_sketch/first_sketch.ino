#include <Pololu3piPlus32U4.h>

using namespace Pololu3piPlus32U4;

Encoders encoders;
Buzzer buzzer;
Motors motors;
ButtonA buttonA;


unsigned long currentMillis;
unsigned long prevMillis;
const unsigned long PERIOD = 20;

long countsLeft = 0;
long countsRight = 0;
long prevLeft = 0;
long prevRight = 0;
long counter = 0;

const int CLICKS_PER_ROTATION = 12;
const float GEAR_RATIO = 75.81F;
const float WHEEL_DIAMETER = 3.2;
const int WHEEL_CIRCUMFERENCE = 10.0531;

float Sl = 0.0F;
float Sr = 0.0F;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  delay(1000);
  buzzer.play("c32");

}

void loop() {
  // put your main code here, to run repeatedly:
  checkEncoders();
}

void checkEncoders() {
  currentMillis = millis();
  if(currentMillis > prevMillis + PERIOD) {
    countsLeft += encoders.getCountsAndResetLeft();
    countsRight += encoders.getCountsAndResetRight();

     Sl += ((countsLeft - prevLeft) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE);
     Sr += ((countsRight - prevRight) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE);

//    
//    int wheelSpeed = 200;
//
//    if (Sl < 1000) {
//    if (Sl < 30) {
//      motors.setSpeeds(wheelSpeed, wheelSpeed);
//    }
//    else if(Sl < 300) {
//      motors.setSpeeds(wheelSpeed, -wheelSpeed);
//    }
//    else {
//      motors.setSpeeds(wheelSpeed, wheelSpeed+150);
//    }
//    }
//      
     
//     if(Sl < 200) {
////      if(Sl > 20) {
////        wheelSpeed = 100 * ((30-Sl) / 10);
////        if(wheelSpeed < 40) wheelSpeed = 40;
////      }
//      
//     }
//     else {
//      motors.setSpeeds(0, 0);
//     }

//Drive forward 12 inches


//Stop for 2 seconds


//Drive backwards for 6 inches


//Stop for 2 seconds


//Drive forward 3 feet (36 inches)


//Beep
     
    Serial.print("Left: ");
    Serial.print(Sl);
    Serial.print("Right: ");
    Serial.println(Sr);

    prevLeft = countsLeft;
    prevRight = countsRight;
    prevMillis = currentMillis;
    counter = counter + 1;
  } 
}
