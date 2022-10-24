#include <Pololu3piPlus32U4.h>

using namespace Pololu3piPlus32U4;

Encoders encoders;
Buzzer buzzer;
Motors motors;

unsigned long currentMillis;
unsigned long prevMillis;
const unsigned long PERIOD = 20;

long countsLeft = 0;
long countsRight = 0;
long prevLeft = 0;
long prevRight = 0;

const int CLICKS_PER_ROTATION = 12;
const float GEAR_RATIO = 75.81F;
const float WHEEL_DIAMETER = 3.2;
const int WHEEL_CIRCUMFERENCE = 10.0531;

float Sl = 0.0F;
float Sr = 0.0F;

float totalSl = 0.0F;
float totalSr = 0.0F;

long currentStep = 1;

// Units in cm.
const float FOOT = 30.5;
const float INCH = 2.54;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  delay(500);
  buzzer.play("c32");
}

void loop() {
  // put your main code here, to run repeatedly:
  checkEncoders();
  
  //Step 1; Drive forward 12 inches.
  if (currentStep == 1) {
    moveForward(FOOT);
  }

  //Step 2; 
  else if (currentStep == 2) {
    currentStep +=1;
    clearLeftRight();
  }

  //Step 3; Drive backwards for 12 inches.
  else if (currentStep == 3) {
    moveBackward(FOOT);
  }

  //Step 4;
  else if (currentStep == 4) {
    currentStep += 1;
    clearLeftRight();
  }

  //Step 5; Drive forward 18 inches.
  else if (currentStep == 5) {
    moveForward(INCH*18);
  }
  
  //Step 6; Beep.
  else if (currentStep == 6) {
    buzzer.play("c32");
    //currentStep = 1;
    clearLeftRight();
  }
}

// Clears Sl and Sr for movement calculation purposes.
void clearLeftRight(){
  Sl = 0.0F;
  Sr = 0.0F;
}

void checkEncoders() {
  currentMillis = millis();
  if (currentMillis > prevMillis + PERIOD) {
    countsLeft += encoders.getCountsAndResetLeft();
    countsRight += encoders.getCountsAndResetRight();

    Sl += ((countsLeft - prevLeft) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE);
    Sr += ((countsRight - prevRight) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE);

    totalSl += ((countsLeft - prevLeft) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE);
    totalSr += ((countsRight - prevRight) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE);
    
    Serial.print("Left: ");
    Serial.print(Sl);
    Serial.print("Right: ");
    Serial.println(Sr);

    prevLeft = countsLeft;
    prevRight = countsRight;
    prevMillis = currentMillis;
  }
}

void moveForward(long distanceInCm) {
  int wheelSpeed = 200;

  if (Sl < distanceInCm) {
    if (Sl > distanceInCm - 10) {
      wheelSpeed = 100 * ((distanceInCm - Sl) / 10);
    } if (wheelSpeed < 40) wheelSpeed = 40;
    motors.setSpeeds(wheelSpeed, wheelSpeed);
  } else {
    motors.setSpeeds(0, 0);
    currentStep = currentStep + 1;
  }
  
}

void moveBackward(long distanceInCm) {
  int wheelSpeed = -125;

  distanceInCm *= -1;

  if (Sl > distanceInCm) {
    if (Sl < distanceInCm + 10) {
      wheelSpeed = 100 * ((distanceInCm - Sl) / 10);
    } if (wheelSpeed > -40) wheelSpeed = -40;
    motors.setSpeeds(wheelSpeed, wheelSpeed);
  } else {
    motors.setSpeeds(0, 0);
    currentStep = currentStep + 1;
  }
}
