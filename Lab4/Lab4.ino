#include <Pololu3piPlus32U4.h>

using namespace Pololu3piPlus32U4;

Encoders encoders;
Buzzer buzzer;
Motors motors;
ButtonA buttonA;

//Switches:
const boolean WHEELS_DEBUG = false;
const boolean XYCOORDS_DEBUG = true;

unsigned long currentMillis;
unsigned long prevMillis;
const unsigned long PERIOD = 20;
float basespeed = 100.0;

long countsLeftWheel = 0;
long countsRightWheel = 0;
long prevLeftWheel = 0;
long prevRightWheel = 0;

// we are using cm for measurements
const int CLICKS_PER_ROTATION = 12; // every single time the motor makes one full ration 12 state changes ack by encoders
const float GEAR_RATIO = 75.81F;
const float WHEEL_DIAMETER = 3.2;
const int WHEEL_CIRCUMFRENCE = 10.0531;
const float B = 8.4; // const distance between wheels of robot

float Sl = 0.0F; // distance traveled by left and right wheel
float Sr = 0.0F;
float prevSl = 0.0F;
float prevSr = 0.0F;

float action = 1.0; // used as goal/current action of old program


int currentStep = 0; // Used to iterate through X & Y array
// Goal X and Y locations.
int X[] = {80, -60, -60, 0};
int Y[]  = {50, 0, -30, 0};
float deltaS;
float goalAngle;

// variables will track the current X and Y location in a grid.
float currentX = 0.0F;
float currentY = 0.0F;
float currentAngle = 0.0F;
float ALLOWEDERROR = 0.9; // allowed error for each wheel to be from goal state

// constants used in proportional;
float kp = 35.0;

// d = sqrt((Xg-X)^2+(Y-Yg)^2)  -> how to slow down when get close to goal

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  
  // Flip motors for robot. 
//  motors.flipLeftMotor(true);
//  motors.flipRightMotor(true);
  
  delay(1000);
  //buzzer.play("!T240 L8 agafaea");
  buzzer.play("c32");
}

void loop() {

  //Step 1: Check if robot has reached goal state.
  
  if (checkGoal()) {
    moveWheels(0, 0);
    buzzer.play("c32");
    currentStep+=1;
    basespeed = 100;
    Serial.println("HIT CHECK GOAL!!");
  }
  Serial.println("DID I HIT CHECK GOAL??");

  //Step 2: Get Sl and Sr
  checkEncoders();

  //Step 3: Calculate current location and goalAngle
  updateCurrentLocationAndGoalAngle();

    if (XYCOORDS_DEBUG) {
    Serial.print("X: ");
    Serial.print(currentX);
    Serial.print("  |  Y: ");
    Serial.println(currentY);
    }
    

  //5: apply pid to wheels
  
    
   moveWheels(basespeed + (proportionalOf()), basespeed + (-proportionalOf())); 
  // use output of calculations to determine how much to move robot

  if(currentStep > 3){
     moveWheels(0, 0);
  }
  
}


void checkEncoders() {
  //currentMillis = millis();
  //if (currentMillis > prevMillis + PERIOD) {
    countsLeftWheel +=  encoders.getCountsAndResetLeft();
    countsRightWheel += encoders.getCountsAndResetRight();

    prevSl = Sl;
    prevSr = Sr;

    Sl = ((countsLeftWheel - prevLeftWheel) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFRENCE); // will give us distance achived by wheel
    Sr = ((countsRightWheel - prevRightWheel) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFRENCE);
    Serial.print("sl: ");
    Serial.println(Sl);
    Serial.print("sr: ");
    Serial.println(Sr);
    prevLeftWheel = countsLeftWheel;
    prevRightWheel = countsRightWheel;
    prevMillis = currentMillis;
 // }
}

void updateCurrentLocationAndGoalAngle() {
   
    // Get the change in S.
    deltaS = (Sl + Sr) / 2;
    
    //Get the change in Angle (Theta), X, and Y
    float deltaTheta = (Sr - Sl) / B;
    float deltaX = deltaS * cos(currentAngle + (deltaTheta / 2));
    float deltaY = deltaS * sin(currentAngle + (deltaTheta / 2));
    
    
    // update current angle and current location. (Subtract because robot is facing backwards?).
    currentAngle += deltaTheta; // This might not be right? Perhaps add here?
    currentX += deltaX;
    currentY += deltaY;
    
    // Get the desired angle and set to 'goalAngle'
    float diffY = Y[currentStep] - currentY; 
    float diffX = X[currentStep] - currentX;
    goalAngle = atan2(diffY, diffX);

    //    --> Side note here: What's the formula for getting goalAngle?   
    //  Response: goalAngle = atan2(Ygoal-Y,Xgoal-X) refrenced in the goals & avoidance slides
    //    --> / Shouldn't we be using our current theta as well? 
    //  Response: current theta comes into play when applying pid (apply proportion based on diff between goalAngle and currentAngle (currentTheta)
      
}

// Function checks to see if you are at goal location using global var currentStep.
// Returns true if you are at goal location, false if not.
boolean checkGoal() {

    // Initialize variables.
    boolean xArrived = false;
    boolean yArrived = false;
    float goalX = X[currentStep];
    float goalY = Y[currentStep];


        // slows down robot when close to goal
      // if current X is within goalX + or - 5.
      if ((currentX <= goalX + 5) && (currentX >= goalX - 5)) {
        // if current Y is within goalY + or - 5.
        if ((currentY <= goalY + 5) && (currentY >= goalY - 5)) {
        basespeed = 60;
        }
      }
        

    
    
      // if current X is within goalX + or - ALLOWEDERROR.
      if ((currentX <= goalX + ALLOWEDERROR) && (currentX >= goalX - ALLOWEDERROR)) {
        xArrived = true;
      }
        
      // if current Y is within goalY + or - ALLOWEDERROR.
      if (currentY <= goalY + ALLOWEDERROR && currentY >= goalY - ALLOWEDERROR) {
        yArrived = true;
      }
      
      return (xArrived && yArrived);
}


/*
    1) calculate theta error and apply proportional pid
*/

// Given "X" or "Y", returns the proportional piece in PID.
float proportionalOf() {
      
     // Proportional.
      float error = goalAngle - currentAngle;
      error = atan2(sin(error), cos(error));
      float proportional = kp * error;

      if (proportional > basespeed){
        proportional = basespeed;
      }
      return proportional;
 
}

// Will move wheels given each wheel speed. (Convenience function)
 void moveWheels(int leftWheel, int rightWheel){
      motors.setSpeeds(rightWheel, leftWheel);
    
      if (WHEELS_DEBUG) {
       Serial.print("left: ");
       Serial.print(leftWheel);
       Serial.print(" | right: ");
       Serial.println(rightWheel);
      }
  
 } // ends function
