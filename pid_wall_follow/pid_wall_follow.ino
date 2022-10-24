#include <Pololu3piPlus32U4.h>
#include <Servo.h>

using namespace Pololu3piPlus32U4;


// Follow wall lab

Buzzer buzzer;

Servo headServo;

Motors motors;

Encoders encoders;

// Switches.
const boolean HEAD_DEBUG = false;
const boolean ULTRASONIC_DEBUG = false;
const boolean AVERAGE_DEBUG = false;
const boolean LEFTFRONTMOSTMID_DEBUG = false;
const boolean WHEELS_DEBUG = false;

// Head servo timing.
unsigned long headCm;
unsigned long headPm;
const unsigned long HEAD_MOVEMENT_PERIOD = 500;

// Head servo constants.
const int HEAD_SERVO_PIN = 21;
const int NUM_HEAD_POSITIONS = 3;
const int HEAD_POSITIONS[NUM_HEAD_POSITIONS] = {180, 135, 100};

// head servo data.
boolean headDirectionClockwise = true;
int currentHeadPosition = 0;

// Initialize Ultrasonic.
const int ECHO_PIN = 22;
const int TRIG_PIN = 4;

// Ultrasonic Maxs.
const float MAX_DISTANCE = 150.00; // (150cm = 1.5 meter)

// Current US distance reading.
int distance = 0;

// Current reading from ultrasonic.
double currentDistance = 0;

// Most updated readings on angles.
double leftmostReading = -1;
double midReading = -1;
double frontmostReading = -1;

// Wheel constants.
const int CLICKS_PER_ROTATION = 12;
const float GEAR_RATIO = 75.81F;
const float WHEEL_DIAMETER = 3.2;
const int WHEEL_CIRCUMFERENCE = 10.0531;

// PID variables. 
const double desiredStateFront = (double) 50;
const double desiredStateLeft = (double) 30;

const double kp = 1;
const double ki = .8;
const double kd = 1;

double kiTotalLeft = 0.0;
double priorLeftError = 0.0;
double kiTotalFront = 0.0;
double priorFrontError = 0.0;

void setup() {
  Serial.begin(57600);

  // Flip motors for robot. 
  motors.flipLeftMotor(true);
  motors.flipRightMotor(true);
  
  // Initialize head position to start.
  headServo.attach(HEAD_SERVO_PIN);
  headServo.write(140);

  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);

  buzzer.play("c32");
  
  // QUICK! Put the robot down!
  delay(2000);

  // Sing a song for brownie points.
  //buzzer.play("!T240 L8 agafaea dac+adaea fa<aa<bac#a dac#adaea f4");
}

void loop() {
  headCm = millis();
  // We only need one time constraint for each of the following actions.
  if(headCm > headPm + HEAD_MOVEMENT_PERIOD) {

  // First, move servo.
  moveHead();

  // Next, grab the average distance of seen wall at this angle.
  grabReadingsPerAngle(); // Updated variables: 'leftmostReading', 'midReading' (unused), 'frontmostReading'.

  if (LEFTFRONTMOSTMID_DEBUG) {
    Serial.print("leftmost: ");
    Serial.println(leftmostReading);
    Serial.print("mid: ");
    Serial.println(midReading);
    Serial.print("frontmost: ");
    Serial.println(frontmostReading);
  }

  float leftpidresult = 0;
  float frontpidresult = 0;

// Do not move wheels until each angle reading has been taken.
if (leftmostReading != -1 && frontmostReading != -1) {

// if object in front is too close turn away from front wall, adjusting using frontPID.
if (frontmostReading <= desiredStateFront) {
  frontpidresult = frontPID();

  if (frontpidresult <= -50) frontpidresult = -50;
  if (frontpidresult >= 50) frontpidresult = 50;
  moveWheels(60 + frontpidresult, 60);

// If it does not see anything to its left and in front, it is probably nearing a corner, proceed with slight turn left.
} else if (leftmostReading > 100 && frontmostReading > 100) {
  moveWheels(80, 100);

// Follow wall besides you, adjust using leftPID.
} else {
  leftpidresult = leftPID();

  if (leftpidresult <= -30) leftpidresult = -30;
  if (leftpidresult >= 30) leftpidresult = 30;

  moveWheels(60 + leftpidresult, 60);
 }
 
}
 
  // Reset previous millis.
  headPm = headCm;
 } // Ends time constraint if.
}

void usReadCm() {
    // Clears the TRIG_PIN (set low).
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);


    // Sets the TRIG_PIN HIGH (activate) for 10 microseconds. 
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
  
    // Reads the ECHO_PIN, returns the sound wave travel time in microseconds.
    // note the duration (3800 microseconds) that will allow for reading up max distance supported by the sensor.
    long duration = pulseIn(ECHO_PIN, HIGH, 38000);
    // Calculating the distance.
    distance = duration * 0.034 / 2; // Time of flight equation: Speed of sound wave divided by 2. 

    // Apply limits
    if (distance > MAX_DISTANCE) distance = MAX_DISTANCE;
    if (distance == 0) distance = MAX_DISTANCE;

    currentDistance = distance;
    
    if (ULTRASONIC_DEBUG) {
    // Displays the distance on the Serial Monitor.
    Serial.print("distance: ");
    Serial.print(distance);
    Serial.println(" cm");
    }
} // ends function


void moveHead() {

  // Head debug output.
  if(HEAD_DEBUG) {
    Serial.print("Head Angle: ");
    Serial.print(HEAD_POSITIONS[currentHeadPosition]);
    Serial.print(" - ");
  }

  // Position head to the current position in the array.
  headServo.write(HEAD_POSITIONS[currentHeadPosition]);


  // Set next head position.
  // Moves servo to the next head position and changes direction when needed.
  
  if(headDirectionClockwise) {
    if(currentHeadPosition >= (NUM_HEAD_POSITIONS -1)) {
      headDirectionClockwise = !headDirectionClockwise;
      currentHeadPosition --;
    }
    else {
      currentHeadPosition++;
   }
  }
  else {
    if(currentHeadPosition <= 0) {
      headDirectionClockwise = !headDirectionClockwise;
      currentHeadPosition++;
    }
  else {
    currentHeadPosition--;
   }
  }
}//ends function

  // Function will check angle of head and take readings according to said angle.
 void grabReadingsPerAngle(){
  double sumDistance = 0;
  // Two readings for front angle, three for side angles.
  
  // Get ultrasonic reading #1.
  if (ULTRASONIC_DEBUG)
  {Serial.println("1st reading");}
  usReadCm();
  // Grab reading and add it to variable.
  sumDistance += currentDistance;

    
  // Get ultrasonic reading #2.
  if (ULTRASONIC_DEBUG)
  {Serial.println("2nd reading");}
  usReadCm();
  // Grab reading and add it to variable.
  sumDistance += currentDistance;
  
  if (currentHeadPosition == 0){ // front angle reading.
    // Divide by 2 for average.
    sumDistance = sumDistance / 2;
    if (AVERAGE_DEBUG)
    {Serial.print("SumDistance: ");
    Serial.println(sumDistance);}
    frontmostReading = sumDistance;
  } else if (currentHeadPosition == 2) { // left most angle reading.
    // Get ultrasonic reading #3
    if (ULTRASONIC_DEBUG)
    {Serial.println("3rd reading");}
    usReadCm();
    //grab reading and add it to variable
    sumDistance += currentDistance;

    //divide by 3 for average
    sumDistance = sumDistance / 3;
    if (AVERAGE_DEBUG)
    {Serial.print("SumDistance: ");
    Serial.println(sumDistance);}
    leftmostReading = sumDistance;
  } else {                          // middle angle reading (unused).
    // Divide by 2 for average.
    sumDistance = sumDistance / 2;
    if (AVERAGE_DEBUG)
    {Serial.print("SumDistance: ");
    Serial.println(sumDistance);}
    midReading = sumDistance;
  }
  
 }// ends function

// function will move wheels given each wheel speed
 void moveWheels(int leftWheel, int rightWheel){
  motors.setSpeeds(rightWheel, leftWheel);

  if (WHEELS_DEBUG) {
   Serial.print("left: ");
   Serial.print(leftWheel);
   Serial.print(" | right: ");
   Serial.println(rightWheel);
  }
  
 } // ends function
 
float leftPID() {

  double currentReading = leftmostReading;

  // Proportional.
  double error = desiredStateLeft - currentReading;
  double proportional = kp * error;

  // Integral.
  kiTotalLeft += error;
  if (kiTotalLeft >= 10) kiTotalLeft = 10;
  if (kiTotalLeft <= -10) kiTotalLeft = -10;
  double integral = ki * kiTotalLeft;

  // Derivative.
  float derivative = kd * (error - priorLeftError);
  priorLeftError = error;

  // PID.
  float pidResult = proportional + integral + derivative;
  return pidResult;
}

float frontPID() {

  double currentReading = frontmostReading;

  // Proportional.
  double error = desiredStateFront - currentReading;
  double proportional = kp * error;

  // Integral.
  kiTotalFront += error;
  if (kiTotalFront >= 10) kiTotalFront = 10;
  if (kiTotalFront <= -10) kiTotalFront = -10;
  double integral = ki * kiTotalFront;

  // Derivative.
  float derivative = kd * (error - priorFrontError);
  priorFrontError = error;

  // PID.
  float pidResult = proportional + integral + derivative;
  return pidResult;
}
