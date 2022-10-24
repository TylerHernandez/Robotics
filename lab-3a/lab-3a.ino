#include <Pololu3piPlus32U4.h>
#include <Servo.h>

using namespace Pololu3piPlus32U4;


// Get angle readings lab

Buzzer buzzer;

Servo headServo;

// switches
const boolean HEAD_DEBUG = true;

// Head servo timing
unsigned long headCm;
unsigned long headPm;
const unsigned long HEAD_MOVEMENT_PERIOD = 500;

// Head servo constants
const int HEAD_SERVO_PIN = 21;
const int NUM_HEAD_POSITIONS = 7;
const int HEAD_POSITIONS[NUM_HEAD_POSITIONS] = {135, 120, 105, 90, 75, 60, 45};

// head servo data
boolean headDirectionClockwise = true;
int currentHeadPosition = 0;

// Ultrasonic timing
unsigned long usCm;
unsigned long usPm;
const unsigned long US_PERIOD = 50; // Time to wait for the 1st US to activate

// Initialize Ultrasonic
const int ECHO_PIN = 22;
const int TRIG_PIN = 4;

// Ultrasonic Maxs
const int MAX_DISTANCE = 100; // (200cm = 2 meters)

// Current US distance reading
int distance = 0;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);

  //initialize head position to start
  headServo.attach(HEAD_SERVO_PIN);
  headServo.write(45);

  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);

  //start delay
  delay(2000);
  buzzer.play("c32");

}


void loop() {
  // perform head movement
  moveHead();
}

void usReadCm() {
  usCm = millis();
  if(usCm > usPm + US_PERIOD) {

    // Clears the TRIG_PIN (set low)
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);


    //Sets the TRIG_PIN HIGH (activate) for 10 microseconds
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
  
    //Reads the ECHO_PIN, returns the sound wave travel time in microseconds
    // note the duration (3800 microseconds) that will allow for reading up max distance supported by the sensor
    long duration = pulseIn(ECHO_PIN, HIGH, 38000);
    //Calculating the distance
    distance = duration * 0.034 / 2; // Time of flight equation: Speed of sound wave divided by 2

    // Apply limits
    if (distance > MAX_DISTANCE) distance = MAX_DISTANCE;
    if (distance == 0) distance = MAX_DISTANCE;

    //Displays the distance on the Serial Monitor
    Serial.print("distance: ");
    Serial.print(distance);
    Serial.println(" cm");


    //update the prevmillis
    usPm = usCm;
  }
}


void moveHead() {
  headCm = millis();
  if(headCm > headPm + HEAD_MOVEMENT_PERIOD) {


  //head debug output
  if(HEAD_DEBUG) {
    Serial.print("Head Angle: ");
    Serial.print(HEAD_POSITIONS[currentHeadPosition]);
    Serial.print(" - ");
  }

  //position head to the current position in the array
  headServo.write( HEAD_POSITIONS[currentHeadPosition]);


  // Set next head position
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

  //reset previous millis
  headPm = headCm;

  // get ultrasonic reading
  usReadCm();

 } // ends beginning if
  
}//ends function
