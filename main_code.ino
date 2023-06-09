/*
  MechEng 706 Base Code

  This code provides basic movement and sensor reading for the MechEng 706 Mecanum Wheel Robot Project

  Hardware:
    Arduino Mega2560 https://www.arduino.cc/en/Guide/ArduinoMega2560
    MPU-9250 https://www.sparkfun.com/products/13762
    Ultrasonic Sensor - HC-SR04 https://www.sparkfun.com/products/13959
    Infrared Proximity Sensor - Sharp https://www.sparkfun.com/products/242
    Infrared Proximity Sensor Short Range - Sharp https://www.sparkfun.com/products/12728
    Servo - Generic (Sub-Micro Size) https://www.sparkfun.com/products/9065
    Vex Motor Controller 29 https://www.vexrobotics.com/276-2193.html
    Vex Motors https://www.vexrobotics.com/motors.html
    Turnigy nano-tech 2200mah 2S https://hobbyking.com/en_us/turnigy-nano-tech-2200mah-2s-25-50c-lipo-pack.html

  Date: 11/11/2016
  Author: Logan Stuart
  Modified: 15/02/2018
  Author: Logan Stuart
*/
#include <Servo.h> //Need for Servo pulse output
#include <math.h>
//#include <SoftwareSerialCom->printh>

#define FAN 26
#define GYRO A2
#define SERVO A3
#define RPT A4
#define LPT A5 
#define TLPT A6
#define TRPT A7
#define BLIR A8
#define BRIR A9
#define FLIR A10
#define FRIR A11

#define CW 1
#define CCW 0
#define FORWARD 1
#define BACKWARDS 0

// #define NO_READ_GYRO  //Uncomment of GYRO is not attached.
// #define NO_HC-SR04 //Uncomment of HC-SR04 ultrasonic ranging sensor is not attached.
// #define NO_BATTERY_V_OK //Uncomment of BATTERY_V_OK if you do not care about battery damage.

// State machine states
enum STATE
{
  INITIALISING,
  TESTING,
  FIRE_FIND,
  HALF_FIND,
  DRIVING,
  EXTINGUISH_FIRE,
  FINISHED,
  RUNNING,
  STOPPED,
  STRAFE,
  PASSED,
  STRAFE_RETURN,
};

// Refer to Shield Pinouts.jpg for pin locations

// Default motor control pins
const byte left_front = 46;
const byte left_rear = 47;
const byte right_rear = 51;
const byte right_front = 50;

// Default ultrasonic ranging sensor pins, these pins are defined my the Shield
const int TRIG_PIN = 48;
const int ECHO_PIN = 49;

// Anything over 400 cm (23200 us pulse) is "out of range". Hit:If you decrease to this the ranging sensor but the timeout is short, you may not need to read up to 4meters.
const unsigned int MAX_DIST = 23200;

Servo left_font_motor;  // create servo object to control Vex Motor Controller 29
Servo left_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_rear_motor; // create servo object to control Vex Motor Controller 29
Servo right_font_motor; // create servo object to control Vex Motor Controller 29
Servo turret_motor;
Servo fan_servo;

int speed_val = 150;
int speed_change;

int firesFound = 0;

// GRYO VALUES
int T = 100;                   // T is the time of one loop, 0.1 sec
float gyroSensitivity = 0.007; // gyro sensitivity unit is (mv/degree/second) get from datasheet
float rotationThreshold = 1.5; // because of gyro drifting, defining rotation angular velocity less
float currentAngle = 0;        // current angle calculated by angular velocity integral
float gyroRate, angularVelocity, angleChange;
float lowestAnglePos = 0;
float IRvolts;

float Toffset = 1.1; // for IR

float speedSlowOffset = 75;

float degSpan = 100;
bool middleSearch = false;
bool leftSearch = false;
bool rightSearch = false;

// Serial Pointer
HardwareSerial *SerialCom;

// Serial Data input pin
//#define BLUETOOTH_RX 10
// Serial Data output pin
//#define BLUETOOTH_TX 11
//SoftwareSerial BluetoothSerial(BLUETOOTH_RX, BLUETOOTH_TX);


//---------------------------------------------------------------------------------------------------------------- SETUP
int pos = 0;
void setup(void)
{
  turret_motor.attach(11);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(FLIR, INPUT);
  pinMode(BLIR, INPUT);
  pinMode(FRIR, INPUT);
  pinMode(BRIR, INPUT);
  pinMode(LPT, INPUT);
  pinMode(RPT, INPUT);
  pinMode(TLPT, INPUT);
  pinMode(TRPT, INPUT);
  pinMode(GYRO, INPUT);
  pinMode(SERVO, OUTPUT);
  pinMode(FAN, OUTPUT);

  // The Trigger pin will tell the sensor to range find
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);

  fan_servo.attach(SERVO);

  // BluetoothSerialCom->printbegin(115200);
  // BluetoothSerialCom->println("Setup....");

  // while(1){
  //   BluetoothSerialCom->println(5, DEC);
  // }

  // Setup the Serial port and pointer, the pointer allows switching the debug info through the USB port(Serial) or Bluetooth port(Serial1) with ease.

  SerialCom = &Serial1;
  SerialCom->begin(115200);
  SerialCom->println("MECHENG706_Base_Code_25/01/2018");
  delay(1000);
  SerialCom->println("Setup....");

  delay(1000); // settling time but no really needed
}

//------------------------------------------------------------------------------------------------------------------------------------- MAIN LOOP
void loop(void) // main loop
{
  static STATE machine_state = INITIALISING;
  // Finite-state machine Code
  // delay(50);

  switch (machine_state)
  {
  case INITIALISING:
    SerialCom->println("INITIALISING");
    machine_state = initialising();
    break;
  case TESTING:
    //SerialCom->println("TESTING");
    machine_state = testing();
    break;
  case FIRE_FIND:
    SerialCom->println("FIRE_FIND");
    machine_state = fire_find();
    break;
  case DRIVING:
    // SerialCom->println("DRIVING");
    machine_state = driving();
    break;
  case EXTINGUISH_FIRE:
    SerialCom->println("EXTINGUISH_FIRE");
    machine_state = extinguish_fire();
    break;
  case FINISHED:
    SerialCom->println("FINISHED");
    machine_state = finished();
    break;
  case RUNNING: // Lipo Battery Volage OK
    SerialCom->println("RUNNING");
    machine_state = running();
    break;
  case STOPPED: // Stop of Lipo Battery voltage is too low, to protect Battery
    SerialCom->println("STOPPED");
    machine_state = stopped();
    break;
  case STRAFE:
    SerialCom->println("STRAFE");
    machine_state = strafe();
    break;
  case PASSED:
    SerialCom->println("PASSED");
    machine_state = passed();
    break;
  case STRAFE_RETURN:
    SerialCom->println("STRAFE_RETURN");
    machine_state = strafe_return();
    break;
  case HALF_FIND:
    SerialCom->println("HALF_FIND");
    machine_state = half_find();
    break;
  };
}

//------------------------------------------------------------------------------------------------------------------------------------- STATES
STATE initialising()
{
  // initialising
  SerialCom->println("INITIALISING....");
  delay(1000); // One second delay to see the serial string "INITIALISING...."
  // SerialCom->println("Enabling Motors...");
  enable_motors();
  SerialCom->println("RUNNING STATE...");

  for(int i = 0; i <= 120; i++)
  {
    turnServo(i);
    delay(10);
  }

  turnServo(60);

  delay(10);

  return FIRE_FIND;
}

//-------------------------------------------------------------------------------------------------------------------------------------


STATE half_find() {

  //re-align servo
  turnServo(60);

  middleSearch = true;

  if(middleSearch){
    //turn half of the span to starting point
    turnDeg(CCW, (degSpan/2));
  }
  // if(leftSearch){
  //   //turn half of the span to starting point
  //   turnDeg(CCW, (degSpan - 10));
  // }
  // if(rightSearch){
  //   //turn half of the span to starting point
  //   turnDeg(CW, (degSpan - 10));
  // }
  

  float currentLeftLightReading, currentRightLightReading, averagedLightReading;
  float highestLightReading = 0.0;  
  float highestLightAngle = 0.0;
  bool turningCW = true;

  cw();
  currentAngle = 1.0; //offset added to account for drift while reading 0

  // SerialCom->print("At ");
  // SerialCom->println(currentAngle);

  delay(100);

  //find the light
  while(turningCW){

    gyroUpdate();

    //read current ptr values
    currentLeftLightReading = topLeftPT();
    currentRightLightReading = topRightPT();    
    averagedLightReading = averagePTR(currentLeftLightReading, currentRightLightReading);

    // SerialCom->print("At ");
    // SerialCom->print(currentAngle);
    // SerialCom->print(" deg: ");
    // SerialCom->println(averagedLightReading);

    if(averagedLightReading > highestLightReading){
      highestLightReading = averagedLightReading;
      highestLightAngle = currentAngle;
    }

    //turn specified degree span
    if( currentAngle >= (degSpan + 1.0) ){
      turningCW = false;
      stop();
    }

    delay (T); 
  }
  // SerialCom->print("Highest Light Angle: ");
  // SerialCom->println(highestLightAngle);

  float error = abs(highestLightAngle - currentAngle);
  // SerialCom->print("Current Error: ");
  // SerialCom->println(error);

  speedSlowOffset = 85;

  while(error >= 1)
  {
    ccwSlower();
    
    // SerialCom->print("Current Error: ");
    // SerialCom->println(error);

    gyroUpdate();

    error = abs(highestLightAngle - currentAngle);

    delay (T); 
  }

  // SerialCom->println("Facing fire");

  stop();

  return DRIVING;
}

STATE fire_find() {

  //re-align servo
  turnServo(60);

  float currentLeftLightReading, currentRightLightReading, averagedLightReading;
  float highestLightReading = 0.0;  
  float highestLightAngle = 0.0;
  bool turningCW = true;

  cw();
  currentAngle = 1.0; //offset added to account for drift while reading 0

  //find the light
  while(turningCW){

    gyroUpdate();

    //read current ptr values
    currentLeftLightReading = topLeftPT();
    currentRightLightReading = topRightPT();    
    averagedLightReading = averagePTR(currentLeftLightReading, currentRightLightReading);

    // SerialCom->print("At ");
    // SerialCom->print(currentAngle);
    // SerialCom->print(" deg: ");
    // SerialCom->println(averagedLightReading);

    if(averagedLightReading > highestLightReading){
      highestLightReading = averagedLightReading;
      highestLightAngle = currentAngle;
    }

    //turn almost 360 degrees
    if( currentAngle >= 354.0 ){
      turningCW = false;
      stop();
    }

    delay (T); 
  }

  // SerialCom->print("Highest Light Angle: ");
  // SerialCom->println(highestLightAngle);

  if(highestLightAngle <= 180){
    cw();
   // SerialCom->println("In the first 180 degrees");
  }
  else{
    ccw();
   // SerialCom->println("In the last 180 degrees");
  }
  
  float error = abs(highestLightAngle - currentAngle);
  // SerialCom->print("Current Error: ");
  // SerialCom->println(error);

  while(error >= 2.0)
  {
    // SerialCom->print("Current Error: ");
    // SerialCom->println(error);

    gyroUpdate();

    error = abs(highestLightAngle - currentAngle);

    delay (T); 
  }

  // SerialCom->println("Facing fire");

  stop();

  return DRIVING;
}

//-------------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------
int lf_drive;
int lr_drive;
int rf_drive;
int rr_drive;
int lf_strafe;
int lr_strafe;
int rf_strafe;
int rr_strafe;
unsigned long strafe_time_start;
unsigned long strafe_time;
unsigned long strafe_back_time_start;
unsigned long strafe_back_time;
unsigned long passing_time_start;
unsigned long passing_time;
bool timer_bool;
bool left;
bool object_left   = false;
bool object_right  = false;
bool object_middle = false;
bool second_fire   = false;

float left_IR;
float right_IR;
float back_left_IR;
float back_right_IR;
float mkUltra;
float fire_sensor;
float leftS;
float rightS;
float topLeftS;
float topRightS;

//Change these values for tuning
const float distVolt = 3.0;
const float ultraDist = 6.0;
const float fire_cutoff   = 1.0;
const float dist_to_fire  = 7.0;
const float passed_cutoff = 25.0; //sensors on side for driving past
const float passing_dist  = 0.0; //sensors on front for strafing past object
const int   sped          = 150; // speed values

void readSensor(){
  //Read sensors
  leftS         = leftPT();
  rightS        = rightPT();
  topLeftS      = topLeftPT();
  topRightS     = topRightPT();
  left_IR       = frontLeftIR();
  right_IR      = frontRightIR();
  back_left_IR  = backLeftIR();
  back_right_IR = backRightIR();
  mkUltra       = ultrasonic();
  fire_sensor   = averagePTR(topLeftPT(),topRightPT());//ave;//(topLeftPT() + topRightPT())/2;
}
//This functions drives straigh untill there is an object
STATE driving(){

  readSensor();

  //INTERPRETING SENSOR READINGS

  //If object left set false
  if (left_IR >= distVolt){
    object_left = true;
    stop();
    turnServo(100);
    delay(500);
  } else {
    object_left = false;
  }

  //If object right set true
  if (right_IR >= distVolt){
    object_right = true;
    stop();
    turnServo(0);
    delay(500);
  } else {
    object_right = false;
  }

  if (mkUltra <= ultraDist){
    object_middle = true;
    stop();
    turnServo(60);
    delay(500);
  } else {
    object_middle = false;
  }

  //EXIT CONDITION

  readSensor();

  //TUNED at 7.4V

  //left
  if ((object_left) && ((topLeftS >= 3.7)||(topRightS >= 3.7))){
    stop();
    SerialCom->print(" topleftS: ");
    SerialCom->print(topLeftS);
    SerialCom->print("topRightS: ");
    SerialCom->print(topRightS);
    SerialCom->print(" -Left exit-");
    return EXTINGUISH_FIRE; 
  }

  //middle
  if((object_middle) && ((topLeftS >= 4.0)||(topRightS >= 4.0))){
    stop();
    SerialCom->print(" topleftS: ");
    SerialCom->print(topLeftS);
    SerialCom->print("topRightS: ");
    SerialCom->print(topRightS);
    SerialCom->print(" -Mid exit- ");
    return EXTINGUISH_FIRE; 
  }
  
  //right
  if((object_right) && ((topLeftS >= 3.7)||(topRightS >= 3.7))){
    stop();
    SerialCom->print(" topleftS: ");
    SerialCom->print(topLeftS);
    SerialCom->print("topRightS: ");
    SerialCom->print(topRightS);
    SerialCom->print("Right exit");
    return EXTINGUISH_FIRE; 
  }

  delay(50);
  
  //OBJECT AVOIDANCE

  if(object_left == true && object_right == true){
    //Cry
    //maybe turn around
  } else if(object_middle && !object_left && !object_right){
    //SerialCom->println("Middle");
    left = true;
    leftSearch = true;
    rightSearch = false;
    strafe_time_start = millis();
    return STRAFE;
  } else if (object_middle && object_left && !object_right){
    //SerialCom->println("Middle & Left");
    left = true;
    leftSearch = true;
    rightSearch = false;
    strafe_time_start = millis();
    return STRAFE;
  } else if (object_middle && !object_left && object_right){
    //SerialCom->println("Middle & Right");
    left = false;
    leftSearch = false;
    rightSearch = true;
    strafe_time_start = millis();
    return STRAFE;
  } else if(object_left && !object_right){
    //SerialCom->println("Left");
    left = true; //If object lft
    leftSearch = true;
    rightSearch = false;
    strafe_time_start = millis();
    return STRAFE;
  } else if(!object_left && object_right){
    //SerialCom->println("Right");
    left = false;
    leftSearch = false;
    rightSearch = true;
    strafe_time_start = millis();
    return STRAFE;
  } 

  //if all okay drive straight
  left_font_motor.writeMicroseconds(1500 + sped); // left front
  left_rear_motor.writeMicroseconds(1500 + sped); // left rear
  right_font_motor.writeMicroseconds(1500 - sped); // rear right
  right_rear_motor.writeMicroseconds(1500 - sped + 15); // front right

  return DRIVING;
}

STATE testing()
{
 //If object left set false
  if (left_IR >= distVolt){
    object_left = true;
    turnServo(100);
  } else {
    object_left = false;
  }

  //If object right set true
  if (right_IR >= distVolt){
    object_right = true;
    turnServo(0);
  } else {
    object_right = false;
  }

  if (mkUltra <= ultraDist){
    object_middle = true;
    turnServo(60);
  } else {
    object_middle = false;
  }

  readSensor();

  //TUNED at 7.4V

  //left
  if ((object_left) && ((topLeftS >= 4.0)||(topRightS >= 4.0))){
    stop();
    SerialCom->print(" topleftS: ");
    SerialCom->print(topLeftS);
    SerialCom->print("topRightS: ");
    SerialCom->print(topRightS);
    SerialCom->println(" -Left exit-");
  }

  //middle
  if((object_middle) && ((topLeftS >= 4.5)||(topRightS >= 4.5))){
    stop();
    SerialCom->print(" topleftS: ");
    SerialCom->print(topLeftS);
    SerialCom->print("topRightS: ");
    SerialCom->print(topRightS);
    SerialCom->println(" -Mid exit- ");
  }
  
  //right
  if((object_right) && ((topLeftS >= 4.0)||(topRightS >= 4.0))){
    stop();
    SerialCom->print(" topleftS: ");
    SerialCom->print(topLeftS);
    SerialCom->print("topRightS: ");
    SerialCom->print(topRightS);
    SerialCom->println("Right exit");
  }
  delay(10);

  return TESTING;
}

//This state strafes the robot until there is no object
STATE strafe(){
  //READ THE SENSOR
  readSensor();

  int offset;
  // float sensor;
  int dir;
  if(left){
    dir = 1;
    offset = -20;
    // sensor = left_IR;
    strafe_right();
  } else{
    dir = -1;
    offset = 13;
    // sensor = right_IR;
    strafe_left();
  }

  strafe_time = millis() - strafe_time_start;
  
  unsigned long timeness;
  if(object_middle && (object_left || object_right)){
    timeness = 1700;
  } else if (object_middle) {
    timeness = 1500;
  } else {
    timeness = 900;
  }

  //SerialCom->print("Object time:");
  //SerialCom->println(timeness);

  if(strafe_time >= timeness)
  {
    passing_time_start = millis();
    stop();
    return PASSED;
  }

  //TODO change values to strafe
  //left_font_motor.writeMicroseconds(1500 + 150*dir); // left front
 /// left_rear_motor.writeMicroseconds(1500 - 150*dir); // left rear
 /// right_font_motor.writeMicroseconds(1500 + 150*dir - offset); // rear right
 ///// right_rear_motor.writeMicroseconds(1500- 150*dir); // front right
  return STRAFE;
}

//This function drives straight until we have passed object
//TODO: make sure to not hit other objects
STATE passed(){
  //if all okay drive straight
  // left_font_motor.writeMicroseconds(1500 + sped); // left front
  // left_rear_motor.writeMicroseconds(1500 + sped); // left rear
  // right_font_motor.writeMicroseconds(1500- sped); // rear right
  // right_rear_motor.writeMicroseconds(1500- sped + 15); // front right
  forward();
  readSensor();

  passing_time = millis() - passing_time_start;
  if(passing_time >= 1200)
  {
    //passing_time_start = millis();
    strafe_back_time_start = millis();
    stop();
    // return STRAFE_RETURN;

    degSpan = 60;
    return HALF_FIND;
  }
  /*
  //If left read left sensor else read right.
  if(left){
    //If we have passed object save time and strafe back
    if(back_left_IR >= passed_cutoff){
      stop();
      strafe_back_time_start = millis();
      return STRAFE_RETURN;
    }
  } else {
    //If we have passed object save time and strafe back
    if(back_right_IR >= passed_cutoff){
      stop();
      strafe_back_time_start = millis();
      return STRAFE_RETURN;
    }
  } */
  return PASSED;
}

//This funciton strafes the robot back until the time = 0;
STATE strafe_return(){

  unsigned long time_left = millis() - strafe_back_time_start;
  strafe_back_time = millis() - strafe_back_time_start;


  unsigned long timeness;
  if(object_middle && (object_left || object_right)){
    timeness = 1700;
  } else if (object_middle) {
    timeness = 1500;
  } else {
    timeness = 600;
  }


  if(strafe_back_time >= timeness)
  {
    //passing_time_start = millis();
    stop();
    return DRIVING;
  }

  int offset;
  // float sensor;
  int dir;
  if(!left){
    dir = 1;
    offset = 13;
    // sensor = left_IR;
    strafe_right();
  } else{
    dir = -1;
    offset = -20;
    // sensor = right_IR;
    strafe_left();
  }
  
  // left_font_motor.writeMicroseconds(1500 + 150*dir); // left front
  // left_rear_motor.writeMicroseconds(1500 - 150*dir); // left rear
  // right_font_motor.writeMicroseconds(1500 + 150*dir - offset); // rear right
  // right_rear_motor.writeMicroseconds(1500- 150*dir); // front right

  return STRAFE_RETURN;
}

//-------------------------------------------------------------------------------------------------------------------------------------
STATE extinguish_fire() {

  int servoAngle, highestLightAngle;
  float currentLeftLightReading, currentRightLightReading, averagedLightReading;
  float highestLeftLightReading = 0;
  float highestRightLightReading = 0;
  float averagedLightReadings[120];

  //turning servo its maximum angle span (0-120 deg) to detect a light
  //the angle at which the maximum light is detected at is found
  for (int i = 0; i <= 5; i++) {
    currentLeftLightReading = topLeftPT();
    currentRightLightReading = topRightPT();
  }

  //turn servo from 0 to 120 degrees with 1 degree increments
  for (servoAngle = 0; servoAngle <= 120; servoAngle++) {
    turnServo(servoAngle);
    currentLeftLightReading = topLeftPT();
    currentRightLightReading = topRightPT();

    averagedLightReadings[servoAngle] = averagePTR(currentLeftLightReading, currentRightLightReading);

    delay(10);
  }

  //filter out random readings & remove first 10 readings
  for(int i = 0; i<= 120; i++){
    if( (averagedLightReadings[i] <= 0.1) || (i <= 10))
    {
      averagedLightReadings[i] = 0;
    }
  }

  //for(int i = 0; i<=120; i++){
    //SerialCom->print("At ");
    // SerialCom->print(i);
    //SerialCom->print(" degrees: ");
    //SerialCom->println(averagedLightReadings[i]);
  //}   

  float highestValue = 0;
  float highestIndex = 0;
  float lowerIndex = 0;
  float higherIndex = 0;

  for(int i = 0; i <= 120; i++){
    if(averagedLightReadings[i] > highestValue){
      highestValue = averagedLightReadings[i];
      highestIndex = i;
    }
  }  

  lowerIndex = highestIndex - 10;
  constrain(lowerIndex, 0, 120);

  higherIndex = highestIndex + 10;
  constrain(higherIndex, 0, 120);

  for(int i = lowerIndex; i <= higherIndex; i++){
    averagedLightReadings[i] = 0;      
  }

  float secondHighestValue = 0;
  float secondHighestIndex = 0;

  for(int i = 0; i <= 120; i++){
    if(averagedLightReadings[i] > secondHighestValue){
      secondHighestValue = averagedLightReadings[i];
      secondHighestIndex = i;
    }
  }  

  //middle of plateau, adjusted due to the effects of the averager
  highestLightAngle = abs(floor((highestIndex + secondHighestIndex)/2));

  //offset due to averager, proportional to how far away from 120
  float averagerOffset = 5*(1 - (highestLightAngle/120));

  highestLightAngle += averagerOffset;
  constrain(highestLightAngle, 0, 120);  // 120 max degree angle

  //move servo to highest location
  turnServo(highestLightAngle);

  averagedLightReading = averagePTR(currentLeftLightReading, currentRightLightReading);
  float initialLightReading = averagePTR(currentLeftLightReading, currentRightLightReading);

  //run fan while fire is detected
  //exits loop if the current light reading is less than half the initial value indiciating the fire has been put out
  while(averagedLightReading >= (initialLightReading * 0.5)) {
    //turn on fan
    digitalWrite(FAN, HIGH);
  
    //get current light reading
    currentLeftLightReading = topLeftPT();
    currentRightLightReading = topRightPT();
    averagedLightReading = averagePTR(currentLeftLightReading, currentRightLightReading);
  }

  firesFound++;

  //turn off fan
  digitalWrite(FAN, LOW);
  
  turnServo(60);  //realign servo

  if(firesFound >= 2){ 
    return FINISHED; 
  } else{ 
    reverse();
    delay(400);   
    stop();    
    return FIRE_FIND; 
  }
}

STATE finished(){

  stop();

  return FINISHED;
}

STATE running(){

  static unsigned long previous_millis;

  read_serial_command();
  fast_flash_double_LED_builtin();

  if (millis() - previous_millis > 500)
  { // Arduino style 500ms timed execution statement
    previous_millis = millis();

    SerialCom->println("RUNNING---------");
    speed_change_smooth();
    Analog_Range_A4();

#ifndef NO_READ_GYRO
    GYRO_reading();
#endif

#ifndef NO_HC - SR04
    HC_SR04_range();
#endif

#ifndef NO_BATTERY_V_OK
    if (!is_battery_voltage_OK())
      return STOPPED;
#endif

    turret_motor.write(pos);

    if (pos == 0)
    {
      pos = 45;
    }
    else
    {
      pos = 0;
    }
  }

  return RUNNING;
}

// Stop if Lipo Battery voltage is too low, to protect Battery
STATE stopped(){
  static byte counter_lipo_voltage_ok;
  static unsigned long previous_millis;
  int Lipo_level_cal;
  disable_motors();
  slow_flash_LED_builtin();

  if (millis() - previous_millis > 500)
  { // print massage every 500ms
    previous_millis = millis();
    SerialCom->println("STOPPED---------");

#ifndef NO_BATTERY_V_OK
    // 500ms timed if statement to check lipo and output speed settings
    if (is_battery_voltage_OK())
    {
      SerialCom->print("Lipo OK waiting of voltage Counter 10 < ");
      SerialCom->println(counter_lipo_voltage_ok);
      counter_lipo_voltage_ok++;
      if (counter_lipo_voltage_ok > 10)
      { // Making sure lipo voltage is stable
        counter_lipo_voltage_ok = 0;
        enable_motors();
        SerialCom->println("Lipo OK returning to RUN STATE");
        return RUNNING;
      }
    }
    else
    {
      counter_lipo_voltage_ok = 0;
    }
#endif
  }
  return STOPPED;
}

//------------------------------------------------------------------------------------------------------------------------------------- FUNCTIONS
void printValues() {

  float lpt = leftPT();
  float rpt = rightPT();
  float tlpt = topLeftPT();
  float trpt = topRightPT();
  float flir = frontLeftIR();
  float frir = frontRightIR();
  float blir = backLeftIR();
  float brir = backRightIR();
  float usc = ultrasonic();

  // SerialCom->print(" LPT: ");
  // SerialCom->print(lpt);
  // SerialCom->print(" RPT: ");
  // SerialCom->print(rpt);
  // SerialCom->print(" TLPT: ");
  // SerialCom->print(tlpt);
  // SerialCom->print(" TRPT: ");
  // SerialCom->print(trpt);
  SerialCom->print(" FLIR: ");
  SerialCom->print(flir);
  SerialCom->print(" FRIR: ");
  SerialCom->print(frir);
  //SerialCom->print(" BLIR: ");
  // SerialCom->print(blir);
  // SerialCom->print(" BRIR: ");
  // SerialCom->print(brir);
  SerialCom->print(" USC: ");
  SerialCom->print(usc);
  SerialCom->println(" ");
  
}

//---------------------------------------------------------------------------------------------------------------- TURN SERVO
void turnServo(float deg)
{
  //deg-=30;

  constrain(deg, 0, 120); // 120 max degree angle
  fan_servo.write(deg);
}

float PTRValues1 = 0;
float PTRValues2 = 0;
float PTRValues3 = 0;
float PTRValues4 = 0;
float PTRValues5 = 0;
float PTRValues6 = 0;
float PTRValues7 = 0;
float PTRValues8 = 0;
float PTRValues9 = 0;
float PTRValues10 = 0;
float averagePTR(float left, float right) {

  PTRValues1 = (left + right)/2;
  PTRValues2 = PTRValues1;
  PTRValues3 = PTRValues2;
  PTRValues4 = PTRValues3;
  PTRValues5 = PTRValues4;
  PTRValues6 = PTRValues5;
  PTRValues7 = PTRValues6;
  PTRValues8 = PTRValues7;
  PTRValues9 = PTRValues8;
  PTRValues10 = PTRValues9;

  return (PTRValues1 + PTRValues2 + PTRValues3 + PTRValues4 + PTRValues5 + PTRValues6 + PTRValues7 + PTRValues8 + PTRValues9 + PTRValues10) / 10;  //averaged values
}

//---------------------------------------------------------------------------------------------------------------- FRONT LEFT IR
float FLIRValues1 = 0;
float FLIRValues2 = 0;
float FLIRValues3 = 0;
float FLIRValues4 = 0;
float FLIRValues5 = 0;
float frontLeftIR() {

  FLIRValues1 = analogRead(FLIR) * 5.0 / 1024.0;
  FLIRValues2 = FLIRValues1;
  FLIRValues3 = FLIRValues2;
  FLIRValues4 = FLIRValues3;
  FLIRValues5 = FLIRValues4;

  IRvolts = (FLIRValues1 + FLIRValues2 + FLIRValues3 + FLIRValues4 + FLIRValues5) / 5;  //averaged values
  return IRvolts;
  //return (IRvolts < 0.3) ? 0 : (1 / ((IRvolts - 0.0587) / 11.159));
}

//---------------------------------------------------------------------------------------------------------------- FRONT RIGHT IR
float FRIRValues1 = 0;
float FRIRValues2 = 0;
float FRIRValues3 = 0;
float FRIRValues4 = 0;
float FRIRValues5 = 0;
float frontRightIR() {

  FRIRValues1 = analogRead(FRIR) * 5.0 / 1024.0;
  FRIRValues2 = FRIRValues1;
  FRIRValues3 = FRIRValues2;
  FRIRValues4 = FRIRValues3;
  FRIRValues5 = FRIRValues4;

  IRvolts = (FRIRValues1 + FRIRValues2 + FRIRValues3 + FRIRValues4 + FRIRValues5) / 5;  //averaged values
  return IRvolts;
  //return (IRvolts < 0.3) ? 0 : (1 / ((IRvolts + 0.0413) / 11.482));
}

//---------------------------------------------------------------------------------------------------------------- BACK LEFT IR
float BLIRValues1 = 0;
float BLIRValues2 = 0;
float BLIRValues3 = 0;
float BLIRValues4 = 0;
float BLIRValues5 = 0;
float backLeftIR() {

  BLIRValues1 = analogRead(BLIR) * 5.0 / 1024.0;
  BLIRValues2 = BLIRValues1;
  BLIRValues3 = BLIRValues2;
  BLIRValues4 = BLIRValues3;
  BLIRValues5 = BLIRValues4;

  IRvolts = (BLIRValues1 + BLIRValues2 + BLIRValues3 + BLIRValues4 + BLIRValues5) / 5;  //averaged values
  return IRvolts;
  //return (IRvolts < 0.4) ? 0 : (1 / ((IRvolts - 0.0804) / 23.929));
}

//---------------------------------------------------------------------------------------------------------------- BACK RIGHT IR
float BRIRValues1 = 0;
float BRIRValues2 = 0;
float BRIRValues3 = 0;
float BRIRValues4 = 0;
float BRIRValues5 = 0;
float backRightIR() {

  BRIRValues1 = analogRead(BRIR) * 5.0 / 1024.0;
  BRIRValues2 = BRIRValues1;
  BRIRValues3 = BRIRValues2;
  BRIRValues4 = BRIRValues3;
  BRIRValues5 = BRIRValues4;

  IRvolts = (BRIRValues1 + BRIRValues2 + BRIRValues3 + BRIRValues4 + BRIRValues5) / 5;  //averaged values
  return IRvolts;
  //return (IRvolts < 0.4) ? 0 : (1 / ((IRvolts - 0.0704) / 23.018));
}

//---------------------------------------------------------------------------------------------------------------- LEFT PHOTOTRANSISTOR
float leftPT() {

  IRvolts = analogRead(LPT) * 5.0 / 1024.0;

  return IRvolts;
  //return (IRvolts < 0.4) ? 0 : (1 / ( ( IRvolts - 0.0704) / 23.018)) + 7.35;
}

//---------------------------------------------------------------------------------------------------------------- RIGHT PHOTOTRANSISTOR
float rightPT() {

  IRvolts = analogRead(RPT) * 5.0 / 1024.0;

  return IRvolts;
  //return (IRvolts < 0.4) ? 0 : (1 / ( ( IRvolts - 0.0704) / 23.018)) + 7.35;
}

//---------------------------------------------------------------------------------------------------------------- TOP LEFT PHOTOTRANSISTOR
float TLPTValues1 = 0;
float TLPTValues2 = 0;
float TLPTValues3 = 0;
float TLPTValues4 = 0;
float TLPTValues5 = 0;
float topLeftPT() {

  TLPTValues1 = analogRead(TLPT) * 5.0 / 1024.0;
  TLPTValues2 = TLPTValues1;
  TLPTValues3 = TLPTValues2;
  TLPTValues4 = TLPTValues3;
  TLPTValues5 = TLPTValues4;

  IRvolts = (TLPTValues1 + TLPTValues2 + TLPTValues3 + TLPTValues4 + TLPTValues5) / 5;  //averaged values

  return (IRvolts < 0.12) ? 0 : IRvolts;
}

//---------------------------------------------------------------------------------------------------------------- TOP RIGHT PHOTOTRANSISTOR
float TRPTValues1 = 0;
float TRPTValues2 = 0;
float TRPTValues3 = 0;
float TRPTValues4 = 0;
float TRPTValues5 = 0;
float topRightPT() {

  TRPTValues1 = analogRead(TRPT) * 5.0 / 1024.0;
  TRPTValues2 = TRPTValues1;
  TRPTValues3 = TRPTValues2;
  TRPTValues4 = TRPTValues3;
  TRPTValues5 = TRPTValues4;

  IRvolts = (TRPTValues1 + TRPTValues2 + TRPTValues3 + TRPTValues4 + TRPTValues5) / 5;  //averaged values

  return (IRvolts < 0.12) ? 0 : IRvolts;
}

//---------------------------------------------------------------------------------------------------------------- ULTRASONIC
float USValues1 = 0;
float USValues2 = 0;
float USValues3 = 0;
float USValues4 = 0;
float USValues5 = 0;
float ultrasonic() {

  unsigned long t1;
  unsigned long t2;
  unsigned long pulse_width;
  float cm;
  float inches;

  // Hold the trigger pin high for at least 10 us
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Wait for pulse on echo pin
  t1 = micros();
  while (digitalRead(ECHO_PIN) == 0) {
    t2 = micros();
    pulse_width = t2 - t1;
    if (pulse_width > (MAX_DIST + 1000)) {
      //SerialCom->println("HC-SR04: NOT found");
      return 300.0;
    }
  }

  // Measure how long the echo pin was held high (pulse width)
  // Note: the micros() counter will overflow after ~70 min

  t1 = micros();
  while (digitalRead(ECHO_PIN) == 1) {
    t2 = micros();
    pulse_width = t2 - t1;
    if (pulse_width > (MAX_DIST + 1000)) {
      //SerialCom->println("HC-SR04: Out of range");
      return;
    }
  }

  t2 = micros();
  pulse_width = t2 - t1;

  // Calculate distance in centimeters and inches. The constants
  // are found in the datasheet, and calculated from the assumed speed
  //of sound in air at sea level (~340 m/s).

  //Average the signal by 5
  USValues1 = pulse_width / 58.0;
  USValues2 = USValues1;  // IRValues1;
  USValues3 = USValues2;  //IRValues2;
  USValues4 = USValues3;  //IRValues3;
  USValues5 = USValues4;  //IRValues4;

  cm = (USValues1 + USValues2 + USValues3 + USValues4 + USValues5) / 5;  //averaged values

  //cm = pulse_width / 58.0;
  inches = pulse_width / 148.0;

  // Print out results
  if (pulse_width > MAX_DIST) {
    //SerialCom->println("HC-SR04: Out of range");
  } else {
    // SerialCom->print("HC-SR04:");
    // SerialCom->print(cm);
    // SerialCom->println("cm");

    return cm;
  }
}

void gyroUpdate() {
  gyroRate = ((analogRead(GYRO) * 5.0) / 1024.0) - 2.5;  // 2.5V = resting value offset

  // read out voltage divided the gyro sensitivity to calculate the angular velocity
  angularVelocity = gyroRate / gyroSensitivity;  // from Data Sheet, gyroSensitivity is 0.007 V/dps

  // if the angular velocity is less than the threshold, ignore it
  if (angularVelocity >= rotationThreshold || angularVelocity <= -rotationThreshold) {
    // we are running a loop in T (of T/1000 second).
    angleChange = (angularVelocity / (1000.0 / T)) * Toffset * 0.955;
    currentAngle += angleChange;
  }

  // keep the angle between 0-360
  if (currentAngle < 0) {
    currentAngle += 360.0;
  } else if (currentAngle > 359) {
    currentAngle -= 360.0;
  }
}

void objectAvoid(int direction){
  return;
}

//---------------------------------------------------------------------------------------------------------------- TURN DEG
void turnDeg(int directionCW, float deg){
  // add initial offset as gyro value drifts slightly, could drift to 360 degrees

  if (directionCW)
  {
    currentAngle = 5.0;
    deg += 5.0;
  }
  else
  {
    currentAngle = 355.0;
    deg = 355.0 - deg;
  }

  directionCW ? cw() : ccw();

  while (1)
  {
    gyroUpdate();

    // keep the angle between 0-360
    if (currentAngle < 0)
    {
      currentAngle += 360.0;
    }
    else if (currentAngle > 359)
    {
      currentAngle -= 360.0;
    }

    // exit condition
    if ((currentAngle >= deg) && directionCW)
    {
      stop();
      return;
    }
    else if ((currentAngle <= deg) && !directionCW)
    {
      stop();
      return;
    }

    // control the time per loop
    delay(T);
  }
}

void fast_flash_double_LED_builtin()
{
  static byte indexer = 0;
  static unsigned long fast_flash_millis;
  if (millis() > fast_flash_millis)
  {
    indexer++;
    if (indexer > 4)
    {
      fast_flash_millis = millis() + 700;
      digitalWrite(LED_BUILTIN, LOW);
      indexer = 0;
    }
    else
    {
      fast_flash_millis = millis() + 100;
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
  }
}

void slow_flash_LED_builtin()
{
  static unsigned long slow_flash_millis;
  if (millis() - slow_flash_millis > 2000)
  {
    slow_flash_millis = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}

void speed_change_smooth()
{
  speed_val += speed_change;
  if (speed_val > 1000)
    speed_val = 1000;
  speed_change = 0;
}

#ifndef NO_BATTERY_V_OK
boolean is_battery_voltage_OK()
{
  static byte Low_voltage_counter;
  static unsigned long previous_millis;

  int Lipo_level_cal;
  int raw_lipo;
  // the voltage of a LiPo cell depends on its chemistry and varies from about 3.5V (discharged) = 717(3.5V Min) https://oscarliang.com/lipo-battery-guide/
  // to about 4.20-4.25V (fully charged) = 860(4.2V Max)
  // Lipo Cell voltage should never go below 3V, So 3.5V is a safety factor.
  raw_lipo = analogRead(A0);
  Lipo_level_cal = (raw_lipo - 717);
  Lipo_level_cal = Lipo_level_cal * 100;
  Lipo_level_cal = Lipo_level_cal / 143;

  if (Lipo_level_cal > 0 && Lipo_level_cal < 160)
  {
    previous_millis = millis();
    SerialCom->print("Lipo level:");
    SerialCom->print(Lipo_level_cal);
    SerialCom->print("%");
    // SerialCom->print(" : Raw Lipo:");
    // SerialCom->println(raw_lipo);
    SerialCom->println("");
    Low_voltage_counter = 0;
    return true;
  }
  else
  {
    if (Lipo_level_cal < 0)
      SerialCom->println("Lipo is Disconnected or Power Switch is turned OFF!!!");
    else if (Lipo_level_cal > 160)
      SerialCom->println("!Lipo is Overchanged!!!");
    else
    {
      SerialCom->println("Lipo voltage too LOW, any lower and the lipo with be damaged");
      SerialCom->print("Please Re-charge Lipo:");
      SerialCom->print(Lipo_level_cal);
      SerialCom->println("%");
    }

    Low_voltage_counter++;
    if (Low_voltage_counter > 5)
      return false;
    else
      return true;
  }
}
#endif

#ifndef NO_HC - SR04
void HC_SR04_range()
{
  unsigned long t1;
  unsigned long t2;
  unsigned long pulse_width;
  float cm;
  float inches;

  // Hold the trigger pin high for at least 10 us
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Wait for pulse on echo pin
  t1 = micros();
  while (digitalRead(ECHO_PIN) == 0)
  {
    t2 = micros();
    pulse_width = t2 - t1;
    if (pulse_width > (MAX_DIST + 1000))
    {
      SerialCom->println("HC-SR04: NOT found");
      return;
    }
  }

  // Measure how long the echo pin was held high (pulse width)
  // Note: the micros() counter will overflow after ~70 min

  t1 = micros();
  while (digitalRead(ECHO_PIN) == 1)
  {
    t2 = micros();
    pulse_width = t2 - t1;
    if (pulse_width > (MAX_DIST + 1000))
    {
      SerialCom->println("HC-SR04: Out of range");
      return;
    }
  }

  t2 = micros();
  pulse_width = t2 - t1;

  // Calculate distance in centimeters and inches. The constants
  // are found in the datasheet, and calculated from the assumed speed
  // of sound in air at sea level (~340 m/s).
  cm = pulse_width / 58.0;
  inches = pulse_width / 148.0;

  // Print out results
  if (pulse_width > MAX_DIST)
  {
    SerialCom->println("HC-SR04: Out of range");
  }
  else
  {
    SerialCom->print("HC-SR04:");
    SerialCom->print(cm);
    SerialCom->println("cm");
  }
}
#endif

void Analog_Range_A4()
{
  SerialCom->print("Analog Range A4:");
  SerialCom->println(analogRead(A4));
}

#ifndef NO_READ_GYRO
void GYRO_reading()
{
  SerialCom->print("GYRO GYRO:");
  SerialCom->println(analogRead(GYRO));
}
#endif

// Serial command pasing
void read_serial_command()
{
  if (SerialCom->available())
  {
    char val = SerialCom->read();
    SerialCom->print("Speed:");
    SerialCom->print(speed_val);
    SerialCom->print(" ms ");

    // Perform an action depending on the command
    switch (val)
    {
    case 'w': // Move Forward
    case 'W':
      forward();
      SerialCom->println("Forward");
      break;
    case 's': // Move Backwards
    case 'S':
      reverse();
      SerialCom->println("Backwards");
      break;
    case 'q': // Turn Left
    case 'Q':
      strafe_left();
      SerialCom->println("Strafe Left");
      break;
    case 'e': // Turn Right
    case 'E':
      strafe_right();
      SerialCom->println("Strafe Right");
      break;
    case 'a': // Turn Right
    case 'A':
      ccw();
      SerialCom->println("ccw");
      break;
    case 'd': // Turn Right
    case 'D':
      cw();
      SerialCom->println("cw");
      break;
    case '-': // Turn Right
    case '_':
      speed_change = -100;
      SerialCom->println("-100");
      break;
    case '=':
    case '+':
      speed_change = 100;
      SerialCom->println("+");
      break;
    default:
      stop();
      SerialCom->println("stop");
      break;
    }
  }
}

//----------------------Motor moments------------------------
// The Vex Motor Controller 29 use Servo Control signals to determine speed and direction, with 0 degrees meaning neutral https://en.wikipedia.org/wiki/Servo_control

void disable_motors()
{
  left_font_motor.detach();  // detach the servo on pin left_front to turn Vex Motor Controller 29 Off
  left_rear_motor.detach();  // detach the servo on pin left_rear to turn Vex Motor Controller 29 Off
  right_rear_motor.detach(); // detach the servo on pin right_rear to turn Vex Motor Controller 29 Off
  right_font_motor.detach(); // detach the servo on pin right_front to turn Vex Motor Controller 29 Off

  pinMode(left_front, INPUT);
  pinMode(left_rear, INPUT);
  pinMode(right_rear, INPUT);
  pinMode(right_front, INPUT);
}

void enable_motors()
{
  left_font_motor.attach(left_front);   // attaches the servo on pin left_front to turn Vex Motor Controller 29 On
  left_rear_motor.attach(left_rear);    // attaches the servo on pin left_rear to turn Vex Motor Controller 29 On
  right_rear_motor.attach(right_rear);  // attaches the servo on pin right_rear to turn Vex Motor Controller 29 On
  right_font_motor.attach(right_front); // attaches the servo on pin right_front to turn Vex Motor Controller 29 On
}
void stop() // Stop
{
  left_font_motor.writeMicroseconds(1500);
  left_rear_motor.writeMicroseconds(1500);
  right_rear_motor.writeMicroseconds(1500);
  right_font_motor.writeMicroseconds(1500);
}

void forward()
{
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val + 5);
  right_font_motor.writeMicroseconds(1500 - speed_val + 15);
}

void reverse()
{
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}

void ccw()
{
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}

void ccwSlower()
{
  left_font_motor.writeMicroseconds(1500 - speedSlowOffset);
  left_rear_motor.writeMicroseconds(1500 - speedSlowOffset);
  right_rear_motor.writeMicroseconds(1500 - speedSlowOffset);
  right_font_motor.writeMicroseconds(1500 - speedSlowOffset);
}

void cw()
{
  speed_val = 100;
  
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}

void cwSlower()
{
  left_font_motor.writeMicroseconds(1500 + speedSlowOffset);
  left_rear_motor.writeMicroseconds(1500 + speedSlowOffset);
  right_rear_motor.writeMicroseconds(1500 + speedSlowOffset);
  right_font_motor.writeMicroseconds(1500 + speedSlowOffset);
}

void cw2(float input)
{
  left_font_motor.writeMicroseconds(1500 + input);
  left_rear_motor.writeMicroseconds(1500 + input);
  right_rear_motor.writeMicroseconds(1500 + input);
  right_font_motor.writeMicroseconds(1500 + input);
}

void strafe_left()
{
  speed_val = 150;

  left_font_motor.writeMicroseconds(1500 - speed_val + 10);
  left_rear_motor.writeMicroseconds(1500 + speed_val - 5);
  right_rear_motor.writeMicroseconds(1500 + speed_val - 15);
  right_font_motor.writeMicroseconds(1500 - speed_val - 5);
}

void strafe_right()
{
  speed_val = 150;

  left_font_motor.writeMicroseconds(1500 + speed_val - 15);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val + 15);
}
