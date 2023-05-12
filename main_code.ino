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
// #include <SoftwareSerial.h>

#define PT1 A0
#define PT2 A1
#define PT3 A2
#define PT4 A3
#define FIR A4
#define BIR A5
#define LIR A6
#define RIR A7
#define GYRO A8
#define SERVO A9
#define FAN A10

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
  DRIVING,
  EXTINGUISH_FIRE,
  FINISHED,
  RUNNING,
  STOPPED
};

// Refer to Shield Pinouts.jpg for pin locations

// Default motor control pins
const byte left_front = 46;
const byte left_rear = 47;
const byte right_rear = 50;
const byte right_front = 51;

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

int speed_val = 100;
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

// Serial Pointer
HardwareSerial *SerialCom;

//---------------------------------------------------------------------------------------------------------------- SETUP
int pos = 0;
void setup(void)
{
  turret_motor.attach(11);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(FIR, INPUT);
  pinMode(BIR, INPUT);
  pinMode(LIR, INPUT);
  pinMode(RIR, INPUT);
  pinMode(PT1, INPUT);
  pinMode(PT2, INPUT);
  pinMode(PT3, INPUT);
  pinMode(PT4, INPUT);
  pinMode(GYRO, INPUT);
  pinMode(SERVO, OUTPUT);
  pinMode(FAN, OUTPUT);

  // The Trigger pin will tell the sensor to range find
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);

  fan_servo.attach(SERVO);

  // BluetoothSerial.begin(115200);
  // BluetoothSerial.println("Setup....");

  // while(1){
  //   BluetoothSerial.println(5, DEC);
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

  switch (machine_state)
  {
  case INITIALISING:
    machine_state = initialising();
    break;
  case TESTING:
    machine_state = testing();
    break;
  case FIRE_FIND:
    machine_state = fire_find();
    break;
  case DRIVING:
    machine_state = driving();
    break;
  case EXTINGUISH_FIRE:
    machine_state = extinguish_fire();
    break;
  case FINISHED:
    machine_state = finished();
    break;
  case RUNNING: // Lipo Battery Volage OK
    machine_state = running();
    break;
  case STOPPED: // Stop of Lipo Battery voltage is too low, to protect Battery
    machine_state = stopped();
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
  return FIRE_FIND;
}

STATE testing()
{

  // while(1){
  //   printValues();
  // }

  return FIRE_FIND;
}

STATE fire_find()
{

  // need to:
  // -make PTR's read 0 if light not detected, find threshhold
  // -work out minimum angle step of motor
  // -work out control of servo, will turnServo() work? otherwise need to make a more complex turnServo function with global servo position variable

  bool fireFound = false;
  int servoAngle, highestLightAngle;
  float currentLightReading, highestLightReading = 0;

  // turning servo its maximum angle span to detect a light
  // the angle at which the maximum light is detected at is found
  while (!fireFound)
  {
    for (servoAngle = 0; servoAngle <= 120; servoAngle++)
    {
      turnServo(servoAngle);
      currentLightReading = photoOne(); // figure out which PTR to use/multiple?

      if (currentLightReading > highestLightReading)
      {
        highestLightReading = currentLightReading;
        highestLightAngle = servoAngle;
      }
    }

    // if a light is not detected, rotate robot to search a different section of the course
    if (highestLightReading == 0)
    {
      turnDeg(CW, 120);
    }
    else
    {
      fireFound = true;
    }
  }

  // turn robot to face direction of light
  if (highestLightAngle <= 60)
  {
    turnDeg(CW, (60 - highestLightAngle));
  }
  else
  {
    turnDeg(CCW, (highestLightAngle - 60));
  }
  turnServo(60); // realign servo

  firesFound++;

  return DRIVING;
}


int lf_drive;
int lr_drive;
int rf_drive;
int rr_drive;
int lf_strafe;
int lr_strafe;
int rf_strafe;
int rr_strafe;

STATE driving()
{

  bool if_object = false;
  bool obj_left;
  bool obj_right;

  //TIMER
  unsigned long strafe_time_start;
  unsigned long strafe_time_end;
  bool timer_bool;

  while (exit < 3)
  {
    
    //TODO: change sensors
    float left_sensor = leftIR();
    float right_sensor= rightIR();
    float back_left   = backIR();
    float back_right  = frontIR();
    float MK_Ultra    = ultrasonic();


    if(left_sensor <= 0.5){ //OBJ infront + left 
      obj_left = true;

      //If the value is false set to true
      if (!timer_bool){
        timer_bool = true;
        strafe_time_start = millis();
      }

    } else { //OBJ not infront
      obj_left = false;
      timer_bool = false;
    }

    if(right_sensor <= 0.5){ //OBJ infront + right
      obj_right = true;

      //If the value is false set to true
      if (!timer_bool){
        timer_bool = true;
        strafe_time_start = millis();
      }

    } else { //OBJ NOT RIGHT
      obj_right = false;
    }

    //Check whether or not both right and left are detecting 
    if (obj_left && obj_right) {
      //Check sonar 
      //If sonar is detecting too travel ????
      if (MK_Ultra < 0.5) {
        //travel left or something 
      }
      else {
        continue; 
      }
    }
    
    //strafe terms and timing 
    if (obj_left) {
      //strafe right till obj is cleared 

      //save the time it took to strafe
    }

    else if (obj_right) {
      //strafe right till obj is cleared 

      //save the time it took to strafe
    }

    else {
      //Drive forward till object
    }




    //If object
    if (obj_left) {
      //Save the current millis() at start of strafe

      //Move to the right TODO: change values
      lf_strafe = 0;
      lr_strafe = 0;
      rf_strafe = 0;
      rr_strafe = 0;     

    } else { //we have moved right from the object. Object not infront
      if (timer_bool) {
        strafe_time_end = millis();
      }

      //Stop strafing
      lf_strafe = 0;
      lr_strafe = 0;
      rf_strafe = 0;
      rr_strafe = 0;   
    }
    
    //IF THERE WAS AN OBJECT AND HAVE PASSED. DETECT FALLING EDGE.
    //OBJLEFT || OBJRIGHT == false


    //IF WE JUST PASSED OBJECT SET TIMMER TARGET TO 0

    
    //Slow down if object
    if(if_object){
      lf_drive = 200;
      lr_drive = 200;
      rf_drive = 200;
      rr_drive = 200;
    }else {
      lf_drive = 200;
      lr_drive = 200;
      rf_drive = 200;
      rr_drive = 200;
    }

     left_font_motor.writeMicroseconds(1500 + lf_drive - lf_strafe); // left front
     left_rear_motor.writeMicroseconds(1500 + lr_drive - lr_strafe); // left rear
    right_font_motor.writeMicroseconds(1500 - rf_drive - rf_strafe); // rear right
    right_rear_motor.writeMicroseconds(1500 - rr_drive - rr_strafe); // front right

    delay(T);
  }

  stop();

  return EXTINGUISH_FIRE;
}

STATE extinguish_fire()
{

  bool fireExtinguished = false;

  while (!fireExtinguished)
  {
  }

  return (firesFound == 2) ? FINISHED : FIRE_FIND;
}

STATE finished()
{

  stop();

  return FINISHED;
}

STATE running()
{

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
STATE stopped()
{
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

void printValues()
{

  /*
  float pt1 = analogRead(PT1);
  float pt2 = analogRead(PT2);
  float pt3 = analogRead(PT3);
  float pt4 = analogRead(PT4);
  float fir = analogRead(FIR);
  float bir = analogRead(BIR);
  float lir = analogRead(LIR);
  float rir = analogRead(RIR);
  */

  float pt1 = photoOne();
  float pt2 = photoTwo();
  float pt3 = photoThree();
  float pt4 = photoFour();
  float fir = frontIR();
  float bir = backIR();
  float lir = leftIR();
  float rir = rightIR();
  float usc = ultrasonic();

  SerialCom->print(" PT1: ");
  SerialCom->print(pt1);
  SerialCom->print(" PT2: ");
  SerialCom->print(pt2);
  SerialCom->print(" PT3: ");
  SerialCom->print(pt3);
  SerialCom->print(" PT4: ");
  SerialCom->print(pt4);
  SerialCom->print(" FIR: ");
  SerialCom->print(fir);
  SerialCom->print(" BIR: ");
  SerialCom->print(bir);
  SerialCom->print(" LIR: ");
  SerialCom->print(lir);
  SerialCom->print(" RIR: ");
  SerialCom->print(rir);
  SerialCom->print(" USC: ");
  SerialCom->print(usc);
  SerialCom->println(" ");
}

//---------------------------------------------------------------------------------------------------------------- ACTIVATE FAN
void activateFan()
{

  analogWrite(FAN, 1);
  delay(1000);
  analogWrite(FAN, 0);
}

//---------------------------------------------------------------------------------------------------------------- TURN SERVO
void turnServo(float deg)
{

  constrain(deg, 0, 120); // 120 max degree angle
  fan_servo.write(deg);
}

//---------------------------------------------------------------------------------------------------------------- FRONT IR
float FIRValues1 = 0;
float FIRValues2 = 0;
float FIRValues3 = 0;
float FIRValues4 = 0;
float FIRValues5 = 0;
float frontIR()
{

  FIRValues1 = analogRead(FIR) * 5.0 / 1024.0;
  FIRValues2 = FIRValues1;
  FIRValues3 = FIRValues2;
  FIRValues4 = FIRValues3;
  FIRValues5 = FIRValues4;

  IRvolts = (FIRValues1 + FIRValues2 + FIRValues3 + FIRValues4 + FIRValues5) / 5; // averaged values

  return (IRvolts < 0.3) ? 0 : (1 / ((IRvolts - 0.0587) / 11.159)) + 11;
}

//---------------------------------------------------------------------------------------------------------------- BIR IR
float BIRValues1 = 0;
float BIRValues2 = 0;
float BIRValues3 = 0;
float BIRValues4 = 0;
float BIRValues5 = 0;
float backIR()
{

  BIRValues1 = analogRead(BIR) * 5.0 / 1024.0;
  BIRValues2 = BIRValues1;
  BIRValues3 = BIRValues2;
  BIRValues4 = BIRValues3;
  BIRValues5 = BIRValues4;

  IRvolts = (BIRValues1 + BIRValues2 + BIRValues3 + BIRValues4 + BIRValues5) / 5; // averaged values

  return (IRvolts < 0.3) ? 0 : (1 / ((IRvolts + 0.0413) / 11.482)) + 8;
}

//---------------------------------------------------------------------------------------------------------------- LEFT IR
float LIRValues1 = 0;
float LIRValues2 = 0;
float LIRValues3 = 0;
float LIRValues4 = 0;
float LIRValues5 = 0;
float leftIR()
{

  LIRValues1 = analogRead(LIR) * 5.0 / 1024.0;
  LIRValues2 = LIRValues1;
  LIRValues3 = LIRValues2;
  LIRValues4 = LIRValues3;
  LIRValues5 = LIRValues4;

  IRvolts = (LIRValues1 + LIRValues2 + LIRValues3 + LIRValues4 + LIRValues5) / 5; // averaged values

  return (IRvolts < 0.4) ? 0 : (1 / ((IRvolts - 0.0804) / 23.929)) + 7.35;
}

//---------------------------------------------------------------------------------------------------------------- RIGHT IR
float RIRValues1 = 0;
float RIRValues2 = 0;
float RIRValues3 = 0;
float RIRValues4 = 0;
float RIRValues5 = 0;
float rightIR()
{

  RIRValues1 = analogRead(RIR) * 5.0 / 1024.0;
  RIRValues2 = RIRValues1;
  RIRValues3 = RIRValues2;
  RIRValues4 = RIRValues3;
  RIRValues5 = RIRValues4;

  IRvolts = (RIRValues1 + RIRValues2 + RIRValues3 + RIRValues4 + RIRValues5) / 5; // averaged values

  return (IRvolts < 0.4) ? 0 : (1 / ((IRvolts - 0.0704) / 23.018)) + 7.35;
}

//---------------------------------------------------------------------------------------------------------------- PHOTOTRANSISTOR 1, NEEDS CALIBRATION?
float PhotoOneValues1 = 0;
float PhotoOneValues2 = 0;
float PhotoOneValues3 = 0;
float PhotoOneValues4 = 0;
float PhotoOneValues5 = 0;
float photoOne()
{

  PhotoOneValues1 = analogRead(PT1) * 5.0 / 1024.0;
  PhotoOneValues2 = PhotoOneValues1;
  PhotoOneValues3 = PhotoOneValues2;
  PhotoOneValues4 = PhotoOneValues3;
  PhotoOneValues5 = PhotoOneValues4;

  IRvolts = (PhotoOneValues1 + PhotoOneValues2 + PhotoOneValues3 + PhotoOneValues4 + PhotoOneValues5) / 5; // averaged values

  return IRvolts;
  // return (IRvolts < 0.4) ? 0 : (1 / ( ( IRvolts - 0.0704) / 23.018)) + 7.35;
}

//---------------------------------------------------------------------------------------------------------------- PHOTOTRANSISTOR 2, NEEDS CALIBRATION?
float PhotoTwoValues1 = 0;
float PhotoTwoValues2 = 0;
float PhotoTwoValues3 = 0;
float PhotoTwoValues4 = 0;
float PhotoTwoValues5 = 0;
float photoTwo()
{

  PhotoTwoValues1 = analogRead(PT2) * 5.0 / 1024.0;
  PhotoTwoValues2 = PhotoTwoValues1;
  PhotoTwoValues3 = PhotoTwoValues2;
  PhotoTwoValues4 = PhotoTwoValues3;
  PhotoTwoValues5 = PhotoTwoValues4;

  IRvolts = (PhotoTwoValues1 + PhotoTwoValues2 + PhotoTwoValues3 + PhotoTwoValues4 + PhotoTwoValues5) / 5; // averaged values

  return IRvolts;
  // return (IRvolts < 0.4) ? 0 : (1 / ( ( IRvolts - 0.0704) / 23.018)) + 7.35;
}

//---------------------------------------------------------------------------------------------------------------- PHOTOTRANSISTOR 3, NEEDS CALIBRATION?
float PhotoThreeValues1 = 0;
float PhotoThreeValues2 = 0;
float PhotoThreeValues3 = 0;
float PhotoThreeValues4 = 0;
float PhotoThreeValues5 = 0;
float photoThree()
{

  PhotoThreeValues1 = analogRead(PT3) * 5.0 / 1024.0;
  PhotoThreeValues2 = PhotoThreeValues1;
  PhotoThreeValues3 = PhotoThreeValues2;
  PhotoThreeValues4 = PhotoThreeValues3;
  PhotoThreeValues5 = PhotoThreeValues4;

  IRvolts = (PhotoThreeValues1 + PhotoThreeValues2 + PhotoThreeValues3 + PhotoThreeValues4 + PhotoThreeValues5) / 5; // averaged values

  return IRvolts;
  // return (IRvolts < 0.4) ? 0 : (1 / ( ( IRvolts - 0.0704) / 23.018)) + 7.35;
}

//---------------------------------------------------------------------------------------------------------------- PHOTOTRANSISTOR 4, NEEDS CALIBRATION?
float PhotoFourValues1 = 0;
float PhotoFourValues2 = 0;
float PhotoFourValues3 = 0;
float PhotoFourValues4 = 0;
float PhotoFourValues5 = 0;
float photoFour()
{

  PhotoFourValues1 = analogRead(PT4) * 5.0 / 1024.0;
  PhotoFourValues2 = PhotoFourValues1;
  PhotoFourValues3 = PhotoFourValues2;
  PhotoFourValues4 = PhotoFourValues3;
  PhotoFourValues5 = PhotoFourValues4;

  IRvolts = (PhotoFourValues1 + PhotoFourValues2 + PhotoFourValues3 + PhotoFourValues4 + PhotoFourValues5) / 5; // averaged values

  return IRvolts;
  // return (IRvolts < 0.4) ? 0 : (1 / ( ( IRvolts - 0.0704) / 23.018)) + 7.35;
}

//---------------------------------------------------------------------------------------------------------------- ULTRASONIC
float USValues1 = 0;
float USValues2 = 0;
float USValues3 = 0;
float USValues4 = 0;
float USValues5 = 0;
float ultrasonic()
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
      return 300.0;
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
      // SerialCom->println("HC-SR04: Out of range");
      return;
    }
  }

  t2 = micros();
  pulse_width = t2 - t1;

  // Calculate distance in centimeters and inches. The constants
  // are found in the datasheet, and calculated from the assumed speed
  // of sound in air at sea level (~340 m/s).

  // Average the signal by 5
  USValues1 = pulse_width / 58.0;
  USValues2 = USValues1; // IRValues1;
  USValues3 = USValues2; // IRValues2;
  USValues4 = USValues3; // IRValues3;
  USValues5 = USValues4; // IRValues4;

  cm = (USValues1 + USValues2 + USValues3 + USValues4 + USValues5) / 5; // averaged values

  // cm = pulse_width / 58.0;
  inches = pulse_width / 148.0;

  // Print out results
  if (pulse_width > MAX_DIST)
  {
    // SerialCom->println("HC-SR04: Out of range");
  }
  else
  {
    // SerialCom->print("HC-SR04:");
    // SerialCom->print(cm);
    // SerialCom->println("cm");

    return cm;
  }
}

void gyroUpdate()
{
  gyroRate = ((analogRead(GYRO) * 5.0) / 1024.0) - 2.5; // 2.5V = resting value offset

  // read out voltage divided the gyro sensitivity to calculate the angular velocity
  angularVelocity = gyroRate / gyroSensitivity; // from Data Sheet, gyroSensitivity is 0.007 V/dps

  // if the angular velocity is less than the threshold, ignore it
  if (angularVelocity >= rotationThreshold || angularVelocity <= -rotationThreshold)
  {
    // we are running a loop in T (of T/1000 second).
    angleChange = (angularVelocity / (1000.0 / T)) * Toffset * 0.955;
    currentAngle += angleChange;
  }

  // keep the angle between 0-360
  if (currentAngle < 0)
  {
    currentAngle += 360.0;
  }
  else if (currentAngle > 359)
  {
    currentAngle -= 360.0;
  }
}

void objectAvoid(int direction)
{
  return;
}

//---------------------------------------------------------------------------------------------------------------- TURN DEG
void turnDeg(int directionCW, float deg)
{
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

//---------------------------------------------------------------------------------------------------------------- DRIVE
void drive(int forwardYes, float wallDist, bool useLeftIR)
{
  float gyroError, distError;
  currentAngle = 0;

  float Kp_gyro = 0;
  float Kp_dist = 25;

  bool veerLeft, veerRight;
  bool strafeLeft, strafeRight;
  float effortFL, effortFR, effortRL, effortRR;

  // int correction1 = 126;
  // int correction2 = 105;
  int correction1 = 165; // LF
  int correction2 = 150; // LR
  int correction3 = 150; // RF
  int correction4 = 145; // RR

  int exit = 0;

  float Kp_FL = 25 * correction1 / 150;
  float Kp_FR = 25 * correction3 / 150;
  float Kp_RL = 25 * correction2 / 150;
  float Kp_RR = 25 * correction4 / 150;

  float Kp = 25;

  if (forwardYes)
  {
    // FORWARD LOOP
    // while( (frontIR() <= 12) || (ultrasonic() > 15)){
    while ((ultrasonic() > 15))
    {

      gyroRate = ((analogRead(GYRO) * 5.0) / 1024.0) - 2.5; // 2.5V = resting value offset

      angularVelocity = gyroRate / gyroSensitivity; // from Data Sheet, gyroSensitivity is 0.007 V/dps
      if (angularVelocity >= rotationThreshold || angularVelocity <= -rotationThreshold)
      {
        angleChange = angularVelocity / (1000.0 / T);
        currentAngle += angleChange;
      }

      gyroError = 0 - currentAngle;

      distError = (useLeftIR) ? wallDist - leftIR() : wallDist - rightIR();
      sendData(useLeftIR);
      // SerialCom->print("Distance Error: ");
      // SerialCom->println(distError);

      if (gyroError > 2)
      {

        veerRight = true;
        veerLeft = false;
      }
      else if (gyroError < -2)
      {

        veerRight = false;
        veerLeft = true;
      }
      else
      {

        veerRight = false;
        veerLeft = false;
      }

      // if(distError > 2){

      //   strafeRight = true;
      //   strafeLeft = false;

      // } else if(distError < -2){

      //   strafeRight = false;
      //   strafeLeft = true;

      // } else{

      //   strafeRight = false;
      //   strafeLeft = false;
      // }

      // effortFL = Kp_gyro*veerRight*abs(gyroError) - Kp_dist*strafeLeft*abs(distError) + Kp_dist*strafeRight*abs(distError);
      // //effortFL = (effortFL < -1*correction1) ? 0 : effortFL;
      // effortFR = Kp_gyro*veerLeft*abs(gyroError) - Kp_dist*strafeLeft*abs(distError) + Kp_dist*strafeRight*abs(distError);
      // //effortFR = (effortFR > correction4) ? 0 : effortFR;
      // effortRL = Kp_dist*strafeLeft*abs(distError) - Kp_dist*strafeRight*abs(distError);
      // //effortRL = (effortRL < -1*correction2) ? 0 : effortRL;
      // effortRR = Kp_dist*strafeLeft*abs(distError) - Kp_dist*strafeRight*abs(distError);
      // //effortRR = (effortRR > correction3) ? 0 : effortRR;

      if (useLeftIR)
      {

        if (distError > 1)
        {

          veerRight = true;
          veerLeft = false;
        }
        else if (distError < -1)
        {

          veerRight = false;
          veerLeft = true;
        }
        else
        {

          veerRight = false;
          veerLeft = false;
        }
      }
      else
      {

        if (distError > 1)
        {

          veerRight = false;
          veerLeft = true;
        }
        else if (distError < -1)
        {

          veerRight = true;
          veerLeft = false;
        }
        else
        {

          veerRight = false;
          veerLeft = false;
        }
      }

      // effortFL = Kp_FL*veerRight*abs(distError);
      // //effortFL = (effortFL < -1*correction1) ? 0 : effortFL;
      // effortFR = Kp_FR*veerLeft*abs(distError);
      // //effortFR = (effortFR > correction4) ? 0 : effortFR;
      // effortRL = Kp_RL*veerLeft*abs(distError);
      // //effortRL = (effortRL < -1*correction2) ? 0 : effortRL;
      // effortRR = Kp_RR*veerRight*abs(distError);
      // //effortRR = (effortRR > correction3) ? 0 : effortRR;

      effortFL = Kp_FL * veerRight * abs(distError);
      // effortFL = (effortFL < -1*correction1) ? 0 : effortFL;
      effortFR = Kp_FR * veerLeft * abs(distError);
      // effortFR = (effortFR > correction4) ? 0 : effortFR;
      effortRL = Kp_RL * veerLeft * abs(distError);
      // effortRL = (effortRL < -1*correction2) ? 0 : effortRL;
      effortRR = Kp_RR * veerRight * abs(distError);
      // effortRR = (effortRR > correction3) ? 0 : effortRR;

      left_font_motor.writeMicroseconds(1500 + correction1 + effortFL);
      left_rear_motor.writeMicroseconds(1500 + correction2 + effortRL);
      right_font_motor.writeMicroseconds(1500 - correction3 - effortRR); // rear right
      right_rear_motor.writeMicroseconds(1500 - correction4 - effortFR); // front right

      // SerialCom->print("effortFL: ");
      // SerialCom->print(effortFL);
      // SerialCom->print(" effortFR: ");
      // SerialCom->print(effortFR);
      // SerialCom->print(" effortRL: ");
      // SerialCom->print(effortRL);
      // SerialCom->print(" effortRR: ");
      // SerialCom->println(effortRR);
      // Serial.print(frontIR());
      // SerialCom->print("  Gyro Error: ");
      // Serial.print(gyroError);
      // SerialCom->print("  Distance Error: ");
      /// Serial.println(distError);

      delay(T);
      // SerialCom->print("LIR IR: ");
      // SerialCom->println(leftIR());
    }
    // SerialCom->print("Exit was: ");
    // SerialCom->println(frontIR());
  }
  else
  { // BACKWARDS

    // while( (backIR() <= 10) || (backIR() > 20) )
    while (exit < 3)
    {
      if ((backIR() < 30) && (backIR() > 10))
      {
        exit++;
      }
      else
      {
        exit = 0;
      }

      gyroRate = ((analogRead(GYRO) * 5.0) / 1024.0) - 2.5; // 2.5V = resting value offset

      angularVelocity = gyroRate / gyroSensitivity; // from Data Sheet, gyroSensitivity is 0.007 V/dps
      if (angularVelocity >= rotationThreshold || angularVelocity <= -rotationThreshold)
      {
        angleChange = angularVelocity / (1000.0 / T);
        currentAngle += angleChange;
      }

      gyroError = 0 - currentAngle;

      distError = (useLeftIR) ? wallDist - leftIR() : wallDist - rightIR();
      sendData(useLeftIR);
      // SerialCom->print("Distance Error: ");
      // SerialCom->println(distError);

      if (gyroError > 2)
      {

        veerRight = true;
        veerLeft = false;
      }
      else if (gyroError < -2)
      {

        veerRight = false;
        veerLeft = true;
      }
      else
      {

        veerRight = false;
        veerLeft = false;
      }

      if (useLeftIR)
      {

        if (distError > 1)
        {

          veerRight = true;
          veerLeft = false;
        }
        else if (distError < -1)
        {

          veerRight = false;
          veerLeft = true;
        }
        else
        {

          veerRight = false;
          veerLeft = false;
        }
      }
      else
      {

        if (distError > 1)
        {

          veerRight = false;
          veerLeft = true;
        }
        else if (distError < -1)
        {

          veerRight = true;
          veerLeft = false;
        }
        else
        {

          veerRight = false;
          veerLeft = false;
        }
      }
      /*
      effortFL = Kp_gyro*veerRight*abs(gyroError) - Kp_dist*strafeLeft*abs(distError) + Kp_dist*strafeRight*abs(distError);
      effortFL = (effortFL > correction1) ? 0 : effortFL;
      effortFR = Kp_gyro*veerLeft*abs(gyroError) - Kp_dist*strafeLeft*abs(distError) + Kp_dist*strafeRight*abs(distError);
      effortFR = (effortFR > correction4) ? 0 : effortFR;
      effortRL = Kp_dist*strafeLeft*abs(distError) - Kp_dist*strafeRight*abs(distError);
      effortRL = (effortRL > correction2) ? 0 : effortRL;
      effortRR = Kp_dist*strafeLeft*abs(distError) - Kp_dist*strafeRight*abs(distError);
      effortRR = (effortRR > correction3) ? 0 : effortRR;*/

      // effortFL = Kp_FL*veerLeft*abs(distError);
      // //effortFL = (effortFL < -1*correction1) ? 0 : effortFL;
      // effortFR = Kp_FR*veerRight*abs(distError);
      // //effortFR = (effortFR > correction4) ? 0 : effortFR;
      // effortRL = Kp_RL*veerRight*abs(distError);
      // //effortRL = (effortRL < -1*correction2) ? 0 : effortRL;
      // effortRR = Kp_RR*veerLeft*abs(distError);
      // //effortRR = (effortRR > correction3) ? 0 : effortRR;

      effortFL = Kp * veerLeft * abs(distError);
      // effortFL = (effortFL < -1*correction1) ? 0 : effortFL;
      effortFR = Kp * veerRight * abs(distError);
      // effortFR = (effortFR > correction4) ? 0 : effortFR;
      effortRL = Kp * veerRight * abs(distError);
      // effortRL = (effortRL < -1*correction2) ? 0 : effortRL;
      effortRR = Kp * veerLeft * abs(distError);
      // effortRR = (effortRR > correction3) ? 0 : effortRR;

      left_font_motor.writeMicroseconds(1500 - correction1 - effortFL);
      left_rear_motor.writeMicroseconds(1500 - correction2 - effortRL);
      right_font_motor.writeMicroseconds(1500 + correction3 + effortRR); // rear right
      right_rear_motor.writeMicroseconds(1500 + correction4 + effortFR); // front right

      // SerialCom->print("Back sensor: ");
      // SerialCom->println(backIR());

      delay(T);
      // SerialCom->print("BIR IR EXIT: ");
      // SerialCom->println(backIR());
    }
  }

  stop();
}

//---------------------------------------------------------------------------------------------------------------- STRAFE
void strafe(int leftYes, float wallDist)
{
  float gyroError, distError;
  currentAngle = 0;

  float Kp_gyro = 5;
  float Kp_dist = 5;
  bool turnCW, turnCCW;
  bool moveForward, moveBack;
  float effortFL, effortFR, effortRL, effortRR;

  float time = 0;

  int correction1 = 100;
  int correction2 = 100;
  int correction3 = 100;
  int correction4 = 100;

  if (leftYes)
  {

    // Rotate counterclockwise
    while (time < 1500)
    {
      sendData(true);
      gyroRate = ((analogRead(GYRO) * 5.0) / 1024.0) - 2.5; // 2.5V = resting value offset

      angularVelocity = gyroRate / gyroSensitivity; // from Data Sheet, gyroSensitivity is 0.007 V/dps
      if (angularVelocity >= rotationThreshold || angularVelocity <= -rotationThreshold)
      {
        angleChange = angularVelocity / (1000.0 / T);
        currentAngle += angleChange;
      }

      gyroError = 0 - currentAngle;

      distError = wallDist - frontIR();

      if (gyroError > 2)
      {

        turnCCW = false;
        turnCW = true;
      }
      else if (gyroError < -2)
      {

        turnCCW = true;
        turnCW = false;
      }
      else
      {

        turnCCW = false;
        turnCW = false;
      }

      if (distError > 2)
      {

        // moveForward = true;
        // moveBack = false;

        moveForward = false;
        moveBack = true;
      }
      else if (distError < -2)
      {

        // moveForward = false;
        // moveBack = true;

        moveForward = true;
        moveBack = false;
      }
      else
      {

        moveForward = false;
        moveBack = false;
      }

      effortFL = Kp_gyro * turnCW * abs(gyroError) - Kp_gyro * turnCCW * abs(gyroError) + Kp_dist * moveForward * abs(distError) - Kp_dist * moveBack * abs(distError);
      effortFL = (effortFL > correction1) ? 0 : effortFL;
      effortFR = Kp_gyro * turnCW * abs(gyroError) - Kp_gyro * turnCCW * abs(gyroError) + Kp_dist * moveForward * abs(distError) - Kp_dist * moveBack * abs(distError);
      effortFR = (effortFR > correction4) ? 0 : effortFR;
      effortRL = Kp_gyro * turnCW * abs(gyroError) - Kp_gyro * turnCCW * abs(gyroError) - Kp_dist * moveForward * abs(distError) + Kp_dist * moveBack * abs(distError);
      effortRL = (effortRL > correction2) ? 0 : effortRL;
      effortRR = Kp_gyro * turnCW * abs(gyroError) - Kp_gyro * turnCCW * abs(gyroError) - Kp_dist * moveForward * abs(distError) + Kp_dist * moveBack * abs(distError);
      effortRR = (effortRR > correction3) ? 0 : effortRR;

      effortFL = 0;
      effortFR = 0;
      effortRL = 0;
      effortRR = 0;

      left_font_motor.writeMicroseconds(1500 - correction1 + effortFL);
      left_rear_motor.writeMicroseconds(1500 + correction2 + effortRL);
      right_font_motor.writeMicroseconds(1500 + correction3 + effortRR); // rear right
      right_rear_motor.writeMicroseconds(1500 - correction4 + effortFR); // front right

      // SerialCom->print(currentAngle);
      // SerialCom->println();

      delay(T);

      time += T;
    }
  }
  else
  { // else rotate clockwise

    while (time < 1500)
    {
      sendData(true);
      gyroRate = ((analogRead(GYRO) * 5.0) / 1024.0) - 2.5; // 2.5V = resting value offset

      angularVelocity = gyroRate / gyroSensitivity; // from Data Sheet, gyroSensitivity is 0.007 V/dps
      if (angularVelocity >= rotationThreshold || angularVelocity <= -rotationThreshold)
      {
        angleChange = angularVelocity / (1000.0 / T);
        currentAngle += angleChange;
      }

      gyroError = 0 - currentAngle;

      distError = wallDist - frontIR();

      if (gyroError > 2)
      {

        turnCCW = false;
        turnCW = true;
      }
      else if (gyroError < -2)
      {

        turnCCW = true;
        turnCW = false;
      }
      else
      {

        turnCCW = false;
        turnCW = false;
      }

      if (distError > 2)
      {

        moveForward = false;
        moveBack = true;
      }
      else if (distError < -2)
      {

        moveForward = true;
        moveBack = false;
      }
      else
      {

        moveForward = false;
        moveBack = false;
      }

      effortFL = Kp_gyro * turnCW * abs(gyroError) - Kp_gyro * turnCCW * abs(gyroError) + Kp_dist * moveForward * abs(distError) - Kp_dist * moveBack * abs(distError);
      effortFL = (effortFL > correction1) ? 0 : effortFL;
      effortFR = Kp_gyro * turnCW * abs(gyroError) - Kp_gyro * turnCCW * abs(gyroError) + Kp_dist * moveForward * abs(distError) - Kp_dist * moveBack * abs(distError);
      effortFR = (effortFR > correction4) ? 0 : effortFR;
      effortRL = Kp_gyro * turnCW * abs(gyroError) - Kp_gyro * turnCCW * abs(gyroError) - Kp_dist * moveForward * abs(distError) + Kp_dist * moveBack * abs(distError);
      effortRL = (effortRL > correction2) ? 0 : effortRL;
      effortRR = Kp_gyro * turnCW * abs(gyroError) - Kp_gyro * turnCCW * abs(gyroError) - Kp_dist * moveForward * abs(distError) + Kp_dist * moveBack * abs(distError);
      effortRR = (effortRR > correction3) ? 0 : effortRR;

      effortFL = 0;
      effortFR = 0;
      effortRL = 0;
      effortRR = 0;

      left_font_motor.writeMicroseconds(1500 + correction1 + effortFL);
      left_rear_motor.writeMicroseconds(1500 - correction2 + effortRL);
      right_font_motor.writeMicroseconds(1500 - correction3 + effortRR + 10); // rear right
      right_rear_motor.writeMicroseconds(1500 + correction4 + effortFR);      // front right

      // SerialCom->print(currentAngle);
      // SerialCom->println();

      delay(T);

      time += T;
    }
  }

  stop();
}

//---------------------------------------------------------------------------------------------------------------- STRAFE LEFT TO WALL
void strafeLeftToWall()
{
  float gyroError;
  currentAngle = 0;

  float Kp_gyro = 0;
  bool turnCW, turnCCW;
  float effortFL, effortFR, effortRL, effortRR;

  float time = 0;

  int correction1 = 110 + 50; // FL
  int correction2 = 100 + 50; // RF
  int correction3 = 90 + 50;  // RR
  int correction4 = 100 + 50; // FR

  int exit;

  while ((leftIR() > 16.0) || (leftIR() <= 7))
  {
    // SerialCom->print("Left IR: ");
    // SerialCom->println(leftIR());

    gyroRate = ((analogRead(GYRO) * 5.0) / 1024.0) - 2.5; // 2.5V = resting value offset

    angularVelocity = gyroRate / gyroSensitivity; // from Data Sheet, gyroSensitivity is 0.007 V/dps
    if (angularVelocity >= rotationThreshold || angularVelocity <= -rotationThreshold)
    {
      angleChange = angularVelocity / (1000.0 / T);
      currentAngle += angleChange;
    }

    gyroError = 0 - currentAngle;

    if (gyroError > 2)
    {

      turnCCW = true;
      turnCW = false;
    }
    else if (gyroError < -2)
    {

      turnCCW = false;
      turnCW = true;
    }
    else
    {

      turnCCW = false;
      turnCW = false;
    }

    effortFL = Kp_gyro * turnCW * abs(gyroError) - Kp_gyro * turnCCW * abs(gyroError);
    effortFL = (effortFL > correction1) ? 0 : effortFL;
    effortFR = Kp_gyro * turnCW * abs(gyroError) - Kp_gyro * turnCCW * abs(gyroError);
    effortFR = (effortFR > correction4) ? 0 : effortFR;
    effortRL = Kp_gyro * turnCW * abs(gyroError) - Kp_gyro * turnCCW * abs(gyroError);
    effortRL = (effortRL > correction2) ? 0 : effortRL;
    effortRR = Kp_gyro * turnCW * abs(gyroError) - Kp_gyro * turnCCW * abs(gyroError);
    effortRR = (effortRR > correction3) ? 0 : effortRR;

    left_font_motor.writeMicroseconds(1500 - correction1 + effortFL);
    left_rear_motor.writeMicroseconds(1500 + correction2 + effortRL);
    right_font_motor.writeMicroseconds(1500 + correction3 + effortRR); // rear right
    right_rear_motor.writeMicroseconds(1500 - correction4 + effortFR); // front right

    delay(T);
  }

  stop();
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
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
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
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}

void strafe_right()
{
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}

// gloabal variable - caliabrate the coords at the end of the corner find
// at the end of corner find set the value so the sensors = 15cm
float plot_x_offset = 0.0;
float plot_y_offset = 0.0;

// for offse3t of the sensor relative to the middle of the robot
float x_sensor_offset = 0.0;
float y_sensor_offset = 0.0;

// values to be sent.
float plot_x = 0.0;
float plot_y = 0.0;

float y_total = 0.0; //(SET VALUE)

// This function sends the x,y values for plotting
void sendData(bool leftYes)
{

  // read ultrasonic
  float x_reading = ultrasonic(); // This takes the reading for the x displacement
  // read ir sides
  float y_reading_left = leftIR(); // takes the reading for the y-displacement
  float y_reading_right = rightIR();

  // output x value with offsets
  plot_x = x_reading + plot_x_offset + x_sensor_offset;
  SerialCom->print(plot_x);
  SerialCom->print(",");

  // output y with offsets
  // If left reading is less than 70cm use the left value
  // else use the right value.
  if (y_reading_left <= 70)
  {
    plot_y = y_reading_left + y_sensor_offset + plot_y_offset;
  }
  else
  {
    plot_y = y_total - (y_reading_right + y_sensor_offset + plot_y_offset);
  }

  if (leftYes)
  {
    SerialCom->print(leftIR());
    SerialCom->print(",");
    SerialCom->println();
  }
  else
  {
    SerialCom->print(120 - rightIR());
    SerialCom->print(",");
    SerialCom->println();
  }
  // SerialCom->print(plot_y);
  // SerialCom->print(",");
  // SerialCom->println();
}