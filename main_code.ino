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
#include <Servo.h>  //Need for Servo pulse output
#include <math.h>
// #include <SoftwareSerial.h>

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

//#define NO_READ_GYRO  //Uncomment of GYRO is not attached.
//#define NO_HC-SR04 //Uncomment of HC-SR04 ultrasonic ranging sensor is not attached.
//#define NO_BATTERY_V_OK //Uncomment of BATTERY_V_OK if you do not care about battery damage.

//State machine states
enum STATE {
  INITIALISING,
  TESTING,
  FIRE_FIND,
  DRIVING,
  EXTINGUISH_FIRE,
  FINISHED,
  RUNNING,
  STOPPED
};

//Refer to Shield Pinouts.jpg for pin locations

//Default motor control pins
const byte left_front = 46;
const byte left_rear = 47;
const byte right_rear = 51;
const byte right_front = 50;

//Default ultrasonic ranging sensor pins, these pins are defined my the Shield
const int TRIG_PIN = 48;
const int ECHO_PIN = 49;

// Anything over 400 cm (23200 us pulse) is "out of range". Hit:If you decrease to this the ranging sensor but the timeout is short, you may not need to read up to 4meters.
const unsigned int MAX_DIST = 23200;

Servo left_font_motor;   // create servo object to control Vex Motor Controller 29
Servo left_rear_motor;   // create servo object to control Vex Motor Controller 29
Servo right_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_font_motor;  // create servo object to control Vex Motor Controller 29
Servo turret_motor;
Servo fan_servo;

int speed_val = 100;
int speed_change;

int firesFound = 0;

//GRYO VALUES
int T = 110;                    // T is the time of one loop, 0.1 sec
float gyroSensitivity = 0.007;  // gyro sensitivity unit is (mv/degree/second) get from datasheet
float rotationThreshold = 1.5;  // because of gyro drifting, defining rotation angular velocity less
float currentAngle = 0;         // current angle calculated by angular velocity integral
float gyroRate, angularVelocity, angleChange;
float lowestAnglePos = 0;
float IRvolts;

float Toffset = 1.1;  //for IR

float speedSlowOffset = 75;

//Serial Pointer
HardwareSerial *SerialCom;

//---------------------------------------------------------------------------------------------------------------- SETUP
int pos = 0;
void setup(void) {
  turret_motor.attach(11);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(FAN, OUTPUT);
  pinMode(GYRO, INPUT);
  pinMode(SERVO, OUTPUT);
  pinMode(LPT, INPUT);
  pinMode(RPT, INPUT);
  pinMode(TLPT, INPUT);
  pinMode(TRPT, INPUT);
  pinMode(BLIR, INPUT);
  pinMode(BRIR, INPUT);
  pinMode(FLIR, INPUT);
  pinMode(FRIR, INPUT);

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

  Serial.begin(115200);

  delay(1000);  //settling time but no really needed
}

//------------------------------------------------------------------------------------------------------------------------------------- MAIN LOOP
void loop(void)  //main loop
{
  static STATE machine_state = INITIALISING;
  //Finite-state machine Code

  switch (machine_state) {
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
    case RUNNING:  //Lipo Battery Volage OK
      machine_state = running();
      break;
    case STOPPED:  //Stop of Lipo Battery voltage is too low, to protect Battery
      machine_state = stopped();
      break;
  };
}

//------------------------------------------------------------------------------------------------------------------------------------- STATES
STATE initialising() {
  //initialising
  Serial.println("INITIALISING....");
  delay(1000);  //One second delay to see the serial string "INITIALISING...."
  //Serial.println("Enabling Motors...");
  enable_motors();
  Serial.println("RUNNING STATE...");
  turnServo(60);
  return TESTING;
}

STATE testing() {

  // while(1){

  //   Serial.println("Fan on....");
  //   digitalWrite(26, HIGH);
  //   delay(5000);
  //   Serial.println("Fan off...");
  //   digitalWrite(26, LOW);
  //   delay(5000);

  // }


  // while(1){
  //   //printValues();
  //   turnDeg(CCW, 90);
  //   //gyroUpdate();
  //   // Serial.print("Current Angle: ");
  //   // Serial.println(currentAngle);

  //   // Serial.print("Gyro reading: ");
  //   // Serial.println((analogRead(GYRO) * 5.0 / 1024.0) - 2.5);
  //   //delay(10000);
  // }

  // for(int i = 0; i <= 120; i++)
  // {
  //   turnServo(i);
  //   delay(10);
  // }
  
  // turnServo(60);

  // delay(100000);

  // while(1){

  //   printValues();
  // }
  // delay(100000);

  return FIRE_FIND;
}

STATE fire_find() {

  bool fireFound = false;
  int servoAngle, highestLightAngle;
  float currentLeftLightReading, currentRightLightReading, averagedLightReading;
  float highestLeftLightReading = 0;
  float highestRightLightReading = 0;
  float averagedLightReadings[120];

  //turning servo its maximum angle span (0-120 deg) to detect a light
  //the angle at which the maximum light is detected at is found
  while (!fireFound) {
    for (int i = 0; i <= 5; i++) {
      currentLeftLightReading = topLeftPT();
      currentRightLightReading = topRightPT();
    }

    //turn servo from 0 to 120 degrees with 1 degree increments
    for (servoAngle = 0; servoAngle <= 120; servoAngle++) {
      turnServo(servoAngle);
      currentLeftLightReading = topLeftPT();
      currentRightLightReading = topRightPT();

      // Serial.print("Current Light Reading: ");
      // Serial.print(currentLightReading);
      // Serial.print(" at ");
      // Serial.print(servoAngle);
      // Serial.println(" degrees");   

      // Serial.print(servoAngle); 
      // Serial.print(","); 
      // Serial.print(currentLeftLightReading); 
      // Serial.print(","); 
      // Serial.print(currentRightLightReading); 
      // Serial.print(","); 
      // Serial.println();

      // if ((currentLeftLightReading > highestLeftLightReading) && (currentRightLightReading > highestRightLightReading)) {
      //   highestLeftLightReading = currentLeftLightReading;
      //   highestRightLightReading = currentRightLightReading;
      //   highestLightAngle = servoAngle;
      // }

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

    for(int i = 0; i<=120; i++){
      Serial.print("At ");
      Serial.print(i);
      Serial.print(" degrees: ");
      Serial.println(averagedLightReadings[i]);
    }   

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

    //if a light is not detected, rotate robot 120 degrees to search a different section of the course
    if (highestLightAngle <= 5) {
      Serial.println("Turning 100 degrees");
      turnDeg(CW, 100);
    } else {
      Serial.print("Fire found at: ");
      Serial.println(highestLightAngle);
      fireFound = true;
    }
  }

  float comDist = 9.5; //distance between robot centre of mass and turning point of the servo motor
  float offsetAngle = 60 - highestLightAngle;
  float offsetAngleRad = offsetAngle * PI / 180.0;
  float turnAngleRad;

  // Calculate the turn angle in radians
  // if (offsetAngleRad != 0.0) {
  //   turnAngleRad = atan((comDist / 2.0) * sin(offsetAngleRad) / (1.0 + cos(offsetAngleRad)));
  // } else {
  //   turnAngleRad = 0.0;
  // }
  
  // float turnAngle = turnAngleRad * 180.0 / PI;

  // Serial.print("turnAngle = ");
  // Serial.println(turnAngle);

  // // //turn robot to face direction of light
  // if (highestLightAngle <= 60) {

  //   Serial.print("Turning CW ");
  //   Serial.print(60 - highestLightAngle);
  //   Serial.println(" degrees");

  //   turnDeg(CW, (60 - highestLightAngle));

  //   Serial.println("Finished turning CW to face fire");
  // } else {

  //   Serial.print("Turning CCW ");
  //   Serial.print(highestLightAngle - 60);
  //   Serial.println(" degrees");

  //   turnDeg(CCW, (highestLightAngle - 60));

  //   Serial.println("Finished CCW turning to face fire");
  // }

float turnAngle = offsetAngle * 0.7;  // Adjust the multiplier as needed

// Limit the turn angle within a reasonable range
if (turnAngle > 30.0) {
  turnAngle = 30.0;
} else if (turnAngle < -30.0) {
  turnAngle = -30.0;
}

  if(turnAngle < 0)
  {
    turnDeg(CCW, abs(turnAngle));
  }
  else if(turnAngle > 0)
  {
    turnDeg(CW, abs(turnAngle));
  }
  turnServo(60);  //realign servo

  firesFound++;

  delay(1000000);

  return DRIVING;
}

STATE driving() {

  //drive in direction of light found
  //robot should start already facing light

  //avoidance function sends robot in reverse direction to object detected to prevent collision from occuring
  objectAvoid(0);  //needs to be implemented

  //alternative method could be to make object avoidance part of driving control effort

  return EXTINGUISH_FIRE;
}

STATE extinguish_fire() {

  bool fireExtinguished = false;
  bool exit = false;
  int angle = 0;
  float hotStuff;

  while (!exit) {
    hotStuff = digitalRead(TRPT);
    turnServo(angle);
    if (digitalRead(TLPT) < digitalRead(TRPT)) {  // requires some tuning
      exit = true;
    } else {
      angle++;
    }
  }
  turnServo(angle - 1);
  digitalWrite(FAN, HIGH);
  while (!fireExtinguished) {
    if ((digitalRead(TLPT) < 4) && (digitalRead(TRPT) < 4))  // needs tuning
      fireExtinguished == true;
    digitalWrite(FAN, LOW);
  }

  return (firesFound == 2) ? FINISHED : FIRE_FIND;
}

STATE finished() {

  stop();

  return FINISHED;
}

STATE running() {

  static unsigned long previous_millis;

  read_serial_command();
  fast_flash_double_LED_builtin();

  if (millis() - previous_millis > 500) {  //Arduino style 500ms timed execution statement
    previous_millis = millis();

    Serial.println("RUNNING---------");
    speed_change_smooth();
    Analog_Range_A4();

#ifndef NO_READ_GYRO
    GYRO_reading();
#endif

#ifndef NO_HC - SR04
    HC_SR04_range();
#endif

#ifndef NO_BATTERY_V_OK
    if (!is_battery_voltage_OK()) return STOPPED;
#endif


    turret_motor.write(pos);

    if (pos == 0) {
      pos = 45;
    } else {
      pos = 0;
    }
  }

  return RUNNING;
}

//Stop if Lipo Battery voltage is too low, to protect Battery
STATE stopped() {
  static byte counter_lipo_voltage_ok;
  static unsigned long previous_millis;
  int Lipo_level_cal;
  disable_motors();
  slow_flash_LED_builtin();

  if (millis() - previous_millis > 500) {  //print massage every 500ms
    previous_millis = millis();
    Serial.println("STOPPED---------");


#ifndef NO_BATTERY_V_OK
    //500ms timed if statement to check lipo and output speed settings
    if (is_battery_voltage_OK()) {
      Serial.print("Lipo OK waiting of voltage Counter 10 < ");
      Serial.println(counter_lipo_voltage_ok);
      counter_lipo_voltage_ok++;
      if (counter_lipo_voltage_ok > 10) {  //Making sure lipo voltage is stable
        counter_lipo_voltage_ok = 0;
        enable_motors();
        Serial.println("Lipo OK returning to RUN STATE");
        return RUNNING;
      }
    } else {
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

  // Serial.print(" LPT: ");
  // Serial.print(lpt);
  // Serial.print(" RPT: ");
  // Serial.print(rpt);
  // Serial.print(" TLPT: ");
  // Serial.print(tlpt);
  // Serial.print(" TRPT: ");
  // Serial.print(trpt);
  // Serial.print(" FLIR: ");
  // Serial.print(flir);
  // Serial.print(" FRIR: ");
  // Serial.print(frir);
  // Serial.print(" BLIR: ");
  // Serial.print(blir);
  // Serial.print(" BRIR: ");
  // Serial.print(brir);
  Serial.print(" USC: ");
  Serial.print(usc);
  Serial.println(" ");
}

//---------------------------------------------------------------------------------------------------------------- ACTIVATE FAN
void activateFan() {

  digitalWrite(FAN, 1);
  delay(1000);
  digitalWrite(FAN, 0);
}

//---------------------------------------------------------------------------------------------------------------- TURN SERVO
void turnServo(float deg) {

  //hardware offset
  deg-=20;  

  constrain(deg, 0, 120);  // 120 max degree angle
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

  return (IRvolts < 0.3) ? 0 : (1 / ((IRvolts - 0.0587) / 11.159)) + 11;
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

  return (IRvolts < 0.3) ? 0 : (1 / ((IRvolts + 0.0413) / 11.482)) + 8;
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

  return (IRvolts < 0.4) ? 0 : (1 / ((IRvolts - 0.0804) / 23.929)) + 7.35;
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

  return (IRvolts < 0.4) ? 0 : (1 / ((IRvolts - 0.0704) / 23.018)) + 7.35;
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
      //Serial.println("HC-SR04: NOT found");
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
      //Serial.println("HC-SR04: Out of range");
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
    //Serial.println("HC-SR04: Out of range");
  } else {
    // Serial.print("HC-SR04:");
    // Serial.print(cm);
    // Serial.println("cm");

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

void objectAvoid(int direction) {
  return;
}

//---------------------------------------------------------------------------------------------------------------- TURN DEG
void turnDeg(int directionCW, float deg) {
  //add initial offset as gyro value drifts slightly, could drift to 360 degrees

  if (directionCW) {
    currentAngle = 5.0;
    deg += 5.0;
  } else {
    currentAngle = 355.0;
    deg = 355.0 - deg;
  }

  directionCW ? cw() : ccw();

  while (1) {
    gyroUpdate();

    // keep the angle between 0-360
    if (currentAngle < 0) {
      currentAngle += 360.0;
    } else if (currentAngle > 359) {
      currentAngle -= 360.0;
    }

    //exit condition
    if ((currentAngle >= deg) && directionCW) {
      stop();
      return;
    } else if ((currentAngle <= deg) && !directionCW) {
      stop();
      return;
    }

    // control the time per loop
    delay(T);
  }
}

void fast_flash_double_LED_builtin() {
  static byte indexer = 0;
  static unsigned long fast_flash_millis;
  if (millis() > fast_flash_millis) {
    indexer++;
    if (indexer > 4) {
      fast_flash_millis = millis() + 700;
      digitalWrite(LED_BUILTIN, LOW);
      indexer = 0;
    } else {
      fast_flash_millis = millis() + 100;
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
  }
}

void slow_flash_LED_builtin() {
  static unsigned long slow_flash_millis;
  if (millis() - slow_flash_millis > 2000) {
    slow_flash_millis = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}

void speed_change_smooth() {
  speed_val += speed_change;
  if (speed_val > 1000)
    speed_val = 1000;
  speed_change = 0;
}

#ifndef NO_BATTERY_V_OK
boolean is_battery_voltage_OK() {
  static byte Low_voltage_counter;
  static unsigned long previous_millis;

  int Lipo_level_cal;
  int raw_lipo;
  //the voltage of a LiPo cell depends on its chemistry and varies from about 3.5V (discharged) = 717(3.5V Min) https://oscarliang.com/lipo-battery-guide/
  //to about 4.20-4.25V (fully charged) = 860(4.2V Max)
  //Lipo Cell voltage should never go below 3V, So 3.5V is a safety factor.
  raw_lipo = analogRead(A0);
  Lipo_level_cal = (raw_lipo - 717);
  Lipo_level_cal = Lipo_level_cal * 100;
  Lipo_level_cal = Lipo_level_cal / 143;

  if (Lipo_level_cal > 0 && Lipo_level_cal < 160) {
    previous_millis = millis();
    Serial.print("Lipo level:");
    Serial.print(Lipo_level_cal);
    Serial.print("%");
    // Serial.print(" : Raw Lipo:");
    // Serial.println(raw_lipo);
    Serial.println("");
    Low_voltage_counter = 0;
    return true;
  } else {
    if (Lipo_level_cal < 0)
      Serial.println("Lipo is Disconnected or Power Switch is turned OFF!!!");
    else if (Lipo_level_cal > 160)
      Serial.println("!Lipo is Overchanged!!!");
    else {
      Serial.println("Lipo voltage too LOW, any lower and the lipo with be damaged");
      Serial.print("Please Re-charge Lipo:");
      Serial.print(Lipo_level_cal);
      Serial.println("%");
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
void HC_SR04_range() {
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
      Serial.println("HC-SR04: NOT found");
      return;
    }
  }

  // Measure how long the echo pin was held high (pulse width)
  // Note: the micros() counter will overflow after ~70 min

  t1 = micros();
  while (digitalRead(ECHO_PIN) == 1) {
    t2 = micros();
    pulse_width = t2 - t1;
    if (pulse_width > (MAX_DIST + 1000)) {
      Serial.println("HC-SR04: Out of range");
      return;
    }
  }

  t2 = micros();
  pulse_width = t2 - t1;

  // Calculate distance in centimeters and inches. The constants
  // are found in the datasheet, and calculated from the assumed speed
  //of sound in air at sea level (~340 m/s).
  cm = pulse_width / 58.0;
  inches = pulse_width / 148.0;

  // Print out results
  if (pulse_width > MAX_DIST) {
    Serial.println("HC-SR04: Out of range");
  } else {
    Serial.print("HC-SR04:");
    Serial.print(cm);
    Serial.println("cm");
  }
}
#endif

void Analog_Range_A4() {
  Serial.print("Analog Range A4:");
  Serial.println(analogRead(A4));
}

#ifndef NO_READ_GYRO
void GYRO_reading() {
  Serial.print("GYRO GYRO:");
  Serial.println(analogRead(GYRO));
}
#endif

//Serial command pasing
void read_serial_command() {
  if (SerialCom->available()) {
    char val = SerialCom->read();
    Serial.print("Speed:");
    Serial.print(speed_val);
    Serial.print(" ms ");

    //Perform an action depending on the command
    switch (val) {
      case 'w':  //Move Forward
      case 'W':
        forward();
        Serial.println("Forward");
        break;
      case 's':  //Move Backwards
      case 'S':
        reverse();
        Serial.println("Backwards");
        break;
      case 'q':  //Turn Left
      case 'Q':
        strafe_left();
        Serial.println("Strafe Left");
        break;
      case 'e':  //Turn Right
      case 'E':
        strafe_right();
        Serial.println("Strafe Right");
        break;
      case 'a':  //Turn Right
      case 'A':
        ccw();
        Serial.println("ccw");
        break;
      case 'd':  //Turn Right
      case 'D':
        cw();
        Serial.println("cw");
        break;
      case '-':  //Turn Right
      case '_':
        speed_change = -100;
        Serial.println("-100");
        break;
      case '=':
      case '+':
        speed_change = 100;
        Serial.println("+");
        break;
      default:
        stop();
        Serial.println("stop");
        break;
    }
  }
}

//----------------------Motor moments------------------------
//The Vex Motor Controller 29 use Servo Control signals to determine speed and direction, with 0 degrees meaning neutral https://en.wikipedia.org/wiki/Servo_control

void disable_motors() {
  left_font_motor.detach();   // detach the servo on pin left_front to turn Vex Motor Controller 29 Off
  left_rear_motor.detach();   // detach the servo on pin left_rear to turn Vex Motor Controller 29 Off
  right_rear_motor.detach();  // detach the servo on pin right_rear to turn Vex Motor Controller 29 Off
  right_font_motor.detach();  // detach the servo on pin right_front to turn Vex Motor Controller 29 Off

  pinMode(left_front, INPUT);
  pinMode(left_rear, INPUT);
  pinMode(right_rear, INPUT);
  pinMode(right_front, INPUT);
}

void enable_motors() {
  left_font_motor.attach(left_front);    // attaches the servo on pin left_front to turn Vex Motor Controller 29 On
  left_rear_motor.attach(left_rear);     // attaches the servo on pin left_rear to turn Vex Motor Controller 29 On
  right_rear_motor.attach(right_rear);   // attaches the servo on pin right_rear to turn Vex Motor Controller 29 On
  right_font_motor.attach(right_front);  // attaches the servo on pin right_front to turn Vex Motor Controller 29 On
}
void stop()  //Stop
{
  left_font_motor.writeMicroseconds(1500);
  left_rear_motor.writeMicroseconds(1500);
  right_rear_motor.writeMicroseconds(1500);
  right_font_motor.writeMicroseconds(1500);
}

void forward() {
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}

void reverse() {
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}

void ccw() {
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}

void ccwSlower() {
  left_font_motor.writeMicroseconds(1500 - speedSlowOffset);
  left_rear_motor.writeMicroseconds(1500 - speedSlowOffset);
  right_rear_motor.writeMicroseconds(1500 - speedSlowOffset);
  right_font_motor.writeMicroseconds(1500 - speedSlowOffset);
}

void cw() {
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}

void cwSlower() {
  left_font_motor.writeMicroseconds(1500 + speedSlowOffset);
  left_rear_motor.writeMicroseconds(1500 + speedSlowOffset);
  right_rear_motor.writeMicroseconds(1500 + speedSlowOffset);
  right_font_motor.writeMicroseconds(1500 + speedSlowOffset);
}

void cw2(float input) {
  left_font_motor.writeMicroseconds(1500 + input);
  left_rear_motor.writeMicroseconds(1500 + input);
  right_rear_motor.writeMicroseconds(1500 + input);
  right_font_motor.writeMicroseconds(1500 + input);
}

void strafe_left() {

  speed_val = 150;

  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val - 20);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}

void strafe_right() {

  speed_val = 150;
  
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val + 13);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}