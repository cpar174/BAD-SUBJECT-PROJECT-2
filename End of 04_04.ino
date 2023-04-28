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
// #include <SoftwareSerial.h>

#define FRONT A4
#define BACK A5
#define LEFT A6
#define RIGHT A7
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
  FIND,
  RUNNING,
  STOPPED
};

//Refer to Shield Pinouts.jpg for pin locations

//Default motor control pins
const byte left_front = 46;
const byte left_rear = 47;
const byte right_rear = 50;
const byte right_front = 51;


//Default ultrasonic ranging sensor pins, these pins are defined my the Shield
const int TRIG_PIN = 48;
const int ECHO_PIN = 49;

// Anything over 400 cm (23200 us pulse) is "out of range". Hit:If you decrease to this the ranging sensor but the timeout is short, you may not need to read up to 4meters.
const unsigned int MAX_DIST = 23200;

Servo left_font_motor;  // create servo object to control Vex Motor Controller 29
Servo left_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_font_motor;  // create servo object to control Vex Motor Controller 29
Servo turret_motor;


int speed_val = 100;
int speed_change;

//GRYO VALUES
int T = 100 ;                     // T is the time of one loop, 0.1 sec  
float gyroSensitivity = 0.007; // gyro sensitivity unit is (mv/degree/second) get from datasheet
float rotationThreshold = 1.5; // because of gyro drifting, defining rotation angular velocity less
float currentAngle = 0;        // current angle calculated by angular velocity integral
float gyroRate, angularVelocity, angleChange;
float lowestAnglePos = 0;
float IRvolts;

float Toffset = 1.1; //for IR
//float Toffset = 1; //for US

float speedSlowOffset= 75;


//Serial Pointer
HardwareSerial *SerialCom;

// // Serial Data input pin
// #define BLUETOOTH_RX 10
// // Serial Data output pin
// #define BLUETOOTH_TX 11
// SoftwareSerial BluetoothSerial(BLUETOOTH_RX, BLUETOOTH_TX);

//---------------------------------------------------------------------------------------------------------------- SETUP
int pos = 0;
void setup(void)
{
  turret_motor.attach(11);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(FRONT, INPUT); 
  pinMode(BACK, INPUT); 
  pinMode(LEFT, INPUT); 
  pinMode(RIGHT, INPUT); 
  // The Trigger pin will tell the sensor to range find
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);


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
  

  delay(1000); //settling time but no really needed

}


//---------------------------------------------------------------------------------------------------------------- MAIN LOOP
void loop(void) //main loop
{
  static STATE machine_state = INITIALISING;
  //Finite-state machine Code

  switch (machine_state) {
    case INITIALISING:
      machine_state = initialising();
      break;
    case FIND:
      machine_state = find();
      break;
    case RUNNING: //Lipo Battery Volage OK
      machine_state =  running();
      break;
    case STOPPED: //Stop of Lipo Battery voltage is too low, to protect Battery
      machine_state =  stopped();
      break;
  };


}


STATE initialising() {
  //initialising
  SerialCom->println("INITIALISING....");
  delay(1000); //One second delay to see the serial string "INITIALISING...."
  //SerialCom->println("Enabling Motors...");
  enable_motors();
  SerialCom->println("RUNNING STATE...");
  return FIND;
}

STATE find() {

  //--------------------------------------------------PUT ULTRASONIC ON RIGHT SIDE--------------------------------------------------
  
  float lowestDist, currentDist;
  float lowestAngle = 10000;
  bool turningCW = true;
  bool turningCCW = true;
  bool shortWall = false;
  float leftReading, rightReading;


  int leftIR = 1;
  int rightIR = 0;
  int strafeLeft = 1;
  int strafeRight = 0;
  /*
  drive(FORWARD, 15, leftIR);
  delay(500);
  strafe(strafeRight, 15);
  delay(500);
  drive(BACKWARDS, 26.25, leftIR);
  delay(500);
  strafe(strafeRight, 15);
  delay(500);
  drive(FORWARD, 37.5, leftIR);
  delay(500);
  strafe(strafeRight, 15);
  delay(500);
  drive(BACKWARDS, 48.75, leftIR);
  delay(500);
  strafe(strafeRight, 15);
  delay(500);
  drive(FORWARD, 60, leftIR);
  delay(500);
  strafe(strafeRight, 15);
  delay(500);
  drive(BACKWARDS, 48.75, rightIR);
  delay(500);
  strafe(strafeRight, 15);
  delay(500);
  drive(FORWARD, 37.5, rightIR);
  delay(500);
  strafe(strafeRight, 15);
  delay(500);
  drive(BACKWARDS, 26.25, rightIR);
  delay(500);
  strafe(strafeRight, 15);
  delay(500);
  drive(FORWARD, 15, rightIR);  


  delay(10000000000000);*/
  // while(1){

  //   strafeLeftToWall();
  //   delay(2000);
  // }

  bool useUS = true;

  lowestDist = 10000; //ultrasonic();
  cw();
  currentAngle = 1.0; //offset added to account for drift while reading 0

  // while(1)
  // {
  //   drive(FORWARD, 16, 1); 
  //   delay(5000);
  // }

  //find lowest distance to a wall
  while(turningCW){

    gyroRate = ((analogRead(A3) * 5.0) / 1024.0) - 2.5; // 2.5V = resting value offset

    // read out voltage divided the gyro sensitivity to calculate the angular velocity  
    angularVelocity = gyroRate/ gyroSensitivity; // from Data Sheet, gyroSensitivity is 0.007 V/dps 
    
    // if the angular velocity is less than the threshold, ignore it 
    if (angularVelocity >= rotationThreshold || angularVelocity <= -rotationThreshold) 
    { 
      // we are running a loop in T (of T/1000 second).  
      //angleChange = angularVelocity / (1000.0/T); 
      angleChange = (angularVelocity / (1000.0/T)) * Toffset; 
      currentAngle += angleChange;  
    }
  
    // keep the angle between 0-360 
    if (currentAngle < 0) { currentAngle += 360.0; } 
    else if (currentAngle > 359) { currentAngle -= 360.0; } 

    if(useUS){
      currentDist = ultrasonic();
    } else{
      currentDist = frontIR();
    }

    if(currentDist < 1){ currentDist = 1000000; }


    if(currentDist < lowestDist){
      lowestDist = currentDist;
      lowestAngle = currentAngle;
      //SerialCom->println("Found a lower value");
    }

    //turn almost 360 degrees
    if( currentAngle > 354.0 ){
      turningCW = false;
      stop();
    }

    // SerialCom->print("Current Angle: ");
    // SerialCom->print(currentAngle);
    // SerialCom->print(" : ");
    // SerialCom->print("Lowest Angle: ");
    // SerialCom->print(lowestAngle);
    // SerialCom->print(" : ");
    // SerialCom->print("Current Distance: ");
    // SerialCom->print(currentDist);
    // SerialCom->print(" : ");
    // SerialCom->print("Lowest Distance: ");
    // SerialCom->print(lowestDist);
    // SerialCom->println(" : ");

    delay (T); 
  }

  delay(1000);

  //SerialCom->print("Lowest Angle: ");
  //SerialCom->println(lowestAngle);

  bool firstSection = (lowestAngle < 40.0) ? true : false;           

  if(!firstSection){

    //SerialCom->println("Lowest angle is greater than 20 degrees");   

    cw();

    while( abs(lowestAngle - currentAngle) > 40.0)
    {
      gyroRate = ((analogRead(A3) * 5.0) / 1024.0) - 2.5; // 2.5V = resting value offset

      // read out voltage divided the gyro sensitivity to calculate the angular velocity  
      angularVelocity = gyroRate/ gyroSensitivity; // from Data Sheet, gyroSensitivity is 0.007 V/dps 
      
      // if the angular velocity is less than the threshold, ignore it 
      if (angularVelocity >= rotationThreshold || angularVelocity <= -rotationThreshold) 
      { 
        // we are running a loop in T (of T/1000 second).  
        //angleChange = angularVelocity / (1000.0/T); 
        angleChange = (angularVelocity / (1000.0/T)) * Toffset; 
        currentAngle += angleChange;  
      }
    
      // keep the angle between 0-360 
      if (currentAngle < 0) { currentAngle += 360.0; } 
      else if (currentAngle > 359) { currentAngle -= 360.0; } 

      if(useUS){
        currentDist = ultrasonic();
      } else{
        currentDist = frontIR();
        if(frontIR() == 0){ currentDist = 1000000; }
      }

      delay (T); 
    }

    stop();
    delay(1000);
  }

  //SerialCom->println("Slow time rolling");

  cwSlower();

  //while( abs(lowestAngle - currentAngle) > 0.8)

  float error = lowestAngle - currentAngle;

  if(useUS){
    currentDist = ultrasonic();
  } else{
    currentDist = frontIR();
  }

  if(currentDist < 1){ currentDist = 1000000; }
  
  float prevDist = currentDist + 10;
  int exit = 0;

  //float prevError = 0; //PENIS----------------------------------------------
  // SerialCom->print("Previous Distance: ");
  // SerialCom->print(prevDist);              
  // SerialCom->print("Current Distance: ");
  // SerialCom->println(currentDist);

  while(exit < 10) 
  {
    
    delay (50); 

    prevDist = currentDist;

  
    currentDist = ultrasonic();
  

    // if(currentDist > prevDist){
    //   exit++;
    // } else{
    //   if ((exit > 0)){
    //     exit--;
    //     SerialCom->print("MINUS MINUS: ");
    //   }
    // }

    if(currentDist > (prevDist - 0.3)){
      exit++;
    } else{
      if(exit > 0){
        exit--;
      }
    }
    /*

    if((currentDist-prevDist) >= 0.6){
      exit++;
    } else{
      if ((exit > 0) && ((currentDist-prevDist >= 1.0))){
        exit--;
        SerialCom->print("MINUS MINUS: ");
        SerialCom->print((currentDist-prevDist));
      }
    }*/
    // SerialCom->print("Previous Distance: ");
    // SerialCom->print(prevDist);              
    // SerialCom->print("  Current Distance: ");
    // SerialCom->print(currentDist);              

    //SerialCom->print("  Exit Buffer: ");
    //SerialCom->println(exit);
  }
  //SerialCom->println("Im out of the loop");    
  stop();

  delay(1000);

  turnDeg(CW, 90);

  delay(1000);

  strafeLeftToWall();

  delay(1000);

  drive(BACKWARDS,20, 1);

  //SerialCom->print("Ultrasonic: ");
  //SerialCom->println(ultrasonic());
  int a = ultrasonic();
  a = ultrasonic();
  a = ultrasonic();
  a = ultrasonic();
  if(ultrasonic() < 150){
    delay(1000);
    drive(FORWARD, 20, 1);
    delay(1000);
    turnDeg(CW,90);      
  }

  delay(500);

  drive(FORWARD, 17, leftIR);
  delay(500);
  strafe(strafeRight, 15);
  delay(500);
  drive(BACKWARDS, 26.25, leftIR);
  delay(500);
  strafe(strafeRight, 15);
  delay(500);
  drive(FORWARD, 37.5, leftIR);
  delay(500);
  strafe(strafeRight, 15);
  delay(500);
  drive(BACKWARDS, 48.75, leftIR);
  delay(500);
  strafe(strafeRight, 15);
  delay(500);
  drive(FORWARD, 60, leftIR);
  delay(500);
  strafe(strafeRight, 15);
  delay(500);
  drive(BACKWARDS, 48.75, rightIR);
  delay(500);
  strafe(strafeRight, 15);
  delay(500);
  drive(FORWARD, 37.5, rightIR);
  delay(500);
  strafe(strafeRight, 15);
  delay(500);
  drive(BACKWARDS, 26.25, rightIR);
  delay(500);
  strafe(strafeRight, 15);
  delay(500);
  drive(FORWARD, 17, rightIR);  

  delay(100000000);

  //MIGHT NEED TO AVERAGE THESE
  //eg. if > 2 out of 10 readings = 0, set to 0
  //leftReading = leftIR();
  //rightReading = rightIR();

  if( (leftReading == 0) && (rightReading == 0) ){
    shortWall = false;
  }
  else if ( (leftReading == 0) || (rightReading == 0) ){

  }

  //float offset = (lowestAngle < 180.0) ? 3.0 : -3.0;
  /*
  //orient front facing the wall
  while( abs(lowestAngle + offset - currentAngle) > 1.0)
  {
    
    if(lowestAngle < 180.0){
      cwSlower();
    } else { ccwSlower(); }
    
    gyroRate = ((analogRead(A3) * 5.0) / 1024.0) - 2.5; // 2.5V = resting value offset

    // read out voltage divided the gyro sensitivity to calculate the angular velocity  
    angularVelocity = gyroRate/ gyroSensitivity; // from Data Sheet, gyroSensitivity is 0.007 V/dps 
    
    // if the angular velocity is less than the threshold, ignore it 
    if (angularVelocity >= rotationThreshold || angularVelocity <= -rotationThreshold) 
    { 
      // we are running a loop in T (of T/1000 second).  
      angleChange = (angularVelocity / (1000.0/T)) * Toffset;
      currentAngle += angleChange;  
    } 
      
    // keep the angle between 0-360 
    if (currentAngle < 0) { currentAngle += 360.0; } 
    else if (currentAngle > 359) { currentAngle -= 360.0; } 
    
    delay (T); 
  }
  stop();

  SerialCom->println("Facing Wall on Left");
  delay(1000);

  //rotate 90 CCW to get distance away from right side
  turnDeg(CCW, 90);
  delay(100);
  rightReading = ultrasonic();

  lowestDist = ultrasonic();
  ccw();
  currentAngle = 1.0; //offset added to account for drift while reading 0

  //re-find lowest distance to a wall
  while(turningCCW){

    gyroRate = ((analogRead(A3) * 5.0) / 1024.0) - 2.5; // 2.5V = resting value offset

    // read out voltage divided the gyro sensitivity to calculate the angular velocity  
    angularVelocity = gyroRate/ gyroSensitivity; // from Data Sheet, gyroSensitivity is 0.007 V/dps 
    
    // if the angular velocity is less than the threshold, ignore it 
    if (angularVelocity >= rotationThreshold || angularVelocity <= -rotationThreshold) 
    { 
      // we are running a loop in T (of T/1000 second).  
      angleChange = (angularVelocity / (1000.0/T)) * Toffset;
      currentAngle += angleChange;  
    } 
  
    // keep the angle between 0-360 
    if (currentAngle < 0) { currentAngle += 360.0; } 
    else if (currentAngle > 359) { currentAngle -= 360.0; } 

    currentDist = ultrasonic();

    if(currentDist < lowestDist){
      lowestDist = currentDist;
      lowestAngle = currentAngle;
    }

    //turn 180 degrees
    if( currentAngle > 181.0 ){
      turningCCW = false;
      stop();
    }

    delay (T); 
  }

  delay(1000);
  rightReading = ultrasonic();  

  //orient front facing the wall
  while( abs(lowestAngle - currentAngle) > 5.0)
  {
    cw();
    
    gyroRate = ((analogRead(A3) * 5.0) / 1024.0) - 2.5; // 2.5V = resting value offset

    // read out voltage divided the gyro sensitivity to calculate the angular velocity  
    angularVelocity = gyroRate/ gyroSensitivity; // from Data Sheet, gyroSensitivity is 0.007 V/dps 
    
    // if the angular velocity is less than the threshold, ignore it 
    if (angularVelocity >= rotationThreshold || angularVelocity <= -rotationThreshold) 
    { 
      // we are running a loop in T (of T/1000 second).  
      angleChange = (angularVelocity / (1000.0/T)) * Toffset;
      currentAngle += angleChange;  
    } 
      
    // keep the angle between 0-360 
    if (currentAngle < 0) { currentAngle += 360.0; } 
    else if (currentAngle > 359) { currentAngle -= 360.0; } 
    
    delay (T); 
  }
  stop();

  delay(1000);

  //rotate a further 90 CW so that ultrasonic sensor is at the front
  turnDeg(CW, 90);

  delay(1000);

  strafeLeftToWall();

  //move to correct corner and orient

  //identify if on short wall or long wall
  if( (leftReading + rightReading) > 150)
  shortWall = ( (leftReading + rightReading) < 150) ? true : false;

  if(shortWall){
    drive(FORWARD, 15, 1);
    turnDeg(CW, 90);
  }
  else{
    drive(BACKWARDS, 15, 1);
  }

  delay(10000000);

*/  
  

  //// TILLING

  // int leftIR = 1;
  // int rightIR = 0;
  // int strafeLeft = 1;
  // int strafeRight = 0;

  // drive(FORWARD, 15, leftIR);
  // strafe(strafeRight, 15);
  // drive(BACKWARDS, 26.25, leftIR);
  // strafe(strafeRight, 15);
  // drive(FORWARD, 37.5, leftIR);
  // strafe(strafeRight, 15);
  // drive(BACKWARDS, 48.75, leftIR);
  // strafe(strafeRight, 15);
  // drive(FORWARD, 60, leftIR);
  // strafe(strafeRight, 15);
  // drive(BACKWARDS, 48.75, rightIR);
  // strafe(strafeRight, 15);
  // drive(FORWARD, 37.5, rightIR);
  // strafe(strafeRight, 15);
  // drive(BACKWARDS, 26.25, rightIR);
  // strafe(strafeRight, 15);
  // drive(FORWARD, 15, rightIR);  

  return FIND;
}


STATE running() {

  static unsigned long previous_millis;

  read_serial_command();
  fast_flash_double_LED_builtin();

  if (millis() - previous_millis > 500) {  //Arduino style 500ms timed execution statement
    previous_millis = millis();

    SerialCom->println("RUNNING---------");
    speed_change_smooth();
    Analog_Range_A4();

#ifndef NO_READ_GYRO
    GYRO_reading();
#endif

#ifndef NO_HC-SR04
    HC_SR04_range();
#endif

#ifndef NO_BATTERY_V_OK
    if (!is_battery_voltage_OK()) return STOPPED;
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

//Stop of Lipo Battery voltage is too low, to protect Battery
STATE stopped() {
  static byte counter_lipo_voltage_ok;
  static unsigned long previous_millis;
  int Lipo_level_cal;
  disable_motors();
  slow_flash_LED_builtin();

  if (millis() - previous_millis > 500) { //print massage every 500ms
    previous_millis = millis();
    SerialCom->println("STOPPED---------");


#ifndef NO_BATTERY_V_OK
    //500ms timed if statement to check lipo and output speed settings
    if (is_battery_voltage_OK()) {
      SerialCom->print("Lipo OK waiting of voltage Counter 10 < ");
      SerialCom->println(counter_lipo_voltage_ok);
      counter_lipo_voltage_ok++;
      if (counter_lipo_voltage_ok > 10) { //Making sure lipo voltage is stable
        counter_lipo_voltage_ok = 0;
        enable_motors();
        SerialCom->println("Lipo OK returning to RUN STATE");
        return RUNNING;
      }
    } else
    {
      counter_lipo_voltage_ok = 0;
    }
#endif
  }
  return STOPPED;
}

//----------------------ADDED FUNCTIONS----------------------

//---------------------------------------------------------------------------------------------------------------- FRONT IR
float FIRValues1 = 0;
float FIRValues2 = 0;
float FIRValues3 = 0;
float FIRValues4 = 0;
float FIRValues5 = 0;
float frontIR() { 
  
  FIRValues1 = analogRead(FRONT) * 5.0 / 1024.0;
  FIRValues2 = FIRValues1;// IRValues1;
  FIRValues3 = FIRValues2;//IRValues2;
  FIRValues4 = FIRValues3;//IRValues3;
  FIRValues5 = FIRValues4;//IRValues4;

  IRvolts = (FIRValues1 + FIRValues2 + FIRValues3 + FIRValues4 + FIRValues5)/5; //averaged values

  //IRvolts = analogRead(FRONT) * 5.0 / 1024.0;

  return (IRvolts < 0.3) ? 0 : (1 / ( ( IRvolts - 0.0587) / 11.159)) + 11; 
}

//---------------------------------------------------------------------------------------------------------------- BACK IR
float BIRValues1 = 0;
float BIRValues2 = 0;
float BIRValues3 = 0;
float BIRValues4 = 0;
float BIRValues5 = 0;
float backIR() { 
  
  //IRvolts = analogRead(BACK) * 5.0 / 1024.0;
  BIRValues1 = analogRead(BACK) * 5.0 / 1024.0;
  BIRValues2 = BIRValues1;// IRValues1;
  BIRValues3 = BIRValues2;//IRValues2;
  BIRValues4 = BIRValues3;//IRValues3;
  BIRValues5 = BIRValues4;//IRValues4;

  IRvolts = (BIRValues1 + BIRValues2 + BIRValues3 + BIRValues4 + BIRValues5)/5; //averaged values 

  return (IRvolts < 0.3) ? 0 : (1 / ( ( IRvolts + 0.0413) / 11.482)) + 8;
}

//---------------------------------------------------------------------------------------------------------------- LEFT IR
float LIRValues1 = 0;
float LIRValues2 = 0;
float LIRValues3 = 0;
float LIRValues4 = 0;
float LIRValues5 = 0;
float leftIR() { 

  //IRvolts = analogRead(LEFT) * 5.0 / 1024.0;
  LIRValues1 = analogRead(LEFT) * 5.0 / 1024.0;
  LIRValues2 = LIRValues1;// IRValues1;
  LIRValues3 = LIRValues2;//IRValues2;
  LIRValues4 = LIRValues3;//IRValues3;
  LIRValues5 = LIRValues4;//IRValues4;

  IRvolts = (LIRValues1 + LIRValues2 + LIRValues3 + LIRValues4 + LIRValues5)/5; //averaged values
  
  //return (IRvolts < 0.4) ? 0 : (1 / ( ( IRvolts - 0.3452) / 15.495)) + 7.35;
  return (IRvolts < 0.4) ? 0 : (1 / ( ( IRvolts - 0.0804) / 23.929)) + 7.35;
  
}

//---------------------------------------------------------------------------------------------------------------- RIGHT IR
float RIRValues1 = 0;
float RIRValues2 = 0;
float RIRValues3 = 0;
float RIRValues4 = 0;
float RIRValues5 = 0;
float rightIR() { 

  //IRvolts = analogRead(RIGHT) * 5.0 / 1024.0;
  RIRValues1 = analogRead(RIGHT) * 5.0 / 1024.0;
  RIRValues2 = RIRValues1;// IRValues1;
  RIRValues3 = RIRValues2;//IRValues2;
  RIRValues4 = RIRValues3;//IRValues3;
  RIRValues5 = RIRValues4;//IRValues4;

  IRvolts = (RIRValues1 + RIRValues2 + RIRValues3 + RIRValues4 + RIRValues5)/5; //averaged values

  //1 / ( ( IRvolts - 0.1529) / 22.811

  //return (IRvolts < 0.4) ? 0 : (1 / ( ( IRvolts - 0.3474) / 15.31)) + 7.35; 
  return (IRvolts < 0.4) ? 0 : (1 / ( ( IRvolts - 0.0704) / 23.018)) + 7.35;
  
}

//---------------------------------------------------------------------------------------------------------------- ULTRASONIOC
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
  while ( digitalRead(ECHO_PIN) == 0 ) {
    t2 = micros();
    pulse_width = t2 - t1;
    if ( pulse_width > (MAX_DIST + 1000)) {
      SerialCom->println("HC-SR04: NOT found");
      return 300.0;
    }
  }

  // Measure how long the echo pin was held high (pulse width)
  // Note: the micros() counter will overflow after ~70 min

  t1 = micros();
  while ( digitalRead(ECHO_PIN) == 1)
  {
    t2 = micros();
    pulse_width = t2 - t1;
    if ( pulse_width > (MAX_DIST + 1000) ) {
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
  USValues2 = USValues1;// IRValues1;
  USValues3 = USValues2;//IRValues2;
  USValues4 = USValues3;//IRValues3;
  USValues5 = USValues4;//IRValues4;

  cm = (USValues1 + USValues2 + USValues3 + USValues4 + USValues5)/5; //averaged values

  //cm = pulse_width / 58.0;
  inches = pulse_width / 148.0;

  // Print out results
  if ( pulse_width > MAX_DIST ) {
    //SerialCom->println("HC-SR04: Out of range");
  } else {
    // SerialCom->print("HC-SR04:");
    // SerialCom->print(cm);
    // SerialCom->println("cm");
    
    return cm;
  }
}

//----------------------------------------------------------------------------------------------------------------
void turnDeg(int directionCW, float deg)
{
  //add initial offset as gyro value drifts slightly, could drift to 360 degrees
  
  if(directionCW){
    currentAngle = 5.0;
    deg += 5.0;
  } else{
    currentAngle = 355.0;
    deg = 355.0 - deg;
  }

  directionCW ? cw() : ccw();

  while(1)
  {
    //UPDATE GYRO VALUES
    gyroRate = ((analogRead(A3) * 5.0) / 1024.0) - 2.5; // 2.5V = resting value offset

    // read out voltage divided the gyro sensitivity to calculate the angular velocity  
    angularVelocity = gyroRate/ gyroSensitivity; // from Data Sheet, gyroSensitivity is 0.007 V/dps 
    
    // if the angular velocity is less than the threshold, ignore it 
    if (angularVelocity >= rotationThreshold || angularVelocity <= -rotationThreshold) 
    { 
      // we are running a loop in T (of T/1000 second).  
      angleChange = (angularVelocity / (1000.0/T)) * Toffset * 0.955;
      currentAngle += angleChange;  
    } 
      
    // keep the angle between 0-360 
    if (currentAngle < 0) { currentAngle += 360.0; } 
    else if (currentAngle > 359) { currentAngle -= 360.0; } 

    //exit condition
    if((currentAngle >= deg) && directionCW){
      stop();
      return;
    }
    else if((currentAngle <= deg) && !directionCW){
      stop();
      return;
    }       
    
    // control the time per loop 
    delay (T);    
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
  int correction1 = 165; //LF
  int correction2 = 150; //LR
  int correction3 = 150; //RF
  int correction4 = 145; //RR

  int exit = 0;

  float Kp_FL = 25 * correction1/150;
  float Kp_FR = 25 * correction3/150;
  float Kp_RL = 25 * correction2/150;
  float Kp_RR = 25 * correction4/150;

  float Kp = 25;

  if(forwardYes){
    //FORWARD LOOP
   // while( (frontIR() <= 12) || (ultrasonic() > 15)){
  while(  (ultrasonic() > 15)){
     
    
      gyroRate = ((analogRead(A3) * 5.0) / 1024.0) - 2.5; // 2.5V = resting value offset

      angularVelocity = gyroRate/ gyroSensitivity; // from Data Sheet, gyroSensitivity is 0.007 V/dps 
      if (angularVelocity >= rotationThreshold || angularVelocity <= -rotationThreshold) { 
        angleChange = angularVelocity / (1000.0/T); 
        currentAngle += angleChange;  
      } 

      gyroError = 0 - currentAngle;

      distError = (useLeftIR) ? wallDist - leftIR() : wallDist - rightIR();
      sendData(useLeftIR);
      //SerialCom->print("Distance Error: ");
      //SerialCom->println(distError);
      
      if(gyroError > 2){

        veerRight = true;
        veerLeft = false;

      } else if(gyroError < -2){

        veerRight = false;
        veerLeft = true;
      } else {

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

      if(useLeftIR){

        if(distError > 1){

          veerRight = true;
          veerLeft = false;

        } else if(distError < -1){

          veerRight = false;
          veerLeft = true;

        } else{

          veerRight = false;
          veerLeft = false;
        }
      }
      else{
        
        if(distError > 1){

          veerRight = false;
          veerLeft = true;

        } else if(distError < -1){

          veerRight = true;
          veerLeft = false;

        } else{

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

      effortFL = Kp_FL*veerRight*abs(distError);
      //effortFL = (effortFL < -1*correction1) ? 0 : effortFL;
      effortFR = Kp_FR*veerLeft*abs(distError);
      //effortFR = (effortFR > correction4) ? 0 : effortFR;
      effortRL = Kp_RL*veerLeft*abs(distError);
      //effortRL = (effortRL < -1*correction2) ? 0 : effortRL;
      effortRR = Kp_RR*veerRight*abs(distError);
      //effortRR = (effortRR > correction3) ? 0 : effortRR;

      left_font_motor.writeMicroseconds(1500 + correction1 + effortFL);
      left_rear_motor.writeMicroseconds(1500 + correction2 + effortRL);
      right_font_motor.writeMicroseconds(1500 - correction3 - effortRR); //rear right
      right_rear_motor.writeMicroseconds(1500 - correction4 - effortFR); //front right

      // SerialCom->print("effortFL: ");
      // SerialCom->print(effortFL);
      // SerialCom->print(" effortFR: ");
      // SerialCom->print(effortFR);
      // SerialCom->print(" effortRL: ");
      // SerialCom->print(effortRL);
      // SerialCom->print(" effortRR: ");
      // SerialCom->println(effortRR);
      //Serial.print(frontIR());
      //SerialCom->print("  Gyro Error: ");
      //Serial.print(gyroError);
      //SerialCom->print("  Distance Error: ");
      ///Serial.println(distError);

      delay (T);
      // SerialCom->print("LEFT IR: ");
      // SerialCom->println(leftIR());
    }
    //SerialCom->print("Exit was: ");
    //SerialCom->println(frontIR());
  }
  else{ //BACKWARDS

    //while( (backIR() <= 10) || (backIR() > 20) )
    while(exit < 3)
    {
      if((backIR() < 30) && (backIR() > 10)){
        exit++;
      } else{
        exit = 0;
      }

      
      gyroRate = ((analogRead(A3) * 5.0) / 1024.0) - 2.5; // 2.5V = resting value offset

      angularVelocity = gyroRate/ gyroSensitivity; // from Data Sheet, gyroSensitivity is 0.007 V/dps 
      if (angularVelocity >= rotationThreshold || angularVelocity <= -rotationThreshold) { 
        angleChange = angularVelocity / (1000.0/T); 
        currentAngle += angleChange;  
      } 

      gyroError = 0 - currentAngle;

      distError = (useLeftIR) ? wallDist - leftIR() : wallDist - rightIR();
      sendData(useLeftIR);
      //SerialCom->print("Distance Error: ");
      //SerialCom->println(distError);

      if(gyroError > 2){

        veerRight = true;
        veerLeft = false;

      } else if(gyroError < -2){

        veerRight = false;
        veerLeft = true;
      } else {

        veerRight = false;
        veerLeft = false;
      }

      if(useLeftIR){

        if(distError > 1){

          veerRight = true;
          veerLeft = false;

        } else if(distError < -1){

          veerRight = false;
          veerLeft = true;

        } else{

          veerRight = false;
          veerLeft = false;
        }
      }
      else{
        
        if(distError > 1){

          veerRight = false;
          veerLeft = true;

        } else if(distError < -1){

          veerRight = true;
          veerLeft = false;

        } else{

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

      effortFL = Kp*veerLeft*abs(distError);
      //effortFL = (effortFL < -1*correction1) ? 0 : effortFL;
      effortFR = Kp*veerRight*abs(distError);
      //effortFR = (effortFR > correction4) ? 0 : effortFR;
      effortRL = Kp*veerRight*abs(distError);
      //effortRL = (effortRL < -1*correction2) ? 0 : effortRL;
      effortRR = Kp*veerLeft*abs(distError);
      //effortRR = (effortRR > correction3) ? 0 : effortRR;

      left_font_motor.writeMicroseconds(1500 - correction1 - effortFL);
      left_rear_motor.writeMicroseconds(1500 - correction2 - effortRL);
      right_font_motor.writeMicroseconds(1500 + correction3 + effortRR); //rear right
      right_rear_motor.writeMicroseconds(1500 + correction4 + effortFR); //front right

      //SerialCom->print("Back sensor: ");
      //SerialCom->println(backIR());

      delay (T);
      // SerialCom->print("BACK IR EXIT: ");
      // SerialCom->println(backIR());
    }
  }


  stop();
}

//---------------------------------------------------------------------------------------------------------------- SRAFT
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


  if(leftYes){
    
    //Rotate counterclockwise
    while( time < 1500)
    {
      sendData(true);
      gyroRate = ((analogRead(A3) * 5.0) / 1024.0) - 2.5; // 2.5V = resting value offset

      angularVelocity = gyroRate/ gyroSensitivity; // from Data Sheet, gyroSensitivity is 0.007 V/dps 
      if (angularVelocity >= rotationThreshold || angularVelocity <= -rotationThreshold) { 
        angleChange = angularVelocity / (1000.0/T); 
        currentAngle += angleChange;  
      } 

      gyroError = 0 - currentAngle;

      distError = wallDist - frontIR();

      if(gyroError > 2){

        turnCCW = false;
        turnCW = true;

      } else if(gyroError < -2){

        turnCCW = true;
        turnCW = false;
        
      } else {

        turnCCW = false;
        turnCW = false;
      }

      if(distError > 2){

        // moveForward = true;
        // moveBack = false;

        moveForward = false;
        moveBack = true;

      } else if(distError < -2){

        // moveForward = false;
        // moveBack = true;

        moveForward = true;
        moveBack = false;

      } else{

        moveForward = false;
        moveBack = false;
      }

      effortFL = Kp_gyro*turnCW*abs(gyroError) - Kp_gyro*turnCCW*abs(gyroError) + Kp_dist*moveForward*abs(distError) - Kp_dist*moveBack*abs(distError);
      effortFL = (effortFL > correction1) ? 0 : effortFL;
      effortFR = Kp_gyro*turnCW*abs(gyroError) - Kp_gyro*turnCCW*abs(gyroError) + Kp_dist*moveForward*abs(distError) - Kp_dist*moveBack*abs(distError);
      effortFR = (effortFR > correction4) ? 0 : effortFR;
      effortRL = Kp_gyro*turnCW*abs(gyroError) - Kp_gyro*turnCCW*abs(gyroError) - Kp_dist*moveForward*abs(distError) + Kp_dist*moveBack*abs(distError);
      effortRL = (effortRL > correction2) ? 0 : effortRL;
      effortRR = Kp_gyro*turnCW*abs(gyroError) - Kp_gyro*turnCCW*abs(gyroError) - Kp_dist*moveForward*abs(distError) + Kp_dist*moveBack*abs(distError);
      effortRR = (effortRR > correction3) ? 0 : effortRR;

      effortFL = 0;
      effortFR = 0;
      effortRL = 0;
      effortRR = 0;

      left_font_motor.writeMicroseconds(1500 - correction1 + effortFL);
      left_rear_motor.writeMicroseconds(1500 + correction2 + effortRL);
      right_font_motor.writeMicroseconds(1500 + correction3 + effortRR); //rear right
      right_rear_motor.writeMicroseconds(1500 - correction4 + effortFR); //front right

      // SerialCom->print(currentAngle);
      // SerialCom->println();

      delay (T);

      time += T;
    }

  }
  else{ // else rotate clockwise

    while( time < 1500)
    {
      sendData(true);
      gyroRate = ((analogRead(A3) * 5.0) / 1024.0) - 2.5; // 2.5V = resting value offset

      angularVelocity = gyroRate/ gyroSensitivity; // from Data Sheet, gyroSensitivity is 0.007 V/dps 
      if (angularVelocity >= rotationThreshold || angularVelocity <= -rotationThreshold) { 
        angleChange = angularVelocity / (1000.0/T); 
        currentAngle += angleChange;  
      } 

      gyroError = 0 - currentAngle;

      distError = wallDist - frontIR();

      if(gyroError > 2){

        turnCCW = false;
        turnCW = true;

      } else if(gyroError < -2){

        turnCCW = true;
        turnCW = false;
        
      } else {

        turnCCW = false;
        turnCW = false;
      }

      if(distError > 2){

        moveForward = false;
        moveBack = true;

      } else if(distError < -2){

        moveForward = true;
        moveBack = false;

      } else{

        moveForward = false;
        moveBack = false;
      }

      effortFL = Kp_gyro*turnCW*abs(gyroError) - Kp_gyro*turnCCW*abs(gyroError) + Kp_dist*moveForward*abs(distError) - Kp_dist*moveBack*abs(distError);
      effortFL = (effortFL > correction1) ? 0 : effortFL;
      effortFR = Kp_gyro*turnCW*abs(gyroError) - Kp_gyro*turnCCW*abs(gyroError) + Kp_dist*moveForward*abs(distError) - Kp_dist*moveBack*abs(distError);
      effortFR = (effortFR > correction4) ? 0 : effortFR;
      effortRL = Kp_gyro*turnCW*abs(gyroError) - Kp_gyro*turnCCW*abs(gyroError) - Kp_dist*moveForward*abs(distError) + Kp_dist*moveBack*abs(distError);
      effortRL = (effortRL > correction2) ? 0 : effortRL;
      effortRR = Kp_gyro*turnCW*abs(gyroError) - Kp_gyro*turnCCW*abs(gyroError) - Kp_dist*moveForward*abs(distError) + Kp_dist*moveBack*abs(distError);
      effortRR = (effortRR > correction3) ? 0 : effortRR;

      effortFL = 0;
      effortFR = 0;
      effortRL = 0;
      effortRR = 0;

      left_font_motor.writeMicroseconds(1500 + correction1 + effortFL);
      left_rear_motor.writeMicroseconds(1500 - correction2 + effortRL);
      right_font_motor.writeMicroseconds(1500 - correction3 + effortRR + 10); //rear right
      right_rear_motor.writeMicroseconds(1500 + correction4 + effortFR); //front right

      // SerialCom->print(currentAngle);
      // SerialCom->println();

      delay (T);

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

  int correction1 = 110+50; //FL
  int correction2 = 100+50; //RF
  int correction3 = 90+50;  //RR
  int correction4 = 100+50; //FR

  int exit;

  while( (leftIR() > 16.0) || (leftIR() <= 7) )
  {
    //SerialCom->print("Left IR: ");
    //SerialCom->println(leftIR());
    
    gyroRate = ((analogRead(A3) * 5.0) / 1024.0) - 2.5; // 2.5V = resting value offset

    angularVelocity = gyroRate/ gyroSensitivity; // from Data Sheet, gyroSensitivity is 0.007 V/dps 
    if (angularVelocity >= rotationThreshold || angularVelocity <= -rotationThreshold) { 
      angleChange = angularVelocity / (1000.0/T); 
      currentAngle += angleChange;  
    } 

    gyroError = 0 - currentAngle;

    if(gyroError > 2){

      turnCCW = true;
      turnCW = false;

    } else if(gyroError < -2){

      turnCCW = false;
      turnCW = true;
      
    } else {

      turnCCW = false;
      turnCW = false;
    }

    effortFL = Kp_gyro*turnCW*abs(gyroError) - Kp_gyro*turnCCW*abs(gyroError);
    effortFL = (effortFL > correction1) ? 0 : effortFL;
    effortFR = Kp_gyro*turnCW*abs(gyroError) - Kp_gyro*turnCCW*abs(gyroError);
    effortFR = (effortFR > correction4) ? 0 : effortFR;
    effortRL = Kp_gyro*turnCW*abs(gyroError) - Kp_gyro*turnCCW*abs(gyroError);
    effortRL = (effortRL > correction2) ? 0 : effortRL;
    effortRR = Kp_gyro*turnCW*abs(gyroError) - Kp_gyro*turnCCW*abs(gyroError);
    effortRR = (effortRR > correction3) ? 0 : effortRR;

    left_font_motor.writeMicroseconds(1500 - correction1 + effortFL);
    left_rear_motor.writeMicroseconds(1500 + correction2 + effortRL);
    right_font_motor.writeMicroseconds(1500 + correction3 + effortRR); //rear right
    right_rear_motor.writeMicroseconds(1500 - correction4 + effortFR); //front right

    delay (T);
  }

  stop();
}

void fast_flash_double_LED_builtin()
{
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

void slow_flash_LED_builtin()
{
  static unsigned long slow_flash_millis;
  if (millis() - slow_flash_millis > 2000) {
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
  //the voltage of a LiPo cell depends on its chemistry and varies from about 3.5V (discharged) = 717(3.5V Min) https://oscarliang.com/lipo-battery-guide/
  //to about 4.20-4.25V (fully charged) = 860(4.2V Max)
  //Lipo Cell voltage should never go below 3V, So 3.5V is a safety factor.
  raw_lipo = analogRead(A0);
  Lipo_level_cal = (raw_lipo - 717);
  Lipo_level_cal = Lipo_level_cal * 100;
  Lipo_level_cal = Lipo_level_cal / 143;

  if (Lipo_level_cal > 0 && Lipo_level_cal < 160) {
    previous_millis = millis();
    SerialCom->print("Lipo level:");
    SerialCom->print(Lipo_level_cal);
    SerialCom->print("%");
    // SerialCom->print(" : Raw Lipo:");
    // SerialCom->println(raw_lipo);
    SerialCom->println("");
    Low_voltage_counter = 0;
    return true;
  } else {
    if (Lipo_level_cal < 0)
      SerialCom->println("Lipo is Disconnected or Power Switch is turned OFF!!!");
    else if (Lipo_level_cal > 160)
      SerialCom->println("!Lipo is Overchanged!!!");
    else {
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

#ifndef NO_HC-SR04
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
  while ( digitalRead(ECHO_PIN) == 0 ) {
    t2 = micros();
    pulse_width = t2 - t1;
    if ( pulse_width > (MAX_DIST + 1000)) {
      SerialCom->println("HC-SR04: NOT found");
      return;
    }
  }

  // Measure how long the echo pin was held high (pulse width)
  // Note: the micros() counter will overflow after ~70 min

  t1 = micros();
  while ( digitalRead(ECHO_PIN) == 1)
  {
    t2 = micros();
    pulse_width = t2 - t1;
    if ( pulse_width > (MAX_DIST + 1000) ) {
      SerialCom->println("HC-SR04: Out of range");
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
  if ( pulse_width > MAX_DIST ) {
    SerialCom->println("HC-SR04: Out of range");
  } else {
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
  SerialCom->print("GYRO A3:");
  SerialCom->println(analogRead(A3));
}
#endif

//Serial command pasing
void read_serial_command()
{
  if (SerialCom->available()) {
    char val = SerialCom->read();
    SerialCom->print("Speed:");
    SerialCom->print(speed_val);
    SerialCom->print(" ms ");

    //Perform an action depending on the command
    switch (val) {
      case 'w'://Move Forward
      case 'W':
        forward ();
        SerialCom->println("Forward");
        break;
      case 's'://Move Backwards
      case 'S':
        reverse ();
        SerialCom->println("Backwards");
        break;
      case 'q'://Turn Left
      case 'Q':
        strafe_left();
        SerialCom->println("Strafe Left");
        break;
      case 'e'://Turn Right
      case 'E':
        strafe_right();
        SerialCom->println("Strafe Right");
        break;
      case 'a'://Turn Right
      case 'A':
        ccw();
        SerialCom->println("ccw");
        break;
      case 'd'://Turn Right
      case 'D':
        cw();
        SerialCom->println("cw");
        break;
      case '-'://Turn Right
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
//The Vex Motor Controller 29 use Servo Control signals to determine speed and direction, with 0 degrees meaning neutral https://en.wikipedia.org/wiki/Servo_control

void disable_motors()
{
  left_font_motor.detach();  // detach the servo on pin left_front to turn Vex Motor Controller 29 Off
  left_rear_motor.detach();  // detach the servo on pin left_rear to turn Vex Motor Controller 29 Off
  right_rear_motor.detach();  // detach the servo on pin right_rear to turn Vex Motor Controller 29 Off
  right_font_motor.detach();  // detach the servo on pin right_front to turn Vex Motor Controller 29 Off

  pinMode(left_front, INPUT);
  pinMode(left_rear, INPUT);
  pinMode(right_rear, INPUT);
  pinMode(right_front, INPUT);
}

void enable_motors()
{
  left_font_motor.attach(left_front);  // attaches the servo on pin left_front to turn Vex Motor Controller 29 On
  left_rear_motor.attach(left_rear);  // attaches the servo on pin left_rear to turn Vex Motor Controller 29 On
  right_rear_motor.attach(right_rear);  // attaches the servo on pin right_rear to turn Vex Motor Controller 29 On
  right_font_motor.attach(right_front);  // attaches the servo on pin right_front to turn Vex Motor Controller 29 On
}
void stop() //Stop
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

void reverse ()
{
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}

void ccw ()
{
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}

void ccwSlower ()
{
  left_font_motor.writeMicroseconds(1500 - speedSlowOffset);
  left_rear_motor.writeMicroseconds(1500 - speedSlowOffset);
  right_rear_motor.writeMicroseconds(1500 - speedSlowOffset);
  right_font_motor.writeMicroseconds(1500 - speedSlowOffset);
}

void cw ()
{
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}

void cwSlower ()
{
  left_font_motor.writeMicroseconds(1500 + speedSlowOffset);
  left_rear_motor.writeMicroseconds(1500 + speedSlowOffset);
  right_rear_motor.writeMicroseconds(1500 + speedSlowOffset);
  right_font_motor.writeMicroseconds(1500 + speedSlowOffset);
}

void cw2 (float input)
{
  left_font_motor.writeMicroseconds(1500 + input);
  left_rear_motor.writeMicroseconds(1500 + input);
  right_rear_motor.writeMicroseconds(1500 + input);
  right_font_motor.writeMicroseconds(1500 + input);
}

void strafe_left ()
{
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}

void strafe_right ()
{
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}

//gloabal variable - caliabrate the coords at the end of the corner find
//at the end of corner find set the value so the sensors = 15cm
float plot_x_offset = 0.0;
float plot_y_offset = 0.0;

//for offse3t of the sensor relative to the middle of the robot
float x_sensor_offset = 0.0;
float y_sensor_offset = 0.0;

//values to be sent.
float plot_x = 0.0;
float plot_y = 0.0;

float y_total = 0.0; //(SET VALUE)

//This function sends the x,y values for plotting
void sendData(bool leftYes){

  //read ultrasonic
  float x_reading = ultrasonic(); //This takes the reading for the x displacement
  //read ir sides
  float y_reading_left = leftIR(); //takes the reading for the y-displacement
  float y_reading_right = rightIR();


  //output x value with offsets
  plot_x = x_reading + plot_x_offset + x_sensor_offset;
  SerialCom->print(plot_x);
  SerialCom->print(",");

  //output y with offsets
  //If left reading is less than 70cm use the left value 
  //else use the right value.
  if(y_reading_left <= 70){
    plot_y = y_reading_left + y_sensor_offset + plot_y_offset;
  } else {
    plot_y = y_total - (y_reading_right + y_sensor_offset + plot_y_offset);
  }

  if(leftYes){
    SerialCom->print(leftIR());
    SerialCom->print(",");
    SerialCom->println();
  }
  else{
    SerialCom->print(120-rightIR());
    SerialCom->print(",");
    SerialCom->println();
  }
  // SerialCom->print(plot_y);
  // SerialCom->print(",");
  // SerialCom->println();

}