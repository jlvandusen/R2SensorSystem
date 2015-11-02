// =======================================================================================
//        SHADOW_MD:  Small Handheld Arduino Droid Operating Wand + MarcDuino
// =======================================================================================
//                          Last Revised Date: 10/31/2015
//                             Version 3.5.31
//                       Revised By: vint43 / jlvandusen
//                Inspired by the PADAWAN / KnightShade SHADOW effort
// =======================================================================================
//
//         This program is free software: you can redistribute it and/or modify it for
//         your personal use and the personal use of other astromech club members.
//
//         This program is distributed in the hope that it will be useful
//         as a courtesy to fellow astromech club members wanting to develop
//         their own droid control system.
//
//         IT IS OFFERED WITHOUT ANY WARRANTY; without even the implied warranty of
//         MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
//
//         You are using this software at your own risk and, as a fellow club member, it is
//         expected you will have the proper experience / background to handle and manage that
//         risk appropriately.  It is completely up to you to insure the safe operation of
//         your droid and to validate and test all aspects of your droid control system.
//
// =======================================================================================
//   Note: You will need a Arduino Mega ADK rev3 or DUE to run this sketch and up to 4 more 
//   arduinos for sensor input.
//   NOTE: A normal Arduino (Uno, Duemilanove etc.) doesn't have enough SRAM and FLASH
//
//   This is written to be a SPECIFIC Sketch - supporting only one type of controller
//      - PS3 Move Navigation + MarcDuino Dome and Body Controller and R2 Sensor System
//      - R2Sensor system which includes complete collision detection and auto pilot of R2
//      - Complete i2c library support for compass, sensor and communications
//      - Voice Control and Recognition using EasyVR 2.0 Shield or higher over i2c
//   PS3 Bluetooth library - developed by Kristian Lauszus (kristianl@tkjelectronics.com)
//   For more information visit my blog: http://blog.tkjelectronics.dk/ or
//
//   Sabertooth (Foot Drive):
//         Set Sabertooth 2x32 or 2x25 Dip Switches: 1 and 2 Down, All Others Up
//
//   SyRen 10 Dome Drive:
//         For SyRen packetized Serial Set Switches: 1, 2 and 4 Down, All Others Up
//         NOTE:  Support for SyRen Simple Serial has been removed, due to problems.
//         Please contact DimensionEngineering to get an RMA to flash your firmware
//         Some place a 10K ohm resistor between S1 & GND on the SyRen 10 itself
//
//    EasyVR 2.x or 3.x shield:
//         Needs a UNO or Duemilanove arduino to work on as its full shield 1.0 compliant
//         Utilizes the R2 Sensor sketch to communicate over I2C
//
//    IR Sensors (PING):
//         Use the 4 wire sensors with seperate ECHO and TRIG for better comms (HC-04)
//
//    I2C Bus:
//         Uses PINS 20,21 for SCL/SDA on Mega to PINS A4 and A5 on the others.  This allows
//         all devices to chat across common interface (2 wire) back to the main SHADOW
//         master controller.  Curren Sensor systems include: Center, Left, Right and voice.
// 
// =======================================================================================
//
// ---------------------------------------------------------------------------------------
// Enable or disable configuration portions of the code below
// ---------------------------------------------------------------------------------------

// #define SHADOW_DEBUG      // uncomment this for console DEBUG output
// #define SHADOW_VERBOSE    // uncomment this for console VERBOSE output
// #define PATROL_DEBUG      // uncomment this to test collision detection using sensors and movement (patrol only)
// #define VOICE_DEBUG      // uncomment this to test collision detection using sensors and movement (patrol only)
// #define COMPASS_DEBUG
#define COMPASS
#define COMPASS_LSM303    // uncomment this for accurate dome to body positioning using 2 LSM303 from Adafruit Technologies
#define COMPASS_HMC5883L  // uncomment this for accurate dome to body positioning using 1 LSM303 and 1 HMC5883L from Adafruit Technologies (without multiplexer)
#define COLLISION         // uncomment this for collision detection and PING ability using 6 IR Sensors R2Sensor System
// #define FOLLOW            // uncomment this for following humans using 5 PIR sensors on BODY sensor system
// #define VOICE             // uncomment this for Voice Recognition and control ability using the EasyVR 2.0 or higher system
// #define SOUNDDETECTION    // uncomment for sound sensor positioning of the dome (will require 4 sound sensors)
// #define USE_GPS           // uncomment this for GPS use for positioning and waypoint navigation using Adafruit GPS breakout board.
#define BTSupport         // uncomment for BT support for programming on serial
// #define USE_LCD           // uncomment this for LCD Support for Navigation and other future options
// #define COINSLOTS         // uncomment this for coinslot led ability through Marcduino controller
// #define UTILARMS          // uncomment this for utility arm control ability through Marcduino controller

bool isDebug = true;         // enable/disable sensor output from I2C as well as decision making functions


// ---------------------------------------------------------------------------------------
//                        General User Settings
// ---------------------------------------------------------------------------------------


String PS3ControllerFootMac = "00:07:04:EC:A5:18";  //Set this to your FOOT PS3 controller MAC address
String PS3ControllerDomeMAC = "00:06:F5:5A:BD:17";  //Set to a secondary DOME PS3 controller MAC address (Optional)

String PS3ControllerBackupFootMac = "00:06:F5:64:60:3E";  //Set to the MAC Address of your BACKUP FOOT controller (Optional)
String PS3ControllerBackupDomeMAC = "XX";  //Set to the MAC Address of your BACKUP DOME controller (Optional)

byte drivespeed1 = 70;    // For Speed Setting (Normal): 0-stop, 127-full speed.
byte drivespeed2 = 127;   // For Speed Setting (Over Throttle): 0-stop, 127-full speed.

byte turnspeed = 75;      // The higher this number the faster it will spin in place, lower - the easier to control.
                          // Recommend beginner: 40 to 50, experienced: 50+, I like 75

byte domespeed = 95;      // If using a speed controller for the dome, sets the top speed
                          // Use a number up to 127

byte ramping = 1;         // Ramping: the lower this number the longer R2 will take to speedup or slow down,
                          // change this by increments of 1

byte joystickFootDeadZoneRange = 15;  // For controllers that centering problems, use the lowest number with no drift
byte joystickDomeDeadZoneRange = 10;  // For controllers that centering problems, use the lowest number with no drift

byte driveDeadBandRange = 10;     // Used to set the Sabertooth DeadZone for foot motors

int invertturnDirection = -1;     // This may need to be set to 1 for some configurations

byte domeAutoSpeed = 50;          // Speed used when dome automation is active - Valid Values: 50 - 100
int time360DomeTurn = 2500;       // milliseconds for dome to complete 360 turn at domeAutoSpeed - Valid Values: 2000 - 8000 (2000 = 2 seconds)
int turnnum = 0;                  // Used for Turning for motor controller (moved global)

// ---------------------------------------------------------------------------------------
//               General User Settings
// ---------------------------------------------------------------------------------------

// ---------------------------------------------------------------------------------------
//                          Drive / Sensor Controller Settings
// ---------------------------------------------------------------------------------------
// int marcDuinoBaudRate =        9600;  // Set the baud rate for the communication to Marcduino Panel/Sensor controller
// int GPSBaudRate =              9600;  // Set the baud rate for the communication to Adafruit GPS Sensor
// int motorControllerBaudRate =  9600;  // Set the baud rate for the Syren and Sabertooth motor controllers (dome and feet)
// int BTdeviceBaudRate =         9600;  // Set the baud rate for the Bluetooch Client for Arduino
// for packetized options are: 2400, 9600, 19200 and 38400 (anything faster not supported)

#define SYREN_ADDR         129      // Serial Address for Dome Syren
#define SABERTOOTH_ADDR    128      // Serial Address for Foot Sabertooth
#define ENABLE_UHS_DEBUGGING 1
#define TCAADDR           0x70      // Define address of the adafruit multiplexer


// ---------------------------------------------------------------------------------------
//                          Libraries
// ---------------------------------------------------------------------------------------

#include <Marcduino.h>              // New Marcduino Library additions (must include marcduino library)
#include <usbhub.h>                 // Support for USB hub to connect Bluetooth
#include <PS3BT.h>                  // PS3 Bluetooth Support
#include <Sabertooth.h>             // Motor Controller(s)


#ifdef COLLISION
  #include <Math.h>
  #include <moving_average.h>         // simple moving average class; for Sonar functionality
#endif

#ifdef COMPASS                                                        // Enable COMPASS Configurations
  #include <Wire.h>
  #include <Adafruit_Sensor.h>

#ifdef COMPASS_LSM303                                                 // i2c accessible LSM303 on the main arduino and LSM303 in the dome arduino VIA Serial
  #include <Adafruit_LSM303_U.h>
  Adafruit_LSM303_Mag_Unified magdome = Adafruit_LSM303_Mag_Unified(1); // Body Compass sensor
  sensors_event_t magdome_event;
#endif

#ifdef COMPASS_HMC5883L
  #include <Adafruit_HMC5883_U.h>                                       // mag sensor
  Adafruit_HMC5883_Unified compass = Adafruit_HMC5883_Unified(0);   // Dome Compass sensor (non LSM303) NO NEED FOR MULTIPLEXER
  sensors_event_t compass_event;
#endif

#ifndef COMPASS_HMC5883L
  Adafruit_LSM303_Mag_Unified magbody = Adafruit_LSM303_Mag_Unified(0); // Dome Compass sensor
  sensors_event_t magbody_event;
#endif

#endif // COMPASS

#ifdef BTSupport
  #include <SoftwareSerial.h>
#endif // BTSupport

#ifdef USE_LCD
  #include <LiquidCrystal_I2C.h>                    // LCD library
#endif

#ifdef USE_GPS
  #include <waypointClass.h>                        // custom class to manaage GPS waypoints
  #include <Adafruit_GPS.h>                         // GPS
// #include <SPI.h>                               // Required library to support SD card driver below
// #include <SD.h>                                // Store GPS data to SD card
#ifndef BTSupport
  #include <SoftwareSerial.h>
#endif
#endif

#ifdef dobogusinclude
#include <spi4teensy3.h>                          // Satisfy IDE, which only needs to see the include statment in the Arduino UNO.
#endif

// ---------------------------------------------------------------------------------------
//                          Variables
// ---------------------------------------------------------------------------------------

long previousDomeMillis = millis();         // if using a dome controller (PS3 bluetooth)
long previousFootMillis = millis();         // if using a foot controller (PS3 bluetooth)
long previousMarcDuinoMillis = millis();    // if using a marcduino sensor controller
long previousDomeToggleMillis = millis();
long previousSpeedToggleMillis = millis();
long currentMillis = millis();

int serialLatency = 25;           // This is a delay factor in ms to prevent queueing of the Serial data.
// 25ms seems approprate for HardwareSerial, values of 50ms or larger are needed for Softare Emulation

int marcDuinoButtonCounter = 0;   // if using a marcduino sensor controller
int speedToggleButtonCounter = 0;
int domeToggleButtonCounter = 0;

#ifdef FOLLOW
int pirPos[] = {-270,-90,0,90,270};   // positions for dome or body (Moving the body or the dome based on current COMPASS finding (dome or body).
#endif

Sabertooth *ST = new Sabertooth(SABERTOOTH_ADDR, Serial2);
Sabertooth *SyR = new Sabertooth(SYREN_ADDR, Serial2);

// ---------------------------------------------------------------------------------------
//                         USB AND BLUETOOTH DEVICES
// ---------------------------------------------------------------------------------------

USB Usb;
BTD Btd(&Usb);
PS3BT *PS3NavFoot = new PS3BT(&Btd);
PS3BT *PS3NavDome = new PS3BT(&Btd);

//Used for PS3 Fault Detection
uint32_t msgLagTime = 0;
uint32_t lastMsgTime = 0;
uint32_t currentTime = 0;
uint32_t lastLoopTime = 0;
int badPS3Data = 0;
int badPS3DataDome = 0;

boolean firstMessage = true;
String output = "";

boolean isFootMotorStopped = true;
boolean isDomeMotorStopped = true;
int footDriveSpeed = 0;

boolean overSpeedSelected = false;

boolean isPS3NavigatonInitialized = false;
boolean isSecondaryPS3NavigatonInitialized = false;

boolean isStickEnabled = true;

boolean WaitingforReconnect = false;
boolean WaitingforReconnectDome = false;

boolean mainControllerConnected = false;
boolean domeControllerConnected = false;

// =======================================================================================
//                            R2Sensor Variables
// =======================================================================================
#ifdef COLLISION
String Part = "shadow";                               // scan targets 1 = left, 2 = right 3 = center, 4 = voice and 5 = dome
int partnum, front, side, back, nav, voicecmd, body;  // Set default distances and direction so if R2 is moving it doesnt come to stop.
int choice;                                           // Used to store the Random choice sequence in R2Decision();

int distance = 0;
int part, leftfront, rightfront, leftside, rightside, centerfront, centerleft, centerright, leftback, rightback, frontcombined, backcombined;
int distanceleftfront, distancerightfront, distanceleftside, distancerightside, distanceleftback, distancerightback;

enum States {Stopped, MovingFwd, MovingBck, Turning, Listen};   //enum state and their status;
States R2state = Stopped;

enum Modes {None, Aware, Sleep, Playfull, Waypoint, Following, Searching, Patroling, Guarding, Scanning};   //enum modes and their status;
Modes R2mode = Aware;

enum DomeStatus {Home,Turn,Ready,Error}; // enum status of Dome
DomeStatus R2domeStatus = Home;

enum BodyStatus {on,off}; // enum status of body
BodyStatus R2bodyStatus = on;


#define R2MOVE_LEFT       -25                    // Sets for turnnum talks to sabertooth controller (negative is left)
#define R2MOVE_RIGHT       25                    // Sets for turnnum talks to sabertooth controller (positive is right)
#define R2MOVE_STRAIGHT    0                     // Sets for turnnum talks to sabertooth controller (neutral is straight)

enum directions {left = R2MOVE_LEFT, right = R2MOVE_RIGHT, straight = R2MOVE_STRAIGHT} ;
directions turnDirection = straight;

#define FAST_SPEED        -90                    // Speeds (range: -127 to 127) where negative is forward and positive is backwards
#define NORMAL_SPEED      -65
#define TURN_SPEED        -50
#define SLOW_SPEED        -25
int speed = NORMAL_SPEED;

// Uncomment for testing sensor readouts and judgements
#define SAFE_DISTANCE     160                    // distance to obstacle in centimeters returned from the sensor controller
#define TURN_DISTANCE     110                    // distance to obstacle that is less will result in turn state
#define FOLLOW_DISTANCE    70                    // distance to target when R2 is following
#define STOP_DISTANCE      55                    // distance to target with R2 stopping
#define MAX_DISTANCE      250                    // maximum distance to track with sensor once it returns from the controller

// Uncomment for LIVE use
//#define SAFE_DISTANCE     200                    // distance to obstacle in centimeters returned from the sensor controller
//#define TURN_DISTANCE     160                    // distance to obstacle that is less will result in turn state
//#define FOLLOW_DISTANCE   120                    // distance to target when R2 is following
//#define STOP_DISTANCE      90                    // distance to target with R2 stopping
//#define MAX_DISTANCE      300                    // maximum distance to track with sensor once it returns from the controller

MovingAverage<int, 3> sonarLeftFrontAverage(MAX_DISTANCE);       // moving average of last n pings for all 3 sensors
MovingAverage<int, 3> sonarRightFrontAverage(MAX_DISTANCE);      // initialize at MAX_DISTANCE
MovingAverage<int, 3> sonarLeftSideAverage(MAX_DISTANCE);
MovingAverage<int, 3> sonarRightSideAverage(MAX_DISTANCE);
MovingAverage<int, 3> sonarLeftBackAverage(MAX_DISTANCE);
MovingAverage<int, 3> sonarRightBackAverage(MAX_DISTANCE);


// =======================================================================================
//                            R2Sensor Variables
// =======================================================================================
int const automationInterval = 30000;       // After 30 seconds, make a decision or change decisions
unsigned long lastDecisionTime = 0;         // The last time - in millis() - that we made a decision (start)
bool makedecision = false, autoNavigation = false, Waypointenabled = false;

                                                                      // R2 Turns his dome towards the loudest sound detected!
#ifdef SOUNDDETECTION                                                 // Mic input variables
int mic1 = 0;                                                         // Microphone input on A0                               
int mic2 = 0;                                                         // Microphone input on A1
int mic3 = 0;                                                         // Microphone input on A2
int mic4 = 0;                                                         // Microphone input on A3

int threshold = 50;                                                   // Noise threshold before changing position
#endif

// =======================================================================================
//                            R2Sensor LCD Display
// =======================================================================================

#ifdef USE_LCD
LiquidCrystal_I2C lcd(0x3F, 20, 4);   // Set the LCD I2C address and size (4x20)
  #define LEFT_ARROW 0x7F
  #define RIGHT_ARROW 0x7E
  #define DEGREE_SYMBOL 0xDF
#endif
                                      // char lcd_buffer[20];
                                      // PString message(lcd_buffer, sizeof(lcd_buffer));    
                                      // holds message we display on line 4 of LCD
#endif

// =======================================================================================
//              R2Sensor  This function determines I2C addressing
// =======================================================================================
/*
 * We use a adafruit multiplexor to allow the same Compass system to be used on i2c
 * Vin is connected to 5V (on a 3V logic Arduino/microcontroller, use 3.3V)
    GND to ground
    SCL to I2C clock
    SDA to I2C data

Then wire up each of the other sensor breakouts to Vin, Ground and use one of the SCn / SDn multiplexed buses: */

#define I2C_ADDRESS_LEFT    0x2       // 0x1 = SHADOW, 0x2 = left, 0x3 = right, 0x4 = dome, 0x5 = center, 0x6 = voice, 0x7 = body
#define I2C_ADDRESS_RIGHT   0x3       // 0x1 = SHADOW, 0x2 = left, 0x3 = right, 0x4 = dome, 0x5 = center, 0x6 = voice, 0x7 = body
#define I2C_ADDRESS_DOME    0x4       // 0x1 = SHADOW, 0x2 = left, 0x3 = right, 0x4 = dome, 0x5 = center, 0x6 = voice, 0x7 = body
#define I2C_ADDRESS_CENTER  0x5       // 0x1 = SHADOW, 0x2 = left, 0x3 = right, 0x4 = dome, 0x5 = center, 0x6 = voice, 0x7 = body
#define I2C_ADDRESS_VOICE   0x6       // 0x1 = SHADOW, 0x2 = left, 0x3 = right, 0x4 = dome, 0x5 = center, 0x6 = voice, 0x7 = body
#define I2C_ADDRESS_BODY    0x7       // 0x1 = SHADOW, 0x2 = left, 0x3 = right, 0x4 = dome, 0x5 = center, 0x6 = voice, 0x7 = body
#define I2C_ADDRESS_SHADOW  0x1       // 0x1 = SHADOW, 0x2 = left, 0x3 = right, 0x4 = dome, 0x5 = center, 0x6 = voice, 0x7 = body
#define I2C_ADDRESS_TCAADDR 0x70      // Defined address of the adafruit multiplexer

// =======================================================================================
//              R2Sensor This function determines COMPASS functions for distance
// =======================================================================================

#ifdef COMPASS
int headingError;                     // signed (+/-) difference between GPS and body compass 
int currentHeading;                   // where we are actually facing now
int targetHeading;                    // where we want to go to reach current waypoint
float viewError;                      // signed (+/-) difference between dome and body compass
float  domenav, bodynav;              // sssigned variable for body and dome Directions
#endif

enum { getDirection };              // Case statement for choices of incoming on i2c
const char serialstart = '<';       // Delimiter for first incoming byte otherwise we do not listen.
const char serialfinish = '>';      // Delimiter for last incoming byte otherwise we continue reading
int whichNumber = getDirection;     // The Compass Direction converted back to float

// =======================================================================================
//              R2Sensor Bluetooth Support for Debuging through Android or Mobile device
// =======================================================================================

#ifdef BTSupport
SoftwareSerial BTSerial(10, 11);        // digital pins 10 & 11 were used BT Serial Connection
#endif


// =======================================================================================
//                            Dome Automation Variables
// =======================================================================================

boolean domeAutomation = false;
int dometurnDirection = 1;              // 1 = positive turn, -1 negative turn
float domeTargetPosition = 0;           // (0 - 359) - degrees in a circle, 0 = home
unsigned long domeStopTurnTime = 0;     // millis() when next turn should stop
unsigned long domeStartTurnTime = 0;    // millis() when next turn should start
int domeStatus = 0;                     // 0 = stopped, 1 = prepare to turn, 2 = turning
byte action = 0;
unsigned long DriveMillis = 0;
unsigned long DomelastDecisionTime = 0; // millis() when last recenter occured


// =======================================================================================
//                          Main Program
// =======================================================================================




// =======================================================================================
//                          Initialize -  GPS:Navigation Targeting
// =======================================================================================
#ifdef USE_GPS
#define GPSECHO false               // set to TRUE for GPS debugging if needed
//#define GPSECHO true              // set to TRUE for GPS debugging if needed
SoftwareSerial GPSSerial(6, 7);    // digital pins 7 & 8

Adafruit_GPS GPS(&GPSSerial);
boolean usingInterrupt = false;
float currentLat,
      currentLong,
      targetLat,
      targetLong;
int distanceToTarget,               // current distance to target (current waypoint)
    originalDistanceToTarget;       // distance to original waypoint when we started navigating to it

// GPS and COMPASS Navigation
#define HEADING_TOLERANCE 5           // tolerance +/- (in degrees) within which we don't attempt to turn to intercept targetHeading with R2 Dome or R2 Body

// Waypoints
#define WAYPOINT_DIST_TOLERANCE  5    // tolerance in meters to waypoint; once within this tolerance, will advance to the next waypoint
#define NUMBER_WAYPOINTS 5            // enter the numebr of way points here (will run from 0 to (n-1))
int waypointNumber = -1;              // current waypoint number; will run from 0 to (NUMBER_WAYPOINTS -1); start at -1 and gets initialized during setup()
waypointClass waypointList[NUMBER_WAYPOINTS] =
{ waypointClass(30.508302, -97.832624),
  waypointClass(30.508085, -97.832494),
  waypointClass(30.507715, -97.832357),
  waypointClass(30.508422, -97.832760),
  waypointClass(30.508518, -97.832665)
};

SIGNAL(TIMER0_COMPA_vect)             // Interrupt is called once a millisecond, looks for any new GPS data, and stores it
  {
    GPS.read();
  }
#endif

// =======================================================================================
//                          Initialize - i2c multiplexer
// =======================================================================================

#ifndef COMPASS_HMC5883L
void tcaselect(uint8_t i) {         // Used to query and initialize the i2c multiplexer board (provide an ID).
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}
#endif

// =======================================================================================
//                          Initialize - Setup Function
// =======================================================================================

void setup()  
{
  Serial.begin(115200);                                           //Debug Serial for use with USB Debugging                                   // Setup Watchdog for 3sec delays
  #ifdef BTSupport
    BTSerial.begin(9600);                                           // Enable Serial comm from BT Connection
    BTSerial.print("AT+BAUD4");                                     // Set baudrate to 9600
    BTSerial.print("AT+NAMER2-MCU-HC06");                           // Set the name to R2-MCU-HC06
  #endif

  #ifdef COLLISION
    Wire.begin(I2C_ADDRESS_SHADOW);                                 // assign i2c addressing for SHADOW controller
    Wire.onReceive(receiveI2C);                                     // if data is sent over i2c recieve it using this subroutine.
    pinMode(20, INPUT_PULLUP);                                      // Enable internal pull-up resistor on pin 20
    pinMode(21, INPUT_PULLUP);                                      // Enable internal pull-up resistor on pin 21
  #endif

// =============================== BODY COMPASS INITIALIZE ==============================================

  #ifdef COMPASS
  #ifndef COMPASS_HMC5883L
    tcaselect(1);                                                   // Initialize the first compass i2c device
    magbody.enableAutoRange(true);                                  // Enable auto-gain on the body compass
    if (!magbody.begin())                                           // Initialize the sensor
    {
      // There was a problem detecting the LSM303 ... check your connections
      Serial.println("BODY: Ooops, no LSM303 detected ... Check your wiring!");
      //while(1);
    }
  #else
    compass.enableAutoRange(true);                                  // Enable auto-gain on the body compass
    if (!compass.begin())                                           // Initialize the sensor
    {
      // There was a problem detecting the HMC5883L ... check your connections
      Serial.println("BODY: Ooops, no HMC5883L detected ... Check your wiring!");
      //while(1);
    }
  #endif
  #endif

// =============================== DOME COMPASS INITIALIZE ==============================================

  #ifdef COMPASS
  #ifndef COMPASS_HMC5883L
    tcaselect(0);                                                   // Initialize the second compass i2c device
  #endif
    magdome.enableAutoRange(true);                                  // Enable auto-gain on the dome compass
    if (!magdome.begin())                                           // Initialize the sensor
    {
      // There was a problem detecting the LSM303 ... check your connections
      Serial.println("DOME: Ooops, no LSM303 detected ... Check your wiring!");
      R2domeStatus = Error;                                         // Set the state for Dome to Error
      //while(1);
    }

  #endif


  while (!Serial);
  if (Usb.Init() == -1) 
  {
    Serial.print(F("\r\n (Bluetooth)OSC did not start"));     // Error check USB dongle for exists/working
    while (1); //halt
  }
  Serial.print(F("\r\nBluetooth Library Started"));

  //Setup for PS3
  PS3NavFoot->attachOnInit(onInitPS3NavFoot);                 // onInitPS3NavFoot is called upon a new connection
  PS3NavDome->attachOnInit(onInitPS3NavDome);

  Serial1.begin(9600);                                        // Setup for Serial1:: MarcDuino Dome Control Board
  Serial2.begin(9600);                                        // Setup for Serial2:: Motor Controllers - Sabertooth (Feet)
  Serial3.begin(9600);                                        // Setup for Serial3:: Optional AdaFruit GPS Module (waypoint navigation)
                                                              // Setup for Serial3:: MarcDuino body Control Board
  ST->autobaud();                                             // Send the autobaud command to the Sabertooth controller(s).
  ST->setTimeout(10);                                         // DMB:  How low can we go for safety reasons?  multiples of 100ms
  ST->setDeadband(driveDeadBandRange);
  ST->stop();
  SyR->autobaud();
  SyR->setTimeout(20);                                        // DMB:  How low can we go for safety reasons?  multiples of 100ms
  SyR->stop();

  randomSeed(analogRead(0));                                  // random number seed for dome automation

}

// =======================================================================================
//           Main Program Loop - This is the recurring check loop for entire sketch
// =======================================================================================

void loop() {
#ifdef TEST_CONROLLER
  testPS3Controller();                          // Useful to enable with serial console when having controller issues.
#endif

  if ( !readUSB() ) {                           // LOOP through functions from highest to lowest priority.
    //printOutput();                            // We have a fault condition that we want to ensure that we do NOT process any controller data!!!
    return;
  }
  footMotorDrive();                             // Check to see if there is PS3 Controller input (Manual Control)
  domeDrive();                                  // Check to see if there is PS3 Controller input (Manual Control)
  marcDuinoDome();                              // Check for input to control servos and sound in the Dome Controller
  marcDuinoFoot();                              // Check for input to control servos and sound in the Foot Controller
  toggleSettings();                             // Enable / Disable settings through controller combinations

#ifdef COLLISION                                // If we have the R2Sensor Controller installed and auto navigation is enabled
  if (autoNavigation)
  {
    #ifdef VOICE
      voicecontrol();                           // Check for input from the voice sensor controller (EasyVR over i2c)
    #endif
    
    R2Decision();                               // Randomly perform an action and change it every 30secs
    R2States();                                 // If autoNavigation is true, begin moving R2
    
    #ifdef BTSupport
      BTsendandReceive();                       // Support for Bluetooth Dongle connections to Mobile device or tablet
    #endif
  }
#endif
  
  if (runningCustRoutine)                       // If running a custom MarcDuino Panel Routine - Call Function
  {
    custMarcDuinoPanel();
  }
  // If dome automation is enabled - Call function
  if (domeAutomation && time360DomeTurn > 1999 && time360DomeTurn < 8001 && domeAutoSpeed > 49 && domeAutoSpeed < 101)
  {
    autoDome();
  }
}

// =======================================================================================
//                             R2 Sensor Automation Function
//
//    Features toggles 'on' via L2 + CROSS.  'off' via L2 + CROSS.  Default is 'off'.
//    Update on 9/21/2015 - enabled it to be toggled using L2 + CIRCLE only...
//    This routines randomly moves R2 forward, spins or other items.  It assumes the
//    R2 unit is in the stopped position when the auto feature is toggled on.  From
//    there it turns the R2 in a random direction and moves forward, stays still and
//    repeats until disabled.  Random control logic: Stay still, Spin in place, Move
//
// =======================================================================================

// =======================================================================================
//           R2Sensor Status Checks and verification
// =======================================================================================
#ifdef COLLISION

void R2States()
{
// =======================================================================================
//                                   states 
// =======================================================================================
  if (R2state == MovingFwd)
  {
    R2Sensors();  // Calculate Sensor readings from Ping Sensors Front left and right
  }
  else if (R2state == MovingBck)
  {
    R2Sensors();  // Calculate Sensor readings from Ping Sensors Front left and right
  }
  else if (R2state == Turning)
  {
    R2Sensors();  // Calculate Sensor readings from Ping Sensors Front left and right
  }
  else if (R2state == Stopped)
  {
    
  }
// =======================================================================================
//                                   Modes 
// =======================================================================================
  if (R2mode == Patroling)
  {
    autoNavigation = true;                      // Engage Autonavigation bypassing throttle controls from joysticks
    R2Patrol();
  }
  else if (R2mode == Waypoint)
  {
    autoNavigation = true;                      // Engage Autonavigation bypassing throttle controls from joysticks
    R2Compass();                                // Get Compass readings from both dome and body
    #ifdef USE_GPS                              // Process GPS module if enabled
    if (GPS.newNMEAreceived())                  // check for updated GPS information
    {
      if (GPS.parse(GPS.lastNMEA()) )           // if we successfully parse it, update our data fields
      processGPS();
    }
    R2Sensors();                                // Calculate Sensor readings from Ping Sensors Front left and right
    calcDesiredTurn();                          // calculate how we would optimatally turn GPS or due to objects
    R2Waypointnav();                            // Move R2 Forward towards the waypoints watching for obstructions
    #endif
  }
  else if (R2mode == Aware)
  {
    #ifdef SOUNDDETECTION
    R2soundcheck();             // Using the microphones check for sounds nearing him
    #endif
    followDome(body);           // pirsensor = 6 on I2C, feeds value to body variable from Human detection (PIR)
  }
  else if (R2mode == Sleep)
  {
    
  }
  else if (R2mode == Playfull)
  {
    autoNavigation = true;      // Engage Autonavigation bypassing throttle controls from joysticks
    R2Sensors();                // Calculate Sensor readings from Ping Sensors Front left and right
  }
  else if (R2mode == Following)
  {
    autoNavigation = true;      // Engage Autonavigation bypassing throttle controls from joysticks
    R2Sensors();                // Calculate Sensor readings from Ping Sensors Front left and right
    R2Follow();                 // Move R2 Forward towards human watching for obstructions
  }
  else if (R2mode == Guarding)
  {
    autoNavigation = true;      // Engage Autonavigation bypassing throttle controls from joysticks
    R2Sensors();                // Calculate Sensor readings from Ping Sensors Front left and right
    #ifdef SOUNDDETECTION
    R2soundcheck();             // Using the microphones check for sounds nearing him'
    #endif
  }
  else if (R2mode == Searching)
  {
    autoNavigation = true;      // Engage Autonavigation bypassing throttle controls from joysticks
  }
  else if (R2mode == Scanning)
  {
    autoNavigation = true;      // Engage Autonavigation bypassing throttle controls from joysticks
  }
// ============================== Dome Status ==============================================

  if (R2domeStatus == Home)             // R2 Dome is in home position
  {
    
  }
  else if (R2domeStatus == Turn)        // R2 Dome is turning
  {
    
  }
  else if (R2domeStatus == Ready)       // R2 Dome is ready to turn
  {
    
  }
  else if (R2domeStatus == Error)       // R2 Dome doesnt match body direction
  {
    
  }
  return;
}
// =======================================================================================
//           R2Sensor Calculate Turning from obstacles calcDesiredTurn()
// =======================================================================================

void calcDesiredTurn()    // calculate where we need to turn to head to destination
{ 
  #ifdef USE_GPS
  headingError = targetHeading - bodynav;
  if (headingError < -180)                        // adjust for compass wrap
    headingError += 360;
  if (headingError > 180)
    headingError -= 360;
  // calculate which way to turn to intercept the targetHeading
  if (abs(headingError) <= HEADING_TOLERANCE)     // if within tolerance, don't turn
    turnDirection = straight;
  else if (headingError < 0)
    turnDirection = left;
  else if (headingError > 0)
    turnDirection = right;
  else
    turnDirection = straight;
    if (isDebug)
    {
      Serial.print("GPS TargetHeading: ");
      Serial.println(targetHeading);
      Serial.print("GPS HeadingError: ");
      Serial.println(headingError);
      Serial.print("GPS turnDirection: ");
      Serial.println(turnDirection);
    }
  return;
  #endif
}
#endif


// =======================================================================================
//           R2Sensor Compass Control Section
// =======================================================================================
// This is the desired direction of travel
// expressed as a 0-360 degree compass heading
// 0.0 = North
// 90.0 = East
// 180.0 = South
// 270 = West

void R2Compass()
{
// =============================== BODY COMPASS Get Readings ==============================================

  #ifdef COMPASS
  #ifndef COMPASS_HMC5883L
  tcaselect(1);
  if (magbody.begin())  
  {
    bodynav = readBodyCompass();  // Get our current heading from Body COMPASS portion of the code
    if (isDebug)  
    {
      Serial.print("Bodynav: ");
      Serial.println(bodynav);
    }
  }
  else
  {
    if (isDebug)  
      Serial.println("Body Compass not found ");
    R2domeStatus = Error;
  }
  #else
    if (compass.begin())  
  {
    bodynav = readBodyCompass();  // Get our current heading from Body COMPASS portion of the code
    if (isDebug)  
    {
      Serial.print("Bodynav: ");
      Serial.println(bodynav);
    }
  }
  else
  {
    if (isDebug)  
      Serial.println("Body Compass not found ");
    R2domeStatus = Error;
  }
  #endif
  
// =============================== DOME COMPASS Get Readings ==============================================
#ifndef COMPASS_HMC5883L
  tcaselect(0);
  if (magdome.begin())  
  {
    domenav = readDomeCompass();  // Get our current heading from Dome COMPASS portion of the code
    if (isDebug)  
    {
      Serial.print("Domenav: ");
      Serial.println(domenav);
    }
    R2domeStatus = Ready;
  } 
#endif
#ifdef COMPASS_HMC5883L
  if (compass.begin())  
  {
    domenav = readDomeCompass();  // Get our current heading from Dome COMPASS portion of the code
    if (isDebug)  
    {
      Serial.print("Domenav: ");
      Serial.println(domenav);
    }
    R2domeStatus = Ready;
  } 
#endif
  else
  {
  if (isDebug)  
    Serial.println("Dome Compass not found ");
  R2domeStatus = Error; // Set the state for Dome to Error
  }
  
  if (R2domeStatus == Ready)
    viewError = domenav - bodynav;  // calculate which way to turn the dome to intercept the difference (if compass is ready)

#endif
}
// =============================== BODY COMPASS ===============================================

#ifdef COMPASS
float readBodyCompass() 
{
  #ifndef COMPASS_HMC5883L
  tcaselect(1);
  magbody.getEvent(&magbody_event);
  float heading = atan2(magbody_event.magnetic.y, magbody_event.magnetic.x);
  #else
  compass.getEvent(&compass_event);

  // Calculate the angle of the vector y,x
  // float heading = (atan2(event.magnetic.y,event.magnetic.x) * 180) / Pi;
  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Cedar Park, TX: Magnetic declination: 4Â° 11' EAST (POSITIVE);  1 degreee = 0.0174532925 radians
  // #define DEC_ANGLE 0.069
  // heading += DEC_ANGLE;
  // int heading = atan2(event.magnetic.y, event.magnetic.x);

  float heading = atan2(compass_event.magnetic.y, compass_event.magnetic.x);
  #endif
  #define DEC_ANGLE 0.2094395102
  heading -= DEC_ANGLE;
  if (isDebug)
  Serial.println("DEC_ANGLE");
  
  if (heading < 0)  // Correct for when signs are reversed.
    heading += 2 * PI;

  
  if (heading > 2 * PI) // Check for wrap due to addition of declination.
    heading -= 2 * PI;

  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180 / M_PI;
  if (isDebug) 
  {
    Serial.print("Body Compass Heading...");
    Serial.println(headingDegrees);
  }
  return ((float)headingDegrees);
}
#endif

// =============================== DOME COMPASS ===============================================

#ifdef COMPASS
float readDomeCompass() 
{
  #ifndef COMPASS_HMC5883L
  tcaselect(0);             // Connecting direct to the i2c channel instead of the multiplexer
  #endif
  magdome.getEvent(&magdome_event);
  
  // Calculate the angle of the vector y,x
  // float heading = (atan2(event.magnetic.y,event.magnetic.x) * 180) / Pi;
  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Cedar Park, TX: Magnetic declination: 4Â° 11' EAST (POSITIVE);  1 degreee = 0.0174532925 radians
  // #define DEC_ANGLE 0.069
  // heading += DEC_ANGLE;
  // int heading = atan2(event.magnetic.y, event.magnetic.x);

  float heading = atan2(magdome_event.magnetic.y, magdome_event.magnetic.x);
  #define DEC_ANGLE 0.2094395102
  heading -= DEC_ANGLE;
  if (isDebug)
  Serial.println("DEC_ANGLE");
  
  if (heading < 0)  // Correct for when signs are reversed.
    heading += 2 * PI;

  
  if (heading > 2 * PI) // Check for wrap due to addition of declination.
    heading -= 2 * PI;

  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180 / M_PI;
  if (isDebug) 
  {
    Serial.print("Body Compass Heading...");
    Serial.println(headingDegrees);
  }
  return ((float)headingDegrees);
}
#endif

// =======================================================================================
//           R2Sensor GPS Section
// =======================================================================================
// Called after new GPS data is received; updates our position and course/distance to waypoint
#ifdef USE_GPS
void processGPS(void) {
  currentLat = convertDegMinToDecDeg(GPS.latitude);
  currentLong = convertDegMinToDecDeg(GPS.longitude);

  if (GPS.lat == 'S')            // make them signed
    currentLat = -currentLat;
  if (GPS.lon = 'W')
    currentLong = -currentLong;

  // update the course and distance to waypoint based on our new position
  distanceToWaypoint();
  courseToWaypoint();
}
#endif

// converts lat/long from Adafruit degree-minute format to decimal-degrees
#ifdef USE_GPS
double convertDegMinToDecDeg (float degMin) 
{
  double min = 0.0;
  double decDeg = 0.0;

  //get the minutes, fmod() requires double
  min = fmod((double)degMin, 100.0);

  //rebuild coordinates in decimal degrees
  degMin = (int) ( degMin / 100 );
  decDeg = degMin + ( min / 60 );
  if (isDebug)  
    {
    Serial.print("GPS: CompassDegress: ");
    Serial.println(decDeg);
    }
  return decDeg;
}
#endif

// =======================================================================================
//           R2Sensor Compass Waypoint Support
// =======================================================================================

#ifdef USE_GPS
void nextWaypoint(void) 
{
  waypointNumber++;
  targetLat  = waypointList[waypointNumber].getLat();
  targetLong = waypointList[waypointNumber].getLong();
  
  if ((targetLat == 0 && targetLong == 0) || waypointNumber >= NUMBER_WAYPOINTS)    // last waypoint reached?
  { 
    while (footDriveSpeed > 0)                                                      // Bring R2 to Stop (ramping)
    {                                                    
      footDriveSpeed = footDriveSpeed -= ramping;
      ST->turn(turnDirection * invertturnDirection);
      ST->drive(footDriveSpeed);
    }
    #ifdef USE_LCD
      lcd.clear();
      lcd.println(F("* LAST WAYPOINT *"));
      WayPointDecision();
    #endif
  }
  processGPS();
  distanceToTarget = originalDistanceToTarget = distanceToWaypoint();
  courseToWaypoint();
}

// returns course in degrees (North=0, West=270) from position 1 to position 2,
// both specified as signed decimal-degrees latitude and longitude.
// Because Earth is no exact sphere, calculated course may be off by a tiny fraction.
// copied from TinyGPS library

int courseToWaypoint()
{
  float dlon = radians(targetLong - currentLong);
  float cLat = radians(currentLat);
  float tLat = radians(targetLat);
  float a1 = sin(dlon) * cos(tLat);
  float a2 = sin(cLat) * cos(tLat) * cos(dlon);
  a2 = cos(cLat) * sin(tLat) - a2;
  a2 = atan2(a1, a2);
  if (a2 < 0.0) 
  {
    a2 += TWO_PI;
  }
  targetHeading = degrees(a2);
  return targetHeading;
}

// returns distance in meters between two positions, both specified
// as signed decimal-degrees latitude and longitude. Uses great-circle
// distance computation for hypothetical sphere of radius 6372795 meters.
// Because Earth is no exact sphere, rounding errors may be up to 0.5%.
// copied from TinyGPS library

int distanceToWaypoint() 
{
  float delta = radians(currentLong - targetLong);
  float sdlong = sin(delta);
  float cdlong = cos(delta);
  float lat1 = radians(currentLat);
  float lat2 = radians(targetLat);
  float slat1 = sin(lat1);
  float clat1 = cos(lat1);
  float slat2 = sin(lat2);
  float clat2 = cos(lat2);
  delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
  delta = sq(delta);
  delta += sq(clat2 * sdlong);
  delta = sqrt(delta);
  float denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
  delta = atan2(delta, denom);
  distanceToTarget =  delta * 6372795;

  // check to see if we have reached the current waypoint
  if (distanceToTarget <= WAYPOINT_DIST_TOLERANCE)
    nextWaypoint();
  return distanceToTarget;
}
#endif

// =======================================================================================
//           R2Sensor WayPointDecision
// =======================================================================================

void WayPointDecision(void)                           // end of program routine, loops forever
{
  // while (1);
}


// =======================================================================================
//           R2Sensor i2c Recieve checkins
// =======================================================================================
// distance data from 2 feet and nav from dome
// left = 1, right =2, center = 3, voice = 4, pirsensor = 6,
// (Dome and Body Compass moved to i2c MultiPlexer)

void receiveI2C(int howMany) 
{
  if (Wire.available() > 0) 
  {
    partnum = Wire.read();
    front   = Wire.read();
    side    = Wire.read();
//    side1   = Wire.read();
//    side2   = Wire.read();
    back    = Wire.read();

    if (partnum == 1) 
    {         // This is the left foot sensor checking in
      leftfront =  front;
      leftside  =  side;
//      leftside1  =  side1;
//      leftside2  =  side2;
      leftback  =  back;
    }
    if (partnum == 2) 
    {         // This is the right foot sensor checking in
      rightfront = front;
      rightside  = side;
//      rightside1  =  side1;
//      rightside2  =  side2;
      rightback  = back;
    }
    if (partnum == 3) 
    {         // This is the center foot sensor checking in
      centerfront = front;
      centerleft  = side;
      centerright = back;
    }
    if (partnum == 4) 
    {         // This is the voice sensor using EasyVR checking in
      voicecmd    = front;
    }
    if (partnum == 6) 
    {         // This is the body sensor using PIR sensors checking in
      body        = front;
    }

    if (isDebug)  
    {
      Serial.print (partnum);
      if (partnum == 1 || partnum == 2) 
      {     // If partnum reported 1 or 2 then its the outer feet checking in
        Serial.print (" front ");
        Serial.print (front);
        Serial.print (" side ");
        Serial.print (side);
        Serial.print (" back ");
        Serial.print(back);
      }
      else if (partnum == 3)  
      {     // if the partnum reported 3 then its the front center foot
        Serial.print (" front ");
        Serial.print (front);
        Serial.print (" left ");
        Serial.print (side);
        Serial.print (" right ");
        Serial.print(back);
      }
      else if (partnum == 4)  
      {     // if the partnum is 4 then its the voice module on a arduino UNO/Duomilanove
        Serial.print (" VoiceCommand: ");
        Serial.print (voicecmd);
      }
      else if (partnum == 6)  
      {     // if the partnum is 6 then its the body module within the body connecting PIRs
        Serial.print (" PIR: ");
        Serial.print (body);
      }
      Serial.println ("");
    }
  }
}

// =======================================================================================
//           R2Sensor i2c Send Data
// =======================================================================================

void SendSensorStatus() {
  //    Wire.beginTransmission(I2C_ADDRESS_SHADOW);  //send to master SHADOW 0x1
  //    if (Part!="dome") {
  //      Wire.write(partnum);
  //      Wire.write(front);
  //      Wire.write(side);
  //      Wire.write(back);
  //      if (Part=="dome") {
  //        Wire.write(nav);
  //      }
  //    }
  //    Wire.endTransmission();
  //    counter++;
  //    delay(25);
}

// =======================================================================================
//           R2Sensor Decision(s) |  Randomization
// =======================================================================================

void R2Decision() 
{
  if (isDebug)
  {
    Serial.print("Current time: ");
    Serial.print(currentTime);
    Serial.print(" || Last Decision time: ");
    Serial.println(lastDecisionTime);
  }

  #ifndef COMPASS_DEBUG
  #ifndef VOICE_DEBUG
  #ifndef PATROL_DEBUG
  if ((currentTime - lastDecisionTime) >= automationInterval) 
  {
    lastDecisionTime = millis();
    choice = random(3);
    if (choice == 0)
    {
      R2state = MovingFwd;
      R2mode = Patroling;
    }
    else if (choice == 1)
    {
      R2state = Stopped;
      R2mode = Aware;
    }
    else if (choice == 2)
    {
      R2state = Stopped;
      R2mode = Sleep;
    }
    else if (choice == 3)
    {
      R2state = Listen;
      R2mode = Scanning;
    }
    if (isDebug)
    {
      Serial.print("MadeDecision: ");
      Serial.println(choice);
    }
    return;
  } else
  #else
      R2state = MovingFwd;
      R2mode = Patroling;
  #endif
  #endif
  #endif
  #ifdef VOICE_DEBUG
    R2state = Listen;
    R2mode = Scanning;
  #endif
  #ifdef COMPASS_DEBUG
    readDomeCompass();
    readBodyCompass();
  #endif
  return;
}

// =======================================================================================
//           R2Sensor Decision(s) |  TurnChoice
// =======================================================================================

void R2TurnChoice ()  
{
  int turnchoice = random(2);
  if (turnchoice == 0) {
    turnDirection = right;
    ST->turn(turnDirection * invertturnDirection);      // Turn R2 Right Full
  } else if (turnchoice == 1)  {
    turnDirection = left;
    ST->turn(turnDirection * invertturnDirection);      // Turn R2 Left Full
  } else
    turnDirection = straight;
  ST->turn(turnDirection * invertturnDirection);      // Turn R2 Left Full
}


void cautiousR2(void)  
{
  int domeSpeed, ObjectIsClose = 65;
  if ((distanceleftfront < ObjectIsClose) || (distancerightfront < ObjectIsClose)) 
  {
    speed = speed * -1;
      
    if ((distanceleftside < ObjectIsClose) || (distancerightside < ObjectIsClose))  
    {
      if ((distanceleftback > TURN_DISTANCE) && (distancerightback > TURN_DISTANCE)) 
      {
        turnDirection = straight;
      }
      else if (distanceleftback > distancerightback) 
      {
        turnDirection = right;
      }
      else if (distancerightback > distanceleftback) 
      {
        turnDirection = left;
      }
    }
    else if ((distanceleftside > ObjectIsClose) && (distancerightside < ObjectIsClose))
      turnDirection = right;
      else
      turnDirection = left;
  }
// =======================================================================================
//          Ramping code for speed variations
// =======================================================================================
  if (footDriveSpeed <= speed)  
    footDriveSpeed = footDriveSpeed += ramping;
  else 
    footDriveSpeed = footDriveSpeed -= ramping;
  ST->drive(footDriveSpeed);
  ST->turn(turnDirection * invertturnDirection);
  if (isDebug)
    {
      Serial.print("Moving Backwards:");
      Serial.println(speed);
    }
}



// =======================================================================================
//           R2Sensor: Patrol
// =======================================================================================

#ifdef COLLISION
void R2Patrol() 
{
  if (distance >= SAFE_DISTANCE)
  {
    speed = FAST_SPEED;
    turnDirection = straight;
    if (isDebug)
      Serial.println("SAFE SPEED:FAST");
    return;
  }
  else if (distance < SAFE_DISTANCE && distance > TURN_DISTANCE)
  {
    speed = NORMAL_SPEED;
    turnDirection = straight;
    if (isDebug)
    {
      Serial.println("SPEED:NORMAL");
    }
  }
  else if (distance <= TURN_DISTANCE && distance > STOP_DISTANCE)
  {
    speed = TURN_SPEED; // slow down to turn, prepare for turn check sensors
    if (distancerightfront > distanceleftfront)
    {
      turnDirection = right;  // turn right because it has more room
      if (isDebug)
      {
        Serial.println("TURN: right");
      }
    }
    else
    {
      turnDirection = left;  // turn left because it has more room
      if (isDebug)
      {
        Serial.println("TURN: left");
      }
    }
  }
  else if (distance <= STOP_DISTANCE)
  {
    speed = 0;
    turnDirection = straight;
    if (isDebug)
    {
      Serial.println("STOP: 0");
    }
  }
}
#endif

// =======================================================================================
//           R2Sensor: Follow
// =======================================================================================
// pirPos[] = {-270,-90,0,90,270};   
// positions for dome or body (Moving the body or the dome based on current COMPASS finding (dome or body).
#ifdef COLLISION
void R2Follow() {
  int FollowDirection;
  if (distance > FOLLOW_DISTANCE)
  {
    speed = ++speed;  // speed up
    if (isDebug)
      Serial.println("SPEED:SPEED UP");
    return;
  }
  else if (distance < FOLLOW_DISTANCE)
  {
    speed = --speed;
    // slow down
    if (isDebug)
      Serial.println("SPEED:SLOW DOWN");
    return;
  }
  else
  {
    speed = speed;
  }
////////if (distancerightfront > distanceleftfront)
////////    {
//////////      turnDirection = left;  // turn left because more likely human that direction
//////////      if (isDebug)
//////////        Serial.println("TURN: left");
////////    }/////////  else if (distancerightfront < distanceleftfront || pirPos[0] == 1 || pirPos[1] == 1)
//////////    {
//////////      turnDirection = right;  // turn right because more likely human that direction
//////////      if (isDebug)
//////////        Serial.println("TURN: left");
//////////    }
//////////    else
//////////      turnDirection = straight;
}
#endif

// =======================================================================================
//           R2Sensor: Waypoint navigation using GPS
// =======================================================================================

#ifdef COLLISION
void R2Waypointnav() 
{
  if (isDebug)
  {
    Serial.print("State: ");
    Serial.print(R2state); 
    Serial.print(" LeftFront: ");
    Serial.print(leftfront); 
    Serial.print(" RightFront: ");
    Serial.print(rightfront); 
    Serial.print(" distance: ");
    Serial.println(distance); 
  }  
  if (distance >= SAFE_DISTANCE)
  {
    speed = FAST_SPEED;
    turnDirection = straight;
    if (isDebug)
    {
      Serial.println("SAFE SPEED:FAST");
    }
    return;
  }
  else if (distance < SAFE_DISTANCE && distance > TURN_DISTANCE)
  {
    speed = NORMAL_SPEED;
    turnDirection = straight;
    if (isDebug)
    {
      Serial.println("SPEED:NORMAL");
    }
    #ifdef GPS_ENABLE
    if (R2mode == Waypoint)
    {
      if (headingError < 0)
        turnDirection = left;
      else
        turnDirection = right;
      if (isDebug)
      {
        Serial.print("GPS turnDirection: ");
        Serial.println(turnDirection);
      }
    }
    #endif
  }
  else if (distance <= TURN_DISTANCE && distance > STOP_DISTANCE)
  {
    speed = TURN_SPEED; // slow down to turn, prepare for turn check sensors
    if (distancerightfront > distanceleftfront)
    {
      turnDirection = right;  // turn right because it has more room
      if (isDebug)
      {
        Serial.println("TURN: right");
      }
    }
    else
    {
      turnDirection = left;  // turn left because it has more room
      if (isDebug)
      {
        Serial.println("TURN: left");
      }
    }
  }
  else if (distance <= STOP_DISTANCE)
  {
    speed = 0;
    turnDirection = straight;
    if (isDebug)
    {
      Serial.println("STOP: 0");
    }
  }
}
#endif

// =======================================================================================
//           R2Sensor math or work against the findings during scanning
// =======================================================================================

#ifdef COLLISION
void R2Sensors() 
{
  distanceleftfront = sonarLeftFrontAverage.add(leftfront);             // add the new value into moving average, use resulting average
  distancerightfront = sonarRightFrontAverage.add(rightfront);          // add the new value into moving average, use resulting average
  distanceleftside = sonarLeftSideAverage.add(leftside);                // add the new value into moving average, use resulting average
  distancerightside = sonarRightSideAverage.add(rightside);             // add the new value into moving average, use resulting average
  distanceleftback = sonarLeftBackAverage.add(leftback);                // add the new value into moving average, use resulting average
  distancerightback = sonarRightBackAverage.add(rightback);             // add the new value into moving average, use resulting average
  if (R2state == MovingFwd)  
  {
    if (distanceleftfront < TURN_DISTANCE) {
      if (distanceleftfront <= 0)
        distanceleftfront = 255;                                          // If the sensor returns 0 (clear) then set it as 255
      else
        distance = distanceleftfront;
    } else if (distancerightfront < TURN_DISTANCE) 
    {
      if (distancerightfront <= 0)
        distancerightfront = 255;                                         // If the sensor returns 0 (clear) then set it as 255
      else
        distance = distancerightfront;
    }
    else
      distance = (distanceleftfront + distancerightfront) / 2;          // Average the returned distance of combined left and right
  } 
  else if (R2state == MovingBck) 
  {
    if (distanceleftback < TURN_DISTANCE) {
      if (distanceleftback <= 0)
        distanceleftback = 255;                                           // If the sensor returns 0 (clear) then set it as 255
      else
        distance = distanceleftback;
    } else if (distancerightback < TURN_DISTANCE) {
      if (distancerightback <= 0)
        distancerightback = 255;                                          // If the sensor returns 0 (clear) then set it as 255
      else
        distance = distancerightback;
    }
    else
      distance = (distanceleftback + distancerightback) / 2;            //Average the returned distance of combined left and right
  } 
  else if (R2state == Turning) 
  {
    if (distanceleftfront < TURN_DISTANCE) {
      if (distanceleftfront <= 0)
        distanceleftfront = 255;                                          // If the sensor returns 0 (clear) then set it as 255
      else
        distance = distanceleftfront;
    } else if (distancerightfront < TURN_DISTANCE) {
      if (distancerightfront <= 0)
        distancerightfront = 255;                                         // If the sensor returns 0 (clear) then set it as 255
      else
        distance = distancerightfront;
    }
    else
      distance = (distanceleftfront + distancerightfront) / 2;          //Average the returned distance of combined left and right
  }
  else if (R2state == Stopped) 
  {
    if (distanceleftfront < TURN_DISTANCE) 
    {
      if (distanceleftfront <= 0)
        distanceleftfront = 255;                                          // If the sensor returns 0 (clear) then set it as 255
      else
        distance = distanceleftfront;
    } 
    else if (distancerightfront < TURN_DISTANCE) 
    {
      if (distancerightfront <= 0)
        distancerightfront = 255;                                         // If the sensor returns 0 (clear) then set it as 255
      else
        distance = distancerightfront;
    }
    else
      distance = (distanceleftfront + distancerightfront) / 2;          //Average the returned distance of combined left and right
  }
  return;
}
#endif

// =======================================================================================
//           R2Sensor obstacle decision making state
// =======================================================================================
#ifdef COLLISION
bool personAhead(unsigned int distance) 
{
  return (distance <= FOLLOW_DISTANCE);
}
#endif

//// =======================================================================================
////           R2Sensor GPSSerial Pull from Dome Sensor Pins (10,11)
//// =======================================================================================
//void processNumber (const long n)
//  {
//  float x = n/100.0;
//
//  switch (whichNumber)
//    {
//    case getDirection:
//      domenav = x;
//      whichNumber = getDirection;
//      #ifdef SHADOW_VERBOSE
//      Serial.print("Dome Compass Heading...");
//      #endif
//      break;
//    }
//    #ifdef SHADOW_VERBOSE
//    Serial.println(domenav);
//    #endif
//  }

//void processInput ()
//  {
//  static float receivedNumber = 0;
//  static boolean negative = false;
//
//  byte c = GPSSerial.read();
//
//  switch (c)
//    {
//
//    case serialfinish:
//      if (negative)
//        processNumber (- receivedNumber);
//      else
//        processNumber (receivedNumber);
//
//    // fall through to start a new number
//    case serialstart:
//      receivedNumber = 0;
//      negative = false;
//      break;
//
//    case '0' ... '9':
//      receivedNumber *= 10;
//      receivedNumber += c - '0';
//      break;
//
//    case '-':
//      negative = true;
//      break;
//
//    }
//  }

// =======================================================================================
//           footDrive Motor Control Section
// =======================================================================================

boolean ps3FootMotorDrive(PS3BT* myPS3 = PS3NavFoot)  
{
  int stickSpeed = 0;
  // int turnnum = 0;

  if (isPS3NavigatonInitialized)
  {
    // Additional fault control.  
    // Do NOT send additional commands to Sabertooth if no controllers have initialized.
    if (!isStickEnabled)  
    {
      if (!isFootMotorStopped)  
      {
        ST->stop();
        isFootMotorStopped = true;
        footDriveSpeed = 0;
      }
      return false;

    } 
    else if (!myPS3->PS3NavigationConnected)  
    {
      if (!isFootMotorStopped)  
      {
        ST->stop();
        isFootMotorStopped = true;
        footDriveSpeed = 0;
      }
      return false;
    } 
    else if (myPS3->getButtonPress(L2) || myPS3->getButtonPress(L1))  
    {
      if (!isFootMotorStopped)  
      {
        ST->stop();
        isFootMotorStopped = true;
        footDriveSpeed = 0;
      }
      return false;
    } 
    else
    {
        speed = 0;
        int joystickPosition = myPS3->getAnalogHat(LeftHatY);
        if (overSpeedSelected)  //Over throttle is selected
        {                                                   
          stickSpeed = (map(joystickPosition, 0, 255, -drivespeed2, drivespeed2));
        } 
        else 
        {
          stickSpeed = (map(joystickPosition, 0, 255, -drivespeed1, drivespeed1));
        }
        if ( abs(joystickPosition - 128) < joystickFootDeadZoneRange) // This is RAMP DOWN code when stick is now at ZERO but prior FootSpeed > 20
        {             
          if (abs(footDriveSpeed) > 50) {
            if (footDriveSpeed > 0) {
              footDriveSpeed -= 3;
            } else
            {
              footDriveSpeed += 3;
            }
            #ifdef SHADOW_VERBOSE
              Serial.print("ZERO FAST RAMP: footSpeed: ");
              Serial.print(footDriveSpeed);
              Serial.print("\nStick Speed: ");
              Serial.print(stickSpeed);
              Serial.println("");
            #endif

          } else if (abs(footDriveSpeed) > 20)
          {
            if (footDriveSpeed > 0)
            {
              footDriveSpeed -= 2;
            } else
            {
              footDriveSpeed += 2;
            }

            #ifdef SHADOW_VERBOSE
              Serial.print("ZERO MID RAMP: footSpeed: ");
              Serial.print(footDriveSpeed);
              Serial.print("\nStick Speed: ");
              Serial.print(stickSpeed);
              Serial.println("");
            #endif

          } 
          else
          {
            footDriveSpeed = 0;
          }

        } 
        else
        {
          isFootMotorStopped = false;
          if (footDriveSpeed < stickSpeed)
          {

            if ((stickSpeed - footDriveSpeed) > (ramping + 1) || (speed - footDriveSpeed) > (ramping + 1))
            {
              footDriveSpeed += ramping;
              #ifdef SHADOW_VERBOSE
                Serial.print("RAMPING DOWN: footSpeed: ");
                Serial.print(footDriveSpeed);
                Serial.print("\nStick Speed: ");
                Serial.print(stickSpeed);
                Serial.print("\ramping: ");
                Serial.print(ramping);
                Serial.println("");
              #endif
            } 
            else
              footDriveSpeed = stickSpeed;

          } 
          else if (footDriveSpeed > stickSpeed)
          {
            if ((footDriveSpeed - stickSpeed) > (ramping + 1) || (footDriveSpeed - speed) > (ramping + 1))
            {
              footDriveSpeed -= ramping;
                #ifdef SHADOW_VERBOSE
                Serial.print("RAMPING UP: footSpeed: ");
                Serial.print(footDriveSpeed);
                Serial.print("\nStick Speed: ");
                Serial.print(stickSpeed);
                Serial.print("\ramping: ");
                Serial.print(ramping);
                Serial.println("");
              #endif

            } 
            else
              footDriveSpeed = stickSpeed;
          } 
          else
            footDriveSpeed = stickSpeed;
        }

        turnnum = (myPS3->getAnalogHat(LeftHatX));

        //TODO:  Is there a better algorithm here?
        if ( abs(footDriveSpeed) > 50)
          turnnum = (map(myPS3->getAnalogHat(LeftHatX), 54, 200, -(turnspeed / 4), (turnspeed / 4)));
        else if (turnnum <= 200 && turnnum >= 54)
          turnnum = (map(myPS3->getAnalogHat(LeftHatX), 54, 200, -(turnspeed / 3), (turnspeed / 3)));
        else if (turnnum > 200)
          turnnum = (map(myPS3->getAnalogHat(LeftHatX), 201, 255, turnspeed / 3, turnspeed));
        else if (turnnum < 54)
          turnnum = (map(myPS3->getAnalogHat(LeftHatX), 0, 53, -turnspeed, -(turnspeed / 3)));

        if (abs(turnnum) > 5)
        {
          isFootMotorStopped = false;
        }

        currentMillis = millis();

        if ( (currentMillis - previousFootMillis) > serialLatency  )
        {

          if (footDriveSpeed != 0 || abs(turnnum) > 5)
          {

#ifdef SHADOW_VERBOSE
            Serial.print("Motor: FootSpeed: ");
            Serial.println(footDriveSpeed);
            Serial.print("\nTurnnum: ");
            Serial.println(turnnum);
            Serial.print("\nTime of command: ");
            Serial.println(millis());
#endif

            ST->turn(turnnum * invertturnDirection);
            ST->drive(footDriveSpeed);

          } else
          {
            if (!isFootMotorStopped)
            {
              ST->stop();
              isFootMotorStopped = true;
              footDriveSpeed = 0;
              //                      #ifdef SHADOW_VERBOSE
              //                         Serial.print("\r\n***Foot Motor STOPPED***\r\n");
              //                      #endif
            }
          }

          // The Sabertooth won't act on mixed mode packet serial commands until
          // it has received power levels for BOTH throttle and turning, since it
          // mixes the two together to get diff-drive power levels for both motors.

          previousFootMillis = currentMillis;
          return true; //we sent a foot command
        }
    }  // end of joystick processing
  }
  return false;
}

// =======================================================================================
//           FootMotorDrive Subroutine
// =======================================================================================

void footMotorDrive() 
{
  if (!autoNavigation)  
  {
    //Flood control prevention
    if ((millis() - previousFootMillis) < serialLatency) return;
    if (PS3NavFoot->PS3NavigationConnected) ps3FootMotorDrive(PS3NavFoot);
  }
  else
    autoFootMotorDrive();
}

// =======================================================================================
//           R2Sensor Automation footMotorDrive
// =======================================================================================
// Set the turnnum to the current direction indicated in Patrol and other commands
// Read in the footdrivespeed which is set from SPEED
void autoFootMotorDrive() 
{
  turnnum = turnDirection;  
  if (footDriveSpeed < speed)
  {
    if ((speed - footDriveSpeed) > (ramping + 1) )
    {
      footDriveSpeed += ramping;
      if (isDebug)
      {
        Serial.print("RAMPING: ");
        Serial.println(footDriveSpeed);
        Serial.print("Speed: ");
        Serial.println(speed);
        Serial.print("Turnnum: ");
        Serial.println(turnnum);  
      }
    }
    else
      footDriveSpeed = speed;
  }
  else if (footDriveSpeed > speed)
  {
    if ((footDriveSpeed - speed) > (ramping + 1))
    {
      footDriveSpeed -= ramping;
      #ifdef SHADOW_DEBUG
        Serial.print("RAMPING: ");
        Serial.println(footDriveSpeed);
        Serial.print("Speed: ");
        Serial.println(speed);
        Serial.print("Turnnum: ");
        Serial.println(turnnum);  
      #endif
    }
    else
      footDriveSpeed = speed;
  }
  else
    footDriveSpeed = speed;
    
// Check footmotor and turndirection and verify its not stopped
  turnnum = turnDirection;
  currentMillis = millis();   // Mark the time
  if ( abs(footDriveSpeed) != 0 || abs(turnnum) > 5)
  {
    isFootMotorStopped = false;
    #ifdef SHADOW_DEBUG
      Serial.print("Motor: FootSpeed: ");
      Serial.println(footDriveSpeed);
      Serial.print("\nturnDirection: ");
      Serial.println(turnDirection);
      Serial.print("Turnnum: ");
      Serial.println(turnnum);
      Serial.print("\nTime of command: ");
      Serial.println(millis());
      Serial.print("\nState: ");
      Serial.println(R2state);
    #endif
    if (R2state == MovingFwd)             // Check to see if you are running forward, if so keep it the same
    {
      speed = speed;
      if (isDebug)
      {
        Serial.print("State: ");
        Serial.print(R2state); 
        Serial.print(" LeftFront: ");
        Serial.print(leftfront); 
        Serial.print(" RightFront: ");
        Serial.print(rightfront); 
        Serial.print(" distance: ");
        Serial.println(distance); 
      }    
    }
  
    else if (R2state == MovingBck)        // If the state machine is set to backwards, then reverse the speed.
    {
      speed = speed * -1;
      if (isDebug)
      {
        Serial.print("State: ");
        Serial.print(R2state); 
        Serial.print(" LeftBack: ");
        Serial.print(leftback); 
        Serial.print(" RightBackt: ");
        Serial.print(rightback); 
        Serial.print(" distance: ");
        Serial.println(distance); 
      } 
    }
    ST->turn(turnnum * invertturnDirection);
    ST->drive(footDriveSpeed);
  } 
  else
  {
    if (!isFootMotorStopped)
    {
      R2state = Stopped;
      ST->stop();
      isFootMotorStopped = true;
      footDriveSpeed = 0;
      #ifdef SHADOW_DEBUG
          Serial.print("\r\nFoot Motor STOPPED...\r\n");
      #endif
      return;
    }
  }
} // end of autoFootmotor drive

// =======================================================================================
//           domeDrive Motor Control Section
// =======================================================================================
int ps3DomeDrive(PS3BT* myPS3 = PS3NavDome)
{
  int domeRotationSpeed = 0;

  int joystickPosition = myPS3->getAnalogHat(LeftHatX);

  domeRotationSpeed = (map(joystickPosition, 0, 255, -domespeed, domespeed));

  if ( abs(joystickPosition - 128) < joystickDomeDeadZoneRange )
    domeRotationSpeed = 0;

  if (domeRotationSpeed != 0 && domeAutomation == true)  // Turn off dome automation if manually moved
  {
    domeAutomation = false;
    domeStatus = 0;
    domeTargetPosition = 0;

    //            #ifdef SHADOW_VERBOSE
    //              Serial.print("Dome Automation OFF\r\n");
    //            #endif

  }

  return domeRotationSpeed;
}

void rotateDome(int domeRotationSpeed, String mesg)
{
  //Constantly sending commands to the SyRen (Dome) is causing foot motor delay.
  //Lets reduce that chatter by trying 3 things:
  // 1.) Eliminate a constant stream of "don't spin" messages (isDomeMotorStopped flag)
  // 2.) Add a delay between commands sent to the SyRen (previousDomeMillis timer)
  // 3.) Switch to real UART on the MEGA (Likely the *CORE* issue and solution)
  // 4.) Reduce the timout of the SyRen - just better for safety!

  currentMillis = millis();
  if ( (!isDomeMotorStopped || domeRotationSpeed != 0) && ((currentMillis - previousDomeMillis) > (2 * serialLatency) )  )
  {

    if (domeRotationSpeed != 0)
    {

      isDomeMotorStopped = false;

#ifdef SHADOW_VERBOSE
      Serial.print("Dome rotation speed: ");
      Serial.print(domeRotationSpeed);
#endif

      SyR->motor(domeRotationSpeed);

    } else
    {
      isDomeMotorStopped = true;

#ifdef SHADOW_VERBOSE
      Serial.print("\n\r***Dome motor is STOPPED***\n\r");
#endif

      SyR->stop();
    }

    previousDomeMillis = currentMillis;
  }
}

void domeDrive()
{
  //Flood control prevention
  //This is intentionally set to double the rate of the Dome Motor Latency
  if ((millis() - previousDomeMillis) < (2 * serialLatency) ) return;

  int domeRotationSpeed = 0;
  int ps3NavControlSpeed = 0;

  if (PS3NavDome->PS3NavigationConnected)
  {

    ps3NavControlSpeed = ps3DomeDrive(PS3NavDome);

    domeRotationSpeed = ps3NavControlSpeed;

    rotateDome(domeRotationSpeed, "Controller Move");

  } else if (PS3NavFoot->PS3NavigationConnected && PS3NavFoot->getButtonPress(L2))
  {

    ps3NavControlSpeed = ps3DomeDrive(PS3NavFoot);

    domeRotationSpeed = ps3NavControlSpeed;

    rotateDome(domeRotationSpeed, "Controller Move");

  } else
  {
    if (!isDomeMotorStopped)
    {
      SyR->stop();
      isDomeMotorStopped = true;
    }
  }
}

// =======================================================================================
//                               Dome Rotation Routines
//            This allows dome navigation to work using multiple compasses
// =======================================================================================
//                               ________________________
//                              |         Mic1          |
//                              |Mic2  (Top View)   Mic3|
//                              |         Mic4          |
//                              |_______________________|

#ifdef SOUNDDETECTION
void R2soundcheck() {
  mic1 = analogRead(A0);
  mic2 = analogRead(A1);
  mic3 = analogRead(A2);
  mic4 = analogRead(A3);
  int domeRotationSpeed;
  long rndNum;
  int domeSpeed, mic1direction, mic2direction, mic3direction, mic4direction;
  domeTargetPosition = bodynav;
  mic1direction =  bodynav;       // Mic 1 is set to the front
  mic2direction = (bodynav - 90); // Mic 2 is 90 degrees to the left of front
  mic3direction = (bodynav + 90); // Mic 3 is 90 degrees to the right of front
  mic4direction = (bodynav - 180); // Mic 4 is 180 degrees opposite front
  if (mic2direction < 0)    mic2direction = mic2direction + 360;
  if (mic4direction < 0)    mic4direction = mic4direction + 360;
  if (mic3direction > 360)  mic3direction = mic3direction - 360;
  //Test is threshold (50) hurdle met before proceeding
  if (mic1 - mic2 > threshold || mic2 - mic1 > threshold || mic2 - mic3 > threshold || mic3 - mic2 > threshold ||  mic3 - mic1 > threshold ||  mic1 - mic3 > threshold || mic4 - mic3 > threshold || mic3 - mic4 > threshold || mic4 - mic1 > threshold || mic1 - mic4 > threshold)  {
    #ifdef SHADOW_VERBOSE
    Serial.print (mic1direction);
    Serial.print (mic2direction);
    Serial.print (mic3direction);
    Serial.println(mic4direction);
    #endif
    // Sound Direction Algorithm
    if (mic1 > mic2 || mic1 > mic3) {
      domeTargetPosition = mic1direction;  // Turn the dome forward
    }
    if (mic2 > mic4 || mic2 > mic1) {
      domeTargetPosition = mic2direction;  // Turn the dome left or to the leftside depending upon where the dome sits
    }
    if (mic3 > mic1 || mic3 > mic4) {
      domeTargetPosition = mic3direction;  // Turn the dome right or to the rightside depending upon where the dome sits
    }
    if (mic4 > mic3 || mic4 > mic2) {
      domeTargetPosition = mic4direction;  // Turn the dome to look backwards
    }
  }
}
#endif

void autodomeDrive()  
{
  int domeRotationSpeed;
  long rndNum;
  int domeSpeed;
  rndNum = random(5, 354);
  if (PS3NavDome->PS3NavigationConnected)   // Verify the PS3 controllers are connnected
  {                         
    if (domeTargetPosition == bodynav)  
    {
      domeRotationSpeed = 0;                // Dome is already facing forward - stop
      domeStatus = 0;                       // Set dome movement status to stopped
    }
    if (domeStatus == 0)                    // 0 = stopped, 1 = prepare to turn, 2 = turning
    {                                         
      if (domenav != domeTargetPosition)    // Dome is currently stopped position - prepare to turn towards sound
      {                        
        if (domeTargetPosition < 180)  // Turn the dome in the positive direction
        {

          dometurnDirection = 1;

          domeStopTurnTime = domeStartTurnTime + ((domeTargetPosition / 360) * time360DomeTurn);

        } else  // Turn the dome in the negative direction
        {

          dometurnDirection = -1;

          domeStopTurnTime = domeStartTurnTime + (((360 - domeTargetPosition) / 360) * time360DomeTurn);

        }

      } else  // Dome is not in the home position - send it back to home
      {

        domeStartTurnTime = millis() + (random(3, 10) * 1000);

        if (domeTargetPosition < 180)
        {

          dometurnDirection = -1;

          domeStopTurnTime = domeStartTurnTime + ((domeTargetPosition / 360) * time360DomeTurn);

        } else
        {

          dometurnDirection = 1;

          domeStopTurnTime = domeStartTurnTime + (((360 - domeTargetPosition) / 360) * time360DomeTurn);

        }

        domeTargetPosition = 0;

      }

      domeStatus = 1;  // Set dome status to preparing for a future turn

#ifdef SHADOW_DEBUG
      Serial.print("Dome Automation: Initial Turn Set\r\n");
      Serial.print( "Current Time: ");
      Serial.print( millis());
      Serial.print("\r\n Next Start Time: ");
      Serial.print(domeStartTurnTime);
      Serial.print("\r\n");
      Serial.print("Next Stop Time: ");
      Serial.print(domeStopTurnTime);
      Serial.print("\r\n");
      Serial.print("Dome Target Position: ");
      Serial.print(domeTargetPosition);
      Serial.print("\r\n");
#endif

    }


    if (domeStatus == 1)  // Dome is prepared for a future move - start the turn when ready
    {

      if (domeStartTurnTime < millis())
      {

        domeStatus = 2;

        //             #ifdef SHADOW_DEBUG
        //                Serial.print("Dome Automation: Ready To Start Turn\r\n");
        //             #endif

      }
    }

    if (domeStatus == 2) // Dome is now actively turning until it reaches its stop time
    {

      if (domeStopTurnTime > millis())
      {

        domeSpeed = domeAutoSpeed * dometurnDirection;

        SyR->motor(domeSpeed);

        //             #ifdef SHADOW_DEBUG
        //                Serial.print("Turning Now!!\r\n");
        //             #endif


      } else  // turn completed - stop the motor
      {
        domeStatus = 0;
        SyR->stop();

        //              #ifdef SHADOW_DEBUG
        //                 Serial.print("STOP TURN!!\r\n");
        //              #endif
      }

    }

  }
  if (!isDomeMotorStopped)
  {
    SyR->stop();
    isDomeMotorStopped = true;
  }
}

// =======================================================================================
//           R2Sensor Decision(s) |  Recenter the Dome
// =======================================================================================

void recenterDome()  
{
  if (PS3NavDome->PS3NavigationConnected)   // Verify the PS3 controllers are connnected
  {
  #ifdef COMPASS
  #ifdef USE_GPS
    int domeSpeed,viewError,domeRotationSpeed;
    currentTime = millis();
    if (isDebug)  
    {
      Serial.print("Current Time:");
      Serial.println(currentTime);
      Serial.print("DomelastDecisionTime:");
      Serial.println(DomelastDecisionTime);
    }
    if ((currentTime - DomelastDecisionTime) >= automationInterval)
    {
      DomelastDecisionTime = millis();
      R2domeStatus == Turn;
    }
    if (R2domeStatus != Error && R2domeStatus == Turn && viewError > HEADING_TOLERANCE) 
    {
      // If dome status is not in error and is turning while the degrees do not match between dome and body above tolerance levels.
      R2domeStatus = Turn;
      #ifndef COMPASS_HMC5883L
      tcaselect(1);
      #endif
      if (magbody.begin())  
      {
        bodynav = readBodyCompass();                // Get our current heading from Body COMPASS portion of the code
        if (isDebug)  
        {
          Serial.print("Bodynav: ");
          Serial.println(bodynav);
        }
      }
      else
      {
        if (isDebug)  
        {
          Serial.println("Body Compass not found ");
        }
        R2domeStatus = Error;
      }
      #ifndef COMPASS_HMC5883L
      tcaselect(0);
      if (magdome.begin())  
      #else
      if (compass.begin()) 
      #endif
      {
        domenav = readDomeCompass();                // Get our current heading from Dome COMPASS portion of the code
        if (isDebug)  
        {
          Serial.print("Domenav: ");
          Serial.println(domenav);
        }
      } 
      else
      {
      if (isDebug)  
          Serial.println("Dome Compass not found ");
        R2domeStatus = Error;
      }
      
      viewError = domenav - bodynav;                          // calculate which way to turn the dome to intercept the targetHeading
      if (abs(viewError) <= HEADING_TOLERANCE)                // if within tolerance, don't turn
      {
        dometurnDirection = 0;
        R2domeStatus = Home;
      }
      else if (viewError < 0)                                 // if error is a negative number, then we assume Dome is less than body number.
      {
        if (abs(viewError) > 180)                             // if error is more than 180 degrees turn left
        {
          dometurnDirection = -1;
          R2domeStatus = Turn;
        }
  
        else
        {
          dometurnDirection = 1;
          R2domeStatus = Turn;
        }
      }
      else if (viewError > 0)                                 // if error is a positive number, then we assume Dome is more than body number.
      {
        if (abs(viewError) < 180)                             // if error is less than 180 degrees turn left
        {
          dometurnDirection = -1;
          R2domeStatus = Turn;
        }
        else
        {
          dometurnDirection = 1;
          R2domeStatus = Turn;
        }
      }
      else
      {
        dometurnDirection = -1;
        R2domeStatus = Turn;
      }
      domeSpeed = domeAutoSpeed * dometurnDirection;
      if (abs(viewError) <= HEADING_TOLERANCE)
      {
        R2domeStatus = Home;
        domeStatus = 0;
        SyR->stop();
        if (isDebug)
          {
            Serial.print  ("viewError: ");
            Serial.print  (viewError);
            Serial.println("...Stopping Dome\r\n");
          }
      } 
      else 
      {
        R2domeStatus = Turn;
        domeSpeed = domeAutoSpeed * dometurnDirection;
        SyR->motor(domeSpeed);
        if (isDebug)
          {
            Serial.print ("...Auto Return Dome\r\n");
            Serial.print  ("viewError: ");
            Serial.print  (viewError);
            Serial.print ("Dome Status:");
            Serial.println (R2domeStatus);
            Serial.print ("Dome Turning Direction:");
            Serial.println (dometurnDirection);
            Serial.print  ("Turning Dome ");
            Serial.println((domespeed));
          }
      }
    }
    else
      R2domeStatus = Home;
  #endif
  #endif
  }
}

// =======================================================================================
//           R2Sensor Decision(s) |  Dome turn towards human (PIR sensor readings)
// =======================================================================================

void followDome(int pir)  
{
  if (PS3NavDome->PS3NavigationConnected)                    // Verify the PS3 controllers are connnected
  {
  #ifdef COMPASS
  #ifdef USE_GPS
    int domeSpeed,viewError,domeRotationSpeed;
    currentTime = millis();
    viewError = domenav - bodynav;                          // calculate which way to turn the dome to intercept the targetHeading
    if (isDebug)  
    {
      Serial.print("I detected human from PIR:");
      Serial.println(pir);
    }
    DomelastDecisionTime = millis();
    R2domeStatus = Turn;
    if (R2domeStatus != Error && R2domeStatus == Turn && viewError > HEADING_TOLERANCE) 
    {
      // If dome status is not in error and is turning while the degrees do not match between dome and body above tolerance levels.
      R2domeStatus = Turn;
      R2Compass();                                            // Calculate headings from both dome and body compass
      viewError = domenav - bodynav;                          // calculate which way to turn the dome to intercept the targetHeading
      if (abs(viewError) <= HEADING_TOLERANCE)                // if within tolerance, don't turn
      {
        dometurnDirection = 0;
        R2domeStatus = Home;
      }
      else if (viewError < 0)                                 // if error is a negative number, then we assume Dome is less than body number.
      {
        if (abs(viewError) > 180)                             // if error is more than 180 degrees turn left
        {
          dometurnDirection = -1;
          R2domeStatus = Turn;
        }
  
        else
        {
          dometurnDirection = 1;
          R2domeStatus = Turn;
        }
      }
      else if (viewError > 0)                                 // if error is a positive number, then we assume Dome is more than body number.
      {
        if (abs(viewError) < 180)                             // if error is less than 180 degrees turn left
        {
          dometurnDirection = -1;
          R2domeStatus = Turn;
        }
        else
        {
          dometurnDirection = 1;
          R2domeStatus = Turn;
        }
      }
      else
      {
        dometurnDirection = -1;
        R2domeStatus = Turn;
      }
      domeSpeed = domeAutoSpeed * dometurnDirection;
      if (abs(viewError) <= HEADING_TOLERANCE)
      {
        R2domeStatus = Home;
        domeStatus = 0;
        SyR->stop();
        if (isDebug)
          {
            Serial.print  ("viewError: ");
            Serial.print  (viewError);
            Serial.println("...Stopping Dome\r\n");
          }
      } 
      else 
      {
        R2domeStatus = Turn;
        domeSpeed = domeAutoSpeed * dometurnDirection;
        SyR->motor(domeSpeed);
        if (isDebug)
          {
            Serial.print ("...Auto Return Dome\r\n");
            Serial.print  ("viewError: ");
            Serial.print  (viewError);
            Serial.print ("Dome Status:");
            Serial.println (R2domeStatus);
            Serial.print ("Dome Turning Direction:");
            Serial.println (dometurnDirection);
            Serial.print  ("Turning Dome ");
            Serial.println((domespeed));
          }
      }
    }
    else
      R2domeStatus = Home;
  #endif
  #endif
  }
}

// =======================================================================================
//                               Voice Toggle Control Section
//    http://www.curiousmarc.com/dome-automation/marcduino-firmware/command-reference
// =======================================================================================
#ifdef VOICE
void voicecontrol()  
{
  if (isPS3NavigatonInitialized)
  {
    if (autoNavigation)
    {
      if (voicecmd == 1)  {                           // Turn on AutoNavigation and go on Patrol command (Patrol)
        R2mode = Patroling;                           // R2 mode is now Patrol
        R2state = MovingFwd;                          // R2 initial state will be set to moving forward
        Serial1.print(":SE14\r");                     // Full Awake Mode reset (panel close, random sound, holo movement, no holo lights)
        voicecmd = -1;
        if (isDebug)
          Serial.println("Go on Patrol\r\n");
      }
      else if (voicecmd == 2)  {                      // Do you remember Luke Skywalker
        R2state = Stopped;                            // R2 Should be standing still
        Serial1.print(":SE05\r");                     // Unknown
        voicecmd = -1;
      }
      else if (voicecmd == 3)  {                      // Do you remember Vader
        R2state = Stopped;                            // R2 Should be standing still
        Serial1.print(":SE01\r");                     // Scream, with all panels open
        voicecmd = -1;
        if (isDebug)
          Serial.println("Vader\r\n");
      }
      else if (voicecmd == 5)  {                      // Do you remember Lea
        R2state = Stopped;                            // R2 Should be standing still
        Serial1.print(":$L\r");                       // Send command to play princess Lea message
        voicecmd = -1;
        if (isDebug)
          Serial.println("Lea\r\n");
      }
      else if (voicecmd == 6)  {                      // Do you remember C3PO
        R2state = Stopped;                            // R2 Should be standing still
        Serial1.print(":SE54\r");                     // Wave 2 (open progressively all panels, then close one by one)
        voicecmd = -1;
        if (isDebug)
          Serial.println("3PO\r\n");
      }
      else if (voicecmd == 8)  {                      // Open the Marcduino Panels (open panels)
        R2state = Stopped;                            // R2 Should be standing still
        Serial1.print(":OP00\r");                     // Send command to Open Panels to Marcduino
        voicecmd = -1;
        if (isDebug)
          Serial.println("Open Panels\r\n");
      }
      else if (voicecmd == 9)  {                      // Close the Marcduino Panels (close panels)
        R2state = Stopped;                            // R2 Should be standing still
        Serial1.print(":CL00\r");                     // Send Soft Close Panels to Marcduino
        voicecmd = -1;
        if (isDebug)
          Serial.println("Close Panels\r\n");
      }
      else if (voicecmd == 7)  {                      // Turn off AutoNavigation and stop (stay)
        R2state = Stopped;                            // Set the state to stopped
        R2mode = None;
        Serial1.print(":$2\r");                       // Send Sound Command for AutoNavigation Off
        Serial1.print(":SE10\r");                     // Quite Mode reset (panel close, stop holos, stop sounds)
        voicecmd = -1;
        if (isDebug)
          Serial.println("Stay... Quiet mode\r\n");
      }
      else if (voicecmd == 10)  {                     // Tell R2 to backup
        R2state = MovingBck;                          // Set the state to MovingBck
        R2mode = Aware;
        voicecmd = -1;
        if (isDebug)
          Serial.println("Moving Backwards\r\n");
      }
      else if (voicecmd == 11)  {                     // Tell R2 to go forward
        R2state = MovingFwd;                          // Set the state to MovingFwd
        R2mode = Aware;
        voicecmd = -1;
        if (isDebug)
          Serial.println("Moving Forwards\r\n");
      }
      else if (voicecmd == 12)  {                     // Tell R2 to Go for Walk
        R2state = MovingFwd;                          // Set the state to MovingFwd
        R2mode = Waypoint;                            // R2 mode is now Waypoint: walk
        voicecmd = -1;
        if (isDebug)
          Serial.println("Waypoint: Walk around block\r\n");
      }
      else if (voicecmd == 98)  
      {                                               // Time out was recieved
        Serial1.print(":SE13\r");                     // Mid Awake Mode reset (panel close, random sound, stop holos)
        voicecmd = -1;
        if (isDebug)
          Serial.println("R2 Is no longer waiting for 2nd command\r\n");
      }
      else if (voicecmd == 99)  
      {                                               // R2 didnt understand the command
        Serial1.print(":SE53\r");                     // Fast (Smirk) back and forth wave
        voicecmd = -1;
        if (isDebug)
          Serial.println("R2 didnt understand the command\r\n");
      }
    } 
    else
    return;
  }
}
#endif

// =======================================================================================
//                               Toggle Control Section
// =======================================================================================

void ps3ToggleSettings(PS3BT* myPS3 = PS3NavFoot)
{
  // enable / disable drive stick
  if (myPS3->getButtonPress(PS) && myPS3->getButtonClick(CROSS))
  {
    #ifdef SHADOW_DEBUG
      Serial.print("Disabling the DriveStick\r\n");
      Serial.print("Stopping Motors");
    #endif
      ST->stop();
      isFootMotorStopped = true;
      isStickEnabled = false;
      footDriveSpeed = 0;
  }

  if (myPS3->getButtonPress(PS) && myPS3->getButtonClick(CIRCLE))
  {
  #ifdef SHADOW_DEBUG
    Serial.print("Enabling the DriveStick\r\n");
  #endif
    isStickEnabled = true;
  }
  // Enable and Disable Overspeed
  if (myPS3->getButtonPress(L3) && myPS3->getButtonPress(L1) && isStickEnabled)
  {
    if ((millis() - previousSpeedToggleMillis) > 1000)
    {
      speedToggleButtonCounter = 0;
      previousSpeedToggleMillis = millis();
    }
    speedToggleButtonCounter += 1;
    if (speedToggleButtonCounter == 1)
    {
      if (!overSpeedSelected)
      {
        overSpeedSelected = true;
        #ifdef SHADOW_VERBOSE
          Serial.print("Over Speed is now: ON");
        #endif
      } 
      else
      {
        overSpeedSelected = false;
        #ifdef SHADOW_VERBOSE
          Serial.print("Over Speed is now: OFF");
        #endif
      }
    }
  }
  if (myPS3->getButtonPress(L2) && myPS3->getButtonClick(CIRCLE))  
  {
    if (domeAutomation == true) 
    {
      #ifdef SHADOW_DEBUG
      Serial.print("Dome Automation Off\r\n");
      #endif
      domeAutomation = false;
    } 
    else 
    {
      domeAutomation = true;
      #ifdef SHADOW_DEBUG
        Serial.print("Dome Automation On\r\n");
      #endif
    }
  }

  #ifdef COLLISION
  // Enable Disable Auto Navigation and Dome Automation - updated on 9/11/2015 to toggle mode SEE BELOW rewritten code
  if (myPS3->getButtonPress(L2) && myPS3->getButtonClick(CROSS)) {
    if (autoNavigation == true) 
    {
      autoNavigation = false;
      speed = -25;
      R2state = Stopped;
      if (isDebug)
        Serial.println("Stop Automation\r\n");
    } 
    else 
    {
      autoNavigation = true;
      if (isDebug)
        Serial.println("Starting Automation\r\n");
    }
  }
  #endif

  #ifdef USE_GPS
  // Enable Disable GPS Waypoint Seeking - updated on 9/24/2015 to toggle mode
  if (myPS3->getButtonPress(L2) && myPS3->getButtonClick(CROSS) && myPS3->getButtonClick(L1)) 
  {
    if (R2mode == Waypoint)
    {
      R2mode = None;
      if (isDebug)
      {
        Serial.print("GPS Off\r\n");
        Serial.print(Waypointenabled);
        Serial.println(""); 
      }
    } 
    else 
    {
      R2mode = Waypoint;
      if (isDebug)
      {
      Serial.print("GPS On\r\n");
      Serial.println(Waypointenabled);
      }
    }
  }
#endif

}


void toggleSettings()
{
  if (PS3NavFoot->PS3NavigationConnected) ps3ToggleSettings(PS3NavFoot);
}

// ====================================================================================================================
// This function determines if MarcDuino buttons were selected and calls main processing function for FOOT controller
// ====================================================================================================================

void marcDuinoFoot()
{
  if (PS3NavFoot->PS3NavigationConnected && (PS3NavFoot->getButtonPress(UP) || PS3NavFoot->getButtonPress(DOWN) || PS3NavFoot->getButtonPress(LEFT) || PS3NavFoot->getButtonPress(RIGHT)))
  {
    if ((millis() - previousMarcDuinoMillis) > 1000)
    {
      marcDuinoButtonCounter = 0;
      previousMarcDuinoMillis = millis();
    }
    marcDuinoButtonCounter += 1;
  } 
  else
  {
    return;
  }
  // Clear inbound buffer of any data sent from the MarcDuino board
  while (Serial1.available()) Serial1.read();
// ====================================================================================================================
// Send triggers for the base buttons
// ====================================================================================================================
  if (PS3NavFoot->getButtonPress(UP) && !PS3NavFoot->getButtonPress(CROSS) && !PS3NavFoot->getButtonPress(CIRCLE) && !PS3NavFoot->getButtonPress(L1) && !PS3NavFoot->getButtonPress(PS) && marcDuinoButtonCounter == 1)
  {
    if (PS3NavDome->PS3NavigationConnected && (PS3NavDome->getButtonPress(CROSS) || PS3NavDome->getButtonPress(CIRCLE) || PS3NavDome->getButtonPress(PS)))
    {
      // Skip this section
    } 
    else
    {
      marcDuinoButtonPush(btnUP_type, btnUP_MD_func, btnUP_cust_MP3_num, btnUP_cust_LD_type, btnUP_cust_LD_text, btnUP_cust_panel,
                          btnUP_use_DP1,
                          btnUP_DP1_open_start_delay,
                          btnUP_DP1_stay_open_time,
                          btnUP_use_DP2,
                          btnUP_DP2_open_start_delay,
                          btnUP_DP2_stay_open_time,
                          btnUP_use_DP3,
                          btnUP_DP3_open_start_delay,
                          btnUP_DP3_stay_open_time,
                          btnUP_use_DP4,
                          btnUP_DP4_open_start_delay,
                          btnUP_DP4_stay_open_time,
                          btnUP_use_DP5,
                          btnUP_DP5_open_start_delay,
                          btnUP_DP5_stay_open_time,
                          btnUP_use_DP6,
                          btnUP_DP6_open_start_delay,
                          btnUP_DP6_stay_open_time,
                          btnUP_use_DP7,
                          btnUP_DP7_open_start_delay,
                          btnUP_DP7_stay_open_time,
                          btnUP_use_DP8,
                          btnUP_DP8_open_start_delay,
                          btnUP_DP8_stay_open_time,
                          btnUP_use_DP9,
                          btnUP_DP9_open_start_delay,
                          btnUP_DP9_stay_open_time,
                          btnUP_use_DP10,
                          btnUP_DP10_open_start_delay,
                          btnUP_DP10_stay_open_time);

#ifdef SHADOW_VERBOSE
      Serial.print("FOOT: btnUP");
#endif
      return;
    }
  }

  if (PS3NavFoot->getButtonPress(DOWN) && !PS3NavFoot->getButtonPress(CROSS) && !PS3NavFoot->getButtonPress(CIRCLE) && !PS3NavFoot->getButtonPress(L1) && !PS3NavFoot->getButtonPress(PS) && marcDuinoButtonCounter == 1)
  {
    if (PS3NavDome->PS3NavigationConnected && (PS3NavDome->getButtonPress(CROSS) || PS3NavDome->getButtonPress(CIRCLE) || PS3NavDome->getButtonPress(PS)))
    {
      // Skip this section
    } 
    else
    {
      marcDuinoButtonPush(btnDown_type, btnDown_MD_func, btnDown_cust_MP3_num, btnDown_cust_LD_type, btnDown_cust_LD_text, btnDown_cust_panel,
                          btnDown_use_DP1,
                          btnDown_DP1_open_start_delay,
                          btnDown_DP1_stay_open_time,
                          btnDown_use_DP2,
                          btnDown_DP2_open_start_delay,
                          btnDown_DP2_stay_open_time,
                          btnDown_use_DP3,
                          btnDown_DP3_open_start_delay,
                          btnDown_DP3_stay_open_time,
                          btnDown_use_DP4,
                          btnDown_DP4_open_start_delay,
                          btnDown_DP4_stay_open_time,
                          btnDown_use_DP5,
                          btnDown_DP5_open_start_delay,
                          btnDown_DP5_stay_open_time,
                          btnDown_use_DP6,
                          btnDown_DP6_open_start_delay,
                          btnDown_DP6_stay_open_time,
                          btnDown_use_DP7,
                          btnDown_DP7_open_start_delay,
                          btnDown_DP7_stay_open_time,
                          btnDown_use_DP8,
                          btnDown_DP8_open_start_delay,
                          btnDown_DP8_stay_open_time,
                          btnDown_use_DP9,
                          btnDown_DP9_open_start_delay,
                          btnDown_DP9_stay_open_time,
                          btnDown_use_DP10,
                          btnDown_DP10_open_start_delay,
                          btnDown_DP10_stay_open_time);

#ifdef SHADOW_VERBOSE
      Serial.print("FOOT: btnDown");
#endif
      return;
    }
  }

  if (PS3NavFoot->getButtonPress(LEFT) && !PS3NavFoot->getButtonPress(CROSS) && !PS3NavFoot->getButtonPress(CIRCLE) && !PS3NavFoot->getButtonPress(L1) && !PS3NavFoot->getButtonPress(PS) && marcDuinoButtonCounter == 1)
  {
    if (PS3NavDome->PS3NavigationConnected && (PS3NavDome->getButtonPress(CROSS) || PS3NavDome->getButtonPress(CIRCLE) || PS3NavDome->getButtonPress(PS)))
    {
      // Skip this section
    } 
    else
    {
      marcDuinoButtonPush(btnLeft_type, btnLeft_MD_func, btnLeft_cust_MP3_num, btnLeft_cust_LD_type, btnLeft_cust_LD_text, btnLeft_cust_panel,
                          btnLeft_use_DP1,
                          btnLeft_DP1_open_start_delay,
                          btnLeft_DP1_stay_open_time,
                          btnLeft_use_DP2,
                          btnLeft_DP2_open_start_delay,
                          btnLeft_DP2_stay_open_time,
                          btnLeft_use_DP3,
                          btnLeft_DP3_open_start_delay,
                          btnLeft_DP3_stay_open_time,
                          btnLeft_use_DP4,
                          btnLeft_DP4_open_start_delay,
                          btnLeft_DP4_stay_open_time,
                          btnLeft_use_DP5,
                          btnLeft_DP5_open_start_delay,
                          btnLeft_DP5_stay_open_time,
                          btnLeft_use_DP6,
                          btnLeft_DP6_open_start_delay,
                          btnLeft_DP6_stay_open_time,
                          btnLeft_use_DP7,
                          btnLeft_DP7_open_start_delay,
                          btnLeft_DP7_stay_open_time,
                          btnLeft_use_DP8,
                          btnLeft_DP8_open_start_delay,
                          btnLeft_DP8_stay_open_time,
                          btnLeft_use_DP9,
                          btnLeft_DP9_open_start_delay,
                          btnLeft_DP9_stay_open_time,
                          btnLeft_use_DP10,
                          btnLeft_DP10_open_start_delay,
                          btnLeft_DP10_stay_open_time);

#ifdef SHADOW_VERBOSE
      Serial.print("FOOT: btnLeft");
#endif
      return;
    }
  }
  if (PS3NavFoot->getButtonPress(RIGHT) && !PS3NavFoot->getButtonPress(CROSS) && !PS3NavFoot->getButtonPress(CIRCLE) && !PS3NavFoot->getButtonPress(L1) && !PS3NavFoot->getButtonPress(PS) && marcDuinoButtonCounter == 1)
  {
    if (PS3NavDome->PS3NavigationConnected && (PS3NavDome->getButtonPress(CROSS) || PS3NavDome->getButtonPress(CIRCLE) || PS3NavDome->getButtonPress(PS)))
    {
      // Skip this section
    } else
    {
      marcDuinoButtonPush(btnRight_type, btnRight_MD_func, btnRight_cust_MP3_num, btnRight_cust_LD_type, btnRight_cust_LD_text, btnRight_cust_panel,
                          btnRight_use_DP1,
                          btnRight_DP1_open_start_delay,
                          btnRight_DP1_stay_open_time,
                          btnRight_use_DP2,
                          btnRight_DP2_open_start_delay,
                          btnRight_DP2_stay_open_time,
                          btnRight_use_DP3,
                          btnRight_DP3_open_start_delay,
                          btnRight_DP3_stay_open_time,
                          btnRight_use_DP4,
                          btnRight_DP4_open_start_delay,
                          btnRight_DP4_stay_open_time,
                          btnRight_use_DP5,
                          btnRight_DP5_open_start_delay,
                          btnRight_DP5_stay_open_time,
                          btnRight_use_DP6,
                          btnRight_DP6_open_start_delay,
                          btnRight_DP6_stay_open_time,
                          btnRight_use_DP7,
                          btnRight_DP7_open_start_delay,
                          btnRight_DP7_stay_open_time,
                          btnRight_use_DP8,
                          btnRight_DP8_open_start_delay,
                          btnRight_DP8_stay_open_time,
                          btnRight_use_DP9,
                          btnRight_DP9_open_start_delay,
                          btnRight_DP9_stay_open_time,
                          btnRight_use_DP10,
                          btnRight_DP10_open_start_delay,
                          btnRight_DP10_stay_open_time);

#ifdef SHADOW_VERBOSE
      Serial.print("FOOT: btnRight");
#endif


      return;
    }

  }

  //------------------------------------
  // Send triggers for the CROSS + base buttons
  //------------------------------------
  if (((!PS3NavDome->PS3NavigationConnected && PS3NavFoot->getButtonPress(UP) && PS3NavFoot->getButtonPress(CROSS)) || (PS3NavDome->PS3NavigationConnected && PS3NavFoot->getButtonPress(UP) && PS3NavDome->getButtonPress(CROSS))) && marcDuinoButtonCounter == 1)
  {

    marcDuinoButtonPush(btnUP_CROSS_type, btnUP_CROSS_MD_func, btnUP_CROSS_cust_MP3_num, btnUP_CROSS_cust_LD_type, btnUP_CROSS_cust_LD_text, btnUP_CROSS_cust_panel,
                        btnUP_CROSS_use_DP1,
                        btnUP_CROSS_DP1_open_start_delay,
                        btnUP_CROSS_DP1_stay_open_time,
                        btnUP_CROSS_use_DP2,
                        btnUP_CROSS_DP2_open_start_delay,
                        btnUP_CROSS_DP2_stay_open_time,
                        btnUP_CROSS_use_DP3,
                        btnUP_CROSS_DP3_open_start_delay,
                        btnUP_CROSS_DP3_stay_open_time,
                        btnUP_CROSS_use_DP4,
                        btnUP_CROSS_DP4_open_start_delay,
                        btnUP_CROSS_DP4_stay_open_time,
                        btnUP_CROSS_use_DP5,
                        btnUP_CROSS_DP5_open_start_delay,
                        btnUP_CROSS_DP5_stay_open_time,
                        btnUP_CROSS_use_DP6,
                        btnUP_CROSS_DP6_open_start_delay,
                        btnUP_CROSS_DP6_stay_open_time,
                        btnUP_CROSS_use_DP7,
                        btnUP_CROSS_DP7_open_start_delay,
                        btnUP_CROSS_DP7_stay_open_time,
                        btnUP_CROSS_use_DP8,
                        btnUP_CROSS_DP8_open_start_delay,
                        btnUP_CROSS_DP8_stay_open_time,
                        btnUP_CROSS_use_DP9,
                        btnUP_CROSS_DP9_open_start_delay,
                        btnUP_CROSS_DP9_stay_open_time,
                        btnUP_CROSS_use_DP10,
                        btnUP_CROSS_DP10_open_start_delay,
                        btnUP_CROSS_DP10_stay_open_time);

#ifdef SHADOW_VERBOSE
    Serial.print("FOOT: btnUP_CROSS");
#endif


    return;

  }

  if (((!PS3NavDome->PS3NavigationConnected && PS3NavFoot->getButtonPress(DOWN) && PS3NavFoot->getButtonPress(CROSS)) || (PS3NavDome->PS3NavigationConnected && PS3NavFoot->getButtonPress(DOWN) && PS3NavDome->getButtonPress(CROSS))) && marcDuinoButtonCounter == 1)
  {

    marcDuinoButtonPush(btnDown_CROSS_type, btnDown_CROSS_MD_func, btnDown_CROSS_cust_MP3_num, btnDown_CROSS_cust_LD_type, btnDown_CROSS_cust_LD_text, btnDown_CROSS_cust_panel,
                        btnDown_CROSS_use_DP1,
                        btnDown_CROSS_DP1_open_start_delay,
                        btnDown_CROSS_DP1_stay_open_time,
                        btnDown_CROSS_use_DP2,
                        btnDown_CROSS_DP2_open_start_delay,
                        btnDown_CROSS_DP2_stay_open_time,
                        btnDown_CROSS_use_DP3,
                        btnDown_CROSS_DP3_open_start_delay,
                        btnDown_CROSS_DP3_stay_open_time,
                        btnDown_CROSS_use_DP4,
                        btnDown_CROSS_DP4_open_start_delay,
                        btnDown_CROSS_DP4_stay_open_time,
                        btnDown_CROSS_use_DP5,
                        btnDown_CROSS_DP5_open_start_delay,
                        btnDown_CROSS_DP5_stay_open_time,
                        btnDown_CROSS_use_DP6,
                        btnDown_CROSS_DP6_open_start_delay,
                        btnDown_CROSS_DP6_stay_open_time,
                        btnDown_CROSS_use_DP7,
                        btnDown_CROSS_DP7_open_start_delay,
                        btnDown_CROSS_DP7_stay_open_time,
                        btnDown_CROSS_use_DP8,
                        btnDown_CROSS_DP8_open_start_delay,
                        btnDown_CROSS_DP8_stay_open_time,
                        btnDown_CROSS_use_DP9,
                        btnDown_CROSS_DP9_open_start_delay,
                        btnDown_CROSS_DP9_stay_open_time,
                        btnDown_CROSS_use_DP10,
                        btnDown_CROSS_DP10_open_start_delay,
                        btnDown_CROSS_DP10_stay_open_time);

#ifdef SHADOW_VERBOSE
    Serial.print("FOOT: btnDown_CROSS");
#endif


    return;

  }

  if (((!PS3NavDome->PS3NavigationConnected && PS3NavFoot->getButtonPress(LEFT) && PS3NavFoot->getButtonPress(CROSS)) || (PS3NavDome->PS3NavigationConnected && PS3NavFoot->getButtonPress(LEFT) && PS3NavDome->getButtonPress(CROSS))) && marcDuinoButtonCounter == 1)
  {

    marcDuinoButtonPush(btnLeft_CROSS_type, btnLeft_CROSS_MD_func, btnLeft_CROSS_cust_MP3_num, btnLeft_CROSS_cust_LD_type, btnLeft_CROSS_cust_LD_text, btnLeft_CROSS_cust_panel,
                        btnLeft_CROSS_use_DP1,
                        btnLeft_CROSS_DP1_open_start_delay,
                        btnLeft_CROSS_DP1_stay_open_time,
                        btnLeft_CROSS_use_DP2,
                        btnLeft_CROSS_DP2_open_start_delay,
                        btnLeft_CROSS_DP2_stay_open_time,
                        btnLeft_CROSS_use_DP3,
                        btnLeft_CROSS_DP3_open_start_delay,
                        btnLeft_CROSS_DP3_stay_open_time,
                        btnLeft_CROSS_use_DP4,
                        btnLeft_CROSS_DP4_open_start_delay,
                        btnLeft_CROSS_DP4_stay_open_time,
                        btnLeft_CROSS_use_DP5,
                        btnLeft_CROSS_DP5_open_start_delay,
                        btnLeft_CROSS_DP5_stay_open_time,
                        btnLeft_CROSS_use_DP6,
                        btnLeft_CROSS_DP6_open_start_delay,
                        btnLeft_CROSS_DP6_stay_open_time,
                        btnLeft_CROSS_use_DP7,
                        btnLeft_CROSS_DP7_open_start_delay,
                        btnLeft_CROSS_DP7_stay_open_time,
                        btnLeft_CROSS_use_DP8,
                        btnLeft_CROSS_DP8_open_start_delay,
                        btnLeft_CROSS_DP8_stay_open_time,
                        btnLeft_CROSS_use_DP9,
                        btnLeft_CROSS_DP9_open_start_delay,
                        btnLeft_CROSS_DP9_stay_open_time,
                        btnLeft_CROSS_use_DP10,
                        btnLeft_CROSS_DP10_open_start_delay,
                        btnLeft_CROSS_DP10_stay_open_time);

#ifdef SHADOW_VERBOSE
    Serial.print("FOOT: btnLeft_CROSS");
#endif


    return;

  }

  if (((!PS3NavDome->PS3NavigationConnected && PS3NavFoot->getButtonPress(RIGHT) && PS3NavFoot->getButtonPress(CROSS)) || (PS3NavDome->PS3NavigationConnected && PS3NavFoot->getButtonPress(RIGHT) && PS3NavDome->getButtonPress(CROSS))) && marcDuinoButtonCounter == 1)
  {

    marcDuinoButtonPush(btnRight_CROSS_type, btnRight_CROSS_MD_func, btnRight_CROSS_cust_MP3_num, btnRight_CROSS_cust_LD_type, btnRight_CROSS_cust_LD_text, btnRight_CROSS_cust_panel,
                        btnRight_CROSS_use_DP1,
                        btnRight_CROSS_DP1_open_start_delay,
                        btnRight_CROSS_DP1_stay_open_time,
                        btnRight_CROSS_use_DP2,
                        btnRight_CROSS_DP2_open_start_delay,
                        btnRight_CROSS_DP2_stay_open_time,
                        btnRight_CROSS_use_DP3,
                        btnRight_CROSS_DP3_open_start_delay,
                        btnRight_CROSS_DP3_stay_open_time,
                        btnRight_CROSS_use_DP4,
                        btnRight_CROSS_DP4_open_start_delay,
                        btnRight_CROSS_DP4_stay_open_time,
                        btnRight_CROSS_use_DP5,
                        btnRight_CROSS_DP5_open_start_delay,
                        btnRight_CROSS_DP5_stay_open_time,
                        btnRight_CROSS_use_DP6,
                        btnRight_CROSS_DP6_open_start_delay,
                        btnRight_CROSS_DP6_stay_open_time,
                        btnRight_CROSS_use_DP7,
                        btnRight_CROSS_DP7_open_start_delay,
                        btnRight_CROSS_DP7_stay_open_time,
                        btnRight_CROSS_use_DP8,
                        btnRight_CROSS_DP8_open_start_delay,
                        btnRight_CROSS_DP8_stay_open_time,
                        btnRight_CROSS_use_DP9,
                        btnRight_CROSS_DP9_open_start_delay,
                        btnRight_CROSS_DP9_stay_open_time,
                        btnRight_CROSS_use_DP10,
                        btnRight_CROSS_DP10_open_start_delay,
                        btnRight_CROSS_DP10_stay_open_time);

#ifdef SHADOW_VERBOSE
    Serial.print("FOOT: btnRight_CROSS");
#endif


    return;

  }

  //------------------------------------
  // Send triggers for the CIRCLE + base buttons
  //------------------------------------
  if (((!PS3NavDome->PS3NavigationConnected && PS3NavFoot->getButtonPress(UP) && PS3NavFoot->getButtonPress(CIRCLE)) || (PS3NavDome->PS3NavigationConnected && PS3NavFoot->getButtonPress(UP) && PS3NavDome->getButtonPress(CIRCLE))) && marcDuinoButtonCounter == 1)
  {

    marcDuinoButtonPush(btnUP_CIRCLE_type, btnUP_CIRCLE_MD_func, btnUP_CIRCLE_cust_MP3_num, btnUP_CIRCLE_cust_LD_type, btnUP_CIRCLE_cust_LD_text, btnUP_CIRCLE_cust_panel,
                        btnUP_CIRCLE_use_DP1,
                        btnUP_CIRCLE_DP1_open_start_delay,
                        btnUP_CIRCLE_DP1_stay_open_time,
                        btnUP_CIRCLE_use_DP2,
                        btnUP_CIRCLE_DP2_open_start_delay,
                        btnUP_CIRCLE_DP2_stay_open_time,
                        btnUP_CIRCLE_use_DP3,
                        btnUP_CIRCLE_DP3_open_start_delay,
                        btnUP_CIRCLE_DP3_stay_open_time,
                        btnUP_CIRCLE_use_DP4,
                        btnUP_CIRCLE_DP4_open_start_delay,
                        btnUP_CIRCLE_DP4_stay_open_time,
                        btnUP_CIRCLE_use_DP5,
                        btnUP_CIRCLE_DP5_open_start_delay,
                        btnUP_CIRCLE_DP5_stay_open_time,
                        btnUP_CIRCLE_use_DP6,
                        btnUP_CIRCLE_DP6_open_start_delay,
                        btnUP_CIRCLE_DP6_stay_open_time,
                        btnUP_CIRCLE_use_DP7,
                        btnUP_CIRCLE_DP7_open_start_delay,
                        btnUP_CIRCLE_DP7_stay_open_time,
                        btnUP_CIRCLE_use_DP8,
                        btnUP_CIRCLE_DP8_open_start_delay,
                        btnUP_CIRCLE_DP8_stay_open_time,
                        btnUP_CIRCLE_use_DP9,
                        btnUP_CIRCLE_DP9_open_start_delay,
                        btnUP_CIRCLE_DP9_stay_open_time,
                        btnUP_CIRCLE_use_DP10,
                        btnUP_CIRCLE_DP10_open_start_delay,
                        btnUP_CIRCLE_DP10_stay_open_time);

#ifdef SHADOW_VERBOSE
    Serial.print("FOOT: btnUP_CIRCLE");
#endif


    return;

  }

  if (((!PS3NavDome->PS3NavigationConnected && PS3NavFoot->getButtonPress(DOWN) && PS3NavFoot->getButtonPress(CIRCLE)) || (PS3NavDome->PS3NavigationConnected && PS3NavFoot->getButtonPress(DOWN) && PS3NavDome->getButtonPress(CIRCLE))) && marcDuinoButtonCounter == 1)
  {

    marcDuinoButtonPush(btnDown_CIRCLE_type, btnDown_CIRCLE_MD_func, btnDown_CIRCLE_cust_MP3_num, btnDown_CIRCLE_cust_LD_type, btnDown_CIRCLE_cust_LD_text, btnDown_CIRCLE_cust_panel,
                        btnDown_CIRCLE_use_DP1,
                        btnDown_CIRCLE_DP1_open_start_delay,
                        btnDown_CIRCLE_DP1_stay_open_time,
                        btnDown_CIRCLE_use_DP2,
                        btnDown_CIRCLE_DP2_open_start_delay,
                        btnDown_CIRCLE_DP2_stay_open_time,
                        btnDown_CIRCLE_use_DP3,
                        btnDown_CIRCLE_DP3_open_start_delay,
                        btnDown_CIRCLE_DP3_stay_open_time,
                        btnDown_CIRCLE_use_DP4,
                        btnDown_CIRCLE_DP4_open_start_delay,
                        btnDown_CIRCLE_DP4_stay_open_time,
                        btnDown_CIRCLE_use_DP5,
                        btnDown_CIRCLE_DP5_open_start_delay,
                        btnDown_CIRCLE_DP5_stay_open_time,
                        btnDown_CIRCLE_use_DP6,
                        btnDown_CIRCLE_DP6_open_start_delay,
                        btnDown_CIRCLE_DP6_stay_open_time,
                        btnDown_CIRCLE_use_DP7,
                        btnDown_CIRCLE_DP7_open_start_delay,
                        btnDown_CIRCLE_DP7_stay_open_time,
                        btnDown_CIRCLE_use_DP8,
                        btnDown_CIRCLE_DP8_open_start_delay,
                        btnDown_CIRCLE_DP8_stay_open_time,
                        btnDown_CIRCLE_use_DP9,
                        btnDown_CIRCLE_DP9_open_start_delay,
                        btnDown_CIRCLE_DP9_stay_open_time,
                        btnDown_CIRCLE_use_DP10,
                        btnDown_CIRCLE_DP10_open_start_delay,
                        btnDown_CIRCLE_DP10_stay_open_time);

#ifdef SHADOW_VERBOSE
    Serial.print("FOOT: btnDown_CIRCLE");
#endif


    return;

  }

  if (((!PS3NavDome->PS3NavigationConnected && PS3NavFoot->getButtonPress(LEFT) && PS3NavFoot->getButtonPress(CIRCLE)) || (PS3NavDome->PS3NavigationConnected && PS3NavFoot->getButtonPress(LEFT) && PS3NavDome->getButtonPress(CIRCLE))) && marcDuinoButtonCounter == 1)
  {

    marcDuinoButtonPush(btnLeft_CIRCLE_type, btnLeft_CIRCLE_MD_func, btnLeft_CIRCLE_cust_MP3_num, btnLeft_CIRCLE_cust_LD_type, btnLeft_CIRCLE_cust_LD_text, btnLeft_CIRCLE_cust_panel,
                        btnLeft_CIRCLE_use_DP1,
                        btnLeft_CIRCLE_DP1_open_start_delay,
                        btnLeft_CIRCLE_DP1_stay_open_time,
                        btnLeft_CIRCLE_use_DP2,
                        btnLeft_CIRCLE_DP2_open_start_delay,
                        btnLeft_CIRCLE_DP2_stay_open_time,
                        btnLeft_CIRCLE_use_DP3,
                        btnLeft_CIRCLE_DP3_open_start_delay,
                        btnLeft_CIRCLE_DP3_stay_open_time,
                        btnLeft_CIRCLE_use_DP4,
                        btnLeft_CIRCLE_DP4_open_start_delay,
                        btnLeft_CIRCLE_DP4_stay_open_time,
                        btnLeft_CIRCLE_use_DP5,
                        btnLeft_CIRCLE_DP5_open_start_delay,
                        btnLeft_CIRCLE_DP5_stay_open_time,
                        btnLeft_CIRCLE_use_DP6,
                        btnLeft_CIRCLE_DP6_open_start_delay,
                        btnLeft_CIRCLE_DP6_stay_open_time,
                        btnLeft_CIRCLE_use_DP7,
                        btnLeft_CIRCLE_DP7_open_start_delay,
                        btnLeft_CIRCLE_DP7_stay_open_time,
                        btnLeft_CIRCLE_use_DP8,
                        btnLeft_CIRCLE_DP8_open_start_delay,
                        btnLeft_CIRCLE_DP8_stay_open_time,
                        btnLeft_CIRCLE_use_DP9,
                        btnLeft_CIRCLE_DP9_open_start_delay,
                        btnLeft_CIRCLE_DP9_stay_open_time,
                        btnLeft_CIRCLE_use_DP10,
                        btnLeft_CIRCLE_DP10_open_start_delay,
                        btnLeft_CIRCLE_DP10_stay_open_time);

#ifdef SHADOW_VERBOSE
    Serial.print("FOOT: btnLeft_CIRCLE");
#endif


    return;

  }

  if (((!PS3NavDome->PS3NavigationConnected && PS3NavFoot->getButtonPress(RIGHT) && PS3NavFoot->getButtonPress(CIRCLE)) || (PS3NavDome->PS3NavigationConnected && PS3NavFoot->getButtonPress(RIGHT) && PS3NavDome->getButtonPress(CIRCLE))) && marcDuinoButtonCounter == 1)
  {

    marcDuinoButtonPush(btnRight_CIRCLE_type, btnRight_CIRCLE_MD_func, btnRight_CIRCLE_cust_MP3_num, btnRight_CIRCLE_cust_LD_type, btnRight_CIRCLE_cust_LD_text, btnRight_CIRCLE_cust_panel,
                        btnRight_CIRCLE_use_DP1,
                        btnRight_CIRCLE_DP1_open_start_delay,
                        btnRight_CIRCLE_DP1_stay_open_time,
                        btnRight_CIRCLE_use_DP2,
                        btnRight_CIRCLE_DP2_open_start_delay,
                        btnRight_CIRCLE_DP2_stay_open_time,
                        btnRight_CIRCLE_use_DP3,
                        btnRight_CIRCLE_DP3_open_start_delay,
                        btnRight_CIRCLE_DP3_stay_open_time,
                        btnRight_CIRCLE_use_DP4,
                        btnRight_CIRCLE_DP4_open_start_delay,
                        btnRight_CIRCLE_DP4_stay_open_time,
                        btnRight_CIRCLE_use_DP5,
                        btnRight_CIRCLE_DP5_open_start_delay,
                        btnRight_CIRCLE_DP5_stay_open_time,
                        btnRight_CIRCLE_use_DP6,
                        btnRight_CIRCLE_DP6_open_start_delay,
                        btnRight_CIRCLE_DP6_stay_open_time,
                        btnRight_CIRCLE_use_DP7,
                        btnRight_CIRCLE_DP7_open_start_delay,
                        btnRight_CIRCLE_DP7_stay_open_time,
                        btnRight_CIRCLE_use_DP8,
                        btnRight_CIRCLE_DP8_open_start_delay,
                        btnRight_CIRCLE_DP8_stay_open_time,
                        btnRight_CIRCLE_use_DP9,
                        btnRight_CIRCLE_DP9_open_start_delay,
                        btnRight_CIRCLE_DP9_stay_open_time,
                        btnRight_CIRCLE_use_DP10,
                        btnRight_CIRCLE_DP10_open_start_delay,
                        btnRight_CIRCLE_DP10_stay_open_time);


#ifdef SHADOW_VERBOSE
    Serial.print("FOOT: btnRight_CIRCLE");
#endif


    return;

  }

  //------------------------------------
  // Send triggers for the L1 + base buttons
  //------------------------------------
  if (PS3NavFoot->getButtonPress(UP) && PS3NavFoot->getButtonPress(L1) && marcDuinoButtonCounter == 1)
  {

    marcDuinoButtonPush(btnUP_L1_type, btnUP_L1_MD_func, btnUP_L1_cust_MP3_num, btnUP_L1_cust_LD_type, btnUP_L1_cust_LD_text, btnUP_L1_cust_panel,
                        btnUP_L1_use_DP1,
                        btnUP_L1_DP1_open_start_delay,
                        btnUP_L1_DP1_stay_open_time,
                        btnUP_L1_use_DP2,
                        btnUP_L1_DP2_open_start_delay,
                        btnUP_L1_DP2_stay_open_time,
                        btnUP_L1_use_DP3,
                        btnUP_L1_DP3_open_start_delay,
                        btnUP_L1_DP3_stay_open_time,
                        btnUP_L1_use_DP4,
                        btnUP_L1_DP4_open_start_delay,
                        btnUP_L1_DP4_stay_open_time,
                        btnUP_L1_use_DP5,
                        btnUP_L1_DP5_open_start_delay,
                        btnUP_L1_DP5_stay_open_time,
                        btnUP_L1_use_DP6,
                        btnUP_L1_DP6_open_start_delay,
                        btnUP_L1_DP6_stay_open_time,
                        btnUP_L1_use_DP7,
                        btnUP_L1_DP7_open_start_delay,
                        btnUP_L1_DP7_stay_open_time,
                        btnUP_L1_use_DP8,
                        btnUP_L1_DP8_open_start_delay,
                        btnUP_L1_DP8_stay_open_time,
                        btnUP_L1_use_DP9,
                        btnUP_L1_DP9_open_start_delay,
                        btnUP_L1_DP9_stay_open_time,
                        btnUP_L1_use_DP10,
                        btnUP_L1_DP10_open_start_delay,
                        btnUP_L1_DP10_stay_open_time);

#ifdef SHADOW_VERBOSE
    Serial.print("FOOT: btnUP_L1");
#endif


    return;

  }

  if (PS3NavFoot->getButtonPress(DOWN) && PS3NavFoot->getButtonPress(L1) && marcDuinoButtonCounter == 1)
  {

    marcDuinoButtonPush(btnDown_L1_type, btnDown_L1_MD_func, btnDown_L1_cust_MP3_num, btnDown_L1_cust_LD_type, btnDown_L1_cust_LD_text, btnDown_L1_cust_panel,
                        btnDown_L1_use_DP1,
                        btnDown_L1_DP1_open_start_delay,
                        btnDown_L1_DP1_stay_open_time,
                        btnDown_L1_use_DP2,
                        btnDown_L1_DP2_open_start_delay,
                        btnDown_L1_DP2_stay_open_time,
                        btnDown_L1_use_DP3,
                        btnDown_L1_DP3_open_start_delay,
                        btnDown_L1_DP3_stay_open_time,
                        btnDown_L1_use_DP4,
                        btnDown_L1_DP4_open_start_delay,
                        btnDown_L1_DP4_stay_open_time,
                        btnDown_L1_use_DP5,
                        btnDown_L1_DP5_open_start_delay,
                        btnDown_L1_DP5_stay_open_time,
                        btnDown_L1_use_DP6,
                        btnDown_L1_DP6_open_start_delay,
                        btnDown_L1_DP6_stay_open_time,
                        btnDown_L1_use_DP7,
                        btnDown_L1_DP7_open_start_delay,
                        btnDown_L1_DP7_stay_open_time,
                        btnDown_L1_use_DP8,
                        btnDown_L1_DP8_open_start_delay,
                        btnDown_L1_DP8_stay_open_time,
                        btnDown_L1_use_DP9,
                        btnDown_L1_DP9_open_start_delay,
                        btnDown_L1_DP9_stay_open_time,
                        btnDown_L1_use_DP10,
                        btnDown_L1_DP10_open_start_delay,
                        btnDown_L1_DP10_stay_open_time);

#ifdef SHADOW_VERBOSE
    Serial.print("FOOT: btnDown_L1");
#endif


    return;

  }

  if (PS3NavFoot->getButtonPress(LEFT) && PS3NavFoot->getButtonPress(L1) && marcDuinoButtonCounter == 1)
  {

    marcDuinoButtonPush(btnLeft_L1_type, btnLeft_L1_MD_func, btnLeft_L1_cust_MP3_num, btnLeft_L1_cust_LD_type, btnLeft_L1_cust_LD_text, btnLeft_L1_cust_panel,
                        btnLeft_L1_use_DP1,
                        btnLeft_L1_DP1_open_start_delay,
                        btnLeft_L1_DP1_stay_open_time,
                        btnLeft_L1_use_DP2,
                        btnLeft_L1_DP2_open_start_delay,
                        btnLeft_L1_DP2_stay_open_time,
                        btnLeft_L1_use_DP3,
                        btnLeft_L1_DP3_open_start_delay,
                        btnLeft_L1_DP3_stay_open_time,
                        btnLeft_L1_use_DP4,
                        btnLeft_L1_DP4_open_start_delay,
                        btnLeft_L1_DP4_stay_open_time,
                        btnLeft_L1_use_DP5,
                        btnLeft_L1_DP5_open_start_delay,
                        btnLeft_L1_DP5_stay_open_time,
                        btnLeft_L1_use_DP6,
                        btnLeft_L1_DP6_open_start_delay,
                        btnLeft_L1_DP6_stay_open_time,
                        btnLeft_L1_use_DP7,
                        btnLeft_L1_DP7_open_start_delay,
                        btnLeft_L1_DP7_stay_open_time,
                        btnLeft_L1_use_DP8,
                        btnLeft_L1_DP8_open_start_delay,
                        btnLeft_L1_DP8_stay_open_time,
                        btnLeft_L1_use_DP9,
                        btnLeft_L1_DP9_open_start_delay,
                        btnLeft_L1_DP9_stay_open_time,
                        btnLeft_L1_use_DP10,
                        btnLeft_L1_DP10_open_start_delay,
                        btnLeft_L1_DP10_stay_open_time);

#ifdef SHADOW_VERBOSE
    Serial.print("FOOT: btnLeft_L1");
#endif


    return;

  }

  if (PS3NavFoot->getButtonPress(RIGHT) && PS3NavFoot->getButtonPress(L1) && marcDuinoButtonCounter == 1)
  {

    marcDuinoButtonPush(btnRight_L1_type, btnRight_L1_MD_func, btnRight_L1_cust_MP3_num, btnRight_L1_cust_LD_type, btnRight_L1_cust_LD_text, btnRight_L1_cust_panel,
                        btnRight_L1_use_DP1,
                        btnRight_L1_DP1_open_start_delay,
                        btnRight_L1_DP1_stay_open_time,
                        btnRight_L1_use_DP2,
                        btnRight_L1_DP2_open_start_delay,
                        btnRight_L1_DP2_stay_open_time,
                        btnRight_L1_use_DP3,
                        btnRight_L1_DP3_open_start_delay,
                        btnRight_L1_DP3_stay_open_time,
                        btnRight_L1_use_DP4,
                        btnRight_L1_DP4_open_start_delay,
                        btnRight_L1_DP4_stay_open_time,
                        btnRight_L1_use_DP5,
                        btnRight_L1_DP5_open_start_delay,
                        btnRight_L1_DP5_stay_open_time,
                        btnRight_L1_use_DP6,
                        btnRight_L1_DP6_open_start_delay,
                        btnRight_L1_DP6_stay_open_time,
                        btnRight_L1_use_DP7,
                        btnRight_L1_DP7_open_start_delay,
                        btnRight_L1_DP7_stay_open_time,
                        btnRight_L1_use_DP8,
                        btnRight_L1_DP8_open_start_delay,
                        btnRight_L1_DP8_stay_open_time,
                        btnRight_L1_use_DP9,
                        btnRight_L1_DP9_open_start_delay,
                        btnRight_L1_DP9_stay_open_time,
                        btnRight_L1_use_DP10,
                        btnRight_L1_DP10_open_start_delay,
                        btnRight_L1_DP10_stay_open_time);

#ifdef SHADOW_VERBOSE
    Serial.print("FOOT: btnRight_L1");
#endif


    return;

  }

  //------------------------------------
  // Send triggers for the PS + base buttons
  //------------------------------------
  if (((!PS3NavDome->PS3NavigationConnected && PS3NavFoot->getButtonPress(UP) && PS3NavFoot->getButtonPress(PS)) || (PS3NavDome->PS3NavigationConnected && PS3NavFoot->getButtonPress(UP) && PS3NavDome->getButtonPress(PS))) && marcDuinoButtonCounter == 1)
  {

    marcDuinoButtonPush(btnUP_PS_type, btnUP_PS_MD_func, btnUP_PS_cust_MP3_num, btnUP_PS_cust_LD_type, btnUP_PS_cust_LD_text, btnUP_PS_cust_panel,
                        btnUP_PS_use_DP1,
                        btnUP_PS_DP1_open_start_delay,
                        btnUP_PS_DP1_stay_open_time,
                        btnUP_PS_use_DP2,
                        btnUP_PS_DP2_open_start_delay,
                        btnUP_PS_DP2_stay_open_time,
                        btnUP_PS_use_DP3,
                        btnUP_PS_DP3_open_start_delay,
                        btnUP_PS_DP3_stay_open_time,
                        btnUP_PS_use_DP4,
                        btnUP_PS_DP4_open_start_delay,
                        btnUP_PS_DP4_stay_open_time,
                        btnUP_PS_use_DP5,
                        btnUP_PS_DP5_open_start_delay,
                        btnUP_PS_DP5_stay_open_time,
                        btnUP_PS_use_DP6,
                        btnUP_PS_DP6_open_start_delay,
                        btnUP_PS_DP6_stay_open_time,
                        btnUP_PS_use_DP7,
                        btnUP_PS_DP7_open_start_delay,
                        btnUP_PS_DP7_stay_open_time,
                        btnUP_PS_use_DP8,
                        btnUP_PS_DP8_open_start_delay,
                        btnUP_PS_DP8_stay_open_time,
                        btnUP_PS_use_DP9,
                        btnUP_PS_DP9_open_start_delay,
                        btnUP_PS_DP9_stay_open_time,
                        btnUP_PS_use_DP10,
                        btnUP_PS_DP10_open_start_delay,
                        btnUP_PS_DP10_stay_open_time);

#ifdef SHADOW_VERBOSE
    Serial.print("FOOT: btnUP_PS");
#endif


    return;

  }

  if (((!PS3NavDome->PS3NavigationConnected && PS3NavFoot->getButtonPress(DOWN) && PS3NavFoot->getButtonPress(PS)) || (PS3NavDome->PS3NavigationConnected && PS3NavFoot->getButtonPress(DOWN) && PS3NavDome->getButtonPress(PS))) && marcDuinoButtonCounter == 1)
  {

    marcDuinoButtonPush(btnDown_PS_type, btnDown_PS_MD_func, btnDown_PS_cust_MP3_num, btnDown_PS_cust_LD_type, btnDown_PS_cust_LD_text, btnDown_PS_cust_panel,
                        btnDown_PS_use_DP1,
                        btnDown_PS_DP1_open_start_delay,
                        btnDown_PS_DP1_stay_open_time,
                        btnDown_PS_use_DP2,
                        btnDown_PS_DP2_open_start_delay,
                        btnDown_PS_DP2_stay_open_time,
                        btnDown_PS_use_DP3,
                        btnDown_PS_DP3_open_start_delay,
                        btnDown_PS_DP3_stay_open_time,
                        btnDown_PS_use_DP4,
                        btnDown_PS_DP4_open_start_delay,
                        btnDown_PS_DP4_stay_open_time,
                        btnDown_PS_use_DP5,
                        btnDown_PS_DP5_open_start_delay,
                        btnDown_PS_DP5_stay_open_time,
                        btnDown_PS_use_DP6,
                        btnDown_PS_DP6_open_start_delay,
                        btnDown_PS_DP6_stay_open_time,
                        btnDown_PS_use_DP7,
                        btnDown_PS_DP7_open_start_delay,
                        btnDown_PS_DP7_stay_open_time,
                        btnDown_PS_use_DP8,
                        btnDown_PS_DP8_open_start_delay,
                        btnDown_PS_DP8_stay_open_time,
                        btnDown_PS_use_DP9,
                        btnDown_PS_DP9_open_start_delay,
                        btnDown_PS_DP9_stay_open_time,
                        btnDown_PS_use_DP10,
                        btnDown_PS_DP10_open_start_delay,
                        btnDown_PS_DP10_stay_open_time);

#ifdef SHADOW_VERBOSE
    Serial.print("FOOT: btnDown_PS");
#endif


    return;

  }

  if (((!PS3NavDome->PS3NavigationConnected && PS3NavFoot->getButtonPress(LEFT) && PS3NavFoot->getButtonPress(PS)) || (PS3NavDome->PS3NavigationConnected && PS3NavFoot->getButtonPress(LEFT) && PS3NavDome->getButtonPress(PS))) && marcDuinoButtonCounter == 1)
  {

    marcDuinoButtonPush(btnLeft_PS_type, btnLeft_PS_MD_func, btnLeft_PS_cust_MP3_num, btnLeft_PS_cust_LD_type, btnLeft_PS_cust_LD_text, btnLeft_PS_cust_panel,
                        btnLeft_PS_use_DP1,
                        btnLeft_PS_DP1_open_start_delay,
                        btnLeft_PS_DP1_stay_open_time,
                        btnLeft_PS_use_DP2,
                        btnLeft_PS_DP2_open_start_delay,
                        btnLeft_PS_DP2_stay_open_time,
                        btnLeft_PS_use_DP3,
                        btnLeft_PS_DP3_open_start_delay,
                        btnLeft_PS_DP3_stay_open_time,
                        btnLeft_PS_use_DP4,
                        btnLeft_PS_DP4_open_start_delay,
                        btnLeft_PS_DP4_stay_open_time,
                        btnLeft_PS_use_DP5,
                        btnLeft_PS_DP5_open_start_delay,
                        btnLeft_PS_DP5_stay_open_time,
                        btnLeft_PS_use_DP6,
                        btnLeft_PS_DP6_open_start_delay,
                        btnLeft_PS_DP6_stay_open_time,
                        btnLeft_PS_use_DP7,
                        btnLeft_PS_DP7_open_start_delay,
                        btnLeft_PS_DP7_stay_open_time,
                        btnLeft_PS_use_DP8,
                        btnLeft_PS_DP8_open_start_delay,
                        btnLeft_PS_DP8_stay_open_time,
                        btnLeft_PS_use_DP9,
                        btnLeft_PS_DP9_open_start_delay,
                        btnLeft_PS_DP9_stay_open_time,
                        btnLeft_PS_use_DP10,
                        btnLeft_PS_DP10_open_start_delay,
                        btnLeft_PS_DP10_stay_open_time);

#ifdef SHADOW_VERBOSE
    Serial.print("FOOT: btnLeft_PS");
#endif


    return;

  }

  if (((!PS3NavDome->PS3NavigationConnected && PS3NavFoot->getButtonPress(RIGHT) && PS3NavFoot->getButtonPress(PS)) || (PS3NavDome->PS3NavigationConnected && PS3NavFoot->getButtonPress(RIGHT) && PS3NavDome->getButtonPress(PS))) && marcDuinoButtonCounter == 1)
  {

    marcDuinoButtonPush(btnRight_PS_type, btnRight_PS_MD_func, btnRight_PS_cust_MP3_num, btnRight_PS_cust_LD_type, btnRight_PS_cust_LD_text, btnRight_PS_cust_panel,
                        btnRight_PS_use_DP1,
                        btnRight_PS_DP1_open_start_delay,
                        btnRight_PS_DP1_stay_open_time,
                        btnRight_PS_use_DP2,
                        btnRight_PS_DP2_open_start_delay,
                        btnRight_PS_DP2_stay_open_time,
                        btnRight_PS_use_DP3,
                        btnRight_PS_DP3_open_start_delay,
                        btnRight_PS_DP3_stay_open_time,
                        btnRight_PS_use_DP4,
                        btnRight_PS_DP4_open_start_delay,
                        btnRight_PS_DP4_stay_open_time,
                        btnRight_PS_use_DP5,
                        btnRight_PS_DP5_open_start_delay,
                        btnRight_PS_DP5_stay_open_time,
                        btnRight_PS_use_DP6,
                        btnRight_PS_DP6_open_start_delay,
                        btnRight_PS_DP6_stay_open_time,
                        btnRight_PS_use_DP7,
                        btnRight_PS_DP7_open_start_delay,
                        btnRight_PS_DP7_stay_open_time,
                        btnRight_PS_use_DP8,
                        btnRight_PS_DP8_open_start_delay,
                        btnRight_PS_DP8_stay_open_time,
                        btnRight_PS_use_DP9,
                        btnRight_PS_DP9_open_start_delay,
                        btnRight_PS_DP9_stay_open_time,
                        btnRight_PS_use_DP10,
                        btnRight_PS_DP10_open_start_delay,
                        btnRight_PS_DP10_stay_open_time);

#ifdef SHADOW_VERBOSE
    Serial.print("FOOT: btnRight_PS");
#endif


    return;

  }

}

// =======================================================================================
// This is the main MarcDuino Button Management Function
// =======================================================================================
void marcDuinoButtonPush(int type, int MD_func, int MP3_num, int LD_type, String LD_text, int panel_type,
                         boolean use_DP1,
                         int DP1_str_delay,
                         int DP1_open_time,
                         boolean use_DP2,
                         int DP2_str_delay,
                         int DP2_open_time,
                         boolean use_DP3,
                         int DP3_str_delay,
                         int DP3_open_time,
                         boolean use_DP4,
                         int DP4_str_delay,
                         int DP4_open_time,
                         boolean use_DP5,
                         int DP5_str_delay,
                         int DP5_open_time,
                         boolean use_DP6,
                         int DP6_str_delay,
                         int DP6_open_time,
                         boolean use_DP7,
                         int DP7_str_delay,
                         int DP7_open_time,
                         boolean use_DP8,
                         int DP8_str_delay,
                         int DP8_open_time,
                         boolean use_DP9,
                         int DP9_str_delay,
                         int DP9_open_time,
                         boolean use_DP10,
                         int DP10_str_delay,
                         int DP10_open_time)
{

  if (type == 1)  // Std Marcduino Function Call Configured
  {

    switch (MD_func)
    {
      case 1:
        Serial1.print(":SE00\r");
        break;

      case 2:
        Serial1.print(":SE01\r");
        break;

      case 3:
        Serial1.print(":SE02\r");
        break;

      case 4:
        Serial1.print(":SE03\r");
        break;

      case 5:
        Serial1.print(":SE04\r");
        break;

      case 6:
        Serial1.print(":SE05\r");
        break;

      case 7:
        Serial1.print(":SE06\r");
        break;

      case 8:
        Serial1.print(":SE07\r");
        break;

      case 9:
        Serial1.print(":SE08\r");
        break;

      case 10:
        Serial1.print(":SE09\r");
        break;

      case 11:
        Serial1.print(":SE10\r");
        break;

      case 12:
        Serial1.print(":SE11\r");
        break;

      case 13:
        Serial1.print(":SE13\r");
        break;

      case 14:
        Serial1.print(":SE14\r");
        break;

      case 15:
        Serial1.print(":SE51\r");
        break;

      case 16:
        Serial1.print(":SE52\r");
        break;

      case 17:
        Serial1.print(":SE53\r");
        break;

      case 18:
        Serial1.print(":SE54\r");
        break;

      case 19:
        Serial1.print(":SE55\r");
        break;

      case 20:
        Serial1.print(":SE56\r");
        break;

      case 21:
        Serial1.print(":SE57\r");
        break;

      case 22:
        Serial1.print("*RD00\r");
        break;

      case 23:
        Serial1.print("*ON00\r");
        break;

      case 24:
        Serial1.print("*OF00\r");
        break;

      case 25:
        Serial1.print("*ST00\r");
        break;

      case 26:
        Serial1.print("$+\r");
        break;

      case 27:
        Serial1.print("$-\r");
        break;

      case 28:
        Serial1.print("$f\r");
        break;

      case 29:
        Serial1.print("$m\r");
        break;

      case 30:
        Serial1.print(":OP00\r");
        break;

      case 31:
        Serial1.print(":OP11\r");
        break;

      case 32:
        Serial1.print(":OP12\r");
        break;

      case 33:
        Serial1.print(":CL00\r");
        break;

      case 34:
        Serial1.print(":OP01\r");
        break;

      case 35:
        Serial1.print(":CL01\r");
        break;

      case 36:
        Serial1.print(":OP02\r");
        break;

      case 37:
        Serial1.print(":CL02\r");
        break;

      case 38:
        Serial1.print(":OP03\r");
        break;

      case 39:
        Serial1.print(":CL03\r");
        break;

      case 40:
        Serial1.print(":OP04\r");
        break;

      case 41:
        Serial1.print(":CL04\r");
        break;

      case 42:
        Serial1.print(":OP05\r");
        break;

      case 43:
        Serial1.print(":CL05\r");
        break;

      case 44:
        Serial1.print(":OP06\r");
        break;

      case 45:
        Serial1.print(":CL06\r");
        break;

      case 46:
        Serial1.print(":OP07\r");
        break;

      case 47:
        Serial1.print(":CL07\r");
        break;

      case 48:
        Serial1.print(":OP08\r");
        break;

      case 49:
        Serial1.print(":CL08\r");
        break;

      case 50:
        Serial1.print(":OP09\r");
        break;

      case 51:
        Serial1.print(":CL09\r");
        break;

      case 52:
        Serial1.print(":OP10\r");
        break;

      case 53:
        Serial1.print(":CL10\r");
        break;

      case 54:
        Serial3.print(":OP00\r");
        break;

      case 55:
        Serial3.print(":CL00\r");
        break;

      case 56:
        Serial3.print(":OP01\r");
        break;

      case 57:
        Serial3.print(":CL01\r");
        break;

      case 58:
        Serial3.print(":OP02\r");
        break;

      case 59:
        Serial3.print(":CL02\r");
        break;

      case 60:
        Serial3.print(":OP03\r");
        break;

      case 61:
        Serial3.print(":CL03\r");
        break;

      case 62:
        Serial3.print(":OP04\r");
        break;

      case 63:
        Serial3.print(":CL04\r");
        break;

      case 64:
        Serial3.print(":OP05\r");
        break;

      case 65:
        Serial3.print(":CL05\r");
        break;

      case 66:
        Serial3.print(":OP06\r");
        break;

      case 67:
        Serial3.print(":CL06\r");
        break;

      case 68:
        Serial3.print(":OP07\r");
        break;

      case 69:
        Serial3.print(":CL07\r");
        break;

      case 70:
        Serial3.print(":OP08\r");
        break;

      case 71:
        Serial3.print(":CL08\r");
        break;

      case 72:
        Serial3.print(":OP09\r");
        break;

      case 73:
        Serial3.print(":CL09\r");
        break;

      case 74:
        Serial3.print(":OP10\r");
        break;

      case 75:
        Serial3.print(":CL10\r");
        break;

      case 76:
        Serial3.print("*MO99\r");
        break;

      case 77:
        Serial3.print("*MO00\r");
        break;

      case 78:
        Serial3.print("*MF10\r");
        break;

    }

  }  // End Std Marcduino Function Calls


  if (type == 2) // Custom Button Configuration
  {

    if (MP3_num > 181 && MP3_num < 201) // Valid Custom Sound Range Selected - Play Custom Sound Selection
    {

      switch (MP3_num)
      {

        case 182:
          Serial1.print("$87\r");
          break;

        case 183:
          Serial1.print("$88\r");
          break;

        case 184:
          Serial1.print("$89\r");
          break;

        case 185:
          Serial1.print("$810\r");
          break;

        case 186:
          Serial1.print("$811\r");
          break;

        case 187:
          Serial1.print("$812\r");
          break;
        case 188:
          Serial1.print("$813\r");
          break;

        case 189:
          Serial1.print("$814\r");
          break;

        case 190:
          Serial1.print("$815\r");
          break;

        case 191:
          Serial1.print("$816\r");
          break;

        case 192:
          Serial1.print("$817\r");
          break;

        case 193:
          Serial1.print("$818\r");
          break;

        case 194:
          Serial1.print("$819\r");
          break;

        case 195:
          Serial1.print("$820\r");
          break;

        case 196:
          Serial1.print("$821\r");
          break;

        case 197:
          Serial1.print("$822\r");
          break;

        case 198:
          Serial1.print("$823\r");
          break;

        case 199:
          Serial1.print("$824\r");
          break;

        case 200:
          Serial1.print("$825\r");
          break;

      }

    }

    if (panel_type > 0 && panel_type < 10) // Valid panel type selected - perform custom panel functions
    {

      // Reset the custom panel flags
      DP1_Status = 0;
      DP2_Status = 0;
      DP3_Status = 0;
      DP4_Status = 0;
      DP5_Status = 0;
      DP6_Status = 0;
      DP7_Status = 0;
      DP8_Status = 0;
      DP9_Status = 0;
      DP10_Status = 0;

      if (panel_type > 1)
      {
        Serial1.print(":CL00\r");  // close all the panels prior to next custom routine
        delay(50); // give panel close command time to process before starting next panel command
      }

      switch (panel_type)
      {

        case 1:
          Serial1.print(":CL00\r");
          break;

        case 2:
          Serial1.print(":SE51\r");
          break;

        case 3:
          Serial1.print(":SE52\r");
          break;

        case 4:
          Serial1.print(":SE53\r");
          break;

        case 5:
          Serial1.print(":SE54\r");
          break;

        case 6:
          Serial1.print(":SE55\r");
          break;

        case 7:
          Serial1.print(":SE56\r");
          break;

        case 8:
          Serial1.print(":SE57\r");
          break;

        case 9: // This is the setup section for the custom panel routines

          runningCustRoutine = true;

          // Configure Dome Panel #1
          if (use_DP1)
          {

            DP1_Status = 1;
            DP1_start = millis();

            if (DP1_str_delay < 31)
            {

              DP1_s_delay = DP1_str_delay;

            } else
            {
              DP1_Status = 0;
            }

            if (DP1_open_time > 0 && DP1_open_time < 31)
            {

              DP1_o_time = DP1_open_time;

            } else
            {
              DP1_Status = 0;
            }

          }

          // Configure Dome Panel #2
          if (use_DP2)
          {

            DP2_Status = 1;
            DP2_start = millis();

            if (DP2_str_delay < 31)
            {

              DP2_s_delay = DP2_str_delay;

            } else
            {
              DP2_Status = 0;
            }

            if (DP2_open_time > 0 && DP2_open_time < 31)
            {

              DP2_o_time = DP2_open_time;

            } else
            {
              DP2_Status = 0;
            }

          }


          // Configure Dome Panel #3
          if (use_DP3)
          {

            DP3_Status = 1;
            DP3_start = millis();

            if (DP3_str_delay < 31)
            {

              DP3_s_delay = DP3_str_delay;

            } else
            {
              DP3_Status = 0;
            }

            if (DP3_open_time > 0 && DP3_open_time < 31)
            {

              DP3_o_time = DP3_open_time;

            } else
            {
              DP3_Status = 0;
            }

          }

          // Configure Dome Panel #4
          if (use_DP4)
          {

            DP4_Status = 1;
            DP4_start = millis();

            if (DP4_str_delay < 31)
            {

              DP4_s_delay = DP4_str_delay;

            } else
            {
              DP4_Status = 0;
            }

            if (DP4_open_time > 0 && DP4_open_time < 31)
            {

              DP4_o_time = DP4_open_time;

            } else
            {
              DP4_Status = 0;
            }

          }

          // Configure Dome Panel #5
          if (use_DP5)
          {

            DP5_Status = 1;
            DP5_start = millis();

            if (DP5_str_delay < 31)
            {

              DP5_s_delay = DP5_str_delay;

            } else
            {
              DP5_Status = 0;
            }

            if (DP5_open_time > 0 && DP5_open_time < 31)
            {

              DP5_o_time = DP5_open_time;

            } else
            {
              DP5_Status = 0;
            }

          }

          // Configure Dome Panel #6
          if (use_DP6)
          {

            DP6_Status = 1;
            DP6_start = millis();

            if (DP6_str_delay < 31)
            {

              DP6_s_delay = DP6_str_delay;

            } else
            {
              DP6_Status = 0;
            }

            if (DP6_open_time > 0 && DP6_open_time < 31)
            {

              DP6_o_time = DP6_open_time;

            } else
            {
              DP6_Status = 0;
            }

          }

          // Configure Dome Panel #7
          if (use_DP7)
          {

            DP7_Status = 1;
            DP7_start = millis();

            if (DP7_str_delay < 31)
            {

              DP7_s_delay = DP7_str_delay;

            } else
            {
              DP7_Status = 0;
            }

            if (DP7_open_time > 0 && DP7_open_time < 31)
            {

              DP7_o_time = DP7_open_time;

            } else
            {
              DP7_Status = 0;
            }

          }

          // Configure Dome Panel #8
          if (use_DP8)
          {

            DP8_Status = 1;
            DP8_start = millis();

            if (DP8_str_delay < 31)
            {

              DP8_s_delay = DP8_str_delay;

            } else
            {
              DP8_Status = 0;
            }

            if (DP8_open_time > 0 && DP8_open_time < 31)
            {

              DP8_o_time = DP8_open_time;

            } else
            {
              DP8_Status = 0;
            }

          }

          // Configure Dome Panel #9
          if (use_DP9)
          {

            DP9_Status = 1;
            DP9_start = millis();

            if (DP9_str_delay < 31)
            {

              DP9_s_delay = DP9_str_delay;

            } else
            {
              DP9_Status = 0;
            }

            if (DP9_open_time > 0 && DP9_open_time < 31)
            {

              DP9_o_time = DP9_open_time;

            } else
            {
              DP9_Status = 0;
            }

          }

          // Configure Dome Panel #10
          if (use_DP10)
          {

            DP10_Status = 1;
            DP10_start = millis();

            if (DP10_str_delay < 31)
            {

              DP10_s_delay = DP10_str_delay;

            } else
            {
              DP10_Status = 0;
            }

            if (DP10_open_time > 0 && DP10_open_time < 31)
            {

              DP10_o_time = DP10_open_time;

            } else
            {
              DP10_Status = 0;
            }

          }

          // If every dome panel config failed to work - reset routine flag to false
          if (DP1_Status + DP2_Status + DP3_Status + DP4_Status + DP5_Status + DP6_Status + DP7_Status + DP8_Status + DP9_Status + DP10_Status == 0)
          {

            runningCustRoutine = false;

          }

          break;
      }
    }


    if (LD_type > 0 && LD_type < 9) // Valid Logic Display Selected - Display Custom Logic Display
    {

      if (panel_type > 1 && panel_type < 10)  // If a custom panel movement was selected - need to briefly pause before changing light sequence to avoid conflict)
      {
        delay(30);
      }

      switch (LD_type)
      {

        case 1:
          Serial1.print("@0T1\r");
          break;

        case 2:
          Serial1.print("@0T4\r");
          break;

        case 3:
          Serial1.print("@0T5\r");
          break;

        case 4:
          Serial1.print("@0T6\r");
          break;

        case 5:
          Serial1.print("@0T10\r");
          break;

        case 6:
          Serial1.print("@0T11\r");
          break;

        case 7:
          Serial1.print("@0T92\r");
          break;

        case 8:
          Serial1.print("@0T100\r");
          delay(50);
          String custString = "@0M";
          custString += LD_text;
          custString += "\r";
          Serial1.print(custString);
          break;
      }
    }

  }

}
// ===================================================================================================================
// This function determines if MarcDuino buttons were selected and calls main processing function for DOME Controller
// ===================================================================================================================
void marcDuinoDome()
{
  if (PS3NavDome->PS3NavigationConnected && (PS3NavDome->getButtonPress(UP) || PS3NavDome->getButtonPress(DOWN) || PS3NavDome->getButtonPress(LEFT) || PS3NavDome->getButtonPress(RIGHT)))
  {

    if ((millis() - previousMarcDuinoMillis) > 1000)
    {
      marcDuinoButtonCounter = 0;
      previousMarcDuinoMillis = millis();
    }

    marcDuinoButtonCounter += 1;

  } else
  {
    return;
  }

  // Clear inbound buffer of any data sent from the MarcDuino board
  while (Serial1.available()) Serial1.read();

  //------------------------------------
  // Send triggers for the base buttons
  //------------------------------------
  if (PS3NavDome->getButtonPress(UP) && !PS3NavDome->getButtonPress(CROSS) && !PS3NavDome->getButtonPress(CIRCLE) && !PS3NavDome->getButtonPress(L1) && !PS3NavDome->getButtonPress(PS) && !PS3NavFoot->getButtonPress(CROSS) && !PS3NavFoot->getButtonPress(CIRCLE) && !PS3NavFoot->getButtonPress(PS) && marcDuinoButtonCounter == 1)
  {

    marcDuinoButtonPush(1, FTbtnUP_MD_func, btnUP_cust_MP3_num, btnUP_cust_LD_type, btnUP_cust_LD_text, btnUP_cust_panel,
                        btnUP_use_DP1,
                        btnUP_DP1_open_start_delay,
                        btnUP_DP1_stay_open_time,
                        btnUP_use_DP2,
                        btnUP_DP2_open_start_delay,
                        btnUP_DP2_stay_open_time,
                        btnUP_use_DP3,
                        btnUP_DP3_open_start_delay,
                        btnUP_DP3_stay_open_time,
                        btnUP_use_DP4,
                        btnUP_DP4_open_start_delay,
                        btnUP_DP4_stay_open_time,
                        btnUP_use_DP5,
                        btnUP_DP5_open_start_delay,
                        btnUP_DP5_stay_open_time,
                        btnUP_use_DP6,
                        btnUP_DP6_open_start_delay,
                        btnUP_DP6_stay_open_time,
                        btnUP_use_DP7,
                        btnUP_DP7_open_start_delay,
                        btnUP_DP7_stay_open_time,
                        btnUP_use_DP8,
                        btnUP_DP8_open_start_delay,
                        btnUP_DP8_stay_open_time,
                        btnUP_use_DP9,
                        btnUP_DP9_open_start_delay,
                        btnUP_DP9_stay_open_time,
                        btnUP_use_DP10,
                        btnUP_DP10_open_start_delay,
                        btnUP_DP10_stay_open_time);

#ifdef SHADOW_VERBOSE
    Serial.println("DOME: btnUP");
#endif

    return;

  }

  if (PS3NavDome->getButtonPress(DOWN) && !PS3NavDome->getButtonPress(CROSS) && !PS3NavDome->getButtonPress(CIRCLE) && !PS3NavDome->getButtonPress(L1) && !PS3NavDome->getButtonPress(PS) && !PS3NavFoot->getButtonPress(CROSS) && !PS3NavFoot->getButtonPress(CIRCLE) && !PS3NavFoot->getButtonPress(PS) && marcDuinoButtonCounter == 1)
  {

    marcDuinoButtonPush(1, FTbtnDown_MD_func, btnDown_cust_MP3_num, btnDown_cust_LD_type, btnDown_cust_LD_text, btnDown_cust_panel,
                        btnDown_use_DP1,
                        btnDown_DP1_open_start_delay,
                        btnDown_DP1_stay_open_time,
                        btnDown_use_DP2,
                        btnDown_DP2_open_start_delay,
                        btnDown_DP2_stay_open_time,
                        btnDown_use_DP3,
                        btnDown_DP3_open_start_delay,
                        btnDown_DP3_stay_open_time,
                        btnDown_use_DP4,
                        btnDown_DP4_open_start_delay,
                        btnDown_DP4_stay_open_time,
                        btnDown_use_DP5,
                        btnDown_DP5_open_start_delay,
                        btnDown_DP5_stay_open_time,
                        btnDown_use_DP6,
                        btnDown_DP6_open_start_delay,
                        btnDown_DP6_stay_open_time,
                        btnDown_use_DP7,
                        btnDown_DP7_open_start_delay,
                        btnDown_DP7_stay_open_time,
                        btnDown_use_DP8,
                        btnDown_DP8_open_start_delay,
                        btnDown_DP8_stay_open_time,
                        btnDown_use_DP9,
                        btnDown_DP9_open_start_delay,
                        btnDown_DP9_stay_open_time,
                        btnDown_use_DP10,
                        btnDown_DP10_open_start_delay,
                        btnDown_DP10_stay_open_time);

#ifdef SHADOW_VERBOSE
    Serial.println("DOME: btnDown");
#endif

    return;
  }

  if (PS3NavDome->getButtonPress(LEFT) && !PS3NavDome->getButtonPress(CROSS) && !PS3NavDome->getButtonPress(CIRCLE) && !PS3NavDome->getButtonPress(L1) && !PS3NavDome->getButtonPress(PS) && !PS3NavFoot->getButtonPress(CROSS) && !PS3NavFoot->getButtonPress(CIRCLE) && !PS3NavFoot->getButtonPress(PS) && marcDuinoButtonCounter == 1)
  {

    marcDuinoButtonPush(1, FTbtnLeft_MD_func, btnLeft_cust_MP3_num, btnLeft_cust_LD_type, btnLeft_cust_LD_text, btnLeft_cust_panel,
                        btnLeft_use_DP1,
                        btnLeft_DP1_open_start_delay,
                        btnLeft_DP1_stay_open_time,
                        btnLeft_use_DP2,
                        btnLeft_DP2_open_start_delay,
                        btnLeft_DP2_stay_open_time,
                        btnLeft_use_DP3,
                        btnLeft_DP3_open_start_delay,
                        btnLeft_DP3_stay_open_time,
                        btnLeft_use_DP4,
                        btnLeft_DP4_open_start_delay,
                        btnLeft_DP4_stay_open_time,
                        btnLeft_use_DP5,
                        btnLeft_DP5_open_start_delay,
                        btnLeft_DP5_stay_open_time,
                        btnLeft_use_DP6,
                        btnLeft_DP6_open_start_delay,
                        btnLeft_DP6_stay_open_time,
                        btnLeft_use_DP7,
                        btnLeft_DP7_open_start_delay,
                        btnLeft_DP7_stay_open_time,
                        btnLeft_use_DP8,
                        btnLeft_DP8_open_start_delay,
                        btnLeft_DP8_stay_open_time,
                        btnLeft_use_DP9,
                        btnLeft_DP9_open_start_delay,
                        btnLeft_DP9_stay_open_time,
                        btnLeft_use_DP10,
                        btnLeft_DP10_open_start_delay,
                        btnLeft_DP10_stay_open_time);

#ifdef SHADOW_VERBOSE
    Serial.println("DOME: btnLeft");
#endif

    return;

  }

  if (PS3NavDome->getButtonPress(RIGHT) && !PS3NavDome->getButtonPress(CROSS) && !PS3NavDome->getButtonPress(CIRCLE) && !PS3NavDome->getButtonPress(L1) && !PS3NavDome->getButtonPress(PS) && !PS3NavFoot->getButtonPress(CROSS) && !PS3NavFoot->getButtonPress(CIRCLE) && !PS3NavFoot->getButtonPress(PS) && marcDuinoButtonCounter == 1)
  {

    marcDuinoButtonPush(1, FTbtnRight_MD_func, btnRight_cust_MP3_num, btnRight_cust_LD_type, btnRight_cust_LD_text, btnRight_cust_panel,
                        btnRight_use_DP1,
                        btnRight_DP1_open_start_delay,
                        btnRight_DP1_stay_open_time,
                        btnRight_use_DP2,
                        btnRight_DP2_open_start_delay,
                        btnRight_DP2_stay_open_time,
                        btnRight_use_DP3,
                        btnRight_DP3_open_start_delay,
                        btnRight_DP3_stay_open_time,
                        btnRight_use_DP4,
                        btnRight_DP4_open_start_delay,
                        btnRight_DP4_stay_open_time,
                        btnRight_use_DP5,
                        btnRight_DP5_open_start_delay,
                        btnRight_DP5_stay_open_time,
                        btnRight_use_DP6,
                        btnRight_DP6_open_start_delay,
                        btnRight_DP6_stay_open_time,
                        btnRight_use_DP7,
                        btnRight_DP7_open_start_delay,
                        btnRight_DP7_stay_open_time,
                        btnRight_use_DP8,
                        btnRight_DP8_open_start_delay,
                        btnRight_DP8_stay_open_time,
                        btnRight_use_DP9,
                        btnRight_DP9_open_start_delay,
                        btnRight_DP9_stay_open_time,
                        btnRight_use_DP10,
                        btnRight_DP10_open_start_delay,
                        btnRight_DP10_stay_open_time);

#ifdef SHADOW_VERBOSE
    Serial.println("DOME: btnRight");
#endif


    return;

  }

  //------------------------------------
  // Send triggers for the CROSS + base buttons
  //------------------------------------
  if (PS3NavDome->getButtonPress(UP) && PS3NavFoot->getButtonPress(CROSS) && marcDuinoButtonCounter == 1)
  {

    marcDuinoButtonPush(1, FTbtnUP_CROSS_MD_func, btnUP_CROSS_cust_MP3_num, btnUP_CROSS_cust_LD_type, btnUP_CROSS_cust_LD_text, btnUP_CROSS_cust_panel,
                        btnUP_CROSS_use_DP1,
                        btnUP_CROSS_DP1_open_start_delay,
                        btnUP_CROSS_DP1_stay_open_time,
                        btnUP_CROSS_use_DP2,
                        btnUP_CROSS_DP2_open_start_delay,
                        btnUP_CROSS_DP2_stay_open_time,
                        btnUP_CROSS_use_DP3,
                        btnUP_CROSS_DP3_open_start_delay,
                        btnUP_CROSS_DP3_stay_open_time,
                        btnUP_CROSS_use_DP4,
                        btnUP_CROSS_DP4_open_start_delay,
                        btnUP_CROSS_DP4_stay_open_time,
                        btnUP_CROSS_use_DP5,
                        btnUP_CROSS_DP5_open_start_delay,
                        btnUP_CROSS_DP5_stay_open_time,
                        btnUP_CROSS_use_DP6,
                        btnUP_CROSS_DP6_open_start_delay,
                        btnUP_CROSS_DP6_stay_open_time,
                        btnUP_CROSS_use_DP7,
                        btnUP_CROSS_DP7_open_start_delay,
                        btnUP_CROSS_DP7_stay_open_time,
                        btnUP_CROSS_use_DP8,
                        btnUP_CROSS_DP8_open_start_delay,
                        btnUP_CROSS_DP8_stay_open_time,
                        btnUP_CROSS_use_DP9,
                        btnUP_CROSS_DP9_open_start_delay,
                        btnUP_CROSS_DP9_stay_open_time,
                        btnUP_CROSS_use_DP10,
                        btnUP_CROSS_DP10_open_start_delay,
                        btnUP_CROSS_DP10_stay_open_time);

#ifdef SHADOW_VERBOSE
    Serial.println("DOME: btnUP_CROSS");
#endif


    return;

  }

  if (PS3NavDome->getButtonPress(DOWN) && PS3NavFoot->getButtonPress(CROSS) && marcDuinoButtonCounter == 1)
  {

    marcDuinoButtonPush(1, FTbtnDown_CROSS_MD_func, btnDown_CROSS_cust_MP3_num, btnDown_CROSS_cust_LD_type, btnDown_CROSS_cust_LD_text, btnDown_CROSS_cust_panel,
                        btnDown_CROSS_use_DP1,
                        btnDown_CROSS_DP1_open_start_delay,
                        btnDown_CROSS_DP1_stay_open_time,
                        btnDown_CROSS_use_DP2,
                        btnDown_CROSS_DP2_open_start_delay,
                        btnDown_CROSS_DP2_stay_open_time,
                        btnDown_CROSS_use_DP3,
                        btnDown_CROSS_DP3_open_start_delay,
                        btnDown_CROSS_DP3_stay_open_time,
                        btnDown_CROSS_use_DP4,
                        btnDown_CROSS_DP4_open_start_delay,
                        btnDown_CROSS_DP4_stay_open_time,
                        btnDown_CROSS_use_DP5,
                        btnDown_CROSS_DP5_open_start_delay,
                        btnDown_CROSS_DP5_stay_open_time,
                        btnDown_CROSS_use_DP6,
                        btnDown_CROSS_DP6_open_start_delay,
                        btnDown_CROSS_DP6_stay_open_time,
                        btnDown_CROSS_use_DP7,
                        btnDown_CROSS_DP7_open_start_delay,
                        btnDown_CROSS_DP7_stay_open_time,
                        btnDown_CROSS_use_DP8,
                        btnDown_CROSS_DP8_open_start_delay,
                        btnDown_CROSS_DP8_stay_open_time,
                        btnDown_CROSS_use_DP9,
                        btnDown_CROSS_DP9_open_start_delay,
                        btnDown_CROSS_DP9_stay_open_time,
                        btnDown_CROSS_use_DP10,
                        btnDown_CROSS_DP10_open_start_delay,
                        btnDown_CROSS_DP10_stay_open_time);

#ifdef SHADOW_VERBOSE
    Serial.println("DOME: btnDown_CROSS");
#endif


    return;

  }

  if (PS3NavDome->getButtonPress(LEFT) && PS3NavFoot->getButtonPress(CROSS) && marcDuinoButtonCounter == 1)
  {

    marcDuinoButtonPush(1, FTbtnLeft_CROSS_MD_func, btnLeft_CROSS_cust_MP3_num, btnLeft_CROSS_cust_LD_type, btnLeft_CROSS_cust_LD_text, btnLeft_CROSS_cust_panel,
                        btnLeft_CROSS_use_DP1,
                        btnLeft_CROSS_DP1_open_start_delay,
                        btnLeft_CROSS_DP1_stay_open_time,
                        btnLeft_CROSS_use_DP2,
                        btnLeft_CROSS_DP2_open_start_delay,
                        btnLeft_CROSS_DP2_stay_open_time,
                        btnLeft_CROSS_use_DP3,
                        btnLeft_CROSS_DP3_open_start_delay,
                        btnLeft_CROSS_DP3_stay_open_time,
                        btnLeft_CROSS_use_DP4,
                        btnLeft_CROSS_DP4_open_start_delay,
                        btnLeft_CROSS_DP4_stay_open_time,
                        btnLeft_CROSS_use_DP5,
                        btnLeft_CROSS_DP5_open_start_delay,
                        btnLeft_CROSS_DP5_stay_open_time,
                        btnLeft_CROSS_use_DP6,
                        btnLeft_CROSS_DP6_open_start_delay,
                        btnLeft_CROSS_DP6_stay_open_time,
                        btnLeft_CROSS_use_DP7,
                        btnLeft_CROSS_DP7_open_start_delay,
                        btnLeft_CROSS_DP7_stay_open_time,
                        btnLeft_CROSS_use_DP8,
                        btnLeft_CROSS_DP8_open_start_delay,
                        btnLeft_CROSS_DP8_stay_open_time,
                        btnLeft_CROSS_use_DP9,
                        btnLeft_CROSS_DP9_open_start_delay,
                        btnLeft_CROSS_DP9_stay_open_time,
                        btnLeft_CROSS_use_DP10,
                        btnLeft_CROSS_DP10_open_start_delay,
                        btnLeft_CROSS_DP10_stay_open_time);

#ifdef SHADOW_VERBOSE
    Serial.println("DOME: btnLeft_CROSS");
#endif


    return;

  }

  if (PS3NavDome->getButtonPress(RIGHT) && PS3NavFoot->getButtonPress(CROSS) && marcDuinoButtonCounter == 1)
  {

    marcDuinoButtonPush(1, FTbtnRight_CROSS_MD_func, btnRight_CROSS_cust_MP3_num, btnRight_CROSS_cust_LD_type, btnRight_CROSS_cust_LD_text, btnRight_CROSS_cust_panel,
                        btnRight_CROSS_use_DP1,
                        btnRight_CROSS_DP1_open_start_delay,
                        btnRight_CROSS_DP1_stay_open_time,
                        btnRight_CROSS_use_DP2,
                        btnRight_CROSS_DP2_open_start_delay,
                        btnRight_CROSS_DP2_stay_open_time,
                        btnRight_CROSS_use_DP3,
                        btnRight_CROSS_DP3_open_start_delay,
                        btnRight_CROSS_DP3_stay_open_time,
                        btnRight_CROSS_use_DP4,
                        btnRight_CROSS_DP4_open_start_delay,
                        btnRight_CROSS_DP4_stay_open_time,
                        btnRight_CROSS_use_DP5,
                        btnRight_CROSS_DP5_open_start_delay,
                        btnRight_CROSS_DP5_stay_open_time,
                        btnRight_CROSS_use_DP6,
                        btnRight_CROSS_DP6_open_start_delay,
                        btnRight_CROSS_DP6_stay_open_time,
                        btnRight_CROSS_use_DP7,
                        btnRight_CROSS_DP7_open_start_delay,
                        btnRight_CROSS_DP7_stay_open_time,
                        btnRight_CROSS_use_DP8,
                        btnRight_CROSS_DP8_open_start_delay,
                        btnRight_CROSS_DP8_stay_open_time,
                        btnRight_CROSS_use_DP9,
                        btnRight_CROSS_DP9_open_start_delay,
                        btnRight_CROSS_DP9_stay_open_time,
                        btnRight_CROSS_use_DP10,
                        btnRight_CROSS_DP10_open_start_delay,
                        btnRight_CROSS_DP10_stay_open_time);

#ifdef SHADOW_VERBOSE
    Serial.println("DOME: btnRight_CROSS");
#endif


    return;

  }

  //------------------------------------
  // Send triggers for the CIRCLE + base buttons
  //------------------------------------
  if (PS3NavDome->getButtonPress(UP) && PS3NavFoot->getButtonPress(CIRCLE) && marcDuinoButtonCounter == 1)
  {

    marcDuinoButtonPush(1, FTbtnUP_CIRCLE_MD_func, btnUP_CIRCLE_cust_MP3_num, btnUP_CIRCLE_cust_LD_type, btnUP_CIRCLE_cust_LD_text, btnUP_CIRCLE_cust_panel,
                        btnUP_CIRCLE_use_DP1,
                        btnUP_CIRCLE_DP1_open_start_delay,
                        btnUP_CIRCLE_DP1_stay_open_time,
                        btnUP_CIRCLE_use_DP2,
                        btnUP_CIRCLE_DP2_open_start_delay,
                        btnUP_CIRCLE_DP2_stay_open_time,
                        btnUP_CIRCLE_use_DP3,
                        btnUP_CIRCLE_DP3_open_start_delay,
                        btnUP_CIRCLE_DP3_stay_open_time,
                        btnUP_CIRCLE_use_DP4,
                        btnUP_CIRCLE_DP4_open_start_delay,
                        btnUP_CIRCLE_DP4_stay_open_time,
                        btnUP_CIRCLE_use_DP5,
                        btnUP_CIRCLE_DP5_open_start_delay,
                        btnUP_CIRCLE_DP5_stay_open_time,
                        btnUP_CIRCLE_use_DP6,
                        btnUP_CIRCLE_DP6_open_start_delay,
                        btnUP_CIRCLE_DP6_stay_open_time,
                        btnUP_CIRCLE_use_DP7,
                        btnUP_CIRCLE_DP7_open_start_delay,
                        btnUP_CIRCLE_DP7_stay_open_time,
                        btnUP_CIRCLE_use_DP8,
                        btnUP_CIRCLE_DP8_open_start_delay,
                        btnUP_CIRCLE_DP8_stay_open_time,
                        btnUP_CIRCLE_use_DP9,
                        btnUP_CIRCLE_DP9_open_start_delay,
                        btnUP_CIRCLE_DP9_stay_open_time,
                        btnUP_CIRCLE_use_DP10,
                        btnUP_CIRCLE_DP10_open_start_delay,
                        btnUP_CIRCLE_DP10_stay_open_time);

#ifdef SHADOW_VERBOSE
    Serial.println("DOME: btnUP_CIRCLE");
#endif


    return;

  }

  if (PS3NavDome->getButtonPress(DOWN) && PS3NavFoot->getButtonPress(CIRCLE) && marcDuinoButtonCounter == 1)
  {

    marcDuinoButtonPush(1, FTbtnDown_CIRCLE_MD_func, btnDown_CIRCLE_cust_MP3_num, btnDown_CIRCLE_cust_LD_type, btnDown_CIRCLE_cust_LD_text, btnDown_CIRCLE_cust_panel,
                        btnDown_CIRCLE_use_DP1,
                        btnDown_CIRCLE_DP1_open_start_delay,
                        btnDown_CIRCLE_DP1_stay_open_time,
                        btnDown_CIRCLE_use_DP2,
                        btnDown_CIRCLE_DP2_open_start_delay,
                        btnDown_CIRCLE_DP2_stay_open_time,
                        btnDown_CIRCLE_use_DP3,
                        btnDown_CIRCLE_DP3_open_start_delay,
                        btnDown_CIRCLE_DP3_stay_open_time,
                        btnDown_CIRCLE_use_DP4,
                        btnDown_CIRCLE_DP4_open_start_delay,
                        btnDown_CIRCLE_DP4_stay_open_time,
                        btnDown_CIRCLE_use_DP5,
                        btnDown_CIRCLE_DP5_open_start_delay,
                        btnDown_CIRCLE_DP5_stay_open_time,
                        btnDown_CIRCLE_use_DP6,
                        btnDown_CIRCLE_DP6_open_start_delay,
                        btnDown_CIRCLE_DP6_stay_open_time,
                        btnDown_CIRCLE_use_DP7,
                        btnDown_CIRCLE_DP7_open_start_delay,
                        btnDown_CIRCLE_DP7_stay_open_time,
                        btnDown_CIRCLE_use_DP8,
                        btnDown_CIRCLE_DP8_open_start_delay,
                        btnDown_CIRCLE_DP8_stay_open_time,
                        btnDown_CIRCLE_use_DP9,
                        btnDown_CIRCLE_DP9_open_start_delay,
                        btnDown_CIRCLE_DP9_stay_open_time,
                        btnDown_CIRCLE_use_DP10,
                        btnDown_CIRCLE_DP10_open_start_delay,
                        btnDown_CIRCLE_DP10_stay_open_time);

#ifdef SHADOW_VERBOSE
    Serial.println("DOME: btnDown_CIRCLE");
#endif


    return;

  }

  if (PS3NavDome->getButtonPress(LEFT) && PS3NavFoot->getButtonPress(CIRCLE) && marcDuinoButtonCounter == 1)
  {

    marcDuinoButtonPush(1, FTbtnLeft_CIRCLE_MD_func, btnLeft_CIRCLE_cust_MP3_num, btnLeft_CIRCLE_cust_LD_type, btnLeft_CIRCLE_cust_LD_text, btnLeft_CIRCLE_cust_panel,
                        btnLeft_CIRCLE_use_DP1,
                        btnLeft_CIRCLE_DP1_open_start_delay,
                        btnLeft_CIRCLE_DP1_stay_open_time,
                        btnLeft_CIRCLE_use_DP2,
                        btnLeft_CIRCLE_DP2_open_start_delay,
                        btnLeft_CIRCLE_DP2_stay_open_time,
                        btnLeft_CIRCLE_use_DP3,
                        btnLeft_CIRCLE_DP3_open_start_delay,
                        btnLeft_CIRCLE_DP3_stay_open_time,
                        btnLeft_CIRCLE_use_DP4,
                        btnLeft_CIRCLE_DP4_open_start_delay,
                        btnLeft_CIRCLE_DP4_stay_open_time,
                        btnLeft_CIRCLE_use_DP5,
                        btnLeft_CIRCLE_DP5_open_start_delay,
                        btnLeft_CIRCLE_DP5_stay_open_time,
                        btnLeft_CIRCLE_use_DP6,
                        btnLeft_CIRCLE_DP6_open_start_delay,
                        btnLeft_CIRCLE_DP6_stay_open_time,
                        btnLeft_CIRCLE_use_DP7,
                        btnLeft_CIRCLE_DP7_open_start_delay,
                        btnLeft_CIRCLE_DP7_stay_open_time,
                        btnLeft_CIRCLE_use_DP8,
                        btnLeft_CIRCLE_DP8_open_start_delay,
                        btnLeft_CIRCLE_DP8_stay_open_time,
                        btnLeft_CIRCLE_use_DP9,
                        btnLeft_CIRCLE_DP9_open_start_delay,
                        btnLeft_CIRCLE_DP9_stay_open_time,
                        btnLeft_CIRCLE_use_DP10,
                        btnLeft_CIRCLE_DP10_open_start_delay,
                        btnLeft_CIRCLE_DP10_stay_open_time);

#ifdef SHADOW_VERBOSE
    Serial.println("DOME: btnLeft_CIRCLE");
#endif


    return;

  }

  if (PS3NavDome->getButtonPress(RIGHT) && PS3NavFoot->getButtonPress(CIRCLE) && marcDuinoButtonCounter == 1)
  {

    marcDuinoButtonPush(1, FTbtnRight_CIRCLE_MD_func, btnRight_CIRCLE_cust_MP3_num, btnRight_CIRCLE_cust_LD_type, btnRight_CIRCLE_cust_LD_text, btnRight_CIRCLE_cust_panel,
                        btnRight_CIRCLE_use_DP1,
                        btnRight_CIRCLE_DP1_open_start_delay,
                        btnRight_CIRCLE_DP1_stay_open_time,
                        btnRight_CIRCLE_use_DP2,
                        btnRight_CIRCLE_DP2_open_start_delay,
                        btnRight_CIRCLE_DP2_stay_open_time,
                        btnRight_CIRCLE_use_DP3,
                        btnRight_CIRCLE_DP3_open_start_delay,
                        btnRight_CIRCLE_DP3_stay_open_time,
                        btnRight_CIRCLE_use_DP4,
                        btnRight_CIRCLE_DP4_open_start_delay,
                        btnRight_CIRCLE_DP4_stay_open_time,
                        btnRight_CIRCLE_use_DP5,
                        btnRight_CIRCLE_DP5_open_start_delay,
                        btnRight_CIRCLE_DP5_stay_open_time,
                        btnRight_CIRCLE_use_DP6,
                        btnRight_CIRCLE_DP6_open_start_delay,
                        btnRight_CIRCLE_DP6_stay_open_time,
                        btnRight_CIRCLE_use_DP7,
                        btnRight_CIRCLE_DP7_open_start_delay,
                        btnRight_CIRCLE_DP7_stay_open_time,
                        btnRight_CIRCLE_use_DP8,
                        btnRight_CIRCLE_DP8_open_start_delay,
                        btnRight_CIRCLE_DP8_stay_open_time,
                        btnRight_CIRCLE_use_DP9,
                        btnRight_CIRCLE_DP9_open_start_delay,
                        btnRight_CIRCLE_DP9_stay_open_time,
                        btnRight_CIRCLE_use_DP10,
                        btnRight_CIRCLE_DP10_open_start_delay,
                        btnRight_CIRCLE_DP10_stay_open_time);


#ifdef SHADOW_VERBOSE
    Serial.println("DOME: btnRight_CIRCLE");
#endif


    return;

  }

  //------------------------------------
  // Send triggers for the L1 + base buttons
  //------------------------------------
  if (PS3NavDome->getButtonPress(UP) && PS3NavDome->getButtonPress(L1) && marcDuinoButtonCounter == 1)
  {

    marcDuinoButtonPush(1, FTbtnUP_L1_MD_func, btnUP_L1_cust_MP3_num, btnUP_L1_cust_LD_type, btnUP_L1_cust_LD_text, btnUP_L1_cust_panel,
                        btnUP_L1_use_DP1,
                        btnUP_L1_DP1_open_start_delay,
                        btnUP_L1_DP1_stay_open_time,
                        btnUP_L1_use_DP2,
                        btnUP_L1_DP2_open_start_delay,
                        btnUP_L1_DP2_stay_open_time,
                        btnUP_L1_use_DP3,
                        btnUP_L1_DP3_open_start_delay,
                        btnUP_L1_DP3_stay_open_time,
                        btnUP_L1_use_DP4,
                        btnUP_L1_DP4_open_start_delay,
                        btnUP_L1_DP4_stay_open_time,
                        btnUP_L1_use_DP5,
                        btnUP_L1_DP5_open_start_delay,
                        btnUP_L1_DP5_stay_open_time,
                        btnUP_L1_use_DP6,
                        btnUP_L1_DP6_open_start_delay,
                        btnUP_L1_DP6_stay_open_time,
                        btnUP_L1_use_DP7,
                        btnUP_L1_DP7_open_start_delay,
                        btnUP_L1_DP7_stay_open_time,
                        btnUP_L1_use_DP8,
                        btnUP_L1_DP8_open_start_delay,
                        btnUP_L1_DP8_stay_open_time,
                        btnUP_L1_use_DP9,
                        btnUP_L1_DP9_open_start_delay,
                        btnUP_L1_DP9_stay_open_time,
                        btnUP_L1_use_DP10,
                        btnUP_L1_DP10_open_start_delay,
                        btnUP_L1_DP10_stay_open_time);

#ifdef SHADOW_VERBOSE
    Serial.println("DOME: btnUP_L1");
#endif


    return;

  }

  if (PS3NavDome->getButtonPress(DOWN) && PS3NavDome->getButtonPress(L1) && marcDuinoButtonCounter == 1)
  {

    marcDuinoButtonPush(1, FTbtnDown_L1_MD_func, btnDown_L1_cust_MP3_num, btnDown_L1_cust_LD_type, btnDown_L1_cust_LD_text, btnDown_L1_cust_panel,
                        btnDown_L1_use_DP1,
                        btnDown_L1_DP1_open_start_delay,
                        btnDown_L1_DP1_stay_open_time,
                        btnDown_L1_use_DP2,
                        btnDown_L1_DP2_open_start_delay,
                        btnDown_L1_DP2_stay_open_time,
                        btnDown_L1_use_DP3,
                        btnDown_L1_DP3_open_start_delay,
                        btnDown_L1_DP3_stay_open_time,
                        btnDown_L1_use_DP4,
                        btnDown_L1_DP4_open_start_delay,
                        btnDown_L1_DP4_stay_open_time,
                        btnDown_L1_use_DP5,
                        btnDown_L1_DP5_open_start_delay,
                        btnDown_L1_DP5_stay_open_time,
                        btnDown_L1_use_DP6,
                        btnDown_L1_DP6_open_start_delay,
                        btnDown_L1_DP6_stay_open_time,
                        btnDown_L1_use_DP7,
                        btnDown_L1_DP7_open_start_delay,
                        btnDown_L1_DP7_stay_open_time,
                        btnDown_L1_use_DP8,
                        btnDown_L1_DP8_open_start_delay,
                        btnDown_L1_DP8_stay_open_time,
                        btnDown_L1_use_DP9,
                        btnDown_L1_DP9_open_start_delay,
                        btnDown_L1_DP9_stay_open_time,
                        btnDown_L1_use_DP10,
                        btnDown_L1_DP10_open_start_delay,
                        btnDown_L1_DP10_stay_open_time);

#ifdef SHADOW_VERBOSE
    Serial.println("DOME: btnDown_L1");
#endif


    return;

  }

  if (PS3NavDome->getButtonPress(LEFT) && PS3NavDome->getButtonPress(L1) && marcDuinoButtonCounter == 1)
  {

    marcDuinoButtonPush(1, FTbtnLeft_L1_MD_func, btnLeft_L1_cust_MP3_num, btnLeft_L1_cust_LD_type, btnLeft_L1_cust_LD_text, btnLeft_L1_cust_panel,
                        btnLeft_L1_use_DP1,
                        btnLeft_L1_DP1_open_start_delay,
                        btnLeft_L1_DP1_stay_open_time,
                        btnLeft_L1_use_DP2,
                        btnLeft_L1_DP2_open_start_delay,
                        btnLeft_L1_DP2_stay_open_time,
                        btnLeft_L1_use_DP3,
                        btnLeft_L1_DP3_open_start_delay,
                        btnLeft_L1_DP3_stay_open_time,
                        btnLeft_L1_use_DP4,
                        btnLeft_L1_DP4_open_start_delay,
                        btnLeft_L1_DP4_stay_open_time,
                        btnLeft_L1_use_DP5,
                        btnLeft_L1_DP5_open_start_delay,
                        btnLeft_L1_DP5_stay_open_time,
                        btnLeft_L1_use_DP6,
                        btnLeft_L1_DP6_open_start_delay,
                        btnLeft_L1_DP6_stay_open_time,
                        btnLeft_L1_use_DP7,
                        btnLeft_L1_DP7_open_start_delay,
                        btnLeft_L1_DP7_stay_open_time,
                        btnLeft_L1_use_DP8,
                        btnLeft_L1_DP8_open_start_delay,
                        btnLeft_L1_DP8_stay_open_time,
                        btnLeft_L1_use_DP9,
                        btnLeft_L1_DP9_open_start_delay,
                        btnLeft_L1_DP9_stay_open_time,
                        btnLeft_L1_use_DP10,
                        btnLeft_L1_DP10_open_start_delay,
                        btnLeft_L1_DP10_stay_open_time);

#ifdef SHADOW_VERBOSE
    Serial.println("DOME: btnLeft_L1");
#endif


    return;

  }

  if (PS3NavDome->getButtonPress(RIGHT) && PS3NavDome->getButtonPress(L1) && marcDuinoButtonCounter == 1)
  {

    marcDuinoButtonPush(1, FTbtnRight_L1_MD_func, btnRight_L1_cust_MP3_num, btnRight_L1_cust_LD_type, btnRight_L1_cust_LD_text, btnRight_L1_cust_panel,
                        btnRight_L1_use_DP1,
                        btnRight_L1_DP1_open_start_delay,
                        btnRight_L1_DP1_stay_open_time,
                        btnRight_L1_use_DP2,
                        btnRight_L1_DP2_open_start_delay,
                        btnRight_L1_DP2_stay_open_time,
                        btnRight_L1_use_DP3,
                        btnRight_L1_DP3_open_start_delay,
                        btnRight_L1_DP3_stay_open_time,
                        btnRight_L1_use_DP4,
                        btnRight_L1_DP4_open_start_delay,
                        btnRight_L1_DP4_stay_open_time,
                        btnRight_L1_use_DP5,
                        btnRight_L1_DP5_open_start_delay,
                        btnRight_L1_DP5_stay_open_time,
                        btnRight_L1_use_DP6,
                        btnRight_L1_DP6_open_start_delay,
                        btnRight_L1_DP6_stay_open_time,
                        btnRight_L1_use_DP7,
                        btnRight_L1_DP7_open_start_delay,
                        btnRight_L1_DP7_stay_open_time,
                        btnRight_L1_use_DP8,
                        btnRight_L1_DP8_open_start_delay,
                        btnRight_L1_DP8_stay_open_time,
                        btnRight_L1_use_DP9,
                        btnRight_L1_DP9_open_start_delay,
                        btnRight_L1_DP9_stay_open_time,
                        btnRight_L1_use_DP10,
                        btnRight_L1_DP10_open_start_delay,
                        btnRight_L1_DP10_stay_open_time);

#ifdef SHADOW_VERBOSE
    Serial.println("DOME: btnRight_L1");
#endif


    return;

  }

  //------------------------------------
  // Send triggers for the PS + base buttons
  //------------------------------------
  if (PS3NavDome->getButtonPress(UP) && PS3NavFoot->getButtonPress(PS) && marcDuinoButtonCounter == 1)
  {

    marcDuinoButtonPush(1, FTbtnUP_PS_MD_func, btnUP_PS_cust_MP3_num, btnUP_PS_cust_LD_type, btnUP_PS_cust_LD_text, btnUP_PS_cust_panel,
                        btnUP_PS_use_DP1,
                        btnUP_PS_DP1_open_start_delay,
                        btnUP_PS_DP1_stay_open_time,
                        btnUP_PS_use_DP2,
                        btnUP_PS_DP2_open_start_delay,
                        btnUP_PS_DP2_stay_open_time,
                        btnUP_PS_use_DP3,
                        btnUP_PS_DP3_open_start_delay,
                        btnUP_PS_DP3_stay_open_time,
                        btnUP_PS_use_DP4,
                        btnUP_PS_DP4_open_start_delay,
                        btnUP_PS_DP4_stay_open_time,
                        btnUP_PS_use_DP5,
                        btnUP_PS_DP5_open_start_delay,
                        btnUP_PS_DP5_stay_open_time,
                        btnUP_PS_use_DP6,
                        btnUP_PS_DP6_open_start_delay,
                        btnUP_PS_DP6_stay_open_time,
                        btnUP_PS_use_DP7,
                        btnUP_PS_DP7_open_start_delay,
                        btnUP_PS_DP7_stay_open_time,
                        btnUP_PS_use_DP8,
                        btnUP_PS_DP8_open_start_delay,
                        btnUP_PS_DP8_stay_open_time,
                        btnUP_PS_use_DP9,
                        btnUP_PS_DP9_open_start_delay,
                        btnUP_PS_DP9_stay_open_time,
                        btnUP_PS_use_DP10,
                        btnUP_PS_DP10_open_start_delay,
                        btnUP_PS_DP10_stay_open_time);

#ifdef SHADOW_VERBOSE
    Serial.println("DOME: btnUP_PS");
#endif

    return;

  }

  if (PS3NavDome->getButtonPress(DOWN) && PS3NavFoot->getButtonPress(PS) && marcDuinoButtonCounter == 1)
  {

    marcDuinoButtonPush(1, FTbtnDown_PS_MD_func, btnDown_PS_cust_MP3_num, btnDown_PS_cust_LD_type, btnDown_PS_cust_LD_text, btnDown_PS_cust_panel,
                        btnDown_PS_use_DP1,
                        btnDown_PS_DP1_open_start_delay,
                        btnDown_PS_DP1_stay_open_time,
                        btnDown_PS_use_DP2,
                        btnDown_PS_DP2_open_start_delay,
                        btnDown_PS_DP2_stay_open_time,
                        btnDown_PS_use_DP3,
                        btnDown_PS_DP3_open_start_delay,
                        btnDown_PS_DP3_stay_open_time,
                        btnDown_PS_use_DP4,
                        btnDown_PS_DP4_open_start_delay,
                        btnDown_PS_DP4_stay_open_time,
                        btnDown_PS_use_DP5,
                        btnDown_PS_DP5_open_start_delay,
                        btnDown_PS_DP5_stay_open_time,
                        btnDown_PS_use_DP6,
                        btnDown_PS_DP6_open_start_delay,
                        btnDown_PS_DP6_stay_open_time,
                        btnDown_PS_use_DP7,
                        btnDown_PS_DP7_open_start_delay,
                        btnDown_PS_DP7_stay_open_time,
                        btnDown_PS_use_DP8,
                        btnDown_PS_DP8_open_start_delay,
                        btnDown_PS_DP8_stay_open_time,
                        btnDown_PS_use_DP9,
                        btnDown_PS_DP9_open_start_delay,
                        btnDown_PS_DP9_stay_open_time,
                        btnDown_PS_use_DP10,
                        btnDown_PS_DP10_open_start_delay,
                        btnDown_PS_DP10_stay_open_time);

#ifdef SHADOW_VERBOSE
    Serial.println("DOME: btnDown_PS");
#endif


    return;

  }

  if (PS3NavDome->getButtonPress(LEFT) && PS3NavFoot->getButtonPress(PS) && marcDuinoButtonCounter == 1)
  {

    marcDuinoButtonPush(1, FTbtnLeft_PS_MD_func, btnLeft_PS_cust_MP3_num, btnLeft_PS_cust_LD_type, btnLeft_PS_cust_LD_text, btnLeft_PS_cust_panel,
                        btnLeft_PS_use_DP1,
                        btnLeft_PS_DP1_open_start_delay,
                        btnLeft_PS_DP1_stay_open_time,
                        btnLeft_PS_use_DP2,
                        btnLeft_PS_DP2_open_start_delay,
                        btnLeft_PS_DP2_stay_open_time,
                        btnLeft_PS_use_DP3,
                        btnLeft_PS_DP3_open_start_delay,
                        btnLeft_PS_DP3_stay_open_time,
                        btnLeft_PS_use_DP4,
                        btnLeft_PS_DP4_open_start_delay,
                        btnLeft_PS_DP4_stay_open_time,
                        btnLeft_PS_use_DP5,
                        btnLeft_PS_DP5_open_start_delay,
                        btnLeft_PS_DP5_stay_open_time,
                        btnLeft_PS_use_DP6,
                        btnLeft_PS_DP6_open_start_delay,
                        btnLeft_PS_DP6_stay_open_time,
                        btnLeft_PS_use_DP7,
                        btnLeft_PS_DP7_open_start_delay,
                        btnLeft_PS_DP7_stay_open_time,
                        btnLeft_PS_use_DP8,
                        btnLeft_PS_DP8_open_start_delay,
                        btnLeft_PS_DP8_stay_open_time,
                        btnLeft_PS_use_DP9,
                        btnLeft_PS_DP9_open_start_delay,
                        btnLeft_PS_DP9_stay_open_time,
                        btnLeft_PS_use_DP10,
                        btnLeft_PS_DP10_open_start_delay,
                        btnLeft_PS_DP10_stay_open_time);

#ifdef SHADOW_VERBOSE
    Serial.println("DOME: btnLeft_PS");
#endif


    return;

  }

  if (PS3NavDome->getButtonPress(RIGHT) && PS3NavFoot->getButtonPress(PS) && marcDuinoButtonCounter == 1)
  {

    marcDuinoButtonPush(1, FTbtnRight_PS_MD_func, btnRight_PS_cust_MP3_num, btnRight_PS_cust_LD_type, btnRight_PS_cust_LD_text, btnRight_PS_cust_panel,
                        btnRight_PS_use_DP1,
                        btnRight_PS_DP1_open_start_delay,
                        btnRight_PS_DP1_stay_open_time,
                        btnRight_PS_use_DP2,
                        btnRight_PS_DP2_open_start_delay,
                        btnRight_PS_DP2_stay_open_time,
                        btnRight_PS_use_DP3,
                        btnRight_PS_DP3_open_start_delay,
                        btnRight_PS_DP3_stay_open_time,
                        btnRight_PS_use_DP4,
                        btnRight_PS_DP4_open_start_delay,
                        btnRight_PS_DP4_stay_open_time,
                        btnRight_PS_use_DP5,
                        btnRight_PS_DP5_open_start_delay,
                        btnRight_PS_DP5_stay_open_time,
                        btnRight_PS_use_DP6,
                        btnRight_PS_DP6_open_start_delay,
                        btnRight_PS_DP6_stay_open_time,
                        btnRight_PS_use_DP7,
                        btnRight_PS_DP7_open_start_delay,
                        btnRight_PS_DP7_stay_open_time,
                        btnRight_PS_use_DP8,
                        btnRight_PS_DP8_open_start_delay,
                        btnRight_PS_DP8_stay_open_time,
                        btnRight_PS_use_DP9,
                        btnRight_PS_DP9_open_start_delay,
                        btnRight_PS_DP9_stay_open_time,
                        btnRight_PS_use_DP10,
                        btnRight_PS_DP10_open_start_delay,
                        btnRight_PS_DP10_stay_open_time);

#ifdef SHADOW_VERBOSE
    Serial.println("DOME: btnRight_PS");
#endif


    return;

  }

}


// =======================================================================================
// This function handles the processing of custom MarcDuino panel routines
// =======================================================================================
void custMarcDuinoPanel()
{

  // Open & Close Logic: Dome Panel #1
  if (DP1_Status == 1)
  {

    if ((DP1_start + (DP1_s_delay * 1000)) < millis())
    {

      Serial1.print(":OP01\r");
      DP1_Status = 2;
    }

  }

  if (DP1_Status == 2)
  {

    if ((DP1_start + ((DP1_s_delay + DP1_o_time) * 1000)) < millis())
    {

      Serial1.print(":CL01\r");
      DP1_Status = 0;
    }

  }

  // Open & Close Logic: Dome Panel #2
  if (DP2_Status == 1)
  {

    if ((DP2_start + (DP2_s_delay * 1000)) < millis())
    {

      Serial1.print(":OP02\r");
      DP2_Status = 2;
    }

  }

  if (DP2_Status == 2)
  {

    if ((DP2_start + ((DP2_s_delay + DP2_o_time) * 1000)) < millis())
    {

      Serial1.print(":CL02\r");
      DP2_Status = 0;
    }

  }

  // Open & Close Logic: Dome Panel #3
  if (DP3_Status == 1)
  {

    if ((DP3_start + (DP3_s_delay * 1000)) < millis())
    {

      Serial1.print(":OP03\r");
      DP3_Status = 2;
    }

  }

  if (DP3_Status == 2)
  {

    if ((DP3_start + ((DP3_s_delay + DP3_o_time) * 1000)) < millis())
    {

      Serial1.print(":CL03\r");
      DP3_Status = 0;
    }

  }

  // Open & Close Logic: Dome Panel #4
  if (DP4_Status == 1)
  {

    if ((DP4_start + (DP4_s_delay * 1000)) < millis())
    {

      Serial1.print(":OP04\r");
      DP4_Status = 2;
    }

  }

  if (DP4_Status == 2)
  {

    if ((DP4_start + ((DP4_s_delay + DP4_o_time) * 1000)) < millis())
    {

      Serial1.print(":CL04\r");
      DP4_Status = 0;
    }

  }

  // Open & Close Logic: Dome Panel #5
  if (DP5_Status == 1)
  {

    if ((DP5_start + (DP5_s_delay * 1000)) < millis())
    {

      Serial1.print(":OP05\r");
      DP5_Status = 2;
    }

  }

  if (DP5_Status == 2)
  {

    if ((DP5_start + ((DP5_s_delay + DP5_o_time) * 1000)) < millis())
    {

      Serial1.print(":CL05\r");
      DP5_Status = 0;
    }

  }

  // Open & Close Logic: Dome Panel #6
  if (DP6_Status == 1)
  {

    if ((DP6_start + (DP6_s_delay * 1000)) < millis())
    {

      Serial1.print(":OP06\r");
      DP6_Status = 2;
    }

  }

  if (DP6_Status == 2)
  {

    if ((DP6_start + ((DP6_s_delay + DP6_o_time) * 1000)) < millis())
    {

      Serial1.print(":CL06\r");
      DP6_Status = 0;
    }

  }

  // Open & Close Logic: Dome Panel #7
  if (DP7_Status == 1)
  {

    if ((DP7_start + (DP7_s_delay * 1000)) < millis())
    {

      Serial1.print(":OP07\r");
      DP7_Status = 2;
    }

  }

  if (DP7_Status == 2)
  {

    if ((DP7_start + ((DP7_s_delay + DP7_o_time) * 1000)) < millis())
    {

      Serial1.print(":CL07\r");
      DP7_Status = 0;
    }

  }

  // Open & Close Logic: Dome Panel #8
  if (DP8_Status == 1)
  {

    if ((DP8_start + (DP8_s_delay * 1000)) < millis())
    {

      Serial1.print(":OP08\r");
      DP8_Status = 2;
    }

  }

  if (DP8_Status == 2)
  {

    if ((DP8_start + ((DP8_s_delay + DP8_o_time) * 1000)) < millis())
    {

      Serial1.print(":CL08\r");
      DP8_Status = 0;
    }

  }

  // Open & Close Logic: Dome Panel #9
  if (DP9_Status == 1)
  {

    if ((DP9_start + (DP9_s_delay * 1000)) < millis())
    {

      Serial1.print(":OP09\r");
      DP9_Status = 2;
    }

  }

  if (DP9_Status == 2)
  {

    if ((DP9_start + ((DP9_s_delay + DP9_o_time) * 1000)) < millis())
    {

      Serial1.print(":CL09\r");
      DP9_Status = 0;
    }

  }

  // Open & Close Logic: Dome Panel #10
  if (DP10_Status == 1)
  {

    if ((DP10_start + (DP10_s_delay * 1000)) < millis())
    {

      Serial1.print(":OP10\r");
      DP10_Status = 2;
    }

  }

  if (DP10_Status == 2)
  {

    if ((DP10_start + ((DP10_s_delay + DP10_o_time) * 1000)) < millis())
    {

      Serial1.print(":CL10\r");
      DP10_Status = 0;
    }

  }

  // If all the panels have now closed - close out the custom routine
  if (DP1_Status + DP2_Status + DP3_Status + DP4_Status + DP5_Status + DP6_Status + DP7_Status + DP8_Status + DP9_Status + DP10_Status == 0)
  {

    runningCustRoutine = false;

  }
}

// =======================================================================================
//                             Dome Automation Function
//
//    Features toggles 'on' via L2 + CIRCLE.  'off' via L2 + CROSS.  Default is 'off'.
//    Update on 8/30/2015 - enabled it to be toggled using L2 + CIRCLE only...
//    This routines randomly turns the dome motor in both directions.  It assumes the
//    dome is in the 'home' position when the auto dome feature is toggled on.  From
//    there it turns the dome in a random direction.  Stops for a random length of
//    of time.  Then returns the dome to the home position.  This randomly repeats.
//
//    It is driven off the user variable - time360DomeTurn.  This records how long
//    it takes the dome to do a 360 degree turn at the given auto dome speed.  Tweaking
//    this parameter to be close provides the best results.
//
//    Activating the dome controller manually immediately cancels the auto dome feature
//    or you can toggle the feature off by pressing L2 + CROSS.
// =======================================================================================
void autoDome()
{
  long rndNum;
  int domeSpeed;

  if (domeStatus == 0)  // Dome is currently stopped - prepare for a future turn
  {

    if (domeTargetPosition == 0)  // Dome is currently in the home position - prepare to turn away
    {

      domeStartTurnTime = millis() + (random(3, 10) * 1000);

      rndNum = random(5, 354);

      domeTargetPosition = rndNum;  // set the target position to a random degree of a 360 circle - shaving off the first and last 5 degrees

      if (domeTargetPosition < 180)  // Turn the dome in the positive direction
      {

        dometurnDirection = 1;

        domeStopTurnTime = domeStartTurnTime + ((domeTargetPosition / 360) * time360DomeTurn);

      } else  // Turn the dome in the negative direction
      {

        dometurnDirection = -1;

        domeStopTurnTime = domeStartTurnTime + (((360 - domeTargetPosition) / 360) * time360DomeTurn);

      }

    } else  // Dome is not in the home position - send it back to home
    {

      domeStartTurnTime = millis() + (random(3, 10) * 1000);

      if (domeTargetPosition < 180)
      {

        dometurnDirection = -1;

        domeStopTurnTime = domeStartTurnTime + ((domeTargetPosition / 360) * time360DomeTurn);

      } else
      {

        dometurnDirection = 1;

        domeStopTurnTime = domeStartTurnTime + (((360 - domeTargetPosition) / 360) * time360DomeTurn);

      }

      domeTargetPosition = 0;

    }

    domeStatus = 1;  // Set dome status to preparing for a future turn

#ifdef SHADOW_DEBUG
    Serial.print("Dome Automation: Initial Turn Set\r\n");
    Serial.print( "Current Time: ");
    Serial.print( millis());
    Serial.print("\r\n Next Start Time: ");
    Serial.print(domeStartTurnTime);
    Serial.print("\r\n");
    Serial.print("Next Stop Time: ");
    Serial.print(domeStopTurnTime);
    Serial.print("\r\n");
    Serial.print("Dome Target Position: ");
    Serial.print(domeTargetPosition);
    Serial.print("\r\n");
#endif

  }


  if (domeStatus == 1)  // Dome is prepared for a future move - start the turn when ready
  {

    if (domeStartTurnTime < millis())
    {

      domeStatus = 2;

      //             #ifdef SHADOW_DEBUG
      //                Serial.print("Dome Automation: Ready To Start Turn\r\n");
      //             #endif

    }
  }

  if (domeStatus == 2) // Dome is now actively turning until it reaches its stop time
  {

    if (domeStopTurnTime > millis())
    {

      domeSpeed = domeAutoSpeed * dometurnDirection;

      SyR->motor(domeSpeed);

      //             #ifdef SHADOW_DEBUG
      //                Serial.print("Turning Now!!\r\n");
      //             #endif


    } else  // turn completed - stop the motor
    {
      domeStatus = 0;
      SyR->stop();

      //              #ifdef SHADOW_DEBUG
      //                 Serial.print("STOP TURN!!\r\n");
      //              #endif
    }

  }

}

// =======================================================================================
//           Program Utility Functions - Called from various locations
// =======================================================================================

// =======================================================================================
//           PPS3 Controller Device Mgt Functions
// =======================================================================================

void onInitPS3NavFoot()
{
  String btAddress = getLastConnectedBtMAC();
  PS3NavFoot->setLedOn(LED1);
  isPS3NavigatonInitialized = true;
  badPS3Data = 0;

#ifdef SHADOW_DEBUG
  Serial.print("\r\nBT Address of Last connected Device when FOOT PS3 Connected: ");
  Serial.print(btAddress);
#endif

  if (btAddress == PS3ControllerFootMac || btAddress == PS3ControllerBackupFootMac)
  {

#ifdef SHADOW_DEBUG
    Serial.print("\r\nWe have our FOOT controller connected.\r\n");
#endif

    mainControllerConnected = true;
    WaitingforReconnect = true;

  } else
  {

    // Prevent connection from anything but the MAIN controllers
#ifdef SHADOW_DEBUG
    Serial.print("\r\nWe have an invalid controller trying to connect as tha FOOT controller, it will be dropped.\r\n");
#endif

    ST->stop();
    SyR->stop();
    isFootMotorStopped = true;
    footDriveSpeed = 0;
    PS3NavFoot->setLedOff(LED1);
    PS3NavFoot->disconnect();
    //    printOutput();

    isPS3NavigatonInitialized = false;
    mainControllerConnected = false;

  }
}

void onInitPS3NavDome()
{
  String btAddress = getLastConnectedBtMAC();
  PS3NavDome->setLedOn(LED1);
  isSecondaryPS3NavigatonInitialized = true;
  badPS3Data = 0;

  if (btAddress == PS3ControllerDomeMAC || btAddress == PS3ControllerBackupDomeMAC)
  {
    //
#ifdef SHADOW_DEBUG
    Serial.print("\r\nWe have our DOME controller connected.\r\n");
#endif

    domeControllerConnected = true;
    WaitingforReconnectDome = true;

  } else
  {

    // Prevent connection from anything but the DOME controllers
#ifdef SHADOW_DEBUG
    Serial.print("\r\nWe have an invalid controller trying to connect as the DOME controller, it will be dropped.\r\n");
#endif

    ST->stop();
    SyR->stop();
    isFootMotorStopped = true;
    footDriveSpeed = 0;
    PS3NavDome->setLedOff(LED1);
    PS3NavDome->disconnect();
    //    printOutput();

    isSecondaryPS3NavigatonInitialized = false;
    domeControllerConnected = false;

  }
}

String getLastConnectedBtMAC()
{
  String btAddress = "";
  for (int8_t i = 5; i > 0; i--)
  {
    if (Btd.disc_bdaddr[i] < 0x10)
    {
      btAddress += "0";
    }
    btAddress += String(Btd.disc_bdaddr[i], HEX);
    btAddress += (":");
  }
  btAddress += String(Btd.disc_bdaddr[0], HEX);
  btAddress.toUpperCase();
  return btAddress;
}

boolean criticalFaultDetect()
{
  if (PS3NavFoot->PS3NavigationConnected || PS3NavFoot->PS3Connected)
  {

    currentTime = millis();
    lastMsgTime = PS3NavFoot->getLastMessageTime();
    msgLagTime = currentTime - lastMsgTime;

    if (WaitingforReconnect)
    {

      if (msgLagTime < 200)
      {

        WaitingforReconnect = false;

      }

      lastMsgTime = currentTime;

    }

    if ( currentTime >= lastMsgTime)
    {
      msgLagTime = currentTime - lastMsgTime;

    } else
    {

      msgLagTime = 0;
    }

    if (msgLagTime > 300 && !isFootMotorStopped)
    {
      //            #ifdef SHADOW_DEBUG
      //              Serial.print("It has been 300ms since we heard from the PS3 Foot Controller\r\n");
      //              Serial.print("Shut downing motors, and watching for a new PS3 Foot message\r\n");
      //            #endif
      ST->stop();
      isFootMotorStopped = true;
      footDriveSpeed = 0;
    }

    if ( msgLagTime > 10000 )
    {
      //            #ifdef SHADOW_DEBUG
      //              Serial.print("It has been 10s since we heard from the PS3 Foot Controller\r\n");
      //              Serial.print("msgLagTime:");
      //              Serial.print(msgLagTime);
      //              Serial.print("  lastMsgTime:");
      //              Serial.print(lastMsgTime);
      //              Serial.print("  millis:");
      //              Serial.print(millis());
      //              Serial.print("\r\nDisconnecting the Foot controller.\r\n");
      //            #endif
      ST->stop();
      isFootMotorStopped = true;
      footDriveSpeed = 0;
      PS3NavFoot->disconnect();
      WaitingforReconnect = true;
      return true;
    }

    //Check PS3 Signal Data
    if (!PS3NavFoot->getStatus(Plugged) && !PS3NavFoot->getStatus(Unplugged))
    {
      //We don't have good data from the controller.
      //Wait 15ms if no second controller - 100ms if some controller connected, Update USB, and try again
      if (PS3NavDome->PS3NavigationConnected)
      {
        delay(100);
      } else
      {
        delay(15);
      }

      Usb.Task();
      lastMsgTime = PS3NavFoot->getLastMessageTime();

      if (!PS3NavFoot->getStatus(Plugged) && !PS3NavFoot->getStatus(Unplugged))
      {
        badPS3Data++;
        //                #ifdef SHADOW_DEBUG
        //                    Serial.print("\r\n**Invalid data from PS3 FOOT Controller. - Resetting Data**\r\n");
        //                #endif
        return true;
      }
    }
    else if (badPS3Data > 0)
    {

      badPS3Data = 0;
    }

    if ( badPS3Data > 10 )
    {
      //            #ifdef SHADOW_DEBUG
      //                Serial.print("Too much bad data coming from the PS3 FOOT Controller\r\n");
      //                Serial.print("Disconnecting the controller and stop motors.\r\n");
      //            #endif
      ST->stop();
      isFootMotorStopped = true;
      footDriveSpeed = 0;
      PS3NavFoot->disconnect();
      WaitingforReconnect = true;
      return true;
    }
  }
  else if (!isFootMotorStopped)
  {
    //        #ifdef SHADOW_DEBUG
    //            Serial.print("No foot controller was found\r\n");
    //            Serial.print("Shuting down motors and watching for a new PS3 foot message\r\n");
    //        #endif
    ST->stop();
    isFootMotorStopped = true;
    footDriveSpeed = 0;
    WaitingforReconnect = true;
    return true;
  }

  return false;
}

boolean criticalFaultDetectDome()
{
  if (PS3NavDome->PS3NavigationConnected || PS3NavDome->PS3Connected)
  {

    currentTime = millis();
    lastMsgTime = PS3NavDome->getLastMessageTime();
    msgLagTime = currentTime - lastMsgTime;

    if (WaitingforReconnectDome)
    {
      if (msgLagTime < 200)
      {

        WaitingforReconnectDome = false;

      }

      lastMsgTime = currentTime;

    }

    if ( currentTime >= lastMsgTime)
    {
      msgLagTime = currentTime - lastMsgTime;

    } else
    {
      msgLagTime = 0;
    }

    if ( msgLagTime > 10000 )
    {
      //            #ifdef SHADOW_DEBUG
      //              Serial.print("It has been 10s since we heard from the PS3 Dome Controller\r\n");
      //              Serial.print("msgLagTime:");
      //              Serial.print(msgLagTime);
      //              Serial.print("  lastMsgTime:");
      //              Serial.print(lastMsgTime);
      //              Serial.print("  millis:");
      //              Serial.print(millis());
      //              Serial.print("\r\nDisconnecting the Dome controller.\r\n");
      //            #endif

      SyR->stop();
      PS3NavDome->disconnect();
      WaitingforReconnectDome = true;
      return true;
    }

    //Check PS3 Signal Data
    if (!PS3NavDome->getStatus(Plugged) && !PS3NavDome->getStatus(Unplugged))
    {

      // We don't have good data from the controller.
      //Wait 100ms, Update USB, and try again
      delay(100);

      Usb.Task();
      lastMsgTime = PS3NavDome->getLastMessageTime();

      if (!PS3NavDome->getStatus(Plugged) && !PS3NavDome->getStatus(Unplugged))
      {
        badPS3DataDome++;
        //                #ifdef SHADOW_DEBUG
        //                    Serial.print("\r\n**Invalid data from PS3 Dome Controller. - Resetting Data**\r\n");
        //                #endif
        return true;
      }
    } else if (badPS3DataDome > 0)
    {
      badPS3DataDome = 0;
    }

    if ( badPS3DataDome > 10 )
    {
      //            #ifdef SHADOW_DEBUG
      //                Serial.print("Too much bad data coming from the PS3 DOME Controller\r\n");
      //                Serial.print("Disconnecting the controller and stop motors.\r\n");
      //            #endif
      SyR->stop();
      PS3NavDome->disconnect();
      WaitingforReconnectDome = true;
      return true;
    }
  }

  return false;
}

// =======================================================================================
//           USB Read Function - Supports Main Program Loop
// =======================================================================================

boolean readUSB()
{

  Usb.Task();

  //The more devices we have connected to the USB or BlueTooth, the more often Usb.Task need to be called to eliminate latency.
  if (PS3NavFoot->PS3NavigationConnected)
  {
    if (criticalFaultDetect())
    {
      //We have a fault condition that we want to ensure that we do NOT process any controller data!!!
      //    printOutput();
      return false;
    }

  } else if (!isFootMotorStopped)
  {
    //        #ifdef SHADOW_DEBUG
    //            Serial.print("No foot controller was found\r\n");
    //            Serial.print("Shuting down motors, and watching for a new PS3 foot message\r\n");
    //        #endif
    ST->stop();
    isFootMotorStopped = true;
    footDriveSpeed = 0;
    WaitingforReconnect = true;
  }

  if (PS3NavDome->PS3NavigationConnected)
  {

    if (criticalFaultDetectDome())
    {
      //We have a fault condition that we want to ensure that we do NOT process any controller data!!!
      ////    printOutput();
      return false;
    }
  }

  return true;
}
#ifdef BTSupport
void BTsendandReceive() 
{ 
  char recvChar;
  while(1)
  {
  if(BTSerial.available()) //check if there's any data sent from the remote bluetooth shield
  {
    recvChar = BTSerial.read();
    if (isDebug)
      Serial.print(recvChar); // Print the character received to the Serial Monitor (if required)
    
    //If the character received = 'r' , then do something
    if(recvChar=='r')
    {
      // Do something here
    }
    
    //If the character received = 'g' , then do something
    if(recvChar=='g')
    {
      // Do something here
    }
    
    //If the character received = 'b' , then do someting
    if(recvChar=='b')
    {
      // Do something here
    }
  }
  
  //You can use the following code to deal with any information coming from the Computer (serial monitor)
  if(Serial.available()){
  recvChar = Serial.read();
  
  //This will send value obtained (recvChar) to the BT Monitoring system. The value will be displayed on the mobile device or tablet.
  BTSerial.print(recvChar);
  }
  }
} 
#endif
// =======================================================================================
//          Print Output Function
// =======================================================================================

//void printOutput()
//{
//    if (output != "")
//    {
//        if (Serial) Serial.println(output);
//        output = ""; // Reset output string
//    }
//}

