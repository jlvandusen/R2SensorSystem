//  Created by James VanDusen | jlvandusen@msn.com
//  for R2Sensor System
//  Slave/Satellite Sensor controller or voice controller
//  Connection information and instructions for correct system integration to PADAWAN, SHADOW and SHADOW_MD
//  version 3.5 10/10/2015  - Uses i2c to send back to SHADOW directly

//  We are currently using Arduino Nano for foot sensors
//  Use Duemilanove or Uno for Voice Commands <-- Very important due to SHIELD integration
//  Uses EasyVR 2.0 shield connections to a UNO or DUEMILANOVE

//  Nano to Mega pins instructions
//  A4 (SCL) goes to SCL on i2c Sensor (navigation) (if Dome specific setup)
//  A5 (SDA) goes to SDA on i2c Sensor (navigation) (if Dome)
//  VIN goes to V5 on Mega (Master Sensor Controller)
//  GND goes to GND on Mega (Master Sensor Controller)

//  GND goes to GND on Any Sensor(s)
//  2-D4 goes to Trig on Ping Sensor(s)
//  D7-D9 goes to ECHO on Ping Sensor(s)


#define LEFTFOOT                  //uncomment this for Left Foot Controller
// #define RIGHTFOOT                 //uncomment this for Right Foot Controller
// #define CENTERFOOT                //uncomment this for Center Foot Controller
// #define DOME                      //uncomment this for Dome Controller
// #define VOICE                     //uncomment this for Voice Controller

#include <Wire.h>
#include <Math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <NewPing.h>

// ====================================================================================================================
// Common Settings
// ====================================================================================================================
bool isDebug = false;  //enable disable output

// ====================================================================================================================
// Satalite Sensor Settings (which sensor is this)
// Location Setup (which sensor is this for the assignment and upload)
// ====================================================================================================================

#ifdef  LEFTFOOT
String Part = "left";
#endif
#ifdef  RIGHTFOOT
String Part = "right";
#endif
#ifdef  CENTERFOOT
String Part = "center";
#endif
#ifdef  DOME
String Part = "dome";
#endif
#ifdef  VOICE
String Part = "voice";                                                //make sure to set the serial to 9600 as the EasyVR only supports 9600
#endif

uint32_t currentTime = 0;
int const WaitInterval = 30000;                                       // After 30 seconds, voice system resets waiting for trigger command
int const PingWaitInterval = 2000;                                        // After 3 seconds, send ping sensor data over i2c
unsigned long lastDecisionTime = 0, DomelastDecisionTime=0;           // The last time - in millis() - that we made a decision (start)
int partnum,front,side,back,left,right,nav;                           // Set default distances and direction so if R2 is moving it doesnt come to stop.
int IsScaning=0;

// ====================================================================================================================
// PING SENSOR settings
// ====================================================================================================================
#define SONAR_NUM     3       // Number or sensors.
#define MAX_DISTANCE 300      // Maximum distance (in cm) to ping.
#define PING_INTERVAL 33      // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).
#define SENSOR_COUNTER 50     // Number of times on the counter before a next sensor check is performed... allowing free query and reducing excess queries against the sensors

// Sensors Setup pins
int DS1T = 2; // Ping (trigger)
int DS2T = 3; // Ping (trigger)
int DS3T = 4; // Ping (trigger)
int DS1E = 7; // Echo PING Sensor FRONT
int DS2E = 8; // Echo PING Sensor SIDE
int DS3E = 9; // Echo PING Sensor BACK

// Voice EasyVR Setup Config
int8_t voicecmd;

unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.                   

NewPing sonar[SONAR_NUM] = {        // Sensor object array.
  NewPing(DS1T, DS1E, MAX_DISTANCE),// Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(DS2T, DS2E, MAX_DISTANCE),
  NewPing(DS3T, DS3E, MAX_DISTANCE)
  };

// int counter;                       // COUNTER that keeps incrementing and being reset based on tasks below.
String ib = "", Output = "";        // Assign Variables for Output to Serial as well as Input from Serial (in Debug)


// Assign a unique ID to the navigation sensor(s) at the same time
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);


// ---------------------------------------------------------------------------------------
//                          EasyVR Library and Configuration
// ---------------------------------------------------------------------------------------
#ifdef VOICE
#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
  #include "Platform.h"
  #include "SoftwareSerial.h"
#ifndef CDC_ENABLED                               // Shield Jumper on SW
  SoftwareSerial port(12,13);
#else                                             // Shield Jumper on HW (for Leonardo)
  #define port Serial1
#endif
#else                                             // Arduino 0022 - use modified SoftwareSerial
  #include "WProgram.h"
  #include "SoftwareSerial.h"
  SoftwareSerial port(12,13);
#endif

#include "EasyVR.h"
EasyVR easyvr(port);

enum Groups                                       //Groups and Commands
{
  GROUP_0  = 0,
  GROUP_1  = 1,
};

enum Group0 
{
  G0_HEY_R2 = 0,
};

enum Group1 
{
  G1_PATROL = 0,
  G1_FOLLOW_ME = 1,
  G1_VADER = 2,
  G1_LUKE = 3,
  G1_LEA = 4,
  G1_C3PO = 5,
  G1_STAY = 6,
  G1_OPEN_PANELS = 7,
  G1_CLOSE_PANELS = 8,
  G1_BACKUP = 9,
  G1_FORWARD = 10,
};

EasyVRBridge bridge;
int8_t group, idx;                        
#endif

// ====================================================================================================================
// This function determines I2C addressing 
// ====================================================================================================================
#define I2C_ADDRESS_SHADOW  0x1       // 0x1 = SHADOW, 0x2 = left, 0x3 = right, 0x4 = dome, 0x5 = center, 0x6 = voice
#define I2C_ADDRESS_LEFT    0x2       // 0x1 = SHADOW, 0x2 = left, 0x3 = right, 0x4 = dome, 0x5 = center, 0x6 = voice
#define I2C_ADDRESS_RIGHT   0x3       // 0x1 = SHADOW, 0x2 = left, 0x3 = right, 0x4 = dome, 0x5 = center, 0x6 = voice
#define I2C_ADDRESS_DOME    0x4       // 0x1 = SHADOW, 0x2 = left, 0x3 = right, 0x4 = dome, 0x5 = center, 0x6 = voice
#define I2C_ADDRESS_CENTER  0x5       // 0x1 = SHADOW, 0x2 = left, 0x3 = right, 0x4 = dome, 0x5 = center, 0x6 = voice
#define I2C_ADDRESS_VOICE   0x6       // 0x1 = SHADOW, 0x2 = left, 0x3 = right, 0x4 = dome, 0x5 = center, 0x6 = voice
#define I2C_ADDRESS_TCAADDR 0x70      // Define address of the adafruit multiplexer

void setup() {
#ifdef LEFTFOOT
  Wire.begin(I2C_ADDRESS_LEFT);
  partnum=1;
#endif
#ifdef RIGHTFOOT
  Wire.begin(I2C_ADDRESS_RIGHT);
  partnum=2;
#endif
#ifdef CENTERFOOT
  Wire.begin(I2C_ADDRESS_CENTER);
  partnum=3; 
#endif
#ifdef VOICE
  Wire.begin(I2C_ADDRESS_VOICE);
  partnum=4;
#endif
#ifdef DOME
  Wire.begin(I2C_ADDRESS_DOME);
  partnum=5;
#endif
  Wire.onReceive(receiveI2C);
  if (Part !="voice") {
  Serial.begin(115200);
  }
  Serial.flush();
  Serial.print("I:");
  Serial.print(Part);
  Serial.println(" - R2Sensor System Online");

// ====================================================================================================================
// First ping starts at 75ms, gives time for the Arduino to chill before starting.
// Set the starting time for each sensor.
// ====================================================================================================================

  pingTimer[0] = millis() + 75;           
  for (uint8_t i = 1; i < SONAR_NUM; i++) 
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;

// ====================================================================================================================
// Initialise the compass sensor
// ====================================================================================================================

if (Part == "dome"){
    if(!mag.begin())  {
      /* There was a problem detecting the LSM303 ... check your connections */
      Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
      while(1);
    }
}

// ====================================================================================================================
// Initialise the EasyVR Voice Recognition System (connect to Duo or Duerenolvo
// ====================================================================================================================

#ifdef VOICE
#ifndef CDC_ENABLED
  if (bridge.check())
  {
    cli();
    bridge.loop(0, 1, 12, 13);
  }
  // run normally
  Serial.begin(9600);
  Serial.println("R2 Sensor: EasyVR 2.0 Bridge disabled...");
#else
  if (bridge.check())
  {
    port.begin(9600);
    bridge.loop(port);
  }
  Serial.println("R2 Sensor: EasyVR 2.0 Bridge connection completed");
#endif
  port.begin(9600);
  while (!easyvr.detect())
  {
    Serial.println("EasyVR not detected!");
    delay(1000);
  }
  easyvr.setPinOutput(EasyVR::IO1, LOW);
  Serial.print("I:");
  Serial.print(Part);
  Serial.println(" - R2Sensor System Online");
  Serial.println("EasyVR detected and ready");
  easyvr.setTimeout(5);
  easyvr.setLanguage(0);
  group = EasyVR::TRIGGER; //<-- start group (customize)
#endif

} //End of Setup

#ifdef VOICE
void action();
#endif

void loop() {
  if (Part == "right" || Part == "left" || Part == "center"){                       // If sensor is a foot sensor then get the ping sensor data and send it over i2c
    getSensorData();
    currentTime = millis();
    if ((currentTime - lastDecisionTime) >= PingWaitInterval) {
      lastDecisionTime = millis(); 
      SendSensorStatus();
    }
  }
  if (Part == "dome") {
    currentTime = millis();
    if ((currentTime - lastDecisionTime) >= PingWaitInterval) {
      lastDecisionTime = millis(); 
      GetDirection();
      SendDirection();
    }
  }
#ifdef VOICE
if (Part == "voice") {                                                               // If sensor is VOICE module then check time and wait for commands on EasyVR
  currentTime = millis();
  if (group == 0)
  lastDecisionTime = millis();                                                       // Set the lastdecision time to now (unless we are waiting for group 1 (commands)
  
  if (isDebug)  {
    Serial.print("Current time: ");
    Serial.println(currentTime);
    Serial.print("Last Decision time: ");
    Serial.println(lastDecisionTime);
  }
  if ((currentTime - lastDecisionTime) >= WaitInterval && group == 1) {
    lastDecisionTime = millis();
    group = 0;
    voicecmd = 98;
    SendVoiceCommand();
  }
    easyvr.setPinOutput(EasyVR::IO1, HIGH); // LED on (listening)
    if (isDebug)  {
      Serial.print("Say a command in Group ");
      Serial.println(group);
    }
    easyvr.recognizeCommand(group);
    do
    {
      // can do some processing while waiting for a spoken command
    }
    while (!easyvr.hasFinished());
easyvr.setPinOutput(EasyVR::IO1, LOW);                                                // While EasyVR is waiting on the next command set the LED off
    idx = easyvr.getWord();
    if (idx >= 0)
    {
      return;
    }
    idx = easyvr.getCommand();
    if (idx >= 0)
    {
      // print debug message
      uint8_t train = 0;
      char name[32];
      if (group==1)
      voicecmd=idx+1;
      else
      voicecmd=0;
      if (isDebug)  {
        Serial.print("Command: ");
        Serial.print(idx);
      }
      if (easyvr.dumpCommand(group, idx, name, train))
      {
        Serial.print(" = ");
        Serial.println(name);
        SendVoiceCommand();
      }
      else
        Serial.println();
      easyvr.playSound(0, EasyVR::VOL_FULL);
      // perform some action
      action();
    }
    else // errors or timeout
    {
      if (easyvr.isTimeout())
        if (isDebug)  {
          Serial.println("Timed out, try again...");
        }
      int16_t err = easyvr.getError();
      if (err >= 0)
      {
        if (isDebug)  {
        Serial.print("Error ");
        Serial.println(err, HEX); 
        }
        voicecmd = 99;
        SendVoiceCommand();
      }
    }
  }
#endif
  
}

void echoCheck() { // If ping received, set the sensor distance to array.
  if (sonar[currentSensor].check_timer())
    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
}

void receiveI2C(int howMany) {
 while (Wire.available() > 0) {
  char c = Wire.read();
  if (isDebug)  {
  Serial.print(c);
  }
 }
 if (isDebug)  {
 Serial.println();
 }
}

void getSensorData()  {
    for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through all the sensors.
      if (millis() >= pingTimer[i]) {         // Is it this sensor's time to ping?
        pingTimer[i] += PING_INTERVAL * SONAR_NUM;  // Set next time this sensor will be pinged.
        if (i == 0 && currentSensor == SONAR_NUM - 1) oneSensorCycle(); // Sensor ping cycle complete, do something with the results.
        sonar[currentSensor].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
        currentSensor = i;                          // Sensor being accessed.
        // cm[currentSensor] = 0;                      // Make distance zero in case there's no ping echo for this sensor.
        sonar[currentSensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
      }
    }
    front = cm[0];
    side  = cm[1];
    back  = cm[2];
    if (isDebug)  {
      Serial.print("front: ");
      Serial.print(front);
      Serial.print(" side: ");
      Serial.print(side);
      Serial.print(" back: ");
      Serial.println(back);
    }
}


void GetDirection() {
  sensors_event_t event;   // Get and assign the sensor to an event
  mag.getEvent(&event);
  
  float Pi = 3.14159;
  
  // Calculate the angle of the vector y,x
  float heading = (atan2(event.magnetic.y,event.magnetic.x) * 180) / Pi;
  
  // Normalize to 0-360 degrees
  if (heading < 0)  {
    heading = 360 + heading;
  }
  nav=heading;
  if (isDebug) {
    Serial.println(heading);
  }
}

void SendDirection() {
  if (random(1,40)==15)  {
    if (isDebug) {
    Serial.print(nav);
    Serial.print(" Nav:");
    }
    Serial.print(nav);
  }
}

void SendSensorStatus() {
    Wire.beginTransmission(I2C_ADDRESS_SHADOW);             // send to master SHADOW 0x1
    if (Part=="left" || Part =="right" || Part =="center")  // If foot sensors then send front, side and back
    {
      Wire.write(partnum);
      Wire.write(front);
      Wire.write(side);
      Wire.write(back);
    }
    if (Part=="dome")
    {
      Wire.write(partnum);
      Wire.write(nav);
    } 
//    if (isDebug)  {
      Serial.print(partnum);
      Serial.print(" Front:");
      Serial.print(front);
      Serial.print("  Side:");
      Serial.print(side);
      Serial.print("  Back:");
      Serial.println(back);
      if (Part =="dome")  {
        Serial.print("  nav:");
        Serial.print(nav); 
      }
//    }
    Wire.endTransmission();
//}
    delay(25);

}

#ifdef VOICE
void SendVoiceCommand() {
    Wire.beginTransmission(I2C_ADDRESS_SHADOW);  //send to master SHADOW 0x1
      Wire.write(partnum);
      Wire.write(voicecmd);
    if (isDebug)  {
      Serial.print("Partnum:");
      Serial.println(partnum);
      Serial.print("voicecmd(idx+1):");
      Serial.println(voicecmd);
    }
    Wire.endTransmission();

}


void action() {
    switch (group)
    {
    case GROUP_0:
      switch (idx)
      {
      case G0_HEY_R2:
        voicecmd = 0;                 // Go to Wait for further commands
        group = GROUP_1;              // or jump to another group X for composite commands
        break;
      }
      break;
    case GROUP_1:
      switch (idx)
      {
      case G1_PATROL:
        voicecmd = 1;                 // GO on Patrol = 1
        lastDecisionTime = millis();  // Reset lastDecisionTime
        group = GROUP_1;              // Wait for Trigger
        break;
      case G1_FOLLOW_ME:
        voicecmd = 2;                 // Follow me = 2
        lastDecisionTime = millis();  // Reset lastDecisionTime
        group = GROUP_1;              // Wait for Trigger
        break;
      case G1_VADER:
        voicecmd = 3;                 // Darth Vader = 3
        lastDecisionTime = millis();  // Reset lastDecisionTime
        group = GROUP_1;              // Wait for Trigger
        break;
      case G1_LUKE:
        voicecmd = 4;                 // Luke Skywalker = 4
        lastDecisionTime = millis();  // Reset lastDecisionTime
        group = GROUP_1;              // Wait for Trigger
        break;
      case G1_LEA:
        voicecmd = 5;                 // Princess Lea = 5
        lastDecisionTime = millis();  // Reset lastDecisionTime
        group = GROUP_1;              // Wait for Trigger
        break;
      case G1_C3PO:
        voicecmd = 6;                 // C3PO = 6
        lastDecisionTime = millis();  // Reset lastDecisionTime
        group = GROUP_1;              // Wait for Trigger
        break;
      case G1_STAY:
        voicecmd = 7;                 // Stay = 7
        lastDecisionTime = millis();  // Reset lastDecisionTime
        group = GROUP_1;              // Wait for Trigger
        break;
      case G1_OPEN_PANELS:
        voicecmd = 8;                 // Open Panels = 8
        lastDecisionTime = millis();  // Reset lastDecisionTime
        group = GROUP_1;              // Wait for Trigger
        break;
      case G1_CLOSE_PANELS:
        voicecmd = 9;                 // Close Panels = 9
        lastDecisionTime = millis();  // Reset lastDecisionTime
        group = GROUP_1;              // Wait for Trigger
        break;
      case G1_BACKUP:
        voicecmd = 10;       // Backup = 10
        lastDecisionTime = millis();
        group = GROUP_1;  // Wait for Trigger
        break;
      case G1_FORWARD:
        voicecmd = 11;       // Forward = 11
        lastDecisionTime = millis();
        group = GROUP_1;  // Wait for Trigger
        break;
      }
      break;
    }
}
#endif
void oneSensorCycle() {    // Sensor ping cycle complete, do something with the results.

}

void getSerialInput() {
  IsScaning = 0;
  if (Serial.available()) {
    if (IsScaning == 0) {
      Output = "";
      ib = Serial.readString();
      if (ib.startsWith("Debug")) { // Enable/Disable Debug
        IsScaning = 0;
        if (isDebug) {
          isDebug=false;
        } else
        isDebug = true;
      }
    }
  delay(20);
  IsScaning = 1;
  }
}
