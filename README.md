# R2SensorSystem
R2 Sensor System + Shadow + Marcduino

The source code consists of a master SHADOW controller and multiple sensor controllers.
The code for each is included.
Moving average is needed as a library as well as the marcduino header files.

Libraries needed:
Adafruit_GPS
Adafruit_LSM303DLHC
Adafruit_Sensor
EasyVR v2.x
LiquidCrystal_I2C
MP3Trigger
NewPing
NewSoftSerial

Hardware configuration
There are 2 distinct systems connecting the members together.
These devices consist of multiple CPUs working together over i2c serial communications sharing the same clock as the main Arduino running SHADOW or SHADOW_md.

Needed hardware:
Quantity	Unit Type and Info	Descriptions and Purpose	Location
2/3
	Arduino NANO CPUs	This contains the Remote Sensor software	Feet
1	Arduino UNO/Dueranolve	This is used to service the EasyVR Shield	Dome
1	Arduino Mega/Due	This contains the SHADOW Contoller software	Body
1	AdaFruit LSM303DLHC	These allow R2 to determine direction and alignment	Body
1	AdaFruit LSM303DLHC	These allow R2 to determine direction and alignment	Dome
1	Marcduino v1.1	This allows for dome automation such as panel and sound controls	Dome
R2Sensor Main Controller
This is the modified SHADOW code running on a Arduino MEGA SDK or DUE board running latest Arduino code instructions.
R2Sensor Satellite controller
This is a Arduino NANO or mini running the remote sensor code uploaded to it and changed for each location installed.  The current locations supported are:
•	Dome
•	Left foot
•	Right foot
•	Center foot

Nano to Mega pins instructions
VIN goes to V5 Need segregation of power due to loads
GND goes to GND on Mega (Master Sensor Controller) as well as to Segregated lines

GND goes to GND on Any Sensor(s)
5V goes to VCC on Any Sensor(s)
D2-D4 goes to Trig on Ping Sensor(s)
D7-D9 goes to ECHO on Ping Sensor(s)

I2C: A4 (SDA) and A5 (SCL). Support I2C (TWI) communication using the Wire library (documentation on the Wiring website).

A4 (SDA) goes to SDA on i2c Sensor (navigation)
A5 (SCL) goes to SCL on i2c Sensor (navigation)
GND goes to the SHADOW controller's GND (must be shared)
Wiring Configuration
Color Coding
I2C
SDA	White or Orange
SCL	Black or Yellow

Easy VR 2.0 / 3.0
This is to be presented in the near future for support of voice commands to your R2Sensor system

Software configuration
The Satellite software is named R2i2CSENSOR and R2i2CsSHADOW respectively.  They are configurable based on:
•	Type of device uploading to
•	Location of the device
•	What is available on that sensor system
•	What type of hardware (sensors)
