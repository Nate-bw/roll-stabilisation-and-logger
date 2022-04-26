#include <NMEAGPS.h>
#include <GPSport.h>
gps_fix fix;
#include <SPI.h>
#include "SdFat.h"                  // <-- SD
SdFat SD;
#include <Wire.h>                   // <-- General
#include <Servo.h>                  // <-- Servos
#include <NMEAGPS.h>
#include <GPSport.h>
#include <Adafruit_L3GD20.h>        // <-- Gyro
#include <Adafruit_BMP085.h>        // <-- Pressure/ Altitude
#include <Adafruit_Sensor.h>        // <-- General
#include <Adafruit_LSM303_Accel.h>  // <-- accelerometer
#include <Adafruit_LSM303DLH_Mag.h> // <-- Magnometer

//------------------------------------------------------------
float angle_pitch = 0, angle_roll = 0, baro_alt = 0;                                    // define measurement variables

float angle_pitch_acc = 0, angle_roll_acc = 0, acc_total_vector = 0;                   // define variables to fill with values from accelerometer (Pitch, Roll, Magnitude)

float gyro_x_cal = 0, gyro_y_cal = 0, gyro_z_cal = 0, p_ref = 0, T_ref = 0;           // define gyro and pressure sensor values for calibration

float angle_pitch_output = 0, angle_roll_output = 0;                                  // define variable to fill with resultant values for pitch and roll

float heading;

String Headings = "date, time, event, lat, lon, gnd_spd, gps_alt, baro_alt, pitch, roll, heading";
String date = "";
String timeString = "";
String locString = "";
String gnd_spd = "";
String gps_alt = "";


// float n = 0;                                                                          // Number of readings taken
boolean set_gyro_angles = false;                                                      // test if IMU has already completed its first cycle
boolean GPSdat = false;
uint32_t timer = millis();                                                            // Set up timer for data Recording

const int pos_neut = 90;                                                              //initialise servo starting position
//const int pos_up = 65;                                                                //initialise value for the aileron up deflection   <-- Currently extreme so as to be easily observable
//const int pos_down = 25;                                                              //initialise value for the aileron down deflection <-- Currently extreme so as to be easily observable

int n = 0;

String filename = ("data");                                                       // make a string to itterate through filenames
String dataString = "";                                                              // make a string for assembling the data to log:

int angle_out = 0;
const int gain = 1;
float mod_roll;
float angleoutL;
float angleoutR;


//------------------------------------------------------------
static NMEAGPS  gps; // This parses the GPS characters

// Log file base name.  Must be six characters or less.
#define FILE_BASE_NAME "Data"
const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;
char fileName[13] = FILE_BASE_NAME "00.txt";

// Comment this next line to use SPI
#define USE_I2C

#ifdef USE_I2C
  // The default constructor uses I2C
  Adafruit_L3GD20 gyro;           // Initialise Gyro
#else
  // To use SPI, you have to define the pins
  #define GYRO_CS 4 // labeled CS
  #define GYRO_DO 5 // labeled SA0
  #define GYRO_DI 6  // labeled SDA
  #define GYRO_CLK 7 // labeled SCL
  Adafruit_L3GD20 gyro(GYRO_CS, GYRO_DO, GYRO_DI, GYRO_CLK);
#endif


#define chipSelect 53                                                               // Set up SD card for Arduino Mega
#define MOSI 51
#define MISO 50
#define SCK 52

/* Assign a unique ID to this sensor at the same time */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);         // Initialise Accelerometer
Adafruit_BMP085 bmp;                                                                // Initialise Pressure Sensor
Adafruit_LSM303DLH_Mag_Unified mag = Adafruit_LSM303DLH_Mag_Unified(12345);         // Initialise Mag Sensor

void setup()
{
  DEBUG_PORT.begin(9600);
  DEBUG_PORT.print( F("NMEAloc.INO: started\n") );
  DEBUG_PORT.print( F("fix object size = ") );
  DEBUG_PORT.println( sizeof(gps.fix()) );
  DEBUG_PORT.print( F("NMEAGPS object size = ") );
  DEBUG_PORT.println( sizeof(gps) );
  DEBUG_PORT.println( F("Looking for GPS device on " GPS_PORT_NAME) );

  #ifdef NMEAGPS_NO_MERGING
    DEBUG_PORT.println( F("Only displaying data from xxRMC sentences.\n  Other sentences may be parsed, but their data will not be displayed.") );
  #endif

  DEBUG_PORT.flush();

  gpsPort.begin(9600);


Serial.print(F("Initializing SD card..."));

if (!SD.begin(chipSelect)) {
    Serial.print(F("SD begin Error"));
    while(1);
  }
  Serial.println(F("card initialized."));

 // Find an unused file name.
  if (BASE_NAME_SIZE > 6) {
    Serial.println(F("FILE_BASE_NAME too long"));
    while(1);
  }
  while (SD.exists(fileName)) {                                       //itterate through filenames until an unused on is found
    if (fileName[BASE_NAME_SIZE + 1] != '9') {
      fileName[BASE_NAME_SIZE + 1]++;
    } else if (fileName[BASE_NAME_SIZE] != '9') {
      fileName[BASE_NAME_SIZE + 1] = '0';
      fileName[BASE_NAME_SIZE]++;
    } else {
      Serial.println(F("Can't create file name"));
    }
  }

  Serial.print(F("Logging to: "));
  Serial.println(fileName);

  File dataFile = SD.open(fileName, FILE_WRITE);
  dataFile.println(Headings);
//  Serial.print(Headings);
  dataFile.close();

  //begin sensors

if (!mag.begin()) {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println(F("Ooops, no LSM303mag detected ... Check your wiring!"));
    while (1);
  }
// Try to initialise and warn if we couldn't detect the chip
if (!gyro.begin(gyro.L3DS20_RANGE_250DPS)){
    Serial.println(F("Oops ... unable to initialize the L3GD20. Check your wiring!"));
    while (1);
    }
if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP085 sensor, check wiring!"));
    while (1) {}
    }

//take a reference pressure to use to measure change in altitude

for(int i =0; i < 50; i++){ 
      p_ref += bmp.readPressure();        //taken during setup calibration
      T_ref += bmp.readTemperature();     //taken during setup calibration  <--- Doesnt seem to work
      Serial.println(F("Pressure Calibration Underway..."));
      //Serial.println(p_ref);
      delay(50);
      }
  p_ref /= 50;
  T_ref /= 50;

  Serial.print("p_ref: " + String(p_ref));
  Serial.print("T_ref: " + String(T_ref));

//take reading of average gyro drift to offset
  
  for (int cal_int = 0; cal_int < 500; cal_int ++){
    gyro.read();
    Serial.println(F("Gyro Calibration Underway..."));
    gyro_x_cal += gyro.data.x;           // define offset values for variables to reduce gyro drift
    gyro_y_cal += gyro.data.y;
    gyro_z_cal += gyro.data.z;
    delay(2);                            // To create 250Hz for loop
    }

  gyro_x_cal /= 500;
  gyro_y_cal /= 500;
  gyro_z_cal /= 500;

  #ifndef ESP8266
  //while (!Serial); // will pause Zero, Leonardo, etc until serial console opens
#endif

  Serial.print("gyro x: " + String(gyro_x_cal));
  Serial.print("gyro y: " + String(gyro_y_cal));
  Serial.print("gyro z: " + String(gyro_z_cal));
  
#ifndef ESP8266
#endif

  /* Initialise the accel */
if (!accel.begin()) {
    /* There was a problem detecting the ADXL345 ... check your connections */
    //Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while (1);
  }

  /* Display some basic information on this sensor */

  accel.setRange(LSM303_RANGE_4G);
  //Serial.print("Range set to: ");
  lsm303_accel_range_t new_range = accel.getRange();
  switch (new_range) {
  case LSM303_RANGE_4G:
    //Serial.println(F("+- 4G"));
    break;
  }
  
  accel.setMode(LSM303_MODE_NORMAL);
 // Serial.print("Mode set to: ");
  lsm303_accel_mode_t new_mode = accel.getMode();
  switch (new_mode) {
  case LSM303_MODE_NORMAL:
    //Serial.println(F("Normal"));
    break;
  }

  
}

//--------------------------

void GPS_data(){
  
  date = "";
  timeString = "";
  locString = "";


  if (gps.available( gpsPort )){
    

  fix = gps.read();
  
  if (fix.valid.location) {
    Serial.print("\nvalid location!");
        
    date += String(fix.dateTime.date, DEC); 
    date += ('-');
    date += String(fix.dateTime.month, DEC); 
    date += ("-20");
    date += String(fix.dateTime.year, DEC);
    date += ",";
    Serial.print("\nDate: " + date);
    



    if (fix.dateTime.hours < 10){timeString += "0";}
    int GPS_hour = fix.dateTime.hours + 1;
    timeString += String(GPS_hour, DEC);
    timeString += ":";
//    Serial.print("\n\n\n\nhours " + timeString);
    if (fix.dateTime.minutes < 10){timeString += "0";}
    timeString += String(fix.dateTime.minutes, DEC);
    timeString += ":";
//    Serial.print("\n\nHours:Minutes " + timeString);
    if (fix.dateTime.seconds < 10){timeString += "0";}
    timeString += String(fix.dateTime.seconds, DEC);
    Serial.print("\nTimeString: " + timeString);

    locString += String(fix.latitude(), 6 ); // floating-point display
   locString += ( ',' );
     locString += String(fix.longitude(), 6 ); // floating-point display
    Serial.print("\nlocString: " + locString);
    
    
    if (fix.valid.satellites)                                                                                                                                                                                                       
    gnd_spd = String( (fix.speed_kph(), 6)/3.6 );
    Serial.print("\ngnd_spd: " + gnd_spd);

    gps_alt = String(fix.altitude()); 
    Serial.print("\ngps_alt: " + gps_alt);
    Serial.print("satelites: ");
    Serial.print( fix.satellites );
    
    }
    
  }

}

void logger()
{
   File dataFile = SD.open(fileName, FILE_WRITE);
    dataString = "";
    dataString += date;
    dataString += ",";
    dataString += timeString;
    dataString += ",";
    dataString += locString;
    dataString += ",";
    dataString += gps_alt;
    dataString += ",";
    dataString += gnd_spd;
    dataString += (", ");
    dataString += (angle_roll);
    dataString += (", ");
    dataString += (angle_pitch);
    Serial.println("\n\ndataString: " + dataString);

dataFile.println(dataString);

dataFile.close();
  
  }



void loop()
{
gyro.read();

 // Subtract offset values from raw gyro input to reduce drift
  gyro.data.x -= gyro_x_cal;
  gyro.data.y -= gyro_y_cal;
  gyro.data.z -= gyro_z_cal;

 // 120/803 is used as this is the time(s) between readings
  angle_pitch += gyro.data.x * 120/803;                                    //Calculate the traveled pitch angle and add this to the angle_pitch variable
  angle_roll  += gyro.data.y * 120/803;                                   //Calculate the traveled roll angle and add this to the angle_roll variable

  //0.00261 = 120/803 * (3.142(PI) / 180degr) The Arduino sin function is in radians 
  angle_pitch += angle_roll * sin(gyro.data.z * 0.0026082131);               //If the IMU has yaw, transfer the roll angle to the pitch angel
  angle_roll -= angle_pitch * sin(gyro.data.z * 0.0026082131);

//Serial.print(F(" \nGyro Pitch Angle: ")); Serial.print(" "); Serial.print(angle_roll);   Serial.print(F(" "));
//Serial.print(F(" \nGyro Roll Angle: ")); Serial.print(F(" "));  Serial.println(angle_pitch);

/////////////////////////////////////////////////////////////////////////////////////////////
// ACCELEROMETER
  
  /* Get a new sensor event */
  sensors_event_t event;
  accel.getEvent(&event);

  //Accelerometer angle calculations
  acc_total_vector = sqrt((event.acceleration.x*event.acceleration.x)+(event.acceleration.y*event.acceleration.y)+(event.acceleration.z*event.acceleration.z));  //Calculate the total accelerometer vector
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians this is to return it to degrees
  angle_roll_acc = asin((float)event.acceleration.x/acc_total_vector)* 57.296;         //Calculate the roll angle
  angle_pitch_acc = asin((float)event.acceleration.y/acc_total_vector)* 57.296;       //Calculate the pitch angle
  
  //two lines for calibration
  angle_pitch_acc -=0.9;                                                 //Accelerometer calibration value for pitch
  angle_roll_acc += 1;  

if(set_gyro_angles){                                                 //If the IMU is already started
    angle_pitch = angle_pitch * 0.96 + angle_pitch_acc * 0.04;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    angle_roll = angle_roll * 0.96 + angle_roll_acc * 0.04;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
  }
  else{                                                                //At first start
    angle_pitch = angle_pitch_acc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle 
    angle_roll = angle_roll_acc;                                       //Set the gyro roll angle equal to the accelerometer roll angle 
    set_gyro_angles = true;                                            //Set the IMU started flag
  }
  
  GPS_data();
  logger();

n++;
   Serial.print(" ");



}
