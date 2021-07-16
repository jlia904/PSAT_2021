#include <SD.h> //SD card
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h> //Temp and pressure sensor
#include <RTClib.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>

RTC_PCF8523 rtc;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

//Definitions for Temp and pressure sensor
#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)

Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);


Adafruit_BMP280 bmp; // I2C

//Definitions for GPS
#define GPSSerial Serial1

//file to write to
File myFile;
File dataFile;

//variable names
float tempC;  // Variable for holding temp in C
float pressure; //Variable for holding pressure reading
float altitude;
int trig = 15;
bool flight = false;
bool altset = false;
float basealt;
float prevalt = -500.0;
float altdiff;
int count = 0;
bool photo500 = false;
bool photo300 = false;
bool recording = false;
int video = 0;
float xaccel;
float yaccel;
float zaccel;
float xmag;
float ymag;
float zmag;
float xgyro;
float ygyro;
float zgyro;

void setup() {

  //while (!Serial);....................................................................................

  // make this baud rate fast enough to we aren't waiting on it
  Serial.begin(115200);

  // 9600 baud is the default rate for the Ultimate GPS
  GPSSerial.begin(9600);

  //temp and pressure setup
  if (!bmp.begin()) {
    while (1);
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500);

  if (!SD.begin(10)) {
    while (1);
  }

  if (! rtc.begin()) {
    while (1);
  }
  pinMode(trig, OUTPUT);
  digitalWrite(trig, HIGH);
  delay(500);

}

void loop() {

  DateTime now = rtc.now();

  //temp and pressure read
  tempC = bmp.readTemperature();
  pressure = bmp.readPressure();
  altitude = bmp.readAltitude(1013.25);
  if (!altset) {
    basealt = altitude;
    altset = true;
  }
  altitude = altitude - basealt;
  //Serial.println(altitude);..........................................................................................


  if (altitude > 0.5) { //make this 10 metres for actual launch
    flight = true;
  }

  if (flight) {
    //Serial.println("in flight");.........................................................................................
    dataFile = SD.open("gpsdata.txt", FILE_WRITE);
    dataFile.print(now.hour(), DEC);
    dataFile.print(':');
    dataFile.print(now.minute(), DEC);
    dataFile.print(':');
    dataFile.println(now.second(), DEC);
    while (GPSSerial.available()) {
      char c = GPSSerial.read();
      dataFile.print(c);
      //Serial.print(c);
    }
    dataFile.close();

    sensors_event_t event;
    accel.getEvent(&event);
    xaccel = event.acceleration.x;
    yaccel = event.acceleration.y;
    zaccel = event.acceleration.z;
    mag.getEvent(&event);
    xmag = event.magnetic.x;
    ymag = event.magnetic.y;
    zmag = event.magnetic.z;
    gyro.getEvent(&event);
    xgyro = event.gyro.x;
    ygyro = event.gyro.y;
    zgyro = event.gyro.z;

    //SD writing
    myFile = SD.open("data2.txt" , FILE_WRITE );
    if (myFile) {
      myFile.print(now.hour(), DEC);
      myFile.print(':');
      myFile.print(now.minute(), DEC);
      myFile.print(':');
      myFile.println(now.second(), DEC);
      myFile.print("Temperature = ");
      myFile.println(tempC);
      myFile.print("Pressure = ");
      myFile.println(pressure);
      myFile.print(" Altitude = ");
      myFile.println(altitude);
      myFile.printf("ACCEL X: %f Y: %f Z: %f", xaccel, yaccel, zaccel);
      myFile.println(" ");
      myFile.printf("MAG X: %f Y: %f Z: %f", xmag, ymag, zmag);
      myFile.println(" ");
      myFile.printf("GYRO X: %f Y: %f Z: %f", xgyro, ygyro, zgyro);
      myFile.println();
      myFile.close();

    }

  }
  if (flight && (prevalt > altitude)) {
    if ((altitude < 1.5) && !photo500) {
      takephoto();
      photo500 = true;
    }
    if ((altitude < 1.0 ) && !photo300) {
      takephoto();
      photo300 = true;
      delay(500);
      digitalWrite(trig, LOW);
    }
    if ((altitude < 0.8 ) && !recording) {
      if (video == 0) {
        takevideo();
        recording = true;
      }
    }

  }
  if (recording && !flight) {
    takevideo();
    recording = false;
    video++;
  }
  
  // stopping sequence
  altdiff = altitude - prevalt;
  if (altdiff < 0.1) {
    count++;
    if (count > 5) {
      flight = false;
    }
  }
  else{
    count = 0;
  }


  prevalt = altitude;

  delay(1000);

}

void takephoto() {
  // camera taking photos every second
  digitalWrite(trig, LOW);
  delay(50);
  digitalWrite(trig, HIGH);
}

void takevideo() {
  digitalWrite(trig, HIGH);
  delay(50);
  digitalWrite(trig, LOW);

}
