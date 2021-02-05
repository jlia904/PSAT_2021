#include <SD.h> //SD card
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h> //Temp and pressure sensor
#include <RTClib.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>
#include <Adafruit_LSM6DS33.h>
#include <Adafruit_LIS3MDL.h>

RTC_PCF8523 rtc;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

//Definitions for Temp and pressure sensor
#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)


Adafruit_BMP280 bmp; // I2C
Adafruit_LSM6DS33 lsm6ds;
Adafruit_LIS3MDL lis3mdl;

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
bool video_taken = false;
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

//VOLTAGE ADDITION
byte voltPin = A0;
float vread;
float voltage;

// *****************************************************************************
// Definitions for GPS
// *****************************************************************************
#define GPSSerial Serial1
#define BUFFER_SIZE 11

enum GPS_state {
  UNKNOWN_STATE,
  START_OF_SENTENCE,
  MID_SENTENCE
};

enum sentence_type {
  UNKNOWN_SENTENCE,
  GPGGA,
  GPGSA,
  GPGSV,
  GPRMC,
  GPVTG
};

typedef struct gpsData_t {

  // GPGGA
  uint32_t UTC_Time;
  uint32_t Latitude;
  uint8_t NS_Indicator;
  uint32_t Longitude;
  uint8_t EW_Indicator;
  uint32_t GPS_Quality_Indicator;
  uint32_t Altitude;

  // GPGSV
  uint32_t Satellites_In_View;
  uint32_t Satellite_1_ID;
  uint32_t Satellite_1_SNR;
  uint32_t Satellite_2_ID;
  uint32_t Satellite_2_SNR;
  uint32_t Satellite_3_ID;
  uint32_t Satellite_3_SNR;
  uint32_t Satellite_4_ID;
  uint32_t Satellite_4_SNR;
  uint32_t Satellite_5_ID;
  uint32_t Satellite_5_SNR;
  uint32_t Satellite_6_ID;
  uint32_t Satellite_6_SNR;
  uint32_t Satellite_7_ID;
  uint32_t Satellite_7_SNR;
  uint32_t Satellite_8_ID;
  uint32_t Satellite_8_SNR;
  uint32_t Satellite_9_ID;
  uint32_t Satellite_9_SNR;
  uint32_t Satellite_10_ID;
  uint32_t Satellite_10_SNR;
  uint32_t Satellite_11_ID;
  uint32_t Satellite_11_SNR;
  uint32_t Satellite_12_ID;
  uint32_t Satellite_12_SNR;

  // GPRMC
  uint32_t Course_Over_Ground;

  // GPGVTG
  uint32_t Speed_Over_Ground_Kmh;

} gpsData_t;


// *****************************************************************************
// Part of fake uart
// *****************************************************************************

int uart_empty(void) {
  return 0;
}


// *****************************************************************************
// actual code
// *****************************************************************************

// This is a heavily modified version of the msp430 uart_recv function, this should work for psat
uint8_t uart_recv2(uint8_t *rx_buffer, uint8_t rx_max) {
  while (!GPSSerial.available()) {
  }
  *rx_buffer = GPSSerial.read();
  return 0;
}

// Takes a character as input and classifies it for converting sentence character to number later
int charTypeClassify(uint8_t character) {
  switch (character) {
    case '0': case '1': case '2': case '3': case '4': case '5': case '6': case '7': case '8': case '9':
      return 0;
    default:
      return 1;
  }
}

// Takes a character as input and return its number
int charToNum(uint8_t character) {
  switch (character) {
    case '0':
      return 0;
    case '1':
      return 1;
    case '2':
      return 2;
    case '3':
      return 3;
    case '4':
      return 4;
    case '5':
      return 5;
    case '6':
      return 6;
    case '7':
      return 7;
    case '8':
      return 8;
    case '9':
      return 9;
    default:
      return 0;
  }
}

// Function that convert generic numerical struct array data from dataObject to reducedDataObject
uint32_t reduce_data(uint8_t* dataField, int size) {

  int idx;
  int charType;
  uint32_t multiplier = 1;
  uint32_t num = 0;

  // loop through the array
  for (idx = size - 1; idx >= 0; idx--) {
    // Check the character type
    charType = charTypeClassify(dataField[idx]);

    // If the character is a number, then scale it and store it
    if (charType == 0) {
      num += charToNum(dataField[idx]) * multiplier;
      multiplier *= 10;
    }
  }

  return num;
}


int get_digit_no(uint32_t num) {

  int count = 0;
  // Run loop till num is greater than 0
  do
  {
    // Increment digit count
    count++;

    //Remove last digit of 'num'
    num /= 10;
  } while (num != 0);

  return count;
}

uint32_t round_nearest_int(float num) {
  return (uint32_t)(num + 0.5);
}

uint32_t raise_to_power(int base, int pow) {

  uint32_t result = 1;
  for (pow; pow > 0; pow--)
  {
    result = result * base;
  }
  return result;
}

// Stores character data from character form to the integer form in the data struct
void gps_ascii_to_int(uint8_t *src, uint8_t *dst, uint8_t ascii) {
  if (*src == ascii) {
    *dst = 1;
  }
  else {
    *dst = 0;
  }
}

// Stores numerical data from character form to the integer form in the data struct
void gps_float_to_int(uint8_t *src, uint32_t *dst, int digits) {
  uint32_t val = reduce_data(src, BUFFER_SIZE);
  int curr_digits = get_digit_no(val);
  if (curr_digits > digits) {
    val = round_nearest_int(val / (float)raise_to_power(10, curr_digits - digits));
  }
  *dst = val;
}

// This function decides which struct field to store the data to depending on the sentence type and field count
void parse_field_data(enum sentence_type* type, uint8_t* field, uint8_t* sequence, uint8_t* buffer, gpsData_t* data) {

  // initialise pointers to data struct fields
  uint32_t* First_Satellite_ID = NULL;
  uint32_t* First_Satellite_SNR = NULL;
  uint32_t* Second_Satellite_ID = NULL;
  uint32_t* Second_Satellite_SNR = NULL;
  uint32_t* Third_Satellite_ID = NULL;
  uint32_t* Third_Satellite_SNR = NULL;
  uint32_t* Fourth_Satellite_ID = NULL;
  uint32_t* Fourth_Satellite_SNR = NULL;

  switch (*type) {
    case GPGGA:
      switch (*field) {
        case 1:
          gps_float_to_int(buffer, &(data->UTC_Time), 9);
          break;
        case 2:
          gps_float_to_int(buffer, &(data->Latitude), 8);
          break;
        case 3:
          gps_ascii_to_int(buffer, &(data->NS_Indicator), 'N');
          break;
        case 4:
          gps_float_to_int(buffer, &(data->Longitude), 9);
          break;
        case 5:
          gps_ascii_to_int(buffer, &(data->EW_Indicator), 'E');
          break;
        case 6:
          gps_float_to_int(buffer, &(data->GPS_Quality_Indicator), 1);
          break;
        case 9:
          gps_float_to_int(buffer, &(data->Altitude), 6);
          break;
        default:
          break;
      }
      break;
    case GPGSA:
      break;

    case GPGSV:

      // Get the sequence number if the we are currently on sequence number field
      if (*field == 2) {
        *sequence = reduce_data(buffer, BUFFER_SIZE);
      }
      // Set the pointers to data struct fields depending on the sequence number
      switch (*sequence) {
        case 1:
          First_Satellite_ID = &(data->Satellite_1_ID);
          First_Satellite_SNR = &(data->Satellite_1_SNR);
          Second_Satellite_ID = &(data->Satellite_2_ID);
          Second_Satellite_SNR = &(data->Satellite_2_SNR);
          Third_Satellite_ID = &(data->Satellite_3_ID);
          Third_Satellite_SNR = &(data->Satellite_3_SNR);
          Fourth_Satellite_ID = &(data->Satellite_4_ID);
          Fourth_Satellite_SNR = &(data->Satellite_4_SNR);
          break;
        case 2:
          First_Satellite_ID = &(data->Satellite_5_ID);
          First_Satellite_SNR = &(data->Satellite_5_SNR);
          Second_Satellite_ID = &(data->Satellite_6_ID);
          Second_Satellite_SNR = &(data->Satellite_6_SNR);
          Third_Satellite_ID = &(data->Satellite_7_ID);
          Third_Satellite_SNR = &(data->Satellite_7_SNR);
          Fourth_Satellite_ID = &(data->Satellite_8_ID);
          Fourth_Satellite_SNR = &(data->Satellite_8_SNR);
          break;
        case 3:
          First_Satellite_ID = &(data->Satellite_9_ID);
          First_Satellite_SNR = &(data->Satellite_9_SNR);
          Second_Satellite_ID = &(data->Satellite_10_ID);
          Second_Satellite_SNR = &(data->Satellite_10_SNR);
          Third_Satellite_ID = &(data->Satellite_11_ID);
          Third_Satellite_SNR = &(data->Satellite_11_SNR);
          Fourth_Satellite_ID = &(data->Satellite_12_ID);
          Fourth_Satellite_SNR = &(data->Satellite_12_SNR);
          break;
        default:    // quit parsing if the sequence number is not one of the expected
          return;
      }

      // parse the field according to the field count
      switch (*field) {
        case 3:
          gps_float_to_int(buffer, &(data->Satellites_In_View), 2);
          break;
        case 4:
          gps_float_to_int(buffer, First_Satellite_ID, 2);
          break;
        case 7:
          gps_float_to_int(buffer, First_Satellite_SNR, 2);
          break;
        case 8:
          gps_float_to_int(buffer, Second_Satellite_ID, 2);
          break;
        case 11:
          gps_float_to_int(buffer, Second_Satellite_SNR, 2);
          break;
        case 12:
          gps_float_to_int(buffer, Third_Satellite_ID, 2);
          break;
        case 15:
          gps_float_to_int(buffer, Third_Satellite_SNR, 2);
          break;
        case 16:
          gps_float_to_int(buffer, Fourth_Satellite_ID, 2);
          break;
        case 19:
          gps_float_to_int(buffer, Fourth_Satellite_SNR, 2);
          break;
        default:
          break;
      }
      break;
    case GPRMC:
      switch (*field) {
        case 8:
          gps_float_to_int(buffer, &(data->Course_Over_Ground), 4);
          break;
        default:
          break;
      }
      break;
    case GPVTG:
      switch (*field) {
        case 7:
          gps_float_to_int(buffer, &(data->Speed_Over_Ground_Kmh), 5);
          break;
        default:
          break;
      }
      break;
    default:
      break;

  }
}

void parse_gps_data(enum GPS_state* state, enum sentence_type* type, uint8_t* idx, uint8_t* field, uint8_t* sequence, uint8_t* buffer, gpsData_t* data) {

  uint8_t byte = 0;

  switch (*state) {
    case UNKNOWN_STATE:

      if (uart_empty()) {
        return;
      }

      // Get a byte
      uart_recv2(&byte, 1);

      // If the byte match the character at the start, initialise to the START_OF_SENTENCE state
      if (byte == '$') {
        *state = START_OF_SENTENCE;
        *idx = 0;
        *field = 0;
        *sequence = 0;
        memset(buffer, '#', BUFFER_SIZE);
        buffer[(*idx)++] = byte;
      }

      break;
    case START_OF_SENTENCE:
      while (!uart_empty() && byte != ',' && *idx < 7) {
        uart_recv2(&byte, 1);
        buffer[(*idx)++] = byte;
      }

      // Currently there is nothing in the buffer
      if (uart_empty()) {
        return;
      }

      // start of sentence is not received properly and restart the algorithm
      if (*idx > 7) {
        *state = UNKNOWN_STATE;
        return;
      }

      // Setting the sentence type
      switch (buffer[4]) {
        case 'G':
          *type = GPGGA;
          break;
        case 'S':
          if (buffer[5] == 'A') {
            *type = GPGSA;
          }
          else if (buffer[5] == 'V') {
            *type = GPGSV;
          }
          else {
            *type = UNKNOWN_SENTENCE;
          }
          break;
        case 'M':
          *type = GPRMC;
          break;
        case 'T':   // This is GPVTG
          *type = GPVTG;
          break;
        default:    // This is not one of the expected sentence
          *type = UNKNOWN_SENTENCE;
          break;
      }

      // initialise to MID_SENTENCE state
      if (*type != UNKNOWN_SENTENCE) {
        *state = MID_SENTENCE;
        *idx = 0;
        (*field)++;
        *sequence = 0;
        memset(buffer, '#', BUFFER_SIZE);
      }
      else {  // The sentence type is not as expected and restart the algorithm
        *state = UNKNOWN_STATE;
        return;
      }

      break;

    case MID_SENTENCE:

      // get next byte
      while (!uart_empty() && byte != ',' && byte != '*' && *idx < BUFFER_SIZE) {
        uart_recv2(&byte, 1);
        if (byte != '.') {
          buffer[(*idx)++] = byte;
        }
      }

      // Currently there is nothing in the buffer
      if (uart_empty()) {
        return;
      }


      // Set state to UNKNOWN if start of sentence is not received properly
      // This is detected when the last bit of the buffer is set, which shouldn't ever been set
      if (*idx >= BUFFER_SIZE) {
        *state = UNKNOWN_STATE;
        return;
      }

      // update field
      parse_field_data(type, field, sequence, buffer, data);

      if (byte == '*') {
        *state = UNKNOWN_STATE;
        return;
      }

      // prepare for next loop
      *idx = 0;
      (*field)++;
      memset(buffer, '#', BUFFER_SIZE);

      break;
  }

}

uint8_t buffer[BUFFER_SIZE];
gpsData_t data = {0};

enum GPS_state state = UNKNOWN_STATE;
enum sentence_type type = UNKNOWN_SENTENCE;
uint8_t idx;
uint8_t field = 0;
uint8_t sequence = 0;

// *****************************************************************************
// End of definitions for GPS
// *****************************************************************************


void setup() {

  //while (!Serial);....................................................................................

  // make this baud rate fast enough to we aren't waiting on it
  Serial.begin(115200);

  while (!Serial);
  
  // 9600 baud is the default rate for the Ultimate GPS
  GPSSerial.begin(9600);

  //temp and pressure setup
  if (!bmp.begin()) {
    while (1);
  }
  bmp.setSampling();

  if (!SD.begin(10)) {
    while (1);
  }

  if (! rtc.begin()) {
    while (1);
  }
  pinMode(trig, OUTPUT);
  digitalWrite(trig, HIGH);

  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  //VOLTAGE ADDITION
  pinMode(voltPin, INPUT);

  delay(500);


  // *****************************************************************************
  // LSM6DS33_LIS3MDL_9DOF initialisation
  // *****************************************************************************

  if (!lsm6ds.begin_I2C()) {
    while (1);
  }

  if (!lis3mdl.begin_I2C()) {
    while (1);
  }

  lsm6ds.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
  lsm6ds.setAccelDataRate(LSM6DS_RATE_104_HZ);
  lsm6ds.setGyroRange(LSM6DS_GYRO_RANGE_1000_DPS );
  lsm6ds.setGyroDataRate(LSM6DS_RATE_104_HZ);
  lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
  lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);

  lis3mdl.setIntThreshold(500);
  lis3mdl.configInterrupt(false, false, true, // enable z axis
                          true, // polarity
                          false, // don't latch
                          true); // enabled!

  // *****************************************************************************
  // End of LSM6DS33_LIS3MDL_9DOF initialisation
  // *****************************************************************************
  Serial.println("Finished setup");
}

void loop() {
  DateTime now = rtc.now();

  //VOLTAGE ADDITION
  vread = analogRead(voltPin);  //input variable DC voltage
  voltage = vread * (5.0 / 1023.0); //Calculates real world voltage

  //temp and pressure read
  tempC = bmp.readTemperature();
  pressure = bmp.readPressure();
  altitude = bmp.readAltitude(1013.25);
  Serial.println(altitude);
  sensors_event_t accel, gyro, mag, temp_unused;

  //  /* Get new normalized sensor events */
  lsm6ds.getEvent(&accel, &gyro, &temp_unused);
  lis3mdl.getEvent(&mag);

  xaccel = accel.acceleration.x;
  yaccel = accel.acceleration.y;
  zaccel = accel.acceleration.z;
  xgyro = gyro.gyro.x;
  ygyro = gyro.gyro.y;
  zgyro = gyro.gyro.z;
  xmag = mag.magnetic.x;
  ymag = mag.magnetic.y;
  zmag = mag.magnetic.z;

  if (!altset) {
    basealt = altitude;
    altset = true;
  }
  altitude = altitude - basealt;

  Serial.println(basealt);
  Serial.println(altitude);
  Serial.println("");

  if (altitude > 1.8) { //make this 10 metres for actual launch
    flight = true;
  }

  //    dataFile = SD.open("gpsdata.txt", FILE_WRITE);
  //    dataFile.print(now.hour(), DEC);
  //    dataFile.print(':');
  //    dataFile.print(now.minute(), DEC);
  //    dataFile.print(':');
  //    dataFile.println(now.second(), DEC);
  //    while (GPSSerial.available()) {
  //      char c = GPSSerial.read();
  //      dataFile.print(c);
  //      //Serial.print(c);
  //    }
  //    dataFile.close();

  //GPS sentence parsing
  state = UNKNOWN_STATE;

  //Parse 200 data fields
  for (int i = 0; i < 200; i++) {
    parse_gps_data(&state, &type, &idx, &field, &sequence, buffer, &data);
  }

  dataFile = SD.open("gpsdata.txt", FILE_WRITE);
  dataFile.println(data.UTC_Time);
  dataFile.println(data.Latitude);
  dataFile.println(data.NS_Indicator);
  dataFile.println(data.Longitude);
  dataFile.println(data.EW_Indicator);
  dataFile.println(data.GPS_Quality_Indicator);
  dataFile.println(data.Altitude);
  dataFile.println(data.Satellites_In_View);
  dataFile.println(data.Satellite_1_ID);
  dataFile.println(data.Satellite_1_SNR);
  dataFile.println(data.Satellite_2_ID);
  dataFile.println(data.Satellite_2_SNR);
  dataFile.println(data.Satellite_3_ID);
  dataFile.println(data.Satellite_3_SNR);
  dataFile.println(data.Satellite_4_ID);
  dataFile.println(data.Satellite_4_SNR);
  dataFile.println(data.Satellite_5_ID);
  dataFile.println(data.Satellite_5_SNR);
  dataFile.println(data.Satellite_6_ID);
  dataFile.println(data.Satellite_6_SNR);
  dataFile.println(data.Satellite_7_ID);
  dataFile.println(data.Satellite_7_SNR);
  dataFile.println(data.Satellite_8_ID);
  dataFile.println(data.Satellite_8_SNR);
  dataFile.println(data.Satellite_9_ID);
  dataFile.println(data.Satellite_9_SNR);
  dataFile.println(data.Satellite_10_ID);
  dataFile.println(data.Satellite_10_SNR);
  dataFile.println(data.Satellite_11_ID);
  dataFile.println(data.Satellite_11_SNR);
  dataFile.println(data.Satellite_12_ID);
  dataFile.println(data.Satellite_12_SNR);
  dataFile.println(data.Course_Over_Ground);
  dataFile.println(data.Speed_Over_Ground_Kmh);
  dataFile.println("");
  dataFile.close();

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

    //VOLTAGE ADDITION
    myFile.print("Voltage = ");
    myFile.println(voltage);
    myFile.close();
  }

  if (flight) {
    Serial.println("in flight");
    digitalWrite(13, HIGH);
  }
  else {
    Serial.println("NOT in flight");
  }

  if (flight && (prevalt > altitude)) {
    if ((altitude < 1.5) && !photo500) {
      Serial.println("photo500");
      takephoto();
      photo500 = true;
    }
    if ((altitude < 1.0 ) && !photo300) {
      Serial.println("photo300");
      takephoto();
      photo300 = true;
    }
    if (altitude < 0.3 && !video_taken) {
        Serial.println("video30");
        takevideo();
        video_taken = true;
    }

  }

  //  // stopping sequence
  //  altdiff = altitude - prevalt;
  //  if (altdiff < 0.1) {
  //    count++;
  //    if (count > 5) {
  //      flight = false;
  //    }
  //  }
  //  else {
  //    count = 0;
  //  }


  prevalt = altitude;

  delay(1000);

}

void takephoto() {
  // camera taking photos every second
  digitalWrite(trig, LOW);
  delay(50);
  digitalWrite(trig, HIGH);
  delay(1000);
}
void takevideo() {
  digitalWrite(trig, LOW);
  delay(1000);
  digitalWrite(trig, HIGH);
  delay(20000);
  digitalWrite(trig, LOW);
  delay(1000);
  digitalWrite(trig, HIGH);
}
