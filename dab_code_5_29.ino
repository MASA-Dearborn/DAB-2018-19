#include <Adafruit_BME280.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_GPS.h>
#include <Adafruit_Sensor.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <utility/imumaths.h> //bno
#include <SD.h> //use SDFat library to format new SD cards
#include <SPI.h>
#include <XBee.h>

//Teensy 3.5 and 3.6 SD Card
const int chipSelect = BUILTIN_SDCARD;

//create SD card files
File BMEData;
File BNOData;
File GPSData;

#define SEALEVELPRESSURE_HPA (1013.25) //for accurate altitude calulations

//BNO and BME on I2C
Adafruit_BME280 bme;
Adafruit_BNO055 bno = Adafruit_BNO055(55);

#define BME_SCK 19 //BME280 pin config.
#define BME_MOSI 18

#define GPSSerial Serial5 //GPS on hardware serial(UART)
Adafruit_GPS GPS(&GPSSerial);
#define GPSECHO true

XBee xbee = XBee(); //xbee

//timer GPS
uint32_t timer = millis();

//GPSSerial teensy hardware serial port
IntervalTimer myTimer1;
IntervalTimer myTimer2;
IntervalTimer myTimer3;

void setup() {
  Serial.begin(9600);
  //troubleshooting of SD card and sensors
  Serial.print("Initializing SD card...");
  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");
  //test if file was created
  GPSData = SD.open("GPS.txt", FILE_WRITE);
  if (GPSData) {
    Serial.print("Writing to test.txt...");
    GPSData.println("GPS data Write");
    GPSData.close();
    Serial.println("GPS data done.");
  } else {
    Serial.println("error opening GPS.txt");
  }
  BNOData = SD.open("BNO.txt", FILE_WRITE);
  if (BNOData) {
    Serial.print("Writing to test.txt...");
    BNOData.println("BNO data Write");
    BNOData.println("Orientation, Gyroscope, Accelerometer, Linear Acceleration");
    BNOData.close(); // close the file:
    Serial.println("BNO data done.");
  }
  else {
    Serial.println("error opening BNO.txt");
  }
  BMEData = SD.open("BME.txt", FILE_WRITE);
  if (BMEData) {
    Serial.print("Writing to test.txt...");
    BMEData.println("BME data Write");
    BMEData.println("Temprature (C), Pressure (hPa), Altitude (m), Humidity (%)");
    BMEData.close();
  }
  else {
    Serial.println("error opening BME.txt");
  }
  GPSSerial.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate For parsing, we don't suggest using anything higher than 1 Hz
  GPS.sendCommand(PGCMD_ANTENNA); // optional: updates on antenna status
  delay(1000);
  GPSSerial.println(PMTK_Q_RELEASE); //firmware realease info
  //start communication with BNO
  Serial.begin(9600);
  Serial.println("BNO055 Test");
  Serial.println("");
  if (!bno.begin()) {
    delay(500);
    Serial.print("Could not find a BNO055 sensor... Check your wiring or I2C ADDR!");
    return;
  }
  delay(1000);
  Serial.println(F("BME280 Test"));
  if (!bme.begin()) {
    delay(1000);
    Serial.println("Could not find a valid BME280 sensor... Check your wiring or I2C ADDR!");
    return;
  }
  else {
    Serial.println("DEFAULT TEST");
    Serial.println("");
    delay(500); // let sensor boot up
    myTimer1.begin(readBNO, 150000);  // micros: readBNO to run every .15 seconds
    delay(1000); // let sensor boot up
    myTimer2.begin(readGPS, 100);  // micros: readGPS to run every 1 seconds
    delay(1000); // let sensor boot up
    myTimer3.begin(readBME, 150000); // micros: readBME to run every .15 seconds
    Serial1.begin(9600);
    xbee.setSerial(Serial1); //Xbee using Teensy's Serial3
  }
}

void loop() {
}

double t_bnoData[3]; // instantiates empty array, x = 0, y = 1, z = 2 : This array will be combined later into larger array for transmission
/* Add way to put vector data into array for transmission into t_bnoData[] */
void readBNO() {
  File BNOData = SD.open("BNO.txt", FILE_WRITE);
  /* Get a new sensor event
  sensors_event_t event;
  bno.getEvent(&event);
  t_Data[0] = event.orientation.x;  // stores x into position 0
  t_Data[1] = event.orientation.y;  // stores y into position 1
  t_Data[2] = event.orientation.z;  // stores z into position 2

  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> linear = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);


    double t_Data[] = {dataX, dataY, dataZ};
    for (int i = 0; i < sizeof(t_Data) / sizeof(double); i++) {
    Serial.write(t_Data[i]);
    }
  */

  /* Display the floating point data
    Serial.print("Orientation X: ");
    Serial.print(dataX, DEC);
    Serial.print("\tY: ");
    Serial.print(dataY, DEC);
    Serial.print("\tZ: ");
    Serial.print(dataZ, DEC);
    Serial.println("");
    Serial.print("Gyro X: ");
    Serial.print(gyro.x(), DEC);
    Serial.print("\tY: ");
    Serial.print(gyro.y(), DEC);
    Serial.print("\tZ: ");
    Serial.print(gyro.z(), DEC);
    Serial.println("");
    Serial.print("Accel_Meter X: ");
    Serial.print(accel.x(), DEC);
    Serial.print("\tY: ");
    Serial.print(accel.y(), DEC);
    Serial.print("\tZ: ");
    Serial.print(accel.z(), DEC);
    Serial.println("");
    Serial.print("Linear_Accel X: ");
    Serial.print(linear.x(), DEC);
    Serial.print("\tY: ");
    Serial.print(linear.y(), DEC);
    Serial.print("\tZ: ");
    Serial.print(linear.z(), DEC);
    Serial.println("");
    Serial.println();
    //log to SD card
    BNOData.print(dataX, 4);
    BNOData.print(",");
    BNOData.print(dataY, 4);
    BNOData.print(",");
    BNOData.print(dataZ, 4);
    BNOData.print(",");
    BNOData.print(gyro.x());
    BNOData.print(",");
    BNOData.print(gyro.y());
    BNOData.print(",");
    BNOData.print(gyro.z());
    BNOData.print(",");
    BNOData.print(accel.x());
    BNOData.print(",");
    BNOData.print(accel.y());
    BNOData.print(",");
    BNOData.print(accel.z());
    BNOData.print(",");
    BNOData.print(linear.x());
    BNOData.print(",");
    BNOData.print(linear.y());
    BNOData.print(",");
    BNOData.println(linear.z());
    BNOData.close();
  */
}
/*
  union {//union between byte array and XYZ data float(double)
    byte b[4];
    float xyz;
    float gyroscope;
    float acceleration;
    float linear_accel;
  };
  xyz = dataX; //changing the data type
  for (int i = 0; i < 4; i++) {
    message[j] = b[j];
    Serial5.write(b[i]);
  }
  xyz = dataY;
  for (int i = 0; i < 4; i++) {
    message[j+2*4] = b[j];
    Serial5.write(b[i]);
  }
  xyz = dataZ;
  for (int i = 0; i < 4; i++) {
    message[j+3*4] = b[j];
    Serial5.write(b[i]);
  }
  gyroscope = gyro.x();
  for (int i = 0; i < 4; i++) {
    Serial5.write(b[i]);
  }
  gyroscope = gyro.y();
  for (int i = 0; i < 4; i++) {
    Serial5.write(b[i]);
  }
  gyroscope = gyro.z();
  for (int i = 0; i < 4; i++) {
    Serial5.write(b[i]);
  }
  acceleration = accel.x();
  for (int i = 0; i < 4; i++) {
    Serial5.write(b[i]);
  }
  acceleration = accel.y();
  for (int i = 0; i < 4; i++) {
    Serial5.write(b[i]);
  }
  acceleration = accel.z();
  for (int i = 0; i < 4; i++) {
    Serial5.write(b[i]);
  }
  linear_accel = linear.x();
  for (int i = 0; i < 4; i++) {
    Serial5.write(b[i]);
  }
  linear_accel = linear.y();
  for (int i = 0; i < 4; i++) {
    Serial5.write(b[i]);
  }
  linear_accel = linear.z();
  for (int i = 0; i < 4; i++) {
    Serial5.write(byte/*b[4], sizeof(byte));
  }
  delay(100);
  }
*/
void readGPS() {
  File GPSData = SD.open("GPS.txt", FILE_WRITE);
  char c = GPS.read();
  if (GPSECHO)
    if (c) Serial.print(c);
 if (GPS.newNMEAreceived()) {
    Serial.println(GPS.lastNMEA());
    if (!GPS.parse(GPS.lastNMEA()))
    return;
  }
  if (timer > millis()) timer = millis();
  if (millis() - timer > 2000) {
    timer = millis(); // reset the timer
    //show in serial monitor
    Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC); //send time in shorts
    Serial.print(':');
    Serial.print(GPS.minute, DEC);
    Serial.print(':');
    Serial.print(GPS.seconds, DEC);
    Serial.print('.');
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC);
    Serial.print('/');
    Serial.print(GPS.month, DEC);
    Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: ");
    Serial.print((int)GPS.fix);
    Serial.print("\nQuality: ");
    Serial.println((int)GPS.fixquality);
    Serial.print("Location: ");
    Serial.print(GPS.latitude, 4); //float
    Serial.print(GPS.lat);
    Serial.print(", ");
    Serial.print(GPS.longitude, 4);
    Serial.println(GPS.lon, DEC);
    Serial.println();
    //log to sd card
    GPSData.print(GPS.read());
    GPSData.print(GPS.latitude, 4);
    GPSData.print(GPS.lat);
    GPSData.print(GPS.longitude, 4);
    GPSData.println(GPS.lon);
    GPSData.println(GPS.angle);
    GPSData.println(GPS.altitude);
    GPSData.println((int)GPS.satellites);
    GPSData.close();
  }
}

double t_bmeData[4];
void readBME() {
  File BMEData = SD.open("BME.txt", FILE_WRITE);
  //temp. press. alt. and humi. as double
  t_bmeData[0] = bme.readTemperature(); //8 bytes
  t_bmeData[1] = bme.readPressure();
  t_bmeData[2] =  bme.readAltitude(SEALEVELPRESSURE_HPA);
  t_bmeData[3] = bme.readHumidity();
}
//view in serial monitor
/*
  Serial.print("Temperature = ");
  Serial.print(temp);
  Serial.println(" *C");
  Serial.print("Pressure = ");
  Serial.print(pres / 100.0F);
  Serial.println(" hPa");
  Serial.print("Approx. Altitude = ");
  Serial.print(alt);
  Serial.println(" m");
  Serial.print("Humidity = ");
  Serial.print(humid);
  Serial.println(" %");
  Serial.println();
  //log to sd card
  BMEData.print(temp);
  BMEData.print(",");
  BMEData.print(pres / 100.0F);
  BMEData.print(",");
  BMEData.print(alt);
  BMEData.print(",");
  BMEData.print(humid);
  BMEData.close();
  }
  //sleep mode will tell sensors to stop collecting data. don't set the xbee to sleep but stop sending.
  /*void readPacket() {
  xbee.readPacket();
  if (xbee.getResponse().isAvailible()) {
    if (xbee.getResponse() == sleepMode
    GPS.standby();
    bme.setSampling(Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_NONE,   // temperature
                  Adafruit_BME280::SAMPLING_NONE, // pressure
                  Adafruit_BME280::SAMPLING_NONE,   // humidity
                  Adafruit_BME280::FILTER_OFF }
  bno.PWR_Mode();
  backup();
  wakeup();
  else();
  }
  const int SLEEP_RQ = 22; //The Teensy pin to tell the XBee, "Goodnight!"
  delay(3000000); //5 min
  digitalWrite(SLEEP_RQ, LOW);
*/
