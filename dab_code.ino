#include <Adafruit_BME280.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_GPS.h>
#include <Adafruit_Sensor.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <utility/imumaths.h> //bno
#include <SD.h>
#include <SPI.h>
#include <Wire.h>

File BMEData;
File BNOData;
File GPSData;

const int chipSelect = BUILTIN_SDCARD;

#define SEALEVELPRESSURE_HPA (1013.25)

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11 
#define BME_CS 10

#define BNO055_SAMPLERATE_DELAY_MS 100 //sample rate delay

Adafruit_BNO055 bno = Adafruit_BNO055(55); //I2C
Adafruit_BME280 bme; // I2C

HardwareSerial GPSSerial = Serial5; //GPSSerial teensy hardware serial port i believe
Adafruit_GPS GPS(&GPSSerial); //GPS on the hardware port
#define GPSECHO true


uint32_t timer = millis(); //timer

IntervalTimer myTimer1;
IntervalTimer myTimer2;
IntervalTimer myTimer3;

void setup()
{
    Serial.begin(9600);
    Serial.println("Adafruit GPS library test!");
    
	while (!Serial) {;
    }
	
  Serial.print("Initializing SD card...");
    
	if (!SD.begin(chipSelect)) {
        Serial.println("initialization failed!");
        return;
    }
	Serial.println("initialization done.");

BNOData = SD.open("BNO.txt", FILE_WRITE);
   if (BNOData) {
        Serial.print("Writing to test.txt...");
        BNOData.println("BNO data Write");
        BNOData.println("orientation,gyroscope,accelerometer,LinearAcceleration");
        BNOData.close(); // close the file:
        Serial.println("BNO data done.");
    }else {
        Serial.println("error opening BNO.txt");
    }    
BMEData = SD.open("BME.txt", FILE_WRITE);
    if (BMEData) {
        Serial.print("Writing to test.txt...");
        BMEData.println("BME data Write");
        BMEData.close();
        Serial.println("BME data done. Temprature (C), Pressure (hPa), Altitude (m), Humidity (%)");
    } else {
        Serial.println("error opening BME.txt");
    }
    GPSData = SD.open("GPS.txt", FILE_WRITE);
    if (GPSData) {
        Serial.print("Writing to test.txt...");
        GPSData.println("GPS data Write");
        GPSData.close();
        Serial.println("GPS data done.");
    } else {
        Serial.println("error opening GPS.txt");
    }

    GPS.begin(9600);
    //  uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    // uncomment this line to turn on only the "minimum recommended" data
    //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
    // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
    // the parser doesn't care about other sentences at this time
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate For parsing, we don't suggest using anything higher than 1 Hz
    GPS.sendCommand(PGCMD_ANTENNA); // Request updates on antenna status. comment to keep out.

    delay(1000);

    GPSSerial.println(PMTK_Q_RELEASE);
    
    Serial.println("Orientation Sensor Test"); Serial.println("");

    if (!bno.begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        return;//while (1);
    }

    delay(1000);

    /* Display some basic information on this sensor */
    displaySensorDetails();

    /* Optional: Display current status */
    displaySensorStatus();

    bno.setExtCrystalUse(true);

    //Serial.begin(9600); //redundant
    Serial.println(F("BME280 test"));

    //bool status;

    // default settings
    // (you can also pass in a Wire library object like &Wire2)
    //status = bme.begin();
    if (!bme.begin()){//!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        return; //while (1);
    }
    else{
    Serial.println("-- Default Test --");
    //    delayTime = 1000;

    Serial.println();

    delay(1000); // let sensor boot up

    myTimer1.begin(readBNO, 1000);  // readBNO to run every 1 seconds

    delay(1000); // let sensor boot up

    myTimer2.begin(readGPS, 1000);  // readGPS to run every 1 seconds

    delay(1000); // let sensor boot up

    myTimer3.begin(readBME, 1000);  // readBME to run every 1 seconds
    }
}

void loop() // run over and over again
{

}

void displaySensorDetails() //(void)
{
    sensor_t sensor;
    bno.getSensor(&sensor);
    Serial.println("------------------------------------");
    Serial.print  ("Sensor:       ");
    Serial.println(sensor.name);
    Serial.print  ("Driver Ver:   ");
    Serial.println(sensor.version);
    Serial.print  ("Unique ID:    ");
    Serial.println(sensor.sensor_id);
    Serial.print  ("Max Value:    ");
    Serial.print(sensor.max_value);
    Serial.println(" xxx");
    Serial.print  ("Min Value:    ");
    Serial.print(sensor.min_value);
    Serial.println(" xxx");
    Serial.print  ("Resolution:   ");
    Serial.print(sensor.resolution);
    Serial.println(" xxx");
    Serial.println("------------------------------------");
    Serial.println("");
    delay(1000);
}

void displaySensorStatus()//(void)
{
    /* Get the system status values (mostly for debugging purposes) */
    uint8_t system_status, self_test_results, system_error;
    system_status = self_test_results = system_error = 0;
    bno.getSystemStatus(&system_status, &self_test_results, &system_error);

    /* Display the results in the Serial Monitor */
    Serial.println("");
    Serial.print("System Status: 0x");
    Serial.println(system_status, HEX);
    Serial.print("Self Test:     0x");
    Serial.println(self_test_results, HEX);
    Serial.print("System Error:  0x");
    Serial.println(system_error, HEX);
    Serial.println("");
    delay(1000);
}

void displayCalStatus() //(void)
{
    /* Get the four calibration values (0..3) */
    /* Any sensor data reporting 0 should be ignored, */
    /* 3 means 'fully calibrated" */
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);

    /* The data should be ignored until the system calibration is > 0 */
    if (!system)
    {
        Serial.println("! ");
    }
	else 
	{
		/* Display the individual values */
		Serial.print("Sys:");
		Serial.print(system, DEC);
		Serial.print(" G:");
		Serial.print(gyro, DEC);
		Serial.print(" A:");
		Serial.print(accel, DEC);
		Serial.print(" M:");
		Serial.println(mag, DEC);
	}
}

void readBNO() {//later
    BNOData = SD.open("BNO.txt", FILE_WRITE);
    /* Get a new sensor event */
    sensors_event_t event;
    bno.getEvent(&event);
    double dataX = event.orientation.x;
    double dataY = event.orientation.y;
    double dataZ = event.orientation.z;
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> Accel_Meter = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> Linear_Accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    /* Display the floating point data */
    Serial.print("Orientation X: ");
    Serial.print(dataX, 4);
    Serial.print("\tY: ");
    Serial.print(dataY, 4);
    Serial.print("\tZ: ");
    Serial.print(dataZ, 4);
    Serial.println("");
    Serial.print("Gyro X: ");
    Serial.print(gyro.x());
    Serial.print("\tY: ");
    Serial.print(gyro.y());
    Serial.print("\tZ: ");
    Serial.print(gyro.z());
    Serial.println("");
    Serial.print("Accel_Meter X: ");
    Serial.print(Accel_Meter.x());
    Serial.print("\tY: ");
    Serial.print(Accel_Meter.y());
    Serial.print("\tZ: ");
    Serial.print(Accel_Meter.z());
    Serial.println("");
    Serial.print("Linear_Accel X: ");
    Serial.print(Accel_Meter.x());
    Serial.print("\tY: ");
    Serial.print(Accel_Meter.y());
    Serial.print("\tZ: ");
    Serial.print(Accel_Meter.z());
    Serial.println("");
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
    BNOData.print(Accel_Meter.x());
    BNOData.print(",");
    BNOData.print(Accel_Meter.y());
    BNOData.print(",");
    BNOData.print(Accel_Meter.z());
    BNOData.print(",");
    BNOData.print(Accel_Meter.x());
    BNOData.print(",");
    BNOData.print(Accel_Meter.y());
    BNOData.print(",");
    BNOData.println(Accel_Meter.z());
    /* Optional: Display calibration status */
    displayCalStatus();

    /* Optional: Display sensor status (debug only) */
    //displaySensorStatus();
    /* Wait the specified delay before requesting nex data */
    delay(BNO055_SAMPLERATE_DELAY_MS);

    BNOData.close();
}
void readGPS() {                                                                                                                                                                           
    GPSData = SD.open("GPS.txt", FILE_WRITE);
    char c = GPS.read();
	if(c != 0)
		GPSSerial.print(c);
    // if a sentence is received, we can check the checksum, parse it...
    
	
	if (GPS.newNMEAreceived()) {
        // a tricky thing here is if we print the NMEA sentence, or data
        // we end up not listening and catching other sentences!
        // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
        Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
        if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
            return; // we can fail to parse a sentence in which case we should just wait for another
    }
    	
    if (timer > millis()) timer = millis();//prevent overflow timer error

    // approximately every 2 seconds or so, print out the current stats
    if (millis() - timer > 2000) {
        timer = millis(); // reset the timer
        Serial.print("\nTime: ");
        Serial.print(GPS.hour, DEC);
        Serial.print(':');
        Serial.print(GPS.minute, DEC);
        Serial.print(':');
        Serial.print(GPS.seconds, DEC);
        Serial.print('.');l
        Serial.println(GPS.milliseconds);
        Serial.print("Date: ");
        Serial.print(GPS.day, DEC);
        Serial.print('/');
        Serial.print(GPS.month, DEC);
        Serial.print("/20");
        Serial.println(GPS.year, DEC);
        Serial.print("Fix: ");
        Serial.print((int)GPS.fix);
        Serial.print(" quality: ");
        Serial.println((int)GPS.fixquality);
        if (GPS.fix) {
            Serial.print("Location: ");
            Serial.print(GPS.latitude, 4);
            Serial.print(GPS.lat);
            Serial.print(", ");
            Serial.print(GPS.longitude, 4);
            Serial.println(GPS.lon);
            GPSData.print("Location: ");
            GPSData.print(GPS.latitude, 4);
            GPSData.print(GPS.lat);
            GPSData.print(", ");
            GPSData.print(GPS.longitude, 4);
            GPSData.println(GPS.lon);
            Serial.print("Speed (knots): ");
            Serial.println(GPS.speed);
            Serial.print("Angle: ");
            Serial.println(GPS.angle);
            Serial.print("Altitude: ");
            Serial.println(GPS.altitude);
            Serial.print("Satellites: ");
            Serial.println((int)GPS.satellites);
            GPSData.close();
   }
    else
      GPSData.println(".");
  }
}
void readBME() {
  BMEData = SD.open("BME.txt", FILE_WRITE);
  double temperature = bme.readTemperature();
  double pressure = bme.readPressure();
  double alt = bme.readAltitude(SEALEVELPRESSURE_HPA);
  double humidity = bme.readHumidity();
    Serial.print("Temperature = ");
    Serial.print(temperature);
    Serial.println(" *C");
    Serial.print("Pressure = ");
    Serial.print(pressure / 100.0F);
    Serial.println(" hPa");
    Serial.print("Approx. Altitude = ");
    Serial.print(alt);
    Serial.println(" m");
    Serial.print("Humidity = ");
    Serial.print(humidity);
    Serial.println(" %");
    Serial.println();

    BMEData.print(temperature);
    BMEData.print(",");
    BMEData.print(pressure/ 100.0F);
    BMEData.print(",");
    BMEData.print(alt);
    BMEData.print(",");
    BMEData.println(humidity);
    BMEData.close();
}

//????????
/*
xbee.print(BMEData);
xbee.print(BNOData);
xbee.print(GPSData);
*/
