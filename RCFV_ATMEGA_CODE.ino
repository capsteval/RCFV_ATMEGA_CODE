#include <Wire.h>;
#include <SoftwareSerial.h>
#include <ArduinoJson.h>
#include <TinyGPSPlus.h>
// StaticJsonDocument<256> doc;
// StaticJsonDocument<512> doc;
SoftwareSerial ArduinoUno(2, 13);

#include <NewPing.h>


// The GPS objects
TinyGPSPlus gps;

SoftwareSerial gpsSerial(7, 8);  //RX TX



// static const int MAX_SATELLITES = 40;
// TinyGPSCustom totalGPGSVMessages(gps, "GPGSV", 1);  // $GPGSV sentence, first element
// TinyGPSCustom messageNumber(gps, "GPGSV", 2);       // $GPGSV sentence, second element
// TinyGPSCustom satsInView(gps, "GPGSV", 3);          // $GPGSV sentence, third element
// TinyGPSCustom satNumber[4];                         // to be initialized later

// TinyGPSCustom snr[4];

int noSatellites = 0;
int noOfActiveSatellites = 0;

// struct
// {
//   bool active;
//   int elevation;
//   int azimuth;
//   int snr;
// } sats[MAX_SATELLITES];


//ultrasonic sensor for water level
#define echoWaterPin 11      // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigWaterPin 10      //attach pin D3 Arduino to pin Trig of HC-SR04
int MAXCONTAINERRANGE = 22;  // maximum height of tank in cm
int MINCONTAINERRANGE = 2;   // muminum level/height of water in tank in cm


int liquidLevelPerc = 0;

//ultrasonic sensor for obstacle
#define echoObstaclePin 6      // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigObstaclePin 9      //attach pin D3 Arduino to pin Trig of HC-SR04
int MAXOBSTACLEDISTANCE = 30;  // Maximum distance to detect obstacles

// NewPing waterSonar(trigWaterPin, echoWaterPin, MAXCONTAINERRANGE),
//WATER AND OBSTACLE ULTRASONIC CONFIG START
#define SONAR_NUM 2  // Number of sensors.

NewPing sonar[SONAR_NUM] = {                               // Sensor object array.
  NewPing(trigWaterPin, echoWaterPin, MAXCONTAINERRANGE),  // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(trigObstaclePin, echoObstaclePin, MAXOBSTACLEDISTANCE)
};
//WATER AND OBSTACLE ULTRASONIC CONFIG END

// VOLTAGE SENSOR DECLARATION SECTION
const int batteryPin = A5;  // Analog pin connected to battery voltage divider
int batteryPercentage;
// VOLTAGE SENSOR DECLARATION SECTION END


//global sensor object
String SensorValuesTelemetry = "{}";
String GPSLocation = "{}";
String obstacleAhead = "{}";

float longitude = 0, latitude = 0;
String obstacleCommand = "";

void setup(void) {

  ArduinoUno.begin(4800);
  Serial.begin(9600);
  gpsSerial.begin(9600);
  //waterSensorSetup
  pinMode(trigWaterPin, OUTPUT);  // Sets the trigPin as an OUTPUT
  pinMode(echoWaterPin, INPUT);   // Sets the echoPin as an INPUT
}

void loop(void) {
  // This sketch displays information every time a new sentence is correctly encoded.
  while (gpsSerial.available() > 0)
    if (gps.encode(gpsSerial.read()))
      displayInfo();

  // If 5000 milliseconds pass and there are no characters coming in
  // over the software serial port, show a "No GPS detected" error
  // if (millis() > 5000 && gps.charsProcessed() < 10) {
  //   Serial.println("No GPS detected");
  //   while (true)
  //     ;
  // };
}
///function to format sending data
void FormatSendingDataObject() {

  // doc["l"]["lg"] = String(longitude);
  // doc["l"]["la"] =  String(latitude);
  // doc["l"]["d"] =  gpsDate;
  // doc["l"]["t"] = gpsTime;
  // doc["l"]["n"] =  String(noOfActiveSatellites);
  // doc["w"] =  String(liquidLevelPerc);
  // doc["b"] =  String(batteryLevel);
  // doc["o"]["cd"] = obstacleCommand;
  // doc["o"]["d"] =  String(obstacleDistance);

  // // Serialize the JSON object to a string
  // String sensorValues;
  // serializeJson(doc, sensorValues);
  //   Serial.print("sensorValues");
  // Serial.println(sensorValues);
  // // SensorValuesTelemetry = "{\"location\":{\"long\":" + String(longitude) + ",\"lat\":" + String(latitude) + ",\"data\":" + String(gpsDate) + ",\"time\":" + String(gpsTime) + ",\"satelliteno\":" + String(noOfActiveSatellites) + "},\"waterlevel\":" + liquidLevelPerc + ",\"batterlevel\":" + batteryLevel + ",\"obstacle\":{\"command\":"+String(obstacleCommand)+",\"distance\":"+ String(obstacleDistance)+"}}";
  // SensorValuesTelemetry = sensorValues;
  // delay(100);
  // SensorValuesTelemetry = "{\"l\":{\"lg\":" +String(batteryLevel)+ ",\"la\":6.90,\"d\":\"04/02/2023\",\"t\":\"08:06:23.00\",\"n\":9,},\"w\":98,\"b\":30,\"o\":{\"cd\":\"stop\",\"d\":32.1}}";
  //   char buffer[256]; // Adjust the buffer size as needed
  // sprintf(buffer, "{\"l\":{\"lg\":%f,\"la\":6.90,\"n\":9},\"w\":98,\"b\":30,\"o\":{\"cd\":\"stop\",\"d\":32.1}}", batteryLevel);
  // SensorValuesTelemetry = buffer;
  // SensorValuesTelemetry = "{\"l\":{\"lg\":\"longm\",\"la\":6.90,\"n\":9},\"w\":98,\"b\":30,\"o\":{\"cd\":\"stop\",\"d\":32.1}}";
  String str1 = "{\"lg\":\"longm\",\"la\":6.90,\"n\":9} ";
  String str2 = "Arduino!";
  str1.concat(str2);
  SensorValuesTelemetry = str1;
}

void SendFormattedJsonObject() {

  //GPS sensor data
  ArduinoUno.print(GPSLocation);
  ArduinoUno.println("\n");
  Serial.print("S_location: ");
  Serial.println(GPSLocation);

  //ReadBatteryLevel (percentage);
  ArduinoUno.print(batteryPercentage);
  ArduinoUno.println("\n");
  Serial.print("S_Battery Percentage: ");
  Serial.println(batteryPercentage);

  // ReadWaterLevel(liquidLevelPerc);
  ArduinoUno.print(liquidLevelPerc);
  ArduinoUno.println("\n");
  Serial.print("S_Liquid level Percentage: ");
  Serial.println(liquidLevelPerc);

  // ReadObstacleSensor();
  ArduinoUno.print(obstacleAhead);
  ArduinoUno.println("\n");
  Serial.print("S_Obstacle: ");
  Serial.println(obstacleAhead);
}
//functions to read from sensors
void displayInfo() {
  StaticJsonDocument<200> locationDoc;
  GPSLocation = "";
  delay(50);
  if (gps.location.isValid()) {
    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 6);
    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 6);
    locationDoc["lg"] = String(gps.location.lng(), 6);
    Serial.print("Altitude: ");
    Serial.println(gps.altitude.meters());
    locationDoc["la"] = String(gps.location.lng(), 6);
    locationDoc["ns"] = "";
  } else {
    Serial.println("Location: Not Available");
  }

  Serial.print("Date: ");
  if (gps.date.isValid()) {
    Serial.print(gps.date.month());
    Serial.print("/");
    Serial.print(gps.date.day());
    Serial.print("/");
    Serial.println(gps.date.year());
  } else {
    Serial.println("Not Available");
  }

  Serial.print("Time: ");
  if (gps.time.isValid()) {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(":");
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(":");
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(".");
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.println(gps.time.centisecond());
  } else {
    Serial.println("Not Available");
  }

  serializeJson(locationDoc, GPSLocation);
  Serial.println();
  Serial.println();
  delay(1000);
  ReadBatteryLevel(batteryPercentage);
  Water_Obstacle_Distance();
  // ReadWaterLevel(liquidLevelPerc);
  // ReadObstacleSensor();
  delay(100);
  FormatSendingDataObject();
  SendFormattedJsonObject();
}
// void ReadGeolocation() {
//   Serial.println("Location1");


//   if (gps.location.isValid()) {
//     Serial.println("Location2");
//     // Latitude in degrees (double)

//     Serial.print("Latitude= ");
//     Serial.print(gps.location.lat(), 6);
//     latitude = gps.location.lat();
//     // Longitude in degrees (double)
//     Serial.print(" Longitude= ");
//     Serial.println(gps.location.lng(), 6);
//     longitude = gps.location.lng();
//   } else {
//     Serial.println("Location: Not Available");
//   }
//   Serial.print("Satellites: ");
//   if (totalGPGSVMessages.isUpdated()) {
//     for (int i = 0; i < 4; ++i) {
//       int no = atoi(satNumber[i].value());
//       // Serial.print(F("SatNumber is ")); Serial.println(no);
//       if (no >= 1 && no <= MAX_SATELLITES) {
//         sats[no - 1].active = true;
//       }
//     }

//     int totalMessages = atoi(totalGPGSVMessages.value());
//     int currentMessage = atoi(messageNumber.value());
//     if (totalMessages == currentMessage) {
//       Serial.print(F("Sats="));
//       Serial.print(gps.satellites.value());
//       Serial.print(F(" Nums="));
//       noOfActiveSatellites = 0;  //start counting active satellites from zero
//       for (int i = 0; i < MAX_SATELLITES; ++i)
//         if (sats[i].active) {
//           noSatellites = i + 1;
//           Serial.print(noSatellites);
//           noOfActiveSatellites++;
//           Serial.print(F(" "));
//         }
//       Serial.print("No of active satelites = ");
//       Serial.println(noOfActiveSatellites);
//       for (int i = 0; i < MAX_SATELLITES; ++i)
//         sats[i].active = false;
//     }
//   }


//   Serial.println("Location4");
// }


// void ReadWaterLevel(int& liquidLevelPerc) {

//   float heightOfWater = waterSonar.ping_cm();
//   delay(200);
//   Serial.print(heightOfWater);
//   Serial.println("cm");
//   if (heightOfWater < MAXCONTAINERRANGE + 100) {  //change if needed
//     int mapVolumePerc = map(heightOfWater, MINCONTAINERRANGE, MAXCONTAINERRANGE, 100, 0);
//     if (mapVolumePerc > -1 && mapVolumePerc < 101) {
//       Serial.print(liquidLevelPerc);
//       Serial.println("%");
//       liquidLevelPerc = mapVolumePerc;
//     }
//   }
// }

int ReadBatteryLevel(int& batteryPercentage) {
  int rawValue = analogRead(batteryPin);  // Read the analog value
  // Convert analog value to voltage (considering voltage divider ratio)
  float voltage = (rawValue / 1024.0) * 5.0;  // Assuming a 5V Arduino
  // Convert voltage to battery percentage (assuming 3.7V LiPo battery)
  // int batteryVoltageMap = map(1, 0, 4.2, 0, 100);
   int batteryVoltageMap = getPercentage(voltage, 0, 4.2);

  Serial.print("Battery Voltage: ");
  Serial.print(voltage, 2);
  Serial.println(" V");
  if (batteryVoltageMap > -1 && batteryVoltageMap < 101) {
    batteryPercentage = batteryVoltageMap;
  }
  delay(3000);  // Delay before taking the next reading
}

// void ReadObstacleSensor() {
//   StaticJsonDocument<200> ObstacleDoc;
//   float obstacleDistance = obstacleSonar.ping_cm();
//   ObstacleDoc["d"] = obstacleDistance;
//   delay(200);
//   Serial.print(obstacleDistance);
//   Serial.println("cm");
//   if (obstacleDistance < MAXOBSTACLEDISTANCE) {
//     // An obstacle is detected within the specified range
//     Serial.println("Obstacle detected!");
//     ObstacleDoc["cd"] = "stop";
//   } else {
//     // No obstacle detected
//     Serial.println("No obstacle.");
//     ObstacleDoc["cd"] = "ggg";
//   }
//   delay(500);
//   serializeJson(ObstacleDoc, obstacleDistance);
// }


void Water_Obstacle_Distance() {
  for (uint8_t i = 0; i < SONAR_NUM; i++) {  // Loop through each sensor and display results.
    delay(50);                               // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
    if (i = 0) {
      // water
      float heightOfWater = sonar[i].ping_cm();
      delay(50);
      Serial.println("Water sensor");
      if (heightOfWater < MAXCONTAINERRANGE + 100) {  //change if needed
        // int mapVolumePerc = map(heightOfWater, MINCONTAINERRANGE, MAXCONTAINERRANGE, 100, 0);
        int mapVolumePerc = getPercentage(heightOfWater, MINCONTAINERRANGE, MAXCONTAINERRANGE );
        if (mapVolumePerc > -1 && mapVolumePerc < 101) {
          liquidLevelPerc = mapVolumePerc;
        }
      }
    } else if (i = 1) {
      //obstacle
      obstacleAhead = "";
      StaticJsonDocument<200> ObstacleDoc;
      float obstacleDistance = sonar[i].ping_cm();
      ObstacleDoc["d"] = obstacleDistance;
      delay(50);
      if (obstacleDistance < MAXOBSTACLEDISTANCE) {
        // An obstacle is detected within the specified range
        Serial.println("Obstacle detected!");
        ObstacleDoc["cd"] = "stop";
      } else {
        // No obstacle detected
        Serial.println("No obstacle.");
        ObstacleDoc["cd"] = "ggg";
      }
      delay(50);
      serializeJson(ObstacleDoc, obstacleAhead);
    }
  }
}

String formatFloat(float number, int decimalPlaces) {
  // Create a string with the desired number of decimal places
  char buffer[20];
  snprintf(buffer, sizeof(buffer), "%.*f", decimalPlaces, number);
  return String(buffer);
}

int getPercentage(float inputValue, float minInput, float maxInput ) {
  // Create a string with the desired number of decimal places
  int percentage = ((inputValue - minInput) / (maxInput - minInput)) * 100;
  return percentage;
}
