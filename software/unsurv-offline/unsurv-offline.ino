/*
  Reading lat and long via UBX binary commands - no more NMEA parsing!
  By: Nathan Seidle
  SparkFun Electronics
  Date: January 3rd, 2019
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.
*/

#include <Wire.h> //Needed for I2C to GPS
#include "BluetoothSerial.h"
#include "LocationUtils.h"

LocationUtils locUtils;


#include "SparkFun_Ublox_Arduino_Library.h" //http://librarymanager/All#SparkFun_Ublox_GPS
SFE_UBLOX_GPS myGPS;
BluetoothSerial ESP_BT;

class SurveillanceCamera{
  public:

  double latitude;
  double longitude;

  String cameraType;
  String id;
  
};

int nearCameraCounter = 0;
SurveillanceCamera nearCameras[5];

long lastTime = 0; //Simple local timer. Limits amount if I2C traffic to Ublox module.

float distance;
SurveillanceCamera currentCamera; 


// populate nearCameras with 5 cameras for debugging
void populateCameras(){
  
  // populate with your own debug coordinates example  {52.51859, 13.37611, lat2.0, lon3.0 ... }
  double latLonPairs[10] = 
  {lat1.0, lon1.0,
  lat2.0, lon2.0,
  lat3.0, lon3.0,
  lat4.0, lon4.0,
  lat5.0, lon5.0};

  SurveillanceCamera tmpCam;
  tmpCam.cameraType = "dome";
  
  for (int i = 0; i < 10; i++) {

    // lat/lon is saved in pairs of two and ordered: lat1,lon1,lat2,lon2

    Serial.println(i);
    Serial.println(latLonPairs[i]);
    
    if (i%2 == 0) {
      tmpCam.latitude = latLonPairs[i];
    } else {
      tmpCam.longitude = latLonPairs[i];
      
      // add cam after every second loop, casting to int floors value
      nearCameras[(int) i / 2] = tmpCam;
      nearCameraCounter++;
      
    }
  }

  nearCameras[0].id = "name1";
  nearCameras[1].id = "name2";
  nearCameras[2].id = "name3";
  nearCameras[3].id = "name4";
  nearCameras[4].id = "name5";
    
  }



void setup()
{
  Serial.begin(9600);
  
  Serial.println("SparkFun Ublox Example");

  // ESP32 Bluetooth name, connect via phone and use a bluetooth terminal app from the app store
  ESP_BT.begin("test");

  // add debugging cameras
  populateCameras();

  Wire.begin();

  if (myGPS.begin() == false) //Connect to the Ublox module using Wire port
  {
    Serial.println(F("Ublox GPS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGPS.saveConfiguration(); //Save the current settings to flash and BBR
}

void loop()
{
  //Query module only every 10 seconds. Doing it more often will just cause I2C traffic.
  //The module only responds when a new position is available
  if (millis() - lastTime > 10000)
  {
    lastTime = millis(); //Update the timer
    
    long latitude = myGPS.getLatitude();
    Serial.print(F("Lat: "));
    Serial.print(latitude);
    ESP_BT.print(F("Lat: "));
    ESP_BT.print(latitude);

    long longitude = myGPS.getLongitude();
    Serial.print(F(" Lon: "));
    Serial.print(longitude);

    ESP_BT.print(F(" Lon: "));
    ESP_BT.print(longitude);

    long altitude = myGPS.getAltitude();
    Serial.print(F(" Alt: "));
    Serial.print(altitude);
    Serial.print(F(" (mm)"));

    ESP_BT.print(F(" Alt: "));
    ESP_BT.print(altitude);

    byte SIV = myGPS.getSIV();
    Serial.print(F(" SIV: "));
    Serial.print(SIV);

    ESP_BT.print(F(" SIV: "));
    ESP_BT.print(SIV);

    Serial.println();
    ESP_BT.println();


    // min satellite in view for relatively accurate pos
    if (SIV > 2){
      // check distance for all debug cameras and print id + distance
      for (int i = 0; i < 5; i++) {
        currentCamera = nearCameras[i];

        distance = locUtils.getDistanceToCamera((double) latitude / 10000000, (double) longitude / 10000000, currentCamera.latitude, currentCamera.longitude);

        Serial.println(latitude / 10000000);
        Serial.println(longitude / 10000000);
        Serial.println(String(currentCamera.latitude, 10));
        Serial.println(String(currentCamera.longitude, 10));

        
        Serial.print("Distance to " + currentCamera.id);
        Serial.print(": " + String(distance, 3) + " m");
        Serial.println();

        ESP_BT.print("Distance to " + currentCamera.id);
        ESP_BT.print(": " + String(distance, 3) + " m");
        ESP_BT.println();
      }
    }
    
  }
}
