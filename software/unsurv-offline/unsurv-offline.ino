/*
  Reading lat and long via UBX binary commands - no more NMEA parsing!
  By: Nathan Seidle
  SparkFun Electronics
  Date: January 3rd, 2019
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.
*/

#include <Arduino.h>
#include <Wire.h> //Needed for I2C to GPS
#include <BluetoothSerial.h>
#include "LocationUtils.h"
#include "StorageUtils.h"
#include "SurveillanceCamera.h"
#include "NfcUtils.h"
#include "SparkFun_Ublox_Arduino_Library.h" //http://librarymanager/All#SparkFun_Ublox_GPS

#define PROXIMITY_ALERT_RADIUS 100 // in m

SFE_UBLOX_GPS myGPS;
BluetoothSerial ESP_BT;

LocationUtils locUtils;
StorageUtils storageUtils;


int nearCameraCounter = 0;
SurveillanceCamera nearCameras[MAXNEARCAMERAS];

long lastTime = 0; //Simple local timer. Limits amount if I2C traffic to Ublox module.

float distance;
SurveillanceCamera currentCamera;

String prePayload = "Ids in proximity\n";
String nfcData = "";



void setup()
{
  Serial.begin(9600);
  
  Serial.println("SparkFun Ublox Example");

  // ESP32 Bluetooth name, connect via phone and use a bluetooth terminal app from the app store
  ESP_BT.begin("test");

  // add debugging cameras
  //populateCameras();

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

  
    
    double latitude = (double) myGPS.getLatitude() / 10000000;
    Serial.print(F("Lat: "));
    Serial.print(String(latitude, 5));
    ESP_BT.print(F("Lat: "));
    ESP_BT.print(String(latitude, 5));

    double longitude = (double) myGPS.getLongitude() / 10000000;
    Serial.print(F(" Lon: "));
    Serial.print(String(longitude, 5));

    ESP_BT.print(F(" Lon: "));
    ESP_BT.print(String(longitude, 5));

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
      short int radius = 250;

      long accuracy = myGPS.getPositionAccuracy();

      Serial.println("Accuracy: " + String(accuracy, 5));
      delay(50);
      ESP_BT.println("Accuracy: " + String(accuracy, 5));

      nearCameraCounter = storageUtils.getCamerasFromSD(latitude, longitude, radius, nearCameras);
      while (nearCameraCounter < 0) // differ between sucess and fail here
 
        {
        radius = radius / 2;
        Serial.println(String(radius));
        nearCameraCounter = storageUtils.getCamerasFromSD(latitude, longitude, radius, nearCameras);
        }
      Serial.print("Distances: ");
      delay(50);
      ESP_BT.println("Distances: ");

      
      nfcData = "";
      // check distance for all debug cameras and print id + distance
      for (int i = 0; i < nearCameraCounter; i++) {
        currentCamera = nearCameras[i];

        distance = locUtils.getDistanceToCamera(latitude, longitude, currentCamera.latitude, currentCamera.longitude);

        if (distance < PROXIMITY_ALERT_RADIUS) 
        {
          nfcData += String(currentCamera.id) + '\n';  
        }
        
        Serial.println("Id: " + String(currentCamera.id) + " Distance:" + String(distance, 3));
        delay(50);
        ESP_BT.println("Id: " + String(currentCamera.id) + " Distance:" + String(distance, 3));

        /*
        Serial.print("Distance to " + currentCamera.id);
        Serial.print(": " + String(distance, 3) + " m");
        Serial.println();

        delay(50);

        ESP_BT.print("Distance to " + currentCamera.id);
        ESP_BT.print(": " + String(distance, 3) + " m");
        ESP_BT.println();
        */
      }
    }

    Serial.println(prePayload + nfcData);
    delay(50);
    ESP_BT.println(prePayload + nfcData);


    // updateNFC(prePayload + nfcData);


    delay(10000);
    
  
}
