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


#include "SparkFun_Ublox_Arduino_Library.h" //http://librarymanager/All#SparkFun_Ublox_GPS
SFE_UBLOX_GPS myGPS;
BluetoothSerial ESP_BT;
long lastTime = 0; //Simple local timer. Limits amount if I2C traffic to Ublox module.

void setup()
{
  Serial.begin(9600);
  Serial.println("SparkFun Ublox Example");
  
  ESP_BT.begin("test");


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
  //Query module only every second. Doing it more often will just cause I2C traffic.
  //The module only responds when a new position is available
  if (millis() - lastTime > 3000)
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


  }
}
