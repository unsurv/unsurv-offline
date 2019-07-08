#include <TinyGPS++.h>
#include "HardwareSerial.h"

#include <Arduino.h>
#include <U8x8lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif


// The TinyGPS++ object
TinyGPSPlus gps;
HardwareSerial gps_serial(2);

U8X8_SSD1327_WS_128X128_4W_SW_SPI u8x8(/* clock=*/ 0, /* data=*/ 23, /* cs=*/ 26, /* dc=*/ 27, /* reset=*/ 1);

double lat;
double lng;
double altitude;

void setup()
{
 Serial.begin(115200);
 gps_serial.begin(9600);
 Serial.println("Serial Txd is on pin: "+String(TX));
 Serial.println("Serial Rxd is on pin: "+String(RX));

 u8x8.begin();
 u8x8.setPowerSave(0);
}

void loop()
{
   while (gps_serial.available()) {
    gps.encode(gps_serial.read());
    if (gps.location.isUpdated())
      
      lat = gps.location.lat();
      lng = gps.location.lng();
      altitude = gps.altitude.meters();

      Serial.print("LAT=");  Serial.println(lat, 6);
      Serial.print("LONG="); Serial.println(lng, 6);
      Serial.print("ALT=");  Serial.println(altitude, 4);

      String strLat = String(lat, 6);
      char cLat[10];
      strLat.toCharArray(cLat, 10);

      String strLng = String(lng, 6);
      char cLng[10];
      strLng.toCharArray(cLng, 10);

      String strAlt = String(altitude, 6);
      char cAlt[10];
      strAlt.toCharArray(cAlt, 10);


      u8x8.setFont(u8x8_font_chroma48medium8_r);
      u8x8.drawString(0, 5, cLat);
      u8x8.drawString(0, 10, cLng);
      u8x8.drawString(0, 15, cAlt);


   }
}
