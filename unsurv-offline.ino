#include <TinyGPS++.h>
#include "HardwareSerial.h"

// The TinyGPS++ object
TinyGPSPlus gps;
HardwareSerial gps_serial(2);

void setup()
{
 // Note the format for setting a serial port is as follows: Serial2.begin(baud-rate, protocol, RX pin, TX pin);
 Serial.begin(115200);
 gps_serial.begin(9600);
 Serial.println("Serial Txd is on pin: "+String(TX));
 Serial.println("Serial Rxd is on pin: "+String(RX));
}

void loop()
{
   while (gps_serial.available()) {
    gps.encode(gps_serial.read());
    if (gps.location.isUpdated())
      Serial.print("LAT=");  Serial.println(gps.location.lat(), 6);
      Serial.print("LONG="); Serial.println(gps.location.lng(), 6);
      Serial.print("ALT=");  Serial.println(gps.altitude.meters());

   //Serial.print(char(gps_serial.read()));  // read from gps, write to serial debug port
   }
}
