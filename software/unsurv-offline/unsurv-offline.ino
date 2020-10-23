// 20 mAh over 1h with 10s sleep 5s active (with BluetoothSerial active)
// 10 mAh over 1h with 25s sleep 5s active
// 6 mAh over 1h with 28s sleep 2s active
// 10 mAh over 1h with 13s sleep 2s active

#include "MPU6050.h"
#include "Wire.h"
#include "driver/adc.h"
#include "LocationUtils.h"
#include "StorageUtils.h"
#include "SurveillanceCamera.h"
#include "NfcUtils.h"
#include "SparkFun_Ublox_Arduino_Library.h" //http://librarymanager/All#SparkFun_Ublox_GPS

#define PROXIMITY_ALERT_RADIUS 100 // in m
#define LED 26
#define GPS_WAKEUP_PIN 5

MPU6050 accelgyro(0x68); // <-- use for AD0 low
int16_t ax, ay, az;
esp_sleep_wakeup_cause_t wakeup_reason;

boolean enableNfc = false;

boolean sleepOnNoMotion = false;
// enables a on/off cycle for the whole device specified with "espSleepDuration" and "wakeTime"
boolean savePower = true;

int espSleepDuration = 11; // in seconds
int wakeTime = 4; // in seconds

SFE_UBLOX_GPS ubloxGPS;
double latitude, longitude;
long deviceAltitude;
byte SIV;
boolean firstFix = true;

int startTime;


//BluetoothSerial ESP_BT;

LocationUtils locUtils;
StorageUtils storageUtils;


int nearCameraCounter = 0;
SurveillanceCamera nearCameras[MAXNEARCAMERAS];

float distance;
SurveillanceCamera currentCamera;

String prePayload = "Ids in proximity\n";
String nfcData = "";


void setup()
{
  
  Serial.begin(9600);

  //Print the wakeup reason for ESP32
  printWakeupReason();
  
  // setCpuFrequencyMhz(160);
  
  
  pinMode(LED, OUTPUT); // power LED
  digitalWrite(LED, HIGH);
  
  wakeGPS();

  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
  
  // setting up MPU 6050 accelerometer
  // initialize device
  Serial.println("Initializing MPU6050...");
  accelgyro.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");


  accelgyro.setXAccelOffset(1324);
  accelgyro.setYAccelOffset(1300);
  accelgyro.setZAccelOffset(736);
  delay(50);
  accelgyro.setZeroMotionDetectionThreshold(50);
  delay(50);
  // counts no motion events, this counting is slowed 
  // because a low frequency of measurements is used
  accelgyro.setZeroMotionDetectionDuration(10);
  delay(50);
  // interrupts twice, on no motion + on motion
  accelgyro.setIntZeroMotionEnabled(true);
  delay(50);
  // enables low power only accelerator mode
  accelgyro.setWakeCycleEnabled(true);
  delay(50);
  // frequency of measurements
  // 0 = 1.25 Hz, 1 = 2.5 Hz, 2 = 5 Hz, 3 = 10 Hz
  // this does seem to slow counting of motion events used for DetectionDuration above
  accelgyro.setWakeFrequency(3);
  delay(50);


  // setting up ublox GPS

  if (ubloxGPS.begin() == false) //Connect to the Ublox module using Wire port
  {
    Serial.println(F("Ublox GPS not detected at default I2C address. Location services NOT AVAILABLE."));
    Serial.println(F("Ublox GPS not detected at default I2C address. Location services NOT AVAILABLE."));
    Serial.println(F("Ublox GPS not detected at default I2C address. Location services NOT AVAILABLE."));
    
  }

  ubloxGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  ubloxGPS.saveConfiguration(); //Save the current settings to flash and BBR

  startTime = millis();
    
}

void loop()
{

  if (accelgyro.getZeroMotionDetected() && sleepOnNoMotion)
  {
  Serial.println("Zero motion detected");
    
  startDeepSleep(0);
  }

  
  if (millis() < startTime + (wakeTime * 1000) || !savePower)
  {
    //The GPS module only responds when a new position is available          
    latitude = (double) ubloxGPS.getLatitude() / 10000000;
    longitude = (double) ubloxGPS.getLongitude() / 10000000;
    deviceAltitude = ubloxGPS.getAltitude(); // in mm

    // satellites in view
    SIV = ubloxGPS.getSIV();

    Serial.println("-------------------------------");
    //ESP_BT.println();

    nfcData = "";
    nfcData += "Battery at: " + String(estimateBatteryLevel()) + " %\n";

    if (SIV > 2) 
    {
      printGpsData();
      short int radius = 100;

      if (firstFix)
      {
      nearCameraCounter = storageUtils.getCamerasFromSD(latitude, longitude, radius, nearCameras);

      while (nearCameraCounter < 0) // differ between sucess and fail here
      {
      radius = radius / 2;
      Serial.println(String(radius));
      nearCameraCounter = storageUtils.getCamerasFromSD(latitude, longitude, radius, nearCameras);
      }

      firstFix = false;
      }


    //ESP_BT.println("Distances: \n");


    nfcData = "";
    nfcData += "timestamp: " + getDateTimeString() + '\n';
    nfcData += "Battery at: " + String(estimateBatteryLevel()) + " %\n";
    

    nfcData += "Location:";
    nfcData += " Lat: " + String(latitude, 5) + " Lon: " + String(longitude, 5) + " Alt: " + String(deviceAltitude) + " mm -- SIV: " + String(SIV) + "\n";
    
    // check distance for all debug cameras and print id + distance
    for (int i = 0; i < nearCameraCounter; i++) 
      {
        currentCamera = nearCameras[i];

        distance = locUtils.getDistanceToCamera(latitude, longitude, currentCamera.latitude, currentCamera.longitude);

        if (distance < PROXIMITY_ALERT_RADIUS)
        {
            // nfcData += "Id: " + String(currentCamera.id) + " Distance: " + String(distance, 3) + '\n';
        }
        
        nfcData += "Id: " + String(currentCamera.id) + " Distance: " + String(distance, 3) + '\n';
        
        delay(50);
        //ESP_BT.println("Id: " + String(currentCamera.id) + " Distance:" + String(distance, 3));

      }

      
      Serial.println(nfcData);

       
  
    }

    if (enableNfc)
      {
        updateNFC(nfcData);
      }     

    delay(200);

  } 
  else 
  {
    startDeepSleep(espSleepDuration);
  }
}


void startDeepSleep(int timer) {

  if (timer != 0)
  {
    esp_sleep_enable_timer_wakeup(timer * 1000000);
    Serial.println("Setup ESP32 to sleep for " + String(espSleepDuration) +" seconds");
    //ESP_BT.println("Setup ESP32 to sleep for " + String(espSleepDuration) +" seconds");

    delay(50);
    //Go to sleep now
    digitalWrite(LED, LOW);
    pinMode(LED, INPUT); // power LED
    ubloxGPS.powerOff(0);
    delay(100);
    esp_deep_sleep_start();
  }
  else
  {
    // wire mpu6050 int pin to GPIO25
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_25, 1); //1 = High, 0 = Low
  
    // lower sensitivity to allow low frequency wakeups
    accelgyro.setZeroMotionDetectionDuration(10);
    
    //Go to sleep now
    Serial.println("Going to sleep now");
    digitalWrite(LED, LOW);
    pinMode(LED, INPUT); // power LED
  
    btStop();
  
    adc_power_off();
    
    delay(200);
    ubloxGPS.powerOff(0); // 0 = indefinetly
    delay(200);
    esp_deep_sleep_start();
  }
}

void printWakeupReason(){

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}

void printAccellData()
{
  Serial.print("a/g:\t");
  Serial.print(ax); Serial.print("\t");
  Serial.print(ay); Serial.print("\t");
  Serial.println(az);
}

void printGpsData()
{

  
  Serial.println(getDateTimeString());

  Serial.print(F(" SIV: "));
  Serial.print(SIV);
  //ESP_BT.print(F(" SIV: "));
  //ESP_BT.print(SIV);
  
  Serial.println();
  Serial.print(F("Lat: "));
  Serial.print(String(latitude, 5));
  //ESP_BT.print(F("Lat: "));
  //ESP_BT.print(String(latitude, 5));

  Serial.print(F(" Lon: "));
  Serial.print(String(longitude, 5));
  //ESP_BT.print(F(" Lon: "));
  //ESP_BT.print(String(longitude, 5));

  Serial.print(F(" Alt: "));
  Serial.print(deviceAltitude);
  Serial.print(F(" (mm)"));
  //ESP_BT.print(F(" Alt: "));
  //ESP_BT.print(altitude);

  Serial.println();
  //ESP_BT.println();

}

void wakeGPS() 
{
  pinMode(GPS_WAKEUP_PIN , OUTPUT);
  delay(100);
  digitalWrite(GPS_WAKEUP_PIN , LOW);
  Serial.println("WAKING UP GPS VIA EXINT PIN");
  delay(100);
  digitalWrite(GPS_WAKEUP_PIN , HIGH);
  delay(100);
  digitalWrite(GPS_WAKEUP_PIN , LOW);
  delay(100);
  pinMode(GPS_WAKEUP_PIN , INPUT);

}

String getDateTimeString()
{
  // THIS IS UGLY
  if (ubloxGPS.getDateValid() && ubloxGPS.getTimeValid())
  {
    return String(ubloxGPS.getYear()) + "-" + String(ubloxGPS.getMonth()) + "-" + String(ubloxGPS.getDay()) 
    + " " + String(ubloxGPS.getHour()) + ":" + String(ubloxGPS.getMinute()) + ":" + String(ubloxGPS.getSecond());
  }
  else 
  {
    return "no valid data";
  }
}

int estimateBatteryLevel()
{
  
  delay(100);
  float voltage = analogRead(35) * 7.29 / 4096.0;
  int percentage = 100;
  float maxChargeVoltage = 4.20;

  
  // lipo discharges nonlinear
  Serial.println("Voltage: " + String(voltage));
  // from 100 to 80 %
  if (voltage > 4.02)
  {
     percentage -= ((maxChargeVoltage - voltage) / 0.12f) * 10.0f;
  }
  else if (voltage <= 4.02 && voltage >= 3.87) // 80 - 60 %
  {
    percentage = 80;
    Serial.println(String(((4.02 - voltage) / 0.07f) * 10.0f));
    percentage -= ((4.02 - voltage) / 0.07f) * 10.0f;
  }
  else if (voltage <= 3.87 && voltage >= 3.64) // 60 - 10 %
  {
    percentage = 60;
    percentage -= (int) ((3.87 - voltage) / 0.04f) * 10.0f;
  }
  else
  {
    percentage = 0;
  }

  return percentage;
}
