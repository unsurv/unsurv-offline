// 20 mAh over 1h with 10s sleep 5s active (with BluetoothSerial active)
// 10 mAh over 1h with 25s sleep 5s active
// 6 mAh over 1h with 28s sleep 2s active
// 10 mAh over 1h with 13s sleep 2s active

// new USB power meter
// 13 mah over 1h with gps powerSave 40s sleep 2s active <- good GPS connection?

// TODO validate SD card input data

// attribute  Copyright2 2018 Tlera Corporation for BMA400 Code

#include "BQ25181.h"
#include "BMA400.h"
#include "Wire.h"
#include "LocationUtils.h"
#include "StorageUtils.h"
#include "SurveillanceCamera.h"
#include "NfcUtils.h"
#include "SparkFun_Ublox_Arduino_Library.h"
#include "driver/adc.h"
#include "driver/periph_ctrl.h"
#include "esp_sleep.h"
#include "Arduino_JSON.h"

#define PROXIMITY_ALERT_RADIUS 100 // in m
#define LED 27
#define GPS_WAKEUP_PIN 5

#define MIN_SATS_IN_VIEW 4

#define SEARCH_DURATION 10

#define BITMASK_PIN_25 0x2000000 // needed for ESP32 deepsleep

//BMA400 definitions
#define BMA400_intPin1 25   // interrupt1 pin definitions, wake-up from STANDBY pin
#define BMA400_intPin2 26   // interrupt2 pin definitions, data ready or sleep interrupt

uint8_t Ascale = AFS_2G, SR = SR_50Hz, power_Mode = lowpower_Mode, OSR = osr0, acc_filter = acc_filt2;

float aRes;             // scale resolutions per LSB for the sensor
int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t tempCount;      // temperature raw count output
float   temperature;    // Stores the real internal chip temperature in degrees Celsius
float ax, ay, az;       // variables to hold latest sensor data values 
float offset[3];        // accel bias offsets

// Logic flags to keep track of device states
bool BMA400_wake_flag = true;
bool BMA400_sleep_flag = false;
bool InMotion = false;

BMA400 BMA400(BMA400_intPin1, BMA400_intPin2); // instantiate BMA400 class


// battery charging IC
BQ25181 BQ25181(4.2);  // set battery voltage

esp_sleep_wakeup_cause_t wakeup_reason;


boolean enableNfc = true;
boolean sleepOnNoMotion = true;
boolean calibrateBMA400 = false;
// enables a on/off cycle for the whole device specified with "espSleepDuration" and "wakeTime"
boolean savePower = true;

boolean logToSd = true;

int espSleepDuration = 30; // in seconds
int wakeTime = 3; // in seconds

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


void setup()
{
  
  Serial.begin(115200);

  //Print the wakeup reason for ESP32
  printWakeupReason();
 
  pinMode(LED, OUTPUT); // power LED
  digitalWrite(LED, HIGH);
  
  // BMA 400 setup
  pinMode(BMA400_intPin1, INPUT);  // define BMA400 wake and sleep interrupt pins as L082 inputs
  pinMode(BMA400_intPin2, INPUT);
  
  BQ25181.setChargingCurrent(2); // 150 mA charging current

  // lower CPU frequency
  setCpuFrequencyMhz(80);
  
  wakeGPS();
  adc_power_on(); 
  delay(100);

  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
  
  // setting up MPU 6050 accelerometer
  // initialize device
  // Serial.println("Initializing MPU6050...");
  // accelgyro.initialize();

  // verify connection
  // Serial.println("Testing device connections...");
  // Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  byte c = BMA400.getChipID();  // Read CHIP_ID register for BMA400
  if(c == 0x90) // check if all I2C sensors with WHO_AM_I have acknowledged
  {
    Serial.println("BMA400 is online..."); Serial.println(" ");

    delay(1000);
    aRes = BMA400.getAres(Ascale);                                       // get sensor resolutions, only need to do this once
    BMA400.resetBMA400();                                                // software reset before initialization
    delay(100);      
    BMA400.selfTestBMA400();                                             // perform sensor self test
    BMA400.resetBMA400();                                                // software reset before initialization
    delay(100);

    

    if (calibrateBMA400) 
    {                                     
      delay(100);                                                         // give some time to read the screen
      BMA400.CompensationBMA400(Ascale, SR, normal_Mode, OSR, acc_filter, offset); // quickly estimate offset bias in normal mode
    }
  }

  BMA400.initBMA400(Ascale, SR, power_Mode, OSR, acc_filter);

  attachInterrupt(BMA400_intPin1, myinthandler1, RISING);  // define wake-up interrupt for INT1 pin output of BMA400
  attachInterrupt(BMA400_intPin2, myinthandler2, RISING);  // define data ready interrupt for INT2 pin output of BMA400 
 
  
  BMA400.getStatus(); // read status of interrupts to clear
  delay(100);
  BMA400.activateNoMotionInterrupt();  

  /*
  accelgyro.setXAccelOffset(1324);
  accelgyro.setYAccelOffset(1300);
  accelgyro.setZAccelOffset(736);
  delay(50);
  accelgyro.setZeroMotionDetectionThreshold(50);
  delay(50);
  // counts no motion events, this counting is slowed 
  // if a low frequency of measurements is used
  accelgyro.setZeroMotionDetectionDuration(50);
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
  accelgyro.setWakeFrequency(2);
  delay(50);
  */

  // setting up ublox GPS

  if (ubloxGPS.begin() == false) //Connect to the Ublox module using Wire port
  {
    Serial.println(F("Ublox GPS not detected at default I2C address. Location services NOT AVAILABLE."));
    Serial.println(F("Ublox GPS not detected at default I2C address. Location services NOT AVAILABLE."));
    Serial.println(F("Ublox GPS not detected at default I2C address. Location services NOT AVAILABLE."));
    
  }

  ubloxGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  ubloxGPS.powerSaveMode(true);
  ubloxGPS.saveConfiguration(); //Save the current settings to flash and BBR

  startTime = millis();

  Serial.println("GPS power save: " + String(ubloxGPS.getPowerSaveMode()));

  Serial.println("Initialization complete...");

  String jsonString;
  JSONVar nfcData;
  nfcData["batt"] = 100;
  JSONVar contactArray;
  contactArray = storageUtils.getContacts(2);
          
  nfcData["c"] = contactArray;
  jsonString = "{\"b\":100,\"c\":[";

  for (int i = 0; i < contactArray.length(); i++)
  {

    String con = JSON.stringify(contactArray[i]);
    con.replace("\\", "");
    con.remove(0, 1);
    con.remove(con.length() - 1, 1);
    Serial.println(con);
    jsonString.concat(con);
    
    if (contactArray.length() - 1 - i != 0)
    {
      jsonString.concat(",");  
    }
    
  }

  jsonString.concat("]}");
  Serial.println(jsonString);
  
     
  // updateNFC(jsonString);
  
  updateNFC("Natus et libero sed possimus nam. Et illum a voluptas numquam consequatur et cum iure. Voluptas dolorem aspernatur est est neque ut fugit quisquam. Et ut placeat libero est voluptatem necessitatibus eum. ur et cum iure. Voluptas dolorem aspernatur est est neque ut fugit quisquam.  ur et cum iure. Voluptas dolorem aspernatur est est neque ut fugit quisquam.  ur et cum iure. Voluptas dolorem aspernatur est est neque ut fugit quisquam.  ur et cum iure. Voluptas dolorem aspernatur est est neque ut fugit quisquam.");
  // updateNFC("loren ipsum asdasdasd");
}

void loop()
{

  /* BMA400 sleep/wake detect*/
  if(BMA400_wake_flag)
  {
    BMA400_wake_flag = false; // clear the wake flag
    InMotion = true;          // set motion state latch
    BMA400.activateNoMotionInterrupt();  
    attachInterrupt(BMA400_intPin2, myinthandler2, RISING);  // attach no-motion interrupt for INT2 pin output of BMA400 

  }

  if(BMA400_sleep_flag)
  {
    BMA400_sleep_flag = false;            // clear the sleep flag
    InMotion = false;                     // set motion state latch
    detachInterrupt(BMA400_intPin2);       // Detach the BMA400 "Go to sleep" interrupt so it doesn't spuriously wake the STM32L4
    BMA400.deactivateNoMotionInterrupt(); // disable no-motion interrupt to save power
   
    if (sleepOnNoMotion)
    {
      startDeepSleep(0); 
    }
   
  }/* end of sleep/wake detect */
  
  
  if (millis() < startTime + (wakeTime * 1000) || !savePower || firstFix)
  {
    //The GPS module only responds when a new position is available          
    latitude = (double) ubloxGPS.getLatitude() / 10000000;
    longitude = (double) ubloxGPS.getLongitude() / 10000000;
    deviceAltitude = ubloxGPS.getAltitude(); // in mm

    // satellites in view
    SIV = ubloxGPS.getSIV();

    Serial.println("-------------------------------");
    Serial.println("SIV: " + String(SIV));
    //ESP_BT.println();

    // creating a JSON to log to SD and later transmit via nfc
    // need to keep data as short as possible, we can only transmit 3kb via the RF 430 NFC chip
    JSONVar nfcData;

    
    if (SIV < MIN_SATS_IN_VIEW) // less than 3 satellites in view, scan for SEARCH_DURATION in seconds
    {
      
      long searchStart = millis();
      ubloxGPS.powerSaveMode(false);
      delay(100);
      Serial.println("GPS power save: " + String(ubloxGPS.getPowerSaveMode()));
      firstFix = true;

      Serial.println(searchStart + millis());
      Serial.println(searchStart + (SEARCH_DURATION * 1000));
      
      while ((millis() < searchStart + (SEARCH_DURATION * 1000)) && SIV < MIN_SATS_IN_VIEW)
      {
        SIV = ubloxGPS.getSIV();
        Serial.println("scanning for GPS satellites: SIV " + String(SIV));
        // change firstFix state to true here?
        delay(1000);      
      }
    }
    else // more than MIN_SATS_IN_VIEW
    { 
      printGpsData();
      short int radius = 100;

      if (firstFix)
      {
        Serial.println("first satellite fix ... querying SD card for data");
        // switch on power save after sats have been found
        ubloxGPS.powerSaveMode(true);
        delay(100);

        
        startTime = millis();
        nearCameraCounter = storageUtils.getCamerasFromSD(latitude, longitude, radius, nearCameras);

        if (nearCameraCounter == -1) {
          startDeepSleep(0);
        }
        
        while (nearCameraCounter == -2) // returns -2 if too many cameras are in current radius
        {
          radius = radius / 2;
          Serial.println(String(radius));
          nearCameraCounter = storageUtils.getCamerasFromSD(latitude, longitude, radius, nearCameras);
        }
  
        firstFix = false;
      }

      JSONVar current_location;

      current_location["lat"] = latitude;
      current_location["lon"] = longitude;
      // current_location["alt"] = deviceAltitude;
      current_location["SIV"] = SIV;
      current_location["t"] = getTimeString();
  
      // nfcData += " Lat: " + String(latitude, 5) + " Lon: " + String(longitude, 5) + " Alt: " + String(deviceAltitude) + " mm -- SIV: " + String(SIV) + "\n";
      
      nfcData["loc"] = current_location;
     
      // check distance for all debug cameras and print id + distance

      
      for (int i = 0; i < nearCameraCounter; i++) 
      {
        currentCamera = nearCameras[i];
  
        distance = locUtils.getDistanceToCamera(latitude, longitude, currentCamera.latitude, currentCamera.longitude);
        
        // nfcData += "Id: " + String(currentCamera.id) + " Distance: " + String(distance, 3) + '\n';
        
        
        delay(50);

         if (distance < PROXIMITY_ALERT_RADIUS)
        {
          Serial.println("object in close proximity");
          JSONVar contact;

          // contact["distance"] = distance;
          Serial.println(String(currentCamera.latitude, 5));
          Serial.println(String(currentCamera.longitude, 5));
          Serial.println(String(currentCamera.cameraType));
          Serial.println(String(currentCamera.id));
          
          nfcData["ids"][i] = String(currentCamera.id);
          
        }
  
      }
    }

    String jsonString = JSON.stringify(nfcData);
    // Serial.println(jsonString);

    if (nfcData["ids"].length() > 0) {
      Serial.println("MORE THAN ONE CAMERA IS IN YOUR AREA");
      Serial.println(jsonString);
      storageUtils.logToSd("contacts.txt", jsonString);
      
    }

    if (enableNfc)
    {
      nfcData["batt"] = estimateBatteryLevel();

      JSONVar contactArray = storageUtils.getContacts(10);    
      nfcData["c"] = contactArray;
      // Serial.println(JSON.stringify(nfcData));
      jsonString = JSON.stringify(nfcData);
      
      updateNFC(jsonString);
    }

    
    if (SIV >= MIN_SATS_IN_VIEW && logToSd)
    {
      // logging to SD card
      storageUtils.logToSd("logFile.txt", String(latitude, 6) + "," + String(longitude, 6) + "," + String(deviceAltitude) + "," + String(SIV) + "," + getDateTimeString());
    }
    
    

    delay(100);

  } 
  else // not in active state
  {
    if (SIV < MIN_SATS_IN_VIEW)
    {
      // increase sleep time here for each search duration without success
      // no satellites in view, sleep for 2 mins and try again
      Serial.println("No Satellites in view, sleeping for 2 minutes.");
      // todo increase sleep time here depending on number of unsuccessful searches for GPS signal beforehand
      startDeepSleep(120);
    }
    else // regular active/sleep cycle while GPS satellites in view
    {
      startDeepSleep(espSleepDuration);
    }
  }
}

void myinthandler1()
{
  BMA400_wake_flag = true; 
  Serial.println("** BMA400 is awake! **");
}


void myinthandler2()
{
  BMA400_sleep_flag = true;
  Serial.println("** BMA400 is asleep! **");
}


void startDeepSleep(int timer) {

  if (timer != 0)
  {
    Serial.println("Setup ESP32 to sleep for " + String(timer) +" seconds");
    //ESP_BT.println("Setup ESP32 to sleep for " + String(espSleepDuration) +" seconds");

    delay(50);
    //Go to sleep now
    pinMode(LED, OUTPUT); // power LED
    digitalWrite(LED, LOW);
    
    
    esp_sleep_enable_timer_wakeup(timer * 1000000); // ys conversion
    delay(100);
    
    // if timer is longer than 45 s switch gps completely off
    if (timer > 45 * 1000000)
    {
      ubloxGPS.powerOff(0);
    }
    
    delay(100);
    // accelgyro.setZeroMotionDetectionDuration(10);
    adc_power_off();

    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
    
    delay(100);
    esp_deep_sleep_start();
  }
  else
  {
    // wire mpu6050 int pin to GPIO25
    // esp_sleep_enable_ext0_wakeup(GPIO_NUM_25, 1); //1 = High, 0 = Low < This keeps gpio stuff powered, no bueno
     
    esp_sleep_enable_ext1_wakeup(BITMASK_PIN_25, ESP_EXT1_WAKEUP_ANY_HIGH);
  
    // lower sensitivity to allow low frequency wakeups
    // accelgyro.setZeroMotionDetectionDuration(10);
    
    //Go to sleep now
    Serial.println("Going to sleep now");
    pinMode(LED, OUTPUT); // power LED
    digitalWrite(LED, LOW);
  
    delay(100);
    ubloxGPS.powerOff(0); // 0 = indefinetly
    Serial.println("shutting down ublox module");
    delay(100);
    adc_power_off();  
    delay(100);
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

String getTimeString()
{
  // THIS IS UGLY
  if (ubloxGPS.getDateValid() && ubloxGPS.getTimeValid())
  {
    return String(ubloxGPS.getHour()) + ":" + String(ubloxGPS.getMinute()) + ":" + String(ubloxGPS.getSecond());
  }
  else 
  {
    return "no valid data";
  }
}

int estimateBatteryLevel()
{
  float voltage = analogRead(35) * 7.29 / 4096.0;
  int percentage = 100;
  float maxChargeVoltage = 4.18;

  
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

  Serial.println("Percentage: " + String(percentage));

  return percentage;
}
