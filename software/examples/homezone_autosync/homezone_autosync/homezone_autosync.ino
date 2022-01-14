#include "BMA400.h"
#include "SdFat.h"
#include "SparkFun_Ublox_Arduino_Library.h"
#include "driver/adc.h"
#include "esp_sleep.h"
#include "LocationUtils.h"
#include <ArduinoJson.h>

#include <WiFi.h>
#include <WiFiMulti.h>
#include <HTTPClient.h>
WiFiMulti wifiMulti;

//unsurv offline specific pin assignments
#define LED 27
#define GPS_WAKEUP_PIN 5
#define MIN_SATS_IN_VIEW 4

// from sdfat example
// Test with reduced SPI speed for breadboards.  SD_SCK_MHZ(4) will select
// the highest speed supported by the board that is not over 4 MHz.
#define SPI_SPEED SD_SCK_MHZ(4)

// BMA400 definitions
#define BMA400_intPin1 25   // interrupt1 
#define BMA400_intPin2 26   // interrupt2
#define BITMASK_PIN_25 0x2000000 // needed for ESP32 deepsleep

// generating one time passwords
#include "SimpleHOTP.h"

// secret for OTPs
uint8_t secret[] = "super_secret";
Key key(secret, sizeof(secret)-1);

BMA400 BMA400(BMA400_intPin1, BMA400_intPin2); // instantiate BMA400 class
uint8_t tap_sensitivity = tap_sens_0, int_registers; // 0 is most sensitive
boolean BMA400_tap_flag, single_tap_flag, double_tap_flag = false;

// GNSS receiver 
SFE_UBLOX_GPS ubloxGPS;
uint8_t SIV = 0;

SdFat SD;

esp_sleep_wakeup_cause_t wakeup_reason;

// functions for distance measuring etc.
LocationUtils locUtils;

double latitude, longitude, homeLatitude, homeLongitude;
long deviceAltitude;

// step counter registers for BMA400
uint8_t step0, step1, step2, step_state;

// saves to RTC memory, persists deepsleep
RTC_DATA_ATTR int OtpCount = 1;
RTC_DATA_ATTR int failedTries = 1;
RTC_DATA_ATTR float distanceTravelled = 0;
RTC_DATA_ATTR long step_count = 0;


int homeZoneRadius = 100; // meters

// int otpGracePeriod = 3;

void setup() {
// put your setup code here, to run once:
  
  Serial.begin(115200);

  printWakeupReason();

  pinMode(LED, OUTPUT); // power LED
  pinMode(GPS_WAKEUP_PIN, INPUT); // set GPS int pin to input for now
  
  pinMode(BMA400_intPin1, INPUT);  // define BMA400 interrupt pins as inputs
  pinMode(BMA400_intPin2, INPUT);

  Wire.begin(); 
  
  // BMA400 setup for step counting to gather data
  BMA400.I2Cscan(); // should detect BMA400 at 0x14

  // Read the BMA400 Chip ID register, this is a good test of communication
  Serial.println("BMA400 accelerometer...");
  byte c = BMA400.getChipID();  // Read CHIP_ID register for BMA400
  Serial.print("BMA400 "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x90, HEX);
  Serial.println(" ");
  delay(100);
  if(c == 0x90 && wakeup_reason != ESP_SLEEP_WAKEUP_TIMER) // do not reset if esp32 woke up from deep sleep
  {
    Serial.println("BMA400 is online..."); Serial.println(" ");
   
    BMA400.resetBMA400();                                                // software reset before initialization
    delay(50);      
    BMA400.selfTestBMA400();                                             // perform sensor self test
    BMA400.resetBMA400();                                                // software reset before initialization
    delay(50);                                                         // give some time to read the screen
    // BMA400.CompensationBMA400(Ascale, SR, normal_Mode, OSR, acc_filter, offset); // quickly estimate offset bias in normal mode
    BMA400.initBMA400forTapping(tap_sensitivity);          // Initialize sensor in desired mode for application       
    delay(50);
    BMA400.enableStepCountNotWrist();              
  }
  else 
  {
  if(c != 0x90) Serial.println(" BMA400 not functioning!");
  }

  attachInterrupt(BMA400_intPin1, myinthandler1, RISING);  // define wake-up interrupt for INT1 pin output of BMA400
  

  wakeGPS();
  
  if (!ubloxGPS.begin()) //Connect to the Ublox module using Wire port
  {
    Serial.println(F("Ublox GPS not detected at default I2C address. Location services NOT AVAILABLE."));
    Serial.println(F("Ublox GPS not detected at default I2C address. Location services NOT AVAILABLE."));
    Serial.println(F("Ublox GPS not detected at default I2C address. Location services NOT AVAILABLE."));
    
  }

  ubloxGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  ubloxGPS.powerSaveMode(true);
  ubloxGPS.saveConfiguration(); //Save the current settings to flash and BBR
  
  
  homeLatitude = 52.48575;
  homeLongitude = 6.98707;

  wifiMulti.addAP("gnanas", "zemFBupa138reichtnichtaus");

}

void loop() {
  // put your main code here, to run repeatedly:

  SIV = 5;

  tapDetection();

  if (single_tap_flag) {
  // satellites in view, location calculation does not work below 3 sats
  if (SIV < MIN_SATS_IN_VIEW)
    {
      
      ubloxGPS.powerSaveMode(false);
      
      while (SIV < MIN_SATS_IN_VIEW)
      {
        // satellites in view
        SIV = ubloxGPS.getSIV();
        Serial.println("scanning for GPS satellites: SIV " + String(SIV));
        digitalWrite(LED, HIGH);
        delay(500);
        digitalWrite(LED, LOW);

      }

    }

  
  if (SIV >= MIN_SATS_IN_VIEW)
    {
      latitude = (double) ubloxGPS.getLatitude() / 10000000;
      longitude = (double) ubloxGPS.getLongitude() / 10000000;
      deviceAltitude = ubloxGPS.getAltitude(); // in mm
      digitalWrite(LED, HIGH);
      delay(2000);
      digitalWrite(LED, LOW);
      uint8_t step0, step1, step2;

      // read step count registers
      step0 = BMA400.readByte(BMA400_ADDRESS, BMA400_STEP_CNT0);
      delay(20);
      step1 = BMA400.readByte(BMA400_ADDRESS, BMA400_STEP_CNT1);
      delay(20);
      step2 = BMA400.readByte(BMA400_ADDRESS, BMA400_STEP_CNT2);
      delay(20);
      step_state = BMA400.readByte(BMA400_ADDRESS, BMA400_STEP_STAT);

      logToSd("logFile.txt", String(latitude, 6) + "," + String(longitude, 6) + "," + String(deviceAltitude) + "," + String(SIV) + "," + getDateTimeString() + "," + step0 + "," + step1 + "," + step2 + "," + step_state);

      // checkIfCameraInRange is used to measure distance between locations
      boolean homeWithinRange = locUtils.checkIfCameraInRange(homeZoneRadius, latitude, longitude, homeLatitude, homeLongitude);
      
      logToSd("logFile.txt", String(latitude, 6) + "," + String(longitude, 6) + "," + String(deviceAltitude) + "," + step0 + "," + step1 + "," + step2 + "," + step_state);
        
      
      // Asks a small server in your home Network for a HMAC based one time password and if server OTP = local OTP uploads the fitness data to the server in a JSON format.
      // adapted from httpclient example and JSON library example 
      if (true)
      {

        Serial.println("--- DEVICE WITHIN HOMEZONE ---");
        Serial.println("-------------------------");

        StaticJsonDocument<50> OtpJson;
        
        SimpleHOTP gen(key, OtpCount);
        Serial.println("OTP COUNT " + String(OtpCount));
        uint32_t localOtp = gen.generateHOTP();

        Serial.println("----- local OTP is: -----");
        Serial.println(localOtp);
        Serial.println("-------------------------");
        uint32_t serverOtp;
        
        
        if((wifiMulti.run() == WL_CONNECTED)) 

        {
        
          HTTPClient http;
  
          Serial.println("Connecting to server");
          // configure traged server and url
          http.begin("http://192.168.178.43:5000/getOTP?count=" + String(OtpCount)); //HTTP
  
          Serial.print("[HTTP] GET...\n");
          // start connection and send HTTP header
          int httpCode = http.GET();
  
          // httpCode will be negative on error
          if(httpCode > 0) {
            // HTTP header has been send and Server response header has been handled
            Serial.printf("[HTTP] GET... code: %d\n", httpCode);

            // file found at server
            if(httpCode == HTTP_CODE_OK) 
              {
              String payload = http.getString();
              Serial.println("----- Server OTP is: -----");
              Serial.println(payload);
              Serial.println("-------------------------");
              
              DeserializationError error = deserializeJson(OtpJson, payload);
              // Test if parsing succeeds.
              if (error) {
              Serial.print(F("deserializeJson() failed: "));
              Serial.println(error.f_str());
              return;
              }

                serverOtp = OtpJson["HOTP"];

              }
        } 
        else 
        {
            Serial.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
        }

        http.end();

        // if code is the same
        if (serverOtp == localOtp)
        {
          Serial.println("--- SAME OTP ---");
          Serial.println("sending data");
          OtpCount++;

          HTTPClient http2;
        
          http2.begin("http://192.168.178.43:5000/postData"); //HTTP
          http2.addHeader("Content-Type", "application/json");

          DynamicJsonDocument  doc(50);
          // doc["steps"] = step0 + 256*step1 + 65536*step2;
          doc["steps"] = 3222;
        

          String httpRequestData;
          serializeJson(doc, httpRequestData);           
          // Send HTTP POST request
          int httpResponseCode = http2.POST(httpRequestData);

          Serial.println("--- server response: ---");
          Serial.println(httpResponseCode);
          
          http2.end();


        }
                
        }
  
      }
      
    }

    startDeepSleep(10);

  }

  delay(1000);

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


void startDeepSleep(int timer) {

  if (timer != 0)
  {
    Serial.println("Setup ESP32 to sleep for " + String(timer) +" seconds");
    //ESP_BT.println("Setup ESP32 to sleep for " + String(espSleepDuration) +" seconds");

    delay(50);
    //Go to sleep now
    digitalWrite(LED, LOW);
    pinMode(LED, INPUT); // power LED
    
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

    // esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
    // esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
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
    digitalWrite(LED, LOW);
    pinMode(LED, INPUT); // power LED
  
    delay(100);
    ubloxGPS.powerOff(0); // 0 = indefinetly
    Serial.println("shutting down ublox module");
    delay(100);
    Serial.println("Going to sleep now");
    adc_power_off();

    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
    // esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
    // esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
    delay(100);
    esp_deep_sleep_start();
  }
}


void logToSd(String file, String logMsg) 
{
  if (!SD.begin(4, SPI_SPEED)) 
  {
    Serial.println("error initializing SD card.");
  }
  Serial.println("logging data");

  File logFile = SD.open(file, FILE_WRITE);

  // logFile = SD.open("/logFile.txt", O_WRONLY | O_CREAT | O_EXCL);

  if (logFile) 
  {
    logFile.print(logMsg);
    logFile.println();
  
    logFile.close();
    Serial.println("success");
  
  }
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

void tapDetection()
{
    int_registers = BMA400.getStatus(); // read status of interrupts to clear

    switch (int_registers)
    {
      case INT_STAT1_SINGLE_TAP:
        Serial.println("Single tap detected");
        digitalWrite(LED, HIGH);
        delay(100);
        digitalWrite(LED, LOW);
        single_tap_flag = true;
        // logToSd("/test.txt", "single tap");
        break;
        
      case INT_STAT1_DOUBLE_TAP:
        Serial.println("Double tap detected");
        digitalWrite(LED, HIGH);
        delay(100);
        digitalWrite(LED, LOW);
        single_tap_flag = false;
        double_tap_flag = true;
        // logToSd("/test.txt", "double tap");
        break;
    } 
    
    BMA400_tap_flag = false;
  
}

void myinthandler1()
{
  BMA400_tap_flag = true;
  Serial.println("** BMA400 got tapped! **");
  
}
