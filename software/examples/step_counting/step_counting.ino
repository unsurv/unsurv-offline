#include "BMA400.h"
#include "SdFat.h"
#include "SparkFun_Ublox_Arduino_Library.h"
#include "driver/adc.h"
#include "esp_sleep.h"

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

boolean log_position = false;
uint8_t SIV = 0;

BMA400 BMA400(BMA400_intPin1, BMA400_intPin2); // instantiate BMA400 class
uint8_t tap_sensitivity = tap_sens_0; // 0 is most sensitive

SFE_UBLOX_GPS ubloxGPS;
SdFat SD;

esp_sleep_wakeup_cause_t wakeup_reason;

double latitude, longitude;
long deviceAltitude;
uint8_t step0, step1, step2, step_state;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  printWakeupReason();

  pinMode(LED, OUTPUT); // power LED
  pinMode(GPS_WAKEUP_PIN, INPUT); // set GPS int pin to input for now
  
  pinMode(BMA400_intPin1, INPUT);  // define BMA400 interrupt pins as inputs
  pinMode(BMA400_intPin2, INPUT);

  Wire.begin(); 
  BMA400.I2Cscan(); // should detect BMA400 at 0x14

  // Read the BMA400 Chip ID register, this is a good test of communication
  Serial.println("BMA400 accelerometer...");
  byte c = BMA400.getChipID();  // Read CHIP_ID register for BMA400
  Serial.print("BMA400 "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x90, HEX);
  Serial.println(" ");
  delay(100);
  if(c == 0x90 && wakeup_reason != ESP_SLEEP_WAKEUP_TIMER) // check if all I2C sensors with WHO_AM_I have acknowledged
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

  // logToSd("logFile.txt", "test");

  // resetting step counting

  /*
  BMA400.writeByte(BMA400_ADDRESS, BMA400_STEP_CNT0, 0x00);
  delay(20);
  BMA400.writeByte(BMA400_ADDRESS, BMA400_STEP_CNT1, 0x00);
  delay(20);
  BMA400.writeByte(BMA400_ADDRESS, BMA400_STEP_CNT2, 0x00);

  delay(50);

  Serial.println("STEPCOUNT0" + BMA400.readByte(BMA400_ADDRESS, BMA400_STEP_CNT0));
  */

  
}

void loop() {
  // put your main code here, to run repeatedly:

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

      step0 = BMA400.readByte(BMA400_ADDRESS, BMA400_STEP_CNT0);
      delay(20);
      step1 = BMA400.readByte(BMA400_ADDRESS, BMA400_STEP_CNT1);
      delay(20);
      step2 = BMA400.readByte(BMA400_ADDRESS, BMA400_STEP_CNT2);
      delay(20);
      step_state = BMA400.readByte(BMA400_ADDRESS, BMA400_STEP_STAT);
      
      logToSd("logFile.txt", String(latitude, 6) + "," + String(longitude, 6) + "," + String(deviceAltitude) + "," + String(SIV) + "," + getDateTimeString() + "," + step0 + "," + step1 + "," + step2 + "," + step_state);
    }

    startDeepSleep(30);


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
    digitalWrite(LED, LOW);
    pinMode(LED, INPUT); // power LED
  
    delay(100);
    ubloxGPS.powerOff(0); // 0 = indefinetly
    Serial.println("shutting down ublox module");
    delay(100);
    Serial.println("Going to sleep now");
    adc_power_off();

    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
    delay(100);
    esp_deep_sleep_start();
  }
}
