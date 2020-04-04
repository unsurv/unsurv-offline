#include "I2Cdev.h"
#include "MPU6050.h"
#include <SD.h>
#include <SPI.h>


// wiring on sparkfun ESP 32 Thing START

// MPU 6050

// VCC  3v3
// GND  GND
// SCL  GPIO 22
// SDA  GPIO 21
// INT  GPIO 33


// sparkfun level shifting micro SD 

// VCC  3v3
// GND  GND
// CS   not connected
// DI   GPIO 23 / MOSI
// SCK  GPIO 18 / SCK
// DO   GPIO 19 / MISO
// CD   GPIO 38

// Using a 16 GB SanDisk Ultra Micro SD card
// A 32 GB Samsung EVO Plus was NOT working with this setup

// wiring END

// MPU6050 accel/gyro start

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

int16_t oldax, olday, oldaz;
int16_t oldgx, oldgy, oldgz;

int16_t ax, ay, az;
int16_t gx, gy, gz;

//#define OUTPUT_READABLE_ACCELGYRO

#define LED_PIN 13
bool blinkState = false;

void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

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

// MPU6050 accel/gyro stop

// SD card start

File file;

class SurveillanceCamera{
  public:

  double latitude;
  double longitude;

  String cameraType;
  String id;  
};

int nearCameraCounter = 0;
SurveillanceCamera nearCameras[1000];

bool readLine(File &f, char* line, size_t maxLen) {
  for (size_t n = 0; n < maxLen; n++) {
    int c = f.read();
    if ( c < 0 && n == 0) return false;  // EOF
    if (c < 0 || c == '\n') {
      line[n] = 0;
      return true;
    }
    line[n] = c;
  }
  return false; // line too long
}

bool readDataFromSdCard(double* v1, double* v2, String* loc,String* loc2) {
  char line[100], *pointer, *str;
  if (!readLine(file, line, sizeof(line))) {
    return false;  // EOF or too long
  }
  
  *v1 = strtod(line, &pointer);
  if (pointer == line) return false;  // bad number if equal
  while (*pointer) {
    if (*pointer++ == ',') break;
  }
  
  *v2 = strtod(pointer, &str);
  //Serial.println(String(*v2, 7));
  while (*pointer) {
    if (*pointer++ == ',') break;
  }
  
  String a = strtok_r(pointer, ",", &str);
  String first(str);
  *loc = first;
  String let(a);
  *loc2 = let;
  
  return str != pointer;  // true if number found
}

double longitudeDegreesToMetersRatio(double latitude){
  int earthRadius = 6371000;
  double latitudeAsRad = latitude * DEG_TO_RAD;

  return PI/180*earthRadius*cos(latitudeAsRad);
  
}

int latitudeDegreeToMetersRatio() {
  return 110574;
}

boolean checkIfCameraInRange(int range, double deviceLatitude, double deviceLongitude, double cameraLatitude,  double cameraLongitude){

  float verticalDistanceInMeters = (deviceLatitude - cameraLatitude)*latitudeDegreeToMetersRatio();
  float horizontalDistanceInMeters = (deviceLongitude - cameraLongitude)*longitudeDegreesToMetersRatio(deviceLatitude);

  float distance = sqrt(sq(verticalDistanceInMeters) + sq(horizontalDistanceInMeters));
  
  if (distance < range) {
    return true;
  }
  return false;
}

void getNearCamerasFromSdCard(double deviceLatitude, double deviceLongitude, int radiusInMeters){

  double latitude, longitude;
  String cameraType,cameraId;
  int datapointsChecked = 0;

  while (readDataFromSdCard(&latitude, &longitude, &cameraType, &cameraId)) {
    datapointsChecked++;
    if (datapointsChecked == 1 || datapointsChecked == 10000){
      Serial.println(datapointsChecked);
    }
    
    if (checkIfCameraInRange(radiusInMeters, deviceLatitude, deviceLongitude, latitude, longitude)) {
      SurveillanceCamera tmpCamera;
      tmpCamera.latitude = latitude;
      tmpCamera.longitude = longitude;
      tmpCamera.cameraType = cameraType;
      tmpCamera.id = cameraId;

      nearCameras[nearCameraCounter] = tmpCamera;
      nearCameraCounter++;

      // TODO if near cameras over length 1000 do again with smaller radius so less than 1000 cameras are "near"

      Serial.println("-----------------------");
      Serial.println("Camera added at: ");
      Serial.println(String(latitude, 5));
      Serial.println(String(longitude, 5));
      Serial.println(String(nearCameraCounter));
      Serial.println("-----------------------");
    }
    
    //First 2 double datatype variables 
    //Serial.println("-----------------------");
    //Serial.println(String(latitude, 5));
    //Serial.println(String(longitude, 5));

    //Last 2 String type variables
    //Serial.println(cameraType);
    //Serial.println(cameraId);
  } 
  
}

// SD card stop

void setup() {
  Serial.begin(38400);

  // MPU6050 start
  
  //Print the wakeup reason for ESP32
  print_wakeup_reason();

  // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
    #endif

  accelgyro.reset();
  delay(500);

  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    
  delay(100);
  accelgyro.setZeroMotionDetectionThreshold(20);
  delay(100);
  // counts no motion events, this counting is slowed 
  // because a low frequency of measurements is used
  accelgyro.setZeroMotionDetectionDuration(20);
  delay(100);
  // interrupts twice, on no motion + on motion
  accelgyro.setIntZeroMotionEnabled(true);
  delay(100);
  // enables low power only accelerator mode
  accelgyro.setWakeCycleEnabled(true);
  delay(100);
  // frequency of measurements
  // 0 = 1.25 Hz, 1 = 2.5 Hz, 2 = 5 Hz, 3 = 10 Hz
  // this does seem to slow counting of motion events used for DetectionDuration above
  accelgyro.setWakeFrequency(2);
   
  int16_t oldax, olday, oldaz;
  int16_t oldgx, oldgy, oldgz;

  // initialize with values that wont be equal to a measurement
  oldax = 0; 
  olday = 0; 
  oldaz = 0;

  // configure Arduino LED pin for output
  pinMode(LED_PIN, OUTPUT);

  // MPU6050 stop

  // SD card start

  if (!SD.begin()) {
    Serial.println("begin error");
    return;
  }
  file = SD.open("/data.csv");
  if (!file) {
    Serial.println("open error");
    return;
  }
  
  Serial.println("start");

  getNearCamerasFromSdCard(50.00659,  8.27667, 5000);

  // SD card stop

}

void loop() {
  // read raw accel/gyro measurements from device
  // we're in accel only mode
  accelgyro.getAcceleration(&ax, &ay, &az);

  // these methods (and a few others) are also available
  //accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  //accelgyro.getRotation(&gx, &gy, &gz);
    
  // low frequency readings of mpu6050 => dont print if theres no
  // new data
  if (oldax != ax && olday != ay && oldaz != az){
    oldax = ax;
    olday = ay;
    oldaz = az;
      
    #ifdef OUTPUT_READABLE_ACCELGYRO
      // display tab-separated accel/gyro x/y/z values
      Serial.print("a/g:\t");
      Serial.print(ax); Serial.print("\t");
      Serial.print(ay); Serial.print("\t");
      Serial.print(az); Serial.print("\t");
      Serial.print(gx); Serial.print("\t");
      Serial.print(gy); Serial.print("\t");
      Serial.println(gz);
    #endif
  }

  if (accelgyro.getIntZeroMotionStatus()){
    Serial.println("Zero motion detected");
      
    // wire mpu6050 int pin to GPIO33
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_33,1); //1 = High, 0 = Low

    // lower sensitivity to allow low frequency wakeups
    accelgyro.setZeroMotionDetectionDuration(3);
    delay(100);
      
    //Go to sleep now
    Serial.println("Going to sleep now");
    esp_deep_sleep_start();
  }
}
