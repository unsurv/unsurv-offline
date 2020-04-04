#include <SD.h>
#include <SPI.h>

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


void setup() {
  // Open serial communications and wait for port to open:
  //SD Card Reader Setup
  Serial.begin(9600);
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

  getNearCamerasFromSdCard(50.00659,  8.27667, 10000);

}

void loop() {
   

 
}
