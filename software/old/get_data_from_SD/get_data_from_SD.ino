/*
  SD card read/write

 This example shows how to read and write data to and from an SD card file


 created   Nov 2010
 by David A. Mellis
 modified 9 Apr 2012
 by Tom Igoe

 This example code is in the public domain.

 File needs to have a blank line at the end!
 */

#include <SPI.h>
//#include <SD.h>
#include <SdFat.h>
SdFat SD;

// from sdfat example
// Test with reduced SPI speed for breadboards.  SD_SCK_MHZ(4) will select
// the highest speed supported by the board that is not over 4 MHz.
// Change SPI_SPEED to SD_SCK_MHZ(50) for best performance.
#define SPI_SPEED SD_SCK_MHZ(32)

#define MAXNEARCAMERAS 100

File myFile;
int fileSize;

class SurveillanceCamera
{
  public:

  double latitude;
  double longitude;

  unsigned short int cameraType;
  unsigned short int id; //  id is randomized for local area
  
  
};






double longitudeDegreesToMetersRatio(double latitude)
{
  int earthRadius = 6371000;
  double latitudeAsRad = latitude * DEG_TO_RAD;

  return PI/180*earthRadius*cos(latitudeAsRad);
  
}

int latitudeDegreeToMetersRatio() 
{
  return 110574;
}

boolean checkIfCameraInRange(int range, double deviceLatitude, double deviceLongitude, double cameraLatitude,  double cameraLongitude)
{

  float verticalDistanceInMeters = (deviceLatitude - cameraLatitude)*latitudeDegreeToMetersRatio();
  float horizontalDistanceInMeters = (deviceLongitude - cameraLongitude)*longitudeDegreesToMetersRatio(deviceLatitude);

  float distance = sqrt(sq(verticalDistanceInMeters) + sq(horizontalDistanceInMeters));
  
  if (distance < range) 
  {
    return true;
  }
  
  return false;
}

int getCamerasFromSD(double deviceLatitude, double deviceLongitude, short radiusInMeters, SurveillanceCamera cameras[MAXNEARCAMERAS])
{

  char curr;
  char info[18];
  int valueStart, index;

  double latitude, longitude;
  short int cameraType, dataType, cameraId;
  String id;

  short int nearCameraCounter = 0;
  
  
  Serial.print("Initializing SD card...");

  if (!SD.begin(4, SPI_SPEED)) 
  {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  
  
  // re-open the file for reading:
  myFile = SD.open("/data.csv");
  if (myFile) 
  {
    Serial.println("data.csv");

    // fileSize = myFile.size();
    // Serial.println(fileSize);

    // myFile.seek(fileSize / 2);

    valueStart = 0;
    index = 0;
    dataType = 0;

    // read from the file until there's nothing else in it:
    while (myFile.available()) 
    {
      
      curr = myFile.read();
      
      if (curr == ',' || curr == '\n' || curr == '\n') 
      {
        
        info[index - valueStart] = '\0';
        valueStart = index + 1;
        
        // TODO check strtox for SECURITY!!!
        switch(dataType) {
          case 0:
            latitude = strtod(info, NULL);
            break;

          case 1:
            longitude = strtod(info, NULL);
            break;

          case 2: 
            cameraType = strtol(info, NULL, 10);
            break;
            
          case 3:
            cameraId = strtol(info, NULL, 10);
            break;

        }

        
        if(dataType == 3) // line of csv data completed
        {

          // if too many cameras in radius around device, half the radius
          if (nearCameraCounter == MAXNEARCAMERAS) {
            Serial.println("RADIUS HALVED ---------------");
            return -1;
          }

          
          if (checkIfCameraInRange(radiusInMeters, deviceLatitude, deviceLongitude, latitude, longitude))
          {
          SurveillanceCamera tmpCamera;
          tmpCamera.latitude = latitude;
          tmpCamera.longitude = longitude;
          tmpCamera.cameraType = cameraType;
          tmpCamera.id = cameraId;

          cameras[nearCameraCounter] = tmpCamera;
          nearCameraCounter++;

          //Serial.println("-----------------------");
          //Serial.println("Camera added at: ");
          //Serial.println(String(latitude, 5));
          //Serial.println(String(longitude, 5));
          //Serial.println(String(cameraType));
          //Serial.println(cameraId);
          //Serial.print("Nearcameras: ");
          //Serial.println(nearCameraCounter);
          //Serial.println("-----------------------");
          
          }
        }

        

        // 4 types of data in csv
        dataType < 3 ? ++dataType: dataType = 0;
        
        info[0] = '\0';

        
      } else 
      {
        info[index - valueStart] = curr;
        
      }

      index++;

      // Serial.println(myFile.position());
      
           
    }

    
    // close the file:
    myFile.close();

    return nearCameraCounter; // success
    
  } else 
  {
    // if the file didn't open, print an error:
    Serial.println("error opening data.csv");
    return -1;
  }
}



void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);

  SurveillanceCamera nearCameras[MAXNEARCAMERAS];

  int radius = 5000;
  unsigned long start = millis();
  // Call to your function

  
  int returnCode = getCamerasFromSD(50.0, 8.2590, radius, nearCameras);
  while (returnCode < 0) // differ between sucess and fail here
  {
    radius = radius / 2;
    Serial.println(String(radius));
    returnCode = getCamerasFromSD(50.0, 8.2590, radius, nearCameras);
  }


  
  
  // Compute the time it took
  unsigned long endop = millis();
  unsigned long delta = endop - start;
  Serial.println("Operation took: " + String(delta) + " ms");

  Serial.println("Cameras added: " + String(returnCode));

  
  
  
  
  
}

void loop() {
  // nothing happens after setup
}
