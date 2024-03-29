#include "StorageUtils.h"
#include "LocationUtils.h"
#include "SurveillanceCamera.h"
#include <SPI.h>
#include <SdFat.h>

SdFat SD;

// from sdfat example
// Test with reduced SPI speed for breadboards.  SD_SCK_MHZ(4) will select
// the highest speed supported by the board that is not over 4 MHz.
// Change SPI_SPEED to SD_SCK_MHZ(50) for best performance.
#define SPI_SPEED SD_SCK_MHZ(4)

LocationUtils locationUtils;

File myFile;
int fileSize;


void StorageUtils::logToSd(String file, String logMsg) 
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
  }
}


JSONVar StorageUtils::getContacts(int limit)
{
  if (!SD.begin(4, SPI_SPEED)) 
  {
    Serial.println("error initializing SD card.");
  }

  File dataFile = SD.open("contacts.txt");

  if (dataFile) 
  {
    
    int i, j;
    
    i = 0;
    j = 0;
    char line[300];

    JSONVar contactArray;
    
    while (dataFile.available()) {

      int fileRead = dataFile.read();
      
      if (fileRead != 0x0A) // \n char
      {
        // Serial.println(fileRead);
        
        line[i] = fileRead;
        i++;

        
      } 
      else
      {
        // if not single char in line
        if (i != 1)
        {
          JSONVar item; 
          item = JSON.parse(String(line));

          contactArray[j] = item;
          j++;
        
          i = 0;

          // reset array
          memset(line, 0, sizeof(line));
        }
        
        
      }
    }
    
    dataFile.close();

    
    JSONVar recentContacts = JSON.parse("[]");;
    int numberOfContacts = contactArray.length();    

    Serial.println(String(numberOfContacts));
    Serial.println(String(limit));
    
    if (numberOfContacts > limit)
    {
      for (int l = numberOfContacts - limit, m = 0; l < numberOfContacts, m < limit; l++, m++)
      {
        /*
        Serial.println(JSON.typeof(recentContacts));
        Serial.println(String(l));
        Serial.println(String(m));
        Serial.println(JSON.stringify(contactArray[l]));
        */
        String recentContact = JSON.stringify(contactArray[l]);
        Serial.println(recentContact);
        recentContact.replace("\\", "");
        Serial.println(recentContact);
        
        
        recentContacts[m] = recentContact;
      }
    }
    else
    {
      recentContacts = contactArray;
    }
    
    return recentContacts;
  }
}

int StorageUtils::getCamerasFromSD(double deviceLatitude, double deviceLongitude, short radiusInMeters, SurveillanceCamera cameras[MAXNEARCAMERAS])
{

  char curr;
  char info[12];
  int valueStart, index;

  double latitude, longitude;
  short int cameraType, dataType;
  char cameraId[12];
  String id;

  short int nearCameraCounter = 0;
  
  
  Serial.print("Initializing SD card...");

  if (!SD.begin(4, SPI_SPEED)) 
  {
    Serial.println("error initializing SD card.");
    return -1;
  }
  Serial.println("accessing SD");

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  
  
  // re-open the file for reading:
  myFile = SD.open("/data.csv");
  if (myFile) 
  {
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
      
      if (curr == ',' || curr == '\n' || curr == '\r') 
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
            // cameraId = strtol(info, NULL, 10);

            // TODO check for security
            memcpy(cameraId, info, 12);
            // cameraId[13] = '\0';
            //Serial.println("Chararray ID:");
            //Serial.println(String(cameraId));
            break;

        }

        
        if(dataType == 3) // line of csv data completed
        {

          // if too many cameras in radius around device, half the radius
          if (nearCameraCounter == MAXNEARCAMERAS) {
            Serial.println("RADIUS HALVED ---------------");
            return -2;
          }

          
          if (locationUtils.checkIfCameraInRange(radiusInMeters, deviceLatitude, deviceLongitude, latitude, longitude))
          {
          SurveillanceCamera tmpCamera;
          tmpCamera.latitude = latitude;
          tmpCamera.longitude = longitude;
          tmpCamera.cameraType = cameraType;
          // tmpCamera.id = cameraId;
          memcpy(tmpCamera.id, cameraId, 12);


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
    
  } 
  else 
  {
    // if the file didn't open, print an error:
    Serial.println("error opening data.csv");
    return -1;
  }
}
