#include "Arduino.h"
#include "LocationUtils.h"

double LocationUtils::longitudeDegreesToMetersRatio(double latitude){
  int earthRadius = 6371000;
  double latitudeAsRad = latitude * DEG_TO_RAD;

  return PI/180*earthRadius*cos(latitudeAsRad);
  
}

int LocationUtils::latitudeDegreeToMetersRatio() {
  return 110574;
}

boolean LocationUtils::checkIfCameraInRange(int range, double deviceLatitude, double deviceLongitude, double cameraLatitude,  double cameraLongitude){

  float verticalDistanceInMeters = (deviceLatitude - cameraLatitude)*latitudeDegreeToMetersRatio();
  float horizontalDistanceInMeters = (deviceLongitude - cameraLongitude)*longitudeDegreesToMetersRatio(deviceLatitude);

  float distance = sqrt(sq(verticalDistanceInMeters) + sq(horizontalDistanceInMeters));
  
  if (distance < range) {
    return true;
  }
  return false;
}

float LocationUtils::getDistanceToCamera(double deviceLatitude, double deviceLongitude, double cameraLatitude,  double cameraLongitude){

  float verticalDistanceInMeters = (deviceLatitude - cameraLatitude)*latitudeDegreeToMetersRatio();
  float horizontalDistanceInMeters = (deviceLongitude - cameraLongitude)*longitudeDegreesToMetersRatio(deviceLatitude);

  float distance = sqrt(sq(verticalDistanceInMeters) + sq(horizontalDistanceInMeters));
  
  return distance;
}
