#ifndef LocationUtils_h
#define LocationUtils_h


class LocationUtils {
  public:
    double longitudeDegreesToMetersRatio(double latitude);
    int latitudeDegreeToMetersRatio();
    boolean checkIfCameraInRange(int range, double deviceLatitude, double deviceLongitude, double cameraLatitude,  double cameraLongitude);
    float getDistanceToCamera(double deviceLatitude, double deviceLongitude, double cameraLatitude,  double cameraLongitude);
  };



#endif
