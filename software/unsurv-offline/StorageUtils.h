#ifndef StorageUtils_h
#define StorageUtils_h

#include "LocationUtils.h"
#include "SurveillanceCamera.h"





class StorageUtils {
  public:
    int getCamerasFromSD(double deviceLatitude, double deviceLongitude, short radiusInMeters, SurveillanceCamera cameras[MAXNEARCAMERAS]);
  };



#endif
