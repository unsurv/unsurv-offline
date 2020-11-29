#ifndef StorageUtils_h
#define StorageUtils_h

#include "LocationUtils.h"
#include "SurveillanceCamera.h"
#include "Arduino_JSON.h"


class StorageUtils {
  public:
    int getCamerasFromSD(double deviceLatitude, double deviceLongitude, short radiusInMeters, SurveillanceCamera cameras[MAXNEARCAMERAS]);
    void logToSd(String file, String logMsg);
    JSONVar getContacts(int limit);
  };



#endif
