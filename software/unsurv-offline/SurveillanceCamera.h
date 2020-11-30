#ifndef SurveillanceCamera_h
#define SurveillanceCamera_h

#include "SurveillanceCamera.h"


#define MAXNEARCAMERAS 100


class SurveillanceCamera 
{
  public:

  double latitude;
  double longitude;

  short int cameraType;
  long id; //  id is randomized for local area

};

#endif
