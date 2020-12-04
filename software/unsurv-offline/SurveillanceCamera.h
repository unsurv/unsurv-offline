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
  char id[12]; //  id is randomized for local area

};

#endif
