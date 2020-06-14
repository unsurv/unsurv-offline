#ifndef SurveillanceCamera_h
#define SurveillanceCamera_h

#include "SurveillanceCamera.h"


#define MAXNEARCAMERAS 100


class SurveillanceCamera 
{
  public:

  double latitude;
  double longitude;

  unsigned short int cameraType;
  unsigned short int id; //  id is randomized for local area

};

#endif
