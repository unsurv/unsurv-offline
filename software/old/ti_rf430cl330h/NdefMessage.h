/* Copyright 2014 Ten Wong, wangtengoo7@gmail.com  
*  https://github.com/awong1900/RF430CL330H_Shield 
*/
#ifndef NDEFMESSAGE_H_
#define NDEFMESSAGE_H_
#include "NdefRecord.h"
class NdefMessage
{
public:
  NdefMessage(NdefRecord records[], uint16_t records_length);
  void toByteArray(byte buffer[]);
  uint16_t getByteArrayLength();
  
private:
  NdefRecord* mRecords;
  uint16_t mRecords_length;
};
#endif //NDEFMESSAGE_H_

