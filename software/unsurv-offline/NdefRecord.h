/* Copyright 2014 Ten Wong, wangtengoo7@gmail.com  
*  https://github.com/awong1900/RF430CL330H_Shield
*/
#ifndef NDEFRECORD_H_
#define NDEFRECORD_H_
#include "Arduino.h"

static const short TNF_EMPTY = 0x01;
static const short TNF_WELL_KNOWN = 0x01;
static const short TNF_MIME_MEDIA = 0x02;
static const short TNF_ABSOLUTE_URI = 0x03;
static const short TNF_EXTERNAL_TYPE = 0x04;
static const short TNF_UNCHANGED = 0x05;
static const short TNF_RESERVED = 0x06;

static byte RTD_TEXT[] = {0x54};   // "T"  
static byte RTD_URI[] = {0x55};   // "U"    
static byte RTD_SMART_POSTER[] = {0x53, 0x70};  // "Sp" 
static byte RTD_ALTERNATIVE_CARRIER[] = {0x61, 0x63};  // "ac"
static byte RTD_HANDOVER_CARRIER[] = {0x48, 0x63};  // "Hc"
static byte RTD_HANDOVER_REQUEST[] = {0x48, 0x72};  // "Hr"
static byte RTD_HANDOVER_SELECT[] = {0x48, 0x73}; // "Hs"
static byte RTD_ANDROID_APP[] = "android.com:pkg";

static byte EMPTY_BYTE_ARRAY[] = {};

/* unicode locale */
static byte ENGLISH[] = {'e', 'n'};
static byte CHINESE[] = {'z', 'n'};
static byte FRENCH[]  = {'f', 'r'};
static byte GERMANY[] = {'d', 'e'};
static byte ITALIAN[] = {'i', 't'};
static byte JAPANESE[]= {'j', 'p'};
static byte KOREAN[]  = {'k', 'o'};

extern void arraycopy(byte src[], int srcPos, byte dst[], int dstPos, int length);

class NdefRecord
{
public:
  NdefRecord();
  void createNdefRecord(short tnf, byte type[], uint16_t type_length, byte id[], 
                        uint16_t id_length, byte payload[], uint16_t payload_length);
  void createUri(String uriString);
  void createExternal(String domain, String type, byte data[], uint16_t data_length);
  void createMime(String mimeType, byte mimeData[], uint16_t mimeData_length);
  void createApplicationRecord(String packageName);
  void createText(byte payload_encode[],uint16_t payload_length, byte language[], boolean encodeInUtf8);
  void writeToByteBuffer(byte buffer[], uint16_t index, boolean mb, boolean me);
  uint16_t getByteLength();
  void freeRecord();

private:
  short mTnf;
  byte* mType;
  byte* mId;
  byte* mPayload;
  uint16_t mType_length;
  uint16_t mId_length;
  uint16_t mPayload_length;
};
#endif //NDEFRECORD_H_

