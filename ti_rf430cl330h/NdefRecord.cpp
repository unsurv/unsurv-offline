/* Copyright 2014 Ten Wong, wangtengoo7@gmail.com  
*  https://github.com/awong1900/RF430CL330H_Shield 
*/
#include "NdefRecord.h"
/* Uncomment these lines to enable debug output  */
//define NDEFRECORD_DEBUG

#define STORED_IN_RAM  true;

static const byte FLAG_MB = (byte) 0x80;
static const byte FLAG_ME = (byte) 0x40;
static const byte FLAG_CF = (byte) 0x20;
static const byte FLAG_SR = (byte) 0x10;
static const byte FLAG_IL = (byte) 0x08;

static const int MAX_PAYLOAD_SIZE = 10 * (1 << 20);  // 10 MB payload limit

/**
 * NFC Forum "URI Record Type Definition"<p>
 * This is a mapping of "URI Identifier Codes" to URI string prefixes,
 * per section 3.2.2 of the NFC Forum URI Record Type Definition document.
 */
static const String URI_PREFIX_MAP[] =  
{
  "", // 0x00
  "http://www.", // 0x01
  "https://www.", // 0x02
  "http://", // 0x03
  "https://", // 0x04
  "tel:", // 0x05
  "mailto:", // 0x06
  "ftp://anonymous:anonymous@", // 0x07
  "ftp://ftp.", // 0x08
  "ftps://", // 0x09
  "sftp://", // 0x0A
  "smb://", // 0x0B
  "nfs://", // 0x0C
  "ftp://", // 0x0D
  "dav://", // 0x0E
  "news:", // 0x0F
  "telnet://", // 0x10
  "imap:", // 0x11
  "rtsp://", // 0x12
  "urn:", // 0x13
  "pop:", // 0x14
  "sip:", // 0x15
  "sips:", // 0x16
  "tftp:", // 0x17
  "btspp://", // 0x18
  "btl2cap://", // 0x19
  "btgoep://", // 0x1A
  "tcpobex://", // 0x1B
  "irdaobex://", // 0x1C
  "file://", // 0x1D
  "urn:epc:id:", // 0x1E
  "urn:epc:tag:", // 0x1F
  "urn:epc:pat:", // 0x20
  "urn:epc:raw:", // 0x21
  "urn:epc:", // 0x22
};


/**
* Indicates the record is empty.
* Type, id and payload fields are empty in a TNF_EMPTY} record.
*/
NdefRecord::NdefRecord() {
  mTnf = TNF_EMPTY;
  
  mType = EMPTY_BYTE_ARRAY;
  mType_length = 0;
  
  mId = EMPTY_BYTE_ARRAY;
  mId_length = 0;
  
  mPayload = EMPTY_BYTE_ARRAY;
  mPayload_length = 0;
}

/**
* @brief  Construct an NDEF Record from its component fields
* @param tnf            a 3-bit TNF constant
* @param type           byte array, containing zero to 255 bytes
* @param type_length    length of type
* @param id   byte      array, containing zero to 255 bytes
* @param id_length      length of id
* @param payload        byte array, containing zero to (2 ** 32 - 1) bytes
* @param payload_length length of payload
* @return
*/
void NdefRecord::createNdefRecord(short tnf, byte type[], uint16_t type_length, byte id[], 
                    uint16_t id_length, byte payload[], uint16_t payload_length) 
{
  mTnf = tnf;
  
  mType = (byte*)malloc(type_length); //import: alloc memory size, free mem on Class NdefMessage
  memcpy(mType, type, type_length); //cope memory
  mType_length = type_length;
  
  mId = (byte*)malloc(id_length); 
  memcpy(mId, payload, id_length); 
  mId_length = id_length;

  mPayload = (byte*)malloc(payload_length); 
  memcpy(mPayload, payload, payload_length); 
  mPayload_length = payload_length;

#ifdef NDEFRECORD_DEBUG 
  Serial.println("--------Record info--------");
  Serial.print("mTnf = ");Serial.println(mTnf, HEX);
  Serial.print("mId = ");
  for (int i=0; i < mId_length; i++){Serial.print(mId[i], HEX);Serial.print(" ");}
  Serial.println();
  Serial.print("mType = ");
  for (int i=0; i < mType_length; i++){Serial.print(mType[i], HEX);Serial.print(" ");}
  Serial.println();
  Serial.print("mPayload = ");
  for (int i=0; i < mPayload_length; i++){Serial.print(mPayload[i], HEX);Serial.print(" ");}
  Serial.println();
  Serial.println("-------------End-----------");
#endif
}

/**
* Reference specification: NFCForum-TS-RTD_URI_1.0
* @brief  an NDEF Record containing the URI
* @param  uriString   string URI to encode.
* @return 
*/
void NdefRecord::createUri(String uriString) {
  if (uriString.length() == 0) {
    Serial.println("[ERROR]uri is empty");
    return;
  }
    
  byte prefix = 0;

  for (int i = 1; i < sizeof(URI_PREFIX_MAP)/sizeof(URI_PREFIX_MAP[0]); i++) { 
    if (uriString.startsWith(URI_PREFIX_MAP[i])) {
      prefix = (byte) i;
      uriString = uriString.substring(URI_PREFIX_MAP[i].length());
      break;
    }
  }


  byte uriBytes[uriString.length() + 1];
  uriString.getBytes(uriBytes, uriString.length() + 1);
  byte recordBytes[uriString.length() + 1];
  recordBytes[0] = prefix;
  arraycopy(uriBytes, 0, recordBytes, 1, uriString.length());

  createNdefRecord(TNF_WELL_KNOWN, RTD_URI, 1, EMPTY_BYTE_ARRAY, 
                    0, recordBytes, sizeof(recordBytes));
}

/**
* Reference specification: NFCForum-TS-RTD_1.0
* @brief  a new NDEF Record containing external (application-specific) data
* @param  domain        domain-name of issuing organization
* @param  type          domain-specific type of data
* @param  data          payload as bytes
* @param  data_length   length of payload
* @return
*/
void NdefRecord::createExternal(String domain, String type, byte data[], uint16_t data_length) {
  if (domain.length() == 0) {
    Serial.println("[ERROR]domain is empty");
    return;
  }
  if (type.length() == 0) {
    Serial.println("[ERROR]type is empty");
    return;
  }
  
  domain.toLowerCase();
  type.toLowerCase();

  byte byteDomain[domain.length()+1];
  domain.getBytes(byteDomain, domain.length()+1);
  byte byteType[type.length()+1];
  type.getBytes(byteType, type.length()+1);
  
  byte b[domain.length() + 1 + type.length()];
  arraycopy(byteDomain, 0, b, 0, domain.length());
  b[domain.length()] = ':';
  arraycopy(byteType, 0, b, domain.length() + 1, type.length());
  
  createNdefRecord(TNF_EXTERNAL_TYPE, b, sizeof(b), EMPTY_BYTE_ARRAY, 0, 
                    data, data_length);
}

/**
* @brief  an NDEF Record containing the MIME-typed data
* @param  mimeType a valid MIME type
* @param  mimeData MIME data as bytes
* @param  mimeData_length  length of MIME data
* @return 
*
*/
void NdefRecord::createMime(String mimeType, byte mimeData[], uint16_t mimeData_length) {
  if (mimeType.length() == 0) {
    Serial.println("[ERROR]mimeType is empty");
    return;
  }
  // We only do basic MIME type validation: trying to follow the
  // RFCs strictly only ends in tears, since there are lots of MIME
  // types in common use that are not strictly valid as per RFC rules
  mimeType.toLowerCase();

  int semicolonIndex = mimeType.indexOf(';');
  if (semicolonIndex != -1) {
    mimeType = mimeType.substring(0, semicolonIndex);
  }
  if (mimeType.length() == 0) {
    Serial.println("[ERROR]mimeType is empty");
    return;
  }
  int slashIndex = mimeType.indexOf('/');
  if (slashIndex == 0) {
    Serial.println("[ERROR]mimeType must have major type");
    return;
  }
  if (slashIndex == mimeType.length()) {
    Serial.println("[ERROR]mimeType must have minor type");
    return;
  }
  // missing '/' is allowed

  // MIME RFCs suggest ASCII encoding for content-type
  byte typeBytes[mimeType.length()+1];
  mimeType.getBytes(typeBytes, mimeType.length()+1);
  
  createNdefRecord(TNF_MIME_MEDIA, typeBytes, mimeType.length(), EMPTY_BYTE_ARRAY, 0,
                      mimeData, mimeData_length);
}

/**
* @breif a new Android Application Record (AAR)
* @param  packageName   Android package name
* @return 
*/
void NdefRecord::createApplicationRecord(String packageName) {
  if (packageName.length() == 0) {
    Serial.println("[ERROR]packageName is empty");
    return;
  }
  byte typeBytes[packageName.length()+1];
  packageName.getBytes(typeBytes, packageName.length()+1);
  createNdefRecord(TNF_EXTERNAL_TYPE, RTD_ANDROID_APP, 15, EMPTY_BYTE_ARRAY, 0,
                    typeBytes, packageName.length());
}

/**
* @breif an NDEF Record containing the Text-typed data
* @param  text_encode       text encoded data as bytes
* @param  language          see NdefRecord.h "unicode locale"
* @param  payload_length    length of payload_encode
* @param  encodeInUtf8      ture:utf-8 encode, false:utf-16 encode
* @return 
*/
void NdefRecord::createText(byte text_encode[],uint16_t payload_length, 
                              byte* language, boolean encodeInUtf8) 
{
  int utfBit = encodeInUtf8 ? 0 : (1 << 7);
  char status = (char) (utfBit + 2);
  byte data[1 + 2 + payload_length];
  data[0] = (byte) status;
  arraycopy(language, 0, data, 1, 2);
  arraycopy(text_encode, 0, data, (1+2), payload_length);
  createNdefRecord(TNF_WELL_KNOWN, RTD_TEXT, 1, EMPTY_BYTE_ARRAY, 0,
                    data, 1 + 2 + payload_length);
}

/**
 * Serialize record for network transmission.<p>
 * Uses specified MB and ME flags.<p>
 * Does not chunk records.
 */
void NdefRecord::writeToByteBuffer(byte buffer[], uint16_t index, boolean mb, boolean me) {
  boolean sr = mPayload_length < 256;
  boolean il = mId_length > 0;

  byte flags = (byte)((mb ? FLAG_MB : 0) | (me ? FLAG_ME : 0) |
          (sr ? FLAG_SR : 0) | (il ? FLAG_IL : 0) | mTnf);
  buffer[index++] = flags;
  buffer[index++] = (byte)mType_length;
  
  if (sr) {
    buffer[index++] = (byte)mPayload_length;
  } else {
    buffer[index++] = (byte)mPayload_length;
  }
  if (il) {
    buffer[index++] = (byte)mId_length;
  }

  arraycopy(mType, 0, buffer, index, mType_length);
  index += mType_length;
  arraycopy(mId, 0, buffer, index, mId_length);
  index += mId_length;
  arraycopy(mPayload, 0, buffer, index, mPayload_length);
  index += mPayload_length;
}


/**
 * Get byte length of serialized record.
 */
uint16_t NdefRecord::getByteLength() {
  int length = 3 + mType_length + mId_length + mPayload_length;

  boolean sr = mPayload_length < 256;
  boolean il = mId_length > 0;
  if (!sr) length += 3;
  if (il) length += 1;

  return length;
}


/**
 * free memory of class NdefRecord
 */
void NdefRecord::freeRecord() {
  free(mType);
  free(mId);
  free(mPayload);
}

/**
* @breif  Copies elements from the array src, starting at offset srcPos, 
*         into the array dst, starting at offset dstPos
* @param src    the source array to copy the content.
* @param srcPos the starting index of the content in src
* @param dst    the destination array to copy the data into
* @param dstPos the starting index for the copied content in dst
* @param length the number of elements to be copied
* @return
*/
void arraycopy(byte src[], int srcPos, byte dst[], int dstPos, int length) {
  for (int i=0; i<length; i++)
    dst[dstPos+i] = src[srcPos+i];
}

