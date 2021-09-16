/* Copyright 2014 Ten Wong, wangtengoo7@gmail.com  
*  https://github.com/awong1900/RF430CL330H_Shield 
*/
#include "NdefMessage.h"

/**
 * @brief Construct an NDEF Message from one or more NDEF Records.
 * @param records one or more records
 * @param records_length  number of records
 */
NdefMessage::NdefMessage(NdefRecord records[], uint16_t records_length) {
  // validate
  if (records_length < 1) {
    Serial.println("[ERROR]must have at least one record");
    return;
  }
  mRecords = records;
  mRecords_length = records_length; 
}


/**
 * Return this NDEF Message as raw bytes.
 * The NDEF Message is formatted as per the NDEF 1.0 specification,
 * and the byte array is suitable for network transmission or storage
 * in an NFC Forum NDEF compatible tag.
 * This method will not chunk any records, and will always use the
 * short record (SR) format and omit the identifier field when possible.
 * @param buffer  store the message data
 * @return 
 * @see getByteArrayLength
 */
void NdefMessage::toByteArray(byte buffer[]) {
  uint16_t length = getByteArrayLength();
  uint16_t index = 0;
  for (int i=0; i<mRecords_length; i++) {
    boolean mb = (i == 0);  // first record
    boolean me = (i == mRecords_length - 1);  // last record
    mRecords[i].writeToByteBuffer(buffer, index, mb, me);
    index += mRecords[i].getByteLength();
  }
  //free memory
  for (int i=0; i<mRecords_length; i++)
    mRecords[i].freeRecord();
}

/**
 * Return the length of this NDEF Message if it is written to a byte array
 * with toByteArray
 * An NDEF Message can be formatted to bytes in different ways
 * depending on chunking, SR, and ID flags, so the length returned
 * by this method may not be equal to the length of the original
 * byte array used to construct this NDEF Message. However it will
 * always be equal to the length of the byte array produced by
 * toByteArray.
 *
 * @return length of this NDEF Message when written to bytes with toByteArray
 * @see toByteArray
 */
uint16_t NdefMessage::getByteArrayLength() {
  uint16_t length = 0;
  for (int i=0; i < mRecords_length; i++) 
    length += mRecords[i].getByteLength();
  return length;
}


