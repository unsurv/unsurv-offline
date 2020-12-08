

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Wire.h>
#include "RF430CL330H_Shield.h"

#define IRQ   (32)
#define RESET (34)
int led = 5;
RF430CL330H_Shield nfc(IRQ, RESET);

volatile byte into_fired = 0;
uint16_t flags = 0;



byte nfcTemplate[] = {
/*NDEF Tag Application Name*/                                                           \
0xD2, 0x76, 0x00, 0x00, 0x85, 0x01, 0x01,                                               \
                                                                                        \
/*Capability Container ID*/                                                             \
0xE1, 0x03,                                                                             \
0x00, 0x0F, /* CCLEN */                                                                 \
0x20,       /* Mapping version 2.0 */                                                   \
0x00, 0xF9, /* MLe (249 bytes); Maximum R-APDU data size */                             \
0x00, 0xF6, /* MLc (246 bytes); Maximum C-APDU data size */                             \
0x04,       /* Tag, File Control TLV (4 = NDEF file) */                                 \
0x06,       /* Length, File Control TLV (6 = 6 bytes of data for this tag) */           \
0xE1, 0x04, /* File Identifier */                                                       \
0x0B, 0xDF, /* Max NDEF size (3037 bytes of useable memory) */                          \
0x00,       /* NDEF file read access condition, read access without any security */     \
0x00,       /* NDEF file write access condition; write access without any security */   \
                                                                                        \
/* NDEF File ID */                                                                      \
0xE1, 0x04,                                                                             \
                                                                                        \
/* NDEF File for Hello World  (48 bytes total length) */                                \
0x00, 0x12, /* NLEN; NDEF length (2 byte long message) all bytes below */                               \


0xC1, /* Record Header  */                                                              \ 
0x01, /* Type Length */                                                                 \
0x00, 0x00, 0x00, 0x00, /* bytes after this -1  = NLEN - 4*/                                              \
0x54, /* type  T = text */                                                              \
/* PAYLOAD NDEF data;*/                                                                
0x65, 0x6E, /* 'e', 'n', */                                                             \
                                                                                        \
            
};

// ndef length is 4 bytes long

byte nfcTemplateBackup[] = {
/*NDEF Tag Application Name*/                                                           \
0xD2, 0x76, 0x00, 0x00, 0x85, 0x01, 0x01,                                               \
                                                                                        \
/*Capability Container ID*/                                                             \
0xE1, 0x03,                                                                             \
0x00, 0x0F, /* CCLEN */                                                                 \
0x20,       /* Mapping version 2.0 */                                                   \
0x00, 0xF9, /* MLe (249 bytes); Maximum R-APDU data size */                             \
0x00, 0xF6, /* MLc (246 bytes); Maximum C-APDU data size */                             \
0x04,       /* Tag, File Control TLV (4 = NDEF file) */                                 \
0x06,       /* Length, File Control TLV (6 = 6 bytes of data for this tag) */           \
0xE1, 0x04, /* File Identifier */                                                       \
0x0B, 0xDF, /* Max NDEF size (3037 bytes of useable memory) */                          \
0x00,       /* NDEF file read access condition, read access without any security */     \
0x00,       /* NDEF file write access condition; write access without any security */   \
                                                                                        \
/* NDEF File ID */                                                                      \
0xE1, 0x04,                                                                             \
                                                                                        \
/* NDEF File for Hello World  (48 bytes total length) */                                \
0x00, 0x12, /* NLEN; NDEF length (2 byte long message) */                               \
0xD1, /* Record Header  */                                                              \ 
0x01, /* Type Length */                                                                 \
0x0E, /* bytes after this -1  = NLEN - 4*/                                              \
0x54, /* type  T = text */                                                              \
0x02,  /* ID length  */                                                                 \
0x65, 0x6E, /* 'e', 'n', */                                                             \
                                                                                        \
/* PAYLOAD NDEF data;*/            
};




/**
**  @brief  interrupt service
**/
void RF430_Interrupt()
{
    into_fired = 1;
    detachInterrupt(1);//cancel interrupt
}


void updateNFC(String nfcString)
{
    int rawDataSize = nfcString.length(); // strings are null terminated
    int templateSize = sizeof(nfcTemplate);
    byte nfcPayload[rawDataSize];
    nfcString.getBytes(nfcPayload, rawDataSize);

    
    //fix the length fields in the NFC tag
    
    int payloadSize = rawDataSize + 2; // -1 to remove null termated string
    int nlen = payloadSize + 7;
    Serial.println(payloadSize);
    Serial.println(nlen);

    if (nlen < 3000) 
    {

      // 2nd lowest byte
      nfcTemplate[26] = (nlen >> (8*1)) & 0xff;
      // lowest byte
      nfcTemplate[27] = (nlen >> (8*0)) & 0xff;
      Serial.println("NLEN ----------");
      Serial.println(nfcTemplate[26]);
      Serial.println(nfcTemplate[27]);
    }
    else 
    {
      return;
    }

    // ndef payload length
    // 2nd lowest byte
    nfcTemplate[30] = (payloadSize >> (8*3)) & 0xff;
    // lowest byte
    nfcTemplate[31] = (payloadSize >> (8*2)) & 0xff;

    nfcTemplate[32] = (payloadSize >> (8*1)) & 0xff;
    // lowest byte
    nfcTemplate[33] = (payloadSize >> (8*0)) & 0xff;
    
    
    Serial.println("PAYLOAD ----------");
    Serial.println(nfcTemplate[30]);
    Serial.println(nfcTemplate[31]);
    Serial.println(nfcTemplate[32]);
    Serial.println(nfcTemplate[33]);
    
    byte nfcTag[templateSize + rawDataSize - 1];

    int pointer;
    for (int i = 0 ; i < templateSize; i++) {
      nfcTag[i] = nfcTemplate[i];
      pointer = i;
      }
    for (int f = 0; f < rawDataSize; f++){
      // Serial.println(nfcPayload[f]);
    }

    for (int i = 0, j = pointer + 1; i < rawDataSize ,  j < pointer + rawDataSize; i++, j++) { 
      nfcTag[j] = nfcPayload[i];
      // Serial.println(nfcPayload[i]);
      }

    // update nfc tag

    while(!(nfc.Read_Register(STATUS_REG) & READY)); //wait until READY bit has been set
    // Serial.print("Firmware Version:"); Serial.println(nfc.Read_Register(VERSION_REG), HEX);

    //write NDEF memory with Capability Container + NDEF message
    nfc.Write_Continuous(0, nfcTag, sizeof(nfcTag));

    //Enable interrupts for End of Read and End of Write
    nfc.Write_Register(INT_ENABLE_REG, EOW_INT_ENABLE + EOR_INT_ENABLE);

    //Configure INTO pin for active low and enable RF
    nfc.Write_Register(CONTROL_REG, INT_ENABLE + INTO_DRIVE + RF_ENABLE );

    //enable interrupt 1
    attachInterrupt(1, RF430_Interrupt, FALLING);

}
