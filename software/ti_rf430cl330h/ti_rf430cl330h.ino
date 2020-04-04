/* Copyright 2013 Ten Wong, wangtengoo7@gmail.com
   and 2013 Sven haiges, sven.haiges@gmail.com for the serial
   communication and the dynamic NDEF message assemply.
*  https://github.com/awong1900/RF430CL330H_Shield
*  RF430CL330H datasheet reference http://www.ti.com/
*/

/*********************************************************
** sample: when reset the rf430, it will write the uri to
** rf430 tag.
***********************************************************/

/**
 * Wiring on a Wemos d32 PRO
 * 
 * RF430    D32
 * 
 * 3v3      3v3
 * GND      GND
 * 
 * SDA      SDA
 * SCL      SCL
 * RST      34
 * INT      32
 * SCMS/CS  GND
 * SCK      GND
 * 
 */


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

byte NDEF_BASE[] = {
    /*NDEF Tag Application */
    0xD2, 0x76, 0x00, 0x00, 0x85, 0x01, 0x01,

    0xE1, 0x03,     /*Capability Container ID*/

    /* CC file start */
    0x00, 0x0F,     /* CCLEN 15bytes fix*/
    0x20,     /* Mapping version 2.0 */
    0x00, 0x3B,     /* MLe (49 bytes); Maximum R-APDU data size */
    0x00, 0x34,     /* MLc (52 bytes); Maximum C-APDU data size */
    0x04,     /* Tag, File Control TLV (4 = NDEF file) */
    0x06,     /* Length, File Control TLV (6 = 6 bytes of data for this tag) */
    0xE1, 0x04,     /* Type4 Tag File Identifier */
    0x0B, 0xDF,     /* Max NDEF size (3037 bytes of RF430CL330 useable memory) */
    0x00,     /* NDEF file read access condition, read access without any security */
    0x00,     /* NDEF file write access condition; write access without any security */
    /* CC file end */

    0xE1, 0x04,     /* NDEF File ID */

    0x00, 0x0F,    /* NDEF Length 15bytes */

    /* NDEF start */
    0xD1,    /* NDEF Header MB=1, ME=1, CF=0, SR=1, IL=0, TNF=1 */

    0x01,    /* Type Length 1 byte */

    0x0B,    /* Payload length 11bytes */

    0x55,    /* Type U (URI) */
    /* Payload start */

    0x01     /* URI Record Type : http://www.  */

};

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

void setup(void)
{
    Serial.begin(115200);
    while (!Serial);
    Serial.println("Hello!");
    pinMode(led, OUTPUT);
    digitalWrite(led, HIGH);
    //reset RF430
    nfc.begin();
    delay(500);
    
}


void loop(void)
{ 
    Serial.println("Updating!");

    updateNFC("this is a test nfc payload \" test231");
    delay(10000);
}

void updateNFC(String nfcString)
{
    int rawDataSize = nfcString.length() + 1; // strings are null terminated
    int templateSize = sizeof(nfcTemplate);
    byte nfcPayload[rawDataSize];
    nfcString.getBytes(nfcPayload, rawDataSize);

    
    //fix the length fields in the NFC tag
    
    int payloadSize = 3 + rawDataSize - 1; // -1 to remove null termated string
    int nlen = payloadSize + 4;
    Serial.println(payloadSize);
    Serial.println(nlen);

    nfcTemplate[27] = nlen;
    nfcTemplate[30] = payloadSize;

    byte nfcTag[templateSize + rawDataSize - 1];

    int pointer;
    for (int i = 0 ; i < templateSize; i++) {
      nfcTag[i] = nfcTemplate[i];
      pointer = i;
      }
    for (int f = 0; f < rawDataSize; f++){
      // Serial.println(nfcPayload[f]);
    }

    for (int i = 0, j = pointer + 1; i < rawDataSize ,  j < pointer + rawDataSize; i++, j++) { // -1 removes null terminating string
      nfcTag[j] = nfcPayload[i];
      Serial.println(nfcPayload[i]);
      }

    // update nfc tag

    while(!(nfc.Read_Register(STATUS_REG) & READY)); //wait until READY bit has been set
    Serial.print("Firmware Version:"); Serial.println(nfc.Read_Register(VERSION_REG), HEX);

    //write NDEF memory with Capability Container + NDEF message
    nfc.Write_Continuous(0, nfcTag, sizeof(nfcTag));

    //Enable interrupts for End of Read and End of Write
    nfc.Write_Register(INT_ENABLE_REG, EOW_INT_ENABLE + EOR_INT_ENABLE);

    //Configure INTO pin for active low and enable RF
    nfc.Write_Register(CONTROL_REG, INT_ENABLE + INTO_DRIVE + RF_ENABLE );

    //enable interrupt 1
    attachInterrupt(1, RF430_Interrupt, FALLING);

    Serial.println("Wait for read or write...");
    //while(1)
    while(1)
    {
        if(into_fired)
        {
            //clear control reg to disable RF
            nfc.Write_Register(CONTROL_REG, INT_ENABLE + INTO_DRIVE);
            delay(750);

            //read the flag register to check if a read or write occurred
            flags = nfc.Read_Register(INT_FLAG_REG);
            Serial.print("INT_FLAG_REG = 0x");Serial.println(flags, HEX);

            //ACK the flags to clear
            nfc.Write_Register(INT_FLAG_REG, EOW_INT_FLAG + EOR_INT_FLAG);

            if(flags & EOW_INT_FLAG)      //check if the tag was written
            {
                Serial.println("The tag was written!");
                digitalWrite(led, HIGH);
            }
            else if(flags & EOR_INT_FLAG) //check if the tag was read
            {
                Serial.println("The tag was read!");
                digitalWrite(led, LOW);
            }
            flags = 0;
            into_fired = 0; //we have serviced INT1


            //Enable interrupts for End of Read and End of Write
            nfc.Write_Register(INT_ENABLE_REG, EOW_INT_ENABLE + EOR_INT_ENABLE);

            //Configure INTO pin for active low and re-enable RF
            nfc.Write_Register(CONTROL_REG, INT_ENABLE + INTO_DRIVE + RF_ENABLE);

            //re-enable INTO
            attachInterrupt(1, RF430_Interrupt, FALLING);

            break;
        }
    }

}



/**
**  @brief  interrupt service
**/
void RF430_Interrupt()
{
    into_fired = 1;
    detachInterrupt(1);//cancel interrupt
}
