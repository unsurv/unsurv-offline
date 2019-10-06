/* Copyright 2013-2014 Ten Wong, wangtengoo7@gmail.com  
*  https://github.com/awong1900/RF430CL330H_Shield 
*  More info : http://www.elecfreaks.com
*/
//the I2C part of this code is borrowed from Adafruit_NFCShield_I2C
//link to original https://github.com/adafruit/Adafruit_NFCShield_I2C

//  RF430 User Address Map
//  -----------------------------------------
//  Address     | Size  | Description       |
//  -----------------------------------------
//  0xFFFE      | 2B    | Control Register  |
//  0xFFFC      | 2B    | Status Register   |
//  0xFFFA      | 2B    | Interrupt Enable  |
//  0xFFF8      | 2B    | Interrupt Flags   |
//  0xFFF6      | 2B    | CRC Result        |
//  0xFFF4      | 2B    | CRC Length        |
//  0xFFF2      | 2B    | CRC Start Address |
//  0xFFF0      | 2B    | Comm WD Ctrl Reg  |
//  -----------------------------------------
//  0x0000 -    | 2kB   | NDEF App Memory   |
//    0x07FF    |       |                   |
//
//
//
//                                /|\  /|\    (Host/Tester)
//                   RF430        10k  10k     Arduino 2560
//                  (Slave)        |    |        Master
//             _________________   |    |   _________________
//            |              SDA|<-|----+->|P20              |
//            |                 |  | I2C   |                 |
//            |              SCL|<-+------>|P21              |
//            |                 |          |                 |
//      GND<--|E(2-0)       /RST|<---------|P4               |
//            |             INTO|--------->|P3(INT1)         |
//            |                 |          |                 |
//            |                 |          |                 |
//            |                 |          |                 |
//            |_________________|          |_________________|
//
//******************************************************************************

#ifndef RF430CL330H_SHIELD_H_
#define RF430CL330H_SHIELD_H_
#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif
#include <Wire.h>

#define RF430_I2C_ADDRESS                   (0x50 >> 1)
#define RF430_I2C_READBIT                   (0x01)
#define RF430_I2C_BUSY                      (0x00)
#define RF430_I2C_READY                     (0x01)
#define RF430_I2C_READYTIMEOUT              (20)

#define BIT(_bit_)          (1 << (_bit_))
#define BIT0                0x0001
#define BIT1                0x0002
#define BIT2                0x0004
#define BIT3                0x0008
#define BIT4                0x0010
#define BIT5                0x0020
#define BIT6                0x0040
#define BIT7                0x0080
#define BIT8                0x0100
#define BIT9                0x0200
#define BIT10               0x0400
#define BIT11               0x0800
#define BIT12               0x1000
#define BIT13               0x2000
#define BIT14               0x4000
#define BIT15               0x8000

//define the values for Granite's registers we want to access
#define CONTROL_REG         0xFFFE
#define STATUS_REG          0xFFFC
#define INT_ENABLE_REG      0xFFFA
#define INT_FLAG_REG        0xFFF8
#define CRC_RESULT_REG      0xFFF6
#define CRC_LENGTH_REG      0xFFF4
#define CRC_START_ADDR_REG  0xFFF2
#define COMM_WD_CTRL_REG    0xFFF0
#define VERSION_REG         0xFFEE //contains the software version of the ROM
#define TEST_FUNCTION_REG   0xFFE2
#define TEST_MODE_REG       0xFFE0

//define the different virtual register bits
//CONTROL_REG bits
#define SW_RESET        BIT0
#define RF_ENABLE       BIT1
#define INT_ENABLE      BIT2
#define INTO_HIGH       BIT3
#define INTO_DRIVE      BIT4
#define BIP8_ENABLE     BIT5
#define STANDBY_ENABLE  BIT6
#define TEST430_ENABLE  BIT7

//STATUS_REG bits
#define READY           BIT0
#define CRC_ACTIVE      BIT1
#define RF_BUSY         BIT2

//INT_ENABLE_REG bits
#define EOR_INT_ENABLE              BIT1
#define EOW_INT_ENABLE              BIT2
#define CRC_INT_ENABLE              BIT3
#define BIP8_ERROR_INT_ENABLE       BIT4
#define NDEF_ERROR_INT_ENABLE       BIT5
#define GENERIC_ERROR_INT_ENABLE    BIT7

//INT_FLAG_REG bits
#define EOR_INT_FLAG            BIT1
#define EOW_INT_FLAG            BIT2
#define CRC_INT_FLAG            BIT3
#define BIP8_ERROR_INT_FLAG     BIT4
#define NDEF_ERROR_INT_FLAG     BIT5
#define GENERIC_ERROR_INT_FLAG  BIT7

//COMM_WD_CTRL_REG bits
#define WD_ENABLE               BIT0
#define TIMEOUT_PERIOD_2_SEC    0
#define TIMEOUT_PERIOD_32_SEC   BIT1
#define TIMEOUT_PERIOD_8_5_MIN  BIT2
#define TIMEOUT_PERIOD_MASK     BIT1 + BIT2 + BIT3

#define TEST_MODE_KEY           0x004E

#define RF430_DEFAULT_DATA                                                              \
{                                                                                       \
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
0x00, 0x2A, /* NLEN; NDEF length (2 byte long message) */                               \
0xD1, /* Record Header  */                                                              \ 
0x01, /* Type Length */                                                                 \
0x26, /* bytes after this -1 byte  = NLEN - 4*/                                 \
0x54, /* type  T = text */                                                              \
0x02,  /* ID length  */                                                                 \
0x65, 0x6E, /* 'e', 'n', */                                                             \
                                                                                        \
/* 'Hello, world!' NDEF data; Empty NDEF message, length should match NLEN*/            \
0x68, 0x65, 0x72, 0x65, 0x20, 0x63, 0x6f, 0x75, 0x6c, 0x64, 0x20, 0x62, 0x65, 0x20,     \
0x61, 0x20, 0x6a, 0x73, 0x6f, 0x6e, 0x20, 0x66, 0x69, 0x6c, 0x65, 0x20, 0x77, 0x69,     \
0x74, 0x68, 0x20, 0x69, 0x6e, 0x66, 0x6f      \
}

class RF430CL330H_Shield
{
public:
    RF430CL330H_Shield(uint8_t irq, uint8_t reset);
    void begin();
    
    uint16_t Read_Register(uint16_t reg_addr);
    uint8_t Read_OneByte(uint16_t reg_addr); 
    void Read_Continuous(uint16_t reg_addr, uint8_t* read_data, uint16_t data_length);

    void Write_Register(uint16_t reg_addr, uint16_t value);
    void Write_Continuous(uint16_t reg_addr, uint8_t* write_data, uint16_t data_length);
    void Write_NDEFmessage(uint8_t* msgNDEF, uint16_t msg_length);
    void SetReadOnly(uint8_t onOff);
private:
    byte RxData[2];
    byte TxData[2];
    byte TxAddr[2];
    
    uint8_t _irq, _reset;
};

#endif /* RF430CL330H_SHIELD_H_ */
