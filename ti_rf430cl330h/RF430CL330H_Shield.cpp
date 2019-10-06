/* Copyright 2013-2014 Ten Wong, wangtengoo7@gmail.com  
*  https://github.com/awong1900/RF430CL330H_Shield 
*  More info : http://www.elecfreaks.com
*/
//the I2C part of this code is borrowed from Adafruit_NFCShield_I2C
//link to original https://github.com/adafruit/Adafruit_NFCShield_I2C

#include "RF430CL330H_Shield.h"
#define I2C_BUFFER_LENGTH  30   //because library Wire's I2C buffer default is 32 bytes

/* Uncomment these lines to enable debug output for RF430(I2C) */
//#define RF430DEBUG
 
/**
**  @brief  Sends a single byte via I2C
**  @param  x    The byte to send
**/
static inline void wiresend(uint8_t x) 
{
  #if ARDUINO >= 100
    Wire.write((uint8_t)x);
  #else
    Wire.send(x);
  #endif
}


/**
**  @brief  Reads a single byte via I2C
**/
static inline uint8_t wirerecv(void) 
{
  #if ARDUINO >= 100
    return Wire.read();
  #else
    return Wire.receive();
  #endif
}


/**
**  @brief  Instantiates a new RF430 class
**  @param  irq       Location of the IRQ pin
**  @param  reset     Location of the RSTPD_N pin
**/
RF430CL330H_Shield::RF430CL330H_Shield(uint8_t irq, uint8_t reset)
{
    byte RxData[2] = {0,0};
    byte TxData[2] = {0,0};
    byte TxAddr[2] = {0,0};

    //_irq = irq;
    _reset = reset;

    //pinMode(_irq, INPUT); //arduino's interrupt do not need init
    pinMode(_reset, OUTPUT);
}


/** 
**  @brief  Setups the HW 
**/
void RF430CL330H_Shield::begin()
{
    uint16_t version;
    Wire.begin();
    // Reset the RF430  
    digitalWrite(_reset, HIGH);
    digitalWrite(_reset, LOW);
    delay(100);                   //Reset:low level 100ms
    digitalWrite(_reset, HIGH);
    delay(1000);
    
    while(!(Read_Register(STATUS_REG) & READY)); //wait until READY bit has been set

    version = Read_Register(VERSION_REG);
    Serial.print("Fireware Version:");Serial.println(version, HEX);    

    /** Errata Fix : Unresponsive RF - recommended firmware
    *   reference: RF430CL330H Device Erratasheet, SLAZ540D-June 2013-Revised January
    */
    if (version == 0x0101 || version == 0x0201)
    { // the issue exists in these two versions
        Write_Register(TEST_MODE_REG, TEST_MODE_KEY);
        Write_Register(CONTROL_REG, 0x0080);
        if (version == 0x0101)
        { // Ver C
            Write_Register(0x2a98, 0x0650);
        }
        else
        { // Ver D
            Write_Register(0x2a6e, 0x0650);
        }
        Write_Register(0x2814, 0);
        Write_Register(TEST_MODE_REG, 0);
    }
    //Upon exit of this block, the control register is set to 0x0
    /** Fix end */
    
    byte NDEF_Application_Data[] = RF430_DEFAULT_DATA;
    //write NDEF memory with Capability Container + NDEF message
    Write_Continuous(0, NDEF_Application_Data, sizeof(NDEF_Application_Data));

    //Enable interrupts for End of Read and End of Write
    Write_Register(INT_ENABLE_REG, EOW_INT_ENABLE + EOR_INT_ENABLE);

    //Configure INTO pin for active low and enable RF
    Write_Register(CONTROL_REG, INT_ENABLE + INTO_DRIVE + RF_ENABLE );
}


/** 
**  @brief  Reads the register at reg_addr, returns the result
**  @param  uint16_t    reg_addr    RF430 Register address
**  @retrun uint16_t    the value of register
**/
uint16_t RF430CL330H_Shield::Read_Register(uint16_t reg_addr)
{
    TxAddr[0] = reg_addr >> 8;      // MSB of address
    TxAddr[1] = reg_addr & 0xFF;    // LSB of address TxAddr[0]TxAddr[1]
#ifdef RF430DEBUG    
    Serial.print("Read_Register[0x");Serial.print(reg_addr, HEX);Serial.print("]:0x");
#endif
    //send slave addr
    Wire.beginTransmission(RF430_I2C_ADDRESS);
    wiresend(TxAddr[0]); //bit15~8
    wiresend(TxAddr[1]); //bit7~0
    Wire.endTransmission(); 
    
    //resend slave addr with data
    Wire.requestFrom((uint8_t)RF430_I2C_ADDRESS, (uint8_t)2);
    RxData[0] = wirerecv();
    RxData[1] = wirerecv();
    
    //send stop
    Wire.endTransmission();
    
#ifdef RF430DEBUG    
    Serial.println(RxData[1] << 8 | RxData[0], HEX);
#endif
    return RxData[1] << 8 | RxData[0];
}

/** 
**  @brief  Reads one byte data at reg_addr, returns the result
**  @param  uint16_t    reg_addr    RF430 Register address (16-bit)
**  @retrun uint8_t    the value(byte) of register
**/
uint8_t RF430CL330H_Shield::Read_OneByte(uint16_t reg_addr)
{
    byte buf[1];
    Read_Continuous(reg_addr, buf, 1);
    return buf[0];
}

/** 
**  @brief  Continuous read data_length bytes and store in the area "read_data"
**  @param  uint16_t    reg_addr       RF430 Register address
**  @param  uint8_t*    read_data      buffer for store the data
**  @param  uint16_t    data_length    length of data    
**  @retrun void
**/
void RF430CL330H_Shield::Read_Continuous(uint16_t reg_addr, uint8_t* read_data, uint16_t data_length)
{
    uint16_t split_num = 0;
    uint16_t remainder = data_length;
    uint16_t index= 0;

    //splite data avoid beyond the wire buffer
    if (data_length > I2C_BUFFER_LENGTH)
    {
        split_num = data_length / I2C_BUFFER_LENGTH;
        remainder = data_length % I2C_BUFFER_LENGTH;
    }
    
    for (uint8_t k=0; k < split_num+1; k++)
    {
        //send slave addr
        Wire.beginTransmission(RF430_I2C_ADDRESS);
        TxAddr[0] = reg_addr >> 8;      // MSB of address
        TxAddr[1] = reg_addr & 0xFF;    // LSB of address
        wiresend(TxAddr[0]); //bit15~8
        wiresend(TxAddr[1]); //bit7~0
        Wire.endTransmission();

        if (k != split_num)
        {
            //resend slave addr with data
            Wire.requestFrom((uint8_t)RF430_I2C_ADDRESS, (uint8_t)I2C_BUFFER_LENGTH);
            while (Wire.available())
            {
              for (uint8_t i=0; i < I2C_BUFFER_LENGTH; i++) 
                  read_data[index+i] = wirerecv();
            }

            //increase addr
            index += I2C_BUFFER_LENGTH;
            reg_addr += I2C_BUFFER_LENGTH;
        }
        else
        {
            //resend slave addr with data
            Wire.requestFrom((uint8_t)RF430_I2C_ADDRESS, remainder);
            while (Wire.available())
            {
                for (uint8_t i=0; i < remainder; i++) 
                  read_data[index+i] = wirerecv();
            }
        }

        //send stop
        Wire.endTransmission();
    }

#ifdef RF430DEBUG    
    Serial.print("RxData[] = 0x");
    for (uint8_t i=0; i<data_length; i++) 
        {Serial.print(read_data[i], HEX);Serial.print(" ");}
    Serial.println("");
#endif
}


/** 
**  @brief  writes the register at reg_addr with value
**  @param  uint16_t    reg_addr    RF430 Register address
**  @param  uint16_t    value       writted value   
**  @retrun void
**/
void RF430CL330H_Shield::Write_Register(uint16_t reg_addr, uint16_t value)
{
    TxAddr[0] = reg_addr >> 8;      // MSB of address
    TxAddr[1] = reg_addr & 0xFF;    // LSB of address
    TxData[0] = value >> 8;
    TxData[1] = value & 0xFF;
#ifdef RF430DEBUG    
        Serial.print("Write_Register[0x");Serial.print(reg_addr, HEX);
        Serial.print("]:0x");Serial.println(value, HEX);
#endif
    //send slave addr
    Wire.beginTransmission(RF430_I2C_ADDRESS);
    wiresend(TxAddr[0]); //bit15~8
    wiresend(TxAddr[1]); //bit7~0

    //send value
    wiresend(TxData[1]); //bit7~0
    wiresend(TxData[0]); //bit15~8

    //send stop
    Wire.endTransmission();
    
}


/** 
**  @brief  writes the register at reg_addr and incrementing addresses with the data at "write_data" of length data_length
**  @param  uint16_t    reg_addr       RF430 Register address
**  @param  uint8_t*    write_data     buffer for store the data
**  @param  uint16_t    data_length    length of data    
**  @retrun void
**/
void RF430CL330H_Shield::Write_Continuous(uint16_t reg_addr, uint8_t* write_data, uint16_t data_length)
{
    uint16_t split_num = 0;
    uint16_t remainder = data_length;
    uint16_t index=0;
#ifdef RF430DEBUG    
    Serial.print("start_addr = 0x");Serial.println(reg_addr, HEX);
    Serial.print("data_length = 0x");Serial.println(data_length, HEX);
    Serial.print("write_data[] = ");
    for (uint8_t i=0; i<data_length; i++)
        {Serial.print(write_data[i], HEX);Serial.print(" ");}
    Serial.println();  
#endif

    //splite data avoid beyond the wire buffer
    if(data_length > I2C_BUFFER_LENGTH)
    {
        split_num = data_length / I2C_BUFFER_LENGTH;
        remainder = data_length % I2C_BUFFER_LENGTH;
    }

    for (int k=0; k < split_num+1; k++)
    {
        //send slave addr
        Wire.beginTransmission(RF430_I2C_ADDRESS);

        //Serial.print("reg_addr = 0x");Serial.println(reg_addr, HEX);
        TxAddr[0] = reg_addr >> 8;        // MSB of address
        TxAddr[1] = reg_addr & 0xFF;      // LSB of address
        wiresend(TxAddr[0]); //bit15~8
        wiresend(TxAddr[1]); //bit7~0

        if (k != split_num)
        {
            //send data
            for (uint8_t i=0; i < I2C_BUFFER_LENGTH; i++) 
                wiresend(write_data[index+i]); 
                
            //increase addr
            index += I2C_BUFFER_LENGTH;
            reg_addr += I2C_BUFFER_LENGTH;
        } 
        else 
        {
            //send data
            for (uint8_t i=0; i < remainder; i++) 
                wiresend(write_data[index+i]); 
        }
        //send stop
        Wire.endTransmission();

    }

}


/** 
**  @brief  writes the NDEF message to RF430 memory
**  @param  uint8_t*    msgNDEF     buffer for store the NDEF message
**  @param  uint16_t    msg_length  length of message    
**  @retrun void
**/
void RF430CL330H_Shield::Write_NDEFmessage(uint8_t* msgNDEF, uint16_t msg_length)
{
    byte buf[2];
    buf[0] = msg_length >> 8;      // MSB of message length
    buf[1] = msg_length & 0xFF;    // LSB of message length 

    while (Read_Register(STATUS_REG) & RF_BUSY)
        delay(1000);
    //clear control reg to disable RF
    Write_Register(CONTROL_REG, Read_Register(CONTROL_REG) & ~RF_ENABLE); 

    //write message data
    Write_Continuous(0x1A, buf, 2);
    Write_Continuous(0x1C, msgNDEF, msg_length);

    //Configure INTO pin for active low and enable RF
    Write_Register(CONTROL_REG, Read_Register(CONTROL_REG) | RF_ENABLE); 
}

/**  @brief  set NDEF message is read-only
**  @param  uint8_t    onOff     true: read-only
**  @retrun void
**/
void RF430CL330H_Shield::SetReadOnly(uint8_t onOff)
{
    byte buf[1];
    
    if (onOff == true)
        buf[0] = 0xFF;    //0x00:write, 0xFF:read-only
    else
        buf[0] = 0x00;
        
    while (Read_Register(STATUS_REG) & RF_BUSY)
        delay(1000);
    //clear control reg to disable RF
    Write_Register(CONTROL_REG, Read_Register(CONTROL_REG) & ~RF_ENABLE); 

    //write access data
    Write_Continuous(0x17, buf, 1); 
    
    //Configure INTO pin for active low and enable RF
    Write_Register(CONTROL_REG, Read_Register(CONTROL_REG) | RF_ENABLE); 
}
