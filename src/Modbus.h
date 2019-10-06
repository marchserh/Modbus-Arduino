/*
    Modbus library for Arduino
    
    Created: 10/2019    
    Author: Serhii Marchuk <marchserh@gmail.com>
    
    Copyright (C) 2019  Serhii Marchuk

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    
*/

#ifndef MODBUS_H
#define MODBUS_H

/*
   Major part of modbuslib version
*/
#define MODBUSLIB_VERSION_MAJOR 0

/*
   Minor part of modbuslib version
*/
#define MODBUSLIB_VERSION_MINOR 1

/*
   Patch part of modbuslib version
*/
#define MODBUSLIB_VERSION_PATCH 0

/*
   MODBUSLIB_VERSION is (major << 16) + (minor << 8) + patch.
*/
#define MODBUSLIB_VERSION ((MODBUSLIB_VERSION_MAJOR<<16)|(MODBUSLIB_VERSION_MINOR<<8)|(MODBUSLIB_VERSION_PATCH))

/*
   MODBUSLIB_VERSION_STR "major.minor.patch"
*/
#define MODBUSLIB_VERSION_STR_HELPER(major,minor,patch) #major"."#minor"."#patch

#define MODBUSLIB_VERSION_STR_MAKE(major,minor,patch) MODBUSLIB_VERSION_STR_HELPER(major,minor,patch)

#define MODBUSLIB_VERSION_STR MODBUSLIB_VERSION_STR_MAKE(MODBUSLIB_VERSION_MAJOR,MODBUSLIB_VERSION_MINOR,MODBUSLIB_VERSION_PATCH)

/*
   Modbus NULL-pointer
*/
#ifdef NULL
#define MB_NULLPTR NULL
#else
#define MB_NULLPTR ((void*)0)
#endif


// --------------------------------------------------------------------------------------------------------
// -------------------------------------------- Helper macros ---------------------------------------------
// --------------------------------------------------------------------------------------------------------

#define GET_BIT(bitBuff, bitNum) ((reinterpret_cast<const uint8_t*>(bitBuff)[bitNum/8] & (1<<(bitNum%8))) != 0)

#define SET_BIT(bitBuff, bitNum, value)                                                                                   \
    if (value)                                                                                                            \
        reinterpret_cast<uint8_t*>(bitBuff)[bitNum/8] |= (1<<(bitNum%8));                                                 \
    else                                                                                                                  \
        reinterpret_cast<uint8_t*>(bitBuff)[bitNum/8] &= (~(1<<(bitNum%8)));

#define GET_BITS(bitBuff, bitNum, bitCount, boolBuff)                                                                     \
    for (uint16_t __i__ = 0; __i__ < bitCount; __i__++)                                                                   \     
        boolBuff[__i__] = (reinterpret_cast<const uint8_t*>(bitBuff)[(bitNum+__i__)/8] & (1<<((bitNum+__i__)%8))) != 0;
                                                          
#define SET_BITS(bitBuff, bitNum, bitCount, boolBuff)                                                                     \
    for (uint16_t __i__ = 0; __i__ < bitCount; __i__++)                                                                   \     
        if (boolBuff[__i__])                                                                                              \
            reinterpret_cast<uint8_t*>(bitBuff)[(bitNum+__i__)/8] |= (1<<((bitNum+__i__)%8));                             \
        else                                                                                                              \
            reinterpret_cast<uint8_t*>(bitBuff)[(bitNum+__i__)/8] &= (~(1<<((bitNum+__i__)%8)));  
            

// --------------------------------------------------------------------------------------------------------
// ----------------------------------------- Modbus function codes ----------------------------------------
// --------------------------------------------------------------------------------------------------------

#define MBF_READ_COIL_STATUS                    1
#define MBF_READ_INPUT_STATUS                   2
#define MBF_READ_HOLDING_REGISTERS              3
#define MBF_READ_INPUT_REGISTERS                4
#define MBF_FORCE_SINGLE_COIL                   5
#define MBF_FORCE_SINGLE_REGISTER               6
#define MBF_READ_EXCEPTION_STATUS               7
#define MBF_LOOPBACK_DIAGNOSTIC_TEST            8
#define MBF_PROGRAM484                          9
#define MBF_POLL484                             10
#define MBF_FETCH_EVENT_COUNTER_COMMUNICATIONS  11
#define MBF_FETCH_COMMUNICATION_EVENT_LOG       12
#define MBF_PROGRAM                             13
#define MBF_POOL_PROGRAM_COMPLETE               14
#define MBF_FORCE_MULTIPLE_COILS                15
#define MBF_FORCE_MULTIPLE_REGISTERS            16
#define MBF_REPORT_SLAVE_ID                     17
#define MBF_RESET_COMMUNICATIONS_LINK           19
#define MBF_READ_GENERAL_REFERENCE              20
#define MBF_WRITE_GENERAL_REFERENCE             21
#define MBF_MASK_WRITE_4X_REGISTER              22
#define MBF_READ_WRITE_4X_REGISTER              23
#define MBF_READ_FIFO_QUEUE                     24
#define MBF_ILLEGAL_FUNCTION                    73 
#define MBF_EXCEPTION                           128    
                 

// --------------------------------------------------------------------------------------------------------
// --------------------------------------------- Maximum count --------------------------------------------
// --------------------------------------------------------------------------------------------------------

// 127 = 255(count_of_bytes-byte in function readHoldingRegisters etc) / 2 (register size in bytes) 
#define MB_MAX_REGISTERS 127

// 2040 = 255(count_of_bytes-byte in function readCoilStatus etc) * 8 (bits in byte) 
#define MB_MAX_DISCRETS 2040

// --------------------------------------------------------------------------------------------------------
// ---------------------------------------------- Buffer size ---------------------------------------------
// --------------------------------------------------------------------------------------------------------

// 1 byte(slave)+1 byte(function)+256 bytes(maximum data length e.g. readCoilStatus etc)+2 bytes(CRC)
#define MB_RTU_IO_BUFF_SZ 260

// 6 bytes(tcp-prefix)+1 byte(slave)+1 byte(function)+256 bytes(maximum data length e.g. readCoilStatus etc)
#define MB_TCP_IO_BUFF_SZ 266


// --------------------------------------------------------------------------------------------------------
// ------------------------------------------- Modbus namespace -------------------------------------------
// --------------------------------------------------------------------------------------------------------

#include <inttypes.h>

class Stream;

namespace Modbus
{

// --------------------------------------------------------------------------------------------------------
// ---------------------------------------- Modbus Constant values ----------------------------------------
// --------------------------------------------------------------------------------------------------------

enum Constants
{
    VALID_MODBUS_ADDRESS_BEGIN = 1,
    VALID_MODBUS_ADDRESS_END = 247,
    STANDARD_TCP_PORT = 502
};


// --------------------------------------------------------------------------------------------------------
// ---------------------------------------- Modbus protocol types ----------------------------------------
// --------------------------------------------------------------------------------------------------------

enum Type
{
    ASC,
    RTU,
    TCP
};


// --------------------------------------------------------------------------------------------------------
// ---------------------------------------- Modbus response values ----------------------------------------
// --------------------------------------------------------------------------------------------------------

enum Response
{
    PROCESSING                          = -1,
    OK                                  = 0,
    ILLEGAL_FUNCTION                    = 1,
    ILLEGAL_DATA_ADDRESS                = 2,      
    ILLEGAL_DATA_VALUE                  = 3,
    SLAVE_DEVICE_FAILURE                = 4,
    ACKNOWLEDGE                         = 5,
    SLAVE_DEVICE_BUSY                   = 6,
    NEGATIVE_ACKNOWLEDGE                = 7,
    MEMORY_PARITY_ERROR                 = 8,
    CMN_ERR_NO_RESPONSE                 = 32,
    CMN_ERR_NOT_CORRECT                 = 33,
    CMN_ERR_READ_BUFF_OVERFLOW          = 34,
    CMN_ERR_WRITE_BUFF_OVERFLOW         = 35,
    SERIAL_ERR_OPEN                     = 64,
    SERIAL_ERR_READ                     = 65,
    SERIAL_ERR_WRITE                    = 66,
    ASCII_ERR_MISS_COLON                = 72,
    ASCII_ERR_MISS_CRLF                 = 73,
    ASCII_ERR_BAD_CHAR                  = 74,
    ASCII_ERR_LRC                       = 75,
    RTU_ERR_CRC                         = 80,
    TCP_ERR_CONNECT                     = 88,
    TCP_ERR_RECV                        = 89,
    TCP_ERR_SEND                        = 90,
    TCP_ERR_DISCONNECT                  = 91,
    UNKNOWN_ERROR                       = 127,
    TCP_ERR_SERVER                      = 256
};


// --------------------------------------------------------------------------------------------------------
// ------------------------------------------ Modbus address type -----------------------------------------
// --------------------------------------------------------------------------------------------------------

enum Address
{
    X0 = 0,
    M = X0,
    X1 = 1,
    I = X1,
    X3 = 3,
    IW = X3,
    X4 = 4,
    MW = X4 
};


// --------------------------------------------------------------------------------------------------------
// ------------------------------------------- Helper functions -------------------------------------------
// --------------------------------------------------------------------------------------------------------

uint16_t crc16(const uint8_t* data, uint16_t szData);
uint8_t lrc(const uint8_t* data, uint16_t szData);
void printBytes(Stream *debug, uint8_t *bytes, uint16_t count);

inline bool getBit(const void* bitBuff, uint16_t bitNum) { return GET_BIT (bitBuff, bitNum); }
inline bool getBit(const void* bitBuff, uint16_t bitNum, uint16_t maxBitCount) { return (bitNum < maxBitCount) ? getBit(bitBuff, bitNum) : false; }

inline void setBit(void* bitBuff, uint16_t bitNum, bool value) { SET_BIT (bitBuff, bitNum, value) }
inline void setBit(void* bitBuff, uint16_t bitNum, bool value, uint16_t maxBitCount) { if (bitNum < maxBitCount) setBit(bitBuff, bitNum, value); }

inline bool* getBits(const void* bitBuff, uint16_t bitNum, uint16_t bitCount, bool* boolBuff) { GET_BITS(bitBuff, bitNum, bitCount, boolBuff) return boolBuff; }
inline bool* getBits(const void* bitBuff, uint16_t bitNum, uint16_t bitCount, bool* boolBuff, uint16_t maxBitCount) { if (bitNum < maxBitCount) getBits(bitBuff, bitNum, bitCount, boolBuff); return boolBuff; }

inline void* setBits(void* bitBuff, uint16_t bitNum, uint16_t bitCount, const bool* boolBuff) { SET_BITS(bitBuff, bitNum, bitCount, boolBuff) return bitBuff; }
inline void* setBits(void* bitBuff, uint16_t bitNum, uint16_t bitCount, const bool* boolBuff, uint16_t maxBitCount) { if (bitNum < maxBitCount) setBits(bitBuff, bitNum, bitCount, boolBuff); return bitBuff; }

}; // namespace Modbus


// --------------------------------------------------------------------------------------------------------
// ------------------------------------------- MODBUS INTERFACE -------------------------------------------
// --------------------------------------------------------------------------------------------------------

/*

Main ModbusInterface parameters:
* slave     - unsigned 8 bit integer reference value that represents modbus slave address  
              It may be NULL and in this case you can get the real address of remote device
              Valid values: 0       - for shared address
                          [1..247]  - valid device's address range    
* offset    - unsigned 16 bit integer that represents address offset, e.g. 400001 address has 0-offset (in most cases)
* count     - unsigned 16 bit integer that represents count of bits/registers to read/write
* bits      - array of bits. It can represent a pointer to any value. 
              Each value of coil/discrete input is represented by 1 bit. Bit0 is a first bit to read/write, bit1 - second and so on
              For example if you want to set 4 coils like:
              0 coil - to 1
              1 coil - to 1
              2 coil - to 0
              3 coil - to 1
              you can write code like:
```c++              
uint8_t values = 0x0B; // 0b00001011
mb.forceMultipleCoils(slave, 0, 4, &values);
```
              
* values    - array of unsigned 16 bit integer register's buffer to read/write
* fact      - unnecessary output parameter. May be NULL. 
              It's pointer to unsigned 16 bit integer that means the factual count of read/write values.
              
Returned value:
* Modbus::Response - returned result of function. May have values:
                     * result == 0  - (Modbus::OK) success result of function
                     * result > 0   - an error occured. It may be standard modbus error [1..8] or library-enumerated error value
                     * result < 0   - (Modbus::PROCESSING) means that function is not finish it execution
*Note*: For ModbusMaster.. derived interface classes used NON blocking execution mode. If interface function not finish it network
      process it returns result < 0 (Modbus::PROCESSING) that is an indicator of not finished operation.
      For ModbusMemory derived class functions return immediately.

*/

class ModbusInterface
{
public:
    virtual Modbus::Response readCoilStatus(uint8_t &slave, uint16_t offset, uint16_t count, void* bits, uint16_t* fact = MB_NULLPTR) = 0;
    virtual Modbus::Response readInputStatus(uint8_t &slave, uint16_t offset, uint16_t count, void* bits, uint16_t* fact = MB_NULLPTR) = 0;
    virtual Modbus::Response readHoldingRegisters(uint8_t &slave, uint16_t offset, uint16_t count, uint16_t* values, uint16_t* fact = MB_NULLPTR) = 0;
    virtual Modbus::Response readInputRegisters(uint8_t &slave, uint16_t offset, uint16_t count, uint16_t* values, uint16_t* fact = MB_NULLPTR) = 0;
    virtual Modbus::Response forceSingleCoil(uint8_t &slave, uint16_t offset, bool value) = 0;
    virtual Modbus::Response forceSingleRegister(uint8_t &slave, uint16_t offset, uint16_t value) = 0;
    virtual Modbus::Response forceMultipleCoils(uint8_t &slave, uint16_t offset, uint16_t count, const void* bits, uint16_t* fact = MB_NULLPTR) = 0;
    virtual Modbus::Response forceMultipleRegisters(uint8_t &slave, uint16_t offset, uint16_t count, const uint16_t* values, uint16_t* fact = MB_NULLPTR) = 0;
};


#endif // MODBUS_H