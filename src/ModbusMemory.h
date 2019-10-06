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

/*
    ModbusMemory class provides some functionality to manage modbus memory
    such as read, write and copy
*/

#ifndef MODBUS_MEMORY_H
#define MODBUS_MEMORY_H

#include <string.h>

#include "Modbus.h"

#define MODBUS_REGE_SZ_BITES 16
#define MODBUS_BYTE_SZ_BITES 8
#define MODBUS_REGE_SZ_BYTES 2

#ifndef MODBUS_MEMORY_COUNT_0x
#define MODBUS_MEMORY_COUNT_0x 256
#endif
#ifndef MODBUS_MEMORY_COUNT_1x
#define MODBUS_MEMORY_COUNT_1x 256
#endif
#ifndef MODBUS_MEMORY_COUNT_3x
#define MODBUS_MEMORY_COUNT_3x 16
#endif
#ifndef MODBUS_MEMORY_COUNT_4x
#define MODBUS_MEMORY_COUNT_4x 16
#endif

#define MODBUS_MEMORY_SZ_0x_BITES (MODBUS_MEMORY_COUNT_0x)
#define MODBUS_MEMORY_SZ_0x_BYTES ((MODBUS_MEMORY_COUNT_0x)/(MODBUS_BYTE_SZ_BITES)+((MODBUS_MEMORY_COUNT_0x)%(MODBUS_BYTE_SZ_BITES)!=0))
#define MODBUS_MEMORY_SZ_0x_REGES ((MODBUS_MEMORY_COUNT_0x)/(MODBUS_REGE_SZ_BITES)+((MODBUS_MEMORY_COUNT_0x)%(MODBUS_REGE_SZ_BITES)!=0))

#define MODBUS_MEMORY_SZ_1x_BITES (MODBUS_MEMORY_COUNT_1x)
#define MODBUS_MEMORY_SZ_1x_BYTES ((MODBUS_MEMORY_COUNT_1x)/(MODBUS_BYTE_SZ_BITES)+((MODBUS_MEMORY_COUNT_1x)%(MODBUS_BYTE_SZ_BITES)!=0))
#define MODBUS_MEMORY_SZ_1x_REGES ((MODBUS_MEMORY_COUNT_1x)/(MODBUS_REGE_SZ_BITES)+((MODBUS_MEMORY_COUNT_1x)%(MODBUS_REGE_SZ_BITES)!=0))

#define MODBUS_MEMORY_SZ_3x_BITES (static_cast<uint32_t>(MODBUS_MEMORY_COUNT_3x)*16)
#define MODBUS_MEMORY_SZ_3x_BYTES (static_cast<uint32_t>(MODBUS_MEMORY_COUNT_3x)*2)
#define MODBUS_MEMORY_SZ_3x_REGES (MODBUS_MEMORY_COUNT_3x)

#define MODBUS_MEMORY_SZ_4x_BITES (static_cast<uint32_t>(MODBUS_MEMORY_COUNT_4x)*16)
#define MODBUS_MEMORY_SZ_4x_BYTES (static_cast<uint32_t>(MODBUS_MEMORY_COUNT_4x)*2)
#define MODBUS_MEMORY_SZ_4x_REGES (MODBUS_MEMORY_COUNT_4x)


// --------------------------------------------------------------------------------------------------------
// ----------------------------------------- MODBUS MEMORY DEVICE -----------------------------------------
// --------------------------------------------------------------------------------------------------------

class ModbusMemory : public ModbusInterface
{  
public:
    ModbusMemory();
 
public: // Modbus Interface
    virtual Modbus::Response readCoilStatus(uint8_t &slave, uint16_t offset, uint16_t count, void* bits, uint16_t* fact = MB_NULLPTR);
    virtual Modbus::Response readInputStatus(uint8_t &slave, uint16_t offset, uint16_t count, void* bits, uint16_t* fact = NULL);
    virtual Modbus::Response readHoldingRegisters(uint8_t &slave, uint16_t offset, uint16_t count, uint16_t* values, uint16_t* fact = MB_NULLPTR);
    virtual Modbus::Response readInputRegisters(uint8_t &slave, uint16_t offset, uint16_t count, uint16_t* values, uint16_t* fact = MB_NULLPTR);
    virtual Modbus::Response forceSingleCoil(uint8_t &slave, uint16_t offset, bool value);
    virtual Modbus::Response forceSingleRegister(uint8_t &slave, uint16_t offset, uint16_t value);
    virtual Modbus::Response forceMultipleCoils(uint8_t &slave, uint16_t offset, uint16_t count, const void* bits, uint16_t* fact = MB_NULLPTR);
    virtual Modbus::Response forceMultipleRegisters(uint8_t &slave, uint16_t offset, uint16_t count, const uint16_t* values, uint16_t* fact = MB_NULLPTR);

public:
    Modbus::Response copy(Modbus::Address srcType, uint16_t srcOffset, uint16_t count, Modbus::Address destType, uint16_t destOffset, uint16_t* fact = NULL);

public:
#if MODBUS_MEMORY_COUNT_0x > 0 
    void print_0x(Stream& serial, uint16_t offset = 0, uint16_t count = MODBUS_MEMORY_COUNT_0x, int number = DEC);
#endif // MODBUS_MEMORY_COUNT_0x > 0
#if MODBUS_MEMORY_COUNT_1x > 0 
    void print_1x(Stream& serial, uint16_t offset = 0, uint16_t count = MODBUS_MEMORY_COUNT_1x, int number = DEC);
#endif // MODBUS_MEMORY_COUNT_1x > 0
#if MODBUS_MEMORY_COUNT_3x > 0
    void print_3x(Stream& serial, uint16_t offset = 0, uint16_t count = MODBUS_MEMORY_COUNT_3x, int number = DEC);
#endif // MODBUS_MEMORY_COUNT_3x > 0
#if MODBUS_MEMORY_COUNT_4x > 0  
    void print_4x(Stream& serial, uint16_t offset = 0, uint16_t count = MODBUS_MEMORY_COUNT_4x, int number = DEC);
#endif // MODBUS_MEMORY_COUNT_4x > 0

#if MODBUS_MEMORY_COUNT_0x > 0   
public: // memory-0x management functions
    inline void zerroAll_0x() { memset(m_mem0x, 0, MODBUS_MEMORY_SZ_0x_BYTES); }
    Modbus::Response read_0x(uint16_t bitOffset, uint16_t bitCount, void* bits, uint16_t* fact = MB_NULLPTR) const;
    Modbus::Response write_0x(uint16_t bitOffset, uint16_t bitCount, const void* bits, uint16_t* fact = MB_NULLPTR);    
    bool bool_0x(uint16_t bitOffset) const;
    void setBool_0x(uint16_t bitOffset, bool v);
    int8_t int8_0x(uint16_t bitOffset) const; 
    void setInt8_0x(uint16_t bitOffset, int8_t v);
    uint8_t uint8_0x(uint16_t bitOffset) const;  
    void setUInt8_0x(uint16_t bitOffset, uint8_t v);
    int16_t int16_0x(uint16_t bitOffset) const; 
    void setInt16_0x(uint16_t bitOffset, int16_t v);
    uint16_t uint16_0x(uint16_t bitOffset) const;  
    void setUInt16_0x(uint16_t bitOffset, uint16_t v);
    int32_t int32_0x(uint16_t bitOffset) const; 
    void setInt32_0x(uint16_t bitOffset, int32_t v);
    uint32_t uint32_0x(uint16_t bitOffset) const;  
    void setUInt32_0x(uint16_t bitOffset, uint32_t v);
    int int_0x(uint16_t bitOffset) const; 
    void setInt_0x(uint16_t bitOffset, int v);
    unsigned int uint_0x(uint16_t bitOffset) const; 
    void setUInt_0x(uint16_t bitOffset, unsigned int v);    
    float float_0x(uint16_t bitOffset) const;
    void setFloat_0x(uint16_t bitOffset, float v);
    double double_0x(uint16_t bitOffset) const;
    void setDouble_0x(uint16_t bitOffset, double v);
#endif  // MODBUS_MEMORY_COUNT_0x > 0   

#if MODBUS_MEMORY_COUNT_1x > 0   
public: // memory-1x management functions
    inline void zerroAll_1x() { memset(m_mem1x, 0, MODBUS_MEMORY_SZ_1x_BYTES); }
    Modbus::Response read_1x(uint16_t bitOffset, uint16_t bitCount, void* bits, uint16_t* fact = MB_NULLPTR) const;
    Modbus::Response write_1x(uint16_t bitOffset, uint16_t bitCount, const void* bits, uint16_t* fact = MB_NULLPTR);
    bool bool_1x(uint16_t bitOffset) const;
    void setBool_1x(uint16_t bitOffset, bool v);     
    int8_t int8_1x(uint16_t bitOffset) const; 
    void setInt8_1x(uint16_t bitOffset, int8_t v);
    uint8_t uint8_1x(uint16_t bitOffset) const;  
    void setUInt8_1x(uint16_t bitOffset, uint8_t v);
    int16_t int16_1x(uint16_t bitOffset) const; 
    void setInt16_1x(uint16_t bitOffset, int16_t v);
    uint16_t uint16_1x(uint16_t bitOffset) const;  
    void setUInt16_1x(uint16_t bitOffset, uint16_t v);
    int32_t int32_1x(uint16_t bitOffset) const; 
    void setInt32_1x(uint16_t bitOffset, int32_t v);
    uint32_t uint32_1x(uint16_t bitOffset) const;  
    void setUInt32_1x(uint16_t bitOffset, uint32_t v);
    int int_1x(uint16_t bitOffset) const; 
    void setInt_1x(uint16_t bitOffset, int v);
    unsigned int uint_1x(uint16_t bitOffset) const; 
    void setUInt_1x(uint16_t bitOffset, unsigned int v); 
    float float_1x(uint16_t bitOffset) const;
    void setFloat_1x(uint16_t bitOffset, float v);
    double double_1x(uint16_t bitOffset) const;
    void setDouble_1x(uint16_t bitOffset, double v);
#endif // MODBUS_MEMORY_COUNT_1x > 0   
        
#if MODBUS_MEMORY_COUNT_3x > 0   
public: // memory-3x management functions
    inline void zerroAll_3x() { memset(m_mem3x, 0, MODBUS_MEMORY_SZ_3x_BYTES); }
    Modbus::Response read_3x(uint16_t offset, uint16_t count, uint16_t* values, uint16_t* fact = MB_NULLPTR) const;
    Modbus::Response write_3x(uint16_t offset, uint16_t count, const uint16_t* values, uint16_t* fact = MB_NULLPTR);
    bool bool_3x(uint32_t bitOffset) const;
    void setBool_3x(uint32_t bitOffset, bool v); 
    int8_t int8_3x(uint32_t byteOffset) const; 
    void setInt8_3x(uint32_t byteOffset, int8_t v);
    uint8_t uint8_3x(uint32_t byteOffset) const;  
    void setUInt8_3x(uint32_t byteOffset, uint8_t v);       
    int16_t int16_3x(uint16_t regOffset) const; 
    void setInt16_3x(uint16_t regOffset, int16_t v);
    uint16_t uint16_3x(uint16_t regOffset) const;  
    void setUInt16_3x(uint16_t regOffset, uint16_t v);
    int32_t int32_3x(uint16_t regOffset) const; 
    void setInt32_3x(uint16_t regOffset, int32_t v);
    uint32_t uint32_3x(uint16_t regOffset) const;  
    void setUInt32_3x(uint16_t regOffset, uint32_t v);
    int int_3x(uint16_t regOffset) const; 
    void setInt_3x(uint16_t regOffset, int v);
    unsigned int uint_3x(uint16_t bitOffset) const; 
    void setUInt_3x(uint16_t bitOffset, unsigned int v); 
    float float_3x(uint16_t regOffset) const;
    void setFloat_3x(uint16_t regOffset, float v);
    double double_3x(uint16_t regOffset) const;
    void setDouble_3x(uint16_t regOffset, double v);
#endif // MODBUS_MEMORY_COUNT_3x > 0   
        
#if MODBUS_MEMORY_COUNT_4x > 0   
public: // memory-4x management functions
    inline void zerroAll_4x() { memset(m_mem4x, 0, MODBUS_MEMORY_SZ_4x_BYTES); }
    Modbus::Response read_4x(uint16_t offset, uint16_t count, uint16_t* values, uint16_t* fact = MB_NULLPTR) const;
    Modbus::Response write_4x(uint16_t offset, uint16_t count, const uint16_t* values, uint16_t* fact = MB_NULLPTR);    
    bool bool_4x(uint32_t bitOffset) const;
    void setBool_4x(uint32_t bitOffset, bool v); 
    int8_t int8_4x(uint32_t byteOffset) const; 
    void setInt8_4x(uint32_t byteOffset, int8_t v);
    uint8_t uint8_4x(uint32_t byteOffset) const;  
    void setUInt8_4x(uint32_t byteOffset, uint8_t v);       
    int16_t int16_4x(uint16_t regOffset) const; 
    void setInt16_4x(uint16_t regOffset, int16_t v);
    uint16_t uint16_4x(uint16_t regOffset) const;  
    void setUInt16_4x(uint16_t regOffset, uint16_t v);
    int32_t int32_4x(uint16_t regOffset) const; 
    void setInt32_4x(uint16_t regOffset, int32_t v);
    uint32_t uint32_4x(uint16_t regOffset) const;  
    void setUInt32_4x(uint16_t regOffset, uint32_t v);
    int int_4x(uint16_t regOffset) const; 
    void setInt_4x(uint16_t regOffset, int v);
    unsigned int uint_4x(uint16_t bitOffset) const; 
    void setUInt_4x(uint16_t bitOffset, unsigned int v); 
    float float_4x(uint16_t regOffset) const;
    void setFloat_4x(uint16_t regOffset, float v);
    double double_4x(uint16_t regOffset) const;
    void setDouble_4x(uint16_t regOffset, double v);
#endif // MODBUS_MEMORY_COUNT_4x > 0   
    
private:   
#if MODBUS_MEMORY_COUNT_0x > 0   
    uint8_t m_mem0x[MODBUS_MEMORY_SZ_0x_BYTES];
#endif // MODBUS_MEMORY_COUNT_0x > 0  
 
#if MODBUS_MEMORY_COUNT_1x > 0   
    uint8_t m_mem1x[MODBUS_MEMORY_SZ_1x_BYTES];
#endif // MODBUS_MEMORY_COUNT_1x > 0  
 
#if MODBUS_MEMORY_COUNT_3x > 0   
    uint16_t m_mem3x[MODBUS_MEMORY_SZ_3x_REGES];
#endif // MODBUS_MEMORY_COUNT_3x > 0  
 
#if MODBUS_MEMORY_COUNT_4x > 0      
    uint16_t m_mem4x[MODBUS_MEMORY_SZ_4x_REGES];
#endif // MODBUS_MEMORY_COUNT_4x > 0  
 
};

static Modbus::Response read_bits(uint16_t bitOffset, const void* mem, size_t sz_bits, void* bits, uint16_t bitCount, uint16_t* fact = MB_NULLPTR)
{
    uint16_t c;
    if (bitOffset >= sz_bits)
        return Modbus::ILLEGAL_DATA_ADDRESS;
     
    if ((bitOffset+bitCount) > sz_bits)
        c = sz_bits - bitOffset;
    else
        c = bitCount;
     
    uint16_t byteOffset = bitOffset/MODBUS_BYTE_SZ_BITES;
    uint16_t bytes = c/MODBUS_BYTE_SZ_BITES;
    uint16_t shift = bitOffset%MODBUS_BYTE_SZ_BITES;
    if (shift)
    {
        for (uint16_t i = 0; i < bytes; i++)
        {
            uint16_t v = *(reinterpret_cast<const uint16_t*>(&reinterpret_cast<const uint8_t*>(mem)[byteOffset+i])) >> shift; 
            reinterpret_cast<uint8_t*>(bits)[i] = static_cast<uint8_t>(v);    
        }
        if (uint16_t resid = c%MODBUS_BYTE_SZ_BITES)
        {
            int8_t mask = 0x80;
            mask = ~(mask>>(7-resid));
            if ((shift+resid) > MODBUS_BYTE_SZ_BITES)
            {
                uint16_t v = ((*reinterpret_cast<const uint16_t*>(&reinterpret_cast<const uint8_t*>(mem)[byteOffset+bytes])) >> shift) & mask;
                reinterpret_cast<uint8_t*>(bits)[bytes] = static_cast<uint8_t>(v);
            }
            else
                reinterpret_cast<uint8_t*>(bits)[bytes] = (reinterpret_cast<const uint8_t*>(mem)[byteOffset+bytes]>>shift) & mask;
        }
    }
    else
    {
        memcpy(bits, &reinterpret_cast<const uint8_t*>(mem)[byteOffset], bytes);
        if (uint16_t resid = c%MODBUS_BYTE_SZ_BITES)
        {
            int8_t mask = 0x80;
            mask = ~(mask>>(7-resid));
            reinterpret_cast<uint8_t*>(bits)[bytes] = reinterpret_cast<const uint8_t*>(mem)[byteOffset+bytes] & mask;
        }
    }
    if (fact)
        *fact = c;
    return Modbus::OK;  
}

static Modbus::Response write_bits(uint16_t bitOffset, void* mem, size_t sz_bits, const void* bits, uint16_t bitCount, uint16_t* fact = MB_NULLPTR)
{
    uint16_t c;
    if (bitOffset >= sz_bits)
        return Modbus::ILLEGAL_DATA_ADDRESS;
     
    if ((bitOffset+bitCount) > sz_bits)
        c = sz_bits - bitOffset;
    else
        c = bitCount;
    uint16_t byteOffset = bitOffset/MODBUS_BYTE_SZ_BITES;    
    uint16_t bytes = c/MODBUS_BYTE_SZ_BITES;    
    uint16_t shift = bitOffset%MODBUS_BYTE_SZ_BITES;    
    if (shift)
    {
        for (uint16_t i = 0; i < bytes; i++)
        {
            uint16_t mask = 0x00FF << shift;
            uint16_t v = static_cast<uint16_t>(reinterpret_cast<const uint8_t*>(bits)[i]) << shift;
            *reinterpret_cast<uint16_t*>(&static_cast<uint8_t*>(mem)[byteOffset+i]) &= ~mask; // zero undermask bits
            *reinterpret_cast<uint16_t*>(&static_cast<uint8_t*>(mem)[byteOffset+i]) |= v; // set bit values  
        }
        if (uint16_t resid = c%MODBUS_BYTE_SZ_BITES)
        {
            if ((shift+resid) > MODBUS_BYTE_SZ_BITES)
            {
                int16_t m = 0x8000; // using signed mask for right shift filled by '1'-bit
                m = m>>(resid-1);
                uint16_t mask = *reinterpret_cast<uint16_t*>(&m);
                mask = mask >> (MODBUS_REGE_SZ_BITES-resid-shift);
                uint16_t v = (static_cast<uint16_t>(reinterpret_cast<const uint8_t*>(bits)[bytes]) << shift) & mask;
                *reinterpret_cast<uint16_t*>(&reinterpret_cast<uint8_t*>(mem)[byteOffset+bytes]) &= ~mask; // zero undermask bits
                *reinterpret_cast<uint16_t*>(&reinterpret_cast<uint8_t*>(mem)[byteOffset+bytes]) |= v;
            }
            else
            {
                int8_t m = 0x80; // using signed mask for right shift filled by '1'-bit
                m = m>>(resid-1);
                uint8_t mask = *reinterpret_cast<uint8_t*>(&m);
                mask = mask >> (MODBUS_BYTE_SZ_BITES-resid-shift);
                uint8_t v = (reinterpret_cast<const uint8_t*>(bits)[bytes] << shift) & mask;
                reinterpret_cast<uint8_t*>(mem)[byteOffset+bytes] &= ~mask; // zero undermask bits
                reinterpret_cast<uint8_t*>(mem)[byteOffset+bytes] |= v;
            }
        }
    }
    else
    {
        memcpy(&reinterpret_cast<uint8_t*>(mem)[byteOffset], bits, bytes);
        if (uint16_t resid = c%MODBUS_BYTE_SZ_BITES)
        {
            int8_t mask = 0x80;
            mask = mask>>(7-resid);
            reinterpret_cast<uint8_t*>(mem)[byteOffset+bytes] &= mask;
            mask = ~mask;
            reinterpret_cast<uint8_t*>(mem)[byteOffset+bytes] |= (reinterpret_cast<const uint8_t*>(bits)[bytes] & mask);
        }
    }
    if (fact)
        *fact = c;
    return Modbus::OK; 
}

static Modbus::Response copy_bits(uint16_t bitOffsetDest, void* dest, size_t sz_bits_dest, uint16_t bitOffsetSrc, const void* src, size_t sz_bits_src, uint16_t bitCount, uint16_t* fact = MB_NULLPTR)
{
    const uint16_t sz_buff_bytes = 32;
    const uint16_t sz_buff_bites = sz_buff_bytes*MODBUS_BYTE_SZ_BITES;
    uint16_t c;
    
    c = bitCount;
    if (bitOffsetDest >= sz_bits_dest)
        return Modbus::ILLEGAL_DATA_ADDRESS;
     
    if ((bitOffsetDest+c) > sz_bits_dest)
        c = sz_bits_dest - bitOffsetDest;

    if (bitOffsetSrc >= sz_bits_src)
        return Modbus::ILLEGAL_DATA_ADDRESS;
     
    if ((bitOffsetSrc+c) > sz_bits_src)
        c = sz_bits_src - bitOffsetSrc;

    uint16_t f = c;
    while (c)
    {
        uint8_t buff[sz_buff_bytes];
        uint16_t cc = (c > sz_buff_bites) ? sz_buff_bites : c;
        read_bits(bitOffsetSrc, src, sz_bits_src, buff, cc);
        write_bits(bitOffsetDest, dest, sz_bits_dest, buff, cc);
        bitOffsetDest += cc;
        bitOffsetSrc += cc;
        c -= cc;
    }
    if (fact)
        *fact = f;
    return Modbus::OK;
}

ModbusMemory::ModbusMemory()
{
}

// --------------------------------------------------------------------------------------------------------
// ---------------------------------------- MODBUS MASTER INTERFACE ---------------------------------------
// --------------------------------------------------------------------------------------------------------

Modbus::Response ModbusMemory::readCoilStatus(uint8_t &/*slave*/, uint16_t offset, uint16_t count, void* values, uint16_t* fact)
{
#if MODBUS_MEMORY_COUNT_0x > 0
    return read_0x(offset, count, values, fact);
#else
    return Modbus::ILLEGAL_FUNCTION;
#endif
}
 
Modbus::Response ModbusMemory::readInputStatus(uint8_t &/*slave*/, uint16_t offset, uint16_t count, void* values, uint16_t* fact)
{
#if MODBUS_MEMORY_COUNT_1x > 0
    return read_1x(offset, count, values, fact);
#else
    return Modbus::ILLEGAL_FUNCTION;
#endif
}
 
Modbus::Response ModbusMemory::readHoldingRegisters(uint8_t &/*slave*/, uint16_t offset, uint16_t count, uint16_t* values, uint16_t* fact)
{
#if MODBUS_MEMORY_COUNT_4x > 0
    return read_4x(offset, count, values, fact);
#else
    return Modbus::ILLEGAL_FUNCTION;
#endif
}
 
Modbus::Response ModbusMemory::readInputRegisters(uint8_t &/*slave*/, uint16_t offset, uint16_t count, uint16_t* values, uint16_t* fact)
{
#if MODBUS_MEMORY_COUNT_3x > 0
    return read_3x(offset, count, values, fact);
#else
    return Modbus::ILLEGAL_FUNCTION;
#endif
}
 
Modbus::Response ModbusMemory::forceSingleCoil(uint8_t &/*slave*/, uint16_t offset, bool value)
{
 #if MODBUS_MEMORY_COUNT_0x > 0
    return write_0x(offset, 1, &value);
#else
    return Modbus::ILLEGAL_FUNCTION;
#endif
}
 
Modbus::Response ModbusMemory::forceSingleRegister(uint8_t &/*slave*/, uint16_t offset, uint16_t value)
{
#if MODBUS_MEMORY_COUNT_4x > 0
    return write_4x(offset, 1, &value);
#else
    return Modbus::ILLEGAL_FUNCTION;
#endif
}
 
Modbus::Response ModbusMemory::forceMultipleCoils(uint8_t &/*slave*/, uint16_t offset, uint16_t count, const void* values, uint16_t* fact)
{
#if MODBUS_MEMORY_COUNT_0x > 0
    return write_0x(offset, count, values, fact);
#else
    return Modbus::ILLEGAL_FUNCTION;
#endif
}
 
Modbus::Response ModbusMemory::forceMultipleRegisters(uint8_t &/*slave*/, uint16_t offset, uint16_t count, const uint16_t* values, uint16_t* fact)
{
#if MODBUS_MEMORY_COUNT_4x > 0
    return write_4x(offset, count, values, fact);
#else
    return Modbus::ILLEGAL_FUNCTION;
#endif
}

Modbus::Response ModbusMemory::copy(Modbus::Address typeFrom, uint16_t offsetFrom, uint16_t count, Modbus::Address typeTo, uint16_t offsetTo, uint16_t* fact)
{
    uint16_t c = count;
    switch (typeFrom)
    {
    case Modbus::X0:
#if MODBUS_MEMORY_COUNT_0x > 0
        switch (typeTo)
        {
        case Modbus::X0:
            return copy_bits(offsetTo, m_mem0x, MODBUS_MEMORY_SZ_0x_BITES, offsetFrom, m_mem0x, MODBUS_MEMORY_SZ_0x_BITES, c, fact);
        case Modbus::X1:
#if MODBUS_MEMORY_COUNT_1x > 0
            return copy_bits(offsetTo, m_mem1x, MODBUS_MEMORY_SZ_1x_BITES, offsetFrom, m_mem0x, MODBUS_MEMORY_SZ_0x_BITES, c, fact);
#else  // MODBUS_MEMORY_COUNT_1x > 0
            return Modbus::ILLEGAL_DATA_ADDRESS;
#endif // MODBUS_MEMORY_COUNT_1x > 0
        case Modbus::X3:
#if MODBUS_MEMORY_COUNT_3x > 0
            if (offsetTo >= MODBUS_MEMORY_SZ_3x_REGES)
                return Modbus::ILLEGAL_DATA_ADDRESS;
            if ((c/MODBUS_REGE_SZ_BITES+(c%MODBUS_REGE_SZ_BITES!=0)) > (MODBUS_MEMORY_SZ_3x_REGES-offsetTo))
                c = (MODBUS_MEMORY_SZ_3x_REGES-offsetTo)*MODBUS_REGE_SZ_BITES;
            return read_bits(offsetFrom, m_mem0x, MODBUS_MEMORY_SZ_0x_BITES, &m_mem3x[offsetTo], c, fact);
#else  // MODBUS_MEMORY_COUNT_3x > 0
            return Modbus::ILLEGAL_DATA_ADDRESS;
#endif // MODBUS_MEMORY_COUNT_3x > 0
        case Modbus::X4:
#if MODBUS_MEMORY_COUNT_4x > 0
            if (offsetTo >= MODBUS_MEMORY_SZ_4x_REGES)
                return Modbus::ILLEGAL_DATA_ADDRESS;
            if ((c/MODBUS_REGE_SZ_BITES+(c%MODBUS_REGE_SZ_BITES!=0)) > (MODBUS_MEMORY_SZ_4x_REGES-offsetTo))
                c = (MODBUS_MEMORY_SZ_4x_REGES-offsetTo)*MODBUS_REGE_SZ_BITES;
            return read_bits(offsetFrom, m_mem0x, MODBUS_MEMORY_SZ_0x_BITES, &m_mem4x[offsetTo], c, fact);
#else  // MODBUS_MEMORY_COUNT_4x > 0
            return Modbus::ILLEGAL_DATA_ADDRESS;
#endif // MODBUS_MEMORY_COUNT_4x > 0
        }
        break;
#else  // MODBUS_MEMORY_COUNT_0x > 0
        return Modbus::ILLEGAL_DATA_ADDRESS;
#endif // MODBUS_MEMORY_COUNT_0x > 0
    case Modbus::X1:
#if MODBUS_MEMORY_COUNT_1x > 0
        switch (typeTo)
        {
        case Modbus::X0:
#if MODBUS_MEMORY_COUNT_0x > 0
            return copy_bits(offsetTo, m_mem0x, MODBUS_MEMORY_SZ_0x_BITES, offsetFrom, m_mem1x, MODBUS_MEMORY_SZ_1x_BITES, c, fact);
#else  // MODBUS_MEMORY_COUNT_0x > 0
            return Modbus::ILLEGAL_DATA_ADDRESS;
#endif // MODBUS_MEMORY_COUNT_0x > 0
        case Modbus::X1:
            return copy_bits(offsetTo, m_mem1x, MODBUS_MEMORY_SZ_1x_BITES, offsetFrom, m_mem1x, MODBUS_MEMORY_SZ_1x_BITES, c, fact);
        case Modbus::X3:
#if MODBUS_MEMORY_COUNT_3x > 0
            if (offsetTo >= MODBUS_MEMORY_SZ_3x_REGES)
                return Modbus::ILLEGAL_DATA_ADDRESS;
            if ((c/MODBUS_REGE_SZ_BITES+(c%MODBUS_REGE_SZ_BITES!=0)) > (MODBUS_MEMORY_SZ_3x_REGES-offsetTo))
                c = (MODBUS_MEMORY_SZ_3x_REGES-offsetTo)*MODBUS_REGE_SZ_BITES;
            return read_bits(offsetFrom, m_mem1x, MODBUS_MEMORY_SZ_1x_BITES, &m_mem3x[offsetTo], c, fact);
#else  // MODBUS_MEMORY_COUNT_3x > 0
            return Modbus::ILLEGAL_DATA_ADDRESS;
#endif // MODBUS_MEMORY_COUNT_3x > 0
        case Modbus::X4:
#if MODBUS_MEMORY_COUNT_4x > 0
            if (offsetTo >= MODBUS_MEMORY_SZ_4x_REGES)
                return Modbus::ILLEGAL_DATA_ADDRESS;
            if ((c/MODBUS_REGE_SZ_BITES+(c%MODBUS_REGE_SZ_BITES!=0)) > (MODBUS_MEMORY_SZ_4x_REGES-offsetTo))
                c = (MODBUS_MEMORY_SZ_4x_REGES-offsetTo)*MODBUS_REGE_SZ_BITES;
            return read_bits(offsetFrom, m_mem1x, MODBUS_MEMORY_SZ_1x_BITES, &m_mem4x[offsetTo], c, fact);
#else  // MODBUS_MEMORY_COUNT_4x > 0
            return Modbus::ILLEGAL_DATA_ADDRESS;
#endif // MODBUS_MEMORY_COUNT_4x > 0
        }
        break;
#else  // MODBUS_MEMORY_COUNT_1x > 0
        return Modbus::ILLEGAL_DATA_ADDRESS;
#endif // MODBUS_MEMORY_COUNT_1x > 0
    case Modbus::X3:
#if MODBUS_MEMORY_COUNT_3x > 0
        switch (typeTo)
        {
        case Modbus::X0:
#if MODBUS_MEMORY_COUNT_0x > 0
            if (offsetFrom >= MODBUS_MEMORY_SZ_3x_REGES)
                return Modbus::ILLEGAL_DATA_ADDRESS;
            if ((c/MODBUS_REGE_SZ_BITES+(c%MODBUS_REGE_SZ_BITES!=0)) > (MODBUS_MEMORY_SZ_3x_REGES-offsetFrom))
                c = (MODBUS_MEMORY_SZ_3x_REGES-offsetTo)*MODBUS_REGE_SZ_BITES;
            return write_bits(offsetTo, m_mem0x, MODBUS_MEMORY_SZ_0x_BITES, &m_mem3x[offsetFrom], c, fact);
#else  // MODBUS_MEMORY_COUNT_0x > 0
            return Modbus::ILLEGAL_DATA_ADDRESS;
#endif // MODBUS_MEMORY_COUNT_0x > 0
        case Modbus::X1:
#if MODBUS_MEMORY_COUNT_1x > 0
            if (offsetFrom >= MODBUS_MEMORY_SZ_3x_REGES)
                return Modbus::ILLEGAL_DATA_ADDRESS;
            if ((c/MODBUS_REGE_SZ_BITES+(c%MODBUS_REGE_SZ_BITES!=0)) > (MODBUS_MEMORY_SZ_3x_REGES-offsetFrom))
                c = (MODBUS_MEMORY_SZ_3x_REGES-offsetTo)*MODBUS_REGE_SZ_BITES;
            return write_bits(offsetTo, m_mem1x, MODBUS_MEMORY_SZ_1x_BITES, &m_mem3x[offsetFrom], c, fact);
#else  // MODBUS_MEMORY_COUNT_1x > 0
            return Modbus::ILLEGAL_DATA_ADDRESS;
#endif // MODBUS_MEMORY_COUNT_1x > 0
        case Modbus::X3:
            if (offsetFrom >= MODBUS_MEMORY_SZ_3x_REGES)
                return Modbus::ILLEGAL_DATA_ADDRESS;             
            if ((offsetFrom+c) > MODBUS_MEMORY_SZ_3x_REGES)
                c = MODBUS_MEMORY_SZ_3x_REGES - offsetFrom;
            if (offsetTo >= MODBUS_MEMORY_SZ_3x_REGES)
                return Modbus::ILLEGAL_DATA_ADDRESS;             
            if ((offsetTo+c) > MODBUS_MEMORY_SZ_3x_REGES)
                c = MODBUS_MEMORY_SZ_3x_REGES - offsetTo;
            memmove(&m_mem3x[offsetTo], &m_mem3x[offsetFrom], c*MODBUS_REGE_SZ_BYTES);
            return Modbus::OK;
        case Modbus::X4:
#if MODBUS_MEMORY_COUNT_4x > 0
            if (offsetFrom >= MODBUS_MEMORY_SZ_3x_REGES)
                return Modbus::ILLEGAL_DATA_ADDRESS;             
            if ((offsetFrom+c) > MODBUS_MEMORY_SZ_3x_REGES)
                c = MODBUS_MEMORY_SZ_3x_REGES - offsetFrom;
            if (offsetTo >= MODBUS_MEMORY_SZ_4x_REGES)
                return Modbus::ILLEGAL_DATA_ADDRESS;             
            if ((offsetTo+c) > MODBUS_MEMORY_SZ_4x_REGES)
                c = MODBUS_MEMORY_SZ_4x_REGES - offsetTo;
            memcpy(&m_mem4x[offsetTo], &m_mem3x[offsetFrom], c*MODBUS_REGE_SZ_BYTES);
            return Modbus::OK;
#else  // MODBUS_MEMORY_COUNT_4x > 0
            return Modbus::ILLEGAL_DATA_ADDRESS;
#endif // MODBUS_MEMORY_COUNT_4x > 0
        }
        break;
#else  // MODBUS_MEMORY_COUNT_3x > 0
        return Modbus::ILLEGAL_DATA_ADDRESS;
#endif // MODBUS_MEMORY_COUNT_3x > 0
    case Modbus::X4:
#if MODBUS_MEMORY_COUNT_4x > 0
        switch (typeTo)
        {
        case Modbus::X0:
#if MODBUS_MEMORY_COUNT_0x > 0
            if (offsetFrom >= MODBUS_MEMORY_SZ_4x_REGES)
                return Modbus::ILLEGAL_DATA_ADDRESS;
            if ((c/MODBUS_REGE_SZ_BITES+(c%MODBUS_REGE_SZ_BITES!=0)) > (MODBUS_MEMORY_SZ_4x_REGES-offsetFrom))
                c = (MODBUS_MEMORY_SZ_4x_REGES-offsetTo)*MODBUS_REGE_SZ_BITES;
            return write_bits(offsetTo, m_mem0x, MODBUS_MEMORY_SZ_0x_BITES, &m_mem4x[offsetFrom], c, fact);
#else  // MODBUS_MEMORY_COUNT_0x > 0
            return Modbus::ILLEGAL_DATA_ADDRESS;
#endif // MODBUS_MEMORY_COUNT_0x > 0
        case Modbus::X1:
#if MODBUS_MEMORY_COUNT_1x > 0
            if (offsetFrom >= MODBUS_MEMORY_SZ_4x_REGES)
                return Modbus::ILLEGAL_DATA_ADDRESS;
            if ((c/MODBUS_REGE_SZ_BITES+(c%MODBUS_REGE_SZ_BITES!=0)) > (MODBUS_MEMORY_SZ_4x_REGES-offsetFrom))
                c = (MODBUS_MEMORY_SZ_4x_REGES-offsetTo)*MODBUS_REGE_SZ_BITES;
            return write_bits(offsetTo, m_mem1x, MODBUS_MEMORY_SZ_1x_BITES, &m_mem4x[offsetFrom], c, fact);
#else  // MODBUS_MEMORY_COUNT_1x > 0
            return Modbus::ILLEGAL_DATA_ADDRESS;
#endif // MODBUS_MEMORY_COUNT_1x > 0
        case Modbus::X3:
#if MODBUS_MEMORY_COUNT_3x > 0
            if (offsetFrom >= MODBUS_MEMORY_SZ_4x_REGES)
                return Modbus::ILLEGAL_DATA_ADDRESS;             
            if ((offsetFrom+c) > MODBUS_MEMORY_SZ_4x_REGES)
                c = MODBUS_MEMORY_SZ_4x_REGES - offsetFrom;
            if (offsetTo >= MODBUS_MEMORY_SZ_3x_REGES)
                return Modbus::ILLEGAL_DATA_ADDRESS;             
            if ((offsetTo+c) > MODBUS_MEMORY_SZ_3x_REGES)
                c = MODBUS_MEMORY_SZ_3x_REGES - offsetTo;
            memcpy(&m_mem3x[offsetTo], &m_mem4x[offsetFrom], c*MODBUS_REGE_SZ_BYTES);
            return Modbus::OK;
#else  // MODBUS_MEMORY_COUNT_3x > 0
            return Modbus::ILLEGAL_DATA_ADDRESS;
#endif // MODBUS_MEMORY_COUNT_3x > 0
        case Modbus::X4:
            if (offsetFrom >= MODBUS_MEMORY_SZ_4x_REGES)
                return Modbus::ILLEGAL_DATA_ADDRESS;             
            if ((offsetFrom+c) > MODBUS_MEMORY_SZ_4x_REGES)
                c = MODBUS_MEMORY_SZ_4x_REGES - offsetFrom;
            if (offsetTo >= MODBUS_MEMORY_SZ_4x_REGES)
                return Modbus::ILLEGAL_DATA_ADDRESS;             
            if ((offsetTo+c) > MODBUS_MEMORY_SZ_4x_REGES)
                c = MODBUS_MEMORY_SZ_4x_REGES - offsetTo;
            memmove(&m_mem4x[offsetTo], &m_mem4x[offsetFrom], c*MODBUS_REGE_SZ_BYTES);
            return Modbus::OK;
        }
        break;
#else  // MODBUS_MEMORY_COUNT_4x > 0
        return Modbus::ILLEGAL_DATA_ADDRESS;
#endif // MODBUS_MEMORY_COUNT_4x > 0
    }
    return Modbus::ILLEGAL_DATA_ADDRESS;
}

// --------------------------------------------------------------------------------------------------------
// ------------------------------------ MODBUS MEMORY PRINT FUNCTIONS -------------------------------------
// --------------------------------------------------------------------------------------------------------

#if MODBUS_MEMORY_COUNT_0x > 0
void ModbusMemory::print_0x(Stream& serial, uint16_t offset, uint16_t count, int /*number*/)
{
    uint16_t last = offset+count;
    if (last > MODBUS_MEMORY_COUNT_0x)
        last = MODBUS_MEMORY_COUNT_0x;
    for (uint16_t i = offset; i < last; i++)
    {
      
        Serial.print((m_mem0x[i/8] & (1<<i%8))!=0);
        Serial.print(' ');
    }
    Serial.println(); 
}
#endif  // MODBUS_MEMORY_COUNT_0x > 0

#if MODBUS_MEMORY_COUNT_1x > 0
void ModbusMemory::print_1x(Stream& serial, uint16_t offset, uint16_t count, int /*number*/)
{
    uint16_t last = offset+count;
    if (last > MODBUS_MEMORY_COUNT_1x)
        last = MODBUS_MEMORY_COUNT_1x;
    for (uint16_t i = offset; i < last; i++)
    {
        Serial.print((m_mem1x[i/8] & (1<<i%8))!=0);
        Serial.print(' ');
    }
    Serial.println(); 
}
#endif  // MODBUS_MEMORY_COUNT_1x > 0

#if MODBUS_MEMORY_COUNT_3x > 0
void ModbusMemory::print_3x(Stream& serial, uint16_t offset, uint16_t count, int number)
{
    uint16_t last = offset+count;
    if (last > MODBUS_MEMORY_COUNT_3x)
        last = MODBUS_MEMORY_COUNT_3x;
    for (uint16_t i = offset; i < last; i++)
    {
        Serial.print(m_mem3x[i], number);
        Serial.print(' ');
    }
    Serial.println();
}

#endif  // MODBUS_MEMORY_COUNT_3x > 0

#if MODBUS_MEMORY_COUNT_4x > 0
void ModbusMemory::print_4x(Stream& serial, uint16_t offset, uint16_t count, int number)
{
    uint16_t last = offset+count;
    if (last > MODBUS_MEMORY_COUNT_3x)
        last = MODBUS_MEMORY_COUNT_3x;
    for (uint16_t i = offset; i < last; i++)
    {
        Serial.print(m_mem4x[i], number);
        Serial.print(' ');
    }
    Serial.println(); 
}
#endif  // MODBUS_MEMORY_COUNT_4x > 0

// --------------------------------------------------------------------------------------------------------
// --------------------------------------- MODBUS MEMORY 0x (COILS) ---------------------------------------
// --------------------------------------------------------------------------------------------------------

#if MODBUS_MEMORY_COUNT_0x > 0

Modbus::Response ModbusMemory::read_0x(uint16_t bitOffset, uint16_t bitCount, void* bits, uint16_t* fact) const
{
    return read_bits(bitOffset, m_mem0x, MODBUS_MEMORY_COUNT_0x, bits, bitCount, fact);
}

Modbus::Response ModbusMemory::write_0x(uint16_t bitOffset, uint16_t bitCount, const void* bits, uint16_t* fact)
{
    return write_bits(bitOffset, m_mem0x, MODBUS_MEMORY_COUNT_0x, bits, bitCount, fact);
}

bool ModbusMemory::bool_0x(uint16_t bitOffset) const
{
    if (bitOffset < MODBUS_MEMORY_COUNT_0x)
        return GET_BIT(m_mem0x, bitOffset);
    return false;
}

void ModbusMemory::setBool_0x(uint16_t bitOffset, bool v)
{
    if (bitOffset < MODBUS_MEMORY_COUNT_0x)
        SET_BIT(m_mem0x, bitOffset, v);
}

int8_t ModbusMemory::int8_0x(uint16_t bitOffset) const
{
    int8_t v = 0;
    read_bits(bitOffset, m_mem0x, MODBUS_MEMORY_COUNT_0x, &v, sizeof(v)*MODBUS_BYTE_SZ_BITES);
    return v;
}
 
void ModbusMemory::setInt8_0x(uint16_t bitOffset, int8_t v)
{
    write_bits(bitOffset, m_mem0x, MODBUS_MEMORY_COUNT_0x, &v, sizeof(v)*MODBUS_BYTE_SZ_BITES);
}

uint8_t ModbusMemory::uint8_0x(uint16_t bitOffset) const
{
    uint8_t v = 0;
    read_bits(bitOffset, m_mem0x, MODBUS_MEMORY_COUNT_0x, &v, sizeof(v)*MODBUS_BYTE_SZ_BITES);
    return v;
}

void ModbusMemory::setUInt8_0x(uint16_t bitOffset, uint8_t v)
{
    write_bits(bitOffset, m_mem0x, MODBUS_MEMORY_COUNT_0x, &v, sizeof(v)*MODBUS_BYTE_SZ_BITES);
}

int16_t ModbusMemory::int16_0x(uint16_t bitOffset) const
{
    int16_t v = 0;
    read_bits(bitOffset, m_mem0x, MODBUS_MEMORY_COUNT_0x, &v, sizeof(v)*MODBUS_BYTE_SZ_BITES);
    return v;
}

void ModbusMemory::setInt16_0x(uint16_t bitOffset, int16_t v)
{
    write_bits(bitOffset, m_mem0x, MODBUS_MEMORY_COUNT_0x, &v, sizeof(v)*MODBUS_BYTE_SZ_BITES);
}

uint16_t ModbusMemory::uint16_0x(uint16_t bitOffset) const
{
    uint16_t v = 0;
    read_bits(bitOffset, m_mem0x, MODBUS_MEMORY_COUNT_0x, &v, sizeof(v)*MODBUS_BYTE_SZ_BITES);
    return v;
}

void ModbusMemory::setUInt16_0x(uint16_t bitOffset, uint16_t v)
{
    write_bits(bitOffset, m_mem0x, MODBUS_MEMORY_COUNT_0x, &v, sizeof(v)*MODBUS_BYTE_SZ_BITES);
}

int32_t ModbusMemory::int32_0x(uint16_t bitOffset) const
{
    int32_t v = 0;
    read_bits(bitOffset, m_mem0x, MODBUS_MEMORY_COUNT_0x, &v, sizeof(v)*MODBUS_BYTE_SZ_BITES);
    return v;
}

void ModbusMemory::setInt32_0x(uint16_t bitOffset, int32_t v)
{
    write_bits(bitOffset, m_mem0x, MODBUS_MEMORY_COUNT_0x, &v, sizeof(v)*MODBUS_BYTE_SZ_BITES);
}

uint32_t ModbusMemory::uint32_0x(uint16_t bitOffset) const
{
    uint32_t v = 0;
    read_bits(bitOffset, m_mem0x, MODBUS_MEMORY_COUNT_0x, &v, sizeof(v)*MODBUS_BYTE_SZ_BITES);
    return v;
}

void ModbusMemory::setUInt32_0x(uint16_t bitOffset, uint32_t v)
{
    write_bits(bitOffset, m_mem0x, MODBUS_MEMORY_COUNT_0x, &v, sizeof(v)*MODBUS_BYTE_SZ_BITES);
}

int ModbusMemory::int_0x(uint16_t bitOffset) const
{
    int v = 0;
    read_bits(bitOffset, m_mem0x, MODBUS_MEMORY_COUNT_0x, &v, sizeof(v)*MODBUS_BYTE_SZ_BITES);
    return v;
}

void ModbusMemory::setInt_0x(uint16_t bitOffset, int v)
{
    write_bits(bitOffset, m_mem0x, MODBUS_MEMORY_COUNT_0x, &v, sizeof(v)*MODBUS_BYTE_SZ_BITES);
}

unsigned int ModbusMemory::uint_0x(uint16_t bitOffset) const
{
    unsigned int v = 0;
    read_bits(bitOffset, m_mem0x, MODBUS_MEMORY_COUNT_0x, &v, sizeof(v)*MODBUS_BYTE_SZ_BITES);
    return v;
}

void ModbusMemory::setUInt_0x(uint16_t bitOffset, unsigned int v)
{
    write_bits(bitOffset, m_mem0x, MODBUS_MEMORY_COUNT_0x, &v, sizeof(v)*MODBUS_BYTE_SZ_BITES);
}

float ModbusMemory::float_0x(uint16_t bitOffset) const
{
    float v = 0.0f;
    read_bits(bitOffset, m_mem0x, MODBUS_MEMORY_COUNT_0x, &v, sizeof(v)*MODBUS_BYTE_SZ_BITES);
    return v;
}

void ModbusMemory::setFloat_0x(uint16_t bitOffset, float v)
{
    write_bits(bitOffset, m_mem0x, MODBUS_MEMORY_COUNT_0x, &v, sizeof(v)*MODBUS_BYTE_SZ_BITES);
}

double ModbusMemory::double_0x(uint16_t bitOffset) const
{
    double v = 0.0;
    read_bits(bitOffset, m_mem0x, MODBUS_MEMORY_COUNT_0x, &v, sizeof(v)*MODBUS_BYTE_SZ_BITES);
    return v;
}

void ModbusMemory::setDouble_0x(uint16_t bitOffset, double v)
{
    write_bits(bitOffset, m_mem0x, MODBUS_MEMORY_COUNT_0x, &v, sizeof(v)*MODBUS_BYTE_SZ_BITES);
}

#endif // MODBUS_MEMORY_COUNT_0x > 0

// --------------------------------------------------------------------------------------------------------
// ---------------------------------- MODBUS MEMORY 1x (INPUT DISCRETES) ----------------------------------
// --------------------------------------------------------------------------------------------------------

#if MODBUS_MEMORY_COUNT_1x > 0

Modbus::Response ModbusMemory::read_1x(uint16_t offset, uint16_t count, void* bits, uint16_t* fact) const
{
    return read_bits(offset, m_mem1x, MODBUS_MEMORY_COUNT_1x, bits, count, fact);
}
 
Modbus::Response ModbusMemory::write_1x(uint16_t offset, uint16_t count, const void* bits, uint16_t* fact)
{
    return write_bits(offset, m_mem1x, MODBUS_MEMORY_COUNT_1x, bits, count, fact);
}

bool ModbusMemory::bool_1x(uint16_t bitOffset) const
{
    if (bitOffset < MODBUS_MEMORY_COUNT_1x)
        return GET_BIT(m_mem1x, bitOffset);
    return false;
}

void ModbusMemory::setBool_1x(uint16_t bitOffset, bool v)
{
    if (bitOffset < MODBUS_MEMORY_COUNT_1x)
        SET_BIT(m_mem1x, bitOffset, v);
}

int8_t ModbusMemory::int8_1x(uint16_t bitOffset) const
{  
    int8_t v = 0;
    read_bits(bitOffset, m_mem1x, MODBUS_MEMORY_COUNT_1x, &v, sizeof(v)*MODBUS_BYTE_SZ_BITES);
    return v;
}
 
void ModbusMemory::setInt8_1x(uint16_t bitOffset, int8_t v)
{
    write_bits(bitOffset, m_mem1x, MODBUS_MEMORY_COUNT_1x, &v, sizeof(v)*MODBUS_BYTE_SZ_BITES);
}

uint8_t ModbusMemory::uint8_1x(uint16_t bitOffset) const
{
    uint8_t v = 0;
    read_bits(bitOffset, m_mem1x, MODBUS_MEMORY_COUNT_1x, &v, sizeof(v)*MODBUS_BYTE_SZ_BITES);
    return v;
}

void ModbusMemory::setUInt8_1x(uint16_t bitOffset, uint8_t v)
{
    write_bits(bitOffset, m_mem1x, MODBUS_MEMORY_COUNT_1x, &v, sizeof(v)*MODBUS_BYTE_SZ_BITES);
}

int16_t ModbusMemory::int16_1x(uint16_t bitOffset) const
{
    int16_t v = 0;
    read_bits(bitOffset, m_mem1x, MODBUS_MEMORY_COUNT_1x, &v, sizeof(v)*MODBUS_BYTE_SZ_BITES);
    return v;
}

void ModbusMemory::setInt16_1x(uint16_t bitOffset, int16_t v)
{
    write_bits(bitOffset, m_mem1x, MODBUS_MEMORY_COUNT_1x, &v, sizeof(v)*MODBUS_BYTE_SZ_BITES);
}

uint16_t ModbusMemory::uint16_1x(uint16_t bitOffset) const
{
    uint16_t v = 0;
    read_bits(bitOffset, m_mem1x, MODBUS_MEMORY_COUNT_1x, &v, sizeof(v)*MODBUS_BYTE_SZ_BITES);
    return v;
}

void ModbusMemory::setUInt16_1x(uint16_t bitOffset, uint16_t v)
{
    write_bits(bitOffset, m_mem1x, MODBUS_MEMORY_COUNT_1x, &v, sizeof(v)*MODBUS_BYTE_SZ_BITES);
}

int32_t ModbusMemory::int32_1x(uint16_t bitOffset) const
{
    int32_t v = 0;
    read_bits(bitOffset, m_mem1x, MODBUS_MEMORY_COUNT_1x, &v, sizeof(v)*MODBUS_BYTE_SZ_BITES);
    return v;
}

void ModbusMemory::setInt32_1x(uint16_t bitOffset, int32_t v)
{
    write_bits(bitOffset, m_mem1x, MODBUS_MEMORY_COUNT_1x, &v, sizeof(v)*MODBUS_BYTE_SZ_BITES);
}

uint32_t ModbusMemory::uint32_1x(uint16_t bitOffset) const
{
    uint32_t v = 0;
    read_bits(bitOffset, m_mem1x, MODBUS_MEMORY_COUNT_1x, &v, sizeof(v)*MODBUS_BYTE_SZ_BITES);
    return v;
}

void ModbusMemory::setUInt32_1x(uint16_t bitOffset, uint32_t v)
{
    write_bits(bitOffset, m_mem1x, MODBUS_MEMORY_COUNT_1x, &v, sizeof(v)*MODBUS_BYTE_SZ_BITES);
}

int ModbusMemory::int_1x(uint16_t bitOffset) const
{
    int v = 0;
    read_bits(bitOffset, m_mem1x, MODBUS_MEMORY_COUNT_1x, &v, sizeof(v)*MODBUS_BYTE_SZ_BITES);
    return v;
}

void ModbusMemory::setInt_1x(uint16_t bitOffset, int v)
{
    write_bits(bitOffset, m_mem1x, MODBUS_MEMORY_COUNT_1x, &v, sizeof(v)*MODBUS_BYTE_SZ_BITES);
}

unsigned int ModbusMemory::uint_1x(uint16_t bitOffset) const
{
    unsigned int v = 0;
    read_bits(bitOffset, m_mem1x, MODBUS_MEMORY_COUNT_1x, &v, sizeof(v)*MODBUS_BYTE_SZ_BITES);
    return v;
}

void ModbusMemory::setUInt_1x(uint16_t bitOffset, unsigned int v)
{
    write_bits(bitOffset, m_mem1x, MODBUS_MEMORY_COUNT_1x, &v, sizeof(v)*MODBUS_BYTE_SZ_BITES);
}

float ModbusMemory::float_1x(uint16_t bitOffset) const
{
    float v = 0.0f;
    read_bits(bitOffset, m_mem1x, MODBUS_MEMORY_COUNT_1x, &v, sizeof(v)*MODBUS_BYTE_SZ_BITES);
    return v;
}

void ModbusMemory::setFloat_1x(uint16_t bitOffset, float v)
{
    write_bits(bitOffset, m_mem1x, MODBUS_MEMORY_COUNT_1x, &v, sizeof(v)*MODBUS_BYTE_SZ_BITES);
}

double ModbusMemory::double_1x(uint16_t bitOffset) const
{
    double v = 0.0;
    read_bits(bitOffset, m_mem1x, MODBUS_MEMORY_COUNT_1x, &v, sizeof(v)*MODBUS_BYTE_SZ_BITES);
    return v;
}

void ModbusMemory::setDouble_1x(uint16_t bitOffset, double v)
{
    write_bits(bitOffset, m_mem1x, MODBUS_MEMORY_COUNT_1x, &v, sizeof(v)*MODBUS_BYTE_SZ_BITES);
}

#endif // MODBUS_MEMORY_COUNT_1x > 0

// --------------------------------------------------------------------------------------------------------
// ---------------------------------- MODBUS MEMORY 3x (INPUT REGISTERS) ----------------------------------
// --------------------------------------------------------------------------------------------------------

#if MODBUS_MEMORY_COUNT_3x > 0

Modbus::Response ModbusMemory::read_3x(uint16_t offset, uint16_t count, uint16_t* values, uint16_t* fact) const
{
    uint16_t c;
    if (offset >= MODBUS_MEMORY_COUNT_3x)
        return Modbus::ILLEGAL_DATA_ADDRESS;
     
    if ((offset+count) > MODBUS_MEMORY_COUNT_3x)
        c = MODBUS_MEMORY_COUNT_3x - offset;
    else
        c = count;
     
    memcpy(values, &m_mem3x[offset], c*sizeof(uint16_t));
     
    if (fact)
        *fact = c;
    return Modbus::OK;
}
 
Modbus::Response ModbusMemory::write_3x(uint16_t offset, uint16_t count, const uint16_t* values, uint16_t* fact)
{
    uint16_t c;
    if (offset >= MODBUS_MEMORY_COUNT_3x)
        return Modbus::ILLEGAL_DATA_ADDRESS;
     
    if ((offset+count) > MODBUS_MEMORY_COUNT_3x)
        c = MODBUS_MEMORY_COUNT_3x - offset;
    else
        c = count;
     
    memcpy(&m_mem3x[offset], values, c*sizeof(uint16_t));
     
    if (fact)
        *fact = c;
    return Modbus::OK;
}

bool ModbusMemory::bool_3x(uint32_t bitOffset) const
{
    if (bitOffset < MODBUS_MEMORY_SZ_3x_BITES)
    {
        uint16_t r;
        uint8_t b;
        r = static_cast<uint16_t>(bitOffset / MODBUS_REGE_SZ_BITES);
        b = static_cast<uint8_t>(bitOffset % MODBUS_REGE_SZ_BITES);
        return (m_mem3x[r] & (1<<b)) != 0;
    }
    return false;
}

void ModbusMemory::setBool_3x(uint32_t bitOffset, bool v)
{
    if (bitOffset < MODBUS_MEMORY_SZ_3x_BITES)
    {
        uint16_t r;
        uint8_t b;
        r = static_cast<uint16_t>(bitOffset / MODBUS_REGE_SZ_BITES);
        b = static_cast<uint8_t>(bitOffset % MODBUS_REGE_SZ_BITES);
        if (v)
            m_mem3x[r] |= (1<<b);
        else
            m_mem3x[r] &= (~(1<<b));
    }
}

int8_t ModbusMemory::int8_3x(uint32_t byteOffset) const
{    
    if (byteOffset < MODBUS_MEMORY_SZ_3x_BYTES)
        return reinterpret_cast<const int8_t*>(m_mem3x)[byteOffset];
    return 0;
}
 
void ModbusMemory::setInt8_3x(uint32_t byteOffset, int8_t v)
{
    if (byteOffset < MODBUS_MEMORY_SZ_3x_BYTES)
        reinterpret_cast<int8_t*>(m_mem3x)[byteOffset] = v;
}

uint8_t ModbusMemory::uint8_3x(uint32_t byteOffset) const
{    
    if (byteOffset < MODBUS_MEMORY_SZ_3x_BYTES)
        return reinterpret_cast<const uint8_t*>(m_mem3x)[byteOffset];
    return 0;
}

void ModbusMemory::setUInt8_3x(uint32_t byteOffset, uint8_t v)
{
    if (byteOffset < MODBUS_MEMORY_SZ_3x_BYTES)
        reinterpret_cast<uint8_t*>(m_mem3x)[byteOffset] = v;
}

int16_t ModbusMemory::int16_3x(uint16_t regOffset) const
{
    int16_t v = 0;
    read_3x(regOffset, sizeof(v)/sizeof(uint16_t), reinterpret_cast<uint16_t*>(&v));
    return v;
}

void ModbusMemory::setInt16_3x(uint16_t regOffset, int16_t v)
{
    write_3x(regOffset, sizeof(v)/sizeof(uint16_t), reinterpret_cast<uint16_t*>(&v));
}

uint16_t ModbusMemory::uint16_3x(uint16_t regOffset) const
{
    uint16_t v = 0;
    read_3x(regOffset, sizeof(v)/sizeof(uint16_t), reinterpret_cast<uint16_t*>(&v));
    return v;
}

void ModbusMemory::setUInt16_3x(uint16_t regOffset, uint16_t v)
{
    write_3x(regOffset, sizeof(v)/sizeof(uint16_t), reinterpret_cast<uint16_t*>(&v));
}

int32_t ModbusMemory::int32_3x(uint16_t regOffset) const
{
    int32_t v = 0;
    read_3x(regOffset, sizeof(v)/sizeof(uint16_t), reinterpret_cast<uint16_t*>(&v));
    return v;
}

void ModbusMemory::setInt32_3x(uint16_t regOffset, int32_t v)
{
    write_3x(regOffset, sizeof(v)/sizeof(uint16_t), reinterpret_cast<uint16_t*>(&v));
}

uint32_t ModbusMemory::uint32_3x(uint16_t regOffset) const
{
    uint32_t v = 0;
    read_3x(regOffset, sizeof(v)/sizeof(uint16_t), reinterpret_cast<uint16_t*>(&v));
    return v;
}

void ModbusMemory::setUInt32_3x(uint16_t regOffset, uint32_t v)
{
    write_3x(regOffset, sizeof(v)/sizeof(uint16_t), reinterpret_cast<uint16_t*>(&v));
}

void ModbusMemory::setInt_3x(uint16_t regOffset, int v)
{
    write_3x(regOffset, sizeof(v)/sizeof(uint16_t), reinterpret_cast<uint16_t*>(&v));
}

unsigned int ModbusMemory::uint_3x(uint16_t regOffset) const
{
    unsigned int v = 0;
    read_3x(regOffset, sizeof(v)/sizeof(uint16_t), reinterpret_cast<uint16_t*>(&v));
    return v;
}

void ModbusMemory::setUInt_3x(uint16_t regOffset, unsigned int v)
{
    write_3x(regOffset, sizeof(v)/sizeof(uint16_t), reinterpret_cast<uint16_t*>(&v));
}

float ModbusMemory::float_3x(uint16_t regOffset) const
{
    float v = 0.0f;
    read_3x(regOffset, sizeof(v)/sizeof(uint16_t), reinterpret_cast<uint16_t*>(&v));
    return v;
}

void ModbusMemory::setFloat_3x(uint16_t regOffset, float v)
{
    write_3x(regOffset, sizeof(v)/sizeof(uint16_t), reinterpret_cast<uint16_t*>(&v));
}

double ModbusMemory::double_3x(uint16_t regOffset) const
{
    double v = 0.0;
    read_3x(regOffset, sizeof(v)/sizeof(uint16_t), reinterpret_cast<uint16_t*>(&v));
    return v;
}

void ModbusMemory::setDouble_3x(uint16_t regOffset, double v)
{
    write_3x(regOffset, sizeof(v)/sizeof(uint16_t), reinterpret_cast<uint16_t*>(&v));
}

#endif // MODBUS_MEMORY_COUNT_3x > 0

//--------------------------------------------------------------------------------------------------------
//--------------------------------- MODBUS MEMORY 4x (HOLDING REGISTERS) ---------------------------------
//--------------------------------------------------------------------------------------------------------

#if MODBUS_MEMORY_COUNT_4x > 0

Modbus::Response ModbusMemory::read_4x(uint16_t offset, uint16_t count, uint16_t* values, uint16_t* fact) const
{
    uint16_t c;
    if (offset >= MODBUS_MEMORY_COUNT_4x)
        return Modbus::ILLEGAL_DATA_ADDRESS;
     
    if ((offset+count) > MODBUS_MEMORY_COUNT_4x)
        c = MODBUS_MEMORY_COUNT_4x - offset;
    else
        c = count;
     
    memcpy(values, &m_mem4x[offset], c*sizeof(uint16_t));
     
    if (fact)
        *fact = c;
    return Modbus::OK;
}
 
Modbus::Response ModbusMemory::write_4x(uint16_t offset, uint16_t count, const uint16_t* values, uint16_t* fact)
{
    uint16_t c;
    if (offset >= MODBUS_MEMORY_COUNT_4x)
        return Modbus::ILLEGAL_DATA_ADDRESS;
     
    if ((offset+count) > MODBUS_MEMORY_COUNT_4x)
        c = MODBUS_MEMORY_COUNT_4x - offset;
    else
        c = count;
     
    memcpy(&m_mem4x[offset], values, c*sizeof(uint16_t));
     
    if (fact)
        *fact = c;
    return Modbus::OK;
}

bool ModbusMemory::bool_4x(uint32_t bitOffset) const
{
    if (bitOffset < MODBUS_MEMORY_SZ_4x_BITES)
    {
        uint16_t r;
        uint8_t b;
        r = static_cast<uint16_t>(bitOffset / MODBUS_REGE_SZ_BITES);
        b = static_cast<uint8_t>(bitOffset % MODBUS_REGE_SZ_BITES);
        return (m_mem4x[r] & (1<<b)) != 0;
    }
    return false;
}

void ModbusMemory::setBool_4x(uint32_t bitOffset, bool v)
{
    if (bitOffset < MODBUS_MEMORY_SZ_4x_BITES)
    {
        uint16_t r;
        uint8_t b;
        r = static_cast<uint16_t>(bitOffset / MODBUS_REGE_SZ_BITES);
        b = static_cast<uint8_t>(bitOffset % MODBUS_REGE_SZ_BITES);
        if (v)
            m_mem4x[r] |= (1<<b);
        else
            m_mem4x[r] &= (~(1<<b));
    }
}

int8_t ModbusMemory::int8_4x(uint32_t byteOffset) const
{
    if (byteOffset < MODBUS_MEMORY_SZ_4x_BYTES)
        return reinterpret_cast<const int8_t*>(m_mem4x)[byteOffset];
    return 0;
}
 
void ModbusMemory::setInt8_4x(uint32_t byteOffset, int8_t v)
{   
    if (byteOffset < MODBUS_MEMORY_SZ_4x_BYTES)
        reinterpret_cast<int8_t*>(m_mem4x)[byteOffset] = v;
}

uint8_t ModbusMemory::uint8_4x(uint32_t byteOffset) const
{
    if (byteOffset < MODBUS_MEMORY_SZ_4x_BYTES)
        return reinterpret_cast<const uint8_t*>(m_mem4x)[byteOffset];
    return 0;
}

void ModbusMemory::setUInt8_4x(uint32_t byteOffset, uint8_t v)
{
    if (byteOffset < MODBUS_MEMORY_SZ_4x_BYTES)
        reinterpret_cast<uint8_t*>(m_mem4x)[byteOffset] = v;
}

int16_t ModbusMemory::int16_4x(uint16_t regOffset) const
{
    int16_t v = 0;
    read_4x(regOffset, sizeof(v)/sizeof(uint16_t), reinterpret_cast<uint16_t*>(&v));
    return v;
}

void ModbusMemory::setInt16_4x(uint16_t regOffset, int16_t v)
{
    write_4x(regOffset, sizeof(v)/sizeof(uint16_t), reinterpret_cast<uint16_t*>(&v));
}

uint16_t ModbusMemory::uint16_4x(uint16_t regOffset) const
{
    uint16_t v = 0;
    read_4x(regOffset, sizeof(v)/sizeof(uint16_t), reinterpret_cast<uint16_t*>(&v));
    return v;
}

void ModbusMemory::setUInt16_4x(uint16_t regOffset, uint16_t v)
{
    write_4x(regOffset, sizeof(v)/sizeof(uint16_t), reinterpret_cast<uint16_t*>(&v));
}

int32_t ModbusMemory::int32_4x(uint16_t regOffset) const
{
    int32_t v = 0;
    read_4x(regOffset, sizeof(v)/sizeof(uint16_t), reinterpret_cast<uint16_t*>(&v));
    return v;
}

void ModbusMemory::setInt32_4x(uint16_t regOffset, int32_t v)
{
    write_4x(regOffset, sizeof(v)/sizeof(uint16_t), reinterpret_cast<uint16_t*>(&v));
}

uint32_t ModbusMemory::uint32_4x(uint16_t regOffset) const
{
    uint32_t v = 0;
    read_4x(regOffset, sizeof(v)/sizeof(uint16_t), reinterpret_cast<uint16_t*>(&v));
    return v;
}

void ModbusMemory::setUInt32_4x(uint16_t regOffset, uint32_t v)
{
    write_4x(regOffset, sizeof(v)/sizeof(uint16_t), reinterpret_cast<uint16_t*>(&v));
}

int ModbusMemory::int_4x(uint16_t regOffset) const
{
    int v = 0;
    read_4x(regOffset, sizeof(v)/sizeof(uint16_t), reinterpret_cast<uint16_t*>(&v));
    return v;
}

void ModbusMemory::setInt_4x(uint16_t regOffset, int v)
{
    write_4x(regOffset, sizeof(v)/sizeof(uint16_t), reinterpret_cast<uint16_t*>(&v));
}

unsigned int ModbusMemory::uint_4x(uint16_t regOffset) const
{
    unsigned int v = 0;
    read_4x(regOffset, sizeof(v)/sizeof(uint16_t), reinterpret_cast<uint16_t*>(&v));
    return v;
}

void ModbusMemory::setUInt_4x(uint16_t regOffset, unsigned int v)
{
    write_4x(regOffset, sizeof(v)/sizeof(uint16_t), reinterpret_cast<uint16_t*>(&v));
}

float ModbusMemory::float_4x(uint16_t regOffset) const
{
    float v = 0.0f;
    read_4x(regOffset, sizeof(v)/sizeof(uint16_t), reinterpret_cast<uint16_t*>(&v));
    return v;
}

void ModbusMemory::setFloat_4x(uint16_t regOffset, float v)
{
    write_4x(regOffset, sizeof(v)/sizeof(uint16_t), reinterpret_cast<uint16_t*>(&v));
}

double ModbusMemory::double_4x(uint16_t regOffset) const
{
    double v = 0.0;
    read_4x(regOffset, sizeof(v)/sizeof(uint16_t), reinterpret_cast<uint16_t*>(&v));
    return v;
}

void ModbusMemory::setDouble_4x(uint16_t regOffset, double v)
{
    write_4x(regOffset, sizeof(v)/sizeof(uint16_t), reinterpret_cast<uint16_t*>(&v));
}

#endif // MODBUS_MEMORY_COUNT_4x > 0

#endif // MODBUS_MEMORY_H
