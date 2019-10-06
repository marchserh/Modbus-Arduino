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

#include "ModbusMaster.h"

#include <Arduino.h>

ModbusMaster::ModbusMaster()
{
    m_name = MB_NULLPTR;
    m_verboseStream = MB_NULLPTR;
    m_state = STATE_UNKNOWN;
}

Modbus::Response ModbusMaster::readCoilStatus(uint8_t &slave, uint16_t offset, uint16_t count, void* bits, uint16_t* fact)
{
    Modbus::Response r;
    uint16_t szOutBuff, fcBites, fcBytes;
    
    switch (m_state)
    {
    case STATE_UNKNOWN:
    case STATE_DISCONNECTED:
    case STATE_CONNECTED:
    case STATE_BEGIN_WRITE:
        m_mem = count;
        setBufferByteAt(0, reinterpret_cast<uint8_t*>(&offset)[1]); // start coil offset - MS BYTE
        setBufferByteAt(1, reinterpret_cast<uint8_t*>(&offset)[0]); // start coil offset - LS BYTE
        setBufferByteAt(2, reinterpret_cast<uint8_t*>(&count)[1]);  // quantity of coils - MS BYTE
        setBufferByteAt(3, reinterpret_cast<uint8_t*>(&count)[0]);  // quantity of coils - LS BYTE
        m_state = STATE_WRITE;
        // no need break
    default:
        r = exec(slave,                  // slave device ID
                 MBF_READ_COIL_STATUS,   // function number
                 4,                      // size of input bytes of data
                 &szOutBuff);            // size of output bytes of data
        if (r != Modbus::OK) // error or processing
            return r;
        if (!szOutBuff)
            return Modbus::CMN_ERR_NOT_CORRECT;        
        fcBytes = bufferByteAt(0); // count of bytes received
        if (fcBytes != szOutBuff-1)
            return Modbus::CMN_ERR_NOT_CORRECT;        
        fcBites = fcBytes * 8; // count of bits (coils) received
        if (fcBites > m_mem)
            fcBites = m_mem;
        fcBytes = (fcBites+7)/8;
        if (fact)
            *fact = fcBites;
        getBufferBytesAt(1, bits, fcBytes);        
        // no need break
    }
    return Modbus::OK;
}

Modbus::Response ModbusMaster::readInputStatus(uint8_t &slave, uint16_t offset, uint16_t count, void* bits, uint16_t* fact)
{
    Modbus::Response r;
    uint16_t szOutBuff, fcBites, fcBytes;
    
    switch (m_state)
    {
    case STATE_UNKNOWN:
    case STATE_DISCONNECTED:
    case STATE_CONNECTED:
    case STATE_BEGIN_WRITE:
        m_mem = count;
        setBufferByteAt(0, reinterpret_cast<uint8_t*>(&offset)[1]); // start discrete offset - MS BYTE
        setBufferByteAt(1, reinterpret_cast<uint8_t*>(&offset)[0]); // start discrete offset - LS BYTE
        setBufferByteAt(2, reinterpret_cast<uint8_t*>(&count)[1]);  // quantity of discrete - MS BYTE
        setBufferByteAt(3, reinterpret_cast<uint8_t*>(&count)[0]);  // quantity of discrete - LS BYTE
        m_state = STATE_WRITE;
        // no need break
    default:
        r = exec(slave,                  // slave device ID
                 MBF_READ_INPUT_STATUS,  // function number
                 4,                      // size of input bytes of data
                 &szOutBuff);            // size of output bytes of data
        if (r != Modbus::OK) // error or processing
            return r; 
        if (!szOutBuff)
            return Modbus::CMN_ERR_NOT_CORRECT;        
        fcBytes = bufferByteAt(0); // count of bytes received
        if (fcBytes != szOutBuff-1)
            return Modbus::CMN_ERR_NOT_CORRECT;        
        fcBites = fcBytes * 8; // count of bits (inputs) received
        if (fcBites > m_mem)
            fcBites = m_mem;
        fcBytes = (fcBites+7)/8;
        if (fact)
            *fact = fcBites;
        getBufferBytesAt(1, bits, fcBytes);        
        // no need break
    }
    return Modbus::OK;
}

Modbus::Response ModbusMaster::readHoldingRegisters(uint8_t &slave, uint16_t offset, uint16_t count, uint16_t* values, uint16_t* fact)
{
    Modbus::Response r;
    uint16_t szOutBuff, fcRegs, fcBytes;

    switch (m_state)
    {
    case STATE_UNKNOWN:
    case STATE_DISCONNECTED:
    case STATE_CONNECTED:
    case STATE_BEGIN_WRITE:
        m_mem = count;
        setBufferByteAt(0, reinterpret_cast<uint8_t*>(&offset)[1]); // start register offset - MS BYTE
        setBufferByteAt(1, reinterpret_cast<uint8_t*>(&offset)[0]); // start register offset - LS BYTE
        setBufferByteAt(2, reinterpret_cast<uint8_t*>(&count)[1]);  // quantity of values - MS BYTE
        setBufferByteAt(3, reinterpret_cast<uint8_t*>(&count)[0]);  // quantity of values - LS BYTE
        m_state = STATE_WRITE;
        // no need break
    default:
        r = exec(slave,                      // slave device ID
                 MBF_READ_HOLDING_REGISTERS, // function number
                 4,			                 // size of input bytes of data
                 &szOutBuff);                // size of output bytes of data
        if (r != Modbus::OK) // error or processing
            return r; 
       if (!szOutBuff)
            return Modbus::CMN_ERR_NOT_CORRECT;        
        fcBytes = bufferByteAt(0);  // count of bytes received
        if (fcBytes != szOutBuff-1)
            return Modbus::CMN_ERR_NOT_CORRECT;        
        fcRegs = fcBytes / sizeof(uint16_t); // count values received
        if (fcRegs > m_mem) // count of values responsed is greater then requested - it's a COLLISION!!!
            return Modbus::CMN_ERR_NOT_CORRECT;
        if (fact) 
            *fact = fcRegs;
        for (int i = 0; i < fcRegs; i++)
            values[i] = bufferByteAt(i*2+2) | (bufferByteAt(i*2+1)<<8);       
        // no need break
    }
    return Modbus::OK;
}

Modbus::Response ModbusMaster::readInputRegisters(uint8_t &slave, uint16_t offset, uint16_t count, uint16_t* values, uint16_t* fact)
{
    Modbus::Response r;
    uint16_t szOutBuff, fcRegs, fcBytes;
    
    switch (m_state)
    {
    case STATE_UNKNOWN:
    case STATE_DISCONNECTED:
    case STATE_CONNECTED:
    case STATE_BEGIN_WRITE:
        m_mem = count;
        setBufferByteAt(0, reinterpret_cast<uint8_t*>(&offset)[1]); // start register offset - MS BYTE
        setBufferByteAt(1, reinterpret_cast<uint8_t*>(&offset)[0]); // start register offset - LS BYTE
        setBufferByteAt(2, reinterpret_cast<uint8_t*>(&m_mem)[1]);  // quantity of values - MS BYTE
        setBufferByteAt(3, reinterpret_cast<uint8_t*>(&m_mem)[0]);  // quantity of values - LS BYTE
        m_state = STATE_WRITE;
        // no need break
    default:
        r = exec(slave,                      // slave device ID
                 MBF_READ_INPUT_REGISTERS,   // function number
                 4,                          // size of input bytes of data
                 &szOutBuff);                // size of output bytes of data
        if (r != Modbus::OK) // error or processing
            return r; 
        if (!szOutBuff)
            return Modbus::CMN_ERR_NOT_CORRECT;
        fcBytes = bufferByteAt(0);  // count of bytes received
        if (fcBytes != szOutBuff-1)
            return Modbus::CMN_ERR_NOT_CORRECT;
        fcRegs = fcBytes / sizeof(uint16_t); // count values received
        if (fcRegs > m_mem) // count of values responsed is greater then requested - it's a COLLISION!!!
            return Modbus::CMN_ERR_NOT_CORRECT;
        if (fact) 
            *fact = fcRegs;
        for (int i = 0; i < fcRegs; i++)
            values[i] = bufferByteAt(i*2+2) | (bufferByteAt(i*2+1)<<8);       
        // no need break
    }
    return Modbus::OK;
}

Modbus::Response ModbusMaster::forceSingleCoil(uint8_t &slave, uint16_t offset, bool value)
{
    Modbus::Response r;
    uint16_t szOutBuff, outOffset;
    
    switch (m_state)
    {
    case STATE_UNKNOWN:
    case STATE_DISCONNECTED:
    case STATE_CONNECTED:
    case STATE_BEGIN_WRITE:
        m_memOffset = offset;
        setBufferByteAt(0, reinterpret_cast<uint8_t*>(&offset)[1]);   // coil offset - MS BYTE
        setBufferByteAt(1, reinterpret_cast<uint8_t*>(&offset)[0]);   // coil offset - LS BYTE
        setBufferByteAt(2, (value ? 0xFF : 0x00));                    // value - 0xFF if true or 0x00 if false
        setBufferByteAt(3, 0x00);                                     // value - must always be NULL
        m_state = STATE_WRITE;
        // no need break
    default:
        r = exec(slave,                  // slave device ID
                 MBF_FORCE_SINGLE_COIL,  // function number
                 4,                      // size of input bytes of data
                 &szOutBuff);            // size of output bytes of data
        if (r != Modbus::OK) // error or processing
            return r; 
        if (szOutBuff != 4)
            return Modbus::CMN_ERR_NOT_CORRECT;
        outOffset = bufferByteAt(1) | (bufferByteAt(0)<<8);
        if (outOffset != m_memOffset)
            return Modbus::CMN_ERR_NOT_CORRECT;
        // no need break
    }
    return Modbus::OK;
}

Modbus::Response ModbusMaster::forceSingleRegister(uint8_t &slave, uint16_t offset, uint16_t value)
{
    Modbus::Response r;
    uint16_t szOutBuff, outOffset, outValue;
    
    switch (m_state)
    {
    case STATE_UNKNOWN:
    case STATE_DISCONNECTED:
    case STATE_CONNECTED:
    case STATE_BEGIN_WRITE:
        m_memOffset = offset;
        m_mem = value;
        setBufferByteAt(0, reinterpret_cast<uint8_t*>(&offset)[1]);     // register offset - MS BYTE
        setBufferByteAt(1, reinterpret_cast<uint8_t*>(&offset)[0]);     // register offset - LS BYTE
        setBufferByteAt(2, reinterpret_cast<uint8_t*>(&value)[1]);      // value - MS BYTE
        setBufferByteAt(3, reinterpret_cast<uint8_t*>(&value)[0]);      // value - LS BYTE
        m_state = STATE_WRITE;
        // no need break
    default:
        r = exec(slave,                      // slave device ID
                 MBF_FORCE_SINGLE_REGISTER,  // function number
                 4,                          // size of input bytes of data
                 &szOutBuff);                // size of output bytes of data
        if (r != Modbus::OK) // error or processing
            return r; 
        if (szOutBuff != 4)
            return Modbus::CMN_ERR_NOT_CORRECT;
        outOffset = bufferByteAt(1) | (bufferByteAt(0)<<8);
        outValue = bufferByteAt(3) | (bufferByteAt(2)<<8);
        if (!(outOffset == m_memOffset) && (outValue == m_mem))
            return Modbus::CMN_ERR_NOT_CORRECT;
        // no need break
    }
    return Modbus::OK;
}

Modbus::Response ModbusMaster::forceMultipleCoils(uint8_t &slave, uint16_t offset, uint16_t count, const void* bits, uint16_t* fact)
{
    Modbus::Response r;
    uint16_t szOutBuff, outOffset, fcCoils;
    
    switch (m_state)
    {
    case STATE_UNKNOWN:
    case STATE_DISCONNECTED:
    case STATE_CONNECTED:
    case STATE_BEGIN_WRITE:
        m_memOffset = offset;
        setBufferByteAt(0, reinterpret_cast<uint8_t*>(&offset)[1]); // start coil offset - MS BYTE
        setBufferByteAt(1, reinterpret_cast<uint8_t*>(&offset)[0]); // start coil offset - LS BYTE
        setBufferByteAt(2, reinterpret_cast<uint8_t*>(&count)[1]);  // quantity of coils - MS BYTE
        setBufferByteAt(3, reinterpret_cast<uint8_t*>(&count)[0]);  // quantity of coils - LS BYTE
        setBufferByteAt(4, static_cast<uint8_t>((count+7)/8));      // quantity of next bytes
        setBufferBytesAt(5, bits, (count+7)/8);
        m_state = STATE_WRITE;
        // no need break
    default:
        r = exec(slave,                      // slave device ID
                 MBF_FORCE_MULTIPLE_COILS,   // function number
                 5 + bufferByteAt(4),    // size of input bytes of data
                 &szOutBuff);                // size of output bytes of data					   
        if (r != Modbus::OK) // error or processing
            return r; 
        if (szOutBuff != 4)
            return Modbus::CMN_ERR_NOT_CORRECT;
        outOffset = bufferByteAt(1) | (bufferByteAt(0) << 8);
        if (outOffset != m_memOffset)
            return Modbus::CMN_ERR_NOT_CORRECT;    
        fcCoils = bufferByteAt(3) | (bufferByteAt(2) << 8);
        if (fact)
            *fact = fcCoils;
        // no need break
    }
    return Modbus::OK;
}

Modbus::Response ModbusMaster::forceMultipleRegisters(uint8_t &slave, uint16_t offset, uint16_t count, const uint16_t* values, uint16_t* fact)
{
    Modbus::Response r;
    uint16_t szOutBuff, outOffset, fcRegs;
    
    switch (m_state)
    {
    case STATE_UNKNOWN:
    case STATE_DISCONNECTED:
    case STATE_CONNECTED:
    case STATE_BEGIN_WRITE:
        m_memOffset = offset;
        setBufferByteAt(0, reinterpret_cast<uint8_t*>(&offset)[1]);   // start register offset - MS BYTE
        setBufferByteAt(1, reinterpret_cast<uint8_t*>(&offset)[0]);   // start register offset - LS BYTE
        setBufferByteAt(2, reinterpret_cast<uint8_t*>(&count)[1]);    // quantity of registers - MS BYTE
        setBufferByteAt(3, reinterpret_cast<uint8_t*>(&count)[0]);    // quantity of registers - LS BYTE
        setBufferByteAt(4, static_cast<uint8_t>(count*2));            // quantity of next bytes
        for(int i = 0; i < count; i++)
        {
            setBufferByteAt(5+i*2, reinterpret_cast<const uint8_t*>(&values[i])[1]);
            setBufferByteAt(6+i*2, reinterpret_cast<const uint8_t*>(&values[i])[0]);
        }
        m_state = STATE_WRITE;
        // no need break
    default:
        r = exec(slave,                        // slave device ID
                 MBF_FORCE_MULTIPLE_REGISTERS, // function number
                 5+bufferByteAt(4),        // size of input bytes of data
                 &szOutBuff);                  // size of output bytes of data					   
        if (r != Modbus::OK) // error or processing
            return r; 
        if (szOutBuff != 4)
            return Modbus::CMN_ERR_NOT_CORRECT;       
        outOffset = bufferByteAt(1) | (bufferByteAt(0)<<8);
        if (outOffset != m_memOffset)
            return Modbus::CMN_ERR_NOT_CORRECT;    
        fcRegs = bufferByteAt(3) | (bufferByteAt(2) << 8);
        if (fact)
            *fact = fcRegs;
        // no need break
    }
    return Modbus::OK;
}
