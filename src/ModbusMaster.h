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

#ifndef MODBUSMASTER_H
#define MODBUSMASTER_H

#include "Modbus.h"

class Stream;

// --------------------------------------------------------------------------------------------------------
// ------------------------------------------ MODBUS MASTER BASE ------------------------------------------
// --------------------------------------------------------------------------------------------------------

class ModbusMaster : public ModbusInterface
{
public:
    // Modbus Master states
    enum State
    {
        STATE_BEGIN                ,
        STATE_UNKNOWN              = STATE_BEGIN,
        STATE_CONNECTED            ,
        STATE_WAIT_FOR_CONNECT     ,
        STATE_WAIT_FOR_DISCONNECT  ,
        STATE_DISCONNECTED         ,
        STATE_BEGIN_READ           ,
        STATE_WAIT_FOR_READ        ,
        STATE_WAIT_FOR_READ_ALL    ,
        STATE_READ                 ,
        STATE_WRITE                ,
        STATE_BEGIN_WRITE          ,
        STATE_WAIT_FOR_WRITE       ,
        STATE_WAIT_FOR_WRITE_ALL   ,
        STATE_END                  = STATE_WAIT_FOR_WRITE_ALL
    };

public:
    ModbusMaster();
    
public:
    virtual Modbus::Type type() const = 0;
    
public:
    inline const char* name() const { return m_name; }
    inline void setName(const char* name) { m_name = name; }
    inline Stream* verboseStream() const { return m_verboseStream; }
    inline void setVerboseStream(Stream* stream) { m_verboseStream = stream; }
    inline State state() const { return m_state; }
    
public: // Modbus Interface
    virtual Modbus::Response readCoilStatus(uint8_t &slave, uint16_t offset, uint16_t count, void* bits, uint16_t* fact = MB_NULLPTR);
    virtual Modbus::Response readInputStatus(uint8_t &slave, uint16_t offset, uint16_t count, void* bits, uint16_t* fact = MB_NULLPTR);
    virtual Modbus::Response readHoldingRegisters(uint8_t &slave, uint16_t offset, uint16_t count, uint16_t* values, uint16_t* fact = MB_NULLPTR);
    virtual Modbus::Response readInputRegisters(uint8_t &slave, uint16_t offset, uint16_t count, uint16_t* values, uint16_t* fact = MB_NULLPTR);
    virtual Modbus::Response forceSingleCoil(uint8_t &slave, uint16_t offset, bool value);
    virtual Modbus::Response forceSingleRegister(uint8_t &slave, uint16_t offset, uint16_t value);
    virtual Modbus::Response forceMultipleCoils(uint8_t &slave, uint16_t offset, uint16_t count, const void* bits, uint16_t* fact = MB_NULLPTR);
    virtual Modbus::Response forceMultipleRegisters(uint8_t &slave, uint16_t offset, uint16_t count, const uint16_t* values, uint16_t* fact = MB_NULLPTR);

protected: // buffer control interface
    virtual uint16_t bufferSize() const = 0;
    virtual uint8_t bufferByteAt(uint16_t offset) const = 0;
    inline uint8_t bufferByte(uint16_t offset) const { if (offset < bufferSize()) return bufferByteAt(offset); return 0; }
    virtual void getBufferBytesAt(uint16_t offset, void *buff, uint16_t count) const = 0;
    virtual void setBufferByteAt(uint16_t offset, uint8_t value) = 0;
    inline void setBufferByte(uint16_t offset, uint8_t value) { if (offset < bufferSize()) setBufferByteAt(offset, value); }
    virtual void setBufferBytesAt(uint16_t offset, const void *buff, uint16_t count) = 0;
    
protected:
    virtual Modbus::Response exec(uint8_t &slave, uint8_t func, uint16_t szInBuff, uint16_t* szOutBuff) = 0;
    
protected:
    const char* m_name;
    Stream* m_verboseStream;
    State m_state;
    uint16_t m_memOffset;
    uint16_t m_mem;
};

#endif // MODBUSMASTER_H
