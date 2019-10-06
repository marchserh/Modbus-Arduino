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

#ifndef MODBUSSLAVEIO_H
#define MODBUSSLAVEIO_H

#include "Modbus.h"

class Stream;

class ModbusSlaveIO
{
public:
    // Modbus Slave states
    enum State
    {
        STATE_BEGIN                ,
        STATE_UNKNOWN              = STATE_BEGIN,
        STATE_BEGIN_READ           ,
        STATE_WAIT_FOR_READ        ,
        STATE_WAIT_FOR_READ_ALL    ,
        STATE_READ                 ,
        STATE_PROCESS_DEVICE       ,
        STATE_WRITE                ,
        STATE_BEGIN_WRITE          ,
        STATE_WAIT_FOR_WRITE       ,
        STATE_WAIT_FOR_WRITE_ALL   ,
        STATE_END                  = STATE_WAIT_FOR_WRITE_ALL
    };

public:
    ModbusSlaveIO() { m_name = MB_NULLPTR; m_verboseStream = MB_NULLPTR; }
    
public:
    virtual Modbus::Type type() const = 0;
    inline State state() const { return m_state; }
    inline const char* name() const { return m_name; }
    inline void setName(const char* name) { m_name = name; }
    inline Stream* verboseStream() const { return m_verboseStream; }
    inline void setVerboseStream(Stream* stream) { m_verboseStream = stream; }

protected: // buffer control interface
    virtual uint16_t bufferSize() const = 0;
    virtual uint8_t bufferByteAt(uint16_t offset) const = 0;
    inline uint8_t bufferByte(uint16_t offset) const { if (offset < bufferSize()) return bufferByteAt(offset); return 0; }
    virtual void getBufferBytesAt(uint16_t offset, void *buff, uint16_t count) const = 0;
    virtual void setBufferByteAt(uint16_t offset, uint8_t value) = 0;
    inline void setBufferByte(uint16_t offset, uint8_t value) { if (offset < bufferSize()) setBufferByteAt(offset, value); }
    virtual void setBufferBytesAt(uint16_t offset, const void *buff, uint16_t count) = 0;

protected: // IO interface
    virtual Modbus::Response begin() = 0;
    virtual Modbus::Response read(uint8_t &slave, uint8_t &func, uint16_t &szBuff) = 0;
    virtual Modbus::Response write(uint8_t slave, uint8_t func, uint16_t szBuff) = 0;
    
protected:
    State m_state;
    const char* m_name;
    Stream* m_verboseStream;
    unsigned long m_start;
};

#endif // MODBUSSLAVEIO_H
