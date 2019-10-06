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

#ifndef MODBUSMASTERRTU_H
#define MODBUSMASTERRTU_H

#include "ModbusMaster.h"

// --------------------------------------------------------------------------------------------------------
// ------------------------------------------- MODBUS MASTER RTU ------------------------------------------
// --------------------------------------------------------------------------------------------------------

class ModbusMasterRTU : public ModbusMaster
{   
public:
    ModbusMasterRTU(Stream* stream);
    
public:
    virtual Modbus::Type type() const { return Modbus::RTU; }

public:
    inline Stream* stream() const { return m_stream; }
    inline void setStream(Stream* stream) { m_stream = stream; }
    inline unsigned long timeoutFirstByte() const { return m_timeoutFB; }
    inline void setTimeoutFirstByte(unsigned long timeout) { m_timeoutFB = timeout; }
    inline unsigned long timeoutInterByte() const { return m_timeoutIB; }
    inline void setTimeoutInterByte(unsigned long timeout) { m_timeoutIB = timeout; }

protected: // buffer control interface
    virtual uint16_t bufferSize() const;
    virtual uint8_t bufferByteAt(uint16_t offset) const;
    virtual void getBufferBytesAt(uint16_t offset, void *buff, uint16_t count) const;
    virtual void setBufferByteAt(uint16_t offset, uint8_t value);
    virtual void setBufferBytesAt(uint16_t offset, const void *buff, uint16_t count);
        
protected:
    virtual Modbus::Response exec(uint8_t &slave, uint8_t func, uint16_t szInBuff, uint16_t* szOutBuff);
    
private:
    Stream* m_stream;
    unsigned long m_timeoutFB;
    unsigned long m_timeoutIB;
    unsigned long m_start;
    uint8_t m_slave;
    uint8_t m_func;
    uint8_t m_buff[MB_RTU_IO_BUFF_SZ];
    uint16_t m_sz;
};

#endif // MODBUSMASTERRTU_H
