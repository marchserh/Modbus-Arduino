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

#ifndef MODBUSMASTERTCP_H
#define MODBUSMASTERTCP_H

#include <IPAddress.h>

#include "ModbusMaster.h"

// --------------------------------------------------------------------------------------------------------
// ------------------------------------------ MODBUS MASTER TCP -------------------------------------------
// --------------------------------------------------------------------------------------------------------

class ModbusMasterTCP : public ModbusMaster
{
public:
    ModbusMasterTCP(const char* host, uint16_t port = Modbus::STANDARD_TCP_PORT);
    
public:
     virtual Modbus::Type type() const { return Modbus::TCP; }
     
public:
    Modbus::Response connect();
    Modbus::Response disconnect();
    bool isConnected() const;
    inline unsigned long timeout() const { return m_timeout; }
    inline void setTimeout(unsigned long timeout) { m_timeout = timeout; }

protected: // buffer control interface
    virtual uint16_t bufferSize() const;
    virtual uint8_t bufferByteAt(uint16_t offset) const;
    virtual void getBufferBytesAt(uint16_t offset, void *buff, uint16_t count) const;
    virtual void setBufferByteAt(uint16_t offset, uint8_t value);
    virtual void setBufferBytesAt(uint16_t offset, const void *buff, uint16_t count);
        
protected:
    virtual Modbus::Response exec(uint8_t &slave, uint8_t func, uint16_t szInBuff, uint16_t* szOutBuff);

private:
    inline void deblockBuffer() { m_block = false; }
    Modbus::Response writeBuffer(uint8_t slave, uint8_t func, uint16_t szInBuff);
    Modbus::Response readBuffer(uint8_t &slave, uint16_t* szOutBuff);
    Modbus::Response write();
    int available() const;
    int read();

private:
    static uint16_t s_srcport;
    uint8_t m_sock;
    IPAddress m_ip;
    uint16_t m_port;
    uint16_t m_transaction;
    unsigned long m_timeout;
    unsigned long m_start;
    uint8_t m_slave;
    uint8_t m_func;
    uint8_t m_buff[MB_TCP_IO_BUFF_SZ];
    uint16_t m_sz;
    bool m_block;
};

#endif // MODBUSMASTERTCP_H
