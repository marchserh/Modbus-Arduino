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

#ifndef MODBUSSLAVEIOTCP_H
#define MODBUSSLAVEIOTCP_H

#include "ModbusSlaveIO.h"

// --------------------------------------------------------------------------------------------------------
// ----------------------------------------- MODBUS SLAVE IO TCP ------------------------------------------
// --------------------------------------------------------------------------------------------------------

class ModbusSlaveIOTCP : public virtual ModbusSlaveIO
{
public:
    ModbusSlaveIOTCP(uint16_t port = Modbus::STANDARD_TCP_PORT);

public:
    virtual Modbus::Type type() const { return Modbus::TCP; }
    uint8_t sockStatus();
    inline unsigned long timeoutRequest() const { return m_timeoutRequest; }
    inline void setTimeoutRequest(unsigned long timeoutRequest) { m_timeoutRequest = timeoutRequest; }
    
protected: // buffer control interface
    virtual uint16_t bufferSize() const;
    virtual uint8_t bufferByteAt(uint16_t offset) const;
    virtual void getBufferBytesAt(uint16_t offset, void *buff, uint16_t count) const;
    virtual void setBufferByteAt(uint16_t offset, uint8_t value);
    virtual void setBufferBytesAt(uint16_t offset, const void *buff, uint16_t count);

protected: // IO interface
    virtual Modbus::Response begin();
    virtual Modbus::Response read(uint8_t &slave, uint8_t &func, uint16_t &szBuff);
    virtual Modbus::Response write(uint8_t slave, uint8_t func, uint16_t szBuff);
    
private:
    bool checkConnection();

private:
    uint16_t m_port;
    uint8_t m_sock;
    uint16_t m_transaction;
    unsigned long m_timeoutRequest;
    unsigned long m_startRequest;
    uint8_t m_buff[MB_TCP_IO_BUFF_SZ];
};

#endif // MODBUSSLAVEIOTCP_H
