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

#ifndef MODBUSSLAVEBRIDGE_H
#define MODBUSSLAVEBRIDGE_H

#include "ModbusSlaveBase.h"

#define MBSLAVEBRIDGE_BUFF_SZ 260

class ModbusInterface;

// --------------------------------------------------------------------------------------------------------
// ------------------------------------------ MODBUS SLAVE BRIDGE -----------------------------------------
// --------------------------------------------------------------------------------------------------------
class ModbusSlaveBridge : public ModbusSlaveBase
{ 
public:
    ModbusSlaveBridge(ModbusInterface* device);
     
public: // base interface
    virtual bool slaveCheck(uint8_t &slave) const;
    
public:
    inline ModbusInterface* device() const { return m_device; }
    Modbus::Response exec();
     
private:
    ModbusInterface* m_device;
    uint8_t m_memSlave;
    uint8_t m_memFunc;
    uint16_t m_memOffset;
    uint16_t m_memCount;
    uint8_t m_memBuff[MBSLAVEBRIDGE_BUFF_SZ];
};

#endif // MODBUSSLAVEBRIDGE_H
