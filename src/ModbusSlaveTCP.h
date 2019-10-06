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

#ifndef MODBUSSLAVEMEMORYTCP_H
#define MODBUSSLAVEMEMORYTCP_H

#include "ModbusSlave.h"
#include "ModbusSlaveIOTCP.h"

// --------------------------------------------------------------------------------------------------------
// ---------------------------------------- MODBUS SLAVE MEMORY TCP ---------------------------------------
// --------------------------------------------------------------------------------------------------------

class ModbusSlaveTCP : public ModbusSlave, public virtual ModbusSlaveIOTCP
{
public:
    ModbusSlaveTCP(ModbusInterface* memory, uint16_t port = Modbus::STANDARD_TCP_PORT) : ModbusSlave(memory), ModbusSlaveIOTCP(port) {}
};

#endif // MODBUSSLAVEMEMORYTCP_H
