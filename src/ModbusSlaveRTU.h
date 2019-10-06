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

#ifndef MODBUSSLAVEMEMORYRTU_H
#define MODBUSSLAVEMEMORYRTU_H

#include "ModbusSlave.h"
#include "ModbusSlaveIORTU.h"

// --------------------------------------------------------------------------------------------------------
// ---------------------------------------- MODBUS SLAVE MEMORY RTU ---------------------------------------
// --------------------------------------------------------------------------------------------------------

class ModbusSlaveRTU : public ModbusSlave, public ModbusSlaveIORTU
{
public:
    ModbusSlaveRTU(ModbusInterface* memory, Stream* stream) : ModbusSlave(memory), ModbusSlaveIORTU(stream) {}
};

#endif // MODBUSSLAVEMEMORYRTU_H
