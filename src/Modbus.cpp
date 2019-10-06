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

#include "Modbus.h"

#include <Stream.h>

uint16_t Modbus::crc16(const uint8_t* data, uint16_t szData)
{
    uint16_t i, j, temp, CRC = 0xFFFF;
    for (i = 0; i < szData; i++)
    {
        CRC ^= data[i];
        for (j = 0; j < 8; j++)
        {
            temp = CRC & 0x0001;
            CRC >>= 1;
            if (temp) CRC ^= 0xA001;
        }
    }
    return CRC;
}

uint8_t Modbus::lrc(const uint8_t* data, uint16_t szData)
{
    uint8_t LRC = 0x00;
    for(; szData; szData--)
        LRC += *data++;
    return (uint8_t)(-((int8_t)(LRC)));
}

void Modbus::printBytes(Stream *debug, uint8_t *bytes, uint16_t count)
{
    for (uint16_t i = 0; i < count; i++)
    {
        if (bytes[i] > 0x0F)
        {
            // print 2 symbols
            debug->print(bytes[i], HEX);
        }
        else
        {
            // manualy print leading zerro
            debug->print('0');
            debug->print(bytes[i], HEX);
        }
        debug->print(' ');
    }
    debug->print('\n');    
}

