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

#include "ModbusSlaveIORTU.h"

#include <Arduino.h>

// --------------------------------------------------------------------------------------------------------
// ------------------------------------------ MODBUS SLAVE IO RTU -----------------------------------------
// --------------------------------------------------------------------------------------------------------

// shift for high-level function (e.g. readCoilStatus etc) = 1 byte(slave)+1 byte(function)
static const uint16_t c_HiLevBuffOffset = 2;

// difference between rtu and high-level buffer size + 2(sizeof(uint16_t) - crc size)
static const uint16_t c_HiLevBuffSzDiff = c_HiLevBuffOffset+2;

// high level buffer size
static const uint16_t c_HiLevBuffSz = MB_RTU_IO_BUFF_SZ-c_HiLevBuffSzDiff; 

ModbusSlaveIORTU::ModbusSlaveIORTU(Stream* stream) : ModbusSlaveIO()
{ 
    m_stream = stream;
    m_timeoutIB = 20;
}

uint16_t ModbusSlaveIORTU::bufferSize() const
{
    return MB_RTU_IO_BUFF_SZ;
}

uint8_t ModbusSlaveIORTU::bufferByteAt(uint16_t offset) const
{
    return m_buff[c_HiLevBuffOffset+offset];
}

void ModbusSlaveIORTU::getBufferBytesAt(uint16_t offset, void *buff, uint16_t count) const
{
    memcpy(buff, &m_buff[c_HiLevBuffOffset+offset], count);
}

void ModbusSlaveIORTU::setBufferByteAt(uint16_t offset, uint8_t value)
{
    m_buff[c_HiLevBuffOffset+offset] = value;
}

void ModbusSlaveIORTU::setBufferBytesAt(uint16_t offset, const void *buff, uint16_t count)
{
    memcpy(&m_buff[c_HiLevBuffOffset+offset], buff, count);
}

Modbus::Response ModbusSlaveIORTU::begin()
{
    return Modbus::OK;
}
Modbus::Response ModbusSlaveIORTU::read(uint8_t &slave, uint8_t &func, uint16_t &szBuff)
{
    uint16_t crc;
    bool fRepeatAgain;

    do
    {
        fRepeatAgain = false;
        switch (m_state)
        {
        case STATE_WAIT_FOR_READ:
            if (m_stream->available()) // read first byte
            {
                for (m_sz = 0; m_stream->available() && (m_sz < MB_RTU_IO_BUFF_SZ); m_sz++)
                    m_buff[m_sz] = m_stream->read();
                if ((m_sz >= MB_RTU_IO_BUFF_SZ) && m_stream->available())
                    return Modbus::CMN_ERR_READ_BUFF_OVERFLOW;
                m_start = millis();
                m_state = STATE_WAIT_FOR_READ_ALL;
            }
            break;
        case STATE_WAIT_FOR_READ_ALL:
            if (m_stream->available()) // read next bytes
            {
                for (; m_stream->available() && (m_sz < MB_RTU_IO_BUFF_SZ); m_sz++)
                    m_buff[m_sz] = m_stream->read();
                if ((m_sz >= MB_RTU_IO_BUFF_SZ) && m_stream->available())
                {
                    // clean read buffer
                    while (m_stream->available())
                        m_stream->read();
                    return Modbus::CMN_ERR_READ_BUFF_OVERFLOW;
                }
                m_start = millis();
                break;
            }
            else if (millis()-m_start >= m_timeoutIB) // waiting timeout elapsed - means that all data read
            {
                ; // go to input data processing
            }
            else
                break;
            // input data processing
            if (m_verboseStream)
            {
                if (m_name)
                {
                    m_verboseStream->print(m_name);
                    m_verboseStream->print(' ');
                }
                m_verboseStream->print("Rx: ");
                Modbus::printBytes(m_verboseStream, m_buff, m_sz);
            }
            if (m_sz < 4) // minimum size of message 4 = 1 byte(slave)+1 byte(func)+2 byte(CRC)
                return Modbus::CMN_ERR_NOT_CORRECT; // Not correct response. Responsed data length to small
        
            crc = m_buff[m_sz-2] | (m_buff[m_sz-1] << 8);
            if (Modbus::crc16(m_buff, m_sz-2) != crc)
                return Modbus::RTU_ERR_CRC; // Wrong crc
            slave = m_buff[0];
            func = m_buff[1];
            szBuff = m_sz - c_HiLevBuffSzDiff;
            return Modbus::OK;
        }
    }
    while (fRepeatAgain);
    return Modbus::PROCESSING;
}

Modbus::Response ModbusSlaveIORTU::write(uint8_t slave, uint8_t func, uint16_t szBuff)
{
    uint16_t crc;
    
    if (szBuff > c_HiLevBuffSz) // crc size in bytes is 2
        return Modbus::CMN_ERR_WRITE_BUFF_OVERFLOW;
    m_buff[0] = slave; 
    m_buff[1] = func;
    m_sz = szBuff+c_HiLevBuffOffset;
    crc = Modbus::crc16(m_buff, m_sz);
    m_buff[m_sz] = reinterpret_cast<uint8_t*>(&crc)[0];
    m_buff[m_sz+1] = reinterpret_cast<uint8_t*>(&crc)[1];
    m_sz += sizeof(crc);
    // put data to serial port to write
    if (m_stream->write(m_buff, m_sz))
    {
        if (m_verboseStream)
        {
            if (m_name)
            {
                m_verboseStream->print(m_name);
                m_verboseStream->print(' ');
            }
            m_verboseStream->print("Tx: ");
            Modbus::printBytes(m_verboseStream, m_buff, m_sz);
        }
        return Modbus::OK;
    }
    //if (m_verboseStream)
    //    m_verboseStream->println("SLAVE(RTU) Tx: Serial error write");
    return Modbus::SERIAL_ERR_WRITE;  
}
