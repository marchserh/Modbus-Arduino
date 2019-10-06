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

#include "ModbusMasterRTU.h"

#include <Arduino.h>
#include <Stream.h>

// --------------------------------------------------------------------------------------------------------
// ------------------------------------------ MODBUS MASTER RTU -------------------------------------------
// --------------------------------------------------------------------------------------------------------

// shift for high-level function (e.g. readCoilStatus etc) = 1 byte(slave)+1 byte(function)
static const uint16_t c_HiLevBuffOffset = 2;

// difference between rtu and high-level buffer size (sizeof(uint16_t) - crc size)
static const uint16_t c_HiLevBuffSzDiff = c_HiLevBuffOffset+sizeof(uint16_t);

// high level buffer size
static const uint16_t c_HiLevBuffSz = MB_RTU_IO_BUFF_SZ-c_HiLevBuffSzDiff; 

ModbusMasterRTU::ModbusMasterRTU(Stream* stream) :
    ModbusMaster(),
    m_stream(stream)
{
    m_timeoutFB = 5000;
    m_timeoutIB = 20;
}

uint16_t ModbusMasterRTU::bufferSize() const
{
    return MB_RTU_IO_BUFF_SZ;
}

uint8_t ModbusMasterRTU::bufferByteAt(uint16_t offset) const
{
    return m_buff[c_HiLevBuffOffset+offset];
}

void ModbusMasterRTU::getBufferBytesAt(uint16_t offset, void *buff, uint16_t count) const
{
    memcpy(buff, &m_buff[c_HiLevBuffOffset+offset], count);
}

void ModbusMasterRTU::setBufferByteAt(uint16_t offset, uint8_t value)
{
    m_buff[c_HiLevBuffOffset+offset] = value;
}

void ModbusMasterRTU::setBufferBytesAt(uint16_t offset, const void *buff, uint16_t count)
{
    memcpy(&m_buff[c_HiLevBuffOffset+offset], buff, count);
}

Modbus::Response ModbusMasterRTU::exec(uint8_t &slave, uint8_t func, uint16_t szInBuff, uint16_t* szOutBuff)
{
    uint16_t crc;
    bool fRepeatAgain;

    do
    {
        fRepeatAgain = false;
        switch (m_state)
        {
        case STATE_WRITE:
            // make final requst RTU frame
            if (szInBuff > c_HiLevBuffSz)
                return Modbus::CMN_ERR_WRITE_BUFF_OVERFLOW;
            m_slave = slave;
            m_func = func;
            m_buff[0] = slave;
            m_buff[1] = func;
            m_sz = szInBuff + c_HiLevBuffOffset;
            crc = Modbus::crc16(m_buff, m_sz);
            m_buff[m_sz] = reinterpret_cast<uint8_t*>(&crc)[0];
            m_buff[m_sz+1] = reinterpret_cast<uint8_t*>(&crc)[1];
            m_sz += sizeof(crc);
            m_state = STATE_WAIT_FOR_WRITE;
            // no need break
        case STATE_WAIT_FOR_WRITE:            
        case STATE_WAIT_FOR_WRITE_ALL:
            // clean read buffer from garbage before write
            while (m_stream->available())
                m_stream->read();
            // send data to slave
            if (!m_stream->write(m_buff, m_sz))
            {
                m_state = STATE_BEGIN_WRITE;
                return Modbus::SERIAL_ERR_WRITE;
            }
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
            m_start = millis();
            // go to the read first bytes state
            m_state = STATE_WAIT_FOR_READ;
            break;
        case STATE_WAIT_FOR_READ:
            // read first byte state
            if (m_stream->available()) // first byte read
            {
                for (m_sz = 0; m_stream->available() && (m_sz < MB_RTU_IO_BUFF_SZ); m_sz++)
                    m_buff[m_sz] = m_stream->read();
                if ((m_sz >= MB_RTU_IO_BUFF_SZ) && m_stream->available())
                {
                    // clean read buffer
                    while (m_stream->available())
                        m_stream->read();
                    // and jump to STATE_BEGIN_WRITE
                    m_state = STATE_BEGIN_WRITE;
                    return Modbus::CMN_ERR_READ_BUFF_OVERFLOW;
                }
                m_start = millis();
                m_state = STATE_WAIT_FOR_READ_ALL;
            }
            else if (millis()-m_start >= m_timeoutFB) // waiting timeout read first byte elapsed
            {
                m_state = STATE_BEGIN_WRITE;
                return Modbus::SERIAL_ERR_READ;
            }
            break;
        case STATE_WAIT_FOR_READ_ALL:
            // read all bytes state until interbyte timeout elapsed
            if (m_stream->available())
            {
                for (; m_stream->available() && (m_sz < MB_RTU_IO_BUFF_SZ); m_sz++)
                    m_buff[m_sz] = m_stream->read();
                if ((m_sz >= MB_RTU_IO_BUFF_SZ) && m_stream->available())
                {
                    // clean read buffer
                    while (m_stream->available())
                        m_stream->read();
                    // and jump to STATE_BEGIN_WRITE
                    m_state = STATE_BEGIN_WRITE;
                    return Modbus::CMN_ERR_READ_BUFF_OVERFLOW;
                }
                m_start = millis();
            }
            else if (millis()-m_start >= m_timeoutIB) // waiting timeout elapsed - means that all data read
            {
                m_state = STATE_READ;
                fRepeatAgain = true;
            }
            break;
        case STATE_BEGIN_READ:
        case STATE_READ:
            // state start processing incoming data
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
            m_state = STATE_BEGIN_WRITE;
            if (m_sz < 5)
                return Modbus::CMN_ERR_NOT_CORRECT; // Not correct response. Responsed data length to small
        
            crc = m_buff[m_sz-2] | (m_buff[m_sz-1] << 8);
            if (Modbus::crc16(m_buff, m_sz-2) != crc)
                return Modbus::RTU_ERR_CRC; // Wrong CRC
        
            if (m_slave && (m_buff[0] != m_slave))
                return Modbus::CMN_ERR_NOT_CORRECT; // Not correct response. Requested slave address is not equal to responsed
            slave = m_buff[0];
            
            if ((m_buff[1] & MBF_EXCEPTION) == MBF_EXCEPTION)
                return m_buff[2] ? m_buff[2] : Modbus::UNKNOWN_ERROR; // Returned modbus exception
        
            if (m_buff[1] != m_func)
                return Modbus::CMN_ERR_NOT_CORRECT; // Not correct response. Requested function is not equal to responsed
        
            *szOutBuff = m_sz - c_HiLevBuffSzDiff;
            return Modbus::OK;
        default:
            m_state = STATE_WRITE;
            fRepeatAgain = true;
            break;
        }
    }
    while (fRepeatAgain);
    return Modbus::PROCESSING;
}
