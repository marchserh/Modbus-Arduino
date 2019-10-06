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

#include "ModbusSlave.h"

#include <Arduino.h>

ModbusSlave::ModbusSlave(ModbusInterface* memory)
{
    m_memory = memory;
    m_state = STATE_UNKNOWN;
    m_slave = Modbus::VALID_MODBUS_ADDRESS_BEGIN;
}

bool ModbusSlave::slaveCheck(uint8_t &slave) const
{
    if (!slave) // if slave is NULL get slave address
    {
        slave = m_slave;
        return true;
    }
    return slave == m_slave;
}

Modbus::Response ModbusSlave::exec()
{
    Modbus::Response r = Modbus::OK;
    uint16_t outBytes, outCount, i, c;
    bool fRepeatAgain;
    
    do
    {
        fRepeatAgain = false;
        switch (m_state)
        {
        case STATE_UNKNOWN:
            r = begin();
            if (r == Modbus::OK)
            {
                m_state = STATE_WAIT_FOR_READ;
                fRepeatAgain = true;
                break;
            }
            return r;
        case STATE_BEGIN_READ:
            m_start = millis();
            m_state = STATE_WAIT_FOR_READ;
            fRepeatAgain = true;
            break;
        case STATE_WAIT_FOR_READ:
        case STATE_WAIT_FOR_READ_ALL:
            r = read(m_memSlave, m_memFunc, outBytes);
            if (r == Modbus::OK)
            {
                m_state = STATE_READ;
                fRepeatAgain = true;
                break;
            }
            else if (r > Modbus::OK) // error occured
                m_state = STATE_BEGIN_READ;
            return r;
        case STATE_READ:
            // verify slave id
            r = Modbus::OK;
            if (!slaveCheck(m_memSlave))
            {
                m_state = STATE_BEGIN_READ;
                return Modbus::PROCESSING; // slave mismatch - do nothing
            }
            // modbus functions
            switch (m_memFunc)
            {
            case MBF_READ_COIL_STATUS:
            case MBF_READ_INPUT_STATUS:
                if (outBytes != 4) // not correct request from master - don't respond
                {
                    m_state = STATE_BEGIN_READ;
                    return Modbus::CMN_ERR_NOT_CORRECT;
                }
                m_memOffset = bufferByteAt(1) | (bufferByteAt(0)<<8);
                m_memCount = bufferByteAt(3) | (bufferByteAt(2)<<8);
                if (m_memCount > MB_MAX_DISCRETS) 
                    m_memCount = MB_MAX_DISCRETS;
                break;
            case MBF_READ_HOLDING_REGISTERS:
            case MBF_READ_INPUT_REGISTERS: // Read input registers
                if (outBytes != 4) // not correct request from master - don't respond
                {
                    m_state = STATE_BEGIN_READ;
                    return Modbus::CMN_ERR_NOT_CORRECT;
                }
                m_memOffset = bufferByteAt(1) | (bufferByteAt(0)<<8);
                m_memCount = bufferByteAt(3) | (bufferByteAt(2)<<8);
                if (m_memCount > MB_MAX_REGISTERS) // prevent valueBuff overflow 
                    m_memCount = MB_MAX_REGISTERS;
                break;
            case MBF_FORCE_SINGLE_COIL:
                if (outBytes != 4) // not correct request from master - don't respond
                {
                    m_state = STATE_BEGIN_READ;
                    return Modbus::CMN_ERR_NOT_CORRECT;
                }
                if (!(bufferByteAt(2) == 0x00 || bufferByteAt(2) == 0xFF) || (bufferByteAt(3) != 0))  // not correct request from master - don't respond
                {
                    m_state = STATE_BEGIN_READ;
                    return Modbus::CMN_ERR_NOT_CORRECT;
                }
                m_memOffset = bufferByteAt(1) | (bufferByteAt(0)<<8);
                break;
            case MBF_FORCE_SINGLE_REGISTER:
                if (outBytes != 4) // not correct request from master - don't respond
                {
                    m_state = STATE_BEGIN_READ;
                    return Modbus::CMN_ERR_NOT_CORRECT;
                }
                m_memOffset = bufferByteAt(1) | (bufferByteAt(0)<<8);
                break;             
            case MBF_FORCE_MULTIPLE_COILS:
                if (outBytes < 5) // not correct request from master - don't respond
                {
                    m_state = STATE_BEGIN_READ;
                    return Modbus::CMN_ERR_NOT_CORRECT;
                }
                if (outBytes != bufferByteAt(4)+5) // don't match readed bytes and number of data bytes to follow
                {
                    m_state = STATE_BEGIN_READ;
                    return Modbus::CMN_ERR_NOT_CORRECT;
                }
                m_memOffset = bufferByteAt(1) | (bufferByteAt(0)<<8);
                m_memCount = bufferByteAt(3) | (bufferByteAt(2)<<8);
                if ((m_memCount+7)/8 != bufferByteAt(4)) // don't match count bites and bytes
                {
                    m_state = STATE_BEGIN_READ;
                    return Modbus::CMN_ERR_NOT_CORRECT;
                }
                if (m_memCount > MB_MAX_DISCRETS) // prevent valueBuff overflow 
                    m_memCount = MB_MAX_DISCRETS;
                break;
            case MBF_FORCE_MULTIPLE_REGISTERS: // Write multiple registers
                if (outBytes < 5) // not correct request from master - don't respond
                {
                    m_state = STATE_BEGIN_READ;
                    return Modbus::CMN_ERR_NOT_CORRECT;
                }
                if (outBytes != bufferByteAt(4)+5) // don't match readed bytes and number of data bytes to follow
                {
                    m_state = STATE_BEGIN_READ;
                    return Modbus::CMN_ERR_NOT_CORRECT;
                }
                m_memOffset = bufferByteAt(1) | (bufferByteAt(0)<<8);
                m_memCount = bufferByteAt(3) | (bufferByteAt(2)<<8);
                if (m_memCount*2 != bufferByteAt(4)) // don't match count values and bytes
                {
                    m_state = STATE_BEGIN_READ;
                    return Modbus::CMN_ERR_NOT_CORRECT;
                }
                if (m_memCount > MB_MAX_REGISTERS) // prevent memBuff overflow 
                    m_memCount = MB_MAX_REGISTERS; 
                break;
            default:
                r = Modbus::ILLEGAL_FUNCTION;
                break;
            }
            if (r > Modbus::OK) // if error occured put it immidiatly to master(client)
                m_state = STATE_WRITE;                   
            else 
                m_state = STATE_PROCESS_DEVICE;
            fRepeatAgain = true;
            break;
        case STATE_PROCESS_DEVICE:
            switch (m_memFunc)
            {
            case MBF_READ_COIL_STATUS:
                outCount = 0;
                c = (m_memCount+7/8); // count bytes needed
                c = (c+MBSLAVEMEM_BUFF_SZ_BYTES-1)/MBSLAVEMEM_BUFF_SZ_BYTES;
                for (i = 0; i < c; i++)
                {
                    uint16_t cn = m_memCount >= (i+1)*MBSLAVEMEM_BUFF_SZ_BITES ? MBSLAVEMEM_BUFF_SZ_BITES : m_memCount%MBSLAVEMEM_BUFF_SZ_BITES;
                    r = m_memory->readCoilStatus(m_memSlave, m_memOffset+i*MBSLAVEMEM_BUFF_SZ_BITES, cn, m_memBuff, &cn);
                    if (r)
                    {
                        // when returns illegal address not in first cycle - it's normal
                        if (r == Modbus::ILLEGAL_DATA_ADDRESS && i > 0)
                            r = Modbus::OK;
                        break;
                    }
                    setBufferBytesAt(1+i*MBSLAVEMEM_BUFF_SZ_BYTES, m_memBuff, (cn+7)/8);
                    outCount += cn;
                } 
                break;
            case MBF_READ_INPUT_STATUS:
                outCount = 0;
                c = (m_memCount+7/8); // count bytes needed
                c = (c+MBSLAVEMEM_BUFF_SZ_BYTES-1)/MBSLAVEMEM_BUFF_SZ_BYTES; // count cycles
                for (i = 0; i < c; i++)
                {
                    uint16_t cn = m_memCount >= (i+1)*MBSLAVEMEM_BUFF_SZ_BITES ? MBSLAVEMEM_BUFF_SZ_BITES : m_memCount%MBSLAVEMEM_BUFF_SZ_BITES;
                    r = m_memory->readInputStatus(m_memSlave, m_memOffset+i*MBSLAVEMEM_BUFF_SZ_BITES, cn, m_memBuff, &cn);
                    if (r)
                    {
                        // when returns illegal address not in first cycle - it's normal
                        if (r == Modbus::ILLEGAL_DATA_ADDRESS && i > 0)
                            r = Modbus::OK;
                        break;
                    }
                    setBufferBytesAt(1+i*MBSLAVEMEM_BUFF_SZ_BYTES, m_memBuff, (cn+7)/8);
                    outCount += cn;
                } 
                break;
            case MBF_READ_HOLDING_REGISTERS:
                outCount = 0;
                c = (m_memCount+MBSLAVEMEM_BUFF_SZ_REGES-1)/MBSLAVEMEM_BUFF_SZ_REGES; // count cycles
                for (i = 0; i < c; i++)
                {
                    uint16_t cn = m_memCount >= (i+1)*MBSLAVEMEM_BUFF_SZ_REGES ? MBSLAVEMEM_BUFF_SZ_REGES : m_memCount%MBSLAVEMEM_BUFF_SZ_REGES;
                    r = m_memory->readHoldingRegisters(m_memSlave, m_memOffset+i*MBSLAVEMEM_BUFF_SZ_REGES, cn, reinterpret_cast<uint16_t*>(m_memBuff), &cn);
                    if (r)
                    {
                        // when returns illegal address not in first cycle - it's normal
                        if (r == Modbus::ILLEGAL_DATA_ADDRESS && i > 0)
                            r = Modbus::OK;
                        break;
                    }
                    for (int j = 1; j < MBSLAVEMEM_BUFF_SZ_BYTES; j+=2)
                    {
                        // exchange values of near bytes
                        m_memBuff[j] = m_memBuff[j-1]^m_memBuff[j];
                        m_memBuff[j-1] = m_memBuff[j-1]^m_memBuff[j];
                        m_memBuff[j] = m_memBuff[j-1]^m_memBuff[j];
                    }
                    setBufferBytesAt(1+i*MBSLAVEMEM_BUFF_SZ_REGES, m_memBuff, cn*2);
                    outCount += cn;
                } 
                break;
            case MBF_READ_INPUT_REGISTERS:
                outCount = 0;
                c = (m_memCount+MBSLAVEMEM_BUFF_SZ_REGES-1)/MBSLAVEMEM_BUFF_SZ_REGES; // count cycles
                for (i = 0; i < c; i++)
                {
                    uint16_t cn = m_memCount >= (i+1)*MBSLAVEMEM_BUFF_SZ_REGES ? MBSLAVEMEM_BUFF_SZ_REGES : m_memCount%MBSLAVEMEM_BUFF_SZ_REGES;
                    r = m_memory->readInputRegisters(m_memSlave, m_memOffset+i*MBSLAVEMEM_BUFF_SZ_REGES, cn, reinterpret_cast<uint16_t*>(m_memBuff), &cn);
                    if (r)
                    {
                        // when returns illegal address not in first cycle - it's normal
                        if (r == Modbus::ILLEGAL_DATA_ADDRESS && i > 0)
                            r = Modbus::OK;
                        break;
                    }
                    for (int j = 1; j < MBSLAVEMEM_BUFF_SZ_BYTES; j+=2)
                    {
                        // exchange values of near bytes
                        m_memBuff[j] = m_memBuff[j-1]^m_memBuff[j];
                        m_memBuff[j-1] = m_memBuff[j-1]^m_memBuff[j];
                        m_memBuff[j] = m_memBuff[j-1]^m_memBuff[j];
                    }
                    setBufferBytesAt(1+i*MBSLAVEMEM_BUFF_SZ_REGES, m_memBuff, cn*2);
                    outCount += cn;
                } 
                break;
            case MBF_FORCE_SINGLE_COIL:
                m_memBuff[0] = bufferByteAt(2);
                r = m_memory->forceSingleCoil(m_memSlave, m_memOffset, m_memBuff[0]);
                break;
            case MBF_FORCE_SINGLE_REGISTER:
                m_memBuff[0] = bufferByteAt(3);
                m_memBuff[1] = bufferByteAt(2);
                r = m_memory->forceSingleRegister(m_memSlave, m_memOffset, reinterpret_cast<uint16_t*>(m_memBuff)[0]);
                break;          
            case MBF_FORCE_MULTIPLE_COILS:
                outCount = 0;
                c = (m_memCount+7/8); // count bytes needed
                c = (c+MBSLAVEMEM_BUFF_SZ_BYTES-1)/MBSLAVEMEM_BUFF_SZ_BYTES;
                for (i = 0; i < c; i++)
                {
                    uint16_t cn = m_memCount >= (i+1)*MBSLAVEMEM_BUFF_SZ_BITES ? MBSLAVEMEM_BUFF_SZ_BITES : m_memCount%MBSLAVEMEM_BUFF_SZ_BITES;
                    getBufferBytesAt(5+i*MBSLAVEMEM_BUFF_SZ_BYTES, m_memBuff, cn);
                    r = m_memory->forceMultipleCoils(m_memSlave, m_memOffset+i*MBSLAVEMEM_BUFF_SZ_BITES, cn, m_memBuff, &cn);
                    if (r)
                    {
                        // when returns illegal address not in first cycle - it's normal
                        if (r == Modbus::ILLEGAL_DATA_ADDRESS && i > 0)
                            r = Modbus::OK;
                        break;
                    }
                    outCount += cn;
                } 
                break;
            case MBF_FORCE_MULTIPLE_REGISTERS:
                outCount = 0;
                c = (m_memCount+MBSLAVEMEM_BUFF_SZ_REGES-1)/MBSLAVEMEM_BUFF_SZ_REGES; // count cycles
                for (i = 0; i < c; i++)
                {
                    uint16_t cn = m_memCount >= (i+1)*MBSLAVEMEM_BUFF_SZ_REGES ? MBSLAVEMEM_BUFF_SZ_REGES : m_memCount%MBSLAVEMEM_BUFF_SZ_REGES;
                    getBufferBytesAt(5+i*MBSLAVEMEM_BUFF_SZ_BYTES, m_memBuff, cn*2);
                    for (int j = 1; j < MBSLAVEMEM_BUFF_SZ_BYTES; j+=2)
                    {
                        // exchange values of near bytes
                        m_memBuff[j] = m_memBuff[j-1]^m_memBuff[j];
                        m_memBuff[j-1] = m_memBuff[j-1]^m_memBuff[j];
                        m_memBuff[j] = m_memBuff[j-1]^m_memBuff[j];
                    }
                    r = m_memory->forceMultipleRegisters(m_memSlave, m_memOffset+i*MBSLAVEMEM_BUFF_SZ_REGES, cn, reinterpret_cast<uint16_t*>(m_memBuff), &cn);
                    if (r)
                    {
                        // when returns illegal address not in first cycle - it's normal
                        if (r == Modbus::ILLEGAL_DATA_ADDRESS && i > 0)
                            r = Modbus::OK;
                        break;
                    }
                    outCount += cn;
                } 
                break;
            }
            if (r < Modbus::OK) // processing
                break;
            m_state = STATE_WRITE;
            // no need break
        case STATE_WRITE:
            if (r > Modbus::OK)
            {
                m_memFunc |= MBF_EXCEPTION;
                setBufferByteAt(0, static_cast<uint8_t>(r));
                outCount = 1;
            }
            else if (r == Modbus::OK)
            {
                switch (m_memFunc)
                {
                case MBF_READ_COIL_STATUS:
                case MBF_READ_INPUT_STATUS:
                    outCount = (outCount+7)/8;
                    setBufferByteAt(0, static_cast<uint8_t>(outCount)); // count next bytes
                    outCount += 1;
                    break;
                case MBF_READ_HOLDING_REGISTERS:
                case MBF_READ_INPUT_REGISTERS:
                    outCount = outCount*2;
                    setBufferByteAt(0, static_cast<uint8_t>(outCount)); // count next bytes
                    outCount += 1;
                    break;
                case MBF_FORCE_SINGLE_COIL:
                    setBufferByteAt(0, static_cast<uint8_t>(m_memOffset>>8));   // address of coil (Hi-byte)
                    setBufferByteAt(1, static_cast<uint8_t>(m_memOffset&0xFF)); // address of coil (Lo-byte)
                    setBufferByteAt(2, m_memBuff[0] ? 0xFF : 0x00);             // value (Hi-byte)
                    setBufferByteAt(3, 0);                                      // value (Lo-byte)
                    outCount = 4;
                    break;
                case MBF_FORCE_SINGLE_REGISTER:
                    setBufferByteAt(0, static_cast<uint8_t>(m_memOffset>>8));   // address of register (Hi-byte)
                    setBufferByteAt(1, static_cast<uint8_t>(m_memOffset&0xFF)); // address of register (Lo-byte)
                    setBufferByteAt(2, m_memBuff[1]);                           // value (Hi-byte)
                    setBufferByteAt(3, m_memBuff[0]);                           // value (Lo-byte)
                    outCount = 4;
                    break;          
                case MBF_FORCE_MULTIPLE_COILS:
                case MBF_FORCE_MULTIPLE_REGISTERS:
                    setBufferByteAt(0, static_cast<uint8_t>(m_memOffset>>8));   // address of written values (Hi-byte)
                    setBufferByteAt(1, static_cast<uint8_t>(m_memOffset&0xFF)); // address of written values (Lo-byte)
                    setBufferByteAt(2, static_cast<uint8_t>(outCount>>8));      // count of written values (Hi-byte)
                    setBufferByteAt(3, static_cast<uint8_t>(outCount&0xFF));    // count of written values (Lo-byte)
                    outCount = 4;
                    break;
                }
            }
            m_state = STATE_BEGIN_WRITE;
            // no need break
        case STATE_BEGIN_WRITE:
            m_start = millis();
            m_state = STATE_WAIT_FOR_WRITE;
            fRepeatAgain = true;
            break;
        case STATE_WAIT_FOR_WRITE:
        case STATE_WAIT_FOR_WRITE_ALL:
            r = write(m_memSlave, m_memFunc, outCount);
            if (r >= Modbus::OK)
                m_state = STATE_BEGIN_READ;
            return r;
        }
    }
    while (fRepeatAgain);
    return Modbus::PROCESSING;
}
 
