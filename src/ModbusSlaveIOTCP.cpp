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

#include "ModbusSlaveIOTCP.h"

#include <Arduino.h>
#include <Ethernet.h>
#include <EthernetClient.h>
#include <Dns.h>
#include <utility/socket.h>

// --------------------------------------------------------------------------------------------------------
// ------------------------------------------ MODBUS SLAVE IO TCP -----------------------------------------
// --------------------------------------------------------------------------------------------------------

// shift for high-level function (e.g. readCoilStatus etc) = 6 bytes(tcp-prefix)+1 byte(slave)+1 byte(function)
static const uint16_t c_HiLevBuffOffset = 8;

// difference between rtu and high-level buffer size
static const uint16_t c_HiLevBuffSzDiff = c_HiLevBuffOffset;

// high level buffer size
static const uint16_t c_HiLevBuffSz = MB_TCP_IO_BUFF_SZ-c_HiLevBuffSzDiff; 

#define MBSLAVE_TCP_DEFAULT_TIMEOUT_REQUEST_ms 10000

ModbusSlaveIOTCP::ModbusSlaveIOTCP(uint16_t port) : ModbusSlaveIO()
{
    m_port = port;
    m_sock = MAX_SOCK_NUM;
    m_timeoutRequest = MBSLAVE_TCP_DEFAULT_TIMEOUT_REQUEST_ms;
    m_startRequest = 0;  
}

uint8_t ModbusSlaveIOTCP::sockStatus()
{
    return socketStatus(m_sock);
}

uint16_t ModbusSlaveIOTCP::bufferSize() const
{
    return MB_TCP_IO_BUFF_SZ;
}

uint8_t ModbusSlaveIOTCP::bufferByteAt(uint16_t offset) const
{
    return m_buff[c_HiLevBuffOffset+offset];
}

void ModbusSlaveIOTCP::getBufferBytesAt(uint16_t offset, void *buff, uint16_t count) const
{
    memcpy(buff, &m_buff[c_HiLevBuffOffset+offset], count);
}

void ModbusSlaveIOTCP::setBufferByteAt(uint16_t offset, uint8_t value)
{
    m_buff[c_HiLevBuffOffset+offset] = value;
}

void ModbusSlaveIOTCP::setBufferBytesAt(uint16_t offset, const void *buff, uint16_t count)
{
    memcpy(&m_buff[c_HiLevBuffOffset+offset], buff, count);
}

Modbus::Response ModbusSlaveIOTCP::begin()
{
    uint8_t sock, s;
    
    if (m_sock == MAX_SOCK_NUM)
    {
        for (sock = 0; sock < MAX_SOCK_NUM; sock++)
        {
            s = socketStatus(sock);
            if (s == SnSR::CLOSED)
            {
                m_sock = sock;
                socket(m_sock, SnMR::TCP, m_port, 0);
                listen(m_sock);
                m_startRequest = millis();
                break;
            }
        }
        if (m_sock == MAX_SOCK_NUM)
            return Modbus::TCP_ERR_SERVER;
    }
    return Modbus::OK;
}

Modbus::Response ModbusSlaveIOTCP::read(uint8_t &slave, uint8_t &func, uint16_t &szBuff)
{
    uint16_t c;
    uint8_t b;
    
    if (!checkConnection())
        return Modbus::TCP_ERR_RECV;
    if (recvAvailable(m_sock))
    { 
        for (c = 0; recvAvailable(m_sock) && c < MB_TCP_IO_BUFF_SZ; c++)
        {
            recv(m_sock, &b, 1);
            m_buff[c] = b;
        }
        if ((c >= MB_TCP_IO_BUFF_SZ) && recvAvailable(m_sock))
        {
            // clean buffer
            while (recvAvailable(m_sock))
                recv(m_sock, &b, 1);
            return Modbus::CMN_ERR_READ_BUFF_OVERFLOW;
        }
        m_startRequest = millis();
        if (m_verboseStream)
        {
            if (m_name)
            {
                m_verboseStream->print(m_name);
                m_verboseStream->print(' ');
            }
            m_verboseStream->print("Rx: ");
            Modbus::printBytes(m_verboseStream, m_buff, c);
        }
        if (c < 8) // minimum size of message 8 = 6 byte(tcp-prefix)+1 byte(slave)+1 byte(func)
            return Modbus::CMN_ERR_NOT_CORRECT;
        if (!((m_buff[2] == 0) && (m_buff[3] == 0) && (m_buff[4] == 0)))
            return Modbus::CMN_ERR_NOT_CORRECT; // Not correct response. Request header is not equal to response header
        m_transaction = m_buff[1] | (m_buff[0]<<8);
        slave = m_buff[6];
        func = m_buff[7];
        szBuff = c - c_HiLevBuffSzDiff;
        return Modbus::OK;
    }
    return Modbus::PROCESSING;
}

Modbus::Response ModbusSlaveIOTCP::write(uint8_t slave, uint8_t func, uint16_t szBuff)
{
    if (szBuff > c_HiLevBuffSz)
        return Modbus::CMN_ERR_WRITE_BUFF_OVERFLOW;
    // standart TCP message prefix
    m_buff[0] = static_cast<uint8_t>(m_transaction>>8); // transaction id
    m_buff[1] = static_cast<uint8_t>(m_transaction);    // transaction id
    m_buff[2] = 0;
    m_buff[3] = 0;
    m_buff[4] = 0;
    m_buff[5] = static_cast<uint8_t>(szBuff+2); // quantity of next bytes=sz_buffer+slave+func
    m_buff[6] = slave;
    m_buff[7] = func;
    // send data to client
    if (send(m_sock, m_buff, szBuff+c_HiLevBuffSzDiff))
    {
        m_startRequest = millis();
        if (m_verboseStream)
        {
            if (m_name)
            {
                m_verboseStream->print(m_name);
                m_verboseStream->print(' ');
            }
            m_verboseStream->print("Tx: ");
            Modbus::printBytes(m_verboseStream, m_buff, szBuff+c_HiLevBuffSzDiff);
        }
        return Modbus::OK;
    }
    //if (m_verboseStream)
    //    m_verboseStream->println("SLAVE(TCP) Tx: TCP send error");
    return Modbus::TCP_ERR_SEND;  
}

bool ModbusSlaveIOTCP::checkConnection()
{
    uint8_t s = socketStatus(m_sock);
    
    switch (s)
    {
    default:
        if (millis()-m_startRequest < m_timeoutRequest)          
            break;
        m_startRequest = millis(); 
        // no need break
    case SnSR::FIN_WAIT:
    case SnSR::CLOSE_WAIT:
        close(m_sock);
        // no need break
    case SnSR::CLOSED:
        socket(m_sock, SnMR::TCP, m_port, 0);
        listen(m_sock);
        break;
    }
    return true;
}
