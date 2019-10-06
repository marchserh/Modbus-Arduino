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

#include "ModbusMasterTCP.h"

#include <string.h>
#include <limits.h>

#include <Arduino.h>
#include <Ethernet.h>
#include <EthernetClient.h>
#include <Dns.h>
#include <utility/socket.h>

#define CLIENT_PORT_MAX  49151 // use this values to avoid conflict with Standart EthernetClient class instances
#define CLIENT_PORT_MIN  CLIENT_PORT_MAX-(MAX_SOCK_NUM-1)

uint16_t ModbusMasterTCP::s_srcport = CLIENT_PORT_MIN;      //Use IANA recommended ephemeral port range 49152-65535

// --------------------------------------------------------------------------------------------------------
// ------------------------------------------ MODBUS MASTER TCP -------------------------------------------
// --------------------------------------------------------------------------------------------------------

// shift for high-level function (e.g. readCoilStatus etc) = 6 bytes(tcp-prefix)+1 byte(slave)+1 byte(function)
static const uint16_t c_HiLevBuffOffset = 8;

// difference between rtu and high-level buffer size (sizeof(uint16_t) - crc size)
static const uint16_t c_HiLevBuffSzDiff = c_HiLevBuffOffset;

// high level buffer size
static const uint16_t c_HiLevBuffSz = MB_TCP_IO_BUFF_SZ-c_HiLevBuffSzDiff; 

ModbusMasterTCP::ModbusMasterTCP(const char* host, uint16_t port) : ModbusMaster()
{
    m_port = port;
    m_transaction = 0;
    m_timeout = 5000;
    m_sock = MAX_SOCK_NUM;
    m_state = STATE_UNKNOWN;
    m_start = 0;
    m_sz = 0;
    m_block = false;
    if (!m_ip.fromString(host))
    {
        DNSClient dns;
        dns.begin(Ethernet.dnsServerIP());
        dns.getHostByName(host, m_ip);
    }
}

Modbus::Response ModbusMasterTCP::connect()
{
    uint8_t s;
    uint32_t ip;
    bool fRepeatAgain;
    
    do
    {
        fRepeatAgain = false;
        switch (m_state)
        {
        case STATE_DISCONNECTED:
            if (isConnected())
            {
                m_state = STATE_CONNECTED;
                return Modbus::OK;
            }

            for (int i = 0; i < MAX_SOCK_NUM; i++)
            {
                s = socketStatus(i);
                if (s == SnSR::CLOSED)
                {
                    m_sock = i;
                    break;
                }
            }
            if (m_sock == MAX_SOCK_NUM)
            {
                return Modbus::TCP_ERR_CONNECT;
            }

            s_srcport++;
            if (s_srcport > CLIENT_PORT_MAX)
                s_srcport = CLIENT_PORT_MIN;          //Use IANA recommended ephemeral port range 49152-65535
            socket(m_sock, SnMR::TCP, s_srcport, 0);
            
            ip = (uint32_t)m_ip;
            ::connect(m_sock, reinterpret_cast<uint8_t*>(&ip), m_port);
            m_start = millis();
            m_state = STATE_WAIT_FOR_CONNECT;
            fRepeatAgain = true;
            break;
        case STATE_WAIT_FOR_CONNECT:
            s = socketStatus(m_sock);
            if (millis()-m_start >= m_timeout)
            {
                close(m_sock);
                m_sock = MAX_SOCK_NUM;
                m_state = STATE_DISCONNECTED;
                return Modbus::TCP_ERR_CONNECT;
            }
            else if (s == SnSR::ESTABLISHED)
            {
                m_state = STATE_CONNECTED;
                return Modbus::OK;
            }
            break;
        default:
            if (!isConnected())
            {
                m_state = STATE_DISCONNECTED;
                fRepeatAgain = true;
                break;
            }
            return Modbus::OK;
        }
    }
    while (fRepeatAgain);
    return Modbus::PROCESSING;
}

Modbus::Response ModbusMasterTCP::disconnect()
{
    uint8_t s;
    bool fRepeatAgain;
    
    if (m_sock == MAX_SOCK_NUM)
        return Modbus::OK;
    
    do
    {
        fRepeatAgain = false;
        switch (m_state)
        {
        case STATE_CONNECTED:
            // attempt to close the connection gracefully (send a FIN to other side)
            ::disconnect(m_sock);
            m_start = millis();
            m_state = STATE_WAIT_FOR_DISCONNECT;
            fRepeatAgain = true;
            break;
            // wait some time for the connection to close
        case STATE_WAIT_FOR_DISCONNECT:
            s = socketStatus(m_sock);
            if (s == SnSR::CLOSED)
            {
                m_state = STATE_DISCONNECTED;
                return Modbus::OK;   
            }
            else if (millis()-m_start >= m_timeout)
            {
                close(m_sock);
                m_state = STATE_DISCONNECTED;
                return Modbus::TCP_ERR_DISCONNECT;
            }
            break;
        default:
            if (isConnected())
            {
                m_state = STATE_CONNECTED;
                fRepeatAgain = true;
                break;
            }
            return Modbus::OK;
        }
    }
    while (fRepeatAgain);
    return Modbus::PROCESSING;
}

bool ModbusMasterTCP::isConnected() const
{
    if (m_sock == MAX_SOCK_NUM)
        return false;

    uint8_t s = socketStatus(m_sock);
    return !(s == SnSR::LISTEN || 
             s == SnSR::CLOSED ||
             s == SnSR::FIN_WAIT ||
            (s == SnSR::CLOSE_WAIT && !available()));
}

uint16_t ModbusMasterTCP::bufferSize() const
{
    return MB_TCP_IO_BUFF_SZ;
}

uint8_t ModbusMasterTCP::bufferByteAt(uint16_t offset) const
{
    return m_buff[c_HiLevBuffOffset+offset];
}

void ModbusMasterTCP::getBufferBytesAt(uint16_t offset, void *buff, uint16_t count) const
{
    memcpy(buff, &m_buff[c_HiLevBuffOffset+offset], count);
}

void ModbusMasterTCP::setBufferByteAt(uint16_t offset, uint8_t value)
{
    m_buff[c_HiLevBuffOffset+offset] = value;
}

void ModbusMasterTCP::setBufferBytesAt(uint16_t offset, const void *buff, uint16_t count)
{
    memcpy(&m_buff[c_HiLevBuffOffset+offset], buff, count);
}

Modbus::Response ModbusMasterTCP::exec(uint8_t &slave, uint8_t func, uint16_t szInBuff, uint16_t* szOutBuff)
{    
    int r;
    bool fRepeatAgain;
    
    do
    {
        fRepeatAgain = false;
        switch (m_state)
        {
        case STATE_DISCONNECTED:
        case STATE_WAIT_FOR_CONNECT:
            writeBuffer(slave, func, szInBuff); // remember data in buffer
            r = connect();
            if (r != Modbus::OK) // if not OK it's mean that an error occured or in process
            {
                if (r > Modbus::OK) // an error occured
                    deblockBuffer(); // mark the buffer is free to store new data
                return r;
            }
            fRepeatAgain = true;
            break;
        case STATE_WAIT_FOR_DISCONNECT:
            writeBuffer(slave, func, szInBuff); // remember data in buffer
            r = disconnect();
            if (r) // if not OK it's mean that an error occured or in process
                return r;
            fRepeatAgain = true;
            break;
        case STATE_CONNECTED:
        case STATE_WRITE:
            writeBuffer(slave, func, szInBuff); // remember data in buffer
            // send data to server
            if (!isConnected())
            {
                r = connect();
                if (r != Modbus::OK) // if not OK it's mean that an error occured or in process
                {
                    if (r > Modbus::OK) // an error occured
                        deblockBuffer(); // mark the buffer is free to store new data
                    return r;
                }
            }
            r = write();
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
            if (r != Modbus::OK)
            {
                deblockBuffer();
                return r;
            }
            m_state = STATE_WAIT_FOR_WRITE;
            // no need break
        case STATE_WAIT_FOR_WRITE:
            m_start = millis();
            m_state = STATE_WAIT_FOR_READ;
            fRepeatAgain = true;
            break;
        case STATE_WAIT_FOR_READ:
            // receive data from server
            if (available())
            {
                for (m_sz = 0; available() && (m_sz < MB_TCP_IO_BUFF_SZ); m_sz++)
                    m_buff[m_sz] = read();
                if ((m_sz >= MB_TCP_IO_BUFF_SZ) && available())
                {
                    // clean read buffer
                    while (read() == -1)
                        ;
                    // and jump to STATE_BEGIN_WRITE
                    m_state = STATE_BEGIN_WRITE;
                    return Modbus::CMN_ERR_READ_BUFF_OVERFLOW;
                }
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
                return readBuffer(slave, szOutBuff);
            }
            else if (millis()-m_start >= m_timeout)
            {
                disconnect();
                deblockBuffer(); // mark the buffer is free to store new data
                return Modbus::TCP_ERR_RECV;
            }
            break;
        default:
            if (isConnected())
                m_state = STATE_CONNECTED;
            else
                m_state = STATE_DISCONNECTED;
            fRepeatAgain = true;
            break;
        }
    }
    while (fRepeatAgain);
    return Modbus::PROCESSING;
}

Modbus::Response ModbusMasterTCP::writeBuffer(uint8_t slave, uint8_t func, uint16_t szInBuff)
{
    if (m_block)
        return Modbus::PROCESSING;
    if (szInBuff > c_HiLevBuffSz)
        return Modbus::CMN_ERR_WRITE_BUFF_OVERFLOW;
    // standart TCP message prefix
    m_transaction++;
    m_slave = slave;
    m_func = func;
    // standart TCP message prefix
    m_buff[0] = static_cast<uint8_t>(m_transaction >> 8);  // transaction id
    m_buff[1] = static_cast<uint8_t>(m_transaction);       // transaction id
    m_buff[2] = 0;
    m_buff[3] = 0;
    m_buff[4] = 0;
    m_buff[5] = static_cast<uint8_t>(szInBuff + 2); // quantity of next bytes
    // slave, function, data
    m_buff[6] = slave;
    m_buff[7] = func;
    m_sz = szInBuff + c_HiLevBuffSzDiff;
    m_block = true;
    return Modbus::OK;
}

Modbus::Response ModbusMasterTCP::readBuffer(uint8_t &slave, uint16_t* szOutBuff)
{
    uint16_t transaction;
    
    m_block = false;
    if (m_sz < 9) // minimum size of message 9 = 6 byte(tcp-prefix)+1 byte(slave)+1 byte(func)+1 byte(data-may be err code)
        return Modbus::CMN_ERR_NOT_CORRECT; // Not correct response. Responsed data length to small

    transaction = m_buff[1] | (m_buff[0]<<8);
    if (!((m_transaction == transaction) && (m_buff[2] == 0) && (m_buff[3] == 0) && (m_buff[4] == 0)))
        return Modbus::CMN_ERR_NOT_CORRECT; // Not correct response. Request header is not equal to response header

    if ((m_buff[7] & MBF_EXCEPTION) == MBF_EXCEPTION)
        return m_buff[8] ? m_buff[8] : Modbus::UNKNOWN_ERROR; // Returned modbus exception

    if (m_slave && (m_buff[6] != m_slave))
        return Modbus::CMN_ERR_NOT_CORRECT; // Not correct response. Requested unit (slave) is not equal to responsed
    slave = m_buff[6];
    
    if (m_buff[7] != m_func)
        return Modbus::CMN_ERR_NOT_CORRECT; // Not correct response. Requested function is not equal to responsed

    *szOutBuff = m_sz - c_HiLevBuffSzDiff;
    return Modbus::OK;
}

Modbus::Response ModbusMasterTCP::write()
{
    if (m_sock == MAX_SOCK_NUM)
        return Modbus::TCP_ERR_SEND;
    if (!send(m_sock, m_buff, m_sz))
        return Modbus::TCP_ERR_SEND;
    return Modbus::OK;
}

int ModbusMasterTCP::available() const
{
    if (m_sock != MAX_SOCK_NUM)
        return recvAvailable(m_sock);
    return 0;
}

int ModbusMasterTCP::read()
{
    uint8_t b;
    if (recv(m_sock, &b, 1) > 0)
        return b;
    return -1;
}
