# Modbus Library for Arduino

## Overview

Library provides Modbus functionality (master/slave/bridge) on different physical layers (rtu/tcp) for your Arduino programs.

Use this library to add Modbus communication functionality to your programs.
This library also provide some functions to control modbus memory.

It also contains some examples that shows how you can use this lib.

## Usage

### `Modbus.h`

This is main header file. It defines common constants and functions for library such as:
* version control macros: `
    - `MODBUSLIB_VERSION_MAJOR` - major part of modbuslib version
    - `MODBUSLIB_VERSION_MINOR` - minor part of modbuslib version
    - `MODBUSLIB_VERSION_PATCH` - patch part of modbuslib version
    - `MODBUSLIB_VERSION`       - version in hex 3-byte notation (0.1.2 <=> 0x000102)
    - `MODBUSLIB_VERSION_STR`   - `const char*` c-string version representation like `"major.minor.patch"`
* `MB_NULLPTR` - main constant used to replace `NULL`-pointer or `((void*)0)`-pointer
* Modbus function codes macro constants
```c++
#define MBF_READ_COIL_STATUS                    1
#define MBF_READ_INPUT_STATUS                   2
#define MBF_READ_HOLDING_REGISTERS              3
#define MBF_READ_INPUT_REGISTERS                4
#define MBF_FORCE_SINGLE_COIL                   5
#define MBF_FORCE_SINGLE_REGISTER               6
...
#define MBF_FORCE_MULTIPLE_COILS                15
#define MBF_FORCE_MULTIPLE_REGISTERS            16
```
* `Modbus`-namespace with enumeration types (such as `Modbus::Response` - returned type of ModbusInterface functions),
  helper functions:
  - control bit functions: `Modbus::getBit, Modbus::setBit, Modbus::getBits, Modbus::setBits`,
  - control sum functions: `Modbus::crc16` (for RTU-mode), `Modbus::lrc` (for ASCII-mode).
* `ModbusInterface` base class. 

### `ModbusInterface` base class

```c++
class ModbusInterface
{
public:
    virtual Modbus::Response readCoilStatus(uint8_t &slave, uint16_t offset, uint16_t count, void* bits, uint16_t* fact = MB_NULLPTR) = 0;
    virtual Modbus::Response readInputStatus(uint8_t &slave, uint16_t offset, uint16_t count, void* bits, uint16_t* fact = MB_NULLPTR) = 0;
    virtual Modbus::Response readHoldingRegisters(uint8_t &slave, uint16_t offset, uint16_t count, uint16_t* values, uint16_t* fact = MB_NULLPTR) = 0;
    virtual Modbus::Response readInputRegisters(uint8_t &slave, uint16_t offset, uint16_t count, uint16_t* values, uint16_t* fact = MB_NULLPTR) = 0;
    virtual Modbus::Response forceSingleCoil(uint8_t &slave, uint16_t offset, bool value) = 0;
    virtual Modbus::Response forceSingleRegister(uint8_t &slave, uint16_t offset, uint16_t value) = 0;
    virtual Modbus::Response forceMultipleCoils(uint8_t &slave, uint16_t offset, uint16_t count, const void* bits, uint16_t* fact = MB_NULLPTR) = 0;
    virtual Modbus::Response forceMultipleRegisters(uint8_t &slave, uint16_t offset, uint16_t count, const uint16_t* values, uint16_t* fact = MB_NULLPTR) = 0;
};
```

It defines an interface for commonly used 8 modbus functions:
* `01 (0x01) readCoilStatus`          - read memory 000001+ (coils/discrete outputs)
* `02 (0x02) readInputStatus`         - read memory 100001+ (discrete inputs)
* `03 (0x03) readHoldingRegisters`    - read memory 400001+ (holding registers/anlaog outputs)
* `04 (0x04) readInputRegisters`      - read memory 300001+ (input registers/analog inputs)
* `05 (0x05) forceSingleCoil`         - write single bit value to memory cell with address 000001+
* `06 (0x06) forceSingleRegister`     - write single 16 bit register's value to memory 400001+
* `15 (0x0F) forceMultipleCoils`      - write bit values to memory 000001+
* `16 (0x10) forceMultipleRegisters`  - write 16 bit register's values to memory 400001+
      
### `ModbusMemory` class

Used to control Modbus-memory.
It uses 1 bit memory to store coil or discrete input value and 16 bits to store holding and input registers

It has function to read/write bits/registers and `copy`-function to inner copy bit and registers 
from one type of memory to another.

### Common classes
* `ModbusMasterTCP` - used to make requests to remote TCP slave(server) to read/write data
* `ModbusMasterRTU` - used to make requests to remote slave(server) via serial port to read/write data
* `ModbusSlaveTCP`  - provide services to read/write data via Modbus TCP/IP protocol
* `ModbusSlaveRTU`  - provide services to read/write data via serial port on Modbus RTU protocol
* `ModbusSlaveBridgeRTU` and `ModbusSlaveBridgeTCP` - provide bridge (protocol converter) functionality


## Examples

Modbus library has examples of code to demonstrate usage of common classes described above.