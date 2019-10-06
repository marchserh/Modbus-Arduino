/*
  Date: 10/2019
  Author: Serhii Marchuk <marchserh@gmail.com>
  
  This example shows how to use ModbusSlaveTCP object in pair with ModbusMemory object.

  Modbus memory mapping:
    
  Discrete inputs:
  +--------------------+--------------------+
  |       Address      |        PIN         |
  +-----------------------------------------+
  |       100001       |        D00         |
  +-----------------------------------------+
  |       100002       |        D01         |
  +-----------------------------------------+
  |       100003       |        D02         |
  +-----------------------------------------+
  |       100004       |        D03         |
  +-----------------------------------------+
  |       100005       |        D04         |
  +-----------------------------------------+
  |       100006       |        D05         |
  +-----------------------------------------+
  |       100007       |        D06         |
  +-----------------------------------------+
  |       100008       |        D07         |
  +-----------------------------------------+
    Discrete outputs:
  +--------------------+--------------------+
  |       Address      |        PIN         |
  +-----------------------------------------+
  |       000001       |        D08         |
  +-----------------------------------------+
  |       000002       |        D09         |
  +-----------------------------------------+
  |       000003       |        D10         |
  +-----------------------------------------+
  |       000004       |        D11         |
  +-----------------------------------------+
  |       000005       |        D12         |
  +-----------------------------------------+
  |       000006       |        D13         |
  +-----------------------------------------+
      Analog inputs:
  +--------------------+--------------------+
  |       Address      |        PIN         |
  +-----------------------------------------+
  |       300001       |        A00         |
  +-----------------------------------------+
  |       300002       |        A01         |
  +-----------------------------------------+
  |       300003       |        A02         |
  +-----------------------------------------+
  |       300004       |        A03         |
  +-----------------------------------------+
  |       300005       |        A04         |
  +-----------------------------------------+
  |       300006       |        A05         |
  +-----------------------------------------+
      Program values:
  +--------------------+--------------------+
  |       Address      |       Value        |
  +-----------------------------------------+
  |       400001       |   Success counter  |
  +-----------------------------------------+
  |       400002       |    Error counter   |
  +-----------------------------------------+
  |       400003       |   Last error code  |
  +-----------------------------------------+
  |    400004..400010  |     Empty values   |
  +-----------------------------------------+

  This program was tested on Arduino UNO and MEGA platforms by use W5100 Ethernet shield
  
*/

#include <Ethernet.h>
#include <ModbusSlaveTCP.h>


// --------------------------------------------------------------------------------------------------------
// --------------------------------- DEFINE DISCRETE INPUT CONFIGURATION ----------------------------------
// --------------------------------------------------------------------------------------------------------

// DISCRETE INPUTS offset
#define DI_BASE_BEGIN  0 // 0
#define DI_BASE_OFFSET DI_BASE_BEGIN

enum DI
{                           // PIN (Discrete)
    DI_BEGIN = 0,           
    DI_00 = DI_BEGIN,       // 00
    DI_01,                  // 01
    DI_02,                  // 02 
    DI_03,                  // 03 
    DI_04,                  // 04 
    DI_05,                  // 05 
    DI_06,                  // 06 
    DI_07,                  // 07 
    DI_COUNT, // =8
    DI_END = DI_COUNT-1,
};

#define DI_BASE_END (DI_BASE_OFFSET+DI_END) // 7


// --------------------------------------------------------------------------------------------------------
// --------------------------------- DEFINE DISCRETE OUTPUT CONFIGURATION ---------------------------------
// --------------------------------------------------------------------------------------------------------

// DISCRETE OUTPUTS offset
#define DO_BASE_BEGIN  DI_COUNT // 8
#define DO_BASE_OFFSET DO_BASE_BEGIN

enum DO
{                           // PIN (Discrete)
    DO_BEGIN = 0,
    DO_00 = DO_BEGIN,       // 08 
    DO_01,                  // 09 
    DO_02,                  // 10 
    DO_03,                  // 11 
    DO_04,                  // 12 
    DO_05,                  // 13 
    DO_COUNT, // =6
    DO_END = DO_COUNT-1
};

#define DO_BASE_END (DO_BASE_OFFSET+DO_END) // 13


// --------------------------------------------------------------------------------------------------------
// ---------------------------------- DEFINE ANALOG INPUT CONFIGURATION -----------------------------------
// --------------------------------------------------------------------------------------------------------

// DISCRETE INPUTS offset
#define AI_BASE_BEGIN  0 // 0
#define AI_BASE_OFFSET AI_BASE_BEGIN

enum AI
{                           // PIN (Analog)
    AI_BEGIN = 0,           
    AI_00 = AI_BEGIN,       // 00
    AI_01,                  // 01
    AI_02,                  // 02 
    AI_03,                  // 03 
    AI_04,                  // 04 
    AI_05,                  // 05 
    AI_COUNT, // =6
    AI_END = AI_COUNT-1,
};

#define AI_BASE_END (AI_BASE_OFFSET+AI_END) // 13


// --------------------------------------------------------------------------------------------------------
// --------------------------------------- INITIALIZE MODBUS MEMORY ---------------------------------------
// --------------------------------------------------------------------------------------------------------

// define count of memory size of each type
#define MODBUS_MEMORY_COUNT_0x 6 //DO_COUNT
#define MODBUS_MEMORY_COUNT_1x 8 //DI_COUNT
#define MODBUS_MEMORY_COUNT_3x 6 //AI_COUNT
#define MODBUS_MEMORY_COUNT_4x 10
#include "ModbusMemory.h"

ModbusMemory mem;


// --------------------------------------------------------------------------------------------------------
// ---------------------------------- INITIALIZE ETHERNET SERVER LIBRARY ----------------------------------
// --------------------------------------------------------------------------------------------------------

// Initialize the Ethernet server library
// with the IP address and port you want to use
// (port 502 is default for Modbus TCP/IP):

// MAC address of current device:
// generated by https://www.miniwebtool.com/mac-address-generator/
byte mac[] = { 0xDC, 0x45, 0xDC, 0xC9, 0xA0, 0x44 };

// IP address of current device:
IPAddress ip(192, 168, 100, 103);


// --------------------------------------------------------------------------------------------------------
// ------------------------------------ INITIALIZE MODBUS TCP SOCKETS -------------------------------------
// --------------------------------------------------------------------------------------------------------

// if yo want to configure 2 TCP sockets for Modbus TCP/IP simultaniously opened communication 
// uncomment all lines with 'sock2' instance.
// That way you can add up to 4 simultaneously working sockets on your own (if enough memory)
ModbusSlaveTCP sock1(&mem);
//ModbusSlaveTCP sock2(&mem);


// --------------------------------------------------------------------------------------------------------
// ------------------------------------------------ SETUP -------------------------------------------------
// --------------------------------------------------------------------------------------------------------

#define MODBUS_ADDRESS 1 // Modbus address (slave/unit). Must be in range [1..247]

int i;
    
void setup()
{
   // Open serial communications and wait for port to open:
    Serial.begin(9600);
    while (!Serial) // wait for serial port to connect. Needed for native USB port only
        delay(1);
    Serial.println("============================================================");
    Serial.println("================= MODBUS SLAVE TCP EXAMPLE =================");
    Serial.println("============================================================");
    Serial.print("Initialize pins from ");Serial.print(DI_BASE_BEGIN);Serial.print(" to ");Serial.print(DI_BASE_END);Serial.println(" as Discrete Inputs");
    for (i = 0; i < DI_COUNT; i++)
        pinMode(DI_BASE_OFFSET+i, INPUT);
    Serial.print("Initialize pins from ");Serial.print(DO_BASE_BEGIN);Serial.print(" to ");Serial.print(DO_BASE_END);Serial.println(" as Discrete Outputs");
    for (i = 0; i < DO_COUNT; i++)
        pinMode(DO_BASE_OFFSET+i, OUTPUT);
    Serial.println("Initialize Ethernet library and start the Ethernet connection and socker");  
    Ethernet.begin(mac, ip);
    Serial.print("Ethernet localIP is at ");Serial.println(Ethernet.localIP());
    Serial.print("Set Modbus address as ");Serial.println(MODBUS_ADDRESS);
    sock1.setSlave(MODBUS_ADDRESS);
    //sock2.setSlave(MODBUS_ADDRESS);
    // VERBOSE MODE
    sock1.setVerboseStream(&Serial);
    //sock2.setVerboseStream(&Serial);
}


// --------------------------------------------------------------------------------------------------------
// ------------------------------------------------- LOOP -------------------------------------------------
// --------------------------------------------------------------------------------------------------------

Modbus::Response r1 = Modbus::PROCESSING, r2 = Modbus::PROCESSING;
int stateL = -1;
void loop()
{
    // ------------------------------------------------
    // 1. READ DATA
    // 1.1. Read discrete input
    for (i = 0; i < DI_COUNT; i++)
        mem.setBool_1x(i, digitalRead(DI_BASE_OFFSET+i));
    // 1.2. Read analog input
    for (i = 0; i < AI_COUNT; i++)
        mem.setUInt16_3x(i, analogRead(AI_BASE_OFFSET+i));
    // 1.3. Read discrete output
    for (i = 0; i < DO_COUNT; i++)
        mem.setBool_0x(i, digitalRead(DO_BASE_OFFSET+i));
    // ------------------------------------------------
    // 2. ALGORHITM
    //
    // Put your code here
    //
    // Example of code (may be deleted (or not :) ):
    // Code writes data to registers as follow:
    // 400001 - success counter
    // 400002 - error counter
    // 400003 - last error
    mem.setUInt16_4x(0, mem.uint16_4x(0)+(r1==Modbus::OK)+(r2==Modbus::OK)); // success counter
    mem.setUInt16_4x(1, mem.uint16_4x(1)+(r1>Modbus::OK)+(r2>Modbus::OK)); // error counter
    if (r1 > Modbus::OK)
        mem.setInt_4x(2, r1); // last error
    if (r2 > Modbus::OK)
        mem.setInt_4x(2, r2); // last error
    // ------------------------------------------------
    // 3. MODBUS EXCHANGE
    //int state = sock1.state();
    r1 = sock1.exec(); // first tcp socket proccessing
    //r2 = sock2.exec(); // second tcp socket proccessing
    // 4. WRITE DATA
    // 4.1 Write discrete output
    for (i = 0; i < DO_COUNT; i++)
        digitalWrite(DO_BASE_OFFSET+i, mem.bool_0x(i));
    delay(1);
}
