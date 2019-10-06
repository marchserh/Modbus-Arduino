/*
  Date: 10/2019
  Author: Serhii Marchuk <marchserh@gmail.com>
  
  This example shows how to make bridge (converter) from Modbus TCP to RTU protocol
  It receives messages via TCP protocol and translate it to RTU remote device.
  IP address of current device is set to '192.168.100.103'.
  In this case used Serial1 (for Arduino MEGA) to send data to remote device via RTU.
  Change this settings (or not) to communicate with your Modbus device or simulator.
  
  Note: This program may use only ONE simultaneously opened TCP socket.
  
  This program was tested on Arduino MEGA platform

*/


#include <Ethernet.h>
#include <SoftwareSerial.h>

#include <ModbusSlaveBridgeTCP.h>
#include <ModbusMasterRTU.h>

// --------------------------------------------------------------------------------------------------------
// ---------------------------------- INITIALIZE ETHERNET SERVER LIBRARY ----------------------------------
// --------------------------------------------------------------------------------------------------------

// Initialize the Ethernet server library
// with the IP address and port you want to use
// (port 502 is default for Modbus TCP/IP):

// MAC address of current device:
// generated by https://www.miniwebtool.com/mac-address-generator/
byte mac[] = { 0x2B, 0xA9, 0x9E, 0xA2, 0xC4, 0x8B };
// IP address of current device:
IPAddress ip(192, 168, 100, 103);


// --------------------------------------------------------------------------------------------------------
// ---------------------------------------- INITIALIZE SERIAL PORT ----------------------------------------
// --------------------------------------------------------------------------------------------------------

//const int rxPin = 10;
//const int txPin = 11;
//SoftwareSerial softSerial(rxPin, txPin); // RX, TX


// --------------------------------------------------------------------------------------------------------
// ------------------------------------ INITIALIZE MODBUS RTU MASTER --------------------------------------
// --------------------------------------------------------------------------------------------------------

//ModbusMasterRTU mb(&softSerial); 
ModbusMasterRTU mb(&Serial1); 


// --------------------------------------------------------------------------------------------------------
// -------------------------------------- INITIALIZE MODBUS TCP SLAVE -------------------------------------
// --------------------------------------------------------------------------------------------------------

ModbusSlaveBridgeTCP tcp(&mb);


// --------------------------------------------------------------------------------------------------------
// ------------------------------------------------ SETUP -------------------------------------------------
// --------------------------------------------------------------------------------------------------------

void setup()
{
   // Open serial communications and wait for port to open:
    Serial.begin(9600);
    while (!Serial) // wait for serial port to connect. Needed for native USB port only
        delay(1);
    Serial.println("============================================================");
    Serial.println("================= MODBUS BRIDGE TCP TO RTU =================");
    Serial.println("============================================================");
    // Initialize Ethernet library
    Serial.println("Initialize Ethernet library");  
    Ethernet.begin(mac, ip);
    Serial.print("Ethernet localIP is at ");
    Serial.println(Ethernet.localIP());
    // Initialize SoftSerial instance
    //pinMode(rxPin, INPUT);
    //pinMode(txPin, OUTPUT);
    //softSerial.begin(9600);
    // tcp.setSlave(MODBUS_ADDRESS); no need to set Slave-address. It must be NULL for bridge
    // Set output mode (Tx, Rx bytes )
    Serial1.begin(9600, SERIAL_8N1);
    // VERBOSE MODE
    tcp.setName("Slave");
    tcp.setVerboseStream(&Serial);
    mb.setName("Master");
    mb.setVerboseStream(&Serial);
}


// --------------------------------------------------------------------------------------------------------
// ------------------------------------------------- LOOP -------------------------------------------------
// --------------------------------------------------------------------------------------------------------

void loop()
{
    tcp.exec();
    delay(1);
}
