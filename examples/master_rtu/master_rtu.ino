/*
  Date: 10/2019
  Author: Serhii Marchuk <marchserh@gmail.com>
  
  This example shows how to use ModbusMasterRTU. 
  It makes requests to remote Modbus slave via serial port,
  in this case used Serial1 (for Arduino MEGA). 
  Change this port (or not) to communicate with your Modbus device or simulator.
  
  This program use 8 modbus functions and shows how to use it in loop.
  1.  READ COIL STATUS          - to read 10 coils [000001..000010]
  2.  READ INPUT STATUS         - to read 10 input discretes [100001..100010]
  3.  READ HOLDING REGISTERS    - to read 10 holding registers [400001..400010]
  4.  READ INPUT REGISTERS      - to read 10 input registers [300001..300010]
  5.  FORCE SINGLE COIL         - to write single coil 000001 in toogle way
  6.  FORCE SINGLE REGISTER     - to write single register 400001 increment value
  15. FORCE MULTIPLE COILS      - to write 2 coils 000011 and 000012 in toogle way
  16. FORCE MULTIPLE REGISTERS  - to write 2 registers 400011 increment value and 400012 decrement value

  This program was tested on Arduino MEGA platform

*/


#include <SoftwareSerial.h>
#include <ModbusMasterRTU.h>

// --------------------------------------------------------------------------------------------------------
// ---------------------------------------- INITIALIZE SERIAL PORT ----------------------------------------
// --------------------------------------------------------------------------------------------------------

//const int rxPin = 10;
//const int txPin = 11;
//// serial port to read/write
//SoftwareSerial softSerial(rxPin, txPin); // RX, TX

// --------------------------------------------------------------------------------------------------------
// ------------------------------------ INITIALIZE MODBUS RTU MASTER --------------------------------------
// --------------------------------------------------------------------------------------------------------

//ModbusMasterRTU mb(&softSerial); // serial port to read/write
ModbusMasterRTU mb(&Serial1); // serial port to read/write


// --------------------------------------------------------------------------------------------------------
// ------------------------------------------------ SETUP -------------------------------------------------
// --------------------------------------------------------------------------------------------------------

int i;
uint8_t slave = Modbus::VALID_MODBUS_ADDRESS_BEGIN; // Modbus slave address =1
    
void setup()
{
   // Open serial communications and wait for port to open:
    Serial.begin(9600);
    while (!Serial) // wait for serial port to connect. Needed for native USB port only
        delay(1);
    Serial.println("============================================================");
    Serial.println("================= MODBUS MASTER RTU EXAMPLE ================");
    Serial.println("============================================================");
//    Serial.println("Initialize SofwareSerial port and set parameters as 9600 speed");
//    pinMode(rxPin, INPUT);
//    pinMode(txPin, OUTPUT);
//    softSerial.begin(9600);
    Serial.println("Initialize Serial1 port and set parameters:\n9600 - speed\n8 - data bits\nNo Parity\n1 - stop bit");
    Serial1.begin(9600, SERIAL_8N1);
    // VERBOSE MODE
    mb.setVerboseStream(&Serial);
    
    Serial.println();
}


// --------------------------------------------------------------------------------------------------------
// ------------------------------------------------- LOOP -------------------------------------------------
// --------------------------------------------------------------------------------------------------------

int func_idx = MBF_READ_COIL_STATUS; // Modbus slave address =1

#define COILS_SZ 2
bool coils[COILS_SZ];
bool coil = false;

#define REGES_SZ 2
uint16_t reges[REGES_SZ];
uint16_t rege = 0;

void loop()
{
    const int discrOffset = 0;
    const int discrCount = 10;
    const int regesOffset = 0;
    const int regesCount = 10;
    uint16_t buff[regesCount];
    uint16_t fact;
    Modbus::Response r;
    int i;
    switch (func_idx)
    {
    // ------------------------------------------------
    // 1. READ COIL STATUS
    case MBF_READ_COIL_STATUS:
        r = mb.readCoilStatus(slave, discrOffset, discrCount, buff, &fact);
        if (r == Modbus::OK)
        {
            Serial.print("READ COIL STATUS success. ");Serial.print(fact);Serial.print(" coils was read.\nValues: ");
            for (i = 0; i < discrCount; i++)
            {
                Serial.print(Modbus::getBit(buff, i));
                Serial.print(' ');
            }
            Serial.println();
            delay(1000);                    
        }
        else if (r > Modbus::OK)
        {
            Serial.print("Error while READ COIL STATUS. Code: ");
            Serial.println(r);
            delay(1000);                    
        }
        else // r < 0 => Modbus::PROCESSING
            break;
        Serial.println();
        func_idx = MBF_READ_INPUT_STATUS; // change function processing
        // no need break
    // ------------------------------------------------
    // 2. READ INPUT STATUS
    case MBF_READ_INPUT_STATUS:
        r = mb.readInputStatus(slave, discrOffset, discrCount, buff, &fact);
        if (r == Modbus::OK)
        {
            Serial.print("READ INPUT STATUS success. ");Serial.print(fact);Serial.print(" discrete inputs was read.\nValues: ");
            for (i = 0; i < discrCount; i++)
            {
                Serial.print(Modbus::getBit(buff, i));
                Serial.print(' ');
            }
            Serial.println();
            delay(1000);                    
        }
        else if (r > Modbus::OK)
        {
            Serial.print("Error while READ INPUT STATUS. Code: ");
            Serial.println(r);
            delay(1000);                    
        }
        else // r < 0 => Modbus::PROCESSING
            break;
        Serial.println();
        func_idx = MBF_READ_HOLDING_REGISTERS; // change function processing
        // no need break
    // ------------------------------------------------
    // 3. READ HOLDING REGISTERS
    case MBF_READ_HOLDING_REGISTERS:
        r = mb.readHoldingRegisters(slave, regesOffset, regesCount, buff, &fact);
        if (r == Modbus::OK)
        {
            Serial.print("READ HOLDING REGISTERS success. ");Serial.print(fact);Serial.print(" holding registers was read.\nValues: ");
            for (i = 0; i < regesCount; i++)
            {
                Serial.print(buff[i]);
                Serial.print(' ');
            }
            Serial.println();
            delay(1000);                    
        }
        else if (r > Modbus::OK)
        {
            Serial.print("Error while READ HOLDING REGISTERS. Code: ");
            Serial.println(r);
            delay(1000); 
        }
        else // r < 0 => Modbus::PROCESSING
            break;
        Serial.println();
        func_idx = MBF_READ_INPUT_REGISTERS; // change function processing
        // no need break
    // ------------------------------------------------
    // 4. READ INPUT REGISTERS
    case MBF_READ_INPUT_REGISTERS:
        r = mb.readInputRegisters(slave, regesOffset, regesCount, buff, &fact);
        if (r == Modbus::OK)
        {
            Serial.print("READ INPUT REGISTERS success. ");Serial.print(fact);Serial.print(" input registers was read.\nValues: ");
            for (i = 0; i < regesCount; i++)
            {
                Serial.print(buff[i]);
                Serial.print(' ');
            }
            Serial.println();
            delay(1000);                    
        }
        else if (r > Modbus::OK)
        {
            Serial.print("Error while READ INPUT REGISTERS. Code: ");
            Serial.println(r);
            delay(1000);                    
        }
        else // r < 0 => Modbus::PROCESSING
            break;
        Serial.println();
        func_idx = MBF_FORCE_SINGLE_COIL; // change function processing
        // no need break
    // ------------------------------------------------
    // 5. FORCE SINGLE COIL
    case MBF_FORCE_SINGLE_COIL:
        r = mb.forceSingleCoil(slave, 0, coil);
        if (r == Modbus::OK)
        {
            Serial.println("FORCE SINGLE COIL success");                   
            delay(1000);                    
        }
        else if (r > Modbus::OK)
        {
            Serial.print("Error while FORCE SINGLE COIL. Code: ");
            Serial.println(r);
            delay(1000);                    
        }
        else // r < 0 => Modbus::PROCESSING
            break;
        coil = !coil;
        Serial.println();
        func_idx = MBF_FORCE_SINGLE_REGISTER; // change function processing
        // no need break
    // ------------------------------------------------
    // 6. FORCE SINGLE REGISTER
    case MBF_FORCE_SINGLE_REGISTER:
        r = mb.forceSingleRegister(slave, 0, rege);
        if (r == Modbus::OK)
        {
            Serial.println("FORCE SINGLE REGISTER success");                   
            delay(1000);                    
        }
        else if (r > Modbus::OK)
        {
            Serial.print("Error while FORCE SINGLE REGISTER. Code: ");
            Serial.println(r);
            delay(1000);                    
        }
        else // r < 0 => Modbus::PROCESSING
            break;
        rege++;
        Serial.println();
        func_idx = MBF_FORCE_MULTIPLE_COILS; // change function processing
        // no need break
    // ------------------------------------------------
    // 15. FORCE MULTIPLE COILS
    case MBF_FORCE_MULTIPLE_COILS:
        r = mb.forceMultipleCoils(slave, 10, COILS_SZ, Modbus::setBits(buff, 0, COILS_SZ, coils), &fact);
        if (r == Modbus::OK)
        {
            Serial.print("FORCE MULTIPLE COILS success. ");Serial.print(fact);Serial.println(" coils was written");                   
            delay(1000);                    
        }
        else if (r > Modbus::OK)
        {
            Serial.print("Error while FORCE MULTIPLE COILS. Code: ");
            Serial.println(r);
            delay(1000);                    
        }
        else // r < 0 => Modbus::PROCESSING
            break;
        coils[0] = coils[1];
        coils[1] = !coils[1];
        Serial.println();
        func_idx = MBF_FORCE_MULTIPLE_REGISTERS; // change function processing
        // no need break
    // ------------------------------------------------
    // 16. FORCE MULTIPLE REGISTERS
    case MBF_FORCE_MULTIPLE_REGISTERS:
        r = mb.forceMultipleRegisters(slave, 10, REGES_SZ, reges, &fact);
        if (r == Modbus::OK)
        {
            Serial.print("FORCE MULTIPLE REGISTERS success. ");Serial.print(fact);Serial.println(" registers was written");                   
            delay(1000);                    
        }
        else if (r > Modbus::OK)
        {
            Serial.print("Error while FORCE MULTIPLE REGISTERS. Code: ");
            Serial.println(r);
            delay(1000);                    
        }
        else // r < 0 => Modbus::PROCESSING
            break;
        reges[0]++;
        reges[1]--;
        Serial.println();
        // no need break
    default:
        func_idx = MBF_READ_COIL_STATUS; // change function processing
        break; 
    }
    // ------------------------------------------------
    delay(1);
}
