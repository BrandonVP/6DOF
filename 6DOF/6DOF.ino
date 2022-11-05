/*
 Name:		_6DOF_V2.ino
 Created:	1/10/2021 3:31:56 PM
 Author:	Brandon Van Pelt
*/

/*=========================================================
    Todo List
===========================================================
Document
- Better comments and improved naming convention
- Error / Range checking
- Acceleration set by controller

- Design way for arm movements of different lengths to end together

- Save arm positions to EEPROM
- Implement wireless estop
- Redesign Run

- Research CRC generation

===========================================================
    End Todo List
=========================================================*/
#include <EEPROM.h>
#include "Actuator.h"
#include <mcp_can_dfs.h>
#include <mcp_can.h>
#include "pinAssignments.h"
#include <SPI.h>
#include "can_buffer.h"

// Debug CAN Bus Connection
//#define DEBUG_CANBUS

// Uncomment an arm for upload
//#define ARM1
#define ARM2
#if defined ARM1
    #include "ch1.h"
#endif
#if defined ARM2
    #include "ch2.h"
#endif

#define REFRESH_RATE 1000
#define ANGLE_ACCELERATION 400

#define axis1StartingAngle 0xB4
#define axis2StartingAngle 0xB4
#define axis3StartingAngle 0x5A
#define axis4StartingAngle 0xB4
#define axis5StartingAngle 0xB4
#define axis6StartingAngle 0xB4

// Balance stepper on / off state
#define PULSE_SPEED_1 100//140                         
#define PULSE_SPEED_2 50                         

// CAN Bus vars
long unsigned int rxId;
byte len = 0;
byte rxBuf[8];

MCP_CAN CAN0(49);

// Actuator objects - (min angle, max angle, current angle)
Actuator axis1(0, 360, axis1StartingAngle);
Actuator axis2(0, 360, axis2StartingAngle);
Actuator axis3(0, 360, axis3StartingAngle);
Actuator axis4(0, 360, axis4StartingAngle);
Actuator axis5(0, 360, axis5StartingAngle);
Actuator axis6(0, 360, axis6StartingAngle);

bool hasAcceleration = true;
bool runSetup = false;
bool runProg = false;
bool delayState = true;
bool isGripOpen = true;
volatile bool eStopActivated = false;

// Run() vars
uint16_t count = 0;
uint16_t acceleration = 0;  
uint32_t maxStep = 0;
volatile uint32_t runIndex = 0;

// Timer for current angle updates
uint32_t timer = 0;

uint32_t updateEEPROM = 0;
bool isPositionSaved = false;

// Wait function variables
uint32_t waitTimer = 0;
bool waitActivated = false;

// Buffer variables
can_buffer myStack;
CAN_Frame incoming;
CAN_Frame buffer;

// Check RAM usage
int freeRam() {
    extern int __heap_start, * __brkval;
    int v;
    return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}

/*=========================================================
    CAN Bus Traffic
===========================================================*/
// Incoming CAN Bus frame pushed to buffer
void MSGBuff()
{
     // Read incoming message
    CAN0.readMsgBuf(&rxId, &len, rxBuf);
    // TODO: Implement this code
    /*
    // Estop check
    if (rxId == ESTOP)
    {
        if (rxBuf[CMD] == ESTOP_ON)
        {
            eStopActivated = true;
        }
        else if (rxBuf[CMD] == ESTOP_OFF)
        {
            eStopActivated = false;
        }
    }
    */
    myStack.push(rxId, rxBuf);
}

// Read and execute messages from buffer
void readMSG()
{
    if (runProg == false && myStack.stack_size() > 0)
    {
        myStack.pop(&buffer);
        controller(buffer);
    }
}

/*
// A very simple "CRC" generator
uint8_t generateByteCRC(volatile uint8_t* data)
{
    return ((data[0] % 2) + (data[1] % 2) + (data[2] % 2) + (data[3] % 2) + (data[4] % 2) + (data[5] % 2) + (data[6] % 2) + (data[7] % 2) + 1);
}
*/
/*=========================================================
CRC - https://barrgroup.com/embedded-systems/how-to/crc-calculation-c-code
===========================================================*/
#define POLYNOMIAL 0xD8  /* 11011 followed by 0's */

/*
 * The width of the CRC calculation and result.
 * Modify the typedef for a 16 or 32-bit CRC standard.
 */

#define WIDTH  (8 * sizeof(uint8_t))
#define TOPBIT (1 << (WIDTH - 1))

uint8_t  crcTable[256];

void initCRC(void)
{
    uint8_t  remainder;

    // Compute the remainder of each possible dividend
    for (int dividend = 0; dividend < 256; ++dividend)
    {
        //Start with the dividend followed by zeros
        remainder = dividend << (WIDTH - 8);

        // Perform modulo-2 division, a bit at a time
        for (uint8_t bit = 8; bit > 0; --bit)
        {
            // Try to divide the current data bit.
            if (remainder & TOPBIT)
            {
                remainder = (remainder << 1) ^ POLYNOMIAL;
            }
            else
            {
                remainder = (remainder << 1);
            }
        }

        // Store the result into the table.
        crcTable[dividend] = remainder;
    }
}

uint8_t generateCRC(volatile uint8_t message[], int nBytes)
{
    uint8_t data;
    uint8_t remainder = 0;

    // Divide the message by the polynomial, a byte at a time.
    for (int byte = 0; byte < nBytes; ++byte)
    {
        data = message[byte] ^ (remainder >> (WIDTH - 8));
        remainder = crcTable[data] ^ (remainder << 8);
    }

    // The final remainder is the CRC.
    return (remainder);
} 


#define DEBUG_CONTROLLER
// Process incoming CAN Frames
void controller(CAN_Frame buffer)
{
    // RX Command List
    #define CRC_BYTE                0x07 // For CONTROL and MANUAL
    #define COMMAND_BYTE            0x01
    #define ACCELERATION_BYTE       0x02
    #define SPEED_BYTE              0x03
    #define LOOP_BYTE               0x04
    #define SUB_COMMAND_BYTE        0x05
    #define GRIP_BYTE               0x06
    #define SET_MIN_BYTE            0x02
    #define SET_SEC_BYTE            0x03

    // List of commands for the COMMAND_BYTE
    #define SEND_AXIS_POSITIONS     0x61
    #define RESET_AXIS_POSITION     0x62
    #define SET_LOWER_AXIS_POSITION 0x63
    #define SET_UPPER_AXIS_POSITION 0x64
    #define MOVE_GRIP               0x6A
    #define HOLD_GRIP               0x00
    #define OPEN_GRIP               0x01
    #define SHUT_GRIP               0x11
    #define SET_WAIT_TIMER          0x6B

    #define EXECUTE_PROGRAM         0x1E // Execute Steps
    #define STOP_PROGRAM            0x0E // Stop Steps

    #define CONFIRMATION            0x1C // Confirm message received
    #define NEG_CONFIRMATION        0x0C // Confirm message not received or failed CRC

    // Used for return confirmation
    byte returnData[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    uint8_t crc = 0;

#if defined DEBUG_CONTROLLER
    Serial.print("ID: ");
    Serial.print(buffer.id, 16);
    Serial.print("   Data: ");
    Serial.print(buffer.data[0], 16);
    Serial.print(" ");
    Serial.print(buffer.data[1], 16);
    Serial.print(" ");
    Serial.print(buffer.data[2], 16);
    Serial.print(" ");
    Serial.print(buffer.data[3], 16);
    Serial.print(" ");
    Serial.print(buffer.data[4], 16);
    Serial.print(" ");
    Serial.print(buffer.data[5], 16);
    Serial.print(" ");
    Serial.print(buffer.data[6], 16);
    Serial.print(" ");
    Serial.print(buffer.data[7], 16);
    Serial.println();
#endif
    // 
    switch (buffer.id)
    {
    case RXID_CONTROL:
        // CRC Check
        //Serial.print(buffer.data[CRC_BYTE]);
        //Serial.print(" = ");
        //Serial.println(generateCRC(buffer.data, 7));
        if (!(buffer.data[CRC_BYTE] = generateCRC(buffer.data, 7)))
        {
            break;
        }
        /*=========================================================
                    Send Current Axis Positions
        ===========================================================*/
        if (buffer.data[COMMAND_BYTE] == SEND_AXIS_POSITIONS)
        {
            updateAxisPos();
            break;
        }
        /*=========================================================
                    Set Axis Angles to Current Postion
        ===========================================================*/
        if (buffer.data[COMMAND_BYTE] == RESET_AXIS_POSITION)
        {
            axis1.set_current_angle(0xB4);
            axis2.set_current_angle(0xB4);
            axis3.set_current_angle(0x5A);
            axis4.set_current_angle(0xB4);
            axis5.set_current_angle(0xB4);
            axis6.set_current_angle(0xB4);
        }
        /*=========================================================
               Set next angles for bottom three axis
        ===========================================================*/
        if (buffer.data[COMMAND_BYTE] == SET_LOWER_AXIS_POSITION)
        {
            if ((buffer.data[2] + buffer.data[3]) > 0) {
                axis1.set_actuator(buffer.data[2] + buffer.data[3]);
            }
            if ((buffer.data[4] + buffer.data[5]) > 0) {
                axis2.set_actuator(buffer.data[4] + buffer.data[5]);
            }
            if ((buffer.data[6] + buffer.data[7]) > 0) {
                axis3.set_actuator(buffer.data[6] + buffer.data[7]);
            }
        }
        /*=========================================================
               Set next angles for top three axis
        ===========================================================*/
        if (buffer.data[COMMAND_BYTE] == SET_UPPER_AXIS_POSITION)
        {
            if ((buffer.data[2] + buffer.data[3]) > 0) {
                axis4.set_actuator(buffer.data[2] + buffer.data[3]);
            }
            if ((buffer.data[4] + buffer.data[5]) > 0) {
                axis5.set_actuator(buffer.data[4] + buffer.data[5]);
            }
            if ((buffer.data[6] + buffer.data[7]) > 0) {
                axis6.set_actuator(buffer.data[6] + buffer.data[7]);
            }
        }
        /*=========================================================
                    Set Wait Timer
        ===========================================================*/
        if (buffer.data[COMMAND_BYTE] == SET_WAIT_TIMER)
        {
            uint8_t seconds = buffer.data[SET_SEC_BYTE];
            uint8_t minutes = buffer.data[SET_MIN_BYTE];
        }
        /*=========================================================
                    Open/Close Grip
        ===========================================================*/
        if ((buffer.data[COMMAND_BYTE] == MOVE_GRIP) || (SUB_COMMAND_BYTE == MOVE_GRIP))
        {
            if (buffer.data[GRIP_BYTE] == OPEN_GRIP)
            {
                open_grip();
            }
            if (buffer.data[GRIP_BYTE] == SHUT_GRIP)
            {
                close_grip();
            }
        }
        /*=========================================================
                    Executes Program Move
        ===========================================================*/
        // |        ID       |  data[0] |   data[1]    |      data[2]      |   data[3]  |   data[4]
        // | EXECUTE_PROGRAM | CRC_BYTE | COMMAND_BYTE | ACCELERATION_BYTE | SPEED_BYTE |  LOOP_BYTE

        if (buffer.data[COMMAND_BYTE] == EXECUTE_PROGRAM)
        {
            returnData[1] = CONFIRMATION;
            CAN0.sendMsgBuf(TXID_CONTROLLER, 0, 8, returnData);
            runProg = true;
            runSetup = true;
            acceleration = ANGLE_ACCELERATION;
            hasAcceleration = true;
        }
        break;
    case RXID_PROGRAM:
        /*=========================================================
                            Program Next Move
        ===========================================================*/
        // Need these temp values to calculate "CRC"
        uint8_t grip;
        uint16_t a1, a2, a3, a4, a5, a6;

        //|                          data                              |
        //|  0  |  1   |   2   |   3   |   4   |   5   |   6   |   7   |
        //| 0-7 | 8-15 | 16-23 | 24-31 | 32-39 | 40-47 | 48-55 | 56-63 |

        //|  crc  |  grip |   a6   |   a5   |   a4   |   a3   |   a2   |   a1   |
        //|  0-4  |  5-9  |  10-18 |  19-27 |  28-36 |  37-45 |  46-54 |  55-63 |
        a1 = ((buffer.data[6] & 0x01) << 8)   | buffer.data[7];
        a2 = (((buffer.data[5] & 0x03)) << 7) | (buffer.data[6] >> 1);
        a3 = (((buffer.data[4] & 0x07)) << 6) | (buffer.data[5] >> 2);
        a4 = (((buffer.data[3] & 0x0F)) << 5) | (buffer.data[4] >> 3);
        a5 = (((buffer.data[2] & 0x1F)) << 4) | (buffer.data[3] >> 4);
        a6 = (((buffer.data[1] & 0x3F)) << 3) | (buffer.data[2] >> 5);
        grip = ((buffer.data[0] & 0x7) << 2)  | (buffer.data[1] >> 6);
        crc = ((buffer.data[0]) >> 3);
        if (crc == (a1 % 2) + (a2 % 2) + (a3 % 2) + (a4 % 2) + (a5 % 2) + (a6 % 2) + (grip % 2) + 1)
        {
            axis1.set_actuator(a1);
            axis2.set_actuator(a2);
            axis3.set_actuator(a3);
            axis4.set_actuator(a4);
            axis5.set_actuator(a5);
            axis6.set_actuator(a6);
        }
        break;
    case RXID_MANUAL:
        /*=========================================================
               Manual Control
        ===========================================================*/
        if (!(buffer.data[CRC_BYTE] == generateCRC(buffer.data, 7)))
        {
            break;
        }
        if ((buffer.data[1] - 0x10) == 1)
        {
            axis1.set_actuator(axis1.get_current_angle() - (buffer.data[1] - 0x10));
        }
        else
        {
            Serial.println("here");
            axis1.set_actuator(axis1.get_current_angle() + (buffer.data[1]));
        }

        if ((buffer.data[2] - 0x10) == 1)
        {
            axis2.set_actuator(axis2.get_current_angle() - (buffer.data[2] - 0x10));
        }
        else
        {
            axis2.set_actuator(axis2.get_current_angle() + (buffer.data[2]));
        }

        if ((buffer.data[3] - 0x10) == 1)
        {
            axis3.set_actuator(axis3.get_current_angle() - ((buffer.data[3] - 0x10)));
        }
        else
        {
            axis3.set_actuator(axis3.get_current_angle() + (buffer.data[3]));
        }

        if ((buffer.data[4] - 0x10) == 1)
        {
            axis4.set_actuator(axis4.get_current_angle() - ((buffer.data[4] - 0x10)));
        }
        else
        {
            axis4.set_actuator(axis4.get_current_angle() + (buffer.data[4]));
        }

        if ((buffer.data[5] - 0x10) == 1)
        {
            axis5.set_actuator(axis5.get_current_angle() - ((buffer.data[5] - 0x10)));
        }
        else
        {
            axis5.set_actuator(axis5.get_current_angle() + (buffer.data[5]));
        }

        if ((buffer.data[6] - 0x10) == 1)
        {
            axis6.set_actuator(axis6.get_current_angle() - ((buffer.data[6] - 0x10)));
        }
        else
        {
            axis6.set_actuator(axis6.get_current_angle() + (buffer.data[6]));
        }
        if ((buffer.data[7] - 0x10) == 1)
        {
            analogWrite(MOTOR_IN2, 0);
            analogWrite(MOTOR_IN1, 160);
            delay(30);
            analogWrite(MOTOR_IN1, 0);
            analogWrite(MOTOR_IN2, 0);
        }
        else if (buffer.data[7] == 1)
        {
            analogWrite(MOTOR_IN1, 0);
            analogWrite(MOTOR_IN2, 255);
            delay(35);
            analogWrite(MOTOR_IN1, 0);
            analogWrite(MOTOR_IN2, 0);
        }
        runProg = true;
        runSetup = true;
        hasAcceleration = false;
        acceleration = 0;
        run();
        break;
    }
}


/*=========================================================
    Movement functions
===========================================================*/
// Finds longest distance to move from all 6 axis
uint32_t findLargest()
{
    uint32_t temp = axis1.get_steps_to_move();
    if (axis2.get_steps_to_move() > temp)
    {
        temp = axis2.get_steps_to_move();
    }
    if (axis3.get_steps_to_move() > temp)
    {
        temp = axis3.get_steps_to_move();
    }
    if (axis4.get_steps_to_move() > temp)
    {
        temp = axis4.get_steps_to_move();
    }
    if (axis5.get_steps_to_move() > temp)
    {
        temp = axis5.get_steps_to_move();
    }
    if (axis6.get_steps_to_move() > temp)
    {
        temp = axis6.get_steps_to_move();
    }
    return temp;
}

// Close grip profile
void close_grip() 
{
    analogWrite(MOTOR_IN2, 0);
    for (uint16_t i = 0; i < 80; i++)
    {
        /*
        if (i % 2)
        {
            analogWrite(MOTOR_IN1, 255);
        }
        else
        {
            analogWrite(MOTOR_IN1, 0);
        }
        */
        // (i % 2) ? analogWrite(MOTOR_IN1, 255) : analogWrite(MOTOR_IN1, 0);
        //delay(1);

    }
    for (uint8_t i = 0; i < 100; i++)
    {
        if (i % 2)
        {
            analogWrite(MOTOR_IN1, 150);
            delay(5);
        }
        else
        {
            analogWrite(MOTOR_IN1, 0);
            delay(25);
        }
        
    }
    
    analogWrite(MOTOR_IN1, 0);
    analogWrite(MOTOR_IN2, 0);
    /*
      uint8_t i;
    analogWrite(MOTOR_IN2, 0);
    analogWrite(MOTOR_IN1, 255);
    delay(1);
    for (i = 188; i >= 1; i--) {
        analogWrite(MOTOR_IN1, i);
        delay(6);
    }
    analogWrite(MOTOR_IN1, 0);
    analogWrite(MOTOR_IN2, 0);
    */
  
}

// Open grip profile
void open_grip() 
{
    analogWrite(MOTOR_IN1, 0);
    for (uint16_t i = 255; i > 100; i--)
    {
        analogWrite(MOTOR_IN2, i);
        delay(2);
    }
    analogWrite(MOTOR_IN2, 0);
    analogWrite(MOTOR_IN1, 0);

    /*
    analogWrite(MOTOR_IN1, 0);
    analogWrite(MOTOR_IN2, 255);
    delay(350);
    //for (int i = 255; i >= 0; i--) {
    //    analogWrite(MOTOR_IN2, i);
    //    delay(1);
    //}
    analogWrite(MOTOR_IN1, 0);
    analogWrite(MOTOR_IN2, 0);
    */
    
}

// Execute movement commands
void run()
{
    if ((eStopActivated) || (runProg == false))
    {
        return;
    }
    if (runSetup)
    {
        maxStep = findLargest();
        runSetup = false;
        runIndex = 0;
        count = 0;
    }

    // TODO: non blocking
    delayMicroseconds(PULSE_SPEED_2 + acceleration);
    if (runIndex < maxStep && delayState == true )
    {
        if ((runIndex < axis1.get_steps_to_move()))
        {
            digitalWrite(DIR_x1, axis1.get_actuator_direction());
            digitalWrite(SPD_x1, true);
            (count == 337) && (axis1.increment_current_angle());
        }
        else if (runIndex == axis1.get_steps_to_move())
        {
            axis1.move();
        }

        if ((runIndex < axis4.get_steps_to_move()))
        {
            digitalWrite(DIR_x2, axis4.get_actuator_direction());
            digitalWrite(SPD_x2, true);
            (count == 337) && (axis4.increment_current_angle());
        }
        else if (runIndex == axis4.get_steps_to_move())
        {
            axis4.move();
        }

        if (runIndex < axis2.get_steps_to_move())
        {
            digitalWrite(DIR_y1, axis2.get_actuator_direction());
            digitalWrite(SPD_y1, true);
            (count == 337) && (axis2.increment_current_angle());
        }
        else if (runIndex == axis2.get_steps_to_move())
        {
            axis2.move();
        }

        if (runIndex < axis5.get_steps_to_move())
        {
            digitalWrite(DIR_y2, axis5.get_actuator_direction());
            digitalWrite(SPD_y2, true);
            (count == 337) && (axis5.increment_current_angle());
        }
        else if (runIndex == axis5.get_steps_to_move())
        {
            axis5.move();
        }

        if (runIndex < axis3.get_steps_to_move())
        {
            digitalWrite(DIR_z1, axis3.get_actuator_direction());
            digitalWrite(SPD_z1, true);
            (count == 337) && (axis3.increment_current_angle());
        }
        else if (runIndex == axis3.get_steps_to_move())
        {
            axis3.move();
        }

        if (runIndex < axis6.get_steps_to_move())
        {
            digitalWrite(DIR_z2, axis6.get_actuator_direction());
            digitalWrite(SPD_z2, true);
            (count == 337) && (axis6.increment_current_angle());
        }
        else if (runIndex == axis6.get_steps_to_move())
        {
            axis6.move();
        }

        delayState = false;
    }
    if (( runIndex < maxStep ) && ( delayState == false ))
    {
        delayMicroseconds(PULSE_SPEED_1 + acceleration);
        digitalWrite(SPD_x1, false);
        digitalWrite(SPD_x2, false);
        digitalWrite(SPD_y1, false);
        digitalWrite(SPD_y2, false);
        digitalWrite(SPD_z1, false);
        digitalWrite(SPD_z2, false);

        runIndex++;
        if (hasAcceleration == true)
        {
            if (acceleration > 0 && maxStep - runIndex > ANGLE_ACCELERATION) {
                acceleration--;
            }
            else if (maxStep - runIndex <= ANGLE_ACCELERATION)
            {
                acceleration++;
            }
        }
        delayState = true;
        (count > 337) ? count = 0 : count++;
    }
    if(runIndex >= maxStep)
    {
        axis1.move();
        axis2.move();
        axis3.move();
        axis4.move();
        axis5.move();
        axis6.move();
        runProg = false;
    }
}


/*=========================================================
    Axis Pos
===========================================================*/
bool swap = false;
// Update pos on a timer
void updateAxisPos()
{
    // Pack data into a single frame using all bits
    if (CAN0.mcp2515_tx_flag_status() == true && millis() - timer > REFRESH_RATE)
    {
        uint8_t data[8];
        uint16_t a1 = axis1.get_current_angle();
        uint16_t a2 = axis2.get_current_angle();
        uint16_t a3 = axis3.get_current_angle();
        uint16_t a4 = axis4.get_current_angle();
        uint16_t a5 = axis5.get_current_angle();
        uint16_t a6 = axis6.get_current_angle();

        // TODO: Add grip value after the grip function is updated
        uint8_t grip = 0;
        uint8_t crc = (a1 % 2) + (a2 % 2) + (a3 % 2) + (a4 % 2) + (a5 % 2) + (a6 % 2) + (grip % 2) + 1;

        data[7] = (a1 & 0xFF);
        data[6] = (a1 >> 8);
        data[6] |= ((a2 & 0xFF) << 1);
        data[5] = (a2 >> 7);
        data[5] |= ((a3 & 0x7F) << 2);
        data[4] = (a3 >> 6);
        data[4] |= ((a4 & 0x3F) << 3);
        data[3] = (a4 >> 5);
        data[3] |= ((a5 & 0x1F) << 4);
        data[2] = (a5 >> 4);
        data[2] |= ((a6 & 0xF) << 5);
        data[1] = (a6 >> 3);
        data[1] |= ((grip & 0x7) << 6);
        data[0] = (grip >> 2);
        data[0] |= (crc << 3);

        cli();
        if (CAN0.mcp2515_tx_flag_status())
        {
            CAN0.mcp2515_set_tx_flag_status();
            CAN0.sendMsgBuf(TXID_POSITION, 0, 8, data);
            timer = millis();
        }
        sei();
    }
}

// Save axis posistions to the EEPROM
// Save after completed move?
void saveAxisPositions()
{
    /*
// TODO: Test and move to function
if (millis() - updateEEPROM > 5000 && !isPositionSaved)
{
    Serial.println("Saving axis positions");
    EEPROM.put(10, axis1.get_current_angle());
    EEPROM.put(20, axis2.get_current_angle());
    EEPROM.put(30, axis3.get_current_angle());
    EEPROM.put(40, axis4.get_current_angle());
    EEPROM.put(50, axis5.get_current_angle());
    EEPROM.put(60, axis6.get_current_angle());
    isPositionSaved = true;
}
*/
}

/*=========================================================
    Setup and Main loop
===========================================================*/
// Setup device
void setup()
{
    // PSU needs time to power up or Mega will hang during setup
#if defined ARM1
    delay(2000);
#endif
#if defined ARM2
    delay(2100);
#endif

    // Disable interrupts
    cli();
    Serial.begin(115200);

    pinMode(INT_PIN, INPUT);

   
    if (CAN0.begin(MCP_STDEXT, CAN_500KBPS, MCP_8MHZ) == CAN_OK)
        Serial.println("MCP2515 Initialized Successfully!");
    else
        Serial.println("Error Initializing MCP2515...");


#if defined ARM1
    // Arm1
    CAN0.init_Mask(0, 0, 0x0FF00000);                // Init first mask...
    CAN0.init_Filt(0, 0, 0x01A00000);                // Init first filter...
    CAN0.init_Filt(1, 0, 0x01A00000);                // Init second filter...
    CAN0.init_Mask(1, 0, 0x0FF00000);                // Init second mask... 
    CAN0.init_Filt(2, 0, 0x01A00000);                // Init third filter...
    CAN0.init_Filt(3, 0, 0x01A00000);                // Init fouth filter...
    CAN0.init_Filt(4, 0, 0x01A00000);                // Init fifth filter...
    CAN0.init_Filt(5, 0, 0x01A00000);                // Init sixth filter...
#endif 
#if defined ARM2
    // Arm2
    CAN0.init_Mask(0, 0, 0x0FF00000);                // Init first mask...
    CAN0.init_Filt(0, 0, 0x02A00000);                // Init first filter...
    CAN0.init_Filt(1, 0, 0x02A00000);                // Init second filter...
    CAN0.init_Mask(1, 0, 0x0FF00000);                // Init second mask...
    CAN0.init_Filt(2, 0, 0x02A00000);                // Init third filter...
    CAN0.init_Filt(3, 0, 0x02A00000);                // Init fouth filter...
    CAN0.init_Filt(4, 0, 0x02A00000);                // Init fifth filter...
    CAN0.init_Filt(5, 0, 0x02A00000);                // Init sixth filter...
#endif

    CAN0.setMode(MCP_NORMAL);  // Change to normal mode to allow messages to be transmitted

    // Pin qssignements for stepper motor drivers
    pinMode(ENA_x1, OUTPUT);
    pinMode(SPD_x1, OUTPUT);
    pinMode(DIR_x1, OUTPUT);

    pinMode(ENA_y1, OUTPUT);
    pinMode(SPD_y1, OUTPUT);
    pinMode(DIR_y1, OUTPUT);

    pinMode(ENA_z1, OUTPUT);
    pinMode(SPD_z1, OUTPUT);
    pinMode(DIR_z1, OUTPUT);

    pinMode(ENA_x2, OUTPUT);
    pinMode(SPD_x2, OUTPUT);
    pinMode(DIR_x2, OUTPUT);

    pinMode(ENA_y2, OUTPUT);
    pinMode(SPD_y2, OUTPUT);
    pinMode(DIR_y2, OUTPUT);

    pinMode(ENA_z2, OUTPUT);
    pinMode(SPD_z2, OUTPUT);
    pinMode(DIR_z2, OUTPUT);

    // Assign grip dc motor as output
    pinMode(MOTOR_IN1, OUTPUT);
    pinMode(MOTOR_IN2, OUTPUT);

    // The interrupt makes movements very choppy
    attachInterrupt(digitalPinToInterrupt(INT_PIN), MSGBuff, FALLING);

    CAN0.sendFlag = true;
    // Enable interrupts
    sei();

    /*
    Serial.print("Free Memory: ");
    Serial.println(freeRam(), DEC);
    uint16_t temp = 0;
    EEPROM.get(10, temp);
    axis1.set_current_angle(temp);
    EEPROM.get(20, temp);
    axis2.set_current_angle(temp);
    EEPROM.get(30, temp);
    axis3.set_current_angle(temp);
    EEPROM.get(40, temp);
    axis4.set_current_angle(temp);
    EEPROM.get(50, temp);
    axis5.set_current_angle(temp);
    EEPROM.get(60, temp);
    axis6.set_current_angle(temp);
    */
    //Serial.end();

    initCRC();
}

#if defined DEBUG_CANBUS
uint32_t CANBusDebugTimer = 0;
uint32_t count1 = 0;
#endif
void CANBus_Debug()
{
    #if defined DEBUG_CANBUS
    const uint8_t TEC_error_register = 0x1C;
    const uint8_t REC_error_register = 0x1D;
    const uint8_t error_register = 0x1D;
    const uint16_t read_register_interval = 2000;

    if (millis() - CANBusDebugTimer > read_register_interval)
    {
        Serial.println(count1++);
        Serial.print("getError: ");
        uint16_t result1 = CAN0.mcp2515_readRegister(error_register);
        Serial.println(result1);

        Serial.print("TEC: ");
        uint16_t result2 = CAN0.mcp2515_readRegister(TEC_error_register);
        Serial.println(result2);

        Serial.print("REC: ");
        uint16_t result3 = CAN0.mcp2515_readRegister(REC_error_register);
        Serial.println(result3);
        Serial.println("");

        Serial.print("Memory: ");
        Serial.println(freeRam());

        Serial.print("Stack Size: ");
        Serial.println(myStack.stack_size());

        /*
        if (result1)
        {
            Serial.println("");
            Serial.println("RESETING ERROR");
            Serial.println(CAN0.getError());
            CAN0.mcp2515_setRegister(0x2D, 0x0);
            Serial.println(CAN0.getError());
            Serial.println("RESETING ERROR");
            Serial.println("");
        }
        if (result2)
        {
            Serial.println("");
            Serial.println("RESETING RX");
            Serial.println(CAN0.getError());
            CAN0.mcp2515_setRegister(0x1D, 0x0);
            Serial.println(CAN0.getError());
            Serial.println("RESETING RX");
            Serial.println("");
        }

        if (result3)
        {
            Serial.println("");
            Serial.println("RESETING TX");
            Serial.println(CAN0.getError());
            CAN0.mcp2515_setRegister(0x1C, 0x0);
            Serial.println(CAN0.getError());
            Serial.println("RESETING TX");
            Serial.println("");
        }
        */
        CANBusDebugTimer = millis();
    }
#endif
}

// Main loop
void loop()
{
    // Debug MCP2515 CAN Bus hardware
    CANBus_Debug();

    // Check if there are any CAN Bus messages in the buffer
    readMSG();

    // Send out the current actuator angles
    updateAxisPos();

    // Execute current commands
    run();

    // Save current axis postions
    saveAxisPositions();
}

