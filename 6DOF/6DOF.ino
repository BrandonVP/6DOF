/*
 ===========================================================================
 Name        : 6DOF.cpp
 Author      : Brandon Van Pelt
 Created	 : 1/10/2021
 Description : Main program file
 ===========================================================================
 */

/*=========================================================
    Todo List
===========================================================
- Acceleration set by controller

- Design way for arm movements of different lengths to end together

- Implement wireless estop

- Actuator remainder sometimes produces wrong axis degrees when calling run()
- Instead of tracking degrees track steps as a float and convert to degrees when needed
- This will preserve the remainder
- Not all steppers have same steps (calculate and correct)

- *** Execute move command without a postion to move to sets all axis to 0 ***

- Grip update axis postion
===========================================================
    End Todo List
=========================================================*/

#include <EEPROM.h>
#include <mcp_can_dfs.h>
#include <mcp_can.h>
#include <SPI.h>

#include "Actuator.h"
#include "pinAssignments.h"
#include "can_buffer.h"

// Debug CAN Bus Connection
//#define DEBUG_CANBUS

// Uncomment an arm for upload
#define ARM1
//#define ARM2
#if defined ARM1
    #include "ch1.h"
#endif
#if defined ARM2
    #include "ch2.h"
#endif

#define REFRESH_RATE 100
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
Actuator axis1(SPD_x1, DIR_x1, ENA_x1, 1, 359, axis1StartingAngle);
Actuator axis2(SPD_y1, DIR_y1, ENA_y1, 1, 359, axis2StartingAngle);
Actuator axis3(SPD_z1, DIR_z1, ENA_z1, 1, 359, axis3StartingAngle);
Actuator axis4(SPD_x2, DIR_x2, ENA_x2, 1, 359, axis4StartingAngle);
Actuator axis5(SPD_y2, DIR_y2, ENA_y2, 1, 359, axis5StartingAngle);
Actuator axis6(SPD_z2, DIR_z2, ENA_z2, 1, 359, axis6StartingAngle);

// Run() vars
bool hasAcceleration = true;
bool runSetup = false;
bool runProg = false;
volatile bool eStopActivated = false;
uint16_t count = 0;
uint16_t acceleration = 0;  
uint32_t maxStep = 0;
uint32_t runIndex = 0;

// Timer for current angle updates
uint32_t timer = 0;

// EEPROM update timer
uint32_t updateEEPROM = 0;

// Wait function variables
uint32_t waitTimer = 0;
bool waitActivated = false;

// Buffer variables
can_buffer myStack;
CAN_Frame incoming;
CAN_Frame buffer;

#define POLYNOMIAL 0xD8  /* 11011 followed by 0's */
#define WIDTH  (8 * sizeof(uint8_t))
#define TOPBIT (1 << (WIDTH - 1))

uint8_t  crcTable[256];

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
    if ((myStack.stack_size() > 0) && (runProg == false))
    {
        myStack.pop(&buffer);
        controller(buffer);
    }
}


/*=========================================================
CRC - https://barrgroup.com/embedded-systems/how-to/crc-calculation-c-code
===========================================================*/
// Create CRC table
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

// All sent messages have a CRC added to data[7]
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

// Process incoming CAN Frames
void controller(CAN_Frame buffer)
{
//#define DEBUG_CONTROLLER
#if defined DEBUG_CONTROLLER
    char msgOut[80];
    sprintf(msgOut, "%3X  %2X %2X %2X %2X %2X %2X %2X %2X", buffer.id, buffer.data[0], buffer.data[1], buffer.data[2], buffer.data[3], buffer.data[4], buffer.data[5], buffer.data[6], buffer.data[7]);
    Serial.println(msgOut);
#endif

    // These are fixed bytes that can not be used for anything else
    #define COMMAND_BYTE            0x00
    #define SUB_COMMAND_BYTE        0x04
    #define CRC_BYTE                0x07 // For CONTROL and MANUAL

    // List of commands for the COMMAND_BYTE
    #define SEND_AXIS_POSITIONS     0x61
    #define RESET_AXIS_POSITION     0x62
    #define HOME_AXIS_POSITION      0x63

    #define MOVE_GRIP_BYTE          0x05
    #define MOVE_GRIP               0x6A
    #define HOLD_GRIP               0x00
    #define OPEN_GRIP               0x01
    #define SHUT_GRIP               0x11

    #define SET_WAIT_TIMER          0x6B
    #define SET_WAIT_MIN_BYTE       0x01
    #define SET_WAIT_SEC_BYTE       0x02
    #define SET_WAIT_MS_BYTE        0x03

    #define EXECUTE_PROGRAM         0x1E // Execute Steps
    #define STOP_PROGRAM            0x0E // Stop Steps
    #define ACCELERATION_BYTE       0x01
    #define SPEED_BYTE              0x02
    #define LOOP_BYTE               0x03

    #define CONFIRMATION_BYTE       0x01
    #define CONFIRMATION            0x1C // Confirm message received
    #define NEG_CONFIRMATION        0x0C // Confirm message not received or failed CRC

    // Used for return confirmation
    byte returnData[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    if (!(buffer.data[CRC_BYTE] == generateCRC(buffer.data, 7)))
    {
        return;
    }

    // 
    switch (buffer.id)
    {
    case RXID_CONTROL:
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
            axis1.set_deg(0xB4);
            axis2.set_deg(0xB4);
            axis3.set_deg(0x5A);
            axis4.set_deg(0xB4);
            axis5.set_deg(0xB4);
            axis6.set_deg(0xB4);
        }
        /*=========================================================
                                HOME ARM
        ===========================================================*/
        if (buffer.data[COMMAND_BYTE] == HOME_AXIS_POSITION)
        {
            axis1.set_actuator(axis1StartingAngle);
            axis2.set_actuator(axis2StartingAngle);
            axis3.set_actuator(axis3StartingAngle);
            axis4.set_actuator(axis4StartingAngle);
            axis5.set_actuator(axis5StartingAngle);
            axis6.set_actuator(axis6StartingAngle);

            buffer.data[COMMAND_BYTE] = EXECUTE_PROGRAM;
        }
        /*=========================================================
                    Set Wait Timer
        ===========================================================*/
        if (buffer.data[COMMAND_BYTE] == SET_WAIT_TIMER)
        {
            uint8_t ms = buffer.data[SET_WAIT_MS_BYTE];
            uint8_t seconds = buffer.data[SET_WAIT_SEC_BYTE];
            uint8_t minutes = buffer.data[SET_WAIT_MIN_BYTE];
            uint32_t tt = millis() + (seconds * 1000);
            // TODO: Determine how the wait timer should be implemented
            while (millis() < tt)
            {
            }
        }
        /*=========================================================
                    Open/Close Grip
        ===========================================================*/
        if ((buffer.data[COMMAND_BYTE] == MOVE_GRIP) || (SUB_COMMAND_BYTE == MOVE_GRIP))
        {
            if (buffer.data[MOVE_GRIP_BYTE] == OPEN_GRIP)
            {
                open_grip();
            }
            if (buffer.data[MOVE_GRIP_BYTE] == SHUT_GRIP)
            {
                close_grip();
            }
        }
        /*=========================================================
                    Executes Program Move
        ===========================================================*/
        // |        ID       |    data[0]   |     data[1]       |   data[2]  |  data[3]  |     data[4]      | data[5] | data[6] | data[7]  |
        // | EXECUTE_PROGRAM | COMMAND_BYTE | ACCELERATION_BYTE | SPEED_BYTE | LOOP_BYTE | SUB_COMMAND_BYTE | SUB_CMD | SUB_CMD | CRC_BYTE |

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

        //|  grip |   a6   |   a5   |   a4   |   a3   |   a2   |   a1   |   crc   |
        //|  0-1  |  2-10  |  11-19 |  20-28 |  29-37 |  38-46 |  47-55 |  56-64  |
        a1 = ((buffer.data[5] & 0x01) << 8)   | buffer.data[6];
        a2 = (((buffer.data[4] & 0x03)) << 7) | (buffer.data[5] >> 1);
        a3 = (((buffer.data[3] & 0x07)) << 6) | (buffer.data[4] >> 2);
        a4 = (((buffer.data[2] & 0x0F)) << 5) | (buffer.data[3] >> 3);
        a5 = (((buffer.data[1] & 0x1F)) << 4) | (buffer.data[2] >> 4);
        a6 = (((buffer.data[0] & 0x3F)) << 3) | (buffer.data[1] >> 5);
        grip = (buffer.data[0] >> 6);

#ifdef DEBUG_CONTROLLER
        char axisOutput[50];
        sprintf(axisOutput, "%3d %3d %3d %3d %3d %3d", a1, a2, a3, a4, a5, a6);
        Serial.println(axisOutput);
#endif

        axis1.set_actuator(a1);
        axis2.set_actuator(a2);
        axis3.set_actuator(a3);
        axis4.set_actuator(a4);
        axis5.set_actuator(a5);
        axis6.set_actuator(a6);
        break;
    case RXID_MANUAL:
        /*=========================================================
               Manual Control
        ===========================================================*/
        if ((buffer.data[1] - 0x10) == 1)
        {
            axis1.set_actuator(axis1.get_deg() - (buffer.data[1] - 0x10));
        }
        else
        {
            axis1.set_actuator(axis1.get_deg() + (buffer.data[1]));
        }

        if ((buffer.data[2] - 0x10) == 1)
        {
            axis2.set_actuator(axis2.get_deg() - (buffer.data[2] - 0x10));
        }
        else
        {
            axis2.set_actuator(axis2.get_deg() + (buffer.data[2]));
        }

        if ((buffer.data[3] - 0x10) == 1)
        {
            axis3.set_actuator(axis3.get_deg() - ((buffer.data[3] - 0x10)));
        }
        else
        {
            axis3.set_actuator(axis3.get_deg() + (buffer.data[3]));
        }

        if ((buffer.data[4] - 0x10) == 1)
        {
            axis4.set_actuator(axis4.get_deg() - ((buffer.data[4] - 0x10)));
        }
        else
        {
            axis4.set_actuator(axis4.get_deg() + (buffer.data[4]));
        }

        if ((buffer.data[5] - 0x10) == 1)
        {
            axis5.set_actuator(axis5.get_deg() - ((buffer.data[5] - 0x10)));
        }
        else
        {
            axis5.set_actuator(axis5.get_deg() + (buffer.data[5]));
        }

        if ((buffer.data[6] - 0x10) == 1)
        {
            axis6.set_actuator(axis6.get_deg() - ((buffer.data[6] - 0x10)));
        }
        else
        {
            axis6.set_actuator(axis6.get_deg() + (buffer.data[6]));
        }
        if ((buffer.data[0] - 0x10) == 1)
        {
            analogWrite(MOTOR_IN2, 0);
            analogWrite(MOTOR_IN1, 160);
            delay(30);
            analogWrite(MOTOR_IN1, 0);
            analogWrite(MOTOR_IN2, 0);
        }
        else if (buffer.data[0] == 1)
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
    uint32_t temp = axis1.get_steps();
    if (axis2.get_steps() > temp)
    {
        temp = axis2.get_steps();
    }
    if (axis3.get_steps() > temp)
    {
        temp = axis3.get_steps();
    }
    if (axis4.get_steps() > temp)
    {
        temp = axis4.get_steps();
    }
    if (axis5.get_steps() > temp)
    {
        temp = axis5.get_steps();
    }
    if (axis6.get_steps() > temp)
    {
        temp = axis6.get_steps();
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
//#define DEBUG_RUN
    if ((eStopActivated) || (runProg == false))
    {
        return;
    }
    if (runSetup)
    {
        count = 0;
        runSetup = false;

        // Acceleration
        maxStep = findLargest();
        runIndex = 0;
#ifdef DEBUG_RUN
        Serial.println("ENTER");
        Serial.print("axis1: ");
        Serial.println(axis1.get_steps());
        Serial.print("axis2: ");
        Serial.println(axis2.get_steps());
        Serial.print("axis3: ");
        Serial.println(axis3.get_steps());
        Serial.print("axis4: ");
        Serial.println(axis4.get_steps());
        Serial.print("axis5: ");
        Serial.println(axis5.get_steps());
        Serial.print("axis6: ");
        Serial.println(axis6.get_steps());
        Serial.print("count: ");
        Serial.println(count);
#endif
    }
    if (axis1.get_steps() || axis2.get_steps() || axis3.get_steps() || axis4.get_steps() || axis5.get_steps() || axis6.get_steps())
    {
        // ** First half move **
        // 
        // Count steps up to 1 degree to increment current axis positions
        (count == 377) ? count = 0 : count++;

        if (axis1.get_steps() > 0)
        {
            digitalWrite(DIR_x1, axis1.get_direction());
            digitalWrite(SPD_x1, true);
            axis1.reduceSteps();
            (count == 100) && (axis1.addSteps(count));
        }
        else
        {
            axis1.move();
        }
        if (axis2.get_steps() > 0)
        {
            digitalWrite(DIR_y1, axis2.get_direction());
            digitalWrite(SPD_y1, true);
            axis2.reduceSteps();
            (count == 100) && (axis2.addSteps(count));
        }
        else
        {
            axis2.move();
        }
        if (axis3.get_steps() > 0)
        {
            digitalWrite(DIR_z1, axis3.get_direction());
            digitalWrite(SPD_z1, true);
            axis3.reduceSteps();
            (count == 100) && (axis3.addSteps(count));
        }
        else
        {
            axis3.move();
        }
        if (axis4.get_steps() > 0)
        {
            digitalWrite(DIR_x2, axis4.get_direction());
            digitalWrite(SPD_x2, true);
            axis4.reduceSteps();
            (count == 100) && (axis4.addSteps(count));
        }
        else
        {
            axis4.move();
        }
        if (axis5.get_steps() > 0)
        {
            digitalWrite(DIR_y2, axis5.get_direction());
            digitalWrite(SPD_y2, true);
            axis5.reduceSteps();
            (count == 100) && (axis5.addSteps(count));
        }
        else
        {
            axis5.move();
        }
        if (axis6.get_steps() > 0)
        {
            digitalWrite(DIR_z2, axis6.get_direction());
            digitalWrite(SPD_z2, true);
            axis6.reduceSteps();
            (count == 100) && (axis6.addSteps(count));
        }
        else
        {
            axis6.move();
        }
        // ** Second half move **
        delayMicroseconds(PULSE_SPEED_1 + acceleration);
        digitalWrite(SPD_x1, false);
        digitalWrite(SPD_x2, false);
        digitalWrite(SPD_y1, false);
        digitalWrite(SPD_y2, false);
        digitalWrite(SPD_z1, false);
        digitalWrite(SPD_z2, false);

        if (hasAcceleration == true)
        {
            if (acceleration > 0 && maxStep - runIndex > ANGLE_ACCELERATION) {
                acceleration--;
            }
            else if (maxStep - runIndex <= ANGLE_ACCELERATION)
            {
                acceleration++;
            }
            runIndex++;
        }
    }
    else
    {
        runProg = false;
        axis1.move();
        axis2.move();
        axis3.move();
        axis4.move();
        axis5.move();
        axis6.move();

#ifdef DEBUG_RUN
        Serial.println("EXIT");
        Serial.print("axis1: ");
        Serial.println(axis1.get_steps());
        Serial.print("axis2: ");
        Serial.println(axis2.get_steps());
        Serial.print("axis3: ");
        Serial.println(axis3.get_steps());
        Serial.print("axis4: ");
        Serial.println(axis4.get_steps());
        Serial.print("axis5: ");
        Serial.println(axis5.get_steps());
        Serial.print("axis6: ");
        Serial.println(axis6.get_steps());
        Serial.print("count: ");
        Serial.println(count);
#endif
    }
}


/*=========================================================
    Axis Pos
===========================================================*/
#define A1_HIGH_BYTE 10
#define A1_LOW_BYTE  11
#define A2_HIGH_BYTE 20
#define A2_LOW_BYTE  21
#define A3_HIGH_BYTE 30
#define A3_LOW_BYTE  31
#define A4_HIGH_BYTE 40
#define A4_LOW_BYTE  41
#define A5_HIGH_BYTE 50
#define A5_LOW_BYTE  51
#define A6_HIGH_BYTE 60
#define A6_LOW_BYTE  61
#define MAX_SAVED_DEGREE 361

// Update pos on a timer
void updateAxisPos()
{
    // Pack data into a single frame using all bits
    if (CAN0.mcp2515_tx_flag_status() == true && millis() - timer > REFRESH_RATE)
    {
        uint8_t data[8];
        uint16_t a1 = axis1.get_deg();
        uint16_t a2 = axis2.get_deg();
        uint16_t a3 = axis3.get_deg();
        uint16_t a4 = axis4.get_deg();
        uint16_t a5 = axis5.get_deg();
        uint16_t a6 = axis6.get_deg();
        uint8_t grip = 0;

        data[6] = (a1 & 0xFF);
        data[5] = (a1 >> 8);
        data[5] |= ((a2 & 0xFF) << 1);
        data[4] = (a2 >> 7);
        data[4] |= ((a3 & 0x7F) << 2);
        data[3] = (a3 >> 6);
        data[3] |= ((a4 & 0x3F) << 3);
        data[2] = (a4 >> 5);
        data[2] |= ((a5 & 0x1F) << 4);
        data[1] = (a5 >> 4);
        data[1] |= ((a6 & 0xF) << 5);
        data[0] = (a6 >> 3);
        data[0] |= ((grip & 0x7) << 6);
        data[7] = generateCRC(data, 7);

        cli();
        if (CAN0.mcp2515_tx_flag_status())
        {
            CAN0.mcp2515_set_tx_flag_status();
            CAN0.sendMsgBuf(TXID_POSITION, 0, 8, data);
        }
        sei();

        timer = millis();
    }
}

// Save axis posistions to the EEPROM
void saveAxisPositions()
{
    if (millis() - updateEEPROM > 10000)
    {
        // Update only saves if value is changed
        EEPROM.put(A1_HIGH_BYTE, (byte)(axis1.get_deg() & 0xFF));
        EEPROM.put(A1_LOW_BYTE, (byte)((axis1.get_deg() >> 8) & 0xFF));
        EEPROM.put(A2_HIGH_BYTE, (byte)(axis2.get_deg() & 0xFF));
        EEPROM.put(A2_LOW_BYTE, (byte)((axis2.get_deg() >> 8) & 0xFF));
        EEPROM.put(A3_HIGH_BYTE, (byte)(axis3.get_deg() & 0xFF));
        EEPROM.put(A3_LOW_BYTE, (byte)((axis3.get_deg() >> 8) & 0xFF));
        EEPROM.put(A4_HIGH_BYTE, (byte)(axis4.get_deg() & 0xFF));
        EEPROM.put(A4_LOW_BYTE, (byte)((axis4.get_deg() >> 8) & 0xFF));
        EEPROM.put(A5_HIGH_BYTE, (byte)(axis5.get_deg() & 0xFF));
        EEPROM.put(A5_LOW_BYTE, (byte)((axis5.get_deg() >> 8) & 0xFF));
        EEPROM.put(A6_HIGH_BYTE, (byte)(axis6.get_deg() & 0xFF));
        EEPROM.put(A6_LOW_BYTE, (byte)((axis6.get_deg() >> 8) & 0xFF));
        updateEEPROM = millis();
    }
}

// Load saved axis angles from EEPROM
void loadSavedAxisPosition()
{
    uint16_t loadA1 = EEPROM.read(A1_HIGH_BYTE) + (EEPROM.read(A1_LOW_BYTE) << 8);
    (loadA1 < MAX_SAVED_DEGREE) && (axis1.set_deg(loadA1));
    uint16_t loadA2 = EEPROM.read(A2_HIGH_BYTE) + (EEPROM.read(A2_LOW_BYTE) << 8);
    (loadA2 < MAX_SAVED_DEGREE) && (axis2.set_deg(loadA2));
    uint16_t loadA3 = EEPROM.read(A3_HIGH_BYTE) + (EEPROM.read(A3_LOW_BYTE) << 8);
    (loadA3 < MAX_SAVED_DEGREE) && axis3.set_deg(loadA3);
    uint16_t loadA4 = EEPROM.read(A4_HIGH_BYTE) + (EEPROM.read(A4_LOW_BYTE) << 8);
    (loadA4 < MAX_SAVED_DEGREE) && axis4.set_deg(loadA4);
    uint16_t loadA5 = EEPROM.read(A5_HIGH_BYTE) + (EEPROM.read(A5_LOW_BYTE) << 8);
    (loadA5 < MAX_SAVED_DEGREE) && axis5.set_deg(loadA5);
    uint16_t loadA6 = EEPROM.read(A6_HIGH_BYTE) + (EEPROM.read(A6_LOW_BYTE) << 8);
    (loadA6 < MAX_SAVED_DEGREE) && axis6.set_deg(loadA6);
}

/*=========================================================
    Setup and Main loop
===========================================================*/
// Setup device
void setup()
{
    // Disable interrupts
    cli();
    Serial.begin(115200);

    pinMode(INT_PIN, INPUT);

    if (CAN0.begin(MCP_STDEXT, CAN_500KBPS, MCP_8MHZ) == CAN_OK)
    {
        Serial.println(F("MCP2515 Initialized Successfully!"));
    }
    else
    {
        Serial.println(F("Error Initializing MCP2515..."));
    }
        
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

    // Incomming message interrupt
    attachInterrupt(digitalPinToInterrupt(INT_PIN), MSGBuff, FALLING);

    CAN0.sendFlag = true;
  
    Serial.print(F("Free Memory: "));
    Serial.println(freeRam(), DEC);
    
    loadSavedAxisPosition();

    initCRC();

    // Enable interrupts
    sei();
}

// Check for CAN Bus hardware related errors
void CANBus_Debug()
{
#if defined DEBUG_CANBUS
    static uint32_t CANBusDebugTimer = 0;
    static uint32_t count1 = 0;

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

    // Send out the current axis angles
    updateAxisPos();

    // Execute movement commands
    run();

    // Save current axis postions to EEPROM if values change
    saveAxisPositions();
}
