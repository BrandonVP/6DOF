/*
 Name:		_6DOF_V2.ino
 Created:	1/10/2021 3:31:56 PM
 Author:	Brandon Van Pelt
*/

/*=========================================================
    Todo List
===========================================================
Replace delayMicroseconds with system timer
- Holding millis() value with int is too slow
- Register uint32_t shows 0 when assigned millis()

Design way for arm movements of different lengths end together

Update actuator angles while moving
===========================================================
    End Todo List
=========================================================*/

#include "Actuator.h"
#include "CANBuffer.h"
#include <LinkedListLib.h>
#include <mcp_can_dfs.h>
#include <mcp_can.h>
#include "PinAssignments.h"
#include <SPI.h>

// Select channel IDs
#include "ch1.h"
//#include "ch2.h"

#define CS_PIN    49
#define INT_PIN   2
#define ANGLE_ACCELERATION 400

#define axis1StartingAngle 0xB4
#define axis2StartingAngle 0xB4
#define axis3StartingAngle 0x5A
#define axis4StartingAngle 0xB4
#define axis5StartingAngle 0xB4
#define axis6StartingAngle 0xB4

// Balance stepper on / off state
constexpr auto PULSE_SPEED_1 = 140;                           
constexpr auto PULSE_SPEED_2 = 10;                          

//
long unsigned int rxId;
INT8U len = 0;
INT8U rxBuf[8];

MCP_CAN CAN0(49);

// Linked list of nodes for a program
LinkedList<CANBuffer*> buffer;

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
bool lowerSendPos = false;
bool upperSendPos = false;
bool delayState = true;
uint16_t acceleration = 0;

uint32_t maxStep = 0;
uint32_t runIndex = 0;

// Open grip is true
bool isGrip = true;

// Close grip profile
void close_grip() {
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
}

// Open grip profile
void open_grip() {
    analogWrite(MOTOR_IN1, 0);
    analogWrite(MOTOR_IN2, 255);
    delay(350);
    //for (int i = 255; i >= 0; i--) {
    //    analogWrite(MOTOR_IN2, i);
    //    delay(1);
    //}
    analogWrite(MOTOR_IN1, 0);
    analogWrite(MOTOR_IN2, 0);
}

//
void setup()
{
    // PSU needs time to power up or Mega will hang during setup
    delay(4000);

    // Disable interrupts
    cli();

    Serial.begin(115200);
    if (CAN0.begin(MCP_STDEXT, CAN_500KBPS, MCP_8MHZ) == CAN_OK)
        Serial.println("MCP2515 Activated");
    else
        Serial.println("MCP2515 Failed");

    pinMode(INT_PIN, INPUT);                       // Setting pin 2 for /INT input

    CAN0.init_Mask(0, 0, 0x00FF0000);                // Init first mask...
    CAN0.init_Filt(0, 0, 0x00A00000);                // Init first filter...
    CAN0.init_Filt(1, 0, 0x00A10000);                // Init second filter...
    CAN0.init_Mask(1, 0, 0x00FF0000);                // Init second mask... 
    CAN0.init_Filt(2, 0, 0x00A20000);                // Init third filter...
    CAN0.init_Filt(3, 0, 0x00A30000);                // Init fouth filter...
    CAN0.init_Filt(4, 0, 0x00A40000);                // Init fifth filter...
    CAN0.init_Filt(5, 0, 0x00A50000);                // Init sixth filter...

   /*
    CAN0.init_Mask(0, 0, 0x00BF0000);                // Init first mask...
    CAN0.init_Filt(0, 0, 0x00B00000);                // Init first filter...
    CAN0.init_Filt(1, 0, 0x00B10000);                // Init second filter...
    CAN0.init_Mask(1, 0, 0x00BF0000);                // Init second mask...
    CAN0.init_Filt(2, 0, 0x00B20000);                // Init third filter...
    CAN0.init_Filt(3, 0, 0x00B30000);                // Init fouth filter...
    CAN0.init_Filt(4, 0, 0x00B40000);                // Init fifth filter...
    CAN0.init_Filt(5, 0, 0x00B50000);                // Init sixth filter...
   */

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

    //Serial.end();
    attachInterrupt(digitalPinToInterrupt(INT_PIN), MSGBuff, FALLING);

    // Enable interrupts
    sei();
}

//
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

//
void run()
{
    if (runProg == false)
    {
        return;
    }
    if (runSetup)
    {
        maxStep = findLargest();
        runSetup = false;
        runIndex = 0;
    }

    delayMicroseconds(PULSE_SPEED_2 + acceleration);
    if (runIndex < maxStep && delayState == true )
    {
        if ((runIndex < axis1.get_steps_to_move()))
        {
            digitalWrite(DIR_x1, axis1.get_actuator_direction());
            digitalWrite(SPD_x1, true);
        }
        if ((runIndex < axis4.get_steps_to_move()))
        {
            digitalWrite(DIR_x2, axis4.get_actuator_direction());
            digitalWrite(SPD_x2, true);
        }
        if (runIndex < axis2.get_steps_to_move())
        {
            digitalWrite(DIR_y1, axis2.get_actuator_direction());
            digitalWrite(SPD_y1, true);
        }
        if (runIndex < axis5.get_steps_to_move())
        {
            digitalWrite(DIR_y2, axis5.get_actuator_direction());
            digitalWrite(SPD_y2, true);
        }
        if (runIndex < axis3.get_steps_to_move())
        {
            digitalWrite(DIR_z1, axis3.get_actuator_direction());
            digitalWrite(SPD_z1, true);
        }
        if (runIndex < axis6.get_steps_to_move())
        {
            digitalWrite(DIR_z2, axis6.get_actuator_direction());
            digitalWrite(SPD_z2, true);
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

// Attached to interupt - Incoming CAN Bus frame saved to buffer
void MSGBuff()
{
    // Read incoming message
    CAN0.readMsgBuf(&rxId, &len, rxBuf);

    if ((rxId == 0xA0 || rxId == 0xB0) && (rxBuf[1] == 0x1 || rxBuf[1] == 0x02))
    {
        lowerSendPos = true;
        upperSendPos = true;
        return;
    }

    // Create object to store message
    CANBuffer* node = new CANBuffer(rxId, rxBuf);

    // Insert object into linked list
    buffer.InsertHead(node);
}

// Delete pointer after poping object
void deleteObj(CANBuffer* obj)
{
    delete obj;
}

// Read messages from buffer
void readMSG()
{
    if (runProg == false && buffer.GetSize() > 0)
    {
        // Retrieve the first message received
        uint16_t ID = buffer.GetTail()->getID();
        uint8_t* MSG = buffer.GetTail()->getMessage();

        // Send the message to the controller for processing
        controller(ID, MSG);

        // Delete CANBuffer object
        deleteObj(buffer.GetTail());

        // Delete position in linkedlist from list
        buffer.RemoveTail();
    }
}

// Send Lower Axis Positions
void sendLowerPos()
{
    if (lowerSendPos == true)
    {
        byte returnData[8] = { 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

        // Get and set angles in returnData
        uint16_t temp = axis1.get_current_angle();
        if (temp > 255)
        {
            returnData[2] = 0x01;
            returnData[3] = temp - 256;
        }
        else
        {
            returnData[3] = temp;
        }
        temp = axis2.get_current_angle();
        if (temp > 255)
        {
            returnData[4] = 0x01;
            returnData[5] = temp - 256;
        }
        else
        {
            returnData[5] = temp;
        }
        temp = axis3.get_current_angle();
        if (temp > 255)
        {
            returnData[6] = 0x01;
            returnData[7] = temp - 256;
        }
        else
        {
            returnData[7] = temp;
        }

        // Return axis positions for bottom three axis
        CAN0.sendMsgBuf(RXID_SEND, 0, 8, returnData);
        lowerSendPos = false;
    }
}

// Send Upper Axis Positions
void sendUpperPos()
{
    if (upperSendPos == true)
    {
        byte returnData[8] = { 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
        uint8_t temp;

        temp = axis4.get_current_angle();
        if (temp > 255)
        {
            returnData[2] = 0x01;
            returnData[3] = temp - 256;
        }
        else
        {
            returnData[3] = temp;
        }
        temp = axis5.get_current_angle();
        if (temp > 255)
        {
            returnData[4] = 0x01;
            returnData[5] = temp - 256;
        }
        else
        {
            returnData[5] = temp;
        }
        temp = axis6.get_current_angle();
        if (temp > 255)
        {
            returnData[6] = 0x01;
            returnData[7] = temp - 256;
        }
        else
        {
            returnData[7] = temp;
        }
        // Return axis positions for top three axis
        CAN0.sendMsgBuf(RXID_SEND, 0, 8, returnData);

        upperSendPos = false;
    }
}

// Process incoming CAN Frames
void controller(uint16_t ID, uint8_t* MSG)
{
    // Used for return confirmation
    byte returnData[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

    // 
    switch (ID)
    {
    case RXID_CONTROL:
        /*=========================================================
                    Send Current Lower Axis Positions
        ===========================================================*/
        if (MSG[1] == 0x01)
        {
            lowerSendPos = true;
            break;
        }

        /*=========================================================
                    Send Current Higher Axis Positions
        ===========================================================*/
        if (MSG[1] == 0x02)
        {
            upperSendPos = true;
            break;
        }

        /*=========================================================
                    Set Axis Angles to Current Postion
        ===========================================================*/
        if (MSG[1] == 0x03)
        {
            axis1.set_current_angle(0xB4);
            axis2.set_current_angle(0xB4);
            axis3.set_current_angle(0x5A);
            axis4.set_current_angle(0xB4);
            axis5.set_current_angle(0xB4);
            axis6.set_current_angle(0xB4);
        }

        /*=========================================================
                    Open/Close Grip
        ===========================================================*/
        if (MSG[6] == 0x01)
        {
            open_grip();
        }
        if (MSG[7] == 0x01)
        {
            close_grip();
        }

        /*=========================================================
                    Executes Move
        ===========================================================*/
        if (MSG[0] == 0x01)
        {
            returnData[1] = 0x03;
            CAN0.sendMsgBuf(RXID_SEND, 0, 8, returnData);
            runProg = true;
            runSetup = true;
            acceleration = ANGLE_ACCELERATION;
            hasAcceleration = true;
        }
        break;
    case RXID_LOWER:
        // Set next angles for bottom three axis
        if ((MSG[2] + MSG[3]) > 0) {
            axis1.set_actuator(MSG[2] + MSG[3]);
        }
        if ((MSG[4] + MSG[5]) > 0) {
            axis2.set_actuator(MSG[4] + MSG[5]);
        }
        if ((MSG[6] + MSG[7]) > 0) {
            axis3.set_actuator(MSG[6] + MSG[7]);
        }
        break;

    case RXID_UPPER:
        // Set next angles for top three axis
        if ((MSG[2] + MSG[3]) > 0) {
            axis4.set_actuator(MSG[2] + MSG[3]);
        }
        if ((MSG[4] + MSG[5]) > 0) {
            axis5.set_actuator(MSG[4] + MSG[5]);
        }
        if ((MSG[6] + MSG[7]) > 0) {
            axis6.set_actuator(MSG[6] + MSG[7]);
        }
        break;

    case RX_MANUAL:
        // Manual Control
        if ((MSG[1] - 0x10) == 1)
        {
            axis1.set_actuator(axis1.get_current_angle() - (MSG[0] * (MSG[1] - 0x10)));
        }
        else
        {
            axis1.set_actuator(axis1.get_current_angle() + (MSG[0] * MSG[1]));
        }

        if ((MSG[2] - 0x10) == 1)
        {
            axis2.set_actuator(axis2.get_current_angle() - (MSG[0] * (MSG[2] - 0x10)));
        }
        else
        {
            axis2.set_actuator(axis2.get_current_angle() + (MSG[0] * MSG[2]));
        }

        if ((MSG[3] - 0x10) == 1)
        {
            axis3.set_actuator(axis3.get_current_angle() - (MSG[0] * (MSG[3] - 0x10)));
        }
        else
        {
            axis3.set_actuator(axis3.get_current_angle() + (MSG[0] * MSG[3]));
        }

        if ((MSG[4] - 0x10) == 1)
        {
            axis4.set_actuator(axis4.get_current_angle() - (MSG[0] * (MSG[4] - 0x10)));
        }
        else
        {
            axis4.set_actuator(axis4.get_current_angle() + (MSG[0] * MSG[4]));
        }

        if ((MSG[5] - 0x10) == 1)
        {
            axis5.set_actuator(axis5.get_current_angle() - (MSG[0] * (MSG[5] - 0x10)));
        }
        else
        {
            axis5.set_actuator(axis5.get_current_angle() + (MSG[0] * MSG[5]));
        }

        if ((MSG[6] - 0x10) == 1)
        {
            axis6.set_actuator(axis6.get_current_angle() - (MSG[0] * (MSG[6] - 0x10)));
        }
        else
        {
            axis6.set_actuator(axis6.get_current_angle() + (MSG[0] * MSG[6]));
        }

        if ((MSG[7] - 0x10) == 1)
        {
            analogWrite(MOTOR_IN2, 0);
            analogWrite(MOTOR_IN1, 160);
            delay(30);
            analogWrite(MOTOR_IN1, 0);
            analogWrite(MOTOR_IN2, 0);
        }
        else if (MSG[7] == 1)
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

//
void loop()
{
    // Check if there are any CAN Bus messages in the buffer
    readMSG();
 
    // Send out the current actuator angles
    sendLowerPos();
    sendUpperPos();

    // Execute current commands
    run();
}

/*
void setupMove()
{
    uint32_t temp = findLargest();
    Serial.print("Largest: ");
    Serial.println(temp);
    if (axis1.get_steps_to_move() > 0)
    {
        delay1 = (temp / axis1.get_steps_to_move());
    }
    else
    {
        delay1 = 0;
    }

    if (axis2.get_steps_to_move() > 0)
    {
        delay2 = (temp / axis2.get_steps_to_move());
    }
    else
    {
        delay2 = 0;
    }

    if (axis3.get_steps_to_move() > 0)
    {
        //delay3 = (temp / axis3.get_steps_to_move());
    }
    else
    {
        //delay3 = 0;
    }

    if (axis4.get_steps_to_move() > 0)
    {
        //delay4 = (temp / axis4.get_steps_to_move());
    }
    else
    {
        //delay4 = 0;
    }

    if (axis5.get_steps_to_move() > 0)
    {
        //delay5 = (temp / axis5.get_steps_to_move());
    }
    else
    {
        //delay5 = 0;
    }

    if (axis6.get_steps_to_move() > 0)
    {
        //delay6 = (temp / axis6.get_steps_to_move());
    }
    else
    {
        //delay6 = 0;
    }


    axis1.move();
    axis2.move();
    axis3.move();
    axis4.move();
    axis5.move();
    axis6.move();

    digitalWrite(DIR_x1, axis1.get_actuator_direction());
    digitalWrite(DIR_y1, axis2.get_actuator_direction());
    digitalWrite(DIR_z1, axis3.get_actuator_direction());
    digitalWrite(DIR_x2, axis4.get_actuator_direction());
    digitalWrite(DIR_y2, axis5.get_actuator_direction());
    digitalWrite(DIR_z2, axis6.get_actuator_direction());

    runProg = true;
}


void moveAxis1()
{
    if (runProg == false)
    {
        return;
    }
    if (axis1.get_steps_to_move() > 0)
    {
        digitalWrite(DIR_y1, axis1.get_actuator_direction());

        if (millis() - timer1 > PULSE_SPEED + 40 && a1On == false)
        {
            a1On = true;
            axis1.reduceSteps();
            // pulse high
            digitalWrite(SPD_x1, true);
            timer1 = millis();
        }
        if (millis() - timer1 > SPEED_ADJUSTED_G1 && a1On == true)
        {
            a1On = false;
            // pulse low
            digitalWrite(SPD_x1, false);
            timer1 = millis();
        }
    }
}

void moveAxis2()
{
    if (runProg == false)
    {
        return;
    }

    if (axis2.get_steps_to_move() > 0)
    {
        digitalWrite(DIR_x1, axis2.get_actuator_direction());
        if (millis() - timer2 > PULSE_SPEED && a2On == false)
        {
            a2On = true;
            axis2.reduceSteps();
            // pulse high
            digitalWrite(SPD_x2, true);
            timer2 = millis();
        }
        if (millis() - timer2 > SPEED_ADJUSTED_G1 && a2On == true)
        {
            a2On = false;
            // pulse low
            digitalWrite(SPD_x2, false);
            timer2 = millis();
        }
    }
    else
    {

    }
}
*/