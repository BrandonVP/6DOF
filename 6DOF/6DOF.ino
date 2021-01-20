/*
 Name:		_6DOF_V2.ino
 Created:	1/10/2021 3:31:56 PM
 Author:	Brandon Van Pelt
*/

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
#define LED_CAN_TX LED_BUILTIN
#define ANGLE_ACCELERATION 500
#define MANUAL_ACCELERATION 1

// Global settings
constexpr auto PULSE_SPEED = 135;                            // Lower number produces higher RPM
constexpr auto SPEED_ADJUSTED_G0 = PULSE_SPEED - 10;         // SPEED_ADJUSTED compensates time used for CPU to run logic
constexpr auto SPEED_ADJUSTED_G1 = PULSE_SPEED - 24;         // SPEED_ADJUSTED compensates time used for CPU to run logic in G1 loop
constexpr auto SPEED_ADJUSTED_G2 = PULSE_SPEED - 10;         // SPEED_ADJUSTED compensates time used for CPU to run logic

//
long unsigned int rxId;
INT8U len = 0;
INT8U rxBuf[8];

MCP_CAN CAN0(49);

// Linked list of nodes for a program
LinkedList<CANBuffer*> buffer;

// Actuator objects
Actuator axis1;
Actuator axis2;
Actuator axis3;
Actuator axis4;
Actuator axis5;
Actuator axis6;

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

    CAN0.setMode(MCP_NORMAL);                // Change to normal mode to allow messages to be transmitted

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

    // Set starting angle for the actuators
    axis1.set_current_angle(0xB4);
    axis2.set_current_angle(0xB4);
    axis3.set_current_angle(0x5A);
    axis4.set_current_angle(0xB4);
    axis5.set_current_angle(0xB4);
    axis6.set_current_angle(0xB4);
    Serial.end();
    attachInterrupt(digitalPinToInterrupt(INT_PIN), MSGBuff, FALLING);
}

//
void run(uint16_t acceleration)
{
    //Serial.println("run");

    axis1.move();
    axis2.move();
    axis3.move();
    axis4.move();
    axis5.move();
    axis6.move();

    uint32_t index = 0;

    digitalWrite(DIR_x1, axis1.get_actuator_direction());
    digitalWrite(DIR_y1, axis2.get_actuator_direction());
    digitalWrite(DIR_z1, axis3.get_actuator_direction());
    digitalWrite(DIR_x2, axis4.get_actuator_direction());
    digitalWrite(DIR_y2, axis5.get_actuator_direction());
    digitalWrite(DIR_z2, axis6.get_actuator_direction());

    while ((index < axis1.get_steps_to_move()) || (index < axis2.get_steps_to_move()) || (index < axis3.get_steps_to_move())
        || (index < axis4.get_steps_to_move()) || (index < axis5.get_steps_to_move()) || (index < axis6.get_steps_to_move()))
    {
        if ((index < axis1.get_steps_to_move())) {
            //actuator_x1.enable_actuator = true;
            digitalWrite(SPD_x1, true);
        }
        if ((index < axis4.get_steps_to_move())) {
            //actuator_x2.enable_actuator = true;
            digitalWrite(SPD_x2, true);
        }
        if (index < axis2.get_steps_to_move()) {
            // actuator_y1.enable_actuator = false;
            digitalWrite(SPD_y1, true);
        }
        if (index < axis5.get_steps_to_move()) {
            // actuator_y2.enable_actuator = false;
            digitalWrite(SPD_y2, true);
        }
        if (index < axis3.get_steps_to_move()) {
            //actuator_z1.enable_actuator = false;
            digitalWrite(SPD_z1, true);
        }
        if (index < axis6.get_steps_to_move()) {
            //actuator_z2.enable_actuator = false;
            digitalWrite(SPD_z2, true);
        }

        delayMicroseconds(PULSE_SPEED + acceleration);
        digitalWrite(SPD_x1, false);
        digitalWrite(SPD_x2, false);
        digitalWrite(SPD_y1, false);
        digitalWrite(SPD_y2, false);
        digitalWrite(SPD_z1, false);
        digitalWrite(SPD_z2, false);
        delayMicroseconds(SPEED_ADJUSTED_G1 + acceleration);
        index++;
        if (acceleration > 0) {
            acceleration--;
        }
    }
    axis1.set_steps_to_move(0);
    axis2.set_steps_to_move(0);
    axis3.set_steps_to_move(0);
    axis4.set_steps_to_move(0);
    axis5.set_steps_to_move(0);
    axis6.set_steps_to_move(0);
}

// Attached to interupt - Incoming CAN Bus frame saved to buffer
void MSGBuff()
{
    // Read incoming message
    CAN0.readMsgBuf(&rxId, &len, rxBuf);      

    // Create object to store message
    CANBuffer* node = new CANBuffer(rxId, rxBuf);

    // Insert object into linked list
    buffer.InsertHead(node);
}

// Read messages from buffer
void readMSG()
{
    if (buffer.GetSize() > 0)
    {
        //Serial.println("readMSG()");

        // Retrieve the first message received
        uint16_t ID = buffer.GetTail()->getID();
        uint8_t* MSG = buffer.GetTail()->getMessage();

        // Send the message to the controller for processing
        controller(ID, MSG);

        // Delete used object from list
        buffer.RemoveTail();

        

        // Debug
        /*
        Serial.print("ID ");
        Serial.print(ID, HEX);
        Serial.print(" MSG ");
        Serial.print(MSG[0], HEX);
        Serial.print(" ");
        Serial.print(MSG[1], HEX);
        Serial.print(" ");
        Serial.print(MSG[2], HEX);
        Serial.print(" ");
        Serial.print(MSG[3], HEX);
        Serial.print(" ");
        Serial.print(MSG[4], HEX);
        Serial.print(" ");
        Serial.print(MSG[5], HEX);
        Serial.print(" ");
        Serial.print(MSG[6], HEX);
        Serial.print(" ");
        Serial.println(MSG[7], HEX);
        */
    }
    return;
}

// Processes incoming CAN Frames
void controller(uint16_t ID, uint8_t* MSG)
{
    //Serial.println("controller");
    // Used for return confirmation
    byte returnData[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

    // Debug
    /*
    Serial.print("ID: ");
    Serial.print(ID, HEX);
    Serial.print(" MSG: ");
    Serial.print(MSG[0], HEX);
    Serial.print(" ");
    Serial.print(MSG[1], HEX);
    Serial.print(" ");
    Serial.print(MSG[2], HEX);
    Serial.print(" ");
    Serial.print(MSG[3], HEX);
    Serial.print(" ");
    Serial.print(MSG[4], HEX);
    Serial.print(" ");
    Serial.print(MSG[5], HEX);
    Serial.print(" ");
    Serial.print(MSG[6], HEX);
    Serial.print(" ");
    Serial.println(MSG[7], HEX);
    */

    // 
    switch (ID)
    {
    case RXID_CONTROL:
        /*=========================================================
                    Return Current Lower Axis Positions
        ===========================================================*/
        if (MSG[1] == 0x01)
        {
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
            // Confirmation
            returnData[0] = 0x01;

            // Return axis positions for bottom three axis
            CAN0.sendMsgBuf(RXID_SEND, 0, 8, returnData);

            // Reset returnData back to zero values
            returnData[0] = 0x00;
            returnData[1] = 0x00;
            returnData[2] = 0x00;
            returnData[3] = 0x00;
            returnData[4] = 0x00;
            returnData[5] = 0x00;
            returnData[6] = 0x00;
            returnData[7] = 0x00;

            break;
        }

        /*=========================================================
                    Return Current Higher Axis Positions
        ===========================================================*/
        if (MSG[1] == 0x02)
        {
            uint8_t temp;

            returnData[0] = 0x02;
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

            returnData[0] = 0x00;
            returnData[1] = 0x00;
            returnData[2] = 0x00;
            returnData[3] = 0x00;
            returnData[4] = 0x00;
            returnData[5] = 0x00;
            returnData[6] = 0x00;
            returnData[7] = 0x00;
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
            delay(10);
            run(ANGLE_ACCELERATION);
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
        run(MANUAL_ACCELERATION);
        break;
    }
}

// the loop function runs over and over again until power down or reset
void loop()
{
    readMSG();
}
