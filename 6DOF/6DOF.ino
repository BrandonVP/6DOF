//#include <SdFat.h>
//#include <SD.h>
//#include <stdint.h>
#include <SPI.h>
#include "mcp_can.h"
#include "PinAssignments.cpp"
#include "Actuator.cpp"


// Global settings
const int PULSE_SPEED = 135;           // Lower number produces higher RPM
const int SPEED_ADJUSTED_G0 = PULSE_SPEED - 10;         // SPEED_ADJUSTED compensates time used for CPU to run logic
const int SPEED_ADJUSTED_G1 = PULSE_SPEED - 24;         // SPEED_ADJUSTED compensates time used for CPU to run logic in G1 loop
const int SPEED_ADJUSTED_G2 = PULSE_SPEED - 10;         // SPEED_ADJUSTED compensates time used for CPU to run logic
//const int NUMBER_OF_ACTUATORS = 6;    

// Should this be moved?
//File myFile;

// Actuator objects
// Base x, y, z, 1
Actuator actuator_x1;
Actuator actuator_y1;
Actuator actuator_z1;
// Wrist x, y, z, 2
Actuator actuator_x2;
Actuator actuator_y2;
Actuator actuator_z2;

#define ANGLE_ACCELERATION 500
#define MANUAL_ACCELERATION 1

// CAN Bus settings
#define CAN0_INT 47      
#define RXID_SEND    0x0C1
#define RXID_CONTROL 0x0A0
#define RXID_LOWER   0x0A1
#define RXID_UPPER   0x0A2
#define RX_MANUAL    0x0A3
INT8U len = 0;
INT8U rxBuf[8];

// CS Pin
MCP_CAN CAN0(49);

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
    delay(340);
    //for (int i = 255; i >= 0; i--) {
    //    analogWrite(MOTOR_IN2, i);
    //    delay(1);
    //}
    analogWrite(MOTOR_IN1, 0);
    analogWrite(MOTOR_IN2, 0);
}

int holdup = 0;
// Read in commands from CAN Bus using 1 of 4 IDs
bool CANBUS() {
    INT32U rxId;
    // Empty return array used to confirm message recieved
    byte returnData[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    while (true) {
        // If CAN0_INT pin is low, read receive buffer
        if (!digitalRead(CAN0_INT))                          
        {
            byte recStat = CAN0.readMsgBuf(&rxId, &len, rxBuf);
            Serial.print("ID: ");
            rxId = (uint16_t)rxId;
            Serial.print(rxId);
            Serial.print(" MSG: ");
            Serial.println(rxBuf[0]);
            switch (rxId)
            {
            case RXID_CONTROL:
                /*=========================================================
                            Return Current Lower Axis Positions
                ===========================================================*/
                if (rxBuf[1] == 1)
                {
                    uint8_t temp;

                    returnData[0] = 0x01;
                    temp = actuator_x1.get_current_angle();
                    if (temp > 255)
                    {
                        returnData[2] = 0x01;
                        returnData[3] = temp - 0xFF;
                    }
                    else
                    {
                        returnData[3] = temp;
                    }
                    temp = actuator_y1.get_current_angle();
                    if (temp > 255)
                    {
                        returnData[4] = 0x01;
                        returnData[5] = temp - 0xFF;
                    }
                    else
                    {
                        returnData[5] = temp;
                    }
                    temp = actuator_z1.get_current_angle();
                    if (temp > 255)
                    {
                        returnData[6] = 0x01;
                        returnData[7] = temp - 0xFF;
                    }
                    else
                    {
                        returnData[7] = temp;
                    }      
                    CAN0.sendMsgBuf(RXID_SEND, 0, 8, returnData);

                    returnData[0] = 0x00;
                    returnData[1] = 0x00;
                    returnData[2] = 0x00;
                    returnData[3] = 0x00;
                    returnData[4] = 0x00;
                    returnData[5] = 0x00;
                    returnData[6] = 0x00;
                    returnData[7] = 0x00;
                }

                /*=========================================================
                            Return Current Higher Axis Positions
                ===========================================================*/
                if (rxBuf[1] == 2)
                {
                    uint8_t temp;

                    returnData[0] = 0x02;
                    temp = actuator_x2.get_current_angle();
                    if (temp > 255)
                    {
                        returnData[2] = 0x01;
                        returnData[3] = temp - 0xFF;
                    }
                    else
                    {
                        returnData[3] = temp;
                    }
                    temp = actuator_y2.get_current_angle();
                    if (temp > 255)
                    {
                        returnData[4] = 0x01;
                        returnData[5] = temp - 0xFF;
                    }
                    else
                    {
                        returnData[5] = temp;
                    }
                    temp = actuator_z2.get_current_angle();
                    if (temp > 255)
                    {
                        returnData[6] = 0x01;
                        returnData[7] = temp - 0xFF;
                    }
                    else
                    {
                        returnData[7] = temp;
                    }
                    CAN0.sendMsgBuf(RXID_SEND, 0, 8, returnData);

                    returnData[0] = 0x00;
                    returnData[1] = 0x00;
                    returnData[2] = 0x00;
                    returnData[3] = 0x00;
                    returnData[4] = 0x00;
                    returnData[5] = 0x00;
                    returnData[6] = 0x00;
                    returnData[7] = 0x00;
                }

                /*=========================================================
                            Set Axis Angles to Current Postion
                ===========================================================*/
                if (rxBuf[1] == 3)
                {
                    actuator_x1.set_current_angle(0xB4);
                    actuator_y1.set_current_angle(0xB4);
                    actuator_z1.set_current_angle(0x5A);
                    actuator_x2.set_current_angle(0xB4);
                    actuator_y2.set_current_angle(0xB4);
                    actuator_z2.set_current_angle(0xB4);
                }

                /*=========================================================
                            Open/Close Grip
                ===========================================================*/
                if (rxBuf[6] == 1)
                {
                    open_grip();
                }
                if (rxBuf[7] == 1)
                {
                    close_grip();
                }
                break;

                /*=========================================================
                            Executes Move
                ===========================================================*/
                if (rxBuf[0] == 1)
                {
                    delay(100);
                    returnData[1] = rxBuf[1];
                    CAN0.sendMsgBuf(RXID_SEND, 0, 8, returnData);
                    delay(50);
                    G1(ANGLE_ACCELERATION);
                }
            case RXID_LOWER:
                delay(50);
                if ((rxBuf[2] + rxBuf[3]) > 0) {
                    actuator_x1.set_actuator(rxBuf[2] + rxBuf[3]);
                }
                if ((rxBuf[4] + rxBuf[5]) > 0) {
                    actuator_y1.set_actuator(rxBuf[4] + rxBuf[5]);
                }
                if ((rxBuf[6] + rxBuf[7]) > 0) {
                    actuator_z1.set_actuator(rxBuf[6] + rxBuf[7]);
                }
                returnData[1] = rxBuf[1];
                CAN0.sendMsgBuf(RXID_SEND, 0, 8, returnData);
                delay(50);
                break;
            case RXID_UPPER:
                delay(50);
                if ((rxBuf[2] + rxBuf[3]) > 0) {
                    actuator_x2.set_actuator(rxBuf[2] + rxBuf[3]);
                }
                if ((rxBuf[4] + rxBuf[5]) > 0) {
                    actuator_y2.set_actuator(rxBuf[4] + rxBuf[5]);
                }
                if ((rxBuf[6] + rxBuf[7]) > 0) {
                    actuator_z2.set_actuator(rxBuf[6] + rxBuf[7]);
                }
                delay(50);
                returnData[1] = rxBuf[1];
                CAN0.sendMsgBuf(RXID_SEND, 0, 8, returnData);
                break;
            case RX_MANUAL:
                if ((rxBuf[1] - 0x10) == 1)
                {
                    actuator_x1.set_actuator(actuator_x1.get_current_angle() - (rxBuf[0] * (rxBuf[1] - 0x10)));
                }
                else
                {
                    actuator_x1.set_actuator(actuator_x1.get_current_angle() + (rxBuf[0] * rxBuf[1]));
                }

                if ((rxBuf[2] - 0x10) == 1)
                {
                    actuator_y1.set_actuator(actuator_y1.get_current_angle() - (rxBuf[0] * (rxBuf[2] - 0x10)));
                }
                else
                {
                    actuator_y1.set_actuator(actuator_y1.get_current_angle() + (rxBuf[0] * rxBuf[2]));
                }

                if ((rxBuf[3] - 0x10) == 1)
                {
                    actuator_z1.set_actuator(actuator_z1.get_current_angle() - (rxBuf[0] * (rxBuf[3] - 0x10)));
                }
                else
                {
                    actuator_z1.set_actuator(actuator_z1.get_current_angle() + (rxBuf[0] * rxBuf[3]));
                }

                if ((rxBuf[4] - 0x10) == 1)
                {
                    actuator_x2.set_actuator(actuator_x2.get_current_angle() - (rxBuf[0] * (rxBuf[4] - 0x10)));
                }
                else
                {
                    actuator_x2.set_actuator(actuator_x2.get_current_angle() + (rxBuf[0] * rxBuf[4]));
                }

                if ((rxBuf[5] - 0x10) == 1)
                {
                    actuator_y2.set_actuator(actuator_y2.get_current_angle() - (rxBuf[0] * (rxBuf[5] - 0x10)));
                }
                else
                {
                    actuator_y2.set_actuator(actuator_y2.get_current_angle() + (rxBuf[0] * rxBuf[5]));
                }

                if ((rxBuf[6] - 0x10) == 1)
                {
                    actuator_z2.set_actuator(actuator_z2.get_current_angle() - (rxBuf[0] * (rxBuf[6] - 0x10)));
                }
                else
                {
                    actuator_z2.set_actuator(actuator_z2.get_current_angle() + (rxBuf[0] * rxBuf[6]));
                }

                if ((rxBuf[7] - 0x10) == 1)
                {
                        analogWrite(MOTOR_IN2, 0);
                        analogWrite(MOTOR_IN1, 160);
                        delay(30);
                        analogWrite(MOTOR_IN1, 0);
                        analogWrite(MOTOR_IN2, 0);
                }
                else if (rxBuf[7] == 1)
                {
                        analogWrite(MOTOR_IN1, 0);
                        analogWrite(MOTOR_IN2, 255);
                        delay(35);
                        analogWrite(MOTOR_IN1, 0);
                        analogWrite(MOTOR_IN2, 0);
                }
                G1(MANUAL_ACCELERATION);
                break;
            }
        }
    }
    return true;
}

// Mega2560 setup
void setup() {
    // Start Serial Monitor
    Serial.begin(115200);

    // Start CANBus
    // Initialize MCP2515 running at 8MHz with a baudrate of 500kb/s and the masks and filters disabled.
    if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK)
        Serial.println(F("MCP2515 Initialized Successfully!"));
    else
        Serial.println(F("Error Initializing MCP2515..."));
    CAN0.setMode(MCP_NORMAL);                     // Set operation mode to normal so the MCP2515 sends acks to received data.
    pinMode(CAN0_INT, INPUT);
    //Serial.println(F("initialization done."));

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
    actuator_x1.set_current_angle(0xB4);
    actuator_y1.set_current_angle(0xB4);
    actuator_z1.set_current_angle(0x5A);
    actuator_x2.set_current_angle(0xB4);
    actuator_y2.set_current_angle(0xB4);
    actuator_z2.set_current_angle(0xB4);
    //Serial.end();
}

// Main loop - Calls CANBUS
void loop() {
    //actuator_x1.set_actuator(110);
    //actuator_y1.set_actuator(110);
    //G1(400);
    //actuator_x1.set_actuator(250);
    //actuator_y1.set_actuator(250);
    //G1(400);
    
    CANBUS();
}

/*
****************************************************
*   Function: G0()                                 *
*    - Each actuator moves one at a time starting  *
*      from the bottom up                          *
*   Parameters: void                               *
*                                                  *
*   Returns: void                                  *
****************************************************
*/
/*
void G0(void) {

}
*/

/*
****************************************************
*   Function: G1()                                 *
*    - Each actuator starts moving at the same     *
*      time and finishes when individual distance  *
*      is reached                                  *
*   Parameters: Acceleration                       *
*                                                  *
*   Returns: void                                  *
****************************************************
*/
void G1(int acceleration) {
    int long index = 0;
    digitalWrite(DIR_x1, actuator_x1.get_actuator_direction());
    digitalWrite(DIR_y1, actuator_y1.get_actuator_direction());
    digitalWrite(DIR_z1, actuator_z1.get_actuator_direction());
    digitalWrite(DIR_x2, actuator_x2.get_actuator_direction());
    digitalWrite(DIR_y2, actuator_y2.get_actuator_direction());
    digitalWrite(DIR_z2, actuator_z2.get_actuator_direction());

    actuator_x1.move();
    actuator_y1.move();
    actuator_z1.move();
    actuator_x2.move();
    actuator_y2.move();
    actuator_z2.move();

    while ((index < actuator_x1.get_steps_to_move()) || (index < actuator_y1.get_steps_to_move()) || (index < actuator_z1.get_steps_to_move())
        || (index < actuator_x2.get_steps_to_move()) || (index < actuator_y2.get_steps_to_move()) || (index < actuator_z2.get_steps_to_move()))
    {
        if ((index < actuator_x1.get_steps_to_move())) {
            //actuator_x1.enable_actuator = true;
            digitalWrite(SPD_x1, true);
        }
        if ((index < actuator_x2.get_steps_to_move())) {
            //actuator_x2.enable_actuator = true;
            digitalWrite(SPD_x2, true);
        }
        if (index < actuator_y1.get_steps_to_move()) {
            // actuator_y1.enable_actuator = false;
            digitalWrite(SPD_y1, true);
        }
        if (index < actuator_y2.get_steps_to_move()) {
            // actuator_y2.enable_actuator = false;
            digitalWrite(SPD_y2, true);
        }
        if (index < actuator_z1.get_steps_to_move()) {
            //actuator_z1.enable_actuator = false;
            digitalWrite(SPD_z1, true);
        }
        if (index < actuator_z2.get_steps_to_move()) {
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
            acceleration = acceleration - 1;
        }
    }
    return;
}

/*
****************************************************
*   Function: G2()                       *
*    - Each actuator moves and finishes at same    *
*    - time by modifying speed of each actuator    *
*                                                  *
*   Parameters: nonev                              *
*                                                  *
*   Returns: void                                  *
****************************************************
*/
/*
void G2(void) {
        // Under Development
    int step_array[NUMBER_OF_ACTUATORS] = { actuator_x1.get_steps_to_move(), actuator_y1.get_steps_to_move(), actuator_z1.get_steps_to_move(), actuator_x2.get_steps_to_move(), actuator_y2.get_steps_to_move(), actuator_z2.get_steps_to_move() };
    int long index = 0;
    int greatest_value = step_array[0];
    for (int i = 1; i < NUMBER_OF_ACTUATORS; i++)
    {
        if (greatest_value < step_array[i])
            greatest_value = step_array[i];
    }
    double k_constant = greatest_value * PULSE_SPEED;
    actuator_x1.set_actuator_speed((k_constant / (step_array[0] * STEPS_PER_DEGREE)));
    actuator_y1.set_actuator_speed((k_constant / (step_array[1] * STEPS_PER_DEGREE)));
    actuator_z1.set_actuator_speed((k_constant / (step_array[2] * STEPS_PER_DEGREE)));
    actuator_x2.set_actuator_speed((k_constant / (step_array[3] * STEPS_PER_DEGREE)));
    actuator_y2.set_actuator_speed((k_constant / (step_array[4] * STEPS_PER_DEGREE)));
    actuator_z2.set_actuator_speed((k_constant / (step_array[5] * STEPS_PER_DEGREE)));

    while (index < greatest_value) {
        digitalWrite(SPD_x1, true);
        digitalWrite(SPD_x2, true);
        delayMicroseconds(PULSE_SPEED);
        digitalWrite(SPD_x1, false);
        digitalWrite(SPD_x2, false);
        delayMicroseconds(SPEED_ADJUSTED_G2);
        index++;
    }
}
*/
