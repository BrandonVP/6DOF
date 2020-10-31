//#include <SdFat.h>
#include <SD.h>
#include <stdint.h>
#include <SPI.h>
#include "mcp_can.h"
#include "PinAssignments.cpp"
#include "Actuator.cpp"


// Global settings
const int PULSE_SPEED = 86;           // Lower number produces higher RPM
const int SPEED_ADJUSTED_G0 = PULSE_SPEED - 10;         // SPEED_ADJUSTED compensates time used for CPU to run logic which is approximately 18 ms
const int SPEED_ADJUSTED_G1 = PULSE_SPEED - 20;         // SPEED_ADJUSTED compensates time used for CPU to run logic which is approximately 18 ms
const int SPEED_ADJUSTED_G2 = PULSE_SPEED - 10;         // SPEED_ADJUSTED compensates time used for CPU to run logic which is approximately 18 ms

const int NUMBER_OF_ACTUATORS = 6;    // Number of actuators in robot

File myFile;

// Actuator objects
// Base x, y, z, 1
Actuator actuator_x1;
Actuator actuator_y1;
Actuator actuator_z1;
// Wrist x, y, z, 2
Actuator actuator_x2;
Actuator actuator_y2;
Actuator actuator_z2;


// START CANBUS
#define CAN0_INT 47                              // Set INT to pin 2
const unsigned int rxId_send = 0x0C1;
const unsigned int rxId_control = 0x0B0;
const unsigned int rxId_lower = 0x0B1;
const unsigned int rxId_upper = 0x0B2;
const unsigned int rxId_manual = 0x0B3;
INT8U len = 0;
INT8U rxBuf[8];
char msgString[128];                        // Array to store serial string
MCP_CAN CAN0(49);                               // Set CS to pin 49
bool isGrip = true;

void close_grip() {
    analogWrite(MOTOR_IN2, 0);
    analogWrite(MOTOR_IN1, 255);
    delay(1);
    for (int i = 188; i >= 0; i--) {
        analogWrite(MOTOR_IN1, i);
        delay(6);
    }
    analogWrite(MOTOR_IN1, 0);
    analogWrite(MOTOR_IN2, 0);
}

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

void demo1() {
    actuator_x1.set_actuator(15);
    actuator_y1.set_actuator(15);
    actuator_z1.set_actuator(-15);
    actuator_x2.set_actuator(7);
    actuator_y2.set_actuator(7);
    actuator_z2.set_actuator(7);
    G1();
    actuator_x1.set_actuator(-15);
    actuator_y1.set_actuator(-15);
    actuator_z1.set_actuator(15);
    actuator_x2.set_actuator(-7);
    actuator_y2.set_actuator(-7);
    actuator_z2.set_actuator(-7);
    G1();
    actuator_x1.set_actuator(1);
    actuator_y1.set_actuator(1);
    actuator_z1.set_actuator(1);
    actuator_x2.set_actuator(1);
    actuator_y2.set_actuator(1);
    actuator_z2.set_actuator(1);
    G1();
}

void demo2() {
    actuator_x1.set_actuator(1);
    actuator_y1.set_actuator(40);
    actuator_z1.set_actuator(-40);
    actuator_x2.set_actuator(1);
    actuator_y2.set_actuator(1);
    actuator_z2.set_actuator(1);
    G1();
    open_grip();
    actuator_x1.set_actuator(1);
    actuator_y1.set_actuator(64);
    actuator_z1.set_actuator(-40);
    actuator_x2.set_actuator(2);
    actuator_y2.set_actuator(1);
    actuator_z2.set_actuator(1);
    G1();
    close_grip();
    delay(1000);
    actuator_x1.set_actuator(40);
    actuator_y1.set_actuator(1);
    actuator_z1.set_actuator(1);
    actuator_x2.set_actuator(-4);
    actuator_y2.set_actuator(1);
    actuator_z2.set_actuator(1);
    G1();
    actuator_x1.set_actuator(40);
    actuator_y1.set_actuator(64);
    actuator_z1.set_actuator(-40);
    actuator_x2.set_actuator(4);
    actuator_y2.set_actuator(1);
    actuator_z2.set_actuator(1);
    G1();
    open_grip();
    actuator_x1.set_actuator(40);
    actuator_y1.set_actuator(1);
    actuator_z1.set_actuator(1);
    actuator_x2.set_actuator(1);
    actuator_y2.set_actuator(1);
    actuator_z2.set_actuator(1);
    G1();
    actuator_x1.set_actuator(1);
    actuator_y1.set_actuator(1);
    actuator_z1.set_actuator(1);
    actuator_x2.set_actuator(1);
    actuator_y2.set_actuator(1);
    actuator_z2.set_actuator(1);
    G1();
    close_grip();
}


bool CANBUS() {
    INT32U rxId;
    byte returnData[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    while (true) {
        if (!digitalRead(CAN0_INT))                          // If CAN0_INT pin is low, read receive buffer
        {
            byte recStat = CAN0.readMsgBuf(&rxId, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)
            switch (rxId)
            {
            case rxId_control:
                CAN0.sendMsgBuf(rxId_send, 0, 8, returnData);
                G1();
                break;
            case rxId_lower:
                CAN0.sendMsgBuf(rxId_send, 0, 8, returnData);
                if ((rxBuf[2] + rxBuf[3]) > 0) {
                    actuator_x1.set_actuator(rxBuf[2] + rxBuf[3]);
                }
                if ((rxBuf[4] + rxBuf[5]) > 0) {
                    actuator_y1.set_actuator(rxBuf[4] + rxBuf[5]);
                }
                if ((rxBuf[6] + rxBuf[7]) > 0) {
                    actuator_z1.set_actuator(rxBuf[6] + rxBuf[7]);
                }
                break;
            case rxId_upper:
                CAN0.sendMsgBuf(rxId_send, 0, 8, returnData);
                if ((rxBuf[2] + rxBuf[3]) > 0) {
                    actuator_x2.set_actuator(rxBuf[2] + rxBuf[3]);
                }
                if ((rxBuf[4] + rxBuf[5]) > 0) {
                    actuator_y2.set_actuator(rxBuf[4] + rxBuf[5]);
                }
                if ((rxBuf[6] + rxBuf[7]) > 0) {
                    actuator_z2.set_actuator(rxBuf[6] + rxBuf[7]);
                }
                break;
            case rxId_manual:
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
                G1();
                break;
            }
        }
    }
    return true;
}
// END CANBUS



void setup() {
    // Start Serial Monitor
    Serial.begin(115200);

    // Start CANBus
    // Initialize MCP2515 running at 8MHz with a baudrate of 500kb/s and the masks and filters disabled.
    if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK)
        Serial.println("MCP2515 Initialized Successfully!");
    else
        Serial.println("Error Initializing MCP2515...");
    CAN0.setMode(MCP_NORMAL);                     // Set operation mode to normal so the MCP2515 sends acks to received data.
    Serial.println("MCP2515 Library Receive Example...");
    // End CANBus

    /*
    // Start SD
    Serial.print("Initializing SD card...");
    if (!SD.begin(47)) {
        Serial.println("initialization failed. Things to check:");
        Serial.println("1. is a card inserted?");
        Serial.println("2. is your wiring correct?");
        Serial.println("3. did you change the chipSelect pin to match your shield or module?");
        Serial.println("Note: press reset or reopen this serial monitor after fixing your issue!");
        //while (1);
    }
    */
    Serial.println("initialization done.");
    // End SD

    // Start Actuator Pin Assignements
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
    // End Actuator Pin Assignements

    // Temporary starting position to be replaces with sd card
    actuator_x1.set_current_angle(180);
    actuator_y1.set_current_angle(180);
    actuator_z1.set_current_angle(180); 
    actuator_x2.set_current_angle(180);
    actuator_y2.set_current_angle(180);
    actuator_z2.set_current_angle(180);
    
    pinMode(MOTOR_IN1, OUTPUT);
    pinMode(MOTOR_IN2, OUTPUT);
}

void loop() {
    //demo1();
    //demo2();
    //close_grip();
    //open_grip();
    //actuator_x1.set_actuator(0xB9);
    //G1();
    //actuator_x1.set_actuator(0xB4);
    //G1();
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
void G0(void) {

}


/*
****************************************************
*   Function: G1()                                 *
*    - Each actuator starts moving at the same     *
*      time and finishes when individual distance  *
*      is reached                                  *
*   Parameters: void                               *
*                                                  *
*   Returns: void                                  *
****************************************************
*/

void G1(void) {
    int long index = 0;
    digitalWrite(DIR_x1, actuator_x1.get_actuator_direction());
    digitalWrite(DIR_y1, actuator_y1.get_actuator_direction());
    digitalWrite(DIR_z1, actuator_z1.get_actuator_direction());
    digitalWrite(DIR_x2, actuator_x2.get_actuator_direction());
    digitalWrite(DIR_y2, actuator_y2.get_actuator_direction());
    digitalWrite(DIR_z2, actuator_z2.get_actuator_direction());

    int acceleration = 100;
    while ((index < actuator_x1.get_steps_to_move()) || (index < actuator_y1.get_steps_to_move()) || (index < actuator_z1.get_steps_to_move())
        || (index < actuator_x2.get_steps_to_move()) || (index < actuator_y2.get_steps_to_move()) || (index < actuator_z2.get_steps_to_move()))
    {
        if ((index < actuator_x1.get_steps_to_move())) {
            //actuator_x1.enable_actuator = true;
            digitalWrite(SPD_x1, true);
        }
        if ((index < actuator_x2.get_steps_to_move())) {
            //actuator_x1.enable_actuator = true;
            digitalWrite(SPD_x2, true);
        }
        if (index < actuator_y1.get_steps_to_move()) {
            // actuator_y1.enable_actuator = false;
            digitalWrite(SPD_y1, true);
        }
        if (index < actuator_y2.get_steps_to_move()) {
            // actuator_y1.enable_actuator = false;
            digitalWrite(SPD_y2, true);
        }
        if (index < actuator_z1.get_steps_to_move()) {
            //actuator_z1.enable_actuator = false;
            digitalWrite(SPD_z1, true);
        }
        if (index < actuator_z2.get_steps_to_move()) {
            //actuator_z1.enable_actuator = false;
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

void G2(void) {
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
