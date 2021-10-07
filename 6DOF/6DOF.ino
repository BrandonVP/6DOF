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

Determine what is locking up device
- Stuck in loop?
- Power problem?
- Error?
- CAN Hardware?

Replace delayMicroseconds with system timer
- Holding millis() value with int is too slow
- Register uint32_t shows 0 when assigned millis()
- Try using system timer

Try to reduce memory usage

Design way for arm movements of different lengths end together

Grips need fixed / added hardware
===========================================================
    End Todo List
=========================================================*/

// Uncomment an arm for upload
#define ARM1
//#define ARM2

#include "Actuator.h"
#include <mcp_can_dfs.h>
#include <mcp_can.h>
#include "pinAssignments.h"
#include <SPI.h>
#include "can_buffer.h"
#if defined ARM1
    #include "ch1.h"
#endif
#if defined ARM2
    #include "ch2.h"
#endif

//#include "CANBuffer.h"
//#include <LinkedListLib.h>

#define BUFFER_SIZE 20
#define REFRESH_RATE 200
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

// CAN Bus vars
long unsigned int rxId;
INT8U len = 0;
INT8U rxBuf[8];

MCP_CAN CAN0(49);

// Linked list of nodes for a program
//LinkedList<CANBuffer*> buffer;

// Actuator objects - (min angle, max angle, current angle)
Actuator axis1(40, 320, axis1StartingAngle);
Actuator axis2(60, 300, axis2StartingAngle);
Actuator axis3(45, 135, axis3StartingAngle);
Actuator axis4(160, 200, axis4StartingAngle);
Actuator axis5(160, 200, axis5StartingAngle);
Actuator axis6(160, 200, axis6StartingAngle);

bool hasAcceleration = true;
bool runSetup = false;
bool runProg = false;
bool delayState = true;
bool isGripOpen = true;
bool eStopActivated = false;

// Run() vars
uint16_t count = 0;
uint16_t acceleration = 0;
uint32_t maxStep = 0;
volatile uint32_t runIndex = 0;

// Timer for current angle updates
uint32_t timer = 0;

// Wait function variables
uint32_t waitTimer = 0;
bool waitActivated = false;

// Buffer variables
can_buffer myStack;
CAN_Frame incoming;
CAN_Frame buffer;

int freeRam() {
    extern int __heap_start, * __brkval;
    int v;
    return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}

/*=========================================================
    CAN Bus Traffic
===========================================================*/
// Attached to interupt - Incoming CAN Bus frame saved to buffer
void MSGBuff()
{
    if (!digitalRead(INT_PIN)) // If CAN0_INT pin is low, read receive buffer
    {
        // Read incoming message
        CAN0.readMsgBuf(&rxId, &len, rxBuf);

        if (rxBuf[1] == 0x04)
        {
            if (rxBuf[2] == 0x02)
            {
                eStopActivated = true;
            }
            else if (rxBuf[2] == 1)
            {
                eStopActivated = false;
            }
        }

        if ((rxId == 0xA0 || rxId == 0xB0) && (rxBuf[1] == 0x1 || rxBuf[1] == 0x02))
        {
            sendLowerPos();
            sendUpperPos();
            return;
        }

        incoming.id = rxId;
        for (uint8_t i = 0; i < 8; i++)
        {
            incoming.data[i] = rxBuf[i];
        }
        myStack.push(incoming);
    }
}

// Read messages from buffer
void readMSG()
{
    if (runProg == false && myStack.stack_size() > 0)
    {
        myStack.pop(&buffer);
        // Send the message to the controller for processing
        controller(buffer);
    }
}

// Process incoming CAN Frames
void controller(CAN_Frame buffer)
{
    // Used for return confirmation
    byte returnData[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

    // 
    switch (buffer.id)
    {
    case RXID_CONTROL:
        /*=========================================================
                    Send Current Lower Axis Positions
        ===========================================================*/
        if (buffer.data[1] == 0x01)
        {
            sendLowerPos();
            break;
        }

        /*=========================================================
                    Send Current Higher Axis Positions
        ===========================================================*/
        if (buffer.data[1] == 0x02)
        {
            sendUpperPos();
            break;
        }

        /*=========================================================
                    Set Axis Angles to Current Postion
        ===========================================================*/
        if (buffer.data[1] == 0x03)
        {
            axis1.set_current_angle(0xB4);
            axis2.set_current_angle(0xB4);
            axis3.set_current_angle(0x5A);
            axis4.set_current_angle(0xB4);
            axis5.set_current_angle(0xB4);
            axis6.set_current_angle(0xB4);
        }

        /*=========================================================
                    Set Wait Timer
        ===========================================================*/
        if (buffer.data[1] == 0x0A)
        {
            uint8_t seconds = buffer.data[7];
            uint8_t minutes = buffer.data[6];

        }


        /*=========================================================
                    Open/Close Grip
        ===========================================================*/
        if (buffer.data[6] == 0x01)
        {
            open_grip();
        }
        if (buffer.data[7] == 0x01)
        {
            close_grip();
        }

        /*=========================================================
                    Executes Move
        ===========================================================*/
        if (buffer.data[0] == 0x01)
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
        /*=========================================================
               Set next angles for bottom three axis
        ===========================================================*/
        if ((buffer.data[2] + buffer.data[3]) > 0) {
            axis1.set_actuator(buffer.data[2] + buffer.data[3]);
        }
        if ((buffer.data[4] + buffer.data[5]) > 0) {
            axis2.set_actuator(buffer.data[4] + buffer.data[5]);
        }
        if ((buffer.data[6] + buffer.data[7]) > 0) {
            axis3.set_actuator(buffer.data[6] + buffer.data[7]);
        }
        break;

    case RXID_UPPER:
        /*=========================================================
               Set next angles for top three axis
        ===========================================================*/
        if ((buffer.data[2] + buffer.data[3]) > 0) {
            axis4.set_actuator(buffer.data[2] + buffer.data[3]);
        }
        if ((buffer.data[4] + buffer.data[5]) > 0) {
            axis5.set_actuator(buffer.data[4] + buffer.data[5]);
        }
        if ((buffer.data[6] + buffer.data[7]) > 0) {
            axis6.set_actuator(buffer.data[6] + buffer.data[7]);
        }
        break;

    case RX_MANUAL:
        /*=========================================================
               Manual Control
        ===========================================================*/
        if ((buffer.data[1] - 0x10) == 1)
        {
            axis1.set_actuator(axis1.get_current_angle() - (buffer.data[0] * (buffer.data[1] - 0x10)));
        }
        else
        {
            axis1.set_actuator(axis1.get_current_angle() + (buffer.data[0] * buffer.data[1]));
        }

        if ((buffer.data[2] - 0x10) == 1)
        {
            axis2.set_actuator(axis2.get_current_angle() - (buffer.data[0] * (buffer.data[2] - 0x10)));
        }
        else
        {
            axis2.set_actuator(axis2.get_current_angle() + (buffer.data[0] * buffer.data[2]));
        }

        if ((buffer.data[3] - 0x10) == 1)
        {
            axis3.set_actuator(axis3.get_current_angle() - (buffer.data[0] * (buffer.data[3] - 0x10)));
        }
        else
        {
            axis3.set_actuator(axis3.get_current_angle() + (buffer.data[0] * buffer.data[3]));
        }

        if ((buffer.data[4] - 0x10) == 1)
        {
            axis4.set_actuator(axis4.get_current_angle() - (buffer.data[0] * (buffer.data[4] - 0x10)));
        }
        else
        {
            axis4.set_actuator(axis4.get_current_angle() + (buffer.data[0] * buffer.data[4]));
        }

        if ((buffer.data[5] - 0x10) == 1)
        {
            axis5.set_actuator(axis5.get_current_angle() - (buffer.data[0] * (buffer.data[5] - 0x10)));
        }
        else
        {
            axis5.set_actuator(axis5.get_current_angle() + (buffer.data[0] * buffer.data[5]));
        }

        if ((buffer.data[6] - 0x10) == 1)
        {
            axis6.set_actuator(axis6.get_current_angle() - (buffer.data[0] * (buffer.data[6] - 0x10)));
        }
        else
        {
            axis6.set_actuator(axis6.get_current_angle() + (buffer.data[0] * buffer.data[6]));
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

// Execute movement commands
void run()
{
    if (eStopActivated)
    {
        return;
    }
    if (runProg == false)
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
// Update pos on a timer
void updateAxisPos()
{
    if (millis() - timer > REFRESH_RATE)
    {
        sendLowerPos();
        sendUpperPos();
        timer = millis();
    }
}

// Send Lower Axis Positions
void sendLowerPos()
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
}

// Send Upper Axis Positions
void sendUpperPos()
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
}


/*=========================================================
    Setup and Main loop
===========================================================*/
// Setup device
void setup()
{
    // PSU needs time to power up or Mega will hang during setup
#if defined ARM1
    delay(4000);
#endif
#if defined ARM2
    delay(4100);
#endif

    // Disable interrupts
    cli();

    Serial.begin(115200);
    if (CAN0.begin(MCP_STDEXT, CAN_500KBPS, MCP_8MHZ) == CAN_OK)
        Serial.println(F("MCP2515 Activated"));
    else
        Serial.println(F("MCP2515 Failed"));

    pinMode(INT_PIN, INPUT);                       // Setting pin 2 for /INT input

#if defined ARM1
    // Arm1
    CAN0.init_Mask(0, 0, 0x00FF0000);                // Init first mask...
    CAN0.init_Filt(0, 0, 0x00A00000);                // Init first filter...
    CAN0.init_Filt(1, 0, 0x00A10000);                // Init second filter...
    CAN0.init_Mask(1, 0, 0x00FF0000);                // Init second mask... 
    CAN0.init_Filt(2, 0, 0x00A20000);                // Init third filter...
    CAN0.init_Filt(3, 0, 0x00A30000);                // Init fouth filter...
    CAN0.init_Filt(4, 0, 0x00A40000);                // Init fifth filter...
    CAN0.init_Filt(5, 0, 0x00A50000);                // Init sixth filter...
#endif 
#if defined ARM2
    // Arm2
    CAN0.init_Mask(0, 0, 0x00BF0000);                // Init first mask...
    CAN0.init_Filt(0, 0, 0x00B00000);                // Init first filter...
    CAN0.init_Filt(1, 0, 0x00B10000);                // Init second filter...
    CAN0.init_Mask(1, 0, 0x00BF0000);                // Init second mask...
    CAN0.init_Filt(2, 0, 0x00B20000);                // Init third filter...
    CAN0.init_Filt(3, 0, 0x00B30000);                // Init fouth filter...
    CAN0.init_Filt(4, 0, 0x00B40000);                // Init fifth filter...
    CAN0.init_Filt(5, 0, 0x00B50000);                // Init sixth filter...
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
    //attachInterrupt(digitalPinToInterrupt(INT_PIN), MSGBuff, FALLING);

    // Enable interrupts
    sei();

    Serial.print("Free Memory: ");
    Serial.println(freeRam(), DEC);
    //Serial.end();
}

// Main loop
void loop()
{
    // Read incoming messages given highest priority
    MSGBuff();

    // Check if there are any CAN Bus messages in the buffer
    readMSG();

    // Read incoming messages given highest priority
    MSGBuff();

    // Send out the current actuator angles
    updateAxisPos();
    
    // Read incoming messages given highest priority
    MSGBuff();

    // Execute current commands
    run();
}


/*=========================================================
    Depreciated Code 
===========================================================*/
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
