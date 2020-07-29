//#include <SdFat.h>
//#include <SPI.h>
//#include <SD.h>
#include <stdint.h>
#include "PinAssignments.cpp"
#include "Actuator.cpp"
// Global settings
const int PULSE_SPEED = 36;           // Lower number produces higher RPM
const int SPEED_ADJUSTED_G0 = PULSE_SPEED - 10;         // SPEED_ADJUSTED compensates time used for CPU to run logic which is approximately 18 ms
const int SPEED_ADJUSTED_G1 = PULSE_SPEED - 30;         // SPEED_ADJUSTED compensates time used for CPU to run logic which is approximately 18 ms
const int SPEED_ADJUSTED_G2 = PULSE_SPEED - 10;         // SPEED_ADJUSTED compensates time used for CPU to run logic which is approximately 18 ms
const int DRIVER_STEPS = 32;          // Current driver hardware pin settings from 1 to 32. (fractions of 1/1 - 1/32)
const int STEPS_PER_ROTATION = 200;   // NEMA steps used for 1 rotation
const int ACTUATOR_GEAR_RATIO = 38;   // Gear reduction for actuators is 1:4
const int NUMBER_OF_ACTUATORS = 6;    // Number of actuators in robot



//#include "Move.h"

// Actuator objects
// Base x, y, z, 1
Actuator actuator_x1;
Actuator actuator_y1;
Actuator actuator_z1;
// Wrist x, y, z, 2
Actuator actuator_x2;
Actuator actuator_y2;
Actuator actuator_z2;

void setup() {
    pinMode(SPD_x1, OUTPUT);
    pinMode(SPD_y1, OUTPUT);
    pinMode(SPD_z1, OUTPUT);
    pinMode(DIR_x1, OUTPUT);
    pinMode(ENA_x1, OUTPUT);
    pinMode(DIR_y1, OUTPUT);
    pinMode(ENA_y1, OUTPUT);
    pinMode(DIR_z1, OUTPUT);
    pinMode(ENA_z1, OUTPUT);

    // Temporary starting position to be replaces with sd card
    actuator_x1.set_current_angle(0);
    actuator_y1.set_current_angle(0);
    actuator_z1.set_current_angle(0); // Start at 180
}


void loop() {

    // Wait for cmd
    // Get string
    // Send string to function
    // string turned into gcode plus 6 positions
    // set_actuator x6
    // Select gcode function to use 
    //  - gcode could be a function or condition

    String test = "G1 A45 B60 C80";
    // ^ need to write this code to produce next lines
    //char g_code = "G1";
    int a = 0;
    int b = 0; //240 max - Starts at 60 degrees
    int c = 180;
    // set_g_code();
    actuator_x1.set_actuator(a);
    actuator_y1.set_actuator(b);
    actuator_z1.set_actuator(c);


    //delay(1000);

    a = 40;
    b = 175;
    c = 90;

    actuator_x1.set_actuator(a);
    actuator_y1.set_actuator(b);
    actuator_z1.set_actuator(c);

    G1();
    delay(1);

    a = 70;
    b = 215;
    c = 90;

    actuator_x1.set_actuator(a);
    actuator_y1.set_actuator(b);
    actuator_z1.set_actuator(c);

    G1();
    delay(5000);

    a = 70;
    b = 210;
    c = 90;

    actuator_x1.set_actuator(a);
    actuator_y1.set_actuator(b);
    actuator_z1.set_actuator(c);

    G1();
    delay(1);

    a = 0;
    b = 0;
    c = 180;

    actuator_x1.set_actuator(a);
    actuator_y1.set_actuator(b);
    actuator_z1.set_actuator(c);

    G1();
    delay(1000);

    endf();
}

void endf() {
    while (1) {
    }
}


/*
****************************************************
*   Function: set_actuator_x1ctuator_x1()            *
*    - Sets paremeters for actuator_x1 object       *
*                                                  *
*   Parameters: int new_pos_x1                      *
*    - Value of new current_angle in degrees            *
*                                                  *
*   Returns: void                                  *
****************************************************


/*
****************************************************
*   Function: convert_char()                       *
*    - Convert G-Code type command line into       *
*    - commands for actuators                      *
*                                                  *
*   Parameters: char input                         *
*    - Value of new current_angle in degrees       *
*                                                  *
*   Returns: Pointer to int array                  *
****************************************************
*/
// Convert char input to array of movement commands for actuator
int convert_char(char input) {
    return;
}


/*
****************************************************
*   Function: calibrate_actuators()                *
*    - Calibrates actuators to a default position  *
*    - using limiting switch hardware              *
*                                                  *
*   Parameters: None                               *
*                                                  *
*   Returns: void                                  *
****************************************************
*/
void calibrate_actuators() {
    // Create me!
}


/*
****************************************************
*   Function: calibrate_actuators()                *
*    - Calibrates actuators to a default position  *
*    - using limiting switch hardware              *
*                                                  *
*   Parameters: None                               *
*                                                  *
*   Returns: void                                  *
****************************************************
*/
void read_in_gCode() {
    
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
        digitalWrite(ENA_x1, actuator_x2.get_enable_actuator());
        digitalWrite(ENA_y1, actuator_y2.get_enable_actuator());
        digitalWrite(ENA_z1, actuator_z2.get_enable_actuator());

        /*
                int step_array[NUMBER_OF_STEPS] = {actuator_x1.steps_to_move, actuator_y1.steps_to_move, actuator_z1.steps_to_move, actuator_x2.steps_to_move, actuator_y2.steps_to_move, actuator_z2.steps_to_move};
                int greatest_value = step_array[0];
                for (int i = 1; i < NUMBER_OF_ACTUATORS; i++)
                {
                    if (greatest_value < step_array[i])
                    greatest_value = step_array[i];
                }
        */
        
            // Idea, find actuator with greatest number of steps then change while to for loop
            while ((index < actuator_x1.get_steps_to_move()) || (index < actuator_y1.get_steps_to_move()) || (index < actuator_z1.get_steps_to_move())
                || (index < actuator_x2.get_steps_to_move()) || (index < actuator_y2.get_steps_to_move()) || (index < actuator_z2.get_steps_to_move()))
            {
                if ((actuator_x1.get_steps_to_move() < index)) {
                    //actuator_x1.enable_actuator = true;
                    digitalWrite(ENA_x1, true);
                }
                if (actuator_y1.get_steps_to_move() < index) {
                    // actuator_y1.enable_actuator = false;
                    digitalWrite(ENA_y1, true);
                }
                if (actuator_z1.get_steps_to_move() < index) {
                    //actuator_z1.enable_actuator = false;
                    digitalWrite(ENA_z1, true);
                }
                if (actuator_x2.get_steps_to_move() < index) {
                    actuator_x2.set_enable_actuator(false);
                }
                if (actuator_y2.get_steps_to_move() < index) {
                    actuator_y2.set_enable_actuator(false);
                }
                if (actuator_z2.get_steps_to_move() < index) {
                    actuator_z2.set_enable_actuator(false);
                }


                digitalWrite(SPD_x1, true);
                digitalWrite(SPD_y1, true);
                digitalWrite(SPD_z1, true);
                delayMicroseconds(PULSE_SPEED);
                digitalWrite(SPD_x1, false);
                digitalWrite(SPD_y1, false);
                digitalWrite(SPD_z1, false);
                delayMicroseconds(SPEED_ADJUSTED_G1);
                index++;
            }
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
    int step_array[NUMBER_OF_ACTUATORS] = { actuator_x1.get_steps_to_move(), actuator_y1.get_steps_to_move(), actuator_z1.get_steps_to_move(), actuator_x2.get_steps_to_move(), actuator_y2.get_steps_to_move(), actuator_z2.get_steps_to_move() };
    int long index = 0;
    int greatest_value = step_array[0];
    for (int i = 1; i < NUMBER_OF_ACTUATORS; i++)
    {
        if (greatest_value < step_array[i])
            greatest_value = step_array[i];
    }
    double k_constant = greatest_value * PULSE_SPEED;
    //actuator_x1.set_actuator_speed((k_constant / (step_array[0] * STEPS_PER_DEGREE)));
    //actuator_y1.set_actuator_speed((k_constant / (step_array[1] * STEPS_PER_DEGREE)));
    //actuator_z1.set_actuator_speed((k_constant / (step_array[2] * STEPS_PER_DEGREE)));
    //actuator_x2.set_actuator_speed((k_constant / (step_array[3] * STEPS_PER_DEGREE)));
    //actuator_y2.set_actuator_speed((k_constant / (step_array[4] * STEPS_PER_DEGREE)));
    //actuator_z2.set_actuator_speed((k_constant / (step_array[5] * STEPS_PER_DEGREE)));


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