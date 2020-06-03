// Declare actuator class used for each stepper motor
class actuator {
public:
    uint8_t g_code;           // Set g_code mode for next cycle  
    int current_angle;        // Angle of actuators position in degrees
    int long steps_to_move;   // Set quantity of steps to move in next cycle
    bool actuator_direction;  // Current direction of actuator
    bool enable_actuator;     // Enable actuator to accept pulse
    double actuator_speed;    // Set individual actuator speed
};


// Declare each actuator as it's own object
// Base x, y, z, 1
actuator actuator_x1;
actuator actuator_y1;
actuator actuator_z1;
// Wrist x, y, z, 2
actuator actuator_x2;
actuator actuator_y2;
actuator actuator_z2;


// Actuator x1
const uint8_t SPD_x1 = 3;  // Pulse
const uint8_t DIR_x1 = 2;  // Direction
const uint8_t ENA_x1 = 4;  // Enable

// Actuator y1
const uint8_t SPD_y1 = 22;  // Pulse
const uint8_t DIR_y1 = 24;  // Direction
const uint8_t ENA_y1 = 26;  // Enable

// Actuator z1
const uint8_t SPD_z1 = 28;  // Pulse
const uint8_t DIR_z1 = 30;  // Direction
const uint8_t ENA_z1 = 32;  // Enable


// Actuator x2
const uint8_t DIR_x2 = 40;  // Direction
const uint8_t ENA_x2 = 42; // Motor
const uint8_t SPD_x2 = 44; // Speed
// Actuator y2
const uint8_t DIR_y2 = 46; // Direction
const uint8_t ENA_y2 = 48; // Motor
const uint8_t SPD_y2 = 50; // Speed
// Actuator z2
const uint8_t DIR_z2 = 52; // Direction
const uint8_t ENA_z2 = 53; // Motor
const uint8_t SPD_z2 = 51; // Speed


const uint8_t PULSE_SPEED = 40;           // Lower number produces higher RPM
const uint8_t SPEED_ADJUSTED_G0 = PULSE_SPEED - 10;         // SPEED_ADJUSTED compensates time used for CPU to run logic which is approximately 18 ms
const uint8_t SPEED_ADJUSTED_G1 = PULSE_SPEED - 30;         // SPEED_ADJUSTED compensates time used for CPU to run logic which is approximately 18 ms
const uint8_t SPEED_ADJUSTED_G2 = PULSE_SPEED - 10;         // SPEED_ADJUSTED compensates time used for CPU to run logic which is approximately 18 ms
const uint8_t DRIVER_STEPS = 32;          // Current driver hardware pin settings from 1 to 32. (fractions of 1/1 - 1/32)
const uint8_t STEPS_PER_ROTATION = 200;   // NEMA steps used for 1 rotation
const uint8_t ACTUATOR_GEAR_RATIO = 38;   // Gear reduction for actuators is 1:4
const uint8_t NUMBER_OF_ACTUATORS = 6;    // Number of actuators in robot
const double  STEPS_PER_DEGREE = 675.555; // (DRIVER_STEPS * STEPS_PER_ROTATION * ACTUATOR_GEAR_RATIO) / DEGREES_IN_CIRCLE = 675.555~


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

    // Temporary starting position to be replaces with calibration function
    actuator_x1.current_angle = 0;
    actuator_y1.current_angle = 0;
    actuator_z1.current_angle = 180; // Start at 180
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
    set_actuator_x1(a);
    set_actuator_y1(b);
    set_actuator_z1(c);


    //delay(1000);

    a = 40;
    b = 175;
    c = 90;

    set_actuator_x1(a);
    set_actuator_y1(b);
    set_actuator_z1(c);

    G1();
    delay(1);

    a = 70;
    b = 215;
    c = 90;

    set_actuator_x1(a);
    set_actuator_y1(b);
    set_actuator_z1(c);

    G1();
    delay(5000);

    a = 70;
    b = 210;
    c = 90;

    set_actuator_x1(a);
    set_actuator_y1(b);
    set_actuator_z1(c);

    G1();
    delay(1);

    a = 0;
    b = 0;
    c = 180;

    set_actuator_x1(a);
    set_actuator_y1(b);
    set_actuator_z1(c);

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
*/
void set_actuator_x1(int new_pos_x1) {
    // Set direction
    if (new_pos_x1 < actuator_x1.current_angle) {
        actuator_x1.actuator_direction = false;
    }
    else {
        actuator_x1.actuator_direction = true;
    }

    // Calculate distance to move
    int dis_to_move_x1 = abs(new_pos_x1 - actuator_x1.current_angle);

    // Convert degrees to steps
    actuator_x1.steps_to_move = STEPS_PER_DEGREE * dis_to_move_x1;

    // Enable is steps are greater than 0
    if (dis_to_move_x1 == 0) {
        actuator_x1.enable_actuator = false;
    }
    else {
        actuator_x1.enable_actuator = true;
    }

    // Set actuator to new current_angle
    actuator_x1.current_angle = new_pos_x1;
}


/*
****************************************************
*   Function: set_actuator_y1()                     *
*    - Sets paremeters for actuator_y1 object       *
*                                                  *
*   Parameters: int new_pos_y1                     *
*    - Value of new current_angle in degrees       *
*                                                  *
*   Returns: void                                  *
****************************************************
*/
void set_actuator_y1(int new_pos_y1) {
    // Set direction
    if (new_pos_y1 < actuator_y1.current_angle) {
        actuator_y1.actuator_direction = false;
    }
    else {
        actuator_y1.actuator_direction = true;
    }

    // Calculate distance to move
    int dis_to_move_y1 = abs(new_pos_y1 - actuator_y1.current_angle);

    // Convert degrees to steps
    actuator_y1.steps_to_move = STEPS_PER_DEGREE * dis_to_move_y1;

    // Enable is steps are greater than 0
    if (dis_to_move_y1 == 0) {
        actuator_y1.enable_actuator = false;
    }
    else {
        actuator_y1.enable_actuator = true;
    }

    // Set actuator to new current_angle
    actuator_y1.current_angle = new_pos_y1;
}


/*
****************************************************
*   Function: set_actuator_z1()                    *
*    - Sets paremeters for actuator_z1 object       *
*                                                  *
*   Parameters: int new_pos_z1                     *
*    - Value of new current_angle in degrees       *
*                                                  *
*   Returns: void                                  *
****************************************************
*/
void set_actuator_z1(int new_pos_z1) {
    if (new_pos_z1 < actuator_z1.current_angle) {
        actuator_z1.actuator_direction = false;
    }
    else {
        actuator_z1.actuator_direction = true;
    }
    // Calculate distance to move
    int dis_to_move_z1 = abs(new_pos_z1 - actuator_z1.current_angle);

    // Convert degrees to steps
    actuator_z1.steps_to_move = STEPS_PER_DEGREE * dis_to_move_z1;

    // Enable is steps are greater than 0
    if (actuator_z1.steps_to_move == 0) {
        actuator_z1.enable_actuator = false;
    }
    else {
        actuator_z1.enable_actuator = true;
    }

    // Set actuator to new current_angle
    actuator_z1.current_angle = new_pos_z1;
}


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
int* convert_char(char input) {
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
    digitalWrite(DIR_x1, actuator_x1.actuator_direction);
    digitalWrite(DIR_y1, actuator_y1.actuator_direction);
    digitalWrite(DIR_z1, actuator_z1.actuator_direction);
    digitalWrite(ENA_x1, actuator_x2.enable_actuator);
    digitalWrite(ENA_y1, actuator_y2.enable_actuator);
    digitalWrite(ENA_z1, actuator_z2.enable_actuator);

    /*
            int step_array[NUMBER_OF_STEPS] = {actuator_x1.steps_to_move, actuator_y1.steps_to_move, actuator_z1.steps_to_move, actuator_x2.steps_to_move, actuator_y2.steps_to_move, actuator_z2.steps_to_move};
            int greatest_value = step_array[0];
            for (int i = 1; i < NUMBER_OF_ACTUATORS; i++)
            {
                if (greatest_value < step_array[i])
                greatest_value = step_array[i];
            }
    */

    while ((index < actuator_x1.steps_to_move) || (index < actuator_y1.steps_to_move) || (index < actuator_z1.steps_to_move)
        || (index < actuator_x2.steps_to_move) || (index < actuator_y2.steps_to_move) || (index < actuator_z2.steps_to_move))
    {
        if ((actuator_x1.steps_to_move < index)) {
            //actuator_x1.enable_actuator = true;
            digitalWrite(ENA_x1, true);
        }
        if (actuator_y1.steps_to_move < index) {
            // actuator_y1.enable_actuator = false;
            digitalWrite(ENA_y1, true);
        }
        if (actuator_z1.steps_to_move < index) {
            //actuator_z1.enable_actuator = false;
            digitalWrite(ENA_z1, true);
        }
        if (actuator_x2.steps_to_move < index) {
            actuator_x2.enable_actuator = false;
        }
        if (actuator_y2.steps_to_move < index) {
            actuator_y2.enable_actuator = false;
        }
        if (actuator_z2.steps_to_move < index) {
            actuator_z2.enable_actuator = false;
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
void G2(void) {
    int step_array[NUMBER_OF_ACTUATORS] = { actuator_x1.steps_to_move, actuator_y1.steps_to_move, actuator_z1.steps_to_move, actuator_x2.steps_to_move, actuator_y2.steps_to_move, actuator_z2.steps_to_move };
    int long index = 0;
    int greatest_value = step_array[0];
    for (int i = 1; i < NUMBER_OF_ACTUATORS; i++)
    {
        if (greatest_value < step_array[i])
            greatest_value = step_array[i];
    }
    double k_constant = greatest_value * PULSE_SPEED;
    actuator_x1.actuator_speed = k_constant / (step_array[0] * STEPS_PER_DEGREE);
    actuator_y1.actuator_speed = k_constant / (step_array[1] * STEPS_PER_DEGREE);
    actuator_z1.actuator_speed = k_constant / (step_array[2] * STEPS_PER_DEGREE);
    actuator_x2.actuator_speed = k_constant / (step_array[3] * STEPS_PER_DEGREE);
    actuator_y2.actuator_speed = k_constant / (step_array[4] * STEPS_PER_DEGREE);
    actuator_z2.actuator_speed = k_constant / (step_array[5] * STEPS_PER_DEGREE);


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