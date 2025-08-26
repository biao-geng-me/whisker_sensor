/*
 * Title: MoveVelocity
 *
 * Objective:
 *    This example demonstrates control of a ClearPath motor in Step and
 *    Direction mode, making velocity moves.
 *
 * Description:
 *    This example enables a ClearPath then commands a series of repeating
 *    velocity moves to the motor.
 *
 * Requirements:
 * 1. A ClearPath motor must be connected to Connector M-0.
 * 2. The connected ClearPath motor must be configured through the MSP software
 *    for Step and Direction mode (In MSP select Mode>>Step and Direction).
 * 3. The ClearPath motor must be set to use the HLFB mode "ASG-Position
 *    w/Measured Torque" with a PWM carrier frequency of 482 Hz through the MSP
 *    software (select Advanced>>High Level Feedback [Mode]... then choose
 *    "ASG-Position w/Measured Torque" from the dropdown, make sure that 482 Hz
 *    is selected in the "PWM Carrier Frequency" dropdown, and hit the OK
 *    button).
 * 4. Set the Input Format in MSP for "Step + Direction".
 *
 * ** Note: Set the Input Resolution in MSP the same as your motor's Positioning
 *    Resolution spec if you'd like the pulse frequency sent by ClearCore to 
 *    command the same frequency in motor encoder counts/sec, a 1:1 ratio.
 *
 * Links:
 * ** ClearCore Documentation: https://teknic-inc.github.io/ClearCore-library/
 * ** ClearCore Manual: https://www.teknic.com/files/downloads/clearcore_user_manual.pdf
 * ** ClearPath Manual (DC Power): https://www.teknic.com/files/downloads/clearpath_user_manual.pdf
 * ** ClearPath Manual (AC Power): https://www.teknic.com/files/downloads/ac_clearpath-mc-sd_manual.pdf
 *
 * 
 * Copyright (c) 2020 Teknic Inc. This work is free to use, copy and distribute under the terms of
 * the standard MIT permissive software license which can be found at https://opensource.org/licenses/MIT
 */
 
#include "ClearCore.h"
 
// Specifies which motor to move.
// Options are: ConnectorM0, ConnectorM1, ConnectorM2, or ConnectorM3.
// Note: generic stepper motors (non-ClearPath) can only be connected to M0
#define aoa_motor ConnectorM0
#define long_axis_motor ConnectorM1
#define short_axis_motor ConnectorM2
 
// Select the baud rate to match the target serial device
#define baudRate 2000000
 
// Specify which serial to use: ConnectorUsb, ConnectorCOM0, or ConnectorCOM1.
// #define SerialPort ConnectorUsb
// Note: using Arduino's Serial class instead of the serial functions from clear core.

// This example has built-in functionality to automatically clear motor alerts, 
//  including motor shutdowns. Any uncleared alert will cancel and disallow motion.
// WARNING: enabling automatic alert handling will clear alerts immediately when 
//  encountered and return a motor to a state in which motion is allowed. Before 
//  enabling this functionality, be sure to understand this behavior and ensure 
//  your system will not enter an unsafe state. 
// To enable automatic alert handling, #define HANDLE_ALERTS (1)
// To disable automatic alert handling, #define HANDLE_ALERTS (0)
#define HANDLE_ALERTS (1)
 
// Define the acceleration limits to be used for each move, pulses per sec^2
int32_t accelerationLimit = 10000;
int32_t velocityLimit = 8000;
int32_t aoa_acc_limit = 10000;
int32_t aoa_vel_limit = 3200;

//--------------------------------------------------------------------------------------
// Declares user-defined helper functions.
// The definition/implementations of these functions are at the bottom of the sketch.
void init_clearpath(MotorDriver &motor, const char* name);
void init_generic_stepper();
bool MoveAtVelocity(MotorDriver &motor, int32_t velocity, const char* name);
bool MoveAbsolutePosition(MotorDriver &motor, int32_t pos, const char* name);
void MoveDistance(MotorDriver &motor, int32_t distance);
void PrintAlerts(MotorDriver &motor);
void HandleAlerts(MotorDriver &motor);
void recv_until_end_marker();
void update_cmd_from_serial();
void move_axis(MotorDriver &motor, const char *name, const char *cmd, int32_t &current_speed);
void move_xy(int32_t la_vel, int32_t sa_vel);
void print_current_position();
void handle_motor_alert(MotorDriver &motor, const char * action);
//--------------------------------------------------------------------------------------

int32_t current_direction = 0;
int32_t current_x_speed = 0;
int32_t current_y_speed = 0;
int32_t x_pos = 0;
int32_t y_pos = 0;

int32_t target_x_speed = 0;
int32_t target_y_speed = 0;

#define tAvg 2000; // time interval for reporting performance
#define STEADY_CYCLE_TIME 25 // time interval to check input, to ensure smoothness, the input firing rate must be equal or higher.
uint32_t loop_count = 0;
uint32_t work_time = 0;
uint32_t total_time = 0; // total loop time for averaging
uint32_t prev_time = 0, current_time;
uint32_t print_count = 0;

uint32_t nAvg = (int)2000/STEADY_CYCLE_TIME;

#define check_performance 1
#define RECV_BUFFER_LENGTH 128 // 128 is the maximum? for ClearCore.

// command parsing
char *command_fields[3];
char cmd_char_array[RECV_BUFFER_LENGTH];

void setup() {

    // Sets up serial communication and waits up to 5 seconds for a port to open.
    uint32_t timeout = 5000;
    uint32_t startTime = Milliseconds();
    Serial.begin(baudRate);
    while (!Serial && Milliseconds() - startTime < timeout) {
        continue;
    }

    // Sets the input clocking rate. This normal rate is ideal for ClearPath
    // step and direction applications.
    MotorMgr.MotorInputClocking(MotorManager::CLOCK_RATE_NORMAL);
 
    // Sets all motor connectors into step and direction mode.
    MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL,
                          Connector::CPM_MODE_STEP_AND_DIR);
    
    // Init the motors
    // init_long_axis_motor("Long axis");
    // init_short_axis_motor("Short axis");
    init_clearpath(short_axis_motor, "Short axis motor");
    init_clearpath(long_axis_motor, "Long axis motor");
    init_generic_stepper();

    print_current_position();
    // Test drive
    if (1==1) {

        move_xy(1000,1000);
        Delay_ms(1000);

        move_xy(-1000,-1000);
        Delay_ms(1000);

        // Command a 0 steps/sec velocity to stop motion for 2000ms
        move_xy(0,0);
        Delay_ms(200);
        print_current_position();
        // reset position reference
        // long_axis_motor.PositionRefSet(0);
        // short_axis_motor.PositionRefSet(0);
        
        // return to power-on position
        MoveAbsolutePosition(long_axis_motor, 0, "Long axis");
        MoveAbsolutePosition(short_axis_motor, 0, "Short axis");
    }
    MoveDistance(aoa_motor,800);
    Delay_ms(1000);

    prev_time = Milliseconds();
}

void loop() {
    int32_t loop_time = Milliseconds();
    loop_count++;
    if (Serial.available()) {
        update_cmd_from_serial();
    }
    else { // set all motor to move at 0 speed (automatic decelerate)
        command_fields[0] = "VEL0";
        command_fields[1] = "VEL0";
        command_fields[2] = "VEL0";
    }

    move_axis(long_axis_motor, "Long axis", command_fields[0], current_x_speed);
    move_axis(short_axis_motor, "Short axis", command_fields[1], current_y_speed);

    print_current_position();

    loop_time = Milliseconds() - loop_time;
    work_time+= loop_time;

    // Serial.print(loop_count);
    // Serial.print('\t');
    // Serial.print(STEADY_CYCLE_TIME - loop_time);
    // Serial.print('\t');
    // Serial.println(loop_time);

    if (STEADY_CYCLE_TIME - loop_time > 0) {
        Delay_ms(STEADY_CYCLE_TIME - loop_time);
    }

    if(check_performance && loop_count == nAvg) {
        print_count++;
        loop_count = 0;
        work_time = 0;
        current_time = Milliseconds();
        total_time = current_time - prev_time;
        prev_time = current_time;
        Serial.print(print_count);
        Serial.print('\t');
        Serial.print("Average loop time: ");
        Serial.print(total_time/nAvg);
        Serial.print(", FPS=");
        Serial.println(1000.0/total_time*nAvg);

        Serial.print('\t');
        Serial.print("Average work time: ");
        Serial.print(work_time/nAvg);
        Serial.print(", Highest possible FPS=");
        Serial.println(1000.0/work_time*nAvg);
        
        // get current position

    }
}

void init_generic_stepper() {
    aoa_motor.VelMax(aoa_vel_limit);
    aoa_motor.AccelMax(aoa_acc_limit);
    aoa_motor.EnableRequest(true);
}

void init_clearpath(MotorDriver &motor, const char* name) {

    // Set the motor's HLFB mode to bipolar PWM
    motor.HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);
    // Set the HFLB carrier frequency to 482 Hz
    motor.HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);
    // Sets the maximum velocity for each move
    motor.VelMax(velocityLimit);
    // Set the maximum acceleration for each move
    motor.AccelMax(accelerationLimit);
    // Enables the motors; homing will begin automatically if enabled
    motor.EnableRequest(true);
    Serial.print(name);
    Serial.println(" enabled.");
 
    // Waits for HLFB to assert (waits for homing to complete if applicable)
    Serial.println("Waiting for HLFB...");
    while (motor.HlfbState() != MotorDriver::HLFB_ASSERTED &&
            !motor.StatusReg().bit.AlertsPresent) {
        Serial.print("HLFB status: ");
        Serial.println(motor.HlfbState());
        Serial.print("Wating for: ");
        Serial.println(MotorDriver::HLFB_ASSERTED);
        Delay_ms(100);
        continue;
    }

    // Check if motor alert occurred during enabling
    // Clear alert if configured to do so 
    handle_motor_alert(motor, "Enabling");
    // if (motor.StatusReg().bit.AlertsPresent) {
    //     Serial.println("Motor alert detected.");       
    //     PrintAlerts(motor);
    //     if(HANDLE_ALERTS){
    //         HandleAlerts(motor);
    //     } else {
    //         Serial.println("Enable automatic alert handling by setting HANDLE_ALERTS to 1.");
    //     }
    //     Serial.println("Enabling may not have completed as expected. Proceed with caution.");      
    //     Serial.println();
    // } else {
    //     Serial.println("Motor Ready"); 
    // }
}

void move_xy(int32_t la_vel, int32_t sa_vel) {
    MoveAtVelocity(long_axis_motor, la_vel, "Long axis");
    MoveAtVelocity(short_axis_motor, sa_vel, "Short axis");
}

void move_axis(MotorDriver &motor,
                    const char* name,
                    const char* cmd,
                    int32_t & current_speed) {

    if (strncmp(cmd,"VEL",3)==0) { // Velocity move
        int32_t target_speed = atoi(cmd+3);
        if (current_speed != target_speed) {
            MoveAtVelocity(motor, target_speed, name);
            current_speed = target_speed;
        }
    }
    else if (strncmp(cmd,"ABS",3)==0) { // Move to absolute position
        int32_t pos = atoi(cmd+3);
        MoveAbsolutePosition(motor, pos, name);
    }
    else if (strncmp(cmd,"SET",3)==0) { // Set current position as absolute 0 (home position)
        motor.PositionRefSet(0);
        handle_motor_alert(motor,cmd);
    }
    else if (strncmp(cmd,"PRE",3)==0) { // Continue Previous action
        /* no change */
    }
    else if (strncmp(cmd,"CVL",3)==0) { // Change velocity limit
        uint32_t vel = atoi(cmd+3);
        motor.VelMax(vel); // todo: add bounds
        handle_motor_alert(motor,cmd);
    }
    else if (strncmp(cmd,"CAL",3)==0) { // Change acceleration limit
        uint32_t acl = atoi(cmd+3);
        motor.AccelMax(acl); // todo: add bounds
        handle_motor_alert(motor,cmd);
    }
    else { // unknown commands treated as move at 0 speed
        Serial.print(name);
        Serial.print(' unknown command: ');
        Serial.print(cmd);
        Serial.print(', Stopping.');
        int32_t target_speed = 0;
        MoveAtVelocity(motor, target_speed, name);
        current_speed = target_speed;
    }
}

/*------------------------------------------------------------------------------
 * MoveAtVelocity
 *
 *    Command the motor to move at the specified "velocity", in steps/second.
 *    Prints the move status to the USB serial port
 *
 * Parameters:
 *    int velocity  - The velocity, in step steps/sec, to command
 *
 * Returns: None
 */
bool MoveAtVelocity(MotorDriver &motor, int32_t velocity, const char* name) {
    // Check if a motor alert is currently preventing motion
    // Clear alert if configured to do so 
    if (motor.StatusReg().bit.AlertsPresent) {
        Serial.print(name);
        Serial.println(" alert detected.");       
        PrintAlerts(motor);
        if(HANDLE_ALERTS){
            HandleAlerts(motor);
        } else {
            Serial.println("Enable automatic alert handling by setting HANDLE_ALERTS to 1.");
        }
        Serial.print(name);
        Serial.println(" move canceled.");      
        Serial.println();
        return false;
    }
    Serial.print(name);
    Serial.print(" commanding velocity: ");
    Serial.println(velocity);
 
    // Command the velocity move
    motor.MoveVelocity(velocity);
    // Ramping is removed for non-blocking run
    // Waits for the step command to ramp up/down to the commanded velocity. 
    // This time will depend on your Acceleration Limit.
    // Serial.println("Ramping to speed...");
    // while (!motor.StatusReg().bit.AtTargetVelocity) {
    //     continue;
    // }

    // Check if motor alert occurred during move
    // Clear alert if configured to do so 
    if (motor.StatusReg().bit.AlertsPresent) {
        Serial.print(name);
        Serial.println(" motor alert detected.");       
        PrintAlerts(motor);
        if(HANDLE_ALERTS){
            HandleAlerts(motor);
        } else {
            Serial.println("Enable automatic fault handling by setting HANDLE_ALERTS to 1.");
        }
        Serial.println("Motion may not have completed as expected. Proceed with caution.");
        Serial.println();
        return false;
    } else {
        Serial.println("Move Done");
        return true;
    }
}
//------------------------------------------------------------------------------

/*------------------------------------------------------------------------------
 * MoveAbsolutePosition
 *
 *    Command step pulses to move the motor's current position to the absolute
 *    position specified by "position"
 *    Prints the move status to the USB serial port
 *    Returns when HLFB asserts (indicating the motor has reached the commanded
 *    position)
 *
 * Parameters:
 *    int position  - The absolute position, in step pulses, to move to
 *
 * Returns: True/False depending on whether the move was successfully triggered.
 */
bool MoveAbsolutePosition(MotorDriver &motor, int32_t position, const char* name) {
    // Check if a motor alert is currently preventing motion
    // Clear alert if configured to do so 
    if (motor.StatusReg().bit.AlertsPresent) {
        Serial.println("Motor alert detected.");       
        PrintAlerts(motor);
        if(HANDLE_ALERTS){
            HandleAlerts(motor);
        } else {
            Serial.println("Enable automatic alert handling by setting HANDLE_ALERTS to 1.");
        }
        Serial.println("Move canceled.");      
        Serial.println();
        return false;
    }
    Serial.print(name);
    Serial.print(" moving to absolute position: ");
    Serial.println(position);
 
    // Command the move of absolute distance
    motor.Move(position, MotorDriver::MOVE_TARGET_ABSOLUTE);
 
    // Waits for HLFB to assert (signaling the move has successfully completed)
    Serial.println("Moving.. HLFB skipped");

    // while ( (!motor.StepsComplete() || motor.HlfbState() != MotorDriver::HLFB_ASSERTED) &&
    //         !motor.StatusReg().bit.AlertsPresent) {
    //     Serial.print(motor.StepsComplete());
    //     Serial.print('\t');
    //     Serial.println(motor.HlfbState());
    //     Delay_ms(100);
    //     continue;
    // }

    // Check if motor alert occurred during move
    // Clear alert if configured to do so 
    if (motor.StatusReg().bit.AlertsPresent) {
        Serial.println("Motor alert detected.");       
        PrintAlerts(motor);
        if(HANDLE_ALERTS){
            HandleAlerts(motor);
        } else {
            Serial.println("Enable automatic fault handling by setting HANDLE_ALERTS to 1.");
        }
        Serial.println("Motion may not have completed as expected. Proceed with caution.");
        Serial.println();
        return false;
    } else {
        Serial.println("Command done");
        return true;
    }
}
//------------------------------------------------------------------------------

// for generic stepper motor
void MoveDistance(MotorDriver &motor, int32_t distance) {
    Serial.print("Moving distance: ");
    Serial.println(distance);
 
    // Command the move of incremental distance
    motor.Move(distance);
 
    // Waits for all step pulses to output
    // Serial.println("Moving... Waiting for the step output to finish...");
    // while (!motor.StepsComplete()) {
    //     continue;
    // }

    Serial.println("Done");
}
 
/*------------------------------------------------------------------------------
 * PrintAlerts
 *
 *    Prints active alerts.
 *
 * Parameters:
 *    requires "motor" to be defined as a ClearCore motor connector
 *
 * Returns: 
 *    none
*/
 void PrintAlerts(MotorDriver &motor){
    // report status of alerts
    Serial.println("Alerts present: ");
    if(motor.AlertReg().bit.MotionCanceledInAlert){
        Serial.println("    MotionCanceledInAlert "); }
    if(motor.AlertReg().bit.MotionCanceledPositiveLimit){
        Serial.println("    MotionCanceledPositiveLimit "); }
    if(motor.AlertReg().bit.MotionCanceledNegativeLimit){
        Serial.println("    MotionCanceledNegativeLimit "); }
    if(motor.AlertReg().bit.MotionCanceledSensorEStop){
        Serial.println("    MotionCanceledSensorEStop "); }
    if(motor.AlertReg().bit.MotionCanceledMotorDisabled){
        Serial.println("    MotionCanceledMotorDisabled "); }
    if(motor.AlertReg().bit.MotorFaulted){
        Serial.println("    MotorFaulted ");
    }
 }
//------------------------------------------------------------------------------
 
 
/*------------------------------------------------------------------------------
 * HandleAlerts
 *
 *    Clears alerts, including motor faults. 
 *    Faults are cleared by cycling enable to the motor.
 *    Alerts are cleared by clearing the ClearCore alert register directly.
 *
 * Parameters:
 *    requires "motor" to be defined as a ClearCore motor connector
 *
 * Returns: 
 *    none
*/
 void HandleAlerts(MotorDriver &motor){
    if(motor.AlertReg().bit.MotorFaulted){
        // if a motor fault is present, clear it by cycling enable
        Serial.println("Faults present. Cycling enable signal to motor to clear faults.");
        motor.EnableRequest(false);
        Delay_ms(10);
        motor.EnableRequest(true);
    }
    // clear alerts
    Serial.println("Clearing alerts.");
    motor.ClearAlerts();
 }
//------------------------------------------------------------------------------
 

void update_cmd_from_serial() {
    while(Serial.available()){
        recv_until_end_marker();
    }

    Serial.print("\nReceived ");
    Serial.println(cmd_char_array);

    // Parse command
    byte index = 0;
    char *ptr = NULL;
    ptr = strtok(cmd_char_array,",");
    while (ptr != NULL) {
        command_fields[index] = ptr;
        // Serial.println(command_fields[index]);
        index++;
        // Subsequent calls to strtok takes NULL and use internal static status to continue from previous position
        ptr = strtok(NULL, ","); 
    }
}

void recv_until_end_marker() {
    // block reading
    // make sure that communication is fast

    byte ndx = 0;
    char endMarker = '>';
    char rc;
    
    rc = Serial.read();
    while (rc != endMarker) {
        cmd_char_array[ndx] = rc;
        // Serial.println(rc);
        ndx++;
        if (ndx >= RECV_BUFFER_LENGTH) {
            ndx = RECV_BUFFER_LENGTH - 1;
        }
        while (!Serial.available()) { 
            // Block until serial is available
            // This is necessary because Serial data is slow compared to the read function
            // Serial.println("waiting for end marker '>'");
            Delay_ms(1);
            continue;
        }
        rc = Serial.read();
    }
    cmd_char_array[ndx] = '\0'; // terminate the string
}

void print_current_position() {
    x_pos = long_axis_motor.PositionRefCommanded();
    y_pos = short_axis_motor.PositionRefCommanded();
    Serial.print("\tcurrent position: [");
    Serial.print(x_pos);
    Serial.print(",");
    Serial.print(y_pos);
    Serial.println("].");
}

void handle_motor_alert(MotorDriver &motor, const char * action) {

     // Check if motor alert occurred during action
    // Clear alert if configured to do so 
    if (motor.StatusReg().bit.AlertsPresent) {
        Serial.println("Motor alert detected.");       
        PrintAlerts(motor);
        if(HANDLE_ALERTS){
            HandleAlerts(motor);
        } else {
            Serial.println("Enable automatic alert handling by setting HANDLE_ALERTS to 1.");
        }
        Serial.print(action);
        Serial.println(" may not have completed as expected. Proceed with caution.");      
        Serial.println();
    } else {
        Serial.print(action);
        Serial.println(" done.");
    }
}
 