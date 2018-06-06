/**
 * @file position-adjust.cpp
 * @author Munzir Zafar
 * @date Jun 15, 2013
 * @brief This program allows the user to correct positions of the waist modules. The user is 
 * first allowed to bring the motor to a position which the user can eye-ball and tell the 
 * expected position, such as, marks (tape-alignment) on the motor and stator. In case the 
 * read position values do not match the ones expected, the user is allowed to set the new 
 * positions in the machine for correcting the position values.
 * In addition to waist, handles torso and arm modules.
 */

#include "somatic.h"
#include "somatic/daemon.h"
#include "somatic/motor.h"
#include <somatic.pb-c.h>
#include <ach.h>

#include <argp.h>
#include <unistd.h>
#include <syslog.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <string>
#include <algorithm>
#include <cmath>
#include <iostream>

using namespace std;

// The somatic context and options
somatic_d_t somaticContext;
somatic_d_opts_t somaticOptions;

#define POSITION SOMATIC__MOTOR_PARAM__MOTOR_POSITION
#define VELOCITY SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY
#define CURRENT SOMATIC__MOTOR_PARAM__MOTOR_CURRENT

/*************************************************************************************************/
// Variables need for ach communication

ach_channel_t stateChan;				///< The state channel, info, for the motors
ach_channel_t cmdChan;					///< The command channels (we send)
ach_channel_t joystickChan;			///< The joystick channel for use to set zeroes

string stateChanName;						///< The name for the state channel
string cmdChanName;							///< The name for the command channel

somatic_motor_t motors;					///< The motors we will set velocity/current values

/*************************************************************************************************/
// Variables to set options

char controlMode;								///< The mode at which the program will run: joystick or current
bool setValues = false;					///< Once the user chooses to set values, we set them.
bool makePositionsZero = 0;			///< If set with the '0' input, sets the current joint values as 0s
bool goToTapeMarks = false;			///< With the 't' flag, the motors are sent to the taped marks

/// The indicators for motor groups - LG/RG for left/right grippers
enum MotorGroupType { WAIST, LEFT_ARM, RIGHT_ARM, LG_SCHUNK, RG_SCHUNK, TORSO, NONE }; 	
MotorGroupType group = NONE;						///< The group that this program is going to zero
int numModules; 												///< The number of modules in each motor group

Somatic__WaistCmd *waistDaemonCmd = somatic_waist_cmd_alloc();		///< The waist command

/* ********************************************************************************************* */
// Argument processing

/// Options that will be presented to the user
static struct argp_option options[] = {
		{"motorGroup",'m', "motorGroup", 0, "group of motors to zero: waist, left, right, torso", 0},
		{"tape", 't', 0, 0, "command the motor group to the tape-marked positions", 0},
		{0, 0, 0, 0, 0, 0}
};

/// The one-line explanation of the executable
static char doc[]= "allows user to correct positions of the pcio modules";

/// The parser function
static int parse_opt( int key, char *arg, struct argp_state *state) {
	(void) state; 

	// Set the tape option on if the command is given
	if(key == 't') goToTapeMarks = true;
		
	// Make sure the input flag for motor group is set
	if(key != 'm') return 0;

	// Determine which motor group we want and set the channel names accordingly
	if(strcmp(strdup(arg), "waist") == 0) {
		stateChanName = "waist-state";
		cmdChanName = "waistd-cmd";
		group = WAIST;
		numModules = 2;
	}
	else if(strcmp(strdup(arg), "left") == 0) {
		stateChanName = "llwa-state";
		cmdChanName = "llwa-cmd";
		group = LEFT_ARM;
		numModules = 7;
	}
	else if(strcmp(strdup(arg), "right") == 0) {
		stateChanName = "rlwa-state";
		cmdChanName = "rlwa-cmd";
		group = RIGHT_ARM;
		numModules = 7;
	}
	else if(strcmp(strdup(arg), "lgSchunk") == 0) {
		stateChanName = "lgripper-state";
		cmdChanName = "lgripper-cmd";
		group = LG_SCHUNK;
		numModules = 1;
	}
	else if(strcmp(strdup(arg), "rgSchunk") == 0) {
		stateChanName = "rgripper-state";
		cmdChanName = "rgripper-cmd";
		group = RG_SCHUNK;
		numModules = 1;
	}
	else if(strcmp(strdup(arg), "torso") == 0) {
		stateChanName = "torso-state";
		cmdChanName = "torso-cmd";
		group = TORSO;
		numModules = 1;
	}
	else {
		printf("Unidentifiable motor group!\n");
		exit(0);
	}
	return 0;
}

/// The argp structure to parse stuff
static struct argp argp = {options, parse_opt, NULL, doc, NULL, NULL, NULL };

/* ********************************************************************************************* */
void *kbhit(void *) {
	// If the input is a or 0, accept it, and change the state
	char input;
	while(true){ 
		input=cin.get(); 
		if(input=='a' || input=='0') 
			break;
	}
	setValues = true;
	makePositionsZero=(input=='0');
}

/* ********************************************************************************************* */
/// Handles the joystick commands for the waist module
void joystickWaist (Somatic__Joystick* js_msg) {

	// Set the mode we want to send to the waist daemon
	double ff = js_msg->axes->data[5];
	Somatic__WaistMode waistMode;
	if(ff < -0.9) waistMode = SOMATIC__WAIST_MODE__MOVE_FWD;
	else if(ff > 0.9) waistMode = SOMATIC__WAIST_MODE__MOVE_REV;
	else waistMode = SOMATIC__WAIST_MODE__STOP;

	// Send message to the krang-waist daemon
	somatic_waist_cmd_set(waistDaemonCmd, waistMode);
	int r = SOMATIC_PACK_SEND(&cmdChan, somatic__waist_cmd, waistDaemonCmd);
	if(ACH_OK != r) fprintf(stderr, "Couldn't send message: %s\n", 
		ach_result_to_string(static_cast<ach_status_t>(r)));
}

/* ********************************************************************************************* */
/// Handles the joystick commands for the left/right arms
void joystickArms (Somatic__Joystick* js_msg) {

	// Get the values
	char b [10];
	double x [6];
	for(size_t i = 0; i < 10; i++) 
		b[i] = js_msg->buttons->data[i] ? 1 : 0;
	memcpy(x, js_msg->axes->data, sizeof(x));
	
	// Scale down the x values
	double scaleDownFactor = 0.5;
	for(size_t i = 0; i < 6; i++) x[i] *= scaleDownFactor;

	// Set the velocities
	size_t arm_idx = (group == LEFT_ARM) ? 0 : 1;
	double dq [] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	size_t lowerButton = 4 + arm_idx, higherButton = 6 + arm_idx;
	bool inputSet = true;
	if(b[lowerButton] && !b[higherButton]) memcpy(&dq[4], x, 3*sizeof(double));
	else if(!b[lowerButton] && b[higherButton]) memcpy(dq, x, 4*sizeof(double));
	else inputSet = false;
	
	// Set the input for this arm
	if(inputSet)
		somatic_motor_cmd(&somaticContext, &motors, VELOCITY, dq, 7, NULL);
}

/* ********************************************************************************************* */
/// Handles the joystick commands for the left/right Schunk grippers
void joystickGrippers (Somatic__Joystick* js_msg) {

	// Button 4 with top/down at the right circular thingy indicates a motion for the left gripper
	double dq [] = {0.0};
	dq[0] = js_msg->axes->data[3] / 50.0;
	if(js_msg->buttons->data[4]) 
		somatic_motor_cmd(&somaticContext, &motors, VELOCITY, dq, 1, NULL);

	// Button 5 with the same circular thingy for the right gripper
	else if(js_msg->buttons->data[5]) 
		somatic_motor_cmd(&somaticContext, &motors, VELOCITY, dq, 1, NULL);
}

/* ********************************************************************************************* */
/// Handles the joystick commands for the torso module
void joystickTorso (Somatic__Joystick* js_msg) {

	// Left and right buttons on the 8-pad control the torso module
	double dq [] = {js_msg->axes->data[4] / 7.0};
	somatic_motor_cmd(&somaticContext, &motors, VELOCITY, dq, 1, NULL);
}

/* ********************************************************************************************* */
/// Handles the current case for the waist module
void currentWaist () {
	somatic_waist_cmd_set(waistDaemonCmd, SOMATIC__WAIST_MODE__CURRENT_MODE);
	int r = SOMATIC_PACK_SEND(&cmdChan, somatic__waist_cmd, waistDaemonCmd);
	if(ACH_OK != r) fprintf(stderr, "Couldn't send message: %s\n", 
		ach_result_to_string(static_cast<ach_status_t>(r)));
}

/* ********************************************************************************************* */
/// Handles the current case for the Schunk modules: arm, torso and grippers
void currentSchunk () {
	double dq [numModules];
	for(size_t i = 0; i < numModules; i++) dq[i] = 0.0;
	somatic_motor_cmd(&somaticContext, &motors, CURRENT, dq, numModules, NULL);
}

/* ********************************************************************************************* */
/// Prints the position data from the waist modules
void printWaist () {

	// Get the data
	int r;
	double position[2];
	struct timespec abstime = aa_tm_future( aa_tm_sec2timespec( 1.0 / 30.0 ));
	Somatic__MotorState *waistState = SOMATIC_WAIT_LAST_UNPACK( r, somatic__motor_state, 
		&protobuf_c_system_allocator, 1024, &stateChan, &abstime );
	aa_hard_assert(r == ACH_OK || r == ACH_TIMEOUT || r == ACH_MISSED_FRAME, 
		"Ach wait failure %s on pcio data receive (%s, line %d)\n", 
		ach_result_to_string(static_cast<ach_status_t>(r)), __FILE__, __LINE__);

	// Print it
	position[0] = waistState->position->data[0];  
	position[1] = waistState->position->data[1];  
	somatic__motor_state__free_unpacked(waistState, &protobuf_c_system_allocator);
	printf("waist positions: %7.3lf, %7.3lf\r", position[0]*180.0/M_PI, position[1]*180.0/M_PI );
	fflush(stdout);
}

/* ********************************************************************************************* */
/// Prints the position data from the Schunk modules
void printSchunk (const char* name) {
	somatic_motor_update(&somaticContext, &motors);
	printf("%s value(s): ", name);
	for(size_t i = 0; i < numModules; i++) { 
		printf("%7.3lf", motors.pos[i]); 
		if(i != (numModules - 1)) printf(", "); 
	}
	printf("\r"); fflush(stdout);
}

/* ********************************************************************************************* */
/// Sends zero position to the grippers or the arms
void tapeSchunk () {

	// Create the zero position vector and send it as position command
	double q [numModules];
	for(size_t i = 0; i < numModules; i++) q[i] = 0.0;
	somatic_motor_cmd(&somaticContext, &motors, POSITION, q, numModules,	NULL);
}

/* ********************************************************************************************* */
void run () {
	
	// Send a message; set the event code and the priority
	somatic_d_event(&somaticContext, SOMATIC__EVENT__PRIORITIES__NOTICE, 
			SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

	// Unless an interrupt or terminate message is received, process the new message
	while(!somatic_sig_received) {

		// Send zero position messages if requested - 'cause Schunk modules have the same interface, 
		// grippers and arms are under one function
		// NOTE Need to continue to let the user Ctrl-C out of the program and send halt messages
		// to the modules
		if(goToTapeMarks && (group != WAIST)) {
			tapeSchunk();
			continue;
		}

		// The user adjusts the positions of the motors with either joystick or by hand (current mode)
		if(!setValues) {

			// Send commands in either joystick or current mode
			if(controlMode == 'j') {
				
				// Read Joystick Channel
				ach_status_t r;
				Somatic__Joystick *js_msg = SOMATIC_GET_LAST_UNPACK(r, somatic__joystick, 
					&protobuf_c_system_allocator, 4096, &joystickChan);
				if(!((ACH_OK == r || ACH_MISSED_FRAME == r) && (js_msg != NULL))) continue; 

				// Make the joystick call for the specific motor group
				if(group == WAIST) joystickWaist(js_msg);
				else if((group == LG_SCHUNK) || (group == RG_SCHUNK)) joystickGrippers(js_msg);
				else if(group == TORSO) joystickTorso(js_msg);
				else joystickArms(js_msg);
	
				// Free the joystick message
				somatic__joystick__free_unpacked( js_msg, &protobuf_c_system_allocator );
			}

			// If current-mode is chosen by the user, set the waist-mode to be sent to krang-waist
			// daemon as CURRENT_MODE 
			else if (controlMode == 'c') {
				if(group == WAIST) currentWaist();
				else currentSchunk();
			}
			
			// Print the positions of the motor
			if(group == WAIST) printWaist();
			else if((group == LG_SCHUNK) || (group == RG_SCHUNK)) printSchunk("Gripper");
			else if(group == TORSO) printSchunk("Torso");
			else printSchunk("Arm");
			usleep(1e5);
		}

		// When the user enters 'a' program state changes to allow user to enter new values for 
		// the positions. See kbhit() function above 
		else {

			// Set the values to zero in the beginning
			double* newPositions = new double [numModules];
			for(size_t i = 0; i < numModules; i++) newPositions[i] = 0.0;

			// Set the new positions to the input values, otw its already zero for '0' input
			if(!makePositionsZero) {
				cout << endl << endl << "Enter new motor positions in degrees:" << endl;
				for(int i=0; i<numModules; i++) {
					cout << i+1 << ". ";
					cin >> newPositions[i];
					newPositions[i]*=M_PI/180.0;
				}
			}

			// Send the pciod command to set the values
			// FIXME: Too much hard-coding. Maybe the bus numbers(-b) and motor numbers(-m) can be
			// detected in run time.
			char str[512];
			if(group == WAIST) {
				sprintf(str,"pciod -S PARAM_ACT_POS -b 11 -m 14 -x %lf -m 15 -x %lf -v -v", 
					newPositions[0], newPositions[1]);
				system(str);
			}
			else if(group == LG_SCHUNK) {
				sprintf(str, "pciod -S PARAM_ACT_POS -b 2 -m 12 -x %lf -v -v", newPositions[0]);
				system(str);
			}
			else if(group == RG_SCHUNK) {
				sprintf(str, "pciod -S PARAM_ACT_POS -b 10 -m 12 -x %lf -v -v", newPositions[0]);
				system(str);
			}
			else if(group == TORSO) {
				sprintf(str, "pciod -S PARAM_ACT_POS -b 11 -m 13 -x %lf -v -v", newPositions[0]);
				system(str);
			}
			else { 

				// Set the bus value based on the arm choice
				int bus[2];
				if(group == LEFT_ARM) { bus[0] = 0; bus[1] = 1; }
				else { bus[0] = 8; bus[1] = 9; }

				// Send the commands for each motor one by one
				for(size_t i = 0; i < 7; i++) {
					sprintf(str, "pciod -S PARAM_ACT_POS -b %d -m %d -x %lf -v -v", bus[i > 3], i + 4, newPositions[i]);
					printf("\n\n\ncommand: '%s'\n\n\n", str); fflush(stdout);
					system(str);
					usleep(1e6);
				}
			} 

			// Print the motor values before leaving
			if(group == WAIST) printWaist();
			else if((group == LG_SCHUNK) || (group == RG_SCHUNK)) printSchunk("Gripper");
			else if(group == TORSO) printSchunk("Torso");
			else printSchunk("Arm");

			// Print some useful information
			cout << "\nThe result of setting the new positions is displayed above. ";
			cout << "If successful, the new positions are displayed in radians. In case of failure";
			cout << "look at the error messages returned by pcio, troubleshoot and run the program";
			cout << "again." << endl;
			break;
		}
		
		aa_mem_region_release(&somaticContext.memreg);
	}
	// Send the stoppig event
	somatic_d_event(&somaticContext, SOMATIC__EVENT__PRIORITIES__NOTICE,
					 SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);
}

/* ********************************************************************************************* */
void destroy() {

	// Clean up the stuff we created based on the choice of group
	if(group == WAIST) {

		// Send message to the krang-waist daemon
		static Somatic__WaistCmd *waistDaemonCmd=somatic_waist_cmd_alloc();
		somatic_waist_cmd_set(waistDaemonCmd, SOMATIC__WAIST_MODE__STOP);
		somatic_metadata_set_time_now(waistDaemonCmd->meta);
		somatic_metadata_set_until_duration(waistDaemonCmd->meta, .1);
		ach_status_t r = SOMATIC_PACK_SEND( &cmdChan, somatic__waist_cmd, waistDaemonCmd );
		if(ACH_OK != r) fprintf(stderr, "Couldn't send message: %s\n", ach_result_to_string(r));

		// Close the values
		somatic_d_channel_close(&somaticContext, &stateChan);
		somatic_d_channel_close(&somaticContext, &cmdChan);
	}
	else 
		somatic_motor_cmd(&somaticContext, &motors, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, numModules, 
			NULL);

	// Close the channel and end the daemon
	somatic_d_channel_close(&somaticContext, &joystickChan);
	somatic_d_destroy(&somaticContext);
}

/* ********************************************************************************************* */
void init(const int argc, char** argv) {
	
	// =================================================================================
	// Handle daemon startup (with joystick)

	// Parse input arguments
	argp_parse (&argp, argc, argv, 0, NULL, NULL);

	// Initialize the somatic context
	somatic_d_init(&somaticContext, &somaticOptions);
	
	// Open ach channel for the joystick
	somatic_d_channel_open(&somaticContext, &joystickChan, "joystick-data", NULL);

	// =================================================================================
	// Create channels for the given group

	// Make sure a group is assigned
	if(group == NONE) {
		printf("Set a group name with -m flag!\n");
		exit(0);
	}

	// Open channels for the waist daemon 
	if(group == WAIST) {
		somatic_d_channel_open(&somaticContext, &stateChan, stateChanName.c_str(), NULL);
		somatic_d_channel_open(&somaticContext, &cmdChan, cmdChanName.c_str(), NULL);
	}

	// Or create motor modules for the left or right arm
	else if((group == LEFT_ARM) || (group == RIGHT_ARM) || (group == LG_SCHUNK) || 
			(group == RG_SCHUNK) || (group == TORSO)) {

		// Initialize the group
		somatic_motor_init(&somaticContext, &motors, numModules, cmdChanName.c_str(), 
			stateChanName.c_str());
		usleep(1e5);

		// Set the min/max values for valid and limits values
		double** limits [] = {
			&motors.pos_valid_min, &motors.vel_valid_min, 
			&motors.pos_limit_min, &motors.pos_limit_min, 
			&motors.pos_valid_max, &motors.vel_valid_max, 
			&motors.pos_limit_max, &motors.pos_limit_max};
		for(size_t i = 0; i < 4; i++) aa_fset(*limits[i], -1024.1, numModules);
		for(size_t i = 4; i < 8; i++) aa_fset(*limits[i], 1024.1, numModules);
		
		// Update and reset them
		somatic_motor_update(&somaticContext, &motors);
		somatic_motor_cmd(&somaticContext, &motors, SOMATIC__MOTOR_PARAM__MOTOR_RESET, NULL, 
			numModules, NULL);
		usleep(1e5);
	}

	// =================================================================================
	// Create the keyboard thread

	pthread_t kbhitThread;
	pthread_create(&kbhitThread, NULL, &kbhit, NULL);

	// =================================================================================
	// Explain the process and get the mode

	// If the user demanded the tape mode, warn them and continue
	if(goToTapeMarks) {

		// Give a warning message before showing them the values
		cout << "Taking the modules to their taped marks can be dangerous if they are not zeroed"
		" properly. Take a look at the readings below. Btw, does not work waist =)" << endl;

		// Print the positions of the motors
		if(group == WAIST) printWaist();
		else if((group == LG_SCHUNK) || (group == RG_SCHUNK)) printSchunk("Gripper");
		else if(group == TORSO) printSchunk("Torso");
		else printSchunk("Arm");
		cout << endl;
		cout << endl;

		// Ask for the consent of the user
		cout << "Do you want to continue? [Enter/Ctrl-C] ";
		cin.get();
		cout << "\nModules will move to their taped marks now! Ctrl-C would halt them!\n" << endl;

		return;
	}

	// Scare the user
	cout << "\nYou can home the modules either with the joystick or move them yourself in current\n"
	"mode. Once you start, the positions will be shown and you can change them until desired\n"
	"values. If you don't know what you're doing, ctrl-c NOW!\n";

	// Get the mode
	char type;
	do {
		cout << "\nJoystick or current mode? [j/c] ";
    cin >> type;
	} while(!cin.fail() && type!='j' && type!='c');
	controlMode = type;

	// Give instructions based on the mode
	if(controlMode=='j') cout << "\nJOYSTICK CONTROL MODE:\n Use joystick to move the motors";
	else cout << "\nMANUAL MODE:\nManually move the motors";
	cout << " and verify that the positions displayed are correct. [a+Enter] to adjust values,"
  " [0+Enter] to set positions to zero, [Ctrl+C] to exit\n" << endl;
}

/* ********************************************************************************************* */
int main(int argc, char ** argv) {

	// Set the somatic context options
	somaticOptions.ident = "home";
	somaticOptions.sched_rt = SOMATIC_D_SCHED_NONE; // logger not realtime
	somaticOptions.skip_mlock = 0; // logger not realtime, other daemons may be

	init( argc, argv );
	cout << "Initalized the program" << endl;
	run();
	cout << "Run finished" << endl;
	destroy();
	cout << "Destroyed resources. Exit complete." << endl;

	return 0;
}
