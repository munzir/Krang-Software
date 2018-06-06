/**
 * @file helpers.cpp 
 * @author Can Erdogan, Munzir Zafar
 * @date Apr 26, 2013
 * @brief The set of helper functions for the kinematics experiments. These include the 
 * initialization function that handles motor, joystick and perception channels.
 */

#include "helpers.h"
#include "initModules.h"
#include "kinematics.h"
#include <iostream>

using namespace Eigen;
using namespace std;
using namespace kinematics;
using namespace dynamics;

// /* ********************************************************************************************* */
// void getEEinKinectFrame (double* q, Vector3d& pos, Vector3d& dir) {

// 	/// Set the constant
// 	const double th = 159.5 / 180.0 * M_PI;

// 	// =============================================================================
// 	// Start with the pos/ori of the shoulder bracket in Kinect frame. We first 
// 	// compute the transformation from the bracket to the hinge, then to hinge to
// 	// Kinect and then inverse all.
	
// 	// Get the transformations T^b_h and T^h_k
// 	MatrixXd Tbh (4,4), Thk (4,4);
// 	Tbh << cos(th), -sin(th), 0, 11.13, sin(th), cos(th), 0, 9.84, 0, 0, 1, 0, 0, 0, 0, 1;
// 	Thk << 0, 0, 1, 5, 0, 1, 0, -4.25, -1, 0, 0, -1.0, 0, 0, 0, 1;

// 	// Compute the inverses
// 	MatrixXd Tkb = (Tbh * Thk).inverse();

// 	// =============================================================================
// 	// Compute the transformations along each joint

// 	// Create the DH table for the arm
// 	double T [7][4] = {
// 		{0.0, -M_PI_2, -L4, q[0]},
// 		{0.0, M_PI_2, 0.0, q[1]},
// 		{0.0, -M_PI_2, -L5, q[2]},
// 		{0.0, M_PI_2, 0.0, q[3]},
// 		{0.0, -M_PI_2, -L6, q[4]},
// 		{0.0, M_PI_2, 0.0, q[5]},
// 		{0.0, 0.0, -L7-L8, q[6]}};

// 	// Loop through the joints and aggregate the transformations multiplying from left
// 	MatrixXd Tk10 = MatrixXd::Identity(4,4);
// 	Tk10(0,0) = Tk10(2,2) = -1;
// 	for(size_t i = 0; i < 7; i++) 
// 		Tk10 *= dh(T[i][0], T[i][1], T[i][2], T[i][3]);		

// 	static size_t co = 0;
// 	if(false && co++ % 1000 == 0) 
// 		cout << "\n\t\t" << Tk10(0,3) << ", " << Tk10(1,3) << ", " << Tk10(2,3) << endl;
// 	Tk10 = Tkb * Tk10;

// 	// Now we have the transformation T^k_10 where 10 is a frame at the end of the
// 	// end-effector. Add a transform that moves along z to get the tip.
// 	MatrixXd T10_11 = MatrixXd::Identity(4,4);
// 	T10_11(2,3) = 7.0;			// TODO 
// 	MatrixXd Tk11 = Tk10 * T10_11;
	
// 	// =============================================================================
// 	// Now get the position of the end effector and the Euler angles of the rotation
// 	// matrix in Tk11 

// 	// Set position
// 	Vector4d origin (0.0, 0.0, 0.0, 1.0); 
// 	pos = (Tk11 * origin).head<3>();

// 	// Set direction
// 	Vector4d temp (0.0, 0.0, 1.0, 0.0); 
// 	dir = (Tk11 * temp).head<3>();
// }

// // /* ********************************************************************************************* */
// // void init (somatic_d_t& daemon_cx, somatic_motor_t& llwa, somatic_motor_t& rlwa, 
// // 	ach_channel_t& js_chan, ach_channel_t& state_chan, ach_channel_t& chan_transform) {
	
// // 	// Initialize this daemon (program!)
// // 	somatic_d_opts_t dopt;
// // 	memset(&dopt, 0, sizeof(dopt)); // zero initialize
// // 	dopt.ident = "bal-hack";
// // 	somatic_d_init( &daemon_cx, &dopt );

// // 	// Initialize the arms
// // 	initArm(daemon_cx, llwa, "llwa");
// // 	initArm(daemon_cx, rlwa, "rlwa");

// // 	// Initialize the joystick channel
// // 	int r = ach_open(&js_chan, "joystick-data", NULL);
// // 	aa_hard_assert(r == ACH_OK, "Ach failure '%s' on opening Joystick channel (%s, line %d)\n", 
// // 		ach_result_to_string(static_cast<ach_status_t>(r)), __FILE__, __LINE__);
	
// // 	// Open the state and transform channels 
// // 	somatic_d_channel_open(&daemon_cx, &state_chan, "krang-state", NULL);
// // 	somatic_d_channel_open(&daemon_cx, &chan_transform, "chan_transform", NULL);	
// // }

// /* ********************************************************************************************* */
// bool getRedMarkerPosition(somatic_d_t& daemon_cx, ach_channel_t& chan_transform, double* x) {

// 	static const bool debug = 0;

// 	// Check if there is anything to read
// 	int result;
// 	size_t numBytes = 0;
// 	struct timespec abstimeout = aa_tm_future(aa_tm_sec2timespec(1));
// 	uint8_t* buffer = (uint8_t*) somatic_d_get(&daemon_cx, &chan_transform, &numBytes, &abstimeout, 
// 		ACH_O_LAST, &result);

// 	// Return if there is nothing to read
// 	if(numBytes == 0) return false;

// 	// Read the message with the base struct to check its type
// 	Somatic__BaseMsg* msg = somatic__base_msg__unpack(&(daemon_cx.pballoc), numBytes, buffer);
// 	if((msg->meta == NULL) || !msg->meta->has_type) return false;
// 	if(msg->meta->type != SOMATIC__MSG_TYPE__TRANSFORM) return false; 

// 	// Read the force-torque message
// 	Somatic__Transform* message = somatic__transform__unpack(&(daemon_cx.pballoc), numBytes, buffer);
// 	for(size_t i = 0; i < 3; i++)
// 		x[i] = message->translation->data[i];

// 	// Print the values
// 	if(debug) {
// 		printf("[server] transform:\t");
// 		for(size_t i = 0; i < 3; i++)
// 			printf("%6.2f  ", message->translation->data[i]); 
// 		for(size_t i = 0; i < 4; i++)
// 			printf("%6.2f  ", message->rotation->data[i]); 
// 		printf("\n"); fflush(stdout);
// 	}

// 	return true;
// }

// /* ********************************************************************************************* */
// // void init (somatic_d_t& daemon_cx, ach_channel_t& js_chan, ach_channel_t& imuChan, 
// // 		ach_channel_t& waistChan, ach_channel_t& ft_chan, somatic_motor_t& lwa, Vector6d& offset, 
// // 		bool left){

// // 	// Set up the index vectors
// // 	int right_arm_ids_a [] = {11, 13, 15, 17, 19, 21, 23};
// // 	int left_arm_ids_a [] = 	{10, 12, 14, 16, 18, 20, 22};
// // 	int * arm_ids_a = left ? left_arm_ids_a : right_arm_ids_a;
// // 	for(size_t i = 0; i < 7; i++) arm_ids.push_back(arm_ids_a[i]);
// // 	imuWaist_ids.push_back(5);	
// // 	imuWaist_ids.push_back(8);	

// // 	// Restart the netcanft daemon. Need to sleep to let OS kill the program first.
// // 	system("killall -s 9 netcanftd");
// // 	usleep(20000);
// // 	if(left) system("netcanftd -v -d -I lft -b 1 -B 1000 -c llwa_ft -k -r");
// // 	else system("netcanftd -v -d -I rft -b 9 -B 1000 -c rlwa_ft -k -r");

// // 	// Load environment from dart for kinematics
// // 	DartLoader dl;
// // 	world = dl.parseWorld("../../common/scenes/01-World-Robot.urdf");
// // 	assert((world != NULL) && "Could not find the world");

// // 	// Initialize this daemon (program!)
// // 	somatic_d_opts_t dopt;
// // 	memset(&dopt, 0, sizeof(dopt)); // zero initialize
// // 	dopt.ident = "01-gripperWeight";
// // 	somatic_d_init( &daemon_cx, &dopt );

// // 	// Initialize the left arm
// // 	if(left) initArm(daemon_cx, lwa, "llwa");
// // 	else{ initArm(daemon_cx, lwa, "rlwa"); }
// // 	somatic_motor_update(&daemon_cx, &lwa);

// // 	// Initialize the channels to the imu and waist sensors
// // 	somatic_d_channel_open(&daemon_cx, &imuChan, "imu-data", NULL);
// // 	somatic_d_channel_open(&daemon_cx, &waistChan, "waist-state", NULL);

// // 	// Get imu data
// // 	double imu = 0.0;
// // 	for(int i = 0; i < 500; i++) {
// // 		double temp;
// // 		getImu(&temp, imuChan);
// // 		imu += temp;
// // 	}
// // 	imu /= 500;
// // 	cout << "imu : " << imu*180.0/M_PI << endl;

// // 	// Get waist data
// // 	double waist;
// // 	while(!getWaist(&waist, waistChan));
// // 	cout << "waist : " << waist*180.0/M_PI << endl;

// // 	// Initialize the joystick channel
// // 	int r = ach_open(&js_chan, "joystick-data", NULL);
// // 	aa_hard_assert(r == ACH_OK, "Ach failure '%s' on opening Joystick channel (%s, line %d)\n", 
// // 		ach_result_to_string(static_cast<ach_status_t>(r)), __FILE__, __LINE__);

// // 	// Open the state and ft channels
// // 	if(left) somatic_d_channel_open(&daemon_cx, &ft_chan, "llwa_ft", NULL);
// // 	else somatic_d_channel_open(&daemon_cx, &ft_chan, "rlwa_ft", NULL);

// // 	// Get the first force-torque reading and compute the offset with it
// // 	cout << "reading FT now" << endl;
// // 	Vector6d ft_data, temp;
// // 	ft_data << 0,0,0,0,0,0;
// // 	for(size_t i = 0; i < 1e3; i++) {
// // 		bool gotReading = false;
// // 		while(!gotReading) 
// // 			gotReading = getFT(daemon_cx, ft_chan, temp);
// // 		ft_data += temp;
// // 	}
// // 	ft_data /= 1e3;
	
// // 	computeOffset(imu, waist, lwa, ft_data, *(world->getSkeleton(0)), offset, left);
// // }
