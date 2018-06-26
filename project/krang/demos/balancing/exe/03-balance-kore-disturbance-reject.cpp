/**
 * @file 01-balance.cpp
 * @author Munzir Zafar
 * @date July 26, 2013
 * @brief This code implements the balancing with force-compensations along with basic joystick 
 * control mainly for the demo on July 31st, 2013 using the newly developed kore library.
 */

#include "helpers.h"
#include "kore/display.hpp"

using namespace std;
using namespace dynamics;
using namespace Krang;

//For disturbance rejection purposes
double theta_hat, theta_dot_hat, f_theta, com_hat, com_dot_hat, f_com;

/* ******************************************************************************************** */
// For logging purposes

int Krang::COLOR_RED_BACKGROUND = 11;
int Krang::COLOR_YELLOW_BACKGROUND = 12;
int Krang::COLOR_GREEN_BACKGROUND = 13;
int Krang::COLOR_WHITE_BACKGROUND = 14;
int Krang::curses_display_row = 30;
int Krang::curses_display_precision = 15;
bool Krang::doing_curses = false;

struct LogState {
	
	// Read state and sensors
	double time;
	Vector3d com;
	double averagedTorque, torque;
	double amcLeft, amcRight;

	// Controller stuff
	Vector6d state, refState;
	double lastUleft, lastUright;

	/// Constructor
	LogState (double t, const Vector3d& c, double aTo, double to, double aL, double aR, 
		const Vector6d& s, const Vector6d& rS, double lUl, double lUr) :
			time(t), com(c), averagedTorque(aTo), torque(to), amcLeft(aL), amcRight(aR), state(s), 
			refState(rS), lastUleft(lUl), lastUright(lUr) {}

	/// Print
	void print () {
		//       torques         currents        state        refstate       time
		printf("%lf\t%lf\t  %lf\t%lf\t%lf\t%lf\t  %lf\t%lf\t%lf\t %lf\t%lf\t%lf\t %lf\n", 
				averagedTorque, torque, lastUleft, lastUright, krang->amc->cur[0], krang->amc->cur[1],  
				state(0)*180.0/M_PI, state(2)*180.0/M_PI, state(4)*180.0/M_PI, refState(0)*180.0/M_PI, 
				refState(2)*180.0/M_PI, refState(4)*180.0/M_PI, time);
	}
	
};

/// The vector of states
vector <LogState*> logStates;

/* ******************************************************************************************** */
// Offset values for FT sensing

Vector6d leftOffset; 
Vector6d leftWheelWrench;
Vector6d rightOffset;
Vector6d rightWheelWrench;
bool debugGlobal = false, logGlobal = true;

/* ******************************************************************************************** */
void computeSpin (double& u_spin) {

	// static const double goal = 10.0;
	static const double kp = 1.0;
	static const double ki = 0.05;
	static const size_t numErrors = 20;
	static size_t c_ = 0;

	static vector <double > errors;
	if(errors.empty()) 
		for(size_t i = 0; i < numErrors; i++) errors.push_back(0);

/*
	if(fabs(goal) < 0.1 && (krang->fts[LEFT]->lastExternal.topLeftCorner<3,1>().norm() < 7)) {
		u_spin = 0.0;
		return;
	}
*/

	double state = krang->fts[LEFT]->lastExternal(1);
	double error = state - spinGoal;
	errors[c_++ % numErrors] = error;
	double totalError = 0.0;
	for(size_t i = 0; i < numErrors; i++) totalError += errors[i];
	
	u_spin = kp * error  + ki * totalError;	
	if(debugGlobal) {
		cout << "\tft y: " << state << endl;
		cout << "\tspin goal: " << spinGoal << endl;
	}
	
}

/* ******************************************************************************************** */
/// Computes the wrench on the wheels due to external force from the f/t sensors' data
void getExternalWrench (Vector6d& external) {

	// If the wrench sensed on left FT sensor is not high use it calculate wrench on the wheel
	if((krang->fts[LEFT]->lastExternal.topLeftCorner<3,1>().norm() > 7) || 
		 (krang->fts[LEFT]->lastExternal.bottomLeftCorner<3,1>().norm() > 0.4)) {
		computeWheelWrench(krang->fts[LEFT]->lastExternal, *robot, leftWheelWrench, true);
	}
	else leftWheelWrench = Vector6d::Zero();

	// If the wrench sensed on right FT sensor is not high use it calculate wrench on the wheel
	if((krang->fts[RIGHT]->lastExternal.topLeftCorner<3,1>().norm() > 7) || 
		 (krang->fts[RIGHT]->lastExternal.bottomLeftCorner<3,1>().norm() > 0.4)) {
		computeWheelWrench(krang->fts[RIGHT]->lastExternal, *robot, rightWheelWrench, false);
	}
	else rightWheelWrench = Vector6d::Zero();
			
	// Sum the wheel wrenches from the two f/t sensors
	// NOTE: In kore library, the FT sensing is returning the negative of the sensed force so we 
	// have to negate it back to get the sensed force
	external = -leftWheelWrench - rightWheelWrench;
	if(debugGlobal) cout << "tangible torque: " << external(4) << " (" << -leftWheelWrench(4) <<
		", " << -rightWheelWrench(4) << ")" << endl;
}

/* ******************************************************************************************** */
/// Computes the reference balancing angle from center of mass, total mass and the felt wrenches.
/// The idea is that we want to set the balancing angle so that the torque due to the external 
/// force/torques and those due to the mass of the robot cancel each other out. 
/// Let com_x be the x component of the desired com such that com_x * mass * gravity = external
/// torque. Now, given that we know com, we can compute its distance from the origin, and then
/// compute the z component. The atan2(x,z) is the desired angle.
void computeBalAngleRef(const Vector3d& com, double externalTorque, double& refImu) {

	// Compute the x component of the desired com
	static const double totalMass = 142.66;
	double com_x = externalTorque / (totalMass * 9.81);

	// Compute the z component of the desired com by first computing the norm (which is fixed)
	// and then computing the z from x component
	double normSq = com(0) * com(0) + com(2) * com(2);
	double com_z = sqrt(normSq - com_x * com_x);

	// Compute the expected balancing angle	
	refImu = atan2(-com_x, com_z);
}

/* ********************************************************************************************* */
// The preset arm configurations: forward, thriller, goodJacobian
double presetArmConfs [][7] = {
	{  0.500, -0.600,  0.000, -1.000,  0.000, -1.450,  0.570},
	{ -0.500,  0.600,  0.000,  1.000,  0.000,  1.450, -0.480},
	{  1.130, -1.000,  0.000, -1.570, -0.000,  1.000,  -1.104},
	{ -1.130,  1.000, -0.000,  1.570,  0.000, -1.000,  -0.958},
	{  1.400, -1.000,  0.000, -0.800,  0.000, -0.500,  -1.000}, 
	{ -1.400,  1.000,  0.000,  0.800,  0.000,  0.500,  -1.000},
	{  0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000},
	{  0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000},
};

/* ********************************************************************************************* */
/// Controls the arms
void controlArms () {

	// Return if the x[3] is being used for robotiq hands
	if(fabs(x[3]) > 0.2) return;

	// Check if one of the preset configurations are requested by pressing 9 and
	// any of the buttons from 1 to 4 at the same time
	if(((b[4] == 1) && (b[6] == 1)) || ((b[5] == 1) && (b[7] == 1))) {

		// Check if the button is pressed for the arm configuration is pressed, if so send pos commands
		bool noConfs = true;
		for(size_t i = 0; i < 4; i++) {
			if(b[i] == 1) {
				if((b[4] == 1) && (b[6] == 1)) 
					somatic_motor_cmd(&daemon_cx, krang->arms[LEFT], POSITION, presetArmConfs[2*i], 7, NULL);
				if((b[5] == 1) && (b[7] == 1))  {
					somatic_motor_cmd(&daemon_cx, krang->arms[RIGHT], POSITION, presetArmConfs[2*i+1], 7, NULL);
				}
				noConfs = false; 
				return;
			}
		}
		
		// If nothing is pressed, stop the arms
		if(noConfs) {
			double dq [] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
			somatic_motor_cmd(&daemon_cx, krang->arms[LEFT], VELOCITY, dq, 7, NULL);
			somatic_motor_cmd(&daemon_cx, krang->arms[RIGHT], VELOCITY, dq, 7, NULL);
			return;
		}
	}
	
	// Check the b for each arm and apply velocities accordingly
	// For left: 4 or 6, for right: 5 or 7, lower arm button is smaller (4 or 5)
	somatic_motor_t* arm [] = {krang->arms[LEFT], krang->arms[RIGHT]};
	for(size_t arm_idx = 0; arm_idx < 2; arm_idx++) {

		// Initialize the input
		double dq [] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

		// Change the input based on the lower or higher button input
		bool inputSet = true;
		size_t lowerButton = 4 + arm_idx, higherButton = 6 + arm_idx;
		if(b[lowerButton] && !b[higherButton]) memcpy(&dq[4], x, 3*sizeof(double));
		else if(!b[lowerButton] && b[higherButton]) memcpy(dq, x, 4*sizeof(double));
		else inputSet = false;
		
		// Set the input for this arm
		if(inputSet) somatic_motor_cmd(&daemon_cx, arm[arm_idx], VELOCITY, dq, 7, NULL);
	}
}

/* ********************************************************************************************* */
/// Handles the joystick commands for the waist module
void controlWaist() {

	// Set the mode we want to send to the waist daemon
	Somatic__WaistMode waistMode;
	if(x[5] < -0.9) waistMode = SOMATIC__WAIST_MODE__MOVE_FWD;
	else if(x[5] > 0.9) waistMode = SOMATIC__WAIST_MODE__MOVE_REV;
	else waistMode = SOMATIC__WAIST_MODE__STOP;

	// Send message to the krang-waist daemon
	somatic_waist_cmd_set(waistDaemonCmd, waistMode);
	int r = SOMATIC_PACK_SEND(krang->waistCmdChan, somatic__waist_cmd, waistDaemonCmd);
	if(ACH_OK != r) fprintf(stderr, "Couldn't send message: %s\n", 
		ach_result_to_string(static_cast<ach_status_t>(r)));
}

/* ********************************************************************************************* */
/// Control the Robotiq hands - button 5 or 6 sets left/right, axis 3 sets the direction
/// TODO: Find a better joystick to command mapping. The results are not very intuitive.
void controlRobotiq() {

	// Return if x[2] is being used for the L7/R7 motors
	if(fabs(x[2]) > 0.2) return;

	// Skip some loops
	static int counter = 0;
	if(counter++ % 20 != 0) return;

	// For each hand, check if the button is pressed and if so give the command
	for(size_t i = 0; i < 2; i++) {

		// Turn the axis value between [-1, 1], to [0,255] - if 0, do nothing.
		int val = fmax(0, fmin(255, ((x[3] + 1.0) / 2.0) * 256));

		// Write the command line command and execute it
		char buf [256];
		if((b[4 + i] == 1) && (fabs(x[3]) > 0.05))  {
			sprintf(buf, "robotiq_cmd b gp%x -c %cgripper-cmd", val, i ? 'r' : 'l');
			printf("'%s'\n", buf);
			system(buf);
		}
	}
}

/* ********************************************************************************************* */
/// Handles the joystick commands for the left/right Schunk grippers
void controlSchunkGrippers () {

	// Button 4 with top/down at the right circular thingy indicates a motion for the left gripper
	double dq [] = {0.0};
	dq[0] = x[3] * 10.0;
	if(b[4]) 
		somatic_motor_cmd(&daemon_cx, krang->grippers[LEFT], SOMATIC__MOTOR_PARAM__MOTOR_CURRENT, dq, 1, NULL);

	// Button 5 with the same circular thingy for the right gripper
	if(b[5]) 
		somatic_motor_cmd(&daemon_cx, krang->grippers[RIGHT], SOMATIC__MOTOR_PARAM__MOTOR_CURRENT, dq, 1, NULL);
}

/* ******************************************************************************************** */
/// The continuous control loop which has 4 state variables, {x, x., psi, psi.}, where
/// x is for the position along the heading direction and psi is the heading angle. We create
/// reference x and psi values from the joystick and follow them with pd control.
void run () {

	// Send a message; set the event code and the priority
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
			SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

	// Initially the reference position and velocities are zero (don't move!) (and error!)
	// Initializing here helps to print logs of the previous state
	Vector6d refState = Vector6d::Zero(), state = Vector6d::Zero(), error = Vector6d::Zero();

	// Read the FT sensor wrenches, shift them on the wheel axis and display
	size_t c_ = 0;
	struct timespec t_now, t_prev = aa_tm_now();
	double time = 0.0;
	Vector6d externalWrench;
	Vector3d com;
	
	// Initialize the running history
	const size_t historySize = 60;
	vector <double> torqueHistory;
	for(size_t i = 0; i < historySize; i++) torqueHistory.push_back(0.0);
	
	// Initialize FT offset stuff
	Vector6d leftFTData, rightFTData, temp;
	leftFTData << 0,0,0,0,0,0;
	rightFTData << 0,0,0,0,0,0; 
	size_t leftFTIter = 0, rightFTIter = 0;
	
	// Continue processing data until stop received
	double js_forw = 0.0, js_spin = 0.0, averagedTorque = 0.0, lastUleft = 0.0, lastUright = 0.0;
	size_t mode4iter = 0, mode4iterLimit = 100;
	size_t lastMode = MODE; bool lastStart = start; 
	while(!somatic_sig_received) {

		bool debug = (c_++ % 20 == 0);
		debugGlobal = debug;
		if(debug) cout << "vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv\n" << endl;

		// Cancel any position built up in previous mode
		if(lastMode != MODE) {
			refState(2) = state(2), refState(4) = state(4);
			lastMode = MODE;
		}
		if(lastStart != start) {
			refState(2) = state(2), refState(4) = state(4);
			lastStart = start;
		}

		// =======================================================================
		// Get inputs: time, joint states, joystick and external forces

		// Get the current time and compute the time difference and update the prev. time
		t_now = aa_tm_now();						
		double dt = (double)aa_tm_timespec2sec(aa_tm_sub(t_now, t_prev));	
		t_prev = t_now;
		time += dt;

		// Get the current state and ask the user if they want to start
		getState(state, dt, &com);
		if(debug) cout << "\nstate: " << state.transpose() << endl;
		if(debug) cout << "com: " << com.transpose() << endl;
		if(debug) cout << "WAIST ANGLE: " << krang->waist->pos[0] << endl;


		// Print the information about the last iteration (after reading effects of it from sensors)
		// NOTE: Constructor order is NOT the print order
		if(logGlobal) {
			logStates.push_back(new LogState(time, com, averagedTorque, externalWrench(4), krang->amc->cur[0],
				krang->amc->cur[1], state, refState, lastUleft, lastUright));
		}

		// Get the joystick input for the js_forw and js_spin axes (to set the gains)
		bool gotInput = false;
		while(!gotInput) gotInput = getJoystickInput(js_forw, js_spin);
		if(debug) cout << "js_forw: " << js_forw << ", js_spin: " << js_spin << endl;

		// Get the wrench on the wheel due to external force
		getExternalWrench(externalWrench);

		// Compute the offsets of the FT sensor again if commanded
		const size_t numResetFTIters = 30;
		if(resetLeftFT) {

			// Accumulate data for LEft FT
			if(debug)	cout << "Resetting left FT" << endl;
			if(krang->fts[LEFT]->getRaw(temp) && leftFTIter < numResetFTIters)  {	leftFTData += temp;	leftFTIter++; }
		
			// If done accumulating data compute the average
			if(leftFTIter == numResetFTIters) {
				leftFTData /= numResetFTIters;
				krang->fts[LEFT]->error(leftFTData, krang->fts[LEFT]->offset, false);
				leftFTData << 0,0,0,0,0,0; 
				leftFTIter = 0;
				resetLeftFT = false;
			}
		} 
		if(resetRightFT) {
			// Accumulate data for right FT
			if(debug)	cout << "Resetting right FT" << endl;
			if(krang->fts[RIGHT]->getRaw(temp) && rightFTIter < numResetFTIters) { rightFTData += temp;	rightFTIter++;}
		
			// If done accumulating data compute the average
			if(rightFTIter == numResetFTIters) {
				rightFTData /= numResetFTIters;
				krang->fts[RIGHT]->error(rightFTData, krang->fts[RIGHT]->offset, false);
				rightFTData << 0,0,0,0,0,0; 
				rightFTIter = 0; 
				resetRightFT = false;
			}
		} 


		// =======================================================================
		// Compute ref state: (1) joystick, (2) running average of external, (3) ref angle

		// Update the reference values for the position and spin
		// NOTE Don't print refState here, the theta ref is going to be overridden
		updateReference(js_forw, js_spin, dt, refState);
		
		// Perform a running average on the felt torque on the wheel by adding the index 
		// and averaging the data again
		averagedTorque = 0.0;
		torqueHistory[c_ % historySize] = externalWrench(4);
		for(size_t i = 0; i < historySize; i++) averagedTorque += torqueHistory[i];
		averagedTorque /= historySize;

		if(overwriteFT) {
			averagedTorque = downGoal;
			if(debug) cout << "averagedTorque overwritten (downGoal): " << averagedTorque << endl;
			if(debug) cout << "\tft x: " << krang->fts[LEFT]->lastExternal(0) << endl;
			if(debug) cout << "\tft z: " << krang->fts[LEFT]->lastExternal(2) << endl;
		}
		
		// Compute the balancing angle reference using the center of mass, total mass and felt wrench.
		if(complyTorque) { 
			if(debug) cout << "Force-Responsive Balancing ..." << endl;
			computeBalAngleRef(com, averagedTorque, refState(0));
		}
		if(debug) cout << "refState: " << refState.transpose() << endl;

		// =======================================================================
		// Apply control: compute error, threshold and send current

		// Compute the error term between reference and current, and weight with gains (spin separate)
		if(debug) cout << "K: " << K.transpose() << endl;
		error = state - refState;

		// If in ground Lo mode and waist angle increases beyond 150.0 goto groundHi mode
		if(MODE == 1) {
			if((krang->waist->pos[0]-krang->waist->pos[1])/2.0 < 150.0*M_PI/180.0) {
				MODE = 6;
				K = K_groundHi;
			}
		}
		// If in ground Hi mode and waist angle decreases below 150.0 goto groundLo mode
		else if(MODE == 6) {
			if((krang->waist->pos[0]-krang->waist->pos[1])/2.0 > 150.0*M_PI/180.0) {
				MODE = 1;
				K = K_groundLo;
			}
		}

		// If we are in the sit down mode, over write the reference
		else if(MODE == 3) {
			static const double limit = ((-103.0 / 180.0) * M_PI);
			if(krang->imu < limit) {
				printf("imu (%lf) < limit (%lf): changing to mode 1\n", krang->imu, limit);
				MODE = 1;
				K = K_groundLo;
			}
			else error(0) = krang->imu - limit;
		}
		// if in standing up mode check if the balancing angle is reached and stayed, if so switch to balLow mode
		else if(MODE == 2) {
			if(fabs(state(0)) < 0.034) mode4iter++;
			// Change to mode 4 (balance low) if stood up enough
			if(mode4iter > mode4iterLimit) {
				MODE = 4;
				mode4iter = 0;
				K = K_balLow;
			}
		}
		// COM error correction in balLow mode
		else if(MODE == 4) {
			// error(0) += 0.005;
		}
		// COM error correction in balHigh mode
		else if(MODE == 5) {
			// error(0) -= 0.005;	
		}
		// disturbance rejection mode
		if(disturb_reject) {
			// only turn on if in a balancing mode
			if (MODE == 4 || MODE == 5) {
				// TODO: robot is skeleton_ptr, we need current q to calculate getSimple
				Eigen::Matrix<double, 18, 1> q;

				// Ask MUNZIR how to get q angles

				// // Extract Joint Angles from Full Model
				// std::vector< BodyNode * > nodes = robot->getBodyNodes();
				// for(auto const& n: nodes) {
				// 	std::string node_name = n->getName();

				// 	// Extract Wheel Parameters
				// 	if(node_name.find("Wheel") != string::npos){
				// 		m_w = n->getMass();
				// 		n->getMomentOfInertia(Iw_xx,Iw_yy,Iw_zz,Iw_xy,Iw_xz,Iw_yz);
				// 		I_wa = Iw_xx;
				// 		r_w = 0.25;
				// 	}

				// 	// Extract Body Parameters
				// 	if(node_name.find("Base") != string::npos){
				// 		M_g = n->getMass();
				// 		n->getMomentOfInertia(I_xx,I_yy,I_zz,I_xy,I_xz,I_yz);
				// 		Eigen::Vector3d com = n->getLocalCOM();
				// 		l_g = com[1];
				// 	}
				// }


				// getSimple(robot,q);
				// after getSimple() is called, we need A,B matrices for LQR
				// Wheeled Inverted Pendulum Parameters
				double I_ra = 0;
				double gamma = 1.0;
				double g = 9.81;
				double c_w = 0.1;

				double r_w, m_w, I_wa; // Wheel Parameters
				double M_g, l_g; // COM Parameters

				double Iw_xx, Iw_yy, Iw_zz, Iw_xy, Iw_xz, Iw_yz; // Wheel Moment of Inertia
				double I_xx, I_yy, I_zz, I_xy, I_xz, I_yz; // Base Moment of Inertia

				double delta, c1, c2; // Intermediate Parameters

				double u_theta, u_com; // Control Signals

				Eigen::VectorXd x_dot(6);
				Eigen::VectorXd x_sys(4);
				Eigen::VectorXd x_obs(6);
				Eigen::VectorXd x_dot_wheel(3);
				Eigen::VectorXd x_dot_com(3);
				Eigen::VectorXd x_obs_wheel(3);
				Eigen::VectorXd x_obs_com(3); 

				// Extract Parameters from Simple Model
				std::vector< BodyNode * > nodesSimple = m3DOF->getBodyNodes();
				for(auto const& n: nodesSimple) {
					std::string node_name = n->getName();

					// Extract Wheel Parameters
					if(node_name.find("Wheel") != string::npos){
						m_w = n->getMass();
						n->getMomentOfInertia(Iw_xx,Iw_yy,Iw_zz,Iw_xy,Iw_xz,Iw_yz);
						I_wa = Iw_xx;
						r_w = 0.25;
					}

					// Extract Body Parameters
					if(node_name.find("Base") != string::npos){
						M_g = n->getMass();
						n->getMomentOfInertia(I_xx,I_yy,I_zz,I_xy,I_xz,I_yz);
						Eigen::Vector3d com = n->getLocalCOM();
						l_g = com[1];
					}
				}


				delta = (M_g*l_g+I_yy+pow(gamma,2)*I_ra)*(M_g+m_w)*pow(r_w,2)+I_wa+I_ra*pow(gamma,2)-pow(M_g*r_w*l_g-I_ra*pow(gamma,2),2);
				c1 = (M_g+m_w)*pow(r_w,2)+I_wa+I_ra*pow(gamma,2)+M_g*r_w*l_g+I_ra*pow(gamma,2);
				c2 = M_g*r_w*l_g+M_g*pow(l_g,2)+I_yy;

				// Hardcode Dynamics + Feedback for Now
				Eigen::MatrixXd A(4,4);
				Eigen::MatrixXd B(4,1);

				Eigen::MatrixXd A_(3,3);
				Eigen::MatrixXd B_1(3,1);
				Eigen::MatrixXd B_2(3,1);
				Eigen::MatrixXd L_1(3,1);
				Eigen::MatrixXd L_2(3,1);
				// Eigen::MatrixXi C_(4,4);

				A << 0, 0, 1, 0,
					 0, 0, 0, 1,
					 ((M_g+m_w)*pow(r_w,2)+I_wa+I_ra*pow(gamma,2))*M_g*g*l_g/delta, 0, -c1*c_w/delta, c1*c_w/delta,
					 (M_g*r_w*l_g-I_ra*pow(gamma,2))*M_g*g*l_g/delta, 0, c2*c_w/delta, -c2*c_w/delta;

				B << 0,
					 0,
					 -c1/delta,
					 c2/delta;
				// TODO: Caclulate K using LQR here. Need a Q and R matrix as well
				Eigen::VectorXd F(4);
				// F = lqr(A,B,Q,R);
				F << -510.449550243191,  -0.244948974282393,  -110.528023245082, -1.14116859943037;
				// Observer Dynamics
				A_ << 0, 1, 0,
					  0, 0, 1,
					  0, 0, 0;

				B_1 << 0,
					   c2/delta,
					   0;

				B_2 << 0,
					   -c1/delta,
					   0;


				L_1 << 1159.99999999673,173438.396407957,1343839.4084839;
				L_2 = L_1;
				// TODO: fill in psi and thetas with proper values from state
				x_sys << com, com_dot, theta, theta_dot;
				//create observer states
				x_obs_com << com_hat, com_dot_hat, f_com;
				x_obs_wheel << theta_hat, theta_dot_hat, f_theta;
				x_obs << x_obs_wheel, x_obs_com;

				// Calculate controls
				Eigen::VectorXd ut1(2); ut1 << F[1], F[3];
				Eigen::VectorXd ut2(2); ut2 << x_obs_wheel[0], x_obs_wheel[1];

				Eigen::VectorXd up1(2); up1 << F[0], F[2];
				Eigen::VectorXd up2(2); up2 << x_obs_com[0], x_obs_com[1];

				// u_theta = F[1]*x_obs_wheel[0] + F[3]*x_obs_wheel[1];
				// u_com = F[0]*x_obs_com[0] + F[2]*x_obs_com[1];
				u_theta = -ut1.transpose()*ut2;
				u_com = -up1.transpose()*up2;

				//Torque to be applied to motors
				double tau = u_theta + u_com - x_obs_com[2] - x_obs_wheel[2];

				// Update osbservers

				x_dot_wheel = A_*x_obs_wheel + L_1*(x_sys[2] - x_obs_wheel[0]) + B_1*u_theta;
				x_dot_com   = A_*x_obs_com   + L_2*(x_sys[0] - x_obs_com[0])   + B_2*u_com;
				x_dot << x_dot_wheel, x_dot_com;
				x_obs = x_obs + x_dot*dt;

				// Save x_obs values for next iteration
				theta_hat = x_obs[0];
				theta_dot_hat = x_obs[1];
				f_theta = x_obs[2];

				com_hat = x_obs[3];
				com_dot_hat = x_obs[4];
				f_com = x_obs[5];

				// Change torques to current for motor input
				// TODO: Verify this is correct ratio and move to top of .cpp
				double motor_constant = 0.85; //Nm/A
				double input [2] = {tau/(2*motor_constant), tau/(2*motor_constant)};

			}
			// Don't attempt disturbance rejection outside of balancing modes
			else {
				printf("Not in a balancing mode. turning off disturbance rejection\n");
				disturb_reject = false;
			}
		}
		// ----Normal balancing code----
		// written as an if statement so that calling disturbance rejection
		// when not in a balancing mode runs this code
		if (!disturb_reject) {
			if(debug) cout << "error: " << error.transpose() << ", imu: " << krang->imu / M_PI * 180.0 << endl;

			// Compute the current
			double u_theta = K.topLeftCorner<2,1>().dot(error.topLeftCorner<2,1>());
			double u_x = K(2)*error(2) + K(3)*error(3);
			double u_spin =  -K.bottomLeftCorner<2,1>().dot(error.bottomLeftCorner<2,1>());
			u_spin = max(-30.0, min(30.0, u_spin));
			// Override the u_spin to exert a force with the end-effector
			if(spinFT) computeSpin(u_spin);

			// Compute the input for left and right wheels
			if(joystickControl && ((MODE == 1) || (MODE == 6))) {u_x = 0.0; u_spin = 0.0;}
			double input [2] = {u_theta + u_x + u_spin, u_theta + u_x - u_spin};
		}
		input[0] = max(-49.0, min(49.0, input[0]));
		input[1] = max(-49.0, min(49.0, input[1]));
		if(debug) printf("u_theta: %lf, u_x: %lf, u_spin: %lf\n", u_theta, u_x, u_spin);
		lastUleft = input[0], lastUright = input[1];

		// Set the motor velocities
		if(start) {
			if(debug) cout << "Started..." << endl;
			somatic_motor_cmd(&daemon_cx, krang->amc, SOMATIC__MOTOR_PARAM__MOTOR_CURRENT, input, 2, NULL);
		}
	
		// =======================================================================
		// Control the arms, waist torso and robotiq grippers based on the joystick input

		if(joystickControl) {
		
			if(debug) cout << "Joystick for Arms and Waist..." << endl;
		
			// Control the arms if necessary
			controlArms();

			// Control the waist
			controlWaist();

		}

		// Control the grippers 
		//controlRobotiq();
		//controlSchunkGrippers();

		// Control the torso
		double dq [] = {x[4] / 7.0};
		somatic_motor_cmd(&daemon_cx, krang->torso, VELOCITY, dq, 1, NULL);

	
		// ==========================================================================
		// Quit if button 9 on the joystick is pressed, stand/sit if button 10 is pressed

		static bool b9Prev = 0; // To store the value of button 9 in last iteration

		// Quit
		if(b[8] == 1) break;

		// Stand/Sit if button 10 is presed and conditions are right
		else if(b9Prev == 0 && b[9] == 1) {

			// If in ground mode and state error is not high stand up
			if(MODE == 1) {
				if(state(0) < 0.0 && error(0) > -10.0*M_PI/180.0)	{
					printf("\n\n\nMode 2\n\n\n");
					K = K_stand;
					MODE = 2;	
				}	else {
					printf("\n\n\nCan't stand up, balancing error is too high!\n\n\n");
				}
			}
			
			// If in balLow mode and waist is not too high, sit down
			else if(MODE == 2 || MODE == 4) {
				if((krang->waist->pos[0] - krang->waist->pos[1])/2.0 > 150.0*M_PI/180.0) {
					printf("\n\n\nMode 3\n\n\n");
					K = K_sit;
					MODE = 3;	
				} else {
					printf("\n\n\nCan't sit down, Waist is too high!\n\n\n");
				}
			}
		}
	
		// Store the value of button 10 in the last iteration
		b9Prev = b[9];

		// Print the mode
		if(debug) printf("Mode : %d\tdt: %lf\n", MODE, dt);
	}

	// Send the stoppig event
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
					 SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);
}

void getSimple(SkeletonPtr threeDOF, Eigen::Matrix<double, 18, 1> q) 
{
	// Load the full body with fixed wheel and set the pose q
	dart::utils::DartLoader loader;
	// TODO: Update this hard-coded location to match
	SkeletonPtr krangFixedWheel =
		loader.parseSkeleton("/home/n8k9/Projects/GaTech/Krang/Krang_Disturbance/dart/3DOF-WIP/krang_fixed_wheel.urdf");
	krangFixedWheel->setName("m18DOF");
	krangFixedWheel->setPositions(q);
	
	// Body Mass
	double mFull = krangFixedWheel->getMass(); 
	double mLWheel = krangFixedWheel->getBodyNode("LWheel")->getMass();
	double mBody = mFull - mLWheel;

	// Body COM
	Eigen::Vector3d bodyCOM;
	dart::dynamics::Frame* baseFrame = krangFixedWheel->getBodyNode("Base");
	bodyCOM = (mFull*krangFixedWheel->getCOM(baseFrame) - mLWheel*krangFixedWheel->getBodyNode("LWheel")->getCOM(baseFrame))/(mFull - mLWheel);

	// Body inertia
	int nBodies = krangFixedWheel->getNumBodyNodes();
	Eigen::Matrix3d iMat;
	Eigen::Matrix3d iBody = Eigen::Matrix3d::Zero();
	double ixx, iyy, izz, ixy, ixz, iyz;  
	Eigen::Matrix3d rot;
	Eigen::Vector3d t;
	Eigen::Matrix3d tMat;
	dart::dynamics::BodyNodePtr b;
	double m;
	for(int i=1; i<nBodies; i++){ // Skipping LWheel
		b = krangFixedWheel->getBodyNode(i);
		b->getMomentOfInertia(ixx, iyy, izz, ixy, ixz, iyz);
		rot = b->getTransform(baseFrame).rotation(); 
		t = bodyCOM - b->getCOM(baseFrame) ; // Position vector from local COM to body COM expressed in base frame
		m = b->getMass();
		iMat << ixx, ixy, ixz, // Inertia tensor of the body around its CoM expressed in body frame
				ixy, iyy, iyz,
				ixz, iyz, izz;
		iMat = rot*iMat*rot.transpose(); // Inertia tensor of the body around its CoM expressed in base frame
		tMat << (t(1)*t(1)+t(2)*t(2)), (-t(0)*t(1)),          (-t(0)*t(2)),
				(-t(0)*t(1)),          (t(0)*t(0)+t(2)*t(2)), (-t(1)*t(2)),
				(-t(0)*t(2)),          (-t(1)*t(2)),          (t(0)*t(0)+t(1)*t(1));
		iMat = iMat + m*tMat; // Parallel Axis Theorem
		iBody += iMat;

	// Aligning threeDOF base frame to have the y-axis pass through the CoM
	double th = atan2(bodyCOM(2), bodyCOM(1));
	rot << 1, 0, 0,
	     0, cos(th), sin(th),
	     0, -sin(th), cos(th);
	bodyCOM = rot*bodyCOM;
	iBody = rot*iBody*rot.transpose();


	// Print them out
	cout << "mass: " << mBody << endl;
	cout << "COM: " << bodyCOM(0) << ", " << bodyCOM(1) << ", " << bodyCOM(2) << endl;
	cout << "ixx: " << iBody(0,0) << ", iyy: " << iBody(1,1) << ", izz: " << iBody(2,2) << endl;
	cout << "ixy: " << iBody(0,1) << ", ixz: " << iBody(0,2) << ", iyz: " << iBody(1,2) << endl;
}


/* ******************************************************************************************** */
/// Initialize the motor and daemons
void init() {

	// Initialize the daemon
	somatic_d_opts_t dopt;
	memset(&dopt, 0, sizeof(dopt)); 
	dopt.ident = "01-balance";
	somatic_d_init(&daemon_cx, &dopt);

	// Initialize the motors and sensors on the hardware and update the kinematics in dart
	int hwMode = Krang::Hardware::MODE_AMC | Krang::Hardware::MODE_LARM | 
		Krang::Hardware::MODE_RARM | Krang::Hardware::MODE_TORSO | Krang::Hardware::MODE_WAIST;
	krang = new Krang::Hardware((Krang::Hardware::Mode) hwMode, &daemon_cx, robot); 

	// Initialize the joystick channel
	int r = ach_open(&js_chan, "joystick-data", NULL);
	aa_hard_assert(r == ACH_OK, "Ach failure '%s' on opening Joystick channel (%s, line %d)\n", 
		ach_result_to_string(static_cast<ach_status_t>(r)), __FILE__, __LINE__);

	// Create a thread to wait for user input to begin balancing
	pthread_t kbhitThread;
	pthread_create(&kbhitThread, NULL, &kbhit, NULL);
	// TODO: use getSimple here to initialize the observer values
	theta_hat = 0;
	theta_dot_hat = 0;
	f_theta = 1;

	com_hat = 0;
	com_dot_hat = 0;
	f_com = 1;
}

/* ******************************************************************************************** */
/// Send zero velocity to the motors and kill daemon. Also clean up daemon structs.
void destroy() {

	cout << "destroying" << endl;

	// ===========================
	// Stop motors, close motor/sensor channels and destroy motor objects
	
	// To prevent arms from halting if joystick control is not on, change mode of krang
	if(!joystickControl) {
		somatic_motor_destroy(&daemon_cx, krang->arms[LEFT]);
		somatic_motor_destroy(&daemon_cx, krang->arms[RIGHT]);
		krang->arms[LEFT] = NULL;
		krang->arms[RIGHT] = NULL;
	}
	delete krang;
		
	// Destroy the daemon resources
	somatic_d_destroy(&daemon_cx);

	// Print the data
	printf("log states size: %lu\n", logStates.size());
	for(size_t i = 0; i < logStates.size(); i++) {
		logStates[i]->print();
		delete logStates[i];
	}
}

/* ******************************************************************************************** */
/// The main thread
int main(int argc, char* argv[]) {

	// Load the world and the robot
	DartLoader dl;
	// world = dl.parseWorld("../../../experiments/common/scenes/01-World-Robot.urdf");
	world = dl.parseWorld("/etc/kore/scenes/01-World-Robot.urdf");
	assert((world != NULL) && "Could not find the world");
	robot = world->getSkeleton(0);

	// Read the gains from the command line
/*
	assert(argc >= 7 && "Where is my gains for th, x and spin?");
	K << atof(argv[1]), atof(argv[2]), atof(argv[3]), atof(argv[4]), atof(argv[5]), atof(argv[6]);
	jsFwdAmp = 0.3;
	jsSpinAmp = 0.4;
	
	cout << "K_bal: " << K_bal.transpose() << "\nPress enter: " << endl;
*/

	readGains();

	// Debug options from command line
	debugGlobal = 1; logGlobal = 0;
	if(argc == 8) { 
		if(argv[7][0]=='l') { debugGlobal = 0; logGlobal = 1;} 
		else if(argv[7][0] == 'd') {debugGlobal = 1; logGlobal = 0; } 
	} 

	getchar();

	// Initialize, run, destroy
	init();
	run();
	destroy();
	return 0;
}
