/**
 * @file helpers.h
 * @author Can Erdogan
 * @date June 12, 2013
 * @brief This file contains some helper functions such as reading force/torque data if available.
 */

#pragma once

#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <syslog.h>
#include <fcntl.h>
#include <argp.h>
#include <Eigen/Dense>
#include <iomanip>

#include "somatic.h"
#include "somatic/daemon.h"
#include <somatic.pb-c.h>
#include <somatic/motor.h>
#include <ach.h>

#include <imud.h>

// #include "kinematics.h"
#include "initModules.h"
#include "motion.h"
#include <iostream>

#include <kinematics/BodyNode.h>
#include <math/UtilsRotation.h>
#include <robotics/parser/dart_parser/DartLoader.h>
#include <simulation/World.h>

using namespace Eigen;

/* ******************************************************************************************** */
#define pv(VEC) std::cout << std::setw(20) << std::left << #VEC; for(int i = 0; i < VEC.size(); i++) std::cout << std::setw(12) << VEC[i]; std::cout << std::endl;
/* #define pv(x) std::cout << #x << ": " << (x).transpose() << std::endl; */

#define parm (cout << llwa.pos[0] << ", " << llwa.pos[1] << ", " << llwa.pos[2] << ", " << \
	llwa.pos[3] << ", " << llwa.pos[4] << ", " << llwa.pos[5] << ", " << llwa.pos[6] << endl);

#define darm (cout << "dq: "<<llwa.vel[0] << ", " <<llwa.vel[1] << ", " << llwa.vel[2] << ", " << \
	llwa.vel[3] << ", " << llwa.vel[4] << ", " << llwa.vel[5] << ", " << llwa.vel[6] << endl);

#define eig7(x) (Vector7d() << (x)[0], (x)[1], (x)[2], (x)[3], (x)[4], (x)[5], (x)[6]).finished()

#define VELOCITY SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY
#define POSITION SOMATIC__MOTOR_PARAM__MOTOR_POSITION

/* ******************************************************************************************** */
typedef Matrix<double, 6, 1> Vector6d;			///< A typedef for convenience to contain f/t values
typedef Matrix<double, 7, 1> Vector7d;			///< A typedef for convenience to contain joint values
typedef Matrix<double, 6, 6> Matrix6d;			///< A typedef for convenience to contain wrenches

/* ******************************************************************************************** */
static const double eeMass = 1.6 + 0.169 + 0.000;			///< The mass of the schunk end-effector
//static const double eeMass = 2.3 + 0.169 + 0.000;			///< The mass of the robotiq end-effector
simulation::World *world = NULL;									///< The dart environment loaded for kinematics

/* ******************************************************************************************** */
/// Set the vector from the sensor origin to the gripper center of mass (m)
static const Vector3d s2com (0.0, 0.0, 0.09); // 0.0683 schunk itself, 0.026 length of ext + 2nd
//static const Vector3d s2com (0.0, -0.008, 0.091); // 0.065 robotiq itself, 0.026 length of ext + 2nd

/// Returns the representation of the end-effector frame in the base frame
void forwardKinematics (const somatic_motor_t& llwa, MatrixXd& Tbee);

/// Computes the initial offset from the given first raw value
void computeOffset (double imu, double waist, const somatic_motor_t& llwa, const Vector6d& raw, 
		dynamics::SkeletonDynamics& robot, Vector6d& offset, bool left);

/// Computes the external force and torque from the values assuming that the input is already
/// corrected for the effect of gravity. That is the arm readings should reflect the weight
/// of the end-effector when there are no external inputs.
void computeExternal (double imu, double waist, const somatic_motor_t& llwa, const Vector6d& input, 
		dynamics::SkeletonDynamics& robot, Vector6d& external, bool left);

/// Initializes the daemon, joystick/ft channels, left arm and computes the initial offset for ft
void init (somatic_d_t& daemon_cx, ach_channel_t& js_chan, ach_channel_t& imuChan, 
		ach_channel_t& waistChan, ach_channel_t& ft_chan, somatic_motor_t& llwa, Vector6d& offset,
		bool left);

/// Returns the f/t data if available at that instance
bool getFT (somatic_d_t& daemon_cx, ach_channel_t& ft_chan, Vector6d& data);

/// Reads imu channel 
void getImu(double* imu, ach_channel_t& imuChan);

/// Reads waist channel 
bool getWaist(double* waist, ach_channel_t& waistChan);

