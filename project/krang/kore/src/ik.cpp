/**
 * @file ik.cpp
 * @author Can Erdogan
 * @date May 03, 2013
 * @brief Handles inverse-kinematics operation for a 7-dof arm with Krang's dimensions.
 */

#include "kore/ik.hpp"
#include "kore/util.hpp"
#include <amino/math.h>

#define transform(x,T) ((T * Eigen::Vector4d(x(0), x(1), x(2), 1.0)).topLeftCorner<3,1>())

bool ignoreIKDist = false;

using namespace Eigen;
using namespace std;

namespace Krang {

/* ********************************************************************************************* */
bool singleArmIKLimitsAndCollsBestWheel (simulation::World* mWorld, 
		dynamics::SkeletonDynamics* krang, const Matrix4d& Twee, bool rightArm, double dtphi, 
		Vector7d& theta) {

	static const bool debug = 1;

	// Perform I.K. with different arm angles until no collisions or limit problems
	vector <int>& arm_ids = rightArm ? right_arm_ids : left_arm_ids;
	Eigen::VectorXd ql = krang->getConfig(arm_ids);
	Eigen::VectorXd qb = krang->getConfig(base_ids);
	bool result, success = false;
	double maxMinDistSq = -16.0;
	Vector7d bestTheta;
	double bestPhi = 0.0;
	for(double phi = 0.0; phi < 2*M_PI; phi+=dtphi) {
	
		// Perform I.K.
		if(debug) cout << "phi: " << phi << endl;
		Vector7d someTheta = VectorXd(7);
		result = singleArmIK(qb, Twee, false, phi, someTheta);
		if(debug) cout << "\tresult: " << result << ", someTheta: " << someTheta.transpose() << endl;
		if(!result) continue;

		// Check for joint angles
		bool badJoint = false;
		for(size_t i = 0; i < 7; i++) {
			kinematics::Dof* dof = krang->getDof(arm_ids[i]);
			if((someTheta(i) < dof->getMin()) || (someTheta(i) > dof->getMax())) {
				badJoint = true;
				break;
			}
		}
		if(debug) cout << "\tjoint angles violated: " << badJoint << endl;
		if(badJoint) continue;

		// Check for collisions
		krang->setConfig(arm_ids, someTheta);
		bool collision = mWorld->checkCollision(false);
		if(debug) cout << "\tcollision: " << collision << endl;
		if(collision) continue;
		success = true;

		// Determine the distance between the closest two links 
		double minDistSq = 16.0;
		const char* names [6] = {"L2", "L3", "L4", "L5", "L6", "lFingerA"}; 
		Eigen::Vector3d wheelLoc = krang->getNode(rightArm ? "RWheel" : "LWheel")->getWorldCOM();
		for(size_t dof_idx = 0; dof_idx < 6; dof_idx++) {

			// Get the link location
			Eigen::Vector3d loc2 = krang->getNode(names[dof_idx])->getWorldCOM();

			// Get the closest location on the wheel to the node
			Eigen::Vector3d dir = (loc2 - wheelLoc).normalized() ;
			dir(1) = 0.0;
			Eigen::Vector3d loc1 = wheelLoc + dir * 0.267;

			// Compute the distance 
			double distSq = (loc1 - loc2).squaredNorm();
			if(distSq < minDistSq) minDistSq = distSq;
	
		}
		
		if(debug) cout << "\tminDistSq: " << minDistSq << " vs. maxMinDistSq: " 
			<< maxMinDistSq << endl;

		// Check if it is more than the current maximum minAngle
		if(minDistSq > maxMinDistSq) {
			maxMinDistSq = minDistSq;	
			bestPhi = phi;
			bestTheta = someTheta;
		}
	}

	if(debug) cout << "bestPhi: " << bestPhi << endl;

	// Set the arm back to its original pose if failure
	theta = bestTheta;
	krang->setConfig(arm_ids, ql);
	return success;
}

/* ********************************************************************************************* */
bool singleArmIKLimitsAndColls (simulation::World* mWorld, dynamics::SkeletonDynamics* krang, 
		const Matrix4d& Twee, bool rightArm, double dtphi, Vector7d& theta) {

	static const bool debug = 0;

	// Perform I.K. with different arm angles until no collisions or limit problems
	vector <int>& arm_ids = rightArm ? right_arm_ids : left_arm_ids;
	Eigen::VectorXd ql = krang->getConfig(arm_ids);
	Eigen::VectorXd qb = krang->getConfig(base_ids);
	bool result, success = false;
	for(double phi = 0.0; phi < 2*M_PI; phi+=dtphi) {
	
		// Perform I.K.
		if(debug) cout << "phi: " << phi << endl;
		result = singleArmIK(qb, Twee, false, phi, theta);
		if(debug) cout << "\tresult: " << result << endl;
		if(!result) continue;

		// Check for joint angles
		bool badJoint = false;
		for(size_t i = 0; i < 7; i++) {
			kinematics::Dof* dof = krang->getDof(arm_ids[i]);
			if((theta(i) < dof->getMin()) || (theta(i) > dof->getMax())) {
				badJoint = true;
				break;
			}
		}
		if(debug) cout << "\tjoint angles violated: " << badJoint << endl;
		if(badJoint) continue;

		// Check for collisions
		krang->setConfig(arm_ids, theta);
		bool collision = mWorld->checkCollision(false);
		if(debug) cout << "\tcollision: " << collision << endl;
		if(!collision) {
			success = true;
			break;
		}
	}

	// Set the arm back to its original pose if failure
	krang->setConfig(arm_ids, ql);
	return success;
}

/* ********************************************************************************************* */
bool singleArmIK (const VectorXd& base_conf, const Matrix4d& Twee, bool rightArm, double phi,
		Vector7d& theta) {
	
	static const bool debug = 0;

	// Get the bracket transform
	Transform <double, 3, Affine> wTba (
		AngleAxis <double> (base_conf(2), Vector3d(0.0, 0.0, 1.0)) *
		AngleAxis <double> (base_conf(3), Vector3d(1.0, 0.0, 0.0)));
	wTba.translation() = Vector3d(base_conf(0), base_conf(1), 0.27);
	Transform <double, 3, Affine> baTs (AngleAxis <double> (base_conf(4), Vector3d(-1.0, 0.0, 0.0)));
	baTs.translation() = Vector3d(0.026, 0.499, -0.091);
	Transform <double, 3, Affine> sTb (AngleAxis <double> (M_PI, Vector3d(0.0, 1.0, 0.0)));
	sTb.translation() = Vector3d(-0.027, 0.667, 0.1088);
	Eigen::Matrix4d Twb = (wTba * baTs * sTb).matrix();

	// Get the relative goal
	Transform <double, 3, Affine> relGoal;
	getWristInShoulder(Twb, Twee, rightArm, relGoal.matrix());

	// Compute the inverse kinematics
	theta = VectorXd (7);
	bool result = ik(relGoal, phi, theta);
	return result;
}

/* ********************************************************************************************* */
/// Returns the goal frame of the wrist in the shoulder frame
void getWristInShoulder (const Matrix4d& Twb_, const Matrix4d& Twee, bool rightArm, 
    Matrix4d& TswM) {

	// =======================================================================
	// Get the bracket frame in the world frame and then ee in bracket frame

	// Need to rotate and move the frame 8.5 cm up in the z frame
	Matrix4d smallChange;	
	smallChange << 
		0.0, 0.0, -1.0, -0.0025, 
		0.0, 1.0, 0.0, 0.00525, 
		1.0, 0.0, 0.0, 0.0008, 
		.0, .0, .0, 1.0;
  MatrixXd Twb = Twb_ * smallChange;
	
	// Flip the x and z axis if working on the right arm
	if(rightArm) {
		Matrix4d flip = Matrix4d::Identity();
		flip(0,0) = flip(2,2) = -1.0;
		Twb = Twb * flip;
	}

	// Get the end-effector in the bracket frame
	Transform <double, 3, Affine> Tbee;
	Eigen::Matrix4d TwbInv = Twb;
	aa_la_inv(4, TwbInv.data());
	Tbee.matrix() = TwbInv * Twee;
	
	// =======================================================================
	// Now, we want the waist frame in the shoulder frame. This requires
	// going backwards from the ee towards the waist and forward from the
	// bracket to the shoulder

	// First of all, we rotate from the lgPlate1, the finger, to the wrist
	Transform <double, 3, Affine> Tsw;
	Matrix3d fingerToWrist;
	fingerToWrist << 0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;
	Tsw.linear() = Tbee.linear() * fingerToWrist;

	// Next, get the wrist location by going backwards from the end-effector towards the wrist
	// I think + () because z3 and z10 are opposite.
	Vector3d posGoal = Tbee.translation() + Tsw.linear() * Vector3d(0.0, 0.0, L7 + L8_Schunk);
	Tsw.translation() = posGoal;

	// Lastly, transform the matrix to the shoulder frame
	Tsw.translation() += Vector3d(0.0, 0.0, L4);
	TswM = Tsw.matrix();
}

/* ********************************************************************************************** */
void getAB (Transform<double, 3, Affine>& A, Transform<double, 3, Affine>& B) {

	// The A matrix is the 4x4 transformation that represents the proximal elbow frame in the
	// distal shoulder frame. In Krang, this represents the 6th frame in the 5th frame.
	Matrix4d& matA = A.matrix();
	matA << 1.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 1.0, 0.0,
         0.0, -1.0, 0.0, -L5,
         0.0, 0.0, 0.0, 1.0;

	// The B matrix is the 4x4 transformation that represents the proximal wrist frame in the
	// distal elbow frame. In Krang, this represents the 7th frame in the 'rotated' 6th frame.
	Matrix4d& matB = B.matrix();
	matB << 1.0, 0.0, 0.0, 0.0,
         0.0, 0.0, -1.0, L6,
         0.0, 1.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 1.0;
}

/* ********************************************************************************************** */
void anglesFromRotationMatrix(double &theta1, double &theta2, double &theta3, const Vector3d &n1, 
		const Vector3d &n2, const Vector3d &n3, const Matrix3d &A) {
	
	double lambda = std::atan2(n3.cross(n2).dot(n1), n3.dot(n1));
	double temp = n1.transpose() * A * n3;
	theta2 = -lambda - acos(temp);
	theta3 = -std::atan2(n1.transpose() * A * n2, -n1.transpose() * A * n3.cross(n2));
	theta1 = -std::atan2(n2.transpose() * A * n3, -n2.cross(n1).transpose() * A * n3);

	if(theta2 < -M_PI) {
		theta2 += 2.0 * M_PI;
	}
}

/* ********************************************************************************************** */
/// Computes a transformation given the DH parameters
Eigen::Matrix4d dh (double a, double alpha, double d, double theta) {

	// Compute the cos and sin functions of the input degrees
	double cth = cos(theta), sth = sin(theta);
	double calp = cos(alpha), salp = sin(alpha);

	// Set the matrix
	Eigen::Matrix4d T;
	T << 
		cth, -sth * calp,  sth * salp, a * cth,
		sth,  cth * calp, -cth * salp, a * sth,
		0.0,        salp,        calp,       d,
		0.0,         0.0,         0.0,     1.0;  

	return T;
}


/* ********************************************************************************************** */
/// Implements Sections 3.2.1 and 3.2.2
void getElbowPosAngle (const Vector3d& relGoalLoc, double Lsw, double phi, Vector3d& elbow, 
		double& theta4) {

	const bool debug = 0;

	// Get the elbow joint angle
	theta4 = M_PI - acos((Lsw*Lsw - L6*L6 - L5*L5)/(-2*L5*L6));
	
	// Compute the direction from shoulder to goal
	const Vector3d n = relGoalLoc.normalized();

	// Compute the center and radius of the circle
	const double cosAlpha = (Lsw*Lsw + L5*L5 - L6*L6) / (2*Lsw*L5);
	const Vector3d c = cosAlpha * L5 * n;
	const double R = sqrt(1 - cosAlpha*cosAlpha) * L5;

	// Create the frame in the circle plane by choosing a random vector a
	const Vector3d a = Vector3d(1.0, 0.0, 0.0);
	const Vector3d u = (a - a.dot(n) * n).normalized();
	const Vector3d v = n.cross(u);

	// Compute the location of the elbow
	elbow = c + R*cos(phi)*u + R*sin(phi)*v;
}

/* ********************************************************************************************** */
/// Returns the transformation between the proximal and distal shoulder frames (3.2.3)
void getT1 (const Transform<double, 3, Affine>& relGoal, 
		const Transform<double, 3, Affine>& Ty, const Vector3d& elbow, 
		Transform<double, 3, Affine>& T1) {

	const bool debug = 0;

	// First, compute the transformation between distal shoulder and 
	// proximal wrist frames, and then, its translation is the position of
	// the wrist in the distal shoulder (world) frame
	Transform<double, 3, Affine> temp (A * Ty.matrix() * B);
	Vector3d w = temp.translation();
	
	// Define the axes for the distal shoulder frame assuming T_1, the 
	// transformation between proximal and distal shoulder frames is 
	// identity (the rest configuration). 
	Vector3d x = A.topRightCorner<3,1>().normalized();
	Vector3d y = (w - w.dot(x) * x);
	
	y = y.normalized();
	Vector3d z = x.cross(y);

	// First, compute the wrist location wrt to the distal shoulder
	// in the goal configuration
	Vector3d wg = relGoal.translation();

	// Then, define the axes for the distal shoulder frame in the
	// goal configuration (T_1 is not I anymore). 
	Vector3d xg = elbow.normalized();
	Vector3d yg = (wg - wg.dot(xg) * xg).normalized();
	Vector3d zg = xg.cross(yg);

	// Create the two rotation matrices from the frames above and
	// compute T1.
	Matrix3d R1r, R1g;
	R1r.col(0) = x; R1r.col(1) = y; R1r.col(2) = z;
	R1g.col(0) = xg; R1g.col(1) = yg; R1g.col(2) = zg;
	T1 = Transform<double, 3, Affine>(R1g * R1r.transpose());
}

/* ********************************************************************************************** */
/// Computes the inverse-kinematics
bool ik (const Transform<double, 3, Affine>& relGoal, double phi, Matrix <double, 7, 1>& theta) {
	
	// Sanity check for triangle inequality computing the length between shoulder and wrist
	double Lsw = relGoal.translation().norm();
	if((Lsw > (L5 + L6))){
		theta(0) = (Lsw - (L5 + L6));
		return false;
	}

	// Get the elbow position and the angle
	Vector3d elbow;
	getElbowPosAngle(relGoal.translation(), Lsw, phi, elbow, theta(3));

	// Create the elbow transformation matrix (only rotation)
	Transform<double, 3, Affine> Te;
	Te.matrix() = dh(0.0, 0.0, 0.0, theta(3));

	// Get the T1 matrix
	Transform <double, 3, Affine> T1;
	getT1(relGoal, Te, elbow, T1);

	// Comptute the transformation between the proximal and distal wrist frames, T2
	Eigen::Matrix4d bla = (T1.matrix() * A * Te.matrix() * B);
	Eigen::Matrix4d blaInv = bla;
	aa_la_inv(4, blaInv.data());
	Transform<double, 3, Affine> T2 (blaInv * relGoal.matrix());

	// Define the two types of motor axes 
	Vector3d type1 = Vector3d(0.0, 0.0, 1.0);
	Vector3d type2 = Vector3d(0.0, 1.0, 0.0);

	// Get the first and last three angles
	anglesFromRotationMatrix(theta(0), theta(1), theta(2), type1, type2, type1, T1.rotation());
	anglesFromRotationMatrix(theta(4), theta(5), theta(6), type1, type2, type1, T2.rotation());

	return true;
}

}; // end of namespace
