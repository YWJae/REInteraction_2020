#pragma once
#include "controller.h"
#include <iostream>
#include <iomanip>
#include <cmath>

#define ERROR_TOLERANCE 0.001 // [m]

using namespace DyrosMath;
ofstream logfile;

void ArmController::logData(Eigen::Vector6d x_error) {
	Vector3d position_target; position_target << 0.25, 0.28, 0.65;
	Matrix3d rotation_target; rotation_target << 0, -1, 0,
												 -1, 0, 0,
												 0, 0, -1;

	Vector3d phi_desired;	phi_desired = -getPhi(EYE(3), rotation_target);
	Vector3d phi;			phi = -getPhi(EYE(3), rotation_);

	logfile << play_time_ << "\t"
			<< position_target(0) << "\t" << x_(0) << "\t"
			<< position_target(1) << "\t" << x_(1) << "\t"
			<< position_target(2) << "\t" << x_(2) << "\t"
			<< phi_desired(0) << "\t" << phi(0) << "\t"
			<< phi_desired(1) << "\t" << phi(1) << "\t"
			<< phi_desired(2) << "\t" << phi(2) << "\t"

			//<< x_error(0) << "\t"
			//<< x_error(1) << "\t"
			//<< x_error(2) << "\t"
			//<< x_error(3) << "\t"
			//<< x_error(4) << "\t"
			//<< x_error(5) << "\t"
			<< endl;

			//logfile << play_time_ << "\t";
			//for (int i = 0; i < x_error.size(); i++) {
			//	logfile << x_error(i) << "\t";
			//}
			//logfile << endl;
}

void ArmController::logData_TaskTransition(Eigen::Vector3d x1, Eigen::Vector3d x2) {
	Vector3d x1_target;		x1_target << 0.25, 0.28, 0.65;
	Vector3d x2_target;		x2_target << 0.0, -0.15, 0.6;

	logfile << play_time_ << "\t"
			<< x1_target(0) << "\t" << x1_(0) << "\t"
			<< x1_target(1) << "\t" << x1_(1) << "\t"
			<< x1_target(2) << "\t" << x1_(2) << "\t"
			<< x2_target(0) << "\t" << x2_(0) << "\t" 
			<< x2_target(1) << "\t" << x2_(1) << "\t" 
			<< x2_target(2) << "\t" << x2_(2) << "\t"
			<< endl;
}

void ArmController::logData_JointError(Eigen::Vector7d q_target, Eigen::Vector7d q_now) {
		logfile << play_time_ << "\t"
				<< (q_target(0) - q_now(0))*RAD2DEG << "\t"
				<< (q_target(1) - q_now(1))*RAD2DEG << "\t"
				<< (q_target(2) - q_now(2))*RAD2DEG << "\t"
				<< (q_target(3) - q_now(3))*RAD2DEG << "\t"
				<< (q_target(4) - q_now(4))*RAD2DEG << "\t"
				<< (q_target(5) - q_now(5))*RAD2DEG << "\t"
				<< (q_target(6) - q_now(6))*RAD2DEG << "\t"
				//<< q_target(0)*RAD2DEG << "\t" << q_now(0)*RAD2DEG << "\t"
				//<< q_target(1)*RAD2DEG << "\t" << q_now(1)*RAD2DEG << "\t"
				//<< q_target(2)*RAD2DEG << "\t" << q_now(2)*RAD2DEG << "\t"
				//<< q_target(3)*RAD2DEG << "\t" << q_now(3)*RAD2DEG << "\t"
				//<< q_target(4)*RAD2DEG << "\t" << q_now(4)*RAD2DEG << "\t"
				//<< q_target(5)*RAD2DEG << "\t" << q_now(5)*RAD2DEG << "\t"
				//<< q_target(6)*RAD2DEG << "\t" << q_now(6)*RAD2DEG << "\t"
				<< endl;
}

// --------------------------------------------------------------------
// hz_				: int			-> control frequency (= 100 hz)
// q_				: Vector7d		-> joint position
// qdot_			: Vector7d		-> joint velocity
// x_				: Vector3d		-> end-effector position 
// j_				: Matrix<6, 7>	-> basic jacobian (wrt. q_)
// j_from_q_desired_: Matrix<6, 7>	-> basic jacobian (wrt. q_desired)
// m_				: Matrix7d		-> mass matrix
// x_dot			: Vector6d		-> (v, w)
// --------------------------------------------------------------------

void ArmController::calcKinematics(Vector7d q, Vector7d qdot)
{
}

void ArmController::compute()
{
	/////* -----		           (pre-defined) Special Joint/Task State                     ----- */////
	//double settling_time = 5.0;
#ifdef FINAL_PROJECT
	double dX(0.08), dY(0.18), dZ(0.05);
	Vector7d joint_initial_position;	joint_initial_position << 0.0, 30.0*DEG2RAD, 0.0, (-120.0)*DEG2RAD, 0.0, (150.0)*DEG2RAD, 0.0;
	Vector3d center;					center	   << 0.58, 0.0, 0.15;
	Matrix3d rotation_target;           rotation_target << 1, 0, 0,
														   0, -1, 0,
														   0, 0, -1;

	Vector3d target_p1;					target_p1 << center[0] + dX, center[1] - dY, center[2];
	Vector3d target_p2;					target_p2 << center[0] - dX, center[1] + dY, center[2];
	Vector3d target_p3;					target_p3 << center[0] + dX, center[1] + dY, center[2];
	Vector3d target_p4;					target_p4 << center[0] - dX, center[1] - dY, center[2];

	Vector3d init_wp1;					init_wp1 << center[0],    center[1],    center[2] + dZ;
	Vector3d init_wp2;					init_wp2 << target_p1[0], target_p1[1], target_p1[2] + dZ;


#endif


#ifdef HW2
	Vector7d joint_initial_position;	joint_initial_position << 0.0, 0.0, 0.0, -M_PI / 2, 0.0, M_PI / 2, 0.0;
	Vector3d position_target; position_target << 0.25, 0.28, 0.65;
	Matrix3d rotation_target; rotation_target << 0, -1, 0,
												 -1, 0, 0,
												 0, 0, -1;
#endif

#ifdef HW3
	Vector3d x1_target;		x1_target << 0.25, 0.28, 0.65;
	Vector3d x2_target;		x2_target << 0.0, -0.15, 0.6;
#endif





	/////* -----          ( q_ ) -> Kinematics & Dynamics Calculation -> EEF Pose, Jacobian          ----- */////
	q_temp_ = q_;
	qdot_temp_ = qdot_;
	RigidBodyDynamics::UpdateKinematicsCustom(*model_, &q_temp_, &qdot_temp_, NULL);
	x_ = CalcBodyToBaseCoordinates(*model_, q_, body_id_[DOF - 1], com_position_[DOF - 1], true);
	rotation_ = CalcBodyWorldOrientation(*model_, q_, body_id_[DOF - 1], true).transpose();
	Matrix3d body_to_ee_rotation;
	body_to_ee_rotation.setIdentity();
	body_to_ee_rotation(1, 1) = -1;
	body_to_ee_rotation(2, 2) = -1;
	rotation_ = rotation_ * body_to_ee_rotation; // To Match RBDL model and CoppeliaSim model
	CalcPointJacobian6D(*model_, q_, body_id_[DOF - 1], com_position_[DOF - 1], j_temp_, true);
	NonlinearEffects(*model_, q_, Vector7d::Zero(), g_temp_);
	CompositeRigidBodyAlgorithm(*model_, q_, m_temp_, true);
	g_ = g_temp_;
	m_ = m_temp_;
	m_inverse_ = m_.inverse();
	for (int i = 0; i<2; i++)	j_.block<3, DOF>(i * 3, 0) = j_temp_.block<3, DOF>(3 - i * 3, 0);
	j_v_ = j_.block < 3, DOF>(0, 0);
	j_inverse_ = j_.transpose() * ((j_ * j_.transpose()).inverse());

	x_ = x_;
	x_dot_ = j_ * qdot_;
	j_ = j_;

	/////* -----      ( q_desired_ ) -> Kinematics & Dynamics Calculation -> EEF Pose, Jacobian      ----- */////
	j_temp_from_q_desired_.resize(6, DOF);
	j_temp_from_q_desired_.setZero();
	x_from_q_desired_ = CalcBodyToBaseCoordinates(*model_, q_desired_, body_id_[DOF - 1], com_position_[DOF - 1], false);
	rotation_from_q_desired_ = CalcBodyWorldOrientation(*model_, q_desired_, body_id_[DOF - 1], false).transpose();
	rotation_from_q_desired_ = rotation_from_q_desired_ * body_to_ee_rotation; // To Match RBDL model and CoppeliaSim model
	CalcPointJacobian6D(*model_, q_desired_, body_id_[DOF - 1], com_position_[DOF - 1], j_temp_from_q_desired_, false);
	for (int i = 0; i < 2; i++)		j_from_q_desired_.block<3, DOF>(i * 3, 0) = j_temp_from_q_desired_.block<3, DOF>(3 - i * 3, 0);
	j_inverse_from_q_desired_ = j_from_q_desired_.transpose() * ((j_from_q_desired_ * j_from_q_desired_.transpose()).inverse());

	x1_ = x_from_q_desired_;
	x1_dot_ = j_from_q_desired_ * qdot_;
	j1_ = j_from_q_desired_;

	/////* -----      ( q_desired_ ) -> 4th Joint's Kinematics & Dynamics     ----- */////
	x2_from_q_desired_ = CalcBodyToBaseCoordinates(*model_, q_desired_, body_id_[DOF - 4], com_position_[DOF - 4], false);
	j2_temp_from_q_desired_.resize(6, DOF);
	j2_temp_from_q_desired_.setZero();
	CalcPointJacobian6D(*model_, q_desired_, body_id_[DOF - 4], com_position_[DOF - 4], j2_temp_from_q_desired_, false);
	for (int i = 0; i < 2; i++)		j2_from_q_desired_.block<3, DOF>(i * 3, 0) = j2_temp_from_q_desired_.block<3, DOF>(3 - i * 3, 0);
	j2_v_from_q_desired_ = j2_from_q_desired_.block < 3, DOF>(0, 0);

	x2_ = x2_from_q_desired_;
	x2_dot_ = j2_from_q_desired_ * qdot_;
	j2_ = j2_from_q_desired_;


	if (is_mode_changed_)
	{
		logfile.close();
		logfile.open(control_mode_ + ".txt");
		is_mode_changed_ = false;

		control_start_time_ = play_time_;

		q_init_ = q_;
		qdot_init_ = qdot_;
		q_error_sum_.setZero();

		x_init_ = x_;

		x1_init_ = x1_;
		x2_init_ = x2_;

		x_dot_init_ = x_dot_.block<3, 1>(0, 0); // 추가 (윤원재)
		x_cubic_old_ = x_;
		rotation_init_ = rotation_;

	}

	if      (control_mode_ == "joint_ctrl_home")
	{
		Vector7d q_target;	q_target << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, M_PI / 4;
#ifdef TORQUE_CONTROL_MODE
		moveJointPositionbyTorque(q_target, settling_time_);
#endif
#ifndef TORQUE_CONTROL_MODE
		moveJointPosition(q_target, settling_time_);
#endif
		
	}
	else if (control_mode_ == "joint_ctrl_init")
	{
		Vector7d q_target;	q_target << 0.0, 0.0, 0.0, -M_PI / 2., 0.0, 0.0, M_PI / 4;
#ifdef TORQUE_CONTROL_MODE
		moveJointPositionbyTorque(q_target, settling_time_);
#endif
#ifndef TORQUE_CONTROL_MODE
		moveJointPosition(q_target, settling_time_);
#endif
	}
	else if (control_mode_ == "torque_ctrl_dynamic")
	{
		Vector7d q_target;	q_target << 0.0, 0.0, 0.0, -M_PI / 2., 0.0, 0.0, M_PI / 4;
		moveJointPositionbyTorque(q_target, settling_time_);
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////
	////* -----                       HOMEWORK   2020-26181   윤원재                      ----- *////
	/////////////////////////////////////////////////////////////////////////////////////////////////
	else if (control_mode_ == "Change Parameters") 
	{
		int n;

		try {
			cout << "=======================================" << endl;
			cout << "1. Settling Time (t)" << endl;
			cout << "2. Control Gain (Kp)" << endl;
			cout << "3. Weight Matrix (W)" << endl;
			cout << "4. Torque Control P-Gain (Kp_joint)" << endl;
			cout << "5. Torque Control D-Gain (Kv_joint)" << endl;
			cout << "Enter number to change parameters: ";
			if (!(cin >> n))	throw 0;
			cout << "=======================================" << endl;

			// 1.settling_time_
			if (n == 1) {		
				cout << "Enter settling time: ";
				if (!(cin >> settling_time_))	throw 1;
				cout << "Success: Parameters have been changed: " << "settling_time_ = " << settling_time_ << endl;
			}
			// 2. Kp_
			else if (n == 2) {	
				Vector6d Kp_vector; Kp_vector << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
				for (int i = 0; i < 6; i++) {
					cout << "Enter Kp(" << i << "): ";
					if (!(cin >> Kp_vector(i)))	throw 2;
				}
				Kp_ = Kp_vector.asDiagonal();
				cout << "Success: Parameters have been changed: " << "Kp = " << endl << Kp_ << endl;
			}
			// 3. W_
			else if (n == 3) {
				Vector7d W_vector;	W_vector << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
				for (int i = 0; i < 7; i++) {
					cout << "Enter W(" << i << "): ";
					if (!(cin >> W_vector(i)))	throw 2;
				}
				W_ = W_vector.asDiagonal();
				cout << "Success: Parameters have been changed: " << "W = " << endl << W_ << endl;
			}
			// 4. Kp_joint_
			else if (n == 4) {
				
				double Kp;
				cout << "Enter Kp_joint: ";
				if (!(cin >> Kp))	throw 2;
				Kp_joint_ = Kp * EYE(7);

				//Vector7d Kp_joint_vector;	Kp_joint_vector << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
				//for (int i = 0; i < DOF; i++) {
				//	cout << "Enter Kp_joint(" << i << "): ";
				//	if (!(cin >> Kp_joint_vector(i)))	throw 2;
				//}
				//Kp_joint_ = Kp_joint_vector.asDiagonal();
				
				cout << "Success: Parameters have been changed: " << "Kp_joint = " << endl << Kp_joint_ << endl;
			}
			// 5. Kv_joint_
			else if (n == 5) {
				
				double Kv;
				cout << "Enter Kv_joint: ";
				if (!(cin >> Kv))	throw 2;
				Kv_joint_ = Kv * EYE(7);
				//Vector7d Kv_joint_vector;	Kv_joint_vector << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
				//for (int i = 0; i < DOF; i++) {
				//	cout << "Enter Kv_joint(" << i << "): ";
				//	if (!(cin >> Kv_joint_vector(i)))	throw 2;
				//}
				//Kv_joint_ = Kv_joint_vector.asDiagonal();
				
				cout << "Success: Parameters have been changed: " << "Kv_joint = " << endl << Kv_joint_ << endl;
			}
			cout << "=======================================" << endl;
		}
		catch (int e) {
			cout << "Error: Parameters is not changed !!! (" << e << ")"<< endl;
			cout << "=======================================" << endl;

			cin.clear();
			cin.ignore(numeric_limits<streamsize>::max(), '\n');
		}
		setMode("Lock Joints");
	}
	
#ifdef FINAL_PROJECT
	else if (control_mode_ == "center") {
		moveJointPosition(joint_initial_position, settling_time_);
		if ((joint_initial_position - q_).norm() < ERROR_TOLERANCE)	setMode("Lock Joints");
	}
	else if (control_mode_ == "init") {
		setMode("init_1");
	}
	else if (control_mode_ == "init_1") {
		init_wp1 << x_init_[0], x_init_[1], x_init_[2] + dZ;
		moveTaskPositionCLIK(x_from_q_desired_, init_wp1,
							 rotation_from_q_desired_, rotation_target,
							 j_inverse_from_q_desired_, settling_time_);

		isMotionCompleted2(init_wp1, rotation_target, ERROR_TOLERANCE, "init_2");
	}
	else if (control_mode_ == "init_2") {
		moveTaskPositionCLIK(x_from_q_desired_, init_wp2,
							 rotation_from_q_desired_, rotation_target,
							 j_inverse_from_q_desired_, settling_time_);

		isMotionCompleted2(init_wp2, rotation_target, ERROR_TOLERANCE, "init_3");
	}
	else if (control_mode_ == "init_3") {
		moveTaskPositionCLIK(x_from_q_desired_, target_p1,
							 rotation_from_q_desired_, rotation_target,
							 j_inverse_from_q_desired_, settling_time_);

		isMotionCompleted2(target_p1, rotation_target, ERROR_TOLERANCE, "Lock Joint");
	}

	
	else if (control_mode_ == "target1") {
		moveTaskPositionCLIK(x_from_q_desired_, target_p1,
							 rotation_from_q_desired_, rotation_target,
							 j_inverse_from_q_desired_, settling_time_);
	}
	else if (control_mode_ == "target2") {
		moveTaskPositionCLIK(x_from_q_desired_, target_p2,
							 rotation_from_q_desired_, rotation_target,
							 j_inverse_from_q_desired_, settling_time_);
	}
	else if (control_mode_ == "target3") {
		moveTaskPositionCLIK(x_from_q_desired_, target_p3,
			rotation_from_q_desired_, rotation_target,
			j_inverse_from_q_desired_, settling_time_);
	}
	else if (control_mode_ == "target4") {
		moveTaskPositionCLIK(x_from_q_desired_, target_p4,
			rotation_from_q_desired_, rotation_target,
			j_inverse_from_q_desired_, settling_time_);
	}
#endif

#ifdef HW2 //HW2
	// --- HW2-0. Joint Space Control --- //
	else if (control_mode_ == "HW2-0") {
		moveJointPosition(joint_initial_position, settling_time_);
		if ((joint_initial_position - q_).norm() < ERROR_TOLERANCE)	setMode("Lock Joints");
	}

	// --- HW2-1(a). q_desired = q + dq  (dq = J_inverse * x_dot * dt , x_dot -> from trajectory) --- //
	else if (control_mode_ == "test_from_initial_joint_position") {
		moveJointPosition(joint_initial_position, settling_time_);
		if ((joint_initial_position - q_).norm() < ERROR_TOLERANCE)	setMode("test");
	}
	else if (control_mode_ == "test") {
		for (int i = 0; i < 3; i++) { // Translation Dot
			x_dot_cubic_(i) = cubicDot(play_time_,
									   control_start_time_,
									   control_start_time_ + settling_time_,
									   x_init_(i), position_target(i),
									   0, 0, hz_);
			dx_(i) = x_cubic_(i) - x_(i);
		}
		rotation_cubic_ = rotationCubic(play_time_,
										control_start_time_,
										control_start_time_ + settling_time_,
										rotation_init_, rotation_target);
		x_dot_cubic_.block<3, 1>(3, 0) = -getPhi(rotation_, rotation_cubic_) * hz_; // Rotation Dot (엄밀히 따지면 trajectory에서의 속도가 아님)
		phi_ = -getPhi(rotation_, rotation_cubic_);

		for (int i = 0; i < 3; i++) {
			x_error_(i) = dx_(i);
			x_error_(i + 3) = phi_(i);
		}

		q_desired_ = q_ + j_inverse_from_q_desired_ * x_dot_cubic_ / hz_;

		isMotionCompleted(position_target, rotation_target, ERROR_TOLERANCE);
		logData(x_error_);
	}

	// --- HW2-1(b). q_desired = q + dq  (dq = J_inverse * dx) --- //
	else if (control_mode_ == "HW2-1_from_initial_joint_position") {
		moveJointPosition(joint_initial_position, settling_time_);
		if ((joint_initial_position - q_).norm() < ERROR_TOLERANCE)	setMode("HW2-1");
	}
	else if (control_mode_ == "HW2-1")
	{
		moveTaskPosition(x_, position_target, rotation_, rotation_target, j_inverse_, settling_time_); // j_ 사용

		isMotionCompleted(position_target, rotation_target, ERROR_TOLERANCE);
		logData(x_error_);
	}

	// --- HW2-2. CLIK, without Weight Matrix ( W=diag(1, 1, 1, 1, 1, 1, 1) ) --- //
	else if (control_mode_ == "HW2-2_from_initial_joint_position") {
		moveJointPosition(joint_initial_position, settling_time_);
		if ((joint_initial_position - q_).norm() < ERROR_TOLERANCE)	setMode("HW2-2");
	}
	else if (control_mode_ == "HW2-2")
	{
		moveTaskPositionCLIK(x_from_q_desired_, position_target,
							 rotation_from_q_desired_, rotation_target,
							 j_inverse_from_q_desired_, settling_time_);

		isMotionCompleted(position_target, rotation_target, ERROR_TOLERANCE);
		logData(x_error_);
	}

	// --- HW2-3. CLIK with Weight Matrix ( W=diag(1, 1, 1, 0.01, 1, 1, 1) ) --- //
	else if (control_mode_ == "HW2-3_from_initial_joint_position") {
		moveJointPosition(joint_initial_position, settling_time_);
		if ((joint_initial_position - q_).norm() < ERROR_TOLERANCE)	setMode("HW2-3");
	}
	else if (control_mode_ == "HW2-3")
	{
		moveTaskPositionCLIK(x_from_q_desired_, position_target,
							 rotation_from_q_desired_, rotation_target,
							 j_inverse_from_q_desired_, settling_time_);

		isMotionCompleted(position_target, rotation_target, ERROR_TOLERANCE);
		logData(x_error_);
		//logfile << play_time_ << "\t" << q_(3) << endl;
	}
#endif

#ifdef HW3 //HW3
	// --- HW3-1. Multi task: Simultaneously --- //
	else if (control_mode_ == "HW3-1_from_initial_joint_position") {
	moveJointPosition(joint_initial_position, settling_time_);
	if ((joint_initial_position - q_).norm() < ERROR_TOLERANCE)	setMode("HW3-1");
	}
	else if (control_mode_ == "HW3-1"){
	j1_v_ = j1_.block<3, 7>(0, 0);
	j2_v_ = j2_.block<3, 7>(0, 0);

	j_tot_transpose_ << j1_v_.transpose(), j2_v_.transpose();
	j_tot_ = j_tot_transpose_.transpose();
	j_tot_inverse_ = j_tot_.transpose()*(j_tot_*j_tot_.transpose() + 0.01*EYE(6)).inverse();

	for (int i = 0; i < 3; i++){
		x1_cubic_(i)	 = cubic	(play_time_, control_start_time_, control_start_time_ + settling_time_, x1_init_(i), x1_target(i), 0.0, 0.0);
		x1_dot_cubic_(i) = cubicDot	(play_time_, control_start_time_, control_start_time_ + settling_time_, x1_init_(i), x1_target(i), 0.0, 0.0, hz_);
		x2_cubic_(i)	 = cubic	(play_time_, control_start_time_, control_start_time_ + settling_time_, x2_init_(i), x2_target(i), 0.0, 0.0);
		x2_dot_cubic_(i) = cubicDot	(play_time_, control_start_time_, control_start_time_ + settling_time_, x2_init_(i), x2_target(i), 0.0, 0.0, hz_);
	}
	x_cubic_tot_ << x1_cubic_, x2_cubic_;
	x_tot_ << x1_, x2_;
	x_cubic_dot_tot_ << x1_dot_cubic_.block<3, 1>(0, 0), x2_dot_cubic_.block<3, 1>(0, 0);
	x_dot_tot_ << x_dot_.block<3, 1>(0, 0), x2_dot_.block <3, 1>(0, 0);

	qdot_desired_ = j_tot_inverse_ * (x_cubic_dot_tot_ + Kp_ * (x_cubic_tot_ - x_tot_));
	q_desired_ = q_ + qdot_desired_ / hz_;

	isMotionCompleted(x1_target, rotation_, ERROR_TOLERANCE);
	logData_TaskTransition(x1_, x2_);
	}

	// --- HW3-2. Multi task: task2 in Nullspace of task1 --- //
	else if (control_mode_ == "HW3-2_from_initial_joint_position") {
		moveJointPosition(joint_initial_position, settling_time_);
		if ((joint_initial_position - q_).norm() < ERROR_TOLERANCE)	setMode("HW3-2");
	}
	else if (control_mode_ == "HW3-2"){
	j1_v_ = j1_.block<3, 7>(0, 0);
	j2_v_ = j2_.block<3, 7>(0, 0);

	j1_v_inverse_ = j1_v_.transpose() * ((j1_v_ * j1_v_.transpose()).inverse());
	j2_v_inverse_ = j2_v_.transpose() * ((j2_v_ * j2_v_.transpose()).inverse());

	N1_ = (EYE(7) - j1_v_inverse_ * j1_v_);

	for (int i = 0; i < 3; i++){
		x1_cubic_(i)	 = cubic	(play_time_, control_start_time_, control_start_time_ + settling_time_, x1_init_(i), x1_target(i), 0.0, 0.0);
		x1_dot_cubic_(i) = cubicDot (play_time_, control_start_time_, control_start_time_ + settling_time_, x1_init_(i), x1_target(i), 0.0, 0.0, hz_);
		x2_cubic_(i)	 = cubic	(play_time_, control_start_time_, control_start_time_ + settling_time_, x2_init_(i), x2_target(i), 0.0, 0.0);
		x2_dot_cubic_(i) = cubicDot	(play_time_, control_start_time_, control_start_time_ + settling_time_, x2_init_(i), x2_target(i), 0.0, 0.0, hz_);
	}

	x1_dot_CLIK_ = x1_dot_cubic_.block<3, 1>(0, 0) + Kp_.block<3, 3>(0, 0) * (x1_cubic_ - x1_);
	x2_dot_CLIK_ = x2_dot_cubic_.block<3, 1>(0, 0) + Kp_.block<3, 3>(3, 3) * (x2_cubic_ - x2_);

	Vector7d q2dot_desired;		q2dot_desired = j2_v_inverse_ * (x2_dot_CLIK_ - j2_v_ * j1_v_inverse_  * x1_dot_CLIK_);

	qdot_desired_ = j1_v_inverse_ * x1_dot_CLIK_  +  N1_ * q2dot_desired;
	q_desired_ = q_ + qdot_desired_ / hz_;
	//q_desired_ = q_ + (j1_v_inverse_ * x1_dot_CLIK_ + N1_ * (j2_v_inverse_ * (x2_dot_CLIK_ - j2_v_ * j1_v_inverse_  * x1_dot_CLIK_))) / hz_;

	isMotionCompleted(x1_target, rotation_, ERROR_TOLERANCE);
	logData_TaskTransition(x1_, x2_);
	}

	// --- HW3-3. Multi task: Task Transition  --- //
	else if (control_mode_ == "HW3-3_from_initial_joint_position") {
		moveJointPosition(joint_initial_position, settling_time_);
		if ((joint_initial_position - q_).norm() < ERROR_TOLERANCE)	setMode("HW3-3");
	}
	else if (control_mode_ == "HW3-3"){
	j1_v_ = j1_.block<3, 7>(0, 0);
	j2_v_ = j2_.block<3, 7>(0, 0);

	j1_v_inverse_ = j1_v_.transpose() * ((j1_v_ * j1_v_.transpose()).inverse());
	j2_v_inverse_ = j2_v_.transpose() * ((j2_v_ * j2_v_.transpose()).inverse());

	N1_ = (EYE(7) - j1_v_inverse_ * j1_v_);

	for (int i = 0; i < 3; i++)
	{
		x1_cubic_(i)	 = cubic	(play_time_, control_start_time_, control_start_time_ + settling_time_, x1_init_(i), x1_target(i), 0.0, 0.0);
		x1_dot_cubic_(i) = cubicDot	(play_time_, control_start_time_, control_start_time_ + settling_time_, x1_init_(i), x1_target(i), 0.0, 0.0, hz_);
		x2_cubic_(i)	 = cubic	(play_time_, control_start_time_, control_start_time_ + settling_time_, x2_init_(i), x2_target(i), 0.0, 0.0);
		x2_dot_cubic_(i) = cubicDot	(play_time_, control_start_time_, control_start_time_ + settling_time_, x2_init_(i), x2_target(i), 0.0, 0.0, hz_);
	}

	h1_ = 1.0;
	h2_ = (play_time_ - control_start_time_) / settling_time_;
	//h2_ = cubic(play_time_, control_start_time_, control_start_time_ + settling_time, 0.0, 1.0, 0.0, 0.0);
	//h2_ = 0.0;

	x1_dot_CLIK_ = x1_dot_cubic_.block<3, 1>(0, 0) + Kp_.block<3, 3>(0, 0) * (x1_cubic_ - x1_);
	x2_dot_CLIK_ = x2_dot_cubic_.block<3, 1>(0, 0) + Kp_.block<3, 3>(3, 3) * (x2_cubic_ - x2_);

	x1_dot_input_ = h1_ * x1_dot_CLIK_ + (1 - h1_)*j1_v_*j2_v_inverse_*h2_*x2_dot_CLIK_;
	x2_dot_input_ = h2_ * x2_dot_CLIK_ + (1 - h2_)*j2_v_*j1_v_inverse_*h1_*x1_dot_CLIK_;

	qdot_desired_ = j1_v_inverse_ * x1_dot_input_ + N1_ * j2_v_inverse_ * (x2_dot_input_ - j2_v_*j1_v_inverse_*x1_dot_input_);
	q_desired_ = q_ + qdot_desired_ / hz_;

	isMotionCompleted(x1_target, rotation_, ERROR_TOLERANCE);
	logData_TaskTransition(x1_, x2_);
	}
#endif

#ifdef HW4 //HW4
	else if (control_mode_ == "HW4_init") {
		Vector7d q_target;	q_target << 0.0, 0.0, 0.0, -30.0*DEG2RAD, 0.0, 90.0*DEG2RAD, 0.0;
		double settling_time;	settling_time = 0.5;

		Matrix7d Kp_joint;		Kp_joint = 400.0 * EYE(7);
		Matrix7d Kv_joint;		Kv_joint =  40.0 * EYE(7);

		for (int i = 0; i < DOF; i++){
			q_cubic_(i) = cubic(play_time_, control_start_time_, control_start_time_ + settling_time, q_init_(i), q_target(i), qdot_init_(i), qdot_target_(i));
			qdot_cubic_(i) = cubicDot(play_time_, control_start_time_, control_start_time_ + settling_time, q_init_(i), q_target(i), qdot_init_(i), qdot_target_(i), hz_);
		}
		torque_desired_ = m_ * (Kp_joint*(q_cubic_ - q_) + Kv_joint * (qdot_cubic_ - qdot_)) + g_;

		if ((q_target - q_).norm()*RAD2DEG < 1.0)	setMode("Lock Joints");
	}

	else if (control_mode_ == "HW4-1_from_initial_joint_position") {
		Vector7d q_target;	q_target << 0.0, 0.0, 0.0, -30.0*DEG2RAD, 0.0, 90.0*DEG2RAD, 0.0;
		for (int i = 0; i<DOF; i++){
			Kp_joint_(i, i) = 400;
			Kv_joint_(i, i) = 40;
		}
		moveJointPositionbyTorque(q_target, 0.5);
		if ((q_target - q_).norm()*RAD2DEG < 1.0)	setMode("HW4-1");
	}
	else if (control_mode_ == "HW4-1") { // Simple PD Controller (step input)
		Vector7d q_target;		q_target << 0.0, 0.0, 0.0, -25.0*DEG2RAD, 0.0, 90.0*DEG2RAD, 0.0;
		qdot_target_.setZero();

		torque_desired_ = Kp_joint_*(q_target - q_) + Kv_joint_*(qdot_target_ - qdot_);

		logData_JointError(q_target, q_);
		if ((q_target - q_).norm()*RAD2DEG < 1.0 || play_time_ - control_start_time_ > settling_time_)	setMode("Lock Joints");
	}

	else if (control_mode_ == "HW4-2(a)_from_initial_joint_position") {
		Vector7d q_target;	q_target << 0.0, 0.0, 0.0, -30.0*DEG2RAD, 0.0, 90.0*DEG2RAD, 0.0;
		for (int i = 0; i<DOF; i++)	{
			Kp_joint_(i, i) = 400;
			Kv_joint_(i, i) = 40;
		}
		moveJointPositionbyTorque(q_target, 0.5);
		if ((q_target - q_).norm()*RAD2DEG < 1.0)	setMode("HW4-2(a)");
	}
	else if (control_mode_ == "HW4-2(a)"){ // Simple PD Controller with gravity compensation (step input)
		Vector7d q_target;	q_target << 0.0, 0.0, 0.0, -25.0*DEG2RAD, 0.0, 90.0*DEG2RAD, 0.0;
		qdot_target_.setZero();

		for (int i = 0; i<DOF; i++)	{
			Kp_joint_(i, i) = 180;
			Kv_joint_(i, i) = 0.8;
		}
		torque_desired_ = Kp_joint_*(q_target - q_) + Kv_joint_*(qdot_target_ - qdot_) + g_;

		logData_JointError(q_target, q_);
		if((q_target - q_).norm()*RAD2DEG < 1.0 || play_time_ - control_start_time_ > settling_time_)	setMode("Lock Joints");
	}

	else if (control_mode_ == "HW4-2(b)_from_initial_joint_position") {
		Vector7d q_target;	q_target << 0.0, 0.0, 0.0, -30.0*DEG2RAD, 0.0, 90.0*DEG2RAD, 0.0;
		for (int i = 0; i<DOF; i++)	{
			Kp_joint_(i, i) = 400;
			Kv_joint_(i, i) = 40;
		}
		moveJointPositionbyTorque(q_target, 0.5);
		if ((q_target - q_).norm()*RAD2DEG < 1.0)	setMode("HW4-2(b)");
	}
	else if (control_mode_ == "HW4-2(b)"){ // Simple PD Controller with gravity compensation (cubic spline input)
		Vector7d q_target;	q_target << 0.0, 0.0, 0.0, -60.0*DEG2RAD, 0.0, 90.0*DEG2RAD, 0.0;
		qdot_target_.setZero();
		for (int i = 0; i < DOF; i++){
			q_cubic_(i)	   = cubic	  (play_time_, control_start_time_, control_start_time_ + settling_time_, q_init_(i), q_target(i), qdot_init_(i), qdot_target_(i));
			qdot_cubic_(i) = cubicDot (play_time_, control_start_time_, control_start_time_ + settling_time_, q_init_(i), q_target(i), qdot_init_(i), qdot_target_(i), hz_);
		}
		for (int i = 0; i<DOF; i++){
			Kp_joint_(i, i) = 180;
			Kv_joint_(i, i) = 0.8;
		}
		torque_desired_ = Kp_joint_ * (q_cubic_ - q_) + Kv_joint_ * (qdot_cubic_ - qdot_) + g_;

		logData_JointError(q_target, q_);
		if ((q_target - q_).norm()*RAD2DEG < 1.0 || play_time_ - control_start_time_ > settling_time_)	setMode("Lock Joints");
	}

	else if (control_mode_ == "HW4-3(a)_from_initial_joint_position") {
		Vector7d q_target;	q_target << 0.0, 0.0, 0.0, -30.0*DEG2RAD, 0.0, 90.0*DEG2RAD, 0.0;
		for (int i = 0; i<DOF; i++)	{
			Kp_joint_(i, i) = 400;
			Kv_joint_(i, i) = 40;
		}
		moveJointPositionbyTorque(q_target, 0.5);
		if ((q_target - q_).norm()*RAD2DEG < 1.0)	setMode("HW4-3(a)");
	}
	else if (control_mode_ == "HW4-3(a)"){ // PD controller with dynamic compensation (step input)
		Vector7d q_target;	q_target << 0.0, 0.0, 0.0, -25.0*DEG2RAD, 0.0, 90.0*DEG2RAD, 0.0;
		qdot_target_.setZero();
		for (int i = 0; i<DOF; i++)	{
			Kp_joint_(i, i) = 400;
			Kv_joint_(i, i) = 40;
		}
		torque_desired_ = m_*(Kp_joint_*(q_target - q_) + Kv_joint_*(qdot_target_ - qdot_)) + g_;

		logData_JointError(q_target, q_);
		if ((q_target - q_).norm()*RAD2DEG < 1.0 || play_time_ - control_start_time_ > settling_time_)	setMode("Lock Joints");
	}

	else if (control_mode_ == "HW4-3(b)_from_initial_joint_position") {
		Vector7d q_target;	q_target << 0.0, 0.0, 0.0, -30.0*DEG2RAD, 0.0, 90.0*DEG2RAD, 0.0;
		for (int i = 0; i<DOF; i++)	{
			Kp_joint_(i, i) = 400;
			Kv_joint_(i, i) = 40;
		}
		moveJointPositionbyTorque(q_target, 0.5);
		if ((q_target - q_).norm()*RAD2DEG < 1.0)	setMode("HW4-3(b)");
	}
	else if (control_mode_ == "HW4-3(b)"){ // PD controller with dynamic compensation (cubic spline input) (W/ wn=20rad/s, critically damped)
		Vector7d q_target;	q_target << 0.0, 0.0, 0.0, -60.0*DEG2RAD, 0.0, 90.0*DEG2RAD, 0.0;

		//Kp_joint_ = 400.0 * EYE(7);
		//Kv_joint_ =  40.0 * EYE(7);

		moveJointPositionbyTorque(q_target, settling_time_); // 함수에서 400, 40으로 세팅하게 바꿈

		logData_JointError(q_target, q_);
		if ((q_target - q_).norm()*RAD2DEG < 1.0 || play_time_ - control_start_time_ > settling_time_)	setMode("Lock Joints");
	}
#endif

	else // --------------------------------------------------------------------------------------------------------------------------------------------
	{
		torque_desired_ = g_;
	}

	printState();

	tick_++;
	play_time_ = tick_ / hz_;	// second
}

void ArmController::isMotionCompleted(Eigen::Vector3d position_target, Eigen::Matrix3d rotation_target, double tolerance) {
	x_error_to_target_.block<3, 1>(0, 0) = position_target - x_;
	x_error_to_target_.block<3, 1>(3, 0) = getPhi(rotation_, rotation_target);
	if ((x_error_to_target_).norm() < tolerance)	setMode("Lock Joints");
}

bool ArmController::isMotionCompleted2(Eigen::Vector3d position_target, Eigen::Matrix3d rotation_target, double tolerance, string state) {
	x_error_to_target_.block<3, 1>(0, 0) = position_target - x_;
	x_error_to_target_.block<3, 1>(3, 0) = getPhi(rotation_, rotation_target);
	if ((x_error_to_target_).norm() < tolerance) {
		setMode(state);
		return 1;
	}
	return 0;
}

void ArmController::moveTaskPositionCLIK(const Vector3d &position_now, const Vector3d &position_target,
										 const Matrix3d &rotation_now, const Matrix3d &rotation_target,
										 const Matrix<double, 7, 6> &jacobian_inverse,
										 double settling_time) {
	for (int i = 0; i < 3; i++)
	{
		x_cubic_(i) = cubic(play_time_,
							control_start_time_,
							control_start_time_ + settling_time,
							x_init_(i),
							position_target(i),
							0, 0);
		dx_(i) = x_cubic_(i) - x_from_q_desired_(i);

		x_dot_cubic_(i) = cubicDot(play_time_,
								   control_start_time_,
								   control_start_time_ + settling_time,
								   x_init_(i), x_target_(i),
								   x_dot_init_(i), x_dot_target_(i),
								   hz_);
	}

	rotation_cubic_ = rotationCubic(play_time_,
									control_start_time_,
									control_start_time_ + settling_time,
									rotation_init_, rotation_target);
	phi_ = -getPhi(rotation_from_q_desired_, rotation_cubic_);

	for (int i = 0; i < 3; i++)
	{
		x_dot_cubic_(i + 3) = phi_(i) * hz_; // Rotation Dot (엄밀히 따지면 trajectory에서의 속도가 아님)

		x_error_(i) = dx_(i);
		x_error_(i + 3) = phi_(i);
	}

	j_inverse_from_q_desired_ = W_.inverse() * j_from_q_desired_.transpose() * ((j_from_q_desired_ * W_.inverse() * j_from_q_desired_.transpose()).inverse());
	q_desired_ = q_ + j_inverse_from_q_desired_ * (x_dot_cubic_ + Kp_ * x_error_) / hz_;

	isMotionCompleted(position_target, rotation_target, ERROR_TOLERANCE);

	//logfile << play_time_ << "\t" << q_(3) << endl;
	logData(x_error_);
}

void ArmController::moveTaskPosition(const Vector3d &position_now, const Vector3d &position_target,
									 const Matrix3d &rotation_now, const Matrix3d &rotation_target,
									 const Matrix<double, 7, 6> &jacobian_inverse, double settling_time) {
	for (int i = 0; i < 3; i++) {
		x_cubic_(i) = cubic(play_time_,
							control_start_time_,
							control_start_time_ + settling_time,
							x_init_(i), position_target(i),
							0, 0);
		dx_(i) = x_cubic_(i) - position_now(i);
	}

	rotation_cubic_ = rotationCubic(play_time_,
									control_start_time_,
									control_start_time_ + settling_time,
									rotation_init_, rotation_target);
	phi_ = -getPhi(rotation_now, rotation_cubic_);


	for (int i = 0; i < 3; i++) {
		x_error_(i) = dx_(i);
		x_error_(i + 3) = phi_(i);
	}

	q_desired_ = q_ + jacobian_inverse * x_error_; // dX = (dx, dy, dz, dRx, dRy, dRz) -> error term으로 바로 제어
}

void ArmController::moveJointPosition(const Vector7d &q_target, double settling_time)
{
	q_desired_ = cubicVector<7>(play_time_,
								control_start_time_,
								control_start_time_ + settling_time,
								q_init_, q_target,
								qdot_init_, qdot_target_);
}

void ArmController::moveJointPositionbyTorque(const Vector7d &q_target, double settling_time)
{
	Kp_joint_ = 400.0 * EYE(7);
	Kv_joint_ =  40.0 * EYE(7);

	for (int i = 0; i < DOF; i++)
	{
		q_cubic_(i)	   = cubic	  (play_time_, control_start_time_, control_start_time_ + settling_time, q_init_(i), q_target(i), qdot_init_(i), qdot_target_(i));
		qdot_cubic_(i) = cubicDot (play_time_, control_start_time_, control_start_time_ + settling_time, q_init_(i), q_target(i), qdot_init_(i), qdot_target_(i), hz_);
	}
	torque_desired_ = m_ * (Kp_joint_*(q_cubic_ - q_) + Kv_joint_ * (qdot_cubic_ - qdot_)) + g_;
}


void ArmController::printState()
{
	// TODO: Modify this method to debug your code

	static int DBG_CNT = 0;
	if (DBG_CNT++ > hz_ / 2.)
	{
		DBG_CNT = 0;
		cout << endl;
		cout << "-------------------------------------------------------" << endl;
		cout << "--------------    < Parameter lists >    --------------" << endl;
		cout << "-------------------------------------------------------" << endl;
		cout << "1. settling_time_ : "	<< std::fixed << std::setprecision(3) << settling_time_ << endl;
		cout << "2. Kp_ : "				<< std::fixed << std::setprecision(3) << Kp_.diagonal().transpose() << endl;
		cout << "3. W_  : "				<< std::fixed << std::setprecision(3) << W_.diagonal().transpose() << endl;
		cout << "4. Kp_joint_  : "		<< std::fixed << std::setprecision(3) << Kp_joint_.diagonal().transpose() << endl;
		cout << "5. Kv_joint_  : "		<< std::fixed << std::setprecision(3) << Kv_joint_.diagonal().transpose() << endl;
		cout << "-------------------------------------------------------" << endl;
		cout << "play_time_	: "			<< std::fixed << std::setprecision(3) << play_time_ << endl;
		cout << "control_mode_	: "		<< std::fixed << control_mode_ << endl;
		cout << "x_		: "				<< std::fixed << std::setprecision(3) << x_.transpose() << endl;
		cout << "x1_		: "			<< std::fixed << std::setprecision(3) << x1_.transpose() << endl;
		cout << "x2_		: "			<< std::fixed << std::setprecision(3) << x2_.transpose() << endl;
		cout << "h2_		: "			<< std::fixed << std::setprecision(3) << h2_ << endl;
		cout << "r_		: "				<< endl << std::fixed << std::setprecision(3) << rotation_.transpose() << endl;
		cout << "q_		: "				<< std::fixed << std::setprecision(3) << RAD2DEG*q_.transpose() << endl;
		//cout << "q_desired_	: "		<< std::fixed << std::setprecision(3) << q_desired_.transpose() << endl;
		//cout << "deltq q	: "			<< std::fixed << std::setprecision(3) << (q_desired_ - q_).transpose() << endl;
		cout << "torque_		: "		<< std::fixed << std::setprecision(3) << torque_.transpose() << endl;
		//cout << "torque_desired_ : "	<< std::fixed << std::setprecision(3) << torque_desired_.transpose() << endl;
		//cout << "j_		: "			<< endl << std::fixed << std::setprecision(3) << j_ << endl;
		//cout << "j_inverse_		: " << endl << std::fixed << std::setprecision(3) << j_inverse_ << endl;

		cout << endl;
	}
}



// Controller Core Methods ----------------------------

void ArmController::setMode(const std::string & mode)
{
	is_mode_changed_ = true;
	control_mode_ = mode;
	cout << "Current mode (changed) : " << mode << endl;
}
void ArmController::initDimension()
{
	dof_ = DOF;
	q_temp_.resize(DOF);
	j_temp_.resize(6, DOF);

	qdot_target_.setZero();
	qddot_.setZero();
	x_target_.setZero();

	q_desired_.setZero();
	qdot_desired_.setZero();
	torque_desired_.setZero();

	g_temp_.resize(DOF);
	m_temp_.resize(DOF, DOF);

	/*---  Default Control Parameters (윤원재) ---*/
	settling_time_ = 2.0;

	Vector6d Kp_vec;	Kp_vec << 120.0, 120.0, 120.0, 60.0, 60.0, 60.0;
	Kp_ = Kp_vec.asDiagonal();

	Vector7d W;		W << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;		
	W_ = W.asDiagonal();

	Kp_joint_ = 400.0 * EYE(7);
	Kv_joint_ =  40.0 * EYE(7);
}

void ArmController::initModel()
{
	model_ = make_shared<Model>();

	model_->gravity = Vector3d(0., 0, -GRAVITY);

	double mass[DOF];
	mass[0] = 1.0;
	mass[1] = 1.0;
	mass[2] = 1.0;
	mass[3] = 1.0;
	mass[4] = 1.0;
	mass[5] = 1.0;
	mass[6] = 1.0;

	Vector3d axis[DOF];
	axis[0] = Eigen::Vector3d::UnitZ();
	axis[1] = Eigen::Vector3d::UnitY();
	axis[2] = Eigen::Vector3d::UnitZ();
	axis[3] = -1.0*Eigen::Vector3d::UnitY();
	axis[4] = Eigen::Vector3d::UnitZ();
	axis[5] = -1.0*Eigen::Vector3d::UnitY();
	axis[6] = -1.0*Eigen::Vector3d::UnitZ();


	Eigen::Vector3d global_joint_position[DOF];

	global_joint_position[0] = Eigen::Vector3d(0.0, 0.0, 0.3330);
	global_joint_position[1] = global_joint_position[0];
	global_joint_position[2] = Eigen::Vector3d(0.0, 0.0, 0.6490);
	global_joint_position[3] = Eigen::Vector3d(0.0825, 0.0, 0.6490);
	global_joint_position[4] = Eigen::Vector3d(0.0, 0.0, 1.0330);
	global_joint_position[5] = Eigen::Vector3d(0.0, 0.0, 1.0330);
	global_joint_position[6] = Eigen::Vector3d(0.0880, 0.0, 1.0330);

	joint_posision_[0] = global_joint_position[0];
	for (int i = 1; i < DOF; i++)
		joint_posision_[i] = global_joint_position[i] - global_joint_position[i - 1];

	com_position_[0] = Vector3d(0.000096, -0.0346, 0.2575);
	com_position_[1] = Vector3d(0.0002, 0.0344, 0.4094);
	com_position_[2] = Vector3d(0.0334, 0.0266, 0.6076);
	com_position_[3] = Vector3d(0.0331, -0.0266, 0.6914);
	com_position_[4] = Vector3d(0.0013, 0.0423, 0.9243);
	com_position_[5] = Vector3d(0.0421, -0.0103, 1.0482);
#ifndef FINAL_PROJECT
	com_position_[6] = Vector3d(0.1, -0.0120, 0.9536);
#endif
#ifdef FINAL_PROJECT
	com_position_[6] = Vector3d(0.0871, 0.0, 0.9089);
#endif

	for (int i = 0; i < DOF; i++)
		com_position_[i] -= global_joint_position[i];

	Math::Vector3d inertia[DOF];
	for (int i = 0; i < DOF; i++)
		inertia[i] = Eigen::Vector3d::Identity() * 0.001;

	for (int i = 0; i < DOF; i++) {
		body_[i] = Body(mass[i], com_position_[i], inertia[i]);
		joint_[i] = Joint(JointTypeRevolute, axis[i]);
		if (i == 0)
			body_id_[i] = model_->AddBody(0, Math::Xtrans(joint_posision_[i]), joint_[i], body_[i]);
		else
			body_id_[i] = model_->AddBody(body_id_[i - 1], Math::Xtrans(joint_posision_[i]), joint_[i], body_[i]);
	}
}

void ArmController::readData(const Vector7d &position, const Vector7d &velocity, const Vector7d &torque)
{
	for (size_t i = 0; i < dof_; i++)
	{
		q_(i) = position(i);
		qdot_(i) = velocity(i);
		torque_(i) = torque(i);
	}
}
void ArmController::readData(const Vector7d &position, const Vector7d &velocity)
{
	for (size_t i = 0; i < dof_; i++)
	{
		q_(i) = position(i);
		qdot_(i) = velocity(i);
		torque_(i) = 0;
	}
}

const Vector7d & ArmController::getDesiredPosition()
{
	return q_desired_;
}

const Vector7d & ArmController::getDesiredTorque()
{
	return torque_desired_;
}



void ArmController::initPosition()
{
	q_init_ = q_;
	q_desired_ = q_init_;
}

bool ArmController::projectFinish()
{
	if (0.48 < x_(0) && x_(0) < 0.52 && -0.2 < x_(1) && x_(1) < -0.16)
		return true;
	else
		return false;
}


// ----------------------------------------------------

