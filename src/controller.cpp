#include "controller.h"
#include <iostream>
#include <iomanip>
#include <cmath>

#define ERROR_TOLERANCE 0.001 // [m]

using namespace DyrosMath;
ofstream logfile;

void ArmController::logData(Eigen::Vector6d x_error) {
	logfile << play_time_ << "\t"
			<< x_error(0) << "\t"
			<< x_error(1) << "\t"
			<< x_error(2) << "\t"
			<< x_error(3) << "\t"
			<< x_error(4) << "\t"
			<< x_error(5) << "\t"
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
	/////* -----		           (pre-defined) Special Joint/Task State (윤원재)                   ----- */////
	Vector7d joint_initial_position;	joint_initial_position << 0.0, 0.0, 0.0, -M_PI / 2, 0.0, M_PI / 2, 0.0;



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
	x_dot_ = j_ * qdot_;

	/////* -----      ( q_desired_ ) -> Kinematics & Dynamics Calculation -> EEF Pose, Jacobian      ----- */////
	j_temp_from_q_desired_.resize(6, DOF);
	j_temp_from_q_desired_.setZero();
	x_from_q_desired_ = CalcBodyToBaseCoordinates(*model_, q_desired_, body_id_[DOF - 1], com_position_[DOF - 1], false);
	rotation_from_q_desired_ = CalcBodyWorldOrientation(*model_, q_desired_, body_id_[DOF - 1], false).transpose();
	rotation_from_q_desired_ = rotation_from_q_desired_ * body_to_ee_rotation; // To Match RBDL model and CoppeliaSim model
	CalcPointJacobian6D(*model_, q_desired_, body_id_[DOF - 1], com_position_[DOF - 1], j_temp_from_q_desired_, false);
	for (int i = 0; i < 2; i++)		j_from_q_desired_.block<3, DOF>(i * 3, 0) = j_temp_from_q_desired_.block<3, DOF>(3 - i * 3, 0);
	j_from_q_desired_inverse_ = j_from_q_desired_.transpose() * ((j_from_q_desired_ * j_from_q_desired_.transpose()).inverse());
	
	
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
		x_dot_init_ = x_dot_.block<3,1>(0,0); // 추가 (윤원재)
		x_cubic_old_ = x_;
		rotation_init_ = rotation_;
	}

	if (control_mode_ == "joint_ctrl_home")
	{
		Vector7d target_position;
		target_position << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, M_PI / 4;
		moveJointPosition(target_position, 3.0);
	}
	else if(control_mode_ == "joint_ctrl_init")
	{
		Vector7d target_position;
		target_position << 0.0, 0.0, 0.0, -M_PI / 2., 0.0, 0.0, M_PI / 4;
		moveJointPosition(target_position, 3.0);
	}
	else if (control_mode_ == "torque_ctrl_dynamic")
	{
		Vector7d target_position;
		target_position << 0.0, 0.0, 0.0, -M_PI / 2., 0.0, 0.0, M_PI / 4;
		//moveJointPositionTorque(target_position, 5.0);
	}


	/////////////////////////////////////////////////////////////////////////////////////////////////
	////* -----                       HOMEWORK   2020-26181   윤원재                      ----- *////
	/////////////////////////////////////////////////////////////////////////////////////////////////
	else if (control_mode_ == "HW1-1") {
		moveJointPosition(joint_initial_position, 2.0);
		if ((joint_initial_position - q_).norm() < ERROR_TOLERANCE)	setMode("Lock Joints");
	}
	else if (control_mode_ == "HW1-2_from_initial_joint_position") {
		moveJointPosition(joint_initial_position, 2.0);
		if ((joint_initial_position - q_).norm() < ERROR_TOLERANCE)	setMode("HW1-2");
	}
	else if (control_mode_ == "HW1-2")
	{
		Vector3d position_target; position_target << 0.25, 0.28, 0.65;
		Matrix3d rotation_target; rotation_target << 0, -1, 0,
													 -1, 0, 0,
													 0, 0, -1;
		double settling_time = 2.0;

		for (int i = 0; i < 3; i++) {
			x_cubic_(i) = cubic(play_time_,
								control_start_time_,
								control_start_time_ + settling_time,
								x_init_(i),
								position_target(i),
								0, 0);
			dx_(i) = x_cubic_(i) - x_(i);
		}

		rotation_cubic_ = rotationCubic(play_time_,
										control_start_time_,
										control_start_time_ + settling_time,
										rotation_init_,
										rotation_target);
		phi_ = -getPhi(rotation_, rotation_cubic_);


		for (int i = 0; i < 3; i++) {
			x_error_(i) = dx_(i);
			x_error_(i + 3) = phi_(i);
		}

		qdot_desired_ = j_inverse_ * x_error_; // (7x1) = (7x6)x(6x1) 
		q_desired_ = q_ + qdot_desired_;

		isMotionCompleted(position_target, rotation_target, ERROR_TOLERANCE);

		logData(x_error_);
	}
	else if (control_mode_ == "HW1-3_from_initial_joint_position") {
		moveJointPosition(joint_initial_position, 2.0);
		if ((joint_initial_position - q_).norm() < ERROR_TOLERANCE)	setMode("HW1-3");
	}
	else if (control_mode_ == "HW1-3")
	{
		Vector3d position_target; position_target << 0.25, 0.28, 0.65;
		Matrix3d rotation_target; rotation_target << 0, -1, 0,
													 -1, 0, 0,
													 0, 0, -1;

		//moveTaskPosition(position_target, rotation_target, 1.0);

		double settling_time = 2.0;

		for (int i = 0; i < 3; i++) {
			x_cubic_(i) = cubic(play_time_,
								control_start_time_,
								control_start_time_ + settling_time,
								x_init_(i),
								position_target(i),
								0, 0);
			dx_(i) = x_cubic_(i) - x_from_q_desired_(i);
		}

		rotation_cubic_ = rotationCubic(play_time_,
										control_start_time_,
										control_start_time_ + settling_time,
										rotation_init_,
										rotation_target);
		phi_ = -getPhi(rotation_from_q_desired_, rotation_cubic_);


		for (int i = 0; i < 3; i++) {
			x_error_(i) = dx_(i);
			x_error_(i + 3) = phi_(i);
		}

		q_desired_ = q_ + j_from_q_desired_inverse_ * x_error_;

		isMotionCompleted(position_target, rotation_target, ERROR_TOLERANCE);

		logData(x_error_);
	}
	else if (control_mode_ == "HW1-4_from_initial_joint_position") {
		moveJointPosition(joint_initial_position, 2.0);
		if ((joint_initial_position - q_).norm() < ERROR_TOLERANCE)	setMode("HW1-4");
	}
	else if (control_mode_ == "HW1-4")
	{
		Vector3d position_target; position_target << 0.25, 0.28, 0.65;
		Matrix3d rotation_target; rotation_target << 0, -1, 0,
													 -1, 0, 0,
													 0, 0, -1;

		double settling_time = 2.0;
		
		Eigen::Vector6d vec6d;
		Eigen::Vector7d vec7d;	
		//vec7d << 1.0, 1.0, 1.0, 1000.0, 1.0, 1.0, 1.0; // w4 -> 1000.0: 현재 joint 값을 유지하려고 함 -> redundant motion 발생
		vec7d << 1.0, 1.0, 1.0, 0.01, 1.0, 1.0, 1.0;	 // w4 -> 0.01	: 현재 joint 활용도 up
		W_ = vec7d.asDiagonal();

		vec6d << 150.0, 150.0, 150.0, 150.0, 150.0, 150.0;
		Kp_ = vec6d.asDiagonal();

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
									   x_init_(i), 
									   x_target_(i), 
									   x_dot_init_(i), 
									   x_dot_target_(i), 
									   hz_);
		}

		rotation_cubic_ = rotationCubic(play_time_,
										control_start_time_,
										control_start_time_ + settling_time,
										rotation_init_,
										rotation_target);
		phi_ = -getPhi(rotation_from_q_desired_, rotation_cubic_);


		for (int i = 0; i < 3; i++)
		{
			x_error_(i)	  = dx_(i);
			x_error_(i+3) = phi_(i);

			x_dot_cubic_(i + 3) = 0.0; // RotationDot -> ???
		}
		
		j_from_q_desired_inverse_ = W_.inverse()*j_from_q_desired_.transpose() * ((j_from_q_desired_ * W_.inverse() * j_from_q_desired_.transpose()).inverse());
		//j_ = W_.inverse()*j_.transpose() * ((j_ * W_.inverse() * j_.transpose()).inverse());
		
		q_desired_ = q_ + j_from_q_desired_inverse_ * (x_dot_cubic_ + Kp_ * x_error_) / hz_;
		//q_desired_ = q_ + j_ * (x_dot_cubic_ + Kp_ * x_error_) / hz_;

		isMotionCompleted(position_target, rotation_target, ERROR_TOLERANCE);
		
		logData(x_error_);
	}
	else
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

void ArmController::moveTaskPosition(const Vector3d &position_target, const Matrix3d &rotation_target, double settling_time) {
	for (int i = 0; i < 3; i++)	{
		x_cubic_(i) = cubic(play_time_,
							control_start_time_,
							control_start_time_ + settling_time,
							x_init_(i),
							position_target(i),
							0, 0);

		//dx_(i) = x_cubic_(i) - x_(i);
		dx_(i) = x_cubic_(i) - x_from_q_desired_(i);
	}
	
	rotation_cubic_ = rotationCubic(play_time_, 
									control_start_time_, 
									control_start_time_ + settling_time, 
									rotation_init_, 
									rotation_target);
	//phi_ = getPhi(rotation_, rotation_cubic_);
	phi_ = -getPhi(rotation_from_q_desired_, rotation_cubic_);


	for (int i = 0; i < 3; i++) {
		x_error_(i)		= dx_(i);
		x_error_(i + 3) = phi_(i);
	}

	//qdot_desired_ = j_inverse_ * x_error_; // (7x1) = (7x6)x(6x1) 
	//q_desired_ = q_ + qdot_desired_;
	q_desired_ = q_ + j_from_q_desired_inverse_ * x_error_;
}

void ArmController::moveJointPosition(const Vector7d &q_target, double settling_time)
{
	for (int i = 0; i < DOF; i++) {
		q_desired_(i) = cubic(play_time_, 
							  control_start_time_, 
							  control_start_time_ + settling_time, 
							  q_init_(i), 
							  q_target(i), 
							  0.0, 0.0);
	}
}


void ArmController::printState()
{
	// TODO: Modify this method to debug your code

	static int DBG_CNT = 0;
	if (DBG_CNT++ > hz_ / 2.)
	{
		DBG_CNT = 0;
		cout << "----------------------------------------------" << endl << endl;
		
		cout << "play_time_	: " << std::fixed << std::setprecision(3) << play_time_ << endl;
		cout << "control_mode_	: " << std::fixed << control_mode_ << endl;
		cout << "x_		: " << std::fixed << std::setprecision(3) << x_.transpose() << endl;
		cout << "r_		: " << endl << std::fixed << std::setprecision(3) << rotation_.transpose() << endl;
		cout << "q_		: " << std::fixed << std::setprecision(3) << q_.transpose() << endl;
		//cout << "q_desired_	: " << std::fixed << std::setprecision(3) << q_desired_.transpose() << endl;
		//cout << "deltq q	: " << std::fixed << std::setprecision(3) << (q_desired_ - q_).transpose() << endl;
		cout << "torque_		: " << std::fixed << std::setprecision(3) << torque_.transpose() << endl;
		//cout << "torque_desired_ : " << std::fixed << std::setprecision(3) << torque_desired_.transpose() << endl;
		//cout << "j_		: " << endl << std::fixed << std::setprecision(3) << j_ << endl;
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

	qddot_.setZero();

	x_target_.setZero();
	q_desired_.setZero();
	torque_desired_.setZero();

	g_temp_.resize(DOF);
	m_temp_.resize(DOF, DOF);
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
	com_position_[6] = Vector3d(0.1, -0.0120, 0.9536);

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

// ----------------------------------------------------

