#pragma once
#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include <iostream>
#include <Eigen/Dense>
#include <rbdl/rbdl.h>
#include <memory>
#include <fstream>
#include "math_type_define.h"

#define EYE(X) Matrix<double, X, X>::Identity()

using namespace RigidBodyDynamics;
using namespace std;
using namespace Eigen;

class ArmController
{
	size_t dof_;

	// Initial state
	Vector7d q_init_;
	Vector7d qdot_init_;
	Vector7d qdot_target_;

	// Current state
	Vector7d q_;
	Vector7d qdot_;
	Vector7d qddot_;
	Vector7d torque_;
	Vector7d q_error_sum_;

	// Control value (position controlled)
	Vector7d q_desired_; // Control value
	Vector7d torque_desired_;
	double settling_time_;

	// Task space
	Vector3d x_init_;
	Vector3d x_;
	Vector3d x_from_q_desired_; // �߰� (ETL)
	Vector3d x_dot_init_;

	// Task Transition (Joint 4 ���� Kinematics) 
	Vector3d x1_;
	Vector3d x1_dot_CLIK_;
	Vector3d x1_dot_input_;
	Vector3d x1_init_; // �߰� (������)
	Vector6d x1_dot_;
	Matrix<double, 6, 7> j1_; // �߰� (������)
	Matrix<double, 3, 7> j1_v_;
	Matrix<double, 7, 3> j1_v_inverse_;
	
	Vector3d x2_;
	Vector3d x2_dot_CLIK_;
	Vector3d x2_dot_input_;
	Vector3d x2_init_; // �߰� (������)
	Vector6d x2_dot_;
	Matrix<double, 6, 7> j2_; // �߰� (������)
	Matrix<double, 3, 7> j2_v_;
	Matrix<double, 7, 3> j2_v_inverse_;

	Matrix<double, 7, 6> j_tot_transpose_;
	Matrix<double, 6, 7> j_tot_;
	Matrix<double, 7, 6> j_tot_inverse_;
	Vector6d x_cubic_tot_;
	Vector6d x_tot_;
	Vector6d x_cubic_dot_tot_;
	Vector6d x_dot_tot_;

	Matrix3d Kp_1_;
	Matrix3d Kp_2_;
	Matrix7d N1_;

	Vector3d x2_from_q_desired_; // �߰� (ETL)
	MatrixXd j2_temp_from_q_desired_; // �߰� (ETL)
	Matrix<double, 6, 7> j2_from_q_desired_; // �߰� (ETL)
	Matrix<double, 3, 7> j2_v_from_q_desired_; // �߰� (������)


	Matrix3d rotation_;
	Matrix3d rotation_from_q_desired_; // �߰� (ETL)
	Matrix3d rotation_init_;
	Matrix3d rotation_target_; // target orientation in rotation matrix // �߰� (������)
	Matrix3d rotation_cubic_; // cubic spline interpolated rotation matrix // �߰� (������)
	Vector3d v_cubic_; // �߰� (������)
	Vector3d w_cubic_; // �߰� (������)
	Vector3d dx_; // �߰� (������)
	Vector3d phi_;
	Vector6d x_dot_; // 6D (linear + angular)
	Vector6d x_error_; // dX (dx, dy, dz, drx, dry, drz)
	Vector6d x_error_to_target_; // error to the target pose from current pose // �߰� (������)

								 // Dynamics
	Vector7d g_; // Gravity torque
	Matrix7d m_; // Mass matrix
	Matrix7d m_inverse_; // Inverse of mass matrix
	Matrix7d W_; // �߰� (������)
	Matrix6d Kp_; // �߰� (������)

				  // For controller
	Matrix<double, 3, 7> j_v_;	// Linear velocity Jacobian matrix
	Matrix<double, 3, 7> j_w_;	// Angular veolicty Jacobain matrix
	Matrix<double, 6, 7> j_;	// Full basic Jacobian matrix
	Matrix<double, 6, 7> j_from_q_desired_; // �߰� (ETL)
	MatrixXd j_temp_from_q_desired_; // �߰� (ETL)
	Matrix<double, 7, 6> j_inverse_from_q_desired_; // �߰� (ETL)
	Matrix<double, 7, 6> j_inverse_;	// Jacobain inverse storage 

	VectorXd q_temp_;	// For RBDL 
	VectorXd qdot_temp_;
	VectorXd qddot_temp_;
	MatrixXd j_temp_;	// For RBDL 
	MatrixXd m_temp_;
	VectorXd g_temp_;   // For RBDL 

	Vector7d q_cubic_;
	Vector7d q_target_;
	Vector7d qdot_desired_; // �߰� (������)

	Vector3d x_cubic_;
	Vector3d x_cubic_old_;
	Vector3d x_target_;
	Vector6d x_dot_cubic_; // �߰� (������)
	Vector3d x_dot_target_; // �߰� (������)

	Vector3d x1_cubic_; // �߰� (������)
	Vector6d x1_dot_cubic_; // �߰� (������)
	Vector3d x2_cubic_; // �߰� (������)
	Vector6d x2_dot_cubic_; // �߰� (������)

	double h1_;
	double h2_;

	unsigned long tick_;
	double play_time_;
	double hz_;
	double control_start_time_;

	std::string control_mode_;
	bool is_mode_changed_;

	// for robot model construction
	Math::Vector3d com_position_[DOF];
	Vector3d joint_posision_[DOF];

	shared_ptr<Model> model_;
	unsigned int body_id_[DOF];
	Body body_[DOF];
	Joint joint_[DOF];

private:
	void printState();
	void moveJointPosition(const Vector7d &target_position, double duration);
	void moveTaskPosition(const Vector3d &position_now, const Vector3d &position_target,
		const Matrix3d &rotation_now, const Matrix3d &rotation_target,
		const Matrix<double, 7, 6> &jacobian_inverse, double settling_time);
	//void moveJointPositionTorque(const Vector7d &target_position, double duration);

public:
	void readData(const Vector7d &position, const Vector7d &velocity, const Vector7d &torque);
	void readData(const Vector7d &position, const Vector7d &velocity);
	const Vector7d & getDesiredPosition();
	const Vector7d & getDesiredTorque();

public:
	ArmController(double hz) :
		tick_(0), play_time_(0.0), hz_(hz), control_mode_("none"), is_mode_changed_(false)
	{
		initDimension(); initModel();
	}


	void setMode(const std::string & mode);
	void initDimension();
	void initModel();
	void initPosition();
	void compute();
	void calcKinematics(Vector7d q, Vector7d qdot);
	void logData(Eigen::Vector6d x_error);
	void logData_TaskTransition(Eigen::Vector3d x1, Eigen::Vector3d x2);
	void isMotionCompleted(Eigen::Vector3d, Eigen::Matrix3d rotation_target, double tolerance);
	void moveTaskPositionCLIK(const Vector3d &position_now, const Vector3d &position_target,
							  const Matrix3d &rotation_now, const Matrix3d &rotation_target,
							  const Matrix<double, 7, 6> &jacobian_inverse,
							  double settling_time);
};

#endif
