#pragma once
#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#define FINAL_PROJECT
//#define HW2
//#define HW3
//#define HW4
//#define HW5
//#define HW6
//#define HW7

#if defined(HW4) || defined(HW5) || defined(HW6) || defined(HW7)
#define TORQUE_CONTROL_MODE // Torque Control Mode (원재 추가)
#endif

#include <iostream>
#include <Eigen/Dense>
#include <rbdl/rbdl.h>
#include <memory>
#include <fstream>
#include "math_type_define.h"
#include "RRT.h"

#define WAYPOINT_NUM_MAX 1000

#define EYE(X) Matrix<double, X, X>::Identity()

using namespace RigidBodyDynamics;
using namespace std;
using namespace Eigen;

class ArmController
{
	/*---  FINAL PROJECT  ---*/
	double obstacle_[9];
	double waypoint_[WAYPOINT_NUM_MAX][2];
	int wp_n_;   // Final project -> waypoint n번째
	int wp_Num_; // waypoint 개수
	double wp_tolerance_;
	double wp_settling_time_;
	double padding_obstacle_;
	double weight_speed_;



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
	Vector3d x_from_q_desired_; // 추가 (ETL)
	Vector3d x_dot_init_;
	Vector3d x_desired_;
	Vector3d x_dot_desired_;

	// Task Transition (Joint 4 기준 Kinematics) 
	Vector3d x1_;
	Vector3d x1_dot_CLIK_;
	Vector3d x1_dot_input_;
	Vector3d x1_init_; // 추가 (윤원재)
	Vector6d x1_dot_;
	Matrix<double, 6, 7> j1_; // 추가 (윤원재)
	Matrix<double, 3, 7> j1_v_;
	Matrix<double, 7, 3> j1_v_inverse_;
	
	Vector3d x2_;
	Vector3d x2_dot_CLIK_;
	Vector3d x2_dot_input_;
	Vector3d x2_init_; // 추가 (윤원재)
	Vector6d x2_dot_;
	Matrix<double, 6, 7> j2_; // 추가 (윤원재)
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

	Matrix7d Kp_joint_;
	Matrix7d Kp_joint_temp_;
	Matrix7d Kv_joint_;
	Matrix7d Kv_joint_temp_;

	Vector3d x2_from_q_desired_; // 추가 (ETL)
	MatrixXd j2_temp_from_q_desired_; // 추가 (ETL)
	Matrix<double, 6, 7> j2_from_q_desired_; // 추가 (ETL)
	Matrix<double, 3, 7> j2_v_from_q_desired_; // 추가 (윤원재)


	Matrix3d rotation_;
	Matrix3d rotation_from_q_desired_; // 추가 (ETL)
	Matrix3d rotation_init_;
	Matrix3d rotation_target_; // target orientation in rotation matrix // 추가 (윤원재)
	Matrix3d rotation_cubic_; // cubic spline interpolated rotation matrix // 추가 (윤원재)
	Vector3d v_cubic_; // 추가 (윤원재)
	Vector3d w_cubic_; // 추가 (윤원재)
	Vector3d dx_; // 추가 (윤원재)
	Vector3d phi_;
	Vector6d x_dot_; // 6D (linear + angular)
	Vector6d x_error_; // dX (dx, dy, dz, drx, dry, drz)
	Vector6d x_error_to_target_; // error to the target pose from current pose // 추가 (윤원재)


	Vector7d g_; // Gravity torque (joint space)
	Matrix7d m_; // Mass matrix (joint space)
	Matrix7d m_inverse_; // Inverse of mass matrix (joint space)
	Matrix7d W_; // 추가 (윤원재)
	Matrix6d Kp_; // 추가 (윤원재)

	Matrix6d m_task_; // Mass matrix (task space)

	Matrix3d Kp_task_; // position gain (task space control)
	Matrix3d Kv_task_; // velocity gain (task space control)
	Vector6d F_star_; // desired force & moment
	Vector3d f_star_attractive_; // attractive force
	Vector3d f_star_repulsive_;  // repulsive force
	double K_obstacle_;
	Vector3d f_star_; // desired force
	Vector3d m_star_; // desired moment


				  // For controller
	Matrix<double, 3, 7> j_v_;	// Linear velocity Jacobian matrix
	Matrix<double, 3, 7> j_w_;	// Angular veolicty Jacobain matrix
	Matrix<double, 6, 7> j_;	// Full basic Jacobian matrix
	Matrix<double, 6, 7> j_from_q_desired_; // 추가 (ETL)
	MatrixXd j_temp_from_q_desired_; // 추가 (ETL)
	Matrix<double, 7, 6> j_inverse_from_q_desired_; // 추가 (ETL)
	Matrix<double, 7, 6> j_inverse_;	// Jacobain inverse storage
	Matrix<double, 6, 7> j_dc_inv_transpose_; // Dynamic Consistent Jacobian (for task space control)

	VectorXd q_temp_;	// For RBDL 
	VectorXd qdot_temp_;
	VectorXd qddot_temp_;
	MatrixXd j_temp_;	// For RBDL 
	MatrixXd m_temp_;
	VectorXd g_temp_;   // For RBDL 

	Vector7d q_cubic_;
	Vector7d qdot_cubic_;
	Vector7d q_target_;
	Vector7d qdot_desired_; // 추가 (윤원재)

	Vector3d x_cubic_;
	Vector3d x_cubic_old_;
	Vector3d x_target_;
	Vector6d x_dot_cubic_; // 추가 (윤원재)
	Vector3d x_dot_target_; // 추가 (윤원재)

	Vector3d x1_cubic_; // 추가 (윤원재)
	Vector6d x1_dot_cubic_; // 추가 (윤원재)
	Vector3d x2_cubic_; // 추가 (윤원재)
	Vector6d x2_dot_cubic_; // 추가 (윤원재)

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
	bool projectFinish();

	void calcKinematics(Vector7d q, Vector7d qdot);
	void logData_joint(Vector7d q_target);
	void logData_task(Vector3d x_target, Matrix3d rotation_target);
	void logData(Eigen::Vector6d x_error);
	void logData_TaskTransition(Eigen::Vector3d x1, Eigen::Vector3d x2);
	void logData_JointError(Eigen::Vector7d q_target, Eigen::Vector7d q);
	void isMotionCompleted(Eigen::Vector3d, Eigen::Matrix3d rotation_target, double tolerance);
	bool isMotionCompleted_Task(Eigen::Vector3d position_target, Eigen::Matrix3d rotation_target, double tolerance, string state);
	bool isMotionCompleted_Joint(Eigen::Vector7d q_target, double tolerance, string state);
	void moveTaskPositionCLIK(const Vector3d &position_now, const Vector3d &position_target,
							  const Matrix3d &rotation_now, const Matrix3d &rotation_target,
							  const Matrix<double, 7, 6> &jacobian_inverse,
							  double settling_time);
	void moveJointPositionbyTorque(const Vector7d &q_target, double settling_time);
	void moveTaskPositionbyTorquePD(const Vector3d &x_target, const Matrix3d &rotation_target, double settling_time);
	void moveTaskPositionbyTorqueVelSat(const Vector3d &x_target, const Matrix3d &rotation_target, double x_dot_max, double settling_time);
	void moveTaskPositionbyTorqueAvoidObstacle(const Vector3d &x_target, const Matrix3d &rotation_target, double x_dot_max, const Vector3d &x_obstacle, double d0, double settling_time);


	void calcSettlingTime(Eigen::Vector3d waypoint);
	void RRT_IDIM(double* start_position, double* target_position, double* obstacle);
};

#endif
