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
	
	// Constant Vectors & Matrices (윤원재 추가)
	

	// Initial state
	Vector7d q_init_;
	Vector7d qdot_init_;

	// Current state
	Vector7d q_;
	Vector7d qdot_;
	Vector7d qddot_;
	Vector7d torque_;
	Vector7d q_error_sum_;

	// Control value (position controlled)
	Vector7d q_desired_; // Control value
	Vector7d torque_desired_;

	// Task space
	Vector3d x_init_;
	Vector3d x_;
	Vector3d x_from_q_desired_; // 추가 (ETL)
	Vector3d x_dot_init_;
	
	
	
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
	Vector6d x_error_; 

	// Dynamics
	Vector7d g_; // Gravity torque
	Matrix7d m_; // Mass matrix
	Matrix7d m_inverse_; // Inverse of mass matrix
	Matrix7d W_; // 추가 (윤원재)
	Matrix6d Kp_; // 추가 (윤원재)

	// For controller
	Matrix<double,3,7> j_v_;	// Linear velocity Jacobian matrix
	Matrix<double, 3, 7> j_w_;	// Angular veolicty Jacobain matrix
	Matrix<double, 6, 7> j_;	// Full basic Jacobian matrix
	Matrix<double, 6, 7> j_from_q_desired_; // 추가 (ETL)
	MatrixXd j_temp_from_q_desired_; // 추가 (ETL)
	Matrix<double, 7, 6> j_from_q_desired_inverse_; // 추가 (ETL)
	Matrix<double, 7, 6> j_inverse_;	// Jacobain inverse storage 

	VectorXd q_temp_;	// For RBDL 
	VectorXd qdot_temp_;
	VectorXd qddot_temp_;
	MatrixXd j_temp_;	// For RBDL 
	MatrixXd m_temp_;
	VectorXd g_temp_;   // For RBDL 

	Vector7d q_cubic_;
	Vector7d q_target_;
	Vector7d qdot_desired_; // 추가 (윤원재)

	Vector3d x_cubic_;
	Vector3d x_cubic_old_;
	Vector3d x_target_;
	Vector6d x_dot_cubic_; // 추가 (윤원재)
	Vector3d x_dot_target_; // 추가 (윤원재)

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
	void moveTaskPosition(const Vector3d &position_target, const Matrix3d &rotation_target, double settling_time);
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
};

#endif
