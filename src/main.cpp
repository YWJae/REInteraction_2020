#pragma once
#include <iostream>
#include <string>
#include <conio.h>
#include "vrep_bridge.h"

#include "controller.h"

using namespace std;



int main()
{
	//VRepBridge vb(VRepBridge::CTRL_TORQUE); // Torque controlled
	VRepBridge vb(VRepBridge::CTRL_POSITION); // Position controlled 
	const double hz = 100;
	ArmController ac(hz);
	bool is_simulation_run = true;
	bool exit_flag = false;
	bool is_first = true;

	while (vb.simConnectionCheck() && !exit_flag)
	{
		vb.read();
		ac.readData(vb.getPosition(), vb.getVelocity());
		if (is_first)
		{
			vb.simLoop();
			vb.read();
			ac.readData(vb.getPosition(), vb.getVelocity());
			cout << "Initial q: " << vb.getPosition().transpose() << endl;
			is_first = false;
			ac.initPosition();
		}

		if (_kbhit())
		{
			int key = _getch();

			switch (key)
			{
				// Implement with user input
			case '`':
				ac.setMode("Change Parameters");
				break;
			case '0':
				ac.setMode("HW1-0");
				break;
			case '1':
				ac.setMode("HW1-1_from_initial_joint_position");
				//ac.setMode("HW1-1");
				break;
			case '2':
				ac.setMode("HW1-2_from_initial_joint_position");
				//ac.setMode("HW1-2");
				break;
			case '3':
				ac.setMode("HW1-3_from_initial_joint_position");
				//ac.setMode("HW1-3");
				break;
			case '4':
				ac.setMode("HW3-1_from_initial_joint_position");
				//ac.setMode("HW1-3");
				break;
			case '5':
				ac.setMode("HW3-2_from_initial_joint_position");
				//ac.setMode("HW1-3");
				break;
			case '6':
				ac.setMode("HW3-3_from_initial_joint_position");
				//ac.setMode("HW1-3");
				break;
			case 'i':
				ac.setMode("joint_ctrl_init");
				break;
			case 'h':
				ac.setMode("joint_ctrl_home");
				break;
			case 't':
				ac.setMode("torque_ctrl_dynamic");
				break;


			case '\t':
				if (is_simulation_run) {
					cout << "Simulation Pause" << endl;
					is_simulation_run = false;
				}
				else {
					cout << "Simulation Run" << endl;
					is_simulation_run = true;
				}
				break;
			case 'q':
				is_simulation_run = false;
				exit_flag = true;
				break;
			default:
				break;
			}
		}

		if (is_simulation_run) {
			ac.compute();
			vb.setDesiredPosition(ac.getDesiredPosition());
			vb.setDesiredTorque(ac.getDesiredTorque());

			vb.write();
			vb.simLoop();
		}
	}

	return 0;
}
