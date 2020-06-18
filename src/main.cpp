#pragma once
#include <iostream>
#include <string>
#include <conio.h>
#include "vrep_bridge.h"

#include "controller.h"

using namespace std;

int main()
{
#ifdef TORQUE_CONTROL_MODE
	VRepBridge vb(VRepBridge::CTRL_TORQUE); // Torque controlled
	const double hz = 1000;
#endif
#ifndef TORQUE_CONTROL_MODE
	VRepBridge vb(VRepBridge::CTRL_POSITION); // Position controlled 
	const double hz = 100;
#endif

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

#ifdef FINAL_PROJECT
			case 'c':
				ac.setMode("center");
				break;
			case '0':
				ac.setMode("init");
				break;
			case '1':
				ac.setMode("target1");
				break;
			case '2':
				ac.setMode("target2");
				break;
			case '3':
				ac.setMode("target3");
				break;
			case '4':
				ac.setMode("target4");
				break;
			case 's':
				ac.setMode("start");
				// 1. RRT Planner	("planning")
				// 2. Robot control ("control")
				vb.getProjectStartTime();
				break;
#endif

#ifdef HW2
			case '0':
				ac.setMode("HW2-0");
				break;
			case '1':
				ac.setMode("HW2-1_from_initial_joint_position");
				//ac.setMode("HW2-1");
				break;
			case '2':
				ac.setMode("HW2-2_from_initial_joint_position");
				//ac.setMode("HW2-2");
				break;
			case '3':
				ac.setMode("HW2-3_from_initial_joint_position");
				//ac.setMode("HW2-3");
				break;
#endif

#ifdef HW3
			case '1':
				ac.setMode("HW3-1_from_initial_joint_position");
				//ac.setMode("HW3-1");
				break;
			case '2':
				ac.setMode("HW3-2_from_initial_joint_position");
				//ac.setMode("HW3-2");
				break;
			case '3':
				ac.setMode("HW3-3_from_initial_joint_position");
				//ac.setMode("HW3-3");
				break;
#endif

#ifdef HW4
			case '0':
				ac.setMode("HW4_init");
				break;
			case '1':
				ac.setMode("HW4-1");
				break;
			case '2':
				ac.setMode("HW4-2(a)");
				break;
			case '3':
				ac.setMode("HW4-2(b)");
				break;
			case '4':
				ac.setMode("HW4-3(a)");
				break;
			case '5':
				ac.setMode("HW4-3(b)");
				//ac.setMode("HW4-2(a)");
				break;
#endif
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
#ifdef FINAL_PROJECT
			if (ac.projectFinish())
				vb.getProjectFinishTime();
		}
#endif
	}

	return 0;
}
