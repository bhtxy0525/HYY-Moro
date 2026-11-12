#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include "HYYRobotInterface.h"
#include <atomic>
#include <thread>
#include <memory>
#include <cassert>
#include "fdcc_hri_controller.h"

using namespace HYYRobotBase;
extern void custom_websocket(void *arg);
extern void custom_http(void *arg);
extern int HumanoidMoveDemo();
extern int HumanoidServoDemo();
extern int kinematics_test();
extern int TeleTest();
extern int TeleRecord();
extern int HumanoidTeleoperationDemo(double position_scale);

int main(int argc, char *argv[])
{
	//------------------------initialize----------------------------------
	int err = 0;
	HYYRobotBase::command_arg arg;
	err = HYYRobotBase::commandLineParser(argc, argv, &arg);
	if (0 != err)
	{
		return -1;
	}
	if (!HYYRobotBase::SystemInit(&arg, custom_http, custom_websocket))
	{
		return -2;
	}

	//  if (!HYYRobotBase::SystemInit(&arg))
	//  {
	//  	return -2;
	//  }

	//----------------Deploy your code--------------------------------------
	// Rsleep(3000);
	// kinematics_test();
	HumanoidMoveDemo();
	// HumanoidTeleoperationDemo(1.5);
	// HumanoidServoDemo();

	//----------------My paper code -------------------------------------
	// fdcc::fdccNoMotionSelfTest();
	// fdcc::fdccHRIExperiment1_1Start();

	return 0;
}
