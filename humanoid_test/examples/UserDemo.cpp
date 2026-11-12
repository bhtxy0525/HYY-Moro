#include "UserDemo.h"
int RobotEnableDemo()
{
	// 如果有驱动错误，先清错
	ClearDeviceError();

	// 所有设备上下电，不建议，当前机械臂无法一次成功上电，而且同时上电电流也比较大，建议子设备分开上电
	// DevicePower();
	// Rsleep(2000);
	// DevicePoweroff();

	// 子设备上下电
	RobotPower(0);
	Rsleep(2000);
	RobotPoweroff(0);
	RobotPower(1);
	Rsleep(2000);
	RobotPoweroff(1);
	RobotPower(2);
	Rsleep(2000);
	RobotPoweroff(2);
	Rsleep(2000);
	RobotPoweroff(2);
	AdditionPower(0); // 头部
	Rsleep(2000);
	AdditionPoweroff(0);

	// 停止任务，配合任务逻辑使用
	// DeviceStopRun(); // 所有设备停止当前运动
	// RobotStopRun(0); // 指定子设备停止当前运动
	return 0;
}
int MoveDemo()
{
	// 以左臂为例,测试关节和笛卡尔空间运动
	IMPORTJOINT(R0_PTEST); // 示教器提前保存的点位
	IMPORTSPEED(v0);
	SETJOINT7(ZERO, 0, 0, 0, 0, 0, 0, 0);

	// 上电
	int ret = 0;
	ret = RobotPower(0);
	// 先运动到初始零点ZERO，再运动到RO_HOME，展示两种关节目标变量的定义方法
	MultiMoveA(ZERO, v0, NULL3, 0);
	Rsleep(2000);
	MultiMoveA(R0_PTEST, v0, NULL3, 0);
	// 在当前位置走笛卡尔MoveL指令,MoveC同理
	robpose tp, pout;
	GetCurrentCartesian(NULL, NULL, &tp, 0);  // 获取机器人末端坐标系笛卡尔位姿信息
	pout = OffsRel(&tp, 0.03, 0, 0, 0, 0, 0); // 相对当前坐标系X轴移动向前移动0.03m，或者使用offs相对基坐标系移动
	int count = 0;
	while (robot_ok())
	{
		if (count == 5)
		{
			break;
		}
		MultiMoveL(&pout, v0, NULL, NULL, NULL, 0);
		MultiMoveL(&tp, v0, NULL, NULL, NULL, 0);
		count += 1;
		std::cout << "current loop is: " << count << std::endl;
	}
	return 0;
}
int DualArmMoveDemo()
{
	IMPORTJOINT(R0_PTEST);
	IMPORTJOINT(R1_PTEST);
	IMPORTSPEED(v0);

	// 上电
	int ret = 0;
	ret = RobotPower(0);
	ret = RobotPower(1);

	// 双臂关节运动至目标位置
	DualMoveA(R0_PTEST, R1_PTEST, v0, v0, NULL3, NULL3);

	// 双臂笛卡尔运动
	robpose tp1, pout1, tp2, pout2;
	GetCurrentCartesian(NULL, NULL, &tp1, 0);
	GetCurrentCartesian(NULL, NULL, &tp2, 1);
	pout1 = Offs(&tp1, 0.03, 0, 0, 0, 0, 0); // 相对基坐标系X轴向前移动0.03m
	pout2 = Offs(&tp2, 0.03, 0, 0, 0, 0, 0); // 相对基坐标系X轴向前移动0.03m
	int count = 0;
	while (robot_ok())
	{
		if (count == 5)
		{
			break;
		}
		//---------------双臂运动----------------------------------------------
		DualMoveL(&pout1, &pout2, v0, v0, NULL3, NULL3);
		DualMoveL(&tp1, &tp2, v0, v0, NULL3, NULL3);
		count += 1;
		std::cout << "current loop is: " << count << std::endl;
	}
	Rsleep(2000);
	return 0;
}
int AdditionAxisDemo()
{
	IMPORTSPEED(v0);
	AdditionPower(0);
	SETJOINT2(ZERO, 0, 0);
	SETJOINT2(TAR, M_PI / 6, M_PI / 6);
	MultiMoveAdd(TAR, v0, NULL, NULL, NULL, 0);
	MultiMoveAdd(ZERO, v0, NULL, NULL, NULL, 0);
	AdditionPoweroff(0);
	return 0;
}
int MoveToPoseDemo()
{
	IMPORTJOINT(R0_PTEST);
	IMPORTJOINT(R1_PTEST);
	IMPORTSPEED(v0);

	// 上电
	int ret = 0;
	ret = RobotPower(0);
	ret = RobotPower(1);

	// 双臂关节运动至目标位置
	DualMoveA(R0_PTEST, R1_PTEST, v0, v0, NULL3, NULL3);

	// 双臂笛卡尔运动
	robpose tp1, pout1, tp2, pout2;
	GetCurrentCartesian(NULL, NULL, &tp1, 0);
	GetCurrentCartesian(NULL, NULL, &tp2, 1);
	pout1 = Offs(&tp1, 0.03, 0, 0.03, 0, 0, 0); // 相对基坐标系X轴向前移动0.03m,Z轴向前移动0.03m
	pout2 = Offs(&tp2, 0.03, 0, 0.03, 0, 0, 0); // 相对基坐标系X轴向前移动0.03m，Z轴向前移动0.03m
	int count = 0;
	while (robot_ok())
	{
		if (count == 5)
		{
			break;
		}
		//---------------双臂运动----------------------------------------------
		DualMoveJ(&pout1, &pout2, v0, v0, NULL3, NULL3);
		DualMoveJ(&tp1, &tp2, v0, v0, NULL3, NULL3);
		count += 1;
		std::cout << "current loop is: " << count << std::endl;
	}
	Rsleep(2000);
	return 0;
}
int StepMoveDemo()
{
	// 以左臂为例，执行步进指令
	IMPORTSPEED(v0);
	IMPORTJOINT(R0_PTEST);

	//  上电
	int ret = 0;
	ret = RobotPower(0);

	MultiMoveA(R0_PTEST, v0, NULL3, 0); // 运动到初始位姿
	// 开始执行步进指令
	set_robot_index(0);
	set_teach_move_type(1);
	set_robot_teach_coordinate(0);
	ret = robot_teach_move(0, 1); // 关节6步进运动，默认速度是0.3rad/s
	Rsleep(10000);
	std::cout << "joint step move finished..." << std::endl;
	set_robot_teach_coordinate(1);
	ret = robot_teach_move(1, 0); // 笛卡尔x向步进，默认速度是0.3m/s
	Rsleep(10000);
	std::cout << "cartesian step move finished..." << std::endl;
	ret = robot_teach_move(1, 3); // 笛卡尔rx步进，默认速度是0.3m/s
	Rsleep(10000);
	std::cout << "cartesian step move finished..." << std::endl;
	return 0;
}
int SpeedAMoveDemo()
{
	// 以左臂5关节为例，测试关节速度运动
	IMPORTSPEED(v0);
	IMPORTJOINT(R0_PTEST);

	//  上电
	int ret = 0;
	ret = RobotPower(0);

	MultiMoveA(R0_PTEST, v0, NULL3, 0);
	setMoveThread(1); // 开启运动线程
	double vel[7] = {0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0};
	ret = MultiSpeedA(vel, NULL, NULL, 0);
	sleep(5);
	DeviceStopRun();
	std::cout << "move finish..." << std::endl;
	return 0;
}
int SpeedLMoveDemo()
{
	// 以左臂为例，测试笛卡尔速度运动
	IMPORTSPEED(v0);
	IMPORTJOINT(R0_PTEST);

	//  上电
	int ret = 0;
	ret = RobotPower(0);

	MultiMoveA(R0_PTEST, v0, NULL3, 0);
	setMoveThread(1);
	double vel[6] = {0.004, 0.000, 0, 0.0, 0.0, 0.0};
	ret = MultiSpeedL(vel, NULL, NULL, 0);
	printf("move finish...");
	sleep(5);
	DeviceStopRun();
	std::cout << "move finish..." << std::endl;
	return 0;
}
int SetZeroDemo()
{
	// 以左臂为例，标定左臂零位、力矩零位，标定前请确保机械臂运动到用户自定义零位
	IMPORTSPEED(v0);
	SETJOINT7(ZERO, 0, 0, 0, 0, 0, 0, 0);

	//  上电
	int ret = 0;
	ret = RobotPower(0);

	MultiMoveA(ZERO, v0, NULL3, 0);
	Rsleep(1000);
	set_robot_index(0); // 标定前需要先设置当前子设备
	calibrate_robot_zero_position();
	// calibrate_robot_zero_axis_position(0);    //标定当前子设备某个轴零位
	calibrate_robot_zero_sensortorque();
	// calibrate_robot_zero_axis_sensortorque(); //标定当前子设备某个轴力矩零位
	calibrate_addition_zero_position();
	// calibrate_addition_zero_axis_position(0);  //标定当前外部轴设备某个轴零位
	return 0;
}

int GetLimitDemo()
{
	// 以左臂为例，查看左臂的关节、速度和加速度极限
	const DeviceConfig *Config = GetDeviceConfig(); // 通过该指针，还可以拿到配置文件中对应各轴组的其他配置参数
	// init joint limit
	double qmax[7];	  // 位置上限
	double qmin[7];	  // 位置下限
	double dqmax[7];  // 速度上限
	double dqmin[7];  // 速度下限
	double ddqmax[7]; // 加速度上限
	double ddqmin[7]; // 加速度下限
	for (int i = 0; i < 7; i++)
	{
		qmax[i] = Config->robot_group->robot[0].joint[i].axis.smax;
		qmin[i] = Config->robot_group->robot[0].joint[i].axis.smin;
		dqmax[i] = Config->robot_group->robot[0].joint[i].axis.vmax;
		dqmin[i] = Config->robot_group->robot[0].joint[i].axis.vmin;
		ddqmax[i] = Config->robot_group->robot[0].joint[i].axis.amax;
		ddqmin[i] = Config->robot_group->robot[0].joint[i].axis.amin;
	}
	return 0;
}
int GetRobotStateDemo()
{
	// 以左臂为例，1ms周期性获取各关节位置、目标位置、速度、角速度、关节力矩、关节力矩传感器力矩和末端笛卡尔位姿反馈数据
	const char *left_arm = get_name_robot_device(get_deviceName(0, NULL), 0);
	int dof = get_group_dof(left_arm);
	double dt = get_control_cycle(get_deviceName(0, NULL));

	// 定义想要记录的数据
	double tmp[100];
	double joint_feedback[7];		 // 关节位置反馈，1*7
	double joint_target[7];			 // 关节目标位置反馈，1*7
	double velocity_feedback[7];	 // 关节角速度反馈，1*7
	double acceleration_feedback[7]; // 关节角加速度反馈，1*7
	double torque[7];				 // 关节力矩反馈，1*7
	double sensor_torque[7];		 // 关节力矩传感器反馈，1*7
	double pose_feedback[6];		 // 机械臂末端笛卡尔位姿反馈，1*6

	// 初始化微分
	CentralDifferences dvelocity;
	init_central_difference_onlines(&dvelocity, dt, dof);
	// 初始化滤波
	TIIRFilters facc;
	initTIIRFilters(&facc, iirLPF, 50, 1.0 / dt, dof);

	// 设置定时器
	RTimer timer;
	initUserTimer(&timer, 0, 1);

	// 设置保存文件
	char feedback_cmp[2 * MAX_CHAR_NUM];
	memset(feedback_cmp, 0, sizeof(feedback_cmp));
	sprintf(feedback_cmp, "feedback_data");
	CreateSaveData1(feedback_cmp, 1, 1000, 47);
	robpose tp;
	int count = 0;
	while (robot_ok())
	{
		if (count == 5000)
		{
			break;
		}

		// 1ms周期循环
		userTimer(&timer);
		// 读取数据
		GetGroupSensorTorque(left_arm, sensor_torque);	// 获取轴组关节力矩传感器力矩
		GetGroupTorque(left_arm, torque);				// 获取轴组关节力矩
		GetGroupPosition(left_arm, joint_feedback);		// 获取轴组关节位置
		GetGroupTargetPosition(left_arm, joint_target); // 获取轴组目标关节位置
		GetGroupVelocity(left_arm, velocity_feedback);	// 获取轴组关节速度
		central_difference_onlines(&dvelocity, velocity_feedback, acceleration_feedback, dof);
		IIRFilters(&facc, acceleration_feedback, acceleration_feedback); // 微分获取轴组关节角加速度
		GetCurrentCartesian(NULL, NULL, &tp, 0);						 // 获取机械臂末端笛卡尔位姿
		pose_feedback[0] = tp.xyz[0];
		pose_feedback[1] = tp.xyz[1];
		pose_feedback[2] = tp.xyz[2];
		pose_feedback[3] = tp.kps[0];
		pose_feedback[4] = tp.kps[1];
		pose_feedback[5] = tp.kps[2];
		// 记录数据
		memcpy(tmp, joint_feedback, dof * sizeof(double));
		memcpy(&(tmp[dof]), joint_target, dof * sizeof(double));
		memcpy(&(tmp[2 * dof]), velocity_feedback, dof * sizeof(double));
		memcpy(&(tmp[3 * dof]), acceleration_feedback, dof * sizeof(double));
		memcpy(&(tmp[4 * dof]), torque, dof * sizeof(double));
		memcpy(&(tmp[5 * dof]), sensor_torque, dof * sizeof(double));
		memcpy(&(tmp[42]), pose_feedback, 6 * sizeof(double));
		if (0 != RSaveData(feedback_cmp, tmp))
		{
			printf("data_record: RSaveData %s lost\n", feedback_cmp);
		}
		count++;
	}
	Rsleep(2000);
	DeleteSaveData(feedback_cmp);
	return 0;
}
int JacoDemo()
{
	// 获取雅可比矩阵，相对基座
	double joint_feedback[7];
	const char *left_arm = get_name_robot_device(get_deviceName(0, NULL), 0);
	double **jac = new double *[6];
	for (int i = 0; i < 6; i++)
	{
		jac[i] = new double[7];
	}
	GetGroupPosition(left_arm, joint_feedback);
	Kine_Jacob(left_arm, joint_feedback, jac, NULL, NULL);
	// 转换Eigen矩阵
	Eigen::Matrix<double, 6, 7> J;
	for (int i = 0; i < 6; i++)
	{
		for (int j = 0; j < 7; j++)
		{
			J(i, j) = jac[i][j];
		}
	}
	std::cout << "J is : " << J << std::endl;
	// 获取雅可比矩阵，相对末端
	Kine_JacobEnd(left_arm, joint_feedback, jac, NULL, NULL);
	// 转换Eigen矩阵
	for (int i = 0; i < 6; i++)
	{
		for (int j = 0; j < 7; j++)
		{
			J(i, j) = jac[i][j];
		}
	}
	std::cout << "J is : " << J << std::endl;
	// 获取任意坐标系下的雅可比矩阵，相对基座
	Kine_JacobSub(left_arm, joint_feedback, 5, jac, NULL, NULL); // dof+1考虑末端附加变换
	// 转换Eigen矩阵
	for (int i = 0; i < 6; i++)
	{
		for (int j = 0; j < 7; j++)
		{
			J(i, j) = jac[i][j];
		}
	}
	std::cout << "J is : " << J << std::endl;
	return 0;
	// 更多相关雅可比矩阵的函数详见KinematicInterface.h
}
static double gaussrand()
{
	/* 简单高斯噪声 (Box-Muller) */
	static int has = 0;
	static double v2;
	if (has)
	{
		has = 0;
		return v2;
	}
	double u1, u2, s = 0.0;
	do
	{
		u1 = (rand() + 1.0) / (RAND_MAX + 2.0);
		u2 = (rand() + 1.0) / (RAND_MAX + 2.0);
		u1 = 2 * u1 - 1;
		u2 = 2 * u2 - 1;
		s = u1 * u1 + u2 * u2;
	} while (s >= 1.0 || s == 0.0);
	double mult = sqrt(-2.0 * log(s) / s);
	v2 = u2 * mult;
	has = 1;
	return u1 * mult;
}
int ServoJointDemo()
{
	// 以左臂5关节为例，测试关节跟随功能
	IMPORTJOINT(R0_PTEST);
	double t = 0;
	double A = 1;
	double f = 0.1;
	double joint_init[7];
	double joint[7];
	robjoint joint_target;
	joint_target.dof = 7;

	RTimer timer;
	initUserTimer(&timer, 0, 50); // 模拟用户摇操作设备更新数据频率50ms
	printf("sss\n");
	// 上电
	int ret = 0;
	ret = RobotPower(0);

	std::cout << "robot0 enable sucess..." << std::endl;
	SETSPEED(sp, 0.1, 0.1);
	MultiMoveA(R0_PTEST, sp, NULL3, 0); // 移动到合适位置
	GetCurrentTargetJoint(joint_init, 0);
	memcpy(joint_target.angle, joint_init, sizeof(joint_init)); // 同步当前位置
	ServoStart(1e-6, 0.2, 0);									// 使用ServoJoint前必须启用
	while (robot_ok())
	{
		userTimer(&timer);
		t = t + 0.05;
		if (t > 10)
		{
			break;
		}
		double pos = A * cos(2 * 3.14 * f * t) - A + joint_init[4] + gaussrand() * 0.01; // 模拟上层规划数据
		joint_target.angle[4] = pos;
		ServoJoint(&joint_target, t, 0);
	}
	ServoEnd(0); // 关闭ServoJoint
	sleep(1);
	std::cout << "move finish..." << std::endl;
	return 0;
}
int ServoCartesianDemo()
{
	// 以左臂为例，测试笛卡尔末端跟随功能，绝对位置
	IMPORTJOINT(R0_PTEST);
	double t = 0;
	double A = 0.05;
	double f = 0.1;
	double joint_init[7];
	GetCurrentTargetJoint(joint_init, 0);
	double joint[7];
	robjoint joint_target;
	joint_target.dof = 7;

	// 上电
	int ret = 0;
	ret = RobotPower(0);

	SETSPEED(sp, 0.1, 0.1);
	MultiMoveA(R0_PTEST, sp, NULL3, 0); // 移动到合适位置

	RTimer timer;
	initUserTimer(&timer, 0, 50);
	robpose pose, pose_target;
	GetCurrentTargetCartesian(NULL, NULL, &pose, 0);
	memcpy(&pose_target, &pose, sizeof(robpose));

	ServoStart(1e-6, 0.7, 0); // 开启伺服
	while (robot_ok())
	{
		userTimer(&timer);
		t = t + 0.05;
		if (t > 10)
		{
			break;
		}
		double pos = A * cos(2 * 3.14 * f * t) - A + pose.xyz[0] + gaussrand() * 0.003; // 模拟上层规划数据
		pose_target.xyz[0] = pos;
		ServoCartesian(&pose_target, t, NULL, NULL, 0);
	}
	ServoEnd(0); // 关闭伺服
	sleep(1);
	std::cout << "move finish..." << std::endl;
	return 0;
}
int ServoCartesianOffBaseDemo()
{
	// 以左臂为例，测试笛卡尔末端增量跟随功能,相对基坐标系
	IMPORTJOINT(R0_PTEST);
	double t = 0;
	double A = 0.001;
	double f = 0.1;
	double joint_init[7];
	GetCurrentTargetJoint(joint_init, 0);
	double joint[7];
	robjoint joint_target;
	joint_target.dof = 7;

	// 上电
	int ret = 0;
	ret = RobotPower(0);

	SETSPEED(sp, 0.1, 0.1);
	MultiMoveA(R0_PTEST, sp, NULL3, 0); // 移动到合适位置

	RTimer timer;
	initUserTimer(&timer, 0, 50);
	robpose pose, pose_target, pos_target_rel;
	// GetCurrentTargetCartesian(NULL, NULL, &pose, 0);
	memcpy(&pose_target, &pose, sizeof(robpose));
	ServoStart(1e-6, 0.5, 0); // 开启伺服
	while (robot_ok())
	{
		userTimer(&timer);
		t = t + 0.05;
		if (t > 10)
		{
			break;
		}
		double pos = A * cos(2 * 3.14 * f * t) - A + gaussrand() * 0.003; // 模拟上层规划数据
		GetCurrentTargetCartesian(NULL, NULL, &pose, 0);
		pose_target = Offs(&pose, pos, 0, 0, 0, 0, 0); // 在当前位置叠加X向增量，相对基坐标系
		ServoCartesian(&pose_target, t, NULL, NULL, 0);
	}
	ServoEnd(0); // 关闭伺服
	sleep(1);
	std::cout << "move finish..." << std::endl;
	return 0;
}
int ServoCartesianOffEndDemo()
{
	// 以左臂为例，测试笛卡尔末端增量跟随功能,相对末端坐标系
	IMPORTJOINT(R0_PTEST);
	double t = 0;
	double A = 0.001;
	double f = 0.1;
	double joint_init[7];
	GetCurrentTargetJoint(joint_init, 0);
	double joint[7];
	robjoint joint_target;
	joint_target.dof = 7;

	// 上电
	int ret = 0;
	ret = RobotPower(0);

	SETSPEED(sp, 0.1, 0.1);
	MultiMoveA(R0_PTEST, sp, NULL3, 0); // 移动到合适位置

	RTimer timer;
	initUserTimer(&timer, 0, 50);
	robpose pose, pose_target, pos_target_rel;
	// GetCurrentTargetCartesian(NULL, NULL, &pose, 0);
	memcpy(&pose_target, &pose, sizeof(robpose));
	ServoStart(1e-6, 0.5, 0); // 开启伺服
	while (robot_ok())
	{
		userTimer(&timer);
		t = t + 0.05;
		if (t > 10)
		{
			break;
		}
		double pos = A * cos(2 * 3.14 * f * t) - A + gaussrand() * 0.003; // 模拟上层规划数据
		GetCurrentTargetCartesian(NULL, NULL, &pose, 0);
		pose_target = OffsRel(&pose, pos, 0, 0, 0, 0, 0); // 在当前位置叠加X向增量，相对末端坐标系
		ServoCartesian(&pose_target, t, NULL, NULL, 0);
	}
	ServoEnd(0); // 关闭伺服
	sleep(1);
	std::cout << "move finish..." << std::endl;
	return 0;
}
int TimerDemo()
{
	auto timer = CreateUserTimer();
	while (true)
	{
		timer->Wait();
		// timer->WaitTimes(10); 10 times the bus cycle;
	}
}