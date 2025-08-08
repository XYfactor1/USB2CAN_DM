// # Copyright (c) 2023-2025 TANGAIR
// # SPDX-License-Identifier: Apache-2.0
#ifndef __Tangair_usb2can__
#define __Tangair_usb2can__

#include <assert.h>
#include <chrono>
#include <cmath>
#include <iostream>
#include <stdio.h>
#include <signal.h>
#include <string>
#include <sys/mman.h>
#include <sys/timerfd.h>
#include <thread>
#include <sched.h>
#include <unistd.h>
#include "usb_can.h"
#include "joystick.h"
#include "trajectory_generator.hpp"
#include <vector>
#include <tuple>
#include <atomic>

// 辅助函数
float uint_to_float(int x_int, float x_min, float x_max, int bits);
int float_to_uint(float x, float x_min, float x_max, int bits);

// 达妙其他电机参数    {P, V, T}
//  {12.5, 30, 10 }, // DM4310
//  {12.5, 50, 10 }, // DM4310_48V
//  {12.5, 8, 28 },  // DM4340
//  {12.5, 10, 28 }, // DM4340_48V
//  {12.5, 45, 20 }, // DM6006
//  {12.5, 45, 40 }, // DM8006
//  {12.5, 45, 54 }, // DM8009
//  {12.5,25,  200}, // DM10010L
//  {12.5,20, 200},  // DM10010
//  {12.5,280,1},    // DMH3510
//  {12.5,45,10},    // DMH6215
//  {12.5,45,10}     // DMG6220

// 达妙电机,此处为DM10010L参数
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -25.0f
#define V_MAX 25.0f
#define T_MIN -200.0f
#define T_MAX 200.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f

#define PI (3.1415926f)

typedef struct
{
	uint16_t id;

	float position;
	float speed;
	float kp;
	float kd;
	float torque;

	// 位置限制
	float max_position;
	float min_position;

} Motor_CAN_Send_Struct;

typedef struct
{
	uint16_t ERR;

	uint16_t current_position;	 //
	uint16_t current_speed;		 //
	uint16_t current_torque;	 //
	uint16_t current_temp_MOS;	 //
	uint16_t current_temp_Rotor; //

	float current_position_f; //
	float current_speed_f;	  //
	float current_torque_f;	  //

} Motor_CAN_Recieve_Struct;

typedef struct
{
	Motor_CAN_Send_Struct ID_1_motor_send, ID_2_motor_send, ID_3_motor_send, ID_4_motor_send, ID_5_motor_send;
	Motor_CAN_Recieve_Struct ID_1_motor_recieve, ID_2_motor_recieve, ID_3_motor_recieve, ID_4_motor_recieve, ID_5_motor_recieve;

} USB2CAN_CAN_Bus_Struct;

enum ArmState
{
	ARM_IDLE,
	ARM_RAISE,
	ARM_BACKSWING,
	ARM_THROW,
	ARM_RECOVER
};

class Tangair_usb2can
{
public:
	bool all_thread_done_;
	bool running_;

	void Spin();

	Tangair_usb2can();

	~Tangair_usb2can();

	// CAN设备0
	int USB2CAN0_;
	std::thread _CAN_RX_device_0_thread;
	void CAN_RX_device_0_thread();

	// CAN设备1
	int USB2CAN1_;
	std::thread _CAN_RX_device_1_thread;
	void CAN_RX_device_1_thread();

	int can_dev0_rx_count;
	int can_dev0_rx_count_thread;
	int can_dev1_rx_count;
	int can_dev1_rx_count_thread;

	// can发送测试线程
	std::thread _CAN_TX_test_thread;
	// can发送测试线程函数
	void CAN_TX_test_thread();

	int speed_input;
	// can键盘输入线程
	std::thread _keyborad_input;
	// can发送测试线程函数
	void keyborad_input();

	// void arm_control_thread();
	/*****************************************************************************************************/
	/*********************************       ***电机相关***      ***********************************************/
	/*****************************************************************************************************/
	// 电机基本操作变量
	FrameInfo txMsg_CAN = {
		.canID = 0,
		.frameType = STANDARD,
		.dataLength = 8,
	};

	uint8_t Data_CAN[8];
	Motor_CAN_Send_Struct Motor_Data_Single;

	// 接收暂存
	Motor_CAN_Recieve_Struct CAN_DEV0_RX;
	Motor_CAN_Recieve_Struct CAN_DEV1_RX;

	// 腿部数据
	USB2CAN_CAN_Bus_Struct USB2CAN0_CAN_Bus_1; // 模块0，can1
	USB2CAN_CAN_Bus_Struct USB2CAN0_CAN_Bus_2; // 模块0，can2
	USB2CAN_CAN_Bus_Struct USB2CAN1_CAN_Bus_1; // 模块1，can1
	USB2CAN_CAN_Bus_Struct USB2CAN1_CAN_Bus_2; // 模块1，can2

	// 初始化
	void USB2CAN_CAN_Bus_inti_set(USB2CAN_CAN_Bus_Struct *Leg_Data);
	void USB2CAN_CAN_Bus_Init();

	void Motor_Enable(int32_t dev, uint8_t channel, Motor_CAN_Send_Struct *Motor_Data);

	void Motor_Disable(int32_t dev, uint8_t channel, Motor_CAN_Send_Struct *Motor_Data);

	void Motor_Zore(int32_t dev, uint8_t channel, Motor_CAN_Send_Struct *Motor_Data);

	void CAN_Send_Control(int32_t dev, uint8_t channel, Motor_CAN_Send_Struct *Motor_Data);

	void CAN_Send_Control_Pos(int32_t dev, uint8_t channel, Motor_CAN_Send_Struct *Motor_Data);

	void Motor_Passive_SET(int32_t dev, uint8_t channel, Motor_CAN_Send_Struct *Motor_Data);

	void ENABLE_ALL_MOTOR(int delay_us);

	void DISABLE_ALL_MOTOR(int delay_us);

	void ZERO_ALL_MOTOR(int delay_us);

	void PASSIVE_ALL_MOTOR(int delay_us);

	void CAN_TX_ALL_MOTOR(int delay_us);

	void pre_shoot(void);

	void shoot(void);

	void pre_dribble(void);

	void pre_dribble1(void);

	void dribble(void);

	void dribble1(void);

	void SASTest2();

	// // === 添加机械臂控制相关成员 ===
	// ArmState current_state = ARM_IDLE;
	// double state_start_time = 0.0;

	// // 关节数据（从电机反馈更新）
	// std::array<double, 5> joint_pos = {0}; // 当前关节角度
	// std::array<double, 5> joint_vel = {0}; // 当前关节速度

	// // 控制参数
	// std::array<double, 5> kp_pos = {50.0, 50.0, 40.0, 30.0, 10.0};
	// std::array<double, 5> kd_pos = {1.0, 1.0, 0.8, 0.6, 0.2};

	// // 目标动作
	// std::array<double, 5> raise_pose = {0.3, -0.8, 1.0, -1.2, 0.0};
	// std::array<double, 5> backswing_pose = {0.2, -1.0, 0.8, -1.5, 0.0};
	// std::array<double, 5> recover_pose = {0.0, 0.0, 0.0, 0.0, 0.0};
	// std::array<double, 5> throw_torque_cmd = {1.0, -2.0, 3.0, -3.5, 0.0};

	// // 机械臂控制函数
	// void update_arm_control(double current_time, double dt);
	// void change_state(ArmState new_state);

	// 添加轨迹控制方法

private:
	std::atomic<char> mode{0};

	using TrajectoryPoint = trajectory_generator::TrajectoryPoint;

	std::atomic<bool> trajectory_running_{false};
	std::vector<std::vector<TrajectoryPoint>> joint_trajectories_;
	size_t current_trajectory_step_{0};
	size_t trajectory_total_steps_{0};
	std::chrono::time_point<std::chrono::system_clock> trajectory_start_time_;
	// 关节的kp/kd值数组
	std::vector<double> joint_kp_ = {0.0, 480.0, 200.0, 80.0, 50.0}; // 默认kp值
	std::vector<double> joint_kd_ = {0.0, 4.0, 2.0, 3.0, 3.0};		 // 默认kd值
	void start_trajectory(const std::vector<double> &target_positions, double duration, const std::vector<double> &kp = {}, const std::vector<double> &kd = {});
	void update_trajectory_control();

	JoystickController joystick_; // 手柄控制对象
	void handle_joystick_input(int button, bool pressed);
};

#endif

// # Copyright (c) 2023-2025 TANGAIR
// # SPDX-License-Identifier: Apache-2.0
