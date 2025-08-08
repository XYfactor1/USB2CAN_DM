// # Copyright (c) 2023-2025 TANGAIR
// # SPDX-License-Identifier: Apache-2.0
#include "Tangair_usb2can.h"
#include "trajectory_generator.hpp"
#include <cfloat>
#include <algorithm>
#include <cmath>
#include <limits>
#include <iostream>

using namespace trajectory_generator;

// 主函数循环
void Tangair_usb2can::Spin()
{
    while (all_thread_done_ != true)
    {
        sleep(1); // 延时1s
    }
    printf("~ ALL Exit ~\n");
}

/// @brief 构造函数，初始化
/// @return
Tangair_usb2can::Tangair_usb2can()
{

    running_ = true;
    all_thread_done_ = false;

    USB2CAN0_ = openUSBCAN("/dev/USB2CAN0");
    if (USB2CAN0_ == -1)
        std::cout << std::endl
                  << "USB2CAN0 open INcorrect!!!" << std::endl;
    else
        std::cout << std::endl
                  << "USB2CAN0 opened ,num=" << USB2CAN0_ << std::endl;

    USB2CAN1_ = openUSBCAN("/dev/USB2CAN1");
    if (USB2CAN1_ == -1)
        std::cout << std::endl
                  << "USB2CAN1 open INcorrect!!!" << std::endl;
    else
        std::cout << std::endl
                  << "USB2CAN1 opened ,num=" << USB2CAN1_ << std::endl;

    // 电机ID配置
    USB2CAN_CAN_Bus_Init();

    // 启动成功
    std::cout << std::endl
              << "USB2CAN   NODE INIT__OK   by TANGAIR" << std::endl
              << std::endl
              << std::endl;

    // 创建CAN接收线程，设备1
    _CAN_RX_device_0_thread = std::thread(&Tangair_usb2can::CAN_RX_device_0_thread, this);

    // 创建CAN接收线程，设备2
    _CAN_RX_device_1_thread = std::thread(&Tangair_usb2can::CAN_RX_device_1_thread, this);

    // can 测试线程
    _CAN_TX_test_thread = std::thread(&Tangair_usb2can::CAN_TX_test_thread, this);

    // 键盘输入线程
    _keyborad_input = std::thread(&Tangair_usb2can::keyborad_input, this);

    // 初始化游戏手柄
    if (joystick_.initialize())
    {
        // 设置按钮回调
        joystick_.setButtonCallback(
            std::bind(&Tangair_usb2can::handle_joystick_input, this,
                      std::placeholders::_1, std::placeholders::_2));
        joystick_.start();
    }
}

/// @brief 析构函数
Tangair_usb2can::~Tangair_usb2can()
{

    running_ = false;

    /*注销线程*/

    // can接收设备0
    _CAN_RX_device_0_thread.join();
    // can接收设备1
    _CAN_RX_device_1_thread.join();

    // can发送测试线程
    _CAN_TX_test_thread.join();
    // 键盘改速度
    _keyborad_input.join();

    // 失能电机
    DISABLE_ALL_MOTOR(200);

    // 关闭设备
    closeUSBCAN(USB2CAN0_);
    closeUSBCAN(USB2CAN1_);

    all_thread_done_ = true;
}

/// @brief can设备0，接收线程函数
void Tangair_usb2can::CAN_RX_device_0_thread()
{
    can_dev0_rx_count = 0;
    can_dev0_rx_count_thread = 0;
    while (running_)
    {

        uint8_t channel;
        FrameInfo info_rx;
        uint8_t data_rx[8] = {0};

        can_dev0_rx_count_thread++;

        // 阻塞1s接收
        int recieve_re = readUSBCAN(USB2CAN0_, &channel, &info_rx, data_rx, 1e6);

        // 接收到数据
        if (recieve_re != -1)
        {
            can_dev0_rx_count++;
            // 解码
            CAN_DEV0_RX.ERR = data_rx[0] >> 4 & 0X0F;

            CAN_DEV0_RX.current_position = (data_rx[1] << 8) | data_rx[2];       // 电机位置数据
            CAN_DEV0_RX.current_speed = (data_rx[3] << 4) | (data_rx[4] >> 4);   // 电机速度数据
            CAN_DEV0_RX.current_torque = ((data_rx[4] & 0xF) << 8) | data_rx[5]; // 电机扭矩数据
            CAN_DEV0_RX.current_temp_MOS = data_rx[6];
            CAN_DEV0_RX.current_temp_Rotor = data_rx[7];
            // 转换
            CAN_DEV0_RX.current_position_f = uint_to_float(CAN_DEV0_RX.current_position, (P_MIN), (P_MAX), 16);
            CAN_DEV0_RX.current_speed_f = uint_to_float(CAN_DEV0_RX.current_speed, (V_MIN), (V_MAX), 12);
            CAN_DEV0_RX.current_torque_f = uint_to_float(CAN_DEV0_RX.current_torque, (T_MIN), (T_MAX), 12);

            if (channel == 1) // 模块0，can1
            {
                switch (info_rx.canID)
                {
                case 0X11:
                {
                    USB2CAN0_CAN_Bus_1.ID_1_motor_recieve = CAN_DEV0_RX;

                    break;
                }
                case 0X12:
                {
                    USB2CAN0_CAN_Bus_1.ID_2_motor_recieve = CAN_DEV0_RX;

                    break;
                }
                case 0X13:
                {
                    USB2CAN0_CAN_Bus_1.ID_3_motor_recieve = CAN_DEV0_RX;

                    break;
                }

                case 0X14:
                {
                    USB2CAN0_CAN_Bus_1.ID_4_motor_recieve = CAN_DEV0_RX;

                    break;
                }
                case 0X15:
                {
                    USB2CAN0_CAN_Bus_1.ID_5_motor_recieve = CAN_DEV0_RX;

                    break;
                }

                default:
                    break;
                }
            }
            else if (channel == 2) // 模块0，can2
            {
                switch (info_rx.canID)
                {
                case 0X11:
                {
                    USB2CAN0_CAN_Bus_2.ID_1_motor_recieve = CAN_DEV0_RX;

                    break;
                }
                case 0X12:
                {
                    USB2CAN0_CAN_Bus_2.ID_2_motor_recieve = CAN_DEV0_RX;

                    break;
                }
                case 0X13:
                {
                    USB2CAN0_CAN_Bus_2.ID_3_motor_recieve = CAN_DEV0_RX;

                    break;
                }

                case 0X14:
                {
                    USB2CAN0_CAN_Bus_2.ID_4_motor_recieve = CAN_DEV0_RX;

                    break;
                }

                case 0X15:
                {
                    USB2CAN0_CAN_Bus_2.ID_5_motor_recieve = CAN_DEV0_RX;

                    break;
                }

                default:
                    break;
                }

                // uint8_t motor_id = info_rx.canID & 0x0F; // 提取电机ID
                // if (motor_id >= 1 && motor_id <= 5)
                // {
                //     update_joint_feedback(motor_id,
                //                           CAN_DEV0_RX.current_position_f,
                //                           CAN_DEV0_RX.current_speed_f);
                // }
            }
        }
    }
    std::cout << "CAN_RX_device_0_thread  Exit~~" << std::endl;
}

// // 添加机械臂控制线程
// void Tangair_usb2can::arm_control_thread()
// {
//     auto last_time = std::chrono::high_resolution_clock::now();
//
//     while (running_)
//     {
//         auto now = std::chrono::high_resolution_clock::now();
//         double dt = std::chrono::duration<double>(now - last_time).count();
//         double current_time = std::chrono::duration<double>(
//                                   std::chrono::system_clock::now().time_since_epoch())
//                                   .count();
//
//         update_arm_control(current_time, dt);
//         last_time = now;
//
//         std::this_thread::sleep_for(std::chrono::milliseconds(5)); // 200Hz
//     }
// }

/// @brief can设备1，接收线程函数
void Tangair_usb2can::CAN_RX_device_1_thread()
{
    can_dev1_rx_count = 0;
    can_dev1_rx_count_thread = 0;
    while (running_)
    {

        uint8_t channel;
        FrameInfo info_rx;
        uint8_t data_rx[8] = {0};

        // 接收有可能是CAN1，也有可能是CAN2，使用if进行分类
        int recieve_re = readUSBCAN(USB2CAN1_, &channel, &info_rx, data_rx, 1e6);

        can_dev1_rx_count_thread++;
        // 接收到数据
        if (recieve_re != -1)
        {
            can_dev1_rx_count++;
            // 解码
            CAN_DEV1_RX.ERR = data_rx[0] >> 4 & 0X0F;

            CAN_DEV1_RX.current_position = (data_rx[1] << 8) | data_rx[2];       // 电机位置数据
            CAN_DEV1_RX.current_speed = (data_rx[3] << 4) | (data_rx[4] >> 4);   // 电机速度数据
            CAN_DEV1_RX.current_torque = ((data_rx[4] & 0xF) << 8) | data_rx[5]; // 电机扭矩数据
            CAN_DEV1_RX.current_temp_MOS = data_rx[6];
            CAN_DEV1_RX.current_temp_Rotor = data_rx[7];

            // 转换
            CAN_DEV1_RX.current_position_f = uint_to_float(CAN_DEV1_RX.current_position, (P_MIN), (P_MAX), 16);
            CAN_DEV1_RX.current_speed_f = uint_to_float(CAN_DEV1_RX.current_speed, (V_MIN), (V_MAX), 12);
            CAN_DEV1_RX.current_torque_f = uint_to_float(CAN_DEV1_RX.current_torque, (T_MIN), (T_MAX), 12);

            if (channel == 1) // 模块1，can1
            {
                switch (info_rx.canID)
                {
                case 0X11:
                {
                    USB2CAN1_CAN_Bus_1.ID_1_motor_recieve = CAN_DEV1_RX;
                    break;
                }
                case 0X12:
                {
                    USB2CAN1_CAN_Bus_1.ID_2_motor_recieve = CAN_DEV1_RX;

                    break;
                }
                case 0X13:
                {
                    USB2CAN1_CAN_Bus_1.ID_3_motor_recieve = CAN_DEV1_RX;

                    break;
                }
                case 0X14:
                {
                    USB2CAN1_CAN_Bus_1.ID_4_motor_recieve = CAN_DEV1_RX;

                    break;
                }
                case 0X15:
                {
                    USB2CAN1_CAN_Bus_1.ID_5_motor_recieve = CAN_DEV1_RX;

                    break;
                }

                default:
                    break;
                }
            }
            else if (channel == 2) // 模块1，can2
            {
                switch (info_rx.canID)
                {
                case 0X11:
                {
                    USB2CAN1_CAN_Bus_2.ID_1_motor_recieve = CAN_DEV1_RX;

                    break;
                }
                case 0X12:
                {
                    USB2CAN1_CAN_Bus_2.ID_2_motor_recieve = CAN_DEV1_RX;

                    break;
                }
                case 0X13:
                {
                    USB2CAN1_CAN_Bus_2.ID_3_motor_recieve = CAN_DEV1_RX;

                    break;
                }
                case 0X14:
                {
                    USB2CAN1_CAN_Bus_2.ID_4_motor_recieve = CAN_DEV1_RX;

                    break;
                }
                case 0X15:
                {
                    USB2CAN1_CAN_Bus_2.ID_5_motor_recieve = CAN_DEV1_RX;

                    break;
                }

                default:
                    break;
                }
            }
        }
    }
    std::cout << "CAN_RX_device_1_thread  Exit~~" << std::endl;
}

void Tangair_usb2can::handle_joystick_input(int button, bool pressed)
{
    // 只处理按下事件
    if (!pressed)
        return;

    // 按钮映射到模式
    switch (button)
    {
    case 0: // A按钮
        mode = '1';
        break;
    case 1: // B按钮
        mode = '2';
        break;
    case 2: // X按钮
        mode = '3';
        break;
    case 3: // Y按钮
        mode = '4';
        break;
    case 4: // LB按钮
        mode = '5';
        break;
    case 5: // RB按钮
        mode = '6';
        break;
    }
}
/*****************************************************************************************************/
/*********************************       ***测试相关***      ***********************************************/
/*****************************************************************************************************/
// can发送测试线程函数
void Tangair_usb2can::CAN_TX_test_thread()
{
    // 发送计数
    uint32_t tx_count = 0;

    // 使能所有电机
    ENABLE_ALL_MOTOR(200);
    // ZERO_ALL_MOTOR(200);
    Motor_Zore(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_1_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(200)); // 单位us
    Motor_Zore(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_2_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(200)); // 单位us
    Motor_Zore(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_3_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(200)); // 单位us
    Motor_Zore(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_4_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(200)); // 单位us
    // Motor_Zore(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_5_motor_send);
    // std::this_thread::sleep_for(std::chrono::microseconds(154)); // 单位us

    while (running_)
    {
        auto state = joystick_.getState();

        if (mode.load())
        {

            // 任务选择（同时处理键盘和手柄命令）
            switch (mode.load())
            {
            case '1': // “X” 键
                pre_shoot();
                break;
            case '2': // “A” 键
                shoot();
                break;
            case '3': // “B” 键
                SASTest2();
                break;
            case '4': // “Y” 键
                pre_dribble();
                break;
            case '5': // “LB” 键
                dribble();
                break;
                // case '6': // “RB” 键
                //     PASSIVE_ALL_MOTOR(100);
                //     break;
                // case '7': // “LT” 键
                //     use_simulation_data = true;
                //     current_simulation_index = 0;
                //     break;
                // case 'W':
                //     forward();
                //     break;
                // case 'X':
                //     back();
                //     break;
                // case 'A':
                //     turn_left();
                //     break;
                // case 'D':
                //     turn_right();
                //     break;
                // case 'S':
                //     stop();
                //     break;

            default:
                break;
            }
            mode = 0; // 重置模式
        }
        // // 更新投篮状态机
        // update_shoot_phase();
        update_trajectory_control();

        // CAN发送,发送频率为1298.7hz,实际间隔约为770us
        CAN_TX_ALL_MOTOR(200);

        // CAN发送计数
        tx_count++;

        std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds> tpMill =
            std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
        time_t tp = tpMill.time_since_epoch().count();

        // 打印数据tp时间ms，1000hz的控制频率的话，1s一次，
        if (tx_count % 1000 == 0)
        {
            std::cout << std::endl;
            // 打印摇杆状态
            std::cout << "\n--- 摇杆状态 ---" << std::endl;
            std::cout << "左摇杆 X: " << state.left_x << ", Y: " << state.left_y << std::endl;
            std::cout << "右摇杆 X: " << state.right_x << ", Y: " << state.right_y << std::endl;

            // 打印按钮状态
            std::cout << "按钮: ";
            for (int i = 0; i < 15; i++)
            {
                if (state.buttons[i])
                {
                    std::cout << i << " ";
                }
            }
            std::cout << std::endl;

            // 打印分割线
            std::cout << "----------------" << std::endl;
            std::cout << std::endl
                      << "USB2CAN0_CAN1——1.current_position_f=  " << USB2CAN0_CAN_Bus_1.ID_1_motor_recieve.current_position_f << "  rad/s" << std::endl
                      << "USB2CAN0_CAN1——2.current_position_f=  " << USB2CAN0_CAN_Bus_1.ID_2_motor_recieve.current_position_f << "  rad/s" << std::endl
                      << "USB2CAN0_CAN1——3.current_position_f=  " << USB2CAN0_CAN_Bus_1.ID_3_motor_recieve.current_position_f << "  rad/s" << std::endl
                      << "USB2CAN0_CAN2——4.current_position_f=  " << USB2CAN0_CAN_Bus_1.ID_4_motor_recieve.current_position_f << "  rad/s" << std::endl
                      << "USB2CAN0_CAN2——5.current_position_f=  " << USB2CAN0_CAN_Bus_1.ID_5_motor_recieve.current_position_f << "  rad/s" << std::endl;
            std::cout << "can_tx_count=" << tx_count << "     " << "can_dev0_rx_count=" << can_dev0_rx_count << "     " << "can_dev1_rx_count=" << can_dev1_rx_count << "     "
                      << "TIME=" << (tp % 1000000) / 1000 << "." << tp % 1000 << "s" << std::endl;
        }
    }

    // 程序终止时的提示信息
    std::cout << "CAN_TX_test_thread  Exit~~" << std::endl;
    std::cout << std::endl
              << "----------------请输入任意数字，按回车，以结束键盘进程  ----------------------" << std::endl
              << std::endl;
}

// 键盘输入线程，可以键盘修改电机速度
void Tangair_usb2can::keyborad_input()
{
    char input_char;
    while (running_)
    {
        std::cin >> input_char;
        mode = input_char;
    }

    while (running_)
    {
        std::cin >> speed_input;
        std::cout << "speed_input=" << speed_input << std::endl;
    }
    std::cout << "keyborad_input_thread  Exit~~" << std::endl;
}

/*****************************************************************************************************/
/*********************************       ***电机相关***      ***********************************************/
/*****************************************************************************************************/

void Tangair_usb2can::USB2CAN_CAN_Bus_inti_set(USB2CAN_CAN_Bus_Struct *CAN_Bus)
{
    CAN_Bus->ID_1_motor_send.id = 0X01;

    CAN_Bus->ID_2_motor_send.id = 0X02;

    CAN_Bus->ID_3_motor_send.id = 0X03;

    CAN_Bus->ID_4_motor_send.id = 0X04;

    CAN_Bus->ID_5_motor_send.id = 0X05;
}

void Tangair_usb2can::USB2CAN_CAN_Bus_Init()
{
    USB2CAN_CAN_Bus_inti_set(&USB2CAN0_CAN_Bus_1);
    USB2CAN_CAN_Bus_inti_set(&USB2CAN0_CAN_Bus_2);
    USB2CAN_CAN_Bus_inti_set(&USB2CAN1_CAN_Bus_1);
    USB2CAN_CAN_Bus_inti_set(&USB2CAN1_CAN_Bus_2);
}

/// @brief 使能
/// @param dev
/// @param channel
/// @param Motor_Data
void Tangair_usb2can::Motor_Enable(int32_t dev, uint8_t channel, Motor_CAN_Send_Struct *Motor_Data)
{
    txMsg_CAN.canID = Motor_Data->id;

    Data_CAN[0] = 0xFF;
    Data_CAN[1] = 0xFF;
    Data_CAN[2] = 0xFF;
    Data_CAN[3] = 0xFF;
    Data_CAN[4] = 0xFF;
    Data_CAN[5] = 0xFF;
    Data_CAN[6] = 0xFF;
    Data_CAN[7] = 0xFC;

    sendUSBCAN(dev, channel, &txMsg_CAN, Data_CAN);
}

/// @brief 电机失能
/// @param dev
/// @param channel
/// @param Motor_Data
void Tangair_usb2can::Motor_Disable(int32_t dev, uint8_t channel, Motor_CAN_Send_Struct *Motor_Data)
{

    txMsg_CAN.canID = Motor_Data->id;

    Data_CAN[0] = 0xFF;
    Data_CAN[1] = 0xFF;
    Data_CAN[2] = 0xFF;
    Data_CAN[3] = 0xFF;
    Data_CAN[4] = 0xFF;
    Data_CAN[5] = 0xFF;
    Data_CAN[6] = 0xFF;
    Data_CAN[7] = 0xFD;

    sendUSBCAN(dev, channel, &txMsg_CAN, Data_CAN);
}

/// @brief 设置零点
/// @param dev
/// @param channel
/// @param Motor_Data
void Tangair_usb2can::Motor_Zore(int32_t dev, uint8_t channel, Motor_CAN_Send_Struct *Motor_Data)
{
    txMsg_CAN.canID = Motor_Data->id;

    Data_CAN[0] = 0xFF;
    Data_CAN[1] = 0xFF;
    Data_CAN[2] = 0xFF;
    Data_CAN[3] = 0xFF;
    Data_CAN[4] = 0xFF;
    Data_CAN[5] = 0xFF;
    Data_CAN[6] = 0xFF;
    Data_CAN[7] = 0xFE;

    sendUSBCAN(dev, channel, &txMsg_CAN, Data_CAN);
}

/// @brief 电机控制MIT
/// @param dev 模块设备号
/// @param channel can1或者can2
/// @param Motor_Data 电机数据
void Tangair_usb2can::CAN_Send_Control(int32_t dev, uint8_t channel, Motor_CAN_Send_Struct *Motor_Data) // 运控�??,CAN1=CAN_TX_MAILBOX0,CAN2=CAN_TX_MAILBOX1
{
    // 运控模式专用的局部变
    FrameInfo txMsg_Control = {
        .canID = Motor_Data->id,
        .frameType = STANDARD,
        .dataLength = 8,
    };
    uint8_t Data_CAN_Control[8];

    // 限制范围
    if (Motor_Data->kp > KP_MAX)
        Motor_Data->kp = KP_MAX;
    else if (Motor_Data->kp < KP_MIN)
        Motor_Data->kp = KP_MIN;
    if (Motor_Data->kd > KD_MAX)
        Motor_Data->kd = KD_MAX;
    else if (Motor_Data->kd < KD_MIN)
        Motor_Data->kd = KD_MIN;
    if (Motor_Data->position > P_MAX)
        Motor_Data->position = P_MAX;
    else if (Motor_Data->position < P_MIN)
        Motor_Data->position = P_MIN;
    if (Motor_Data->speed > V_MAX)
        Motor_Data->speed = V_MAX;
    else if (Motor_Data->speed < V_MIN)
        Motor_Data->speed = V_MIN;
    if (Motor_Data->torque > T_MAX)
        Motor_Data->torque = T_MAX;
    else if (Motor_Data->torque < T_MIN)
        Motor_Data->torque = T_MIN;

    Data_CAN_Control[0] = float_to_uint(Motor_Data->position, P_MIN, P_MAX, 16) >> 8;                                                                    // 位置�?? 8
    Data_CAN_Control[1] = float_to_uint(Motor_Data->position, P_MIN, P_MAX, 16) & 0xFF;                                                                  // 位置�?? 8
    Data_CAN_Control[2] = float_to_uint(Motor_Data->speed, V_MIN, V_MAX, 12) >> 4;                                                                       // 速度�?? 8 �??
    Data_CAN_Control[3] = ((float_to_uint(Motor_Data->speed, V_MIN, V_MAX, 12) & 0xF) << 4) | (float_to_uint(Motor_Data->kp, KP_MIN, KP_MAX, 12) >> 8);  // 速度�?? 4 �?? KP �?? 4 �??
    Data_CAN_Control[4] = float_to_uint(Motor_Data->kp, KP_MIN, KP_MAX, 12) & 0xFF;                                                                      // KP �?? 8 �??
    Data_CAN_Control[5] = float_to_uint(Motor_Data->kd, KD_MIN, KD_MAX, 12) >> 4;                                                                        // Kd �?? 8 �??
    Data_CAN_Control[6] = ((float_to_uint(Motor_Data->kd, KD_MIN, KD_MAX, 12) & 0xF) << 4) | (float_to_uint(Motor_Data->torque, T_MIN, T_MAX, 12) >> 8); // KP �?? 4 位扭矩高 4 �??
    Data_CAN_Control[7] = float_to_uint(Motor_Data->torque, T_MIN, T_MAX, 12) & 0xFF;                                                                    // 扭矩�?? 8

    int ret = sendUSBCAN(dev, channel, &txMsg_Control, Data_CAN_Control);
}

/* @brief:      	pos_speed_ctrl: 位置速度控制函数
 * @param[in]:   hcan:			指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
 * @param[in]:   motor_id:	电机ID，指定目标电机
 * @param[in]:   vel:			速度给定值
 * @retval:     	void
 * @details:    	通过CAN总线向电机发送位置速度控制命令
 */
void Tangair_usb2can::CAN_Send_Control_Pos(int32_t dev, uint8_t channel, Motor_CAN_Send_Struct *Motor_Data) // 运控�??,CAN1=CAN_TX_MAILBOX0,CAN2=CAN_TX_MAILBOX1
{
    uint16_t frame_id = 0x100 + Motor_Data->id;

    FrameInfo txMsg_Control = {
        .canID = frame_id,
        .frameType = STANDARD,
        .dataLength = 8,
    };
    uint8_t Data_CAN_Control[8] = {0};

    if (Motor_Data->position > P_MAX)
        Motor_Data->position = P_MAX;
    else if (Motor_Data->position < P_MIN)
        Motor_Data->position = P_MIN;
    if (Motor_Data->speed > V_MAX)
        Motor_Data->speed = V_MAX;
    else if (Motor_Data->speed < V_MIN)
        Motor_Data->speed = V_MIN;

    float p_des = Motor_Data->position;
    float v_des = Motor_Data->speed;

    uint8_t *p_bytes = (uint8_t *)&p_des;
    uint8_t *v_bytes = (uint8_t *)&v_des;

    Data_CAN_Control[0] = p_bytes[0]; // P_des低字节
    Data_CAN_Control[1] = p_bytes[1];
    Data_CAN_Control[2] = p_bytes[2];
    Data_CAN_Control[3] = p_bytes[3]; // P_des高字节

    Data_CAN_Control[4] = v_bytes[0]; // V_des低字节
    Data_CAN_Control[5] = v_bytes[1];
    Data_CAN_Control[6] = v_bytes[2];
    Data_CAN_Control[7] = v_bytes[3]; // V_des高字节

    int ret = sendUSBCAN(dev, channel, &txMsg_Control, Data_CAN_Control);
}

/// @brief 电机阻尼模式
/// @param dev
/// @param channel
/// @param Motor_Data
void Tangair_usb2can::Motor_Passive_SET(int32_t dev, uint8_t channel, Motor_CAN_Send_Struct *Motor_Data)
{
    Motor_Data->speed = 0;
    Motor_Data->kp = 0;
    Motor_Data->kd = 2.0;
    Motor_Data->torque = 0;

    CAN_Send_Control(dev, channel, Motor_Data);
}

void Tangair_usb2can::ENABLE_ALL_MOTOR(int delay_us)
{
    Motor_Enable(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_1_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    Motor_Enable(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_2_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    Motor_Enable(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_3_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    Motor_Enable(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_4_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    Motor_Enable(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_5_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    Motor_Enable(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_1_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    Motor_Enable(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_2_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    Motor_Enable(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_3_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    Motor_Enable(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_4_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    Motor_Enable(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_5_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
}

void Tangair_usb2can::Tangair_usb2can::DISABLE_ALL_MOTOR(int delay_us)
{
    Motor_Disable(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_1_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    Motor_Disable(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_2_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    Motor_Disable(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_3_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    Motor_Disable(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_4_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    Motor_Disable(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_5_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    Motor_Disable(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_1_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    Motor_Disable(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_2_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    Motor_Disable(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_3_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    Motor_Disable(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_4_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    Motor_Disable(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_5_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
}

void Tangair_usb2can::ZERO_ALL_MOTOR(int delay_us)
{

    Motor_Zore(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_1_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    Motor_Zore(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_2_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    Motor_Zore(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_3_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    Motor_Zore(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_4_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    Motor_Zore(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_5_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    Motor_Zore(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_1_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    Motor_Zore(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_2_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    Motor_Zore(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_3_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    Motor_Zore(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_4_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    Motor_Zore(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_5_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
}

void Tangair_usb2can::PASSIVE_ALL_MOTOR(int delay_us)
{

    Motor_Passive_SET(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_1_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    Motor_Passive_SET(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_2_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    Motor_Passive_SET(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_3_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    Motor_Passive_SET(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_4_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    Motor_Passive_SET(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_5_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
}

/// @brief can控制发送，12个电机的数据
// 目前能达到1000hz的控制频率--------3000hz的总线发送频率---------同一路can的发送间隔在300us
void Tangair_usb2can::CAN_TX_ALL_MOTOR(int delay_us)
{
    auto t = std::chrono::high_resolution_clock::now(); // 这一句耗时50us
    CAN_Send_Control_Pos(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_1_motor_send);
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);
    CAN_Send_Control(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_2_motor_send);
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);
    CAN_Send_Control(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_3_motor_send);
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);
    CAN_Send_Control(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_4_motor_send);
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);
    CAN_Send_Control(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_5_motor_send);
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);
}

/// @brief 辅助函数
int float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    if (x > x_max)
        x = x_max;
    else if (x < x_min)
        x = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

void Tangair_usb2can::start_trajectory(const std::vector<double> &target_positions, double duration,
                                       const std::vector<double> &kp, const std::vector<double> &kd)
{
    // 确保有5个关节
    if (target_positions.size() != 5)
    {
        std::cerr << "错误: 需要5个目标位置" << std::endl;
        return;
    }

    if (!kp.empty() && kp.size() == 5)
    {
        joint_kp_ = kp;
    }
    if (!kd.empty() && kd.size() == 5)
    {
        joint_kd_ = kd;
    }
    // 获取当前关节位置
    std::vector<double> current_positions = {
        USB2CAN0_CAN_Bus_1.ID_1_motor_recieve.current_position_f,
        USB2CAN0_CAN_Bus_1.ID_2_motor_recieve.current_position_f,
        USB2CAN0_CAN_Bus_1.ID_3_motor_recieve.current_position_f,
        USB2CAN0_CAN_Bus_1.ID_4_motor_recieve.current_position_f,
        USB2CAN0_CAN_Bus_1.ID_5_motor_recieve.current_position_f};

    // 清除旧轨迹
    joint_trajectories_.clear();

    // 为每个关节生成新轨迹
    const double dt = 0.001; // 1ms控制周期
    for (int i = 0; i < 5; ++i)
    {
        auto traj = trajectory_generator::TrajectoryGenerator::generateCubicTrajectory(double(current_positions[i]),
                                                                                       double(target_positions[i]),
                                                                                       duration, dt);
        joint_trajectories_.push_back(traj);
    }

    // 设置轨迹参数
    current_trajectory_step_ = 0;
    trajectory_total_steps_ = joint_trajectories_[0].size();
    trajectory_start_time_ = std::chrono::system_clock::now();
    trajectory_running_ = true;

    std::cout << "轨迹开始: 从当前位置到目标位置" << std::endl;
    std::cout << "KP: [" << joint_kp_[0] << ", " << joint_kp_[1] << ", "
              << joint_kp_[2] << ", " << joint_kp_[3] << ", " << joint_kp_[4] << "]" << std::endl;
    std::cout << "KD: [" << joint_kd_[0] << ", " << joint_kd_[1] << ", "
              << joint_kd_[2] << ", " << joint_kd_[3] << ", " << joint_kd_[4] << "]" << std::endl;

    // 添加：清除旧控制模式
    mode.store(0); // 重置键盘模式
    trajectory_running_ = true;
}
void Tangair_usb2can::update_trajectory_control()
{
    if (!trajectory_running_)
        return;

    // 计算当前轨迹点索引
    auto now = std::chrono::system_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - trajectory_start_time_).count();
    size_t step = static_cast<size_t>(elapsed);

    // 检查是否完成轨迹
    if (step >= trajectory_total_steps_)
    {
        trajectory_running_ = false;
        std::cout << "轨迹完成" << std::endl;
        return;
    }

    // 设置每个关节的当前位置和速度
    for (int i = 0; i < 5; ++i)
    {
        const auto &point = joint_trajectories_[i][step];

        // 根据关节ID设置到对应的电机
        switch (i)
        {
        case 0:
            USB2CAN0_CAN_Bus_1.ID_1_motor_send.position = point.position;
            USB2CAN0_CAN_Bus_1.ID_1_motor_send.speed = point.velocity;
            USB2CAN0_CAN_Bus_1.ID_1_motor_send.kp = joint_kp_[i];
            USB2CAN0_CAN_Bus_1.ID_1_motor_send.kd = joint_kd_[i];
            USB2CAN0_CAN_Bus_1.ID_1_motor_send.torque = 0.0;
            break;
        case 1:
            USB2CAN0_CAN_Bus_1.ID_2_motor_send.position = point.position;
            USB2CAN0_CAN_Bus_1.ID_2_motor_send.speed = point.velocity;
            USB2CAN0_CAN_Bus_1.ID_2_motor_send.kp = joint_kp_[i];
            USB2CAN0_CAN_Bus_1.ID_2_motor_send.kd = joint_kd_[i];
            USB2CAN0_CAN_Bus_1.ID_2_motor_send.torque = -1.2;
            break;
        case 2:
            USB2CAN0_CAN_Bus_1.ID_3_motor_send.position = point.position;
            USB2CAN0_CAN_Bus_1.ID_3_motor_send.speed = point.velocity;
            USB2CAN0_CAN_Bus_1.ID_3_motor_send.kp = joint_kp_[i];
            USB2CAN0_CAN_Bus_1.ID_3_motor_send.kd = joint_kd_[i];
            USB2CAN0_CAN_Bus_1.ID_3_motor_send.torque = 0.5;
            break;
        case 3:
            USB2CAN0_CAN_Bus_1.ID_4_motor_send.position = point.position;
            USB2CAN0_CAN_Bus_1.ID_4_motor_send.speed = point.velocity;
            USB2CAN0_CAN_Bus_1.ID_4_motor_send.kp = joint_kp_[i];
            USB2CAN0_CAN_Bus_1.ID_4_motor_send.kd = joint_kd_[i];
            USB2CAN0_CAN_Bus_1.ID_4_motor_send.torque = 0.3;
            break;
        case 4:
            USB2CAN0_CAN_Bus_1.ID_5_motor_send.position = point.position;
            USB2CAN0_CAN_Bus_1.ID_5_motor_send.speed = point.velocity;
            USB2CAN0_CAN_Bus_1.ID_5_motor_send.kp = joint_kp_[i];
            USB2CAN0_CAN_Bus_1.ID_5_motor_send.kd = joint_kd_[i];
            USB2CAN0_CAN_Bus_1.ID_5_motor_send.torque = 0.0;
            break;
        }
        if (step % 100 == 0)
        { // 每100ms打印一次
            std::cout << "Joint " << i
                      << " | Target: " << point.position
                      << " | Actual: "
                      << USB2CAN0_CAN_Bus_1.ID_2_motor_recieve.current_position_f
                      << " | Error: "
                      << point.position - USB2CAN0_CAN_Bus_1.ID_2_motor_recieve.current_position_f
                      << std::endl;
        }
    }
}

// void Tangair_usb2can::pre_shoot(void)
// {
//     USB2CAN0_CAN_Bus_1.ID_1_motor_send.position = 0.0;
//     USB2CAN0_CAN_Bus_1.ID_1_motor_send.speed = 0.0;
//     USB2CAN0_CAN_Bus_1.ID_1_motor_send.torque = 0.0;
//     USB2CAN0_CAN_Bus_1.ID_1_motor_send.kp = 60.0;
//     USB2CAN0_CAN_Bus_1.ID_1_motor_send.kd = 0.5;
//
//     USB2CAN0_CAN_Bus_1.ID_2_motor_send.position = 1.462;
//     USB2CAN0_CAN_Bus_1.ID_2_motor_send.speed = 0.0;
//     USB2CAN0_CAN_Bus_1.ID_2_motor_send.torque = 0.0;
//     USB2CAN0_CAN_Bus_1.ID_2_motor_send.kp = 20.0;
//     USB2CAN0_CAN_Bus_1.ID_2_motor_send.kd = 1.0;
//
//     USB2CAN0_CAN_Bus_1.ID_3_motor_send.position = 1.66152;
//     USB2CAN0_CAN_Bus_1.ID_3_motor_send.speed = 0.0;
//     USB2CAN0_CAN_Bus_1.ID_3_motor_send.torque = 0.0;
//     USB2CAN0_CAN_Bus_1.ID_3_motor_send.kp = 20.0;
//     USB2CAN0_CAN_Bus_1.ID_3_motor_send.kd = 1;       //
//     USB2CAN0_CAN_Bus_1.ID_4_motor_send.position = -0.0757227;
//     USB2CAN0_CAN_Bus_1.ID_4_motor_send.speed = 0.0;
//     USB2CAN0_CAN_Bus_1.ID_4_motor_send.torque = 0.0;
//     USB2CAN0_CAN_Bus_1.ID_4_motor_send.kp = 10.0;
//     USB2CAN0_CAN_Bus_1.ID_4_motor_send.kd = 1.0;
//
//     USB2CAN0_CAN_Bus_1.ID_5_motor_send.position = -0.0452051;
//     USB2CAN0_CAN_Bus_1.ID_5_motor_send.speed = 0.8;
//     USB2CAN0_CAN_Bus_1.ID_5_motor_send.torque = 0.0;
//     USB2CAN0_CAN_Bus_1.ID_5_motor_send.kp = 0.0;
//     USB2CAN0_CAN_Bus_1.ID_5_motor_send.kd = 0.0;
// }

void Tangair_usb2can::pre_shoot(void)
{
    // 设置拍球动作的kp/kd值
    std::vector<double> kp = {0.0, 480.0, 200.0, 80.0, 50.0};
    std::vector<double> kd = {0.0, 4.0, 2.0, 3.0, 3.0};

    // 使用轨迹控制
    std::vector<double> target_positions = {0.0, 0.0, 0.0, 1.127, -0.35};
    start_trajectory(target_positions, 1.0, kp, kd);
}

void Tangair_usb2can::shoot(void)
{
    USB2CAN0_CAN_Bus_1.ID_1_motor_send.position = 0.0;
    USB2CAN0_CAN_Bus_1.ID_1_motor_send.speed = 20.0;
    // USB2CAN0_CAN_Bus_1.ID_1_motor_send.torque = 0.0;
    // USB2CAN0_CAN_Bus_1.ID_1_motor_send.kp = 60.0;
    // USB2CAN0_CAN_Bus_1.ID_1_motor_send.kd = 0.5;

    USB2CAN0_CAN_Bus_1.ID_2_motor_send.position = -1.483164;
    USB2CAN0_CAN_Bus_1.ID_2_motor_send.speed = 0.0;
    USB2CAN0_CAN_Bus_1.ID_2_motor_send.torque = 0.0;
    USB2CAN0_CAN_Bus_1.ID_2_motor_send.kp = 500.0;
    USB2CAN0_CAN_Bus_1.ID_2_motor_send.kd = 5.0;

    USB2CAN0_CAN_Bus_1.ID_3_motor_send.position = -2.260247;
    USB2CAN0_CAN_Bus_1.ID_3_motor_send.speed = 0.0;
    USB2CAN0_CAN_Bus_1.ID_3_motor_send.torque = 0.0;
    USB2CAN0_CAN_Bus_1.ID_3_motor_send.kp = 500.0;
    USB2CAN0_CAN_Bus_1.ID_3_motor_send.kd = 5;

    USB2CAN0_CAN_Bus_1.ID_4_motor_send.position = 1.12757227;
    USB2CAN0_CAN_Bus_1.ID_4_motor_send.speed = 0.0;
    USB2CAN0_CAN_Bus_1.ID_4_motor_send.torque = 0.0;
    USB2CAN0_CAN_Bus_1.ID_4_motor_send.kp = 80.0;
    USB2CAN0_CAN_Bus_1.ID_4_motor_send.kd = 3.0;

    USB2CAN0_CAN_Bus_1.ID_5_motor_send.position = -0.4;
    USB2CAN0_CAN_Bus_1.ID_5_motor_send.speed = 0.0;
    USB2CAN0_CAN_Bus_1.ID_5_motor_send.torque = 0.0;
    USB2CAN0_CAN_Bus_1.ID_5_motor_send.kp = 50.0;
    USB2CAN0_CAN_Bus_1.ID_5_motor_send.kd = 3.0;
}

void Tangair_usb2can::SASTest2(void)
{
    // int cycle_count = 0;
    // const int max_cycles = 1000;

    pre_dribble();
    auto test1_start = std::chrono::steady_clock::now();
    float test1_duration = 1.1f;
    while (running_)
    {
        auto now = std::chrono::steady_clock::now();
        float elapsed = std::chrono::duration<float>(now - test1_start).count();

        if (elapsed >= test1_duration)
        {
            break;
        }
        CAN_TX_ALL_MOTOR(200);
    }
    pre_dribble1();
    auto test2_start = std::chrono::steady_clock::now();
    float test2_duration = 0.7f;
    while (running_)
    {
        auto now = std::chrono::steady_clock::now();
        float elapsed = std::chrono::duration<float>(now - test2_start).count();
        if (elapsed >= test2_duration)
        {
            break;
        }
        CAN_TX_ALL_MOTOR(200);
    }
    // dribble();
    // auto test3_start = std::chrono::steady_clock::now();
    // float test3_duration = 0.1f;
    // while (running_)
    // {
    //     auto now = std::chrono::steady_clock::now();
    //     float elapsed = std::chrono::duration<float>(now - test3_start).count();
    //     if (elapsed >= test3_duration)
    //     {
    //         break;
    //     }
    //     CAN_TX_ALL_MOTOR(200);
    // }
    // dribble1();
    // auto test4_start = std::chrono::steady_clock::now();
    // float test4_duration = 0.5f;
    // while (running_)
    // {
    //     auto now = std::chrono::steady_clock::now();
    //     float elapsed = std::chrono::duration<float>(now - test4_start).count();
    //     if (elapsed >= test4_duration)
    //     {
    //         break;
    //     }
    //     CAN_TX_ALL_MOTOR(200);
    // }
    // pre_dribble();
    // auto test5_start = std::chrono::steady_clock::now();
    // float test5_duration = 5.0f;
    // while (running_)
    // {
    //     auto now = std::chrono::steady_clock::now();
    //     float elapsed = std::chrono::duration<float>(now - test5_start).count();

    //     if (elapsed >= test5_duration)
    //     {
    //         break;
    //     }
    //     CAN_TX_ALL_MOTOR(200);
    // }
    // cycle_count++;
    // std::cout << "Cycle count: " << cycle_count << "/1000" << std::endl;
    // if (cycle_count >= max_cycles)
    // {
    //     running_ = false;
    //     std::cout << "Reached 100 cycles. Stopping program." << std::endl;
    // }
}

// void Tangair_usb2can::shoot(void)
// {
//     USB2CAN0_CAN_Bus_1.ID_1_motor_send.position = 0.0;
//     USB2CAN0_CAN_Bus_1.ID_1_motor_send.speed = 0.0;
//     USB2CAN0_CAN_Bus_1.ID_1_motor_send.torque = 0.0;
//     USB2CAN0_CAN_Bus_1.ID_1_motor_send.kp = 60.0;
//     USB2CAN0_CAN_Bus_1.ID_1_motor_send.kd = 0.5;

//     USB2CAN0_CAN_Bus_1.ID_2_motor_send.position = 0.718891;
//     USB2CAN0_CAN_Bus_1.ID_2_motor_send.speed = 20;

//     USB2CAN0_CAN_Bus_1.ID_3_motor_send.position = -0.29469;
//     USB2CAN0_CAN_Bus_1.ID_3_motor_send.speed = 20;

//     USB2CAN0_CAN_Bus_1.ID_4_motor_send.position = 0.364882;
//     USB2CAN0_CAN_Bus_1.ID_4_motor_send.speed = 5;

//     USB2CAN0_CAN_Bus_1.ID_5_motor_send.position = -0.1;
//     USB2CAN0_CAN_Bus_1.ID_5_motor_send.speed = 0.8;
// }

/// 拍球
void Tangair_usb2can::pre_dribble(void)
{
    // 设置拍球动作的kp/kd值
    std::vector<double> kp = {0.0, 480.0, 200.0, 80.0, 50.0};
    std::vector<double> kd = {0.0, 4.0, 2.0, 3.0, 3.0};

    // 使用轨迹控制
    std::vector<double> target_positions = {0.0, -2.06, -2.846, 2.41, -0.35};
    start_trajectory(target_positions, 1.0, kp, kd);
}

// void Tangair_usb2can::dribble(void)
// {
//     // 设置运球动作的kp/kd值
//     std::vector<double> kp = {60.0, 50.0, 50.0, 20.0, 0.0};
//     std::vector<double> kd = {0.5, 1.0, 1.0, 1.0, 0.0};

//     // 使用轨迹控制
//     std::vector<double> target_positions = {0.0, 0.267605, -1.58255, 1.41083, -0.0};
//     start_trajectory(target_positions, 0.5, kp, kd);
// }
//     // // 将轨迹点赋值给电机发送结构体
//     USB2CAN0_CAN_Bus_1.ID_1_motor_send.position = 0.0;
//     USB2CAN0_CAN_Bus_1.ID_1_motor_send.speed = 2.0;
//     // USB2CAN0_CAN_Bus_1.ID_1_motor_send.torque = 0.0;
//     // USB2CAN0_CAN_Bus_1.ID_1_motor_send.kp = 60.0;
//     // USB2CAN0_CAN_Bus_1.ID_1_motor_send.kd = 0.5;

//     USB2CAN0_CAN_Bus_1.ID_2_motor_send.position = 1.700;
//     USB2CAN0_CAN_Bus_1.ID_2_motor_send.speed = 0.0;
//     USB2CAN0_CAN_Bus_1.ID_2_motor_send.torque = -1.2;
//     USB2CAN0_CAN_Bus_1.ID_2_motor_send.kp = 480.0;
//     USB2CAN0_CAN_Bus_1.ID_2_motor_send.kd = 4.0;

//     USB2CAN0_CAN_Bus_1.ID_3_motor_send.position = 3.2;
//     USB2CAN0_CAN_Bus_1.ID_3_motor_send.speed = 0.0;
//     USB2CAN0_CAN_Bus_1.ID_3_motor_send.torque = 0.5;
//     USB2CAN0_CAN_Bus_1.ID_3_motor_send.kp = 200.0;
//     USB2CAN0_CAN_Bus_1.ID_3_motor_send.kd = 2;

//     USB2CAN0_CAN_Bus_1.ID_4_motor_send.position = -0.575;
//     USB2CAN0_CAN_Bus_1.ID_4_motor_send.speed = 0.0;
//     USB2CAN0_CAN_Bus_1.ID_4_motor_send.torque = 0.3;
//     USB2CAN0_CAN_Bus_1.ID_4_motor_send.kp = 80.0;
//     USB2CAN0_CAN_Bus_1.ID_4_motor_send.kd = 1.0;

//     USB2CAN0_CAN_Bus_1.ID_5_motor_send.position = -0.05;
//     USB2CAN0_CAN_Bus_1.ID_5_motor_send.speed = 0.0;
//     USB2CAN0_CAN_Bus_1.ID_5_motor_send.torque = 0.0;
//     USB2CAN0_CAN_Bus_1.ID_5_motor_send.kp = 0.0;
//     USB2CAN0_CAN_Bus_1.ID_5_motor_send.kd = 0.0;
// }

// void Tangair_usb2can::pre_dribble(void)
// {
//     USB2CAN0_CAN_Bus_1.ID_1_motor_send.position = 0.0;
//     USB2CAN0_CAN_Bus_1.ID_1_motor_send.speed = 2.0;
//     USB2CAN0_CAN_Bus_1.ID_1_motor_send.torque = 0.0;
//     USB2CAN0_CAN_Bus_1.ID_1_motor_send.kp = 60.0;
//     USB2CAN0_CAN_Bus_1.ID_1_motor_send.kd = 0.5;

//     USB2CAN0_CAN_Bus_1.ID_2_motor_send.position = 0.0;
//     USB2CAN0_CAN_Bus_1.ID_2_motor_send.speed = -0.3;
//     USB2CAN0_CAN_Bus_1.ID_2_motor_send.torque = 0.0;
//     USB2CAN0_CAN_Bus_1.ID_2_motor_send.kp = 0.0;
//     USB2CAN0_CAN_Bus_1.ID_2_motor_send.kd = 1.7;

//     USB2CAN0_CAN_Bus_1.ID_3_motor_send.position = 0.0;
//     USB2CAN0_CAN_Bus_1.ID_3_motor_send.speed = -0.25;
//     USB2CAN0_CAN_Bus_1.ID_3_motor_send.torque = 0.0;
//     USB2CAN0_CAN_Bus_1.ID_3_motor_send.kp = 0.0;
//     USB2CAN0_CAN_Bus_1.ID_3_motor_send.kd = 1.0;

//     USB2CAN0_CAN_Bus_1.ID_4_motor_send.position = 0.0;
//     USB2CAN0_CAN_Bus_1.ID_4_motor_send.speed = 0.24;
//     USB2CAN0_CAN_Bus_1.ID_4_motor_send.kp = 0.0;
//     USB2CAN0_CAN_Bus_1.ID_4_motor_send.kd = 1.0;
//     USB2CAN0_CAN_Bus_1.ID_4_motor_send.torque = 0.0;

//     USB2CAN0_CAN_Bus_1.ID_5_motor_send.position = -0.3;
//     USB2CAN0_CAN_Bus_1.ID_5_motor_send.speed = 0.0;
//     USB2CAN0_CAN_Bus_1.ID_5_motor_send.torque = 0.0;
//     USB2CAN0_CAN_Bus_1.ID_5_motor_send.kp = 50.0;
//     USB2CAN0_CAN_Bus_1.ID_5_motor_send.kd = 3.0;
// }

void Tangair_usb2can::pre_dribble1(void)
{
    USB2CAN0_CAN_Bus_1.ID_1_motor_send.position = 0.0;
    USB2CAN0_CAN_Bus_1.ID_1_motor_send.speed = 0.5;
    USB2CAN0_CAN_Bus_1.ID_1_motor_send.torque = 0.0;
    USB2CAN0_CAN_Bus_1.ID_1_motor_send.kp = 0.0;
    USB2CAN0_CAN_Bus_1.ID_1_motor_send.kd = 1.0;

    USB2CAN0_CAN_Bus_1.ID_2_motor_send.position = -2.0;
    USB2CAN0_CAN_Bus_1.ID_2_motor_send.speed = 0.0;
    USB2CAN0_CAN_Bus_1.ID_2_motor_send.torque = -1.2;
    USB2CAN0_CAN_Bus_1.ID_2_motor_send.kp = 480.0;
    USB2CAN0_CAN_Bus_1.ID_2_motor_send.kd = 4.0;

    USB2CAN0_CAN_Bus_1.ID_3_motor_send.position = -3;
    USB2CAN0_CAN_Bus_1.ID_3_motor_send.speed = 0.0;
    USB2CAN0_CAN_Bus_1.ID_3_motor_send.torque = 0.5;
    USB2CAN0_CAN_Bus_1.ID_3_motor_send.kp = 300.0;
    USB2CAN0_CAN_Bus_1.ID_3_motor_send.kd = 3.0;

    USB2CAN0_CAN_Bus_1.ID_4_motor_send.position = 1.2;
    USB2CAN0_CAN_Bus_1.ID_4_motor_send.speed = 0.0;
    USB2CAN0_CAN_Bus_1.ID_4_motor_send.kp = 80.0;
    USB2CAN0_CAN_Bus_1.ID_4_motor_send.kd = 3.0;
    USB2CAN0_CAN_Bus_1.ID_4_motor_send.torque = 0.3;

    USB2CAN0_CAN_Bus_1.ID_5_motor_send.position = -0.4;
    USB2CAN0_CAN_Bus_1.ID_5_motor_send.speed = 0.0;
    USB2CAN0_CAN_Bus_1.ID_5_motor_send.torque = 0.01;
    USB2CAN0_CAN_Bus_1.ID_5_motor_send.kp = 50.0;
    USB2CAN0_CAN_Bus_1.ID_5_motor_send.kd = 3.0;
}

void Tangair_usb2can::dribble(void)
{
    USB2CAN0_CAN_Bus_1.ID_1_motor_send.position = 0.0;
    USB2CAN0_CAN_Bus_1.ID_1_motor_send.speed = 0.5;

    USB2CAN0_CAN_Bus_1.ID_2_motor_send.position = -2.06;
    USB2CAN0_CAN_Bus_1.ID_2_motor_send.speed = 0.0;
    USB2CAN0_CAN_Bus_1.ID_2_motor_send.torque = -1.2;
    USB2CAN0_CAN_Bus_1.ID_2_motor_send.kp = 480.0;
    USB2CAN0_CAN_Bus_1.ID_2_motor_send.kd = 4.0;

    USB2CAN0_CAN_Bus_1.ID_3_motor_send.position = -3.79;
    USB2CAN0_CAN_Bus_1.ID_3_motor_send.speed = 0.0;
    USB2CAN0_CAN_Bus_1.ID_3_motor_send.torque = 0.5;
    USB2CAN0_CAN_Bus_1.ID_3_motor_send.kp = 350.0;
    USB2CAN0_CAN_Bus_1.ID_3_motor_send.kd = 3.0;

    USB2CAN0_CAN_Bus_1.ID_4_motor_send.position = 2.41;
    USB2CAN0_CAN_Bus_1.ID_4_motor_send.speed = 0.0;
    USB2CAN0_CAN_Bus_1.ID_4_motor_send.torque = 0.3;
    USB2CAN0_CAN_Bus_1.ID_4_motor_send.kp = 80.0;
    USB2CAN0_CAN_Bus_1.ID_4_motor_send.kd = 3.0;

    USB2CAN0_CAN_Bus_1.ID_5_motor_send.position = -0.42;
    USB2CAN0_CAN_Bus_1.ID_5_motor_send.speed = 0.0;
    USB2CAN0_CAN_Bus_1.ID_5_motor_send.torque = 0.0;
    USB2CAN0_CAN_Bus_1.ID_5_motor_send.kp = 50.0;
    USB2CAN0_CAN_Bus_1.ID_5_motor_send.kd = 3.0;
}

void Tangair_usb2can::dribble1(void)
{
    USB2CAN0_CAN_Bus_1.ID_1_motor_send.position = 0.0;
    USB2CAN0_CAN_Bus_1.ID_1_motor_send.speed = 0.5;

    USB2CAN0_CAN_Bus_1.ID_2_motor_send.position = -1.34451;
    USB2CAN0_CAN_Bus_1.ID_2_motor_send.speed = 20.0;
    // USB2CAN0_CAN_Bus_1.ID_2_motor_send.torque = -1.2;
    // USB2CAN0_CAN_Bus_1.ID_2_motor_send.kp = 480.0;
    // USB2CAN0_CAN_Bus_1.ID_2_motor_send.kd = 4.0;

    USB2CAN0_CAN_Bus_1.ID_3_motor_send.position = -6.24113;
    USB2CAN0_CAN_Bus_1.ID_3_motor_send.speed = 20.0;
    // USB2CAN0_CAN_Bus_1.ID_3_motor_send.torque = 0.5;
    // USB2CAN0_CAN_Bus_1.ID_3_motor_send.kp = 200.0;
    // USB2CAN0_CAN_Bus_1.ID_3_motor_send.kd = 2.0;

    USB2CAN0_CAN_Bus_1.ID_4_motor_send.position = 1.7958;
    USB2CAN0_CAN_Bus_1.ID_4_motor_send.speed = 0.0;
    USB2CAN0_CAN_Bus_1.ID_4_motor_send.torque = 0.3;
    USB2CAN0_CAN_Bus_1.ID_4_motor_send.kp = 50.0;
    USB2CAN0_CAN_Bus_1.ID_4_motor_send.kd = 3.0;

    USB2CAN0_CAN_Bus_1.ID_5_motor_send.position = 0.06;
    USB2CAN0_CAN_Bus_1.ID_5_motor_send.speed = 0.0;
    USB2CAN0_CAN_Bus_1.ID_5_motor_send.torque = 0.0;
    USB2CAN0_CAN_Bus_1.ID_5_motor_send.kp = 20.0;
    USB2CAN0_CAN_Bus_1.ID_5_motor_send.kd = 1.0;
}

// void Tangair_usb2can::update_joint_feedback(uint8_t motor_id, float position, float velocity)
// {
//     if (motor_id >= 1 && motor_id <= 5)
//     {
//         joint_pos[motor_id - 1] = position;
//         joint_vel[motor_id - 1] = velocity;
//     }
// }

// void Tangair_usb2can::change_state(ArmState new_state)
// {
//     current_state = new_state;
//     state_start_time = std::chrono::duration_cast<std::chrono::milliseconds>(
//                            std::chrono::system_clock::now().time_since_epoch())
//                            .count() /
//                        1000.0;
// }

// void Tangair_usb2can::update_arm_control(double current_time, double dt)
// {
//     switch (current_state)
//     {
//     case ARM_IDLE:
//         // 保持初始位置
//         for (int i = 0; i < 5; i++)
//         {
//             arm_motors[i]->position = 0;
//             arm_motors[i]->speed = 0;
//             arm_motors[i]->kp = kp_pos[i];
//             arm_motors[i]->kd = kd_pos[i];
//             arm_motors[i]->torque = 0;
//         }
//         break;
//
//     case ARM_RAISE:
//     {
//         for (int i = 0; i < 5; i++)
//         {
//             double error = raise_pose[i] - joint_pos[i];
//             double tau = kp_pos[i] * error - kd_pos[i] * joint_vel[i];
//             arm_motors[i]->position = raise_pose[i];
//             arm_motors[i]->speed = 0.0;
//             arm_motors[i]->kp = kp_pos[i];
//             arm_motors[i]->kd = kd_pos[i];
//             arm_motors[i]->torque = 0.0;
//         }
//         if (current_time - state_start_time > 1.0)
//             change_state(ARM_BACKSWING);
//         break;
//     }
//
//     case ARM_BACKSWING:
//     {
//         for (int i = 0; i < 5; i++)
//         {
//             double error = backswing_pose[i] - joint_pos[i];
//             arm_motors[i]->position = backswing_pose[i];
//             arm_motors[i]->speed = 0.0;
//             arm_motors[i]->kp = kp_pos[i];
//             arm_motors[i]->kd = kd_pos[i];
//             arm_motors[i]->torque = 0.0;
//         }
//
//         if (current_time - state_start_time > 0.8)
//             change_state(ARM_THROW);
//         break;
//     }
//
//     case ARM_THROW:
//         // 直接输出力矩控制
//         for (int i = 0; i < 5; i++)
//         {
//             arm_motors[i]->position = 0;
//             arm_motors[i]->speed = 0;
//             arm_motors[i]->kp = 0;
//             arm_motors[i]->kd = 0;
//             arm_motors[i]->torque = throw_torque_cmd[i];
//         }
//
//         if (current_time - state_start_time > 0.3)
//             change_state(ARM_RECOVER);
//         break;
//
//     case ARM_RECOVER:
//     {
//         for (int i = 0; i < 5; i++)
//         {
//             arm_motors[i]->position = recover_pose[i];
//             arm_motors[i]->speed = 0.0;
//             arm_motors[i]->kp = kp_pos[i];
//             arm_motors[i]->kd = kd_pos[i];
//             arm_motors[i]->torque = 0.0;
//         }
//
//         if (current_time - state_start_time > 1.0)
//             change_state(ARM_IDLE);
//         break;
//     }
//     }
//
//     // 发送控制命令到所有关节电机
//     for (int i = 0; i < 5; i++)
//     {
//         CAN_Send_Control(USB2CAN0_, 1, arm_motors[i]);
//     }
// }

// # Copyright (c) 2023-2025 TANGAIR
// # SPDX-License-Identifier: Apache-2.0
