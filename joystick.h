#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <SDL2/SDL.h>
#include <atomic>
#include <thread>
#include <functional>
#include <iostream>
#include <mutex>

class JoystickController

{
public:
    // 手柄状态结构体
    struct GamepadState
    {
        float left_x = 0.0f;        // 左摇杆X轴 [-1.0, 1.0]
        float left_y = 0.0f;        // 左摇杆Y轴 [-1.0, 1.0]
        float right_x = 0.0f;       // 右摇杆X轴 [-1.0, 1.0]
        float right_y = 0.0f;       // 右摇杆Y轴 [-1.0, 1.0]
        bool buttons[15] = {false}; // 按钮状态，假设最多15个按钮
    };

    // 按钮事件回调函数类型
    using ButtonCallback = std::function<void(int button, bool pressed)>;

    JoystickController();
    ~JoystickController();

    bool initialize(); // 初始化手柄
    void start();      // 开始监听手柄事件
    void stop();       // 停止监听

    // 获取当前手柄状态（线程安全）
    GamepadState getState() const;

    // 设置按钮回调函数
    void setButtonCallback(ButtonCallback callback);

private:
    void eventLoop(); // 事件循环

    SDL_Joystick *gamepad_ = nullptr;
    std::atomic<bool> running_{false};
    std::thread event_thread_;
    mutable std::mutex state_mutex_;
    GamepadState state_;
    ButtonCallback button_callback_;
};

#endif // JOYSTICK_H