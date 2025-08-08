#include "joystick.h"

JoystickController::JoystickController() {
    // SDL初始化在构造函数中完成
}

JoystickController::~JoystickController() {
    stop();
    if (gamepad_) {
        SDL_JoystickClose(gamepad_);
        gamepad_ = nullptr;
    }
    SDL_QuitSubSystem(SDL_INIT_JOYSTICK);
}

bool JoystickController::initialize() {
    if (SDL_Init(SDL_INIT_JOYSTICK) < 0) {
        std::cerr << "SDL初始化失败: " << SDL_GetError() << std::endl;
        return false;
    }
    
    if (SDL_NumJoysticks() > 0) {
        gamepad_ = SDL_JoystickOpen(0);
        if (gamepad_) {
            std::cout << "检测到游戏手柄: " << SDL_JoystickName(gamepad_) 
                      << " (轴数: " << SDL_JoystickNumAxes(gamepad_)
                      << ", 按钮: " << SDL_JoystickNumButtons(gamepad_) << ")" << std::endl;
            return true;
        } else {
            std::cerr << "无法打开游戏手柄: " << SDL_GetError() << std::endl;
        }
    } else {
        std::cout << "未检测到游戏手柄" << std::endl;
    }
    return false;
}

void JoystickController::start() {
    if (running_ || !gamepad_) return;
    running_ = true;
    event_thread_ = std::thread(&JoystickController::eventLoop, this);
}

void JoystickController::stop() {
    running_ = false;
    if (event_thread_.joinable()) {
        event_thread_.join();
    }
}

JoystickController::GamepadState JoystickController::getState() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return state_;
}

void JoystickController::setButtonCallback(ButtonCallback callback) {
    button_callback_ = callback;
}

void JoystickController::eventLoop() {
    const float JOYSTICK_DEADZONE = 0.1f;
    SDL_Event event;

    while (running_) {
        while (SDL_PollEvent(&event)) {
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                
                switch (event.type) {
                    case SDL_JOYAXISMOTION:
                        // 处理摇杆输入
                        switch (event.jaxis.axis) {
                            case 0: // 左摇杆X轴
                                state_.left_x = (std::abs(event.jaxis.value) < 32767.0f * JOYSTICK_DEADZONE) ? 
                                    0.0f : event.jaxis.value / 32767.0f;
                                break;
                            case 1: // 左摇杆Y轴
                                state_.left_y = (std::abs(event.jaxis.value) < 32767.0f * JOYSTICK_DEADZONE) ? 
                                    0.0f : event.jaxis.value / 32767.0f;
                                break;
                            case 2: // 右摇杆X轴
                                state_.right_x = (std::abs(event.jaxis.value) < 32767.0f * JOYSTICK_DEADZONE) ? 
                                    0.0f : event.jaxis.value / 32767.0f;
                                break;
                            case 3: // 右摇杆Y轴
                                state_.right_y = (std::abs(event.jaxis.value) < 32767.0f * JOYSTICK_DEADZONE) ? 
                                    0.0f : event.jaxis.value / 32767.0f;
                                break;
                        }
                        break;
                        
                    case SDL_JOYBUTTONDOWN:
                    case SDL_JOYBUTTONUP:
                        // 处理按钮输入
                        if (event.jbutton.button < 15) {
                            bool pressed = (event.type == SDL_JOYBUTTONDOWN);
                            state_.buttons[event.jbutton.button] = pressed;
                            
                            // 如果有回调函数，则调用
                            if (button_callback_) {
                                button_callback_(event.jbutton.button, pressed);
                            }
                        }
                        break;
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}