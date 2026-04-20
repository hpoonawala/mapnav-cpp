#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <string>

class MotorController {
    std::string port_;
public:
    MotorController(const std::string& port) : port_(port) {}
    void stop();
    void send(int v, int w);
};

#endif // MOTOR_CONTROLLER_H
