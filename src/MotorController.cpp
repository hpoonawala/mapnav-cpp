#include "../include/MotorController.h"
#include "../include/SerialWriter.h"
#include <sstream>
#include <iomanip>
#include <iostream>

void MotorController::stop() {
    try {
        write_serial_message(port_, "S.+000.+000.0\n");
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
}

void MotorController::send(int v, int w) {
    bool signflag = w > 0 ? false : true;
    std::ostringstream oss_v;
    oss_v << std::setfill('0') << std::setw(3) << std::abs(v);
    std::ostringstream oss_w;
    oss_w << std::setfill('0') << std::setw(3) << std::abs(w);
    std::string msg = "D.+";
    msg += oss_v.str();
    msg += ".";
    msg += (signflag ? "+" : "-");
    msg += oss_w.str();
    msg += ".0\n";
    try {
        write_serial_message(port_, msg.c_str());
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
}
