#pragma once
#include <stdint.h>
#include "Arduino.h"

class myMotor {
private:

    uint8_t RPWM_pin, LPWM_pin;
    static uint8_t motor_count;
    uint8_t motor_num;

public:

    bool reverse;
    myMotor();
    void init(uint8_t RPWM_pin, uint8_t LPWM_pin, bool reverse=false);
    void setVelocity(uint8_t pwm, bool route);
    void setSpeed(int16_t pwm);
    void setStop();
    uint8_t getMotorNum();
    uint8_t getMotorCount();
};
