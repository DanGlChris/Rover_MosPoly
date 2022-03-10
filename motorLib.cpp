#include "motorLib.h"

uint8_t myMotor::motor_count = 0;

myMotor::myMotor() { }

void myMotor::init(uint8_t RPWM_pin, uint8_t LPWM_pin, bool reverse=false)
{
    this->reverse = reverse;

    this->RPWM_pin = RPWM_pin;
    this->LPWM_pin = LPWM_pin;
    pinMode(this->RPWM_pin , OUTPUT);
    pinMode(this->LPWM_pin , OUTPUT);
    this->setSpeed(0);

    motor_num = motor_count;
    motor_count++;
}

/**
 * pwm должен быть в %
 */
void myMotor::setVelocity(uint8_t pwm, bool route)
{
    if ((route || reverse) && (!route || !reverse))
    {
        analogWrite(this->LPWM_pin, 0);
        analogWrite(this->RPWM_pin, map(pwm, 0, 100, 0, 255));
    } else {
        analogWrite(this->LPWM_pin, map(pwm, 0, 100, 0, 255));
        analogWrite(this->RPWM_pin, 0);
    }
}

void myMotor::setSpeed(int16_t pwm)
{
    if (((pwm > 0) || reverse) && (!(pwm > 0) || !reverse))
    {
        analogWrite(this->LPWM_pin, 0);
        analogWrite(this->RPWM_pin, constrain(abs(pwm), 0, 255));
    } else {
        analogWrite(this->LPWM_pin, 0);
        analogWrite(this->RPWM_pin, constrain(abs(pwm), 0, 255));
    }
}

void myMotor::setStop()
{
    analogWrite(this->LPWM_pin, 0);
    analogWrite(this->RPWM_pin, 0);
}

uint8_t myMotor::getMotorNum()
{
    return motor_num;
}

uint8_t myMotor::getMotorCount()
{
    return motor_count;
}
