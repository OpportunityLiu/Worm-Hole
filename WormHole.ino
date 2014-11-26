
//接口定义

//PIN: 左侧电机转速控制 (PMW)
#define pin_motor_l_pmw 6
//PIN: 左侧电机转速控制 (PMW)
#define pin_motor_l_h 5
//PIN: 左侧电机转速控制 (PMW)
#define pin_motor_l_l 7
//PIN: 右侧电机转速控制 (PMW)
#define pin_motor_r_pmw 3
//PIN: 右侧电机转速控制 (PMW)
#define pin_motor_r_h 2
//PIN: 右侧电机转速控制 (PMW)
#define pin_motor_r_l 4
#include "Wire\Wire.h"

class Motor
{
public:
    Motor(uint8_t pin_pmw, uint8_t pin_low, uint8_t pin_high)
    {
        pmw = pin_pmw;
        high = pin_high;
        low = pin_low;
        pinMode(pin_low, OUTPUT);
        pinMode(pin_high, OUTPUT);
    }

    int getSpeed()
    {
        return speed;
    }

    void setSpeed(int speed)
    {
        if (speed > 255 || speed < -255)
        {
            return;
        }
        if (speed >= 0)
        {
            digitalWrite(high, HIGH);
            digitalWrite(low, LOW);
            analogWrite(pmw, speed);
        }
        else
        {
            digitalWrite(low, HIGH);
            digitalWrite(high, LOW);
            analogWrite(pmw, -speed);
        }
        speed = this->speed;
    }

private:
    uint8_t pmw, low, high;
    int speed = 0;
};

Motor motorL(6, 7, 5);
Motor motorR(3, 4, 2);

void setup()
{
    motorL.setSpeed(100);
    motorR.setSpeed(200);
}

void loop()
{
}


