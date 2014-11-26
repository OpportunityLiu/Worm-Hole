
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

    //获取速度，-255~255
    int getSpeed()
    {
        return speed;
    }

    //设置速度，-255~255
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

//左侧电机
Motor motorL = Motor(6, 7, 5);
//右侧电机
Motor motorR = Motor(3, 4, 2);

void setup()
{
    motorL.setSpeed(100);
    motorR.setSpeed(200);
}

void loop()
{
}


