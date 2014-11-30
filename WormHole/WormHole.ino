
#include "Wire\Wire.h"

class Motor
{
public:
    //构造函数
    Motor(uint8_t pin_pmw, uint8_t pin_low, uint8_t pin_high)
    {
        pmw = pin_pmw;
        high = pin_high;
        low = pin_low;
        motorspeed = 0;
    }

    //初始化
    void Init()
    {
        pinMode(low, OUTPUT);
        pinMode(high, OUTPUT);
    }

    //获取速度，-255~255
    int GetSpeed()
    {
        return motorspeed;
    }

    //设置速度，-255~255
    void SetSpeed(int speed)
    {
        if (speed > 255 || speed < -255)
        {
            return;
        }
        motorspeed = speed;
        if (motorspeed >= 0)
        {
            digitalWrite(high, HIGH);
            digitalWrite(low, LOW);
            analogWrite(pmw, motorspeed);
        }
        else
        {
            digitalWrite(low, HIGH);
            digitalWrite(high, LOW);
            analogWrite(pmw, -motorspeed);
        }
    }

private:
    uint8_t pmw, low, high;
    int motorspeed;
};

//左侧电机
Motor motorL = Motor(6, 5, 7);
//右侧电机
Motor motorR = Motor(3, 2, 4);

void setup()
{
    motorL.Init();
    motorR.Init();
    motorL.SetSpeed(100);
    motorR.SetSpeed(200);
}

void loop()
{
}


