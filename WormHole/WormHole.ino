
#include "Wire\Wire.h"

class Motor
{
public:
    //构造函数
    Motor(uint8_t pin_pmw, uint8_t pin_low, uint8_t pin_high,uint8_t pin_speed,void(*callback)())
    {
        pmw = pin_pmw;
        high = pin_high;
        low = pin_low;
        speed = pin_speed;
        motorspeed = 0;
        distance = 0;
        distanceCallback = callback;
    }

    //初始化
    void Init()
    {
        pinMode(low, OUTPUT);
        pinMode(high, OUTPUT);
        attachInterrupt(speed, distanceCallback, RISING);
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

    //修改路程计数，在中断函数中调用
    void DistanceMeasure()
    {
        if (speed >= 0)
            distance++;
        else
            distance--;
    }

    //获取路程，以码盘转过的齿数计算
    long GetDistance()
    {
        return distance;
    }

private:
    uint8_t pmw, low, high, speed;
    int motorspeed;
    long distance;
    void(*distanceCallback)();
};

class UART
{
public:
    //构造函数
    UART(const uint32_t speed)
    {
        this->speed = speed;
    }

    //初始化
    void Init()
    {
        Serial.begin(speed);
    }

    //从串口获取一个字节
    byte GetByte()
    {
        return Serial.read();
    }

    //向串口发送一个字节
    void SentByte(byte val)
    {
        Serial.write(val);
    }

private:
    uint32_t speed;
};

class LigetSensor
{
public:
    LigetSensor()
    {
    }

    void Init()
    {
        Wire.begin();
        Wire.beginTransmission(0x13);
        Wire.write(0x01);
        Wire.endTransmission();
    }

    double GetLux()
    {
        char temp[2] = {};
        Wire.beginTransmission(0x13);
        Wire.write(0x10);
        Wire.endTransmission();
        delay(180);
        Wire.requestFrom(0x13, 2);
        Wire.readBytes(temp, 2);
        return (uint16_t)((temp[0] << 8) | temp[1]) / 1.2;
    }
};

//光线传感器
LigetSensor ligetSensor = LigetSensor();
//蓝牙串口
UART BlueTeeth = UART(9600);
//左侧电机
Motor motorL = Motor(6, 7, 5, 13, InterruptL);
void InterruptL()
{
    motorL.DistanceMeasure();
}
//右侧电机
Motor motorR = Motor(3, 4, 2, 12, InterruptR);
void InterruptR()
{
    motorR.DistanceMeasure();
}

void setup()
{
    BlueTeeth.Init();
    motorL.Init();
    motorR.Init();
    motorL.SetSpeed(100);
    motorR.SetSpeed(200);
    ligetSensor.Init();

}

void loop()
{
}