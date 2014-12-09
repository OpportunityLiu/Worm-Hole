
#include "Wire\Wire.h"

//针脚定义
typedef uint8_t pin;

//驱动电机及码盘
class Motor
{
public:
    //构造函数
    Motor(pin pin_pmw, pin pin_low, pin pin_high, pin pin_speed, void(*callback)())
    {
        pmw = pin_pmw;
        high = pin_high;
        low = pin_low;
        speed = pin_speed;
        motorspeed = 0;
        distance = 0;
        range = 0;
        distanceCallback = callback;

    }

    //析构函数
    ~Motor()
    {
        detachInterrupt(speed);
    }

    //初始化
    void Init()
    {
        pinMode(low, OUTPUT);
        pinMode(high, OUTPUT);
        attachInterrupt(speed, distanceCallback, RISING);
        SetSpeed(0);
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
        distance++;
        if (speed >= 0)
            range++;
        else
            range--;
    }

    //获取路程，以码盘转过的齿数计算（只增不减）
    unsigned long GetDistance()
    {
        return distance;
    }

    //获取位移，以码盘转过的齿数计算（正向前进增加，反向前进减小）
    long GetRange()
    {
        return range;
    }

private:
    pin pmw, low, high, speed;
    int motorspeed;
    unsigned long distance;
    long range;
    void(*distanceCallback)();
};

//蓝牙串口
class BlueTeeth
{
public:
    //构造函数
    BlueTeeth(const uint32_t speed)
    {
        this->speed = speed;
    }

    //析构函数
    ~BlueTeeth()
    {
        Serial1.end();
    }

    //初始化
    void Init()
    {
        Serial1.begin(speed);
    }

    //返回缓冲区存有的字节数
    int GetAvailable()
    {
        return Serial1.available();
    }

    //返回缓冲区第一个字节，缓冲区为空时返回 0
    byte GetByte()
    {
        if (Serial1.available())
        {
            return Serial1.read();
        }
        return 0;
    }

    //返回缓冲区的多个字节，返回值为实际获取的字节数
    size_t GetBytes(char* buffer, size_t length)
    {
        return Serial1.readBytes(buffer, length);
    }

    //向串口发送一个字节，返回值为实际发送的字节数
    size_t SentByte(byte val)
    {
        return Serial1.write(val);
    }

    //向串口发送数据的字符串形式，返回值为实际发送的字节数
    size_t PrintLn(const char* val)
    {
        return Serial1.println(val);
    }

    //向串口发送数据的字符串形式，返回值为实际发送的字节数
    size_t PrintLn(char val)
    {
        return Serial1.println(val);
    }

    //向串口发送数据的字符串形式，返回值为实际发送的字节数
    size_t PrintLn(unsigned char val, int base = 10)
    {
        return Serial1.println(val, base);
    }

    //向串口发送数据的字符串形式，返回值为实际发送的字节数
    size_t PrintLn(int val, int base = 10)
    {
        return Serial1.println(val, base);
    }

    //向串口发送数据的字符串形式，返回值为实际发送的字节数
    size_t PrintLn(unsigned int val, int base = 10)
    {
        return Serial1.println(val, base);
    }

    //向串口发送数据的字符串形式，返回值为实际发送的字节数
    size_t PrintLn(long val, int base = 10)
    {
        return Serial1.println(val, base);
    }

    //向串口发送数据的字符串形式，返回值为实际发送的字节数
    size_t PrintLn(unsigned long val, int base = 10)
    {
        return Serial1.println(val, base);
    }

    //向串口发送数据的字符串形式，返回值为实际发送的字节数
    size_t PrintLn(double val, int length = 6)
    {
        return Serial1.println(val, length);
    }   
    
    //向串口发送数据的字符串形式，返回值为实际发送的字节数
    size_t PrintLn()
    {
        return Serial1.println();
    }

private:
    uint32_t speed;
};

//环境光传感器
class LightSensor
{
public:
    //构造函数
    LightSensor()
    {
    }

    //析构函数
    ~LightSensor()
    {
        Wire.beginTransmission(i2cadd);
        Wire.write(codePowerDown);
        Wire.endTransmission();
    }

    //初始化
    void Init()
    {
        Wire.begin();
        Wire.beginTransmission(i2cadd);
        Wire.write(codePowerOn);
        Wire.endTransmission();
        SetDirection(0);
    }

    //获取光强，4 lux 分辨率，24 ms 响应时间
    uint16_t GetLuxL()
    {
        uint16_t temp = 0;
        Wire.beginTransmission(i2cadd);
        Wire.write(codeLResolution);
        Wire.endTransmission();
        delay(24);
        Wire.requestFrom(i2cadd, 2);
        Wire.readBytes((char*)&temp, 2);
        return temp;
    }

    //获取光强，1 lux 分辨率，180 ms 响应时间
    uint16_t GetLuxH()
    {
        uint16_t temp = 0;
        Wire.beginTransmission(i2cadd);
        Wire.write(codeHResolution);
        Wire.endTransmission();
        delay(180);
        Wire.requestFrom(i2cadd, 2);
        Wire.readBytes((char*)&temp, 2);
        return temp;
    }

    //设置舵机方向，-128~127
    void SetDirection(int8_t val)
    {
        direction = val;
        analogWrite(pinDirection, val + 128);
    }

    //获取舵机方向，-128~127
    int8_t GetDirection()
    {
        return direction;
    }

private:
    const int i2cadd = 0x13;
    enum optionCode
    {
        codePowerDown = 0x00,
        codePowerOn = 0x01,
        codeHResolution = 0x10,
        codeLResolution = 0x13
    };
    const pin pinDirection = 11;
    int8_t direction;
};

//距离传感器
class DistanceSensor
{
public:
    //构造函数
    DistanceSensor(pin pin_trig, pin pin_echo)
    {
        trig = pin_trig;
        echo = pin_echo;
    }

    //初始化
    void Init()
    {
        pinMode(trig, OUTPUT);
    }

    //测试距离，单位为 mm
    double GetDistance()
    {
        digitalWrite(trig, HIGH);
        delay(1);
        digitalWrite(trig, LOW);
        return (double)pulseIn(echo, HIGH, 60000) * 0.17;
    }

private:
    pin trig, echo;
};

//环境光传感器
LightSensor lightSensor = LightSensor();

//蓝牙串口
BlueTeeth blueTeeth = BlueTeeth(9600);

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

//前方距离传感器
DistanceSensor distanceF = DistanceSensor(8, A0);

//左侧距离传感器
DistanceSensor distanceL = DistanceSensor(9, A1);

//右侧距离传感器
DistanceSensor distanceR = DistanceSensor(10, A2);

//当前模式
byte Mode = 0;

//初始化函数
void Init()
{
    blueTeeth.Init();
    motorL.Init();
    motorR.Init();
    lightSensor.Init();
    distanceF.Init();
    distanceL.Init();
    distanceR.Init();
}

void Test(byte message)
{
    blueTeeth.PrintLn("OK");
    blueTeeth.PrintLn(message, 2);
}

void ModeRemoteCtrl(byte message)
{
    int8_t _speed = message << 4;
    int16_t speed = 2 * _speed + 16;
    if (bitRead(message,4))
    {
        //右轮
        motorR.SetSpeed(speed);
    }
    else
    {
        //左轮
        motorL.SetSpeed(speed);
    }
}

void ModeStraight(byte message)
{
    if (bitRead(message,0))
    {
        //开始执行
    }
}

void ModeFindLight(byte message)
{
    if (bitRead(message, 0))
    {
        int i, j, k, maxLux = 0;
        for (i = -12; i <= 12; i++)
        {
            lightSensor.SetDirection(i * 10);
            delay(100);
            j = lightSensor.GetLuxL();
            if (j > maxLux)
            {
                maxLux = j;
                k = i;
            };
        };

        lightSensor.SetDirection(0);
        if (k < 0)
        {
            motorL.SetSpeed(-100);
            motorR.SetSpeed(100);
        };
        if (k > 0)
        {
            motorL.SetSpeed(100);
            motorR.SetSpeed(-100);
        };
        while (lightSensor.GetLuxL() < maxLux - 10);
        motorL.SetSpeed(200);
        motorR.SetSpeed(200);
    }
}

void SendState(byte message)
{
    if (bitRead(message, 5))
    {
        //lightSensor
        blueTeeth.PrintLn("Light sensor:");
        blueTeeth.PrintLn("direction:");
        blueTeeth.PrintLn(lightSensor.GetDirection());
        blueTeeth.PrintLn("lux:");
        blueTeeth.PrintLn(lightSensor.GetLuxL());
        blueTeeth.PrintLn();
    }
    if (bitRead(message, 4))
    {
        //motorL
        blueTeeth.PrintLn("Left motor:");
        blueTeeth.PrintLn("speed:");
        blueTeeth.PrintLn(motorL.GetSpeed());
        blueTeeth.PrintLn("distance:");
        blueTeeth.PrintLn(motorL.GetDistance());
        blueTeeth.PrintLn("distance:");
        blueTeeth.PrintLn(motorL.GetDistance());
        blueTeeth.PrintLn();
    }
    if (bitRead(message, 3))
    {
        //motolR
        blueTeeth.PrintLn("Right motor:");
        blueTeeth.PrintLn("speed:");
        blueTeeth.PrintLn(motorR.GetSpeed());
        blueTeeth.PrintLn("distance:");
        blueTeeth.PrintLn(motorR.GetDistance());
        blueTeeth.PrintLn("distance:");
        blueTeeth.PrintLn(motorR.GetDistance());
        blueTeeth.PrintLn();
    }
    if (bitRead(message, 2))
    {
        //distanceF
        blueTeeth.PrintLn("Front distance sensor:");
        blueTeeth.PrintLn(distanceF.GetDistance());
        blueTeeth.PrintLn();
    }
    if (bitRead(message, 1))
    {
        //distanceL
        blueTeeth.PrintLn("Left distance sensor:");
        blueTeeth.PrintLn(distanceL.GetDistance());
        blueTeeth.PrintLn();
    }
    if (bitRead(message, 0))
    {
        //distanceR}
        blueTeeth.PrintLn("Right distance sensor:");
        blueTeeth.PrintLn(distanceR.GetDistance());
        blueTeeth.PrintLn();
    }
}

void setup()
{
    Init();
}

//#define DEBUG
#ifndef DEBUG

void loop()
{
    if (blueTeeth.GetAvailable())
    {
        //蓝牙指令
        byte message = blueTeeth.GetByte();
        if (bitRead(message, 7))
        {
            //1xxxxxxx
            if (bitRead(message, 6))
            {
                //11xxxxxx
                SendState(message);
            }
            else
            {
                //10xxxxxx
                if (bitRead(message,5))
                {
                    //101xxxxx
                    switch (Mode)
                    {
                    case 0x01:
                        ModeRemoteCtrl(message);
                        break;
                    case 0x02:
                        ModeStraight(message);
                        break;
                    case 0x03:
                        ModeFindLight(message);
                        break;
                    default:
                        break;
                    }
                }
                else
                {
                    //100xxxxx
                    switch (message)
                    {
                    case 0x80:
                        Mode = 0x00;
                        break;
                    case 0x81:
                        Mode = 0x01;
                        break;
                    case 0x82:
                        Mode = 0x02;
                        break;
                    case 0x83:
                        Mode = 0x03;
                        break;
                    default:
                        break;
                    }
                }
            }
        }
        else
        {
            //0xxxxxxx
            if (bitRead(message, 6))
            {
                //01xxxxxx
            }
            else
            {
                //00xxxxxx
                Test(message);
            }
        }
    }
}

#else

void loop()
{
    blueTeeth.SentByte(blueTeeth.GetByte());
}

#endif