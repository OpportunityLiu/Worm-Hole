
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
    uint64_t GetDistance()
    {
        return distance;
    }

    //获取位移，以码盘转过的齿数计算（正向前进增加，反向前进减小）
    int64_t GetRange()
    {
        return range;
    }

private:
    pin pmw, low, high, speed;
    int motorspeed;
    uint64_t distance;
    int64_t range;
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

private:
    uint32_t speed;
};

//环境光传感器
class LightSensor
{
public:
    LightSensor()
    {
    }

    //初始化
    void Init()
    {
        Wire.begin();
        Wire.beginTransmission(i2cadd);
        Wire.write(codeBegin);
        Wire.endTransmission();
        SetDirection(0);
    }

    //获取光强，4 lux 分辨率，24 ms 响应时间
    uint16_t GetLuxL()
    {
        uint16_t temp = 0;
        Wire.beginTransmission(i2cadd);
        Wire.write(codeL);
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
        Wire.write(codeH);
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
        codeBegin = 0x01,
        codeH = 0x10,
        codeL = 0x13
    };
    const pin pinDirection = 11;
    int8_t direction;
};

//距离传感器
class DistanceSensor
{
public:
    DistanceSensor(pin pin_trig, pin pin_echo)
    {
        trig = pin_trig;
        echo = pin_echo;
    }

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

void setup()
{
    Init();
    //motorL.SetSpeed(100);
    //motorR.SetSpeed(200);
}

#ifdef abc

void loop()
{
    byte box;
    box = blueTeeth.GetByte();
    if (bitRead(box, 7))
    {
        int a, b, c;
        if (bitRead(box, 4)) a = 1;
        else a = -1;
        if (bitRead(box, 3)) b = 1;
        else b = -1;
        if (bitRead(box, 2)) c = 100;
        else c = 0;
        motorL.SetSpeed(a*c);
        motorR.SetSpeed(b*c);
    }

    if (bitRead(box, 6))
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

#else

void loop()
{
    blueTeeth.SentByte(blueTeeth.GetByte());
}

#endif