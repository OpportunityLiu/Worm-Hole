
#include "Wire\Wire.h"

//针脚定义
typedef uint8_t pin;

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

    //向串口发送一个字节
    void SentByte(byte val)
    {
		delay(70);
        Serial1.write(val);
    }

    //向串口发送数据的字符串形式
    void Print(const char* val)
    {
        for (char* i = (char*)val; *i != '\0'; i++)
        {
			delay(70);
            Serial1.write(*i);
        }
    }

private:
    uint32_t speed;
};

//蓝牙串口
BlueTeeth blueTeeth = BlueTeeth(9600);

//驱动电机及码盘
class Motor
{
public:
    //构造函数
    Motor(pin pin_pmw, pin pin_low, pin pin_high, pin pin_speed)
    {
        pmw = pin_pmw;
        high = pin_high;
        low = pin_low;
        speed = pin_speed;
        motorspeed = 0;
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

    //码盘转过相应齿数后返回
    void WaitDistance(uint32_t times)
    {
        int state = digitalRead(speed);
        int newstate;
        int64_t i = -(int64_t)times;
        while (i<times)
        {
            newstate = digitalRead(speed);
            if (state!=newstate)
            {
                state = newstate;
                i++;
            }
        }
    }

private:
    pin pmw, low, high, speed;
    int motorspeed;
};

//左侧电机
Motor motorL = Motor(6, 5, 7, 13);

//右侧电机
Motor motorR = Motor(3, 2, 4, 12);
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
    const int i2cadd = 0x23;
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

//环境光传感器
LightSensor lightSensor = LightSensor();

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

//前方距离传感器
DistanceSensor distanceF = DistanceSensor(8, A0);

//左侧距离传感器
DistanceSensor distanceL = DistanceSensor(9, A1);

//右侧距离传感器
DistanceSensor distanceR = DistanceSensor(10, A2);

//当前模式
//  0 停止
//  1 遥控
//  2 直线
//  3 寻光
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

//测试蓝牙链接
void Test(byte message)
{
    char out[20] = {0};
    sprintf(out, "OK\n%#X\n", message);
    blueTeeth.Print(out);
}

//蓝牙遥控
void ModeRemoteCtrl(byte message)
{
    int8_t _speed = message << 4;
    int16_t speed = 2 * _speed + 16;
    if (bitRead(message, 4))
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

//直线前进 AI
void ModeStraight(byte message)
{
    if (bitRead(message, 0))
    {
        //开始执行
    }
}

//寻光 AI
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

//发送状态至蓝牙
void SendState(byte message)
{
    if (bitRead(message, 5))
    {
        //lightSensor
        char out5[100] = {0};
        sprintf(out5, "Light:\n\tdire: %d\n\tlux: %d\n\n", 
                lightSensor.GetDirection(), lightSensor.GetLuxL());
        blueTeeth.Print(out5);
    }
    if (bitRead(message, 4))
    {
        //motorL
        char out4[50] = {0};
        sprintf(out4, "L_motor: %d\n\n", motorL.GetSpeed());
        blueTeeth.Print(out4);
    }
    if (bitRead(message, 3))
    {
        //motolR
        char out3[50] = {0};
        sprintf(out3, "R_motor: %d\n\n", motorR.GetSpeed());
        blueTeeth.Print(out3);
    }
    if (bitRead(message, 2))
    {
        //distanceF
        char out2[50] = {0};
        sprintf(out2, "F_dist: %f\n\n", distanceF.GetDistance());
        blueTeeth.Print(out2);
    }
    if (bitRead(message, 1))
    {
        //distanceL
        char out1[50] = {0};
        sprintf(out1, "L_dist: %f\n\n", distanceL.GetDistance());
        blueTeeth.Print(out1);
    }
    if (bitRead(message, 0))
    {
        //distanceR
        char out0[50] = {0};
        sprintf(out0, "R_dist: %f\n\n", distanceR.GetDistance());
        blueTeeth.Print(out0);
    }
}

void setup()
{
    Init();
}

#define DEBUG
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
                if (bitRead(message, 5))
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
    motorL.WaitDistance(1000);
}

#endif