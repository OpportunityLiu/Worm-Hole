
#include "Wire\Wire.h"

//��Ŷ���
typedef uint8_t pin;

//�������������
class Motor
{
public:
    //���캯��
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

    //��ʼ��
    void Init()
    {
        pinMode(low, OUTPUT);
        pinMode(high, OUTPUT);
        attachInterrupt(speed, distanceCallback, RISING);
        SetSpeed(0);
    }

    //��ȡ�ٶȣ�-255~255
    int GetSpeed()
    {
        return motorspeed;
    }

    //�����ٶȣ�-255~255
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

    //�޸�·�̼��������жϺ����е���
    void DistanceMeasure()
    {
        distance++;
        if (speed >= 0)
            range++;
        else
            range--;
    }

    //��ȡ·�̣�������ת���ĳ������㣨ֻ��������
    uint64_t GetDistance()
    {
        return distance;
    }

    //��ȡλ�ƣ�������ת���ĳ������㣨����ǰ�����ӣ�����ǰ����С��
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

//��������
class BlueTeeth
{
public:
    //���캯��
    BlueTeeth(const uint32_t speed)
    {
        this->speed = speed;
    }

    //��������
    ~BlueTeeth()
    {
        Serial1.end();
    }

    //��ʼ��
    void Init()
    {
        Serial1.begin(speed);
    }

    //���ػ��������е��ֽ���
    int GetAvailable()
    {
        return Serial1.available();
    }

    //���ػ�������һ���ֽڣ�������Ϊ��ʱ���� 0
    byte GetByte()
    {
        if (Serial1.available())
        {
            return Serial1.read();
        }
        return 0;
    }

    //���ػ������Ķ���ֽڣ�����ֵΪʵ�ʻ�ȡ���ֽ���
    size_t GetBytes(char* buffer, size_t length)
    {
        return Serial1.readBytes(buffer, length);
    }

    //�򴮿ڷ���һ���ֽڣ�����ֵΪʵ�ʷ��͵��ֽ���
    size_t SentByte(byte val)
    {
        return Serial1.write(val);
    }

    //�򴮿ڷ������ݵ��ַ�����ʽ������ֵΪʵ�ʷ��͵��ֽ���
    size_t PrintLn(const char* val)
    {
        return Serial1.println(val);
    }

    //�򴮿ڷ������ݵ��ַ�����ʽ������ֵΪʵ�ʷ��͵��ֽ���
    size_t PrintLn(char val)
    {
        return Serial1.println(val);
    }

    //�򴮿ڷ������ݵ��ַ�����ʽ������ֵΪʵ�ʷ��͵��ֽ���
    size_t PrintLn(unsigned char val, int base = 10)
    {
        return Serial1.println(val, base);
    }

    //�򴮿ڷ������ݵ��ַ�����ʽ������ֵΪʵ�ʷ��͵��ֽ���
    size_t PrintLn(int val, int base = 10)
    {
        return Serial1.println(val, base);
    }

    //�򴮿ڷ������ݵ��ַ�����ʽ������ֵΪʵ�ʷ��͵��ֽ���
    size_t PrintLn(unsigned int val, int base = 10)
    {
        return Serial1.println(val, base);
    }

    //�򴮿ڷ������ݵ��ַ�����ʽ������ֵΪʵ�ʷ��͵��ֽ���
    size_t PrintLn(long val, int base = 10)
    {
        return Serial1.println(val, base);
    }

    //�򴮿ڷ������ݵ��ַ�����ʽ������ֵΪʵ�ʷ��͵��ֽ���
    size_t PrintLn(unsigned long val, int base = 10)
    {
        return Serial1.println(val, base);
    }

    //�򴮿ڷ������ݵ��ַ�����ʽ������ֵΪʵ�ʷ��͵��ֽ���
    size_t PrintLn(double val, int length = 6)
    {
        return Serial1.println(val, length);
    }

private:
    uint32_t speed;
};

//�����⴫����
class LightSensor
{
public:
    LightSensor()
    {
    }

    //��ʼ��
    void Init()
    {
        Wire.begin();
        Wire.beginTransmission(i2cadd);
        Wire.write(codeBegin);
        Wire.endTransmission();
        SetDirection(0);
    }

    //��ȡ��ǿ��4 lux �ֱ��ʣ�24 ms ��Ӧʱ��
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

    //��ȡ��ǿ��1 lux �ֱ��ʣ�180 ms ��Ӧʱ��
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

    //���ö������-128~127
    void SetDirection(int8_t val)
    {
        direction = val;
        analogWrite(pinDirection, val + 128);
    }

    //��ȡ�������-128~127
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

//���봫����
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

    //���Ծ��룬��λΪ mm
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

//�����⴫����
LightSensor lightSensor = LightSensor();

//��������
BlueTeeth blueTeeth = BlueTeeth(9600);

//�����
Motor motorL = Motor(6, 7, 5, 13, InterruptL);
void InterruptL()
{
    motorL.DistanceMeasure();
}

//�Ҳ���
Motor motorR = Motor(3, 4, 2, 12, InterruptR);
void InterruptR()
{
    motorR.DistanceMeasure();
}

//ǰ�����봫����
DistanceSensor distanceF = DistanceSensor(8, A0);

//�����봫����
DistanceSensor distanceL = DistanceSensor(9, A1);

//�Ҳ���봫����
DistanceSensor distanceR = DistanceSensor(10, A2);

//��ʼ������
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