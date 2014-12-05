
#include "Wire\Wire.h"

class Motor
{
public:
    //���캯��
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

    //��ʼ��
    void Init()
    {
        pinMode(low, OUTPUT);
        pinMode(high, OUTPUT);
        attachInterrupt(speed, distanceCallback, RISING);
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
        if (speed >= 0)
            distance++;
        else
            distance--;
    }

    //��ȡ·�̣�������ת���ĳ�������
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
    //���캯��
    UART(const uint32_t speed)
    {
        this->speed = speed;
    }

    //��ʼ��
    void Init()
    {
        Serial.begin(speed);
    }

    //�Ӵ��ڻ�ȡһ���ֽ�
    byte GetByte()
    {
        return Serial.read();
    }

    //�򴮿ڷ���һ���ֽ�
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

//���ߴ�����
LigetSensor ligetSensor = LigetSensor();
//��������
UART BlueTeeth = UART(9600);
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