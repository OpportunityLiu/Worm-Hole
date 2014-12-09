
#include "Wire\Wire.h"

//��Ŷ���
typedef uint8_t pin;

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

    //�򴮿ڷ���һ���ֽ�
    void SentByte(byte val)
    {
		delay(70);
        Serial1.write(val);
    }

    //�򴮿ڷ������ݵ��ַ�����ʽ
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

//��������
BlueTeeth blueTeeth = BlueTeeth(9600);

//�������������
class Motor
{
public:
    //���캯��
    Motor(pin pin_pmw, pin pin_low, pin pin_high, pin pin_speed)
    {
        pmw = pin_pmw;
        high = pin_high;
        low = pin_low;
        speed = pin_speed;
        motorspeed = 0;
    }

    //��������
    ~Motor()
    {
        detachInterrupt(speed);
    }

    //��ʼ��
    void Init()
    {
        pinMode(low, OUTPUT);
        pinMode(high, OUTPUT);
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

    //����ת����Ӧ�����󷵻�
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

//�����
Motor motorL = Motor(6, 5, 7, 13);

//�Ҳ���
Motor motorR = Motor(3, 2, 4, 12);
//�����⴫����
class LightSensor
{
public:
    //���캯��
    LightSensor()
    {
    }

    //��������
    ~LightSensor()
    {
        Wire.beginTransmission(i2cadd);
        Wire.write(codePowerDown);
        Wire.endTransmission();
    }

    //��ʼ��
    void Init()
    {
        Wire.begin();
        Wire.beginTransmission(i2cadd);
        Wire.write(codePowerOn);
        Wire.endTransmission();
        SetDirection(0);
    }

    //��ȡ��ǿ��4 lux �ֱ��ʣ�24 ms ��Ӧʱ��
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

    //��ȡ��ǿ��1 lux �ֱ��ʣ�180 ms ��Ӧʱ��
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

//�����⴫����
LightSensor lightSensor = LightSensor();

//���봫����
class DistanceSensor
{
public:
    //���캯��
    DistanceSensor(pin pin_trig, pin pin_echo)
    {
        trig = pin_trig;
        echo = pin_echo;
    }

    //��ʼ��
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

//ǰ�����봫����
DistanceSensor distanceF = DistanceSensor(8, A0);

//�����봫����
DistanceSensor distanceL = DistanceSensor(9, A1);

//�Ҳ���봫����
DistanceSensor distanceR = DistanceSensor(10, A2);

//��ǰģʽ
//  0 ֹͣ
//  1 ң��
//  2 ֱ��
//  3 Ѱ��
byte Mode = 0;

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

//������������
void Test(byte message)
{
    char out[20] = {0};
    sprintf(out, "OK\n%#X\n", message);
    blueTeeth.Print(out);
}

//����ң��
void ModeRemoteCtrl(byte message)
{
    int8_t _speed = message << 4;
    int16_t speed = 2 * _speed + 16;
    if (bitRead(message, 4))
    {
        //����
        motorR.SetSpeed(speed);
    }
    else
    {
        //����
        motorL.SetSpeed(speed);
    }
}

//ֱ��ǰ�� AI
void ModeStraight(byte message)
{
    if (bitRead(message, 0))
    {
        //��ʼִ��
    }
}

//Ѱ�� AI
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

//����״̬������
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
        //����ָ��
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