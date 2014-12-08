#define bit0 0x01
#define bit1 0x02
#define bit2 0x04
#define bit3 0x08
#define bit4 0x10
#define bit5 0x20
#define bit6 0x40
#define bit7 0x80


#include "Wire\Wire.h"

//��ʾ��ŵĶ���
typedef uint8_t pin;

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

    //��ȡ���룬������ת���ĳ������㣨ֻ��������
    uint64_t GetDistance()
    {
        return distance;
    }   
    
    //��ȡ·�̣�������ת���ĳ������㣨����ǰ�����ӣ�����ǰ����С��
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

    //�򴮿ڷ���һ���ֽ�
    void SentByte(byte val)
    {
        Serial1.write(val);
    }

    //�򴮿ڷ������ݵ��ַ�����ʽ
    void PrintLn(const char* val)
    {
        Serial1.println(val);
    }

    //�򴮿ڷ������ݵ��ַ�����ʽ
    void PrintLn(char val)
    {
        Serial1.println(val);
    }

    //�򴮿ڷ������ݵ��ַ�����ʽ
    void PrintLn(unsigned char val,int base=10)
    {
        Serial1.println(val,base);
    }

    //�򴮿ڷ������ݵ��ַ�����ʽ
    void PrintLn(int val, int base = 10)
    {
        Serial1.println(val, base);
    }

    //�򴮿ڷ������ݵ��ַ�����ʽ
    void PrintLn(unsigned int val, int base=10)
    {
        Serial1.println(val,base);
    }

    //�򴮿ڷ������ݵ��ַ�����ʽ
    void PrintLn(long val,int base=10)
    {
        Serial1.println(val,base);
    }

    //�򴮿ڷ������ݵ��ַ�����ʽ
    void PrintLn(unsigned long val, int base = 10)
    {
        Serial1.println(val,base);
    }

    //�򴮿ڷ������ݵ��ַ�����ʽ
    void PrintLn(double val,int length=6)
    {
        Serial1.println(val,length);
    }

private:
    uint32_t speed;
};

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
        Wire.write(0x01);
        Wire.endTransmission();
        SetDirection(0);
    }

    //��ȡ��ǿ��4 lux �ֱ��ʣ�24 ms ��Ӧʱ��
    uint16_t GetLuxL()
    {
        uint16_t temp = 0;
        Wire.beginTransmission(i2cadd);
        Wire.write(0x13);
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
        Wire.write(0x10);
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
    const pin pinDirection = 11;
    int8_t direction;
};

class DistanceSensor
{
public:
    DistanceSensor(pin pin_trig,pin pin_echo)
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


//���ߴ�����
LightSensor lightSensor = LightSensor();

//��������
UART blueTeeth = UART(9600);

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
	if (box&bit7)
	{
		int a,b,c;
		if (box&bit4) a = 1;
		else a = -1;
		if (box&bit3) b = 1;
		else b = -1;
		if (box&bit2) c = 100;
		else c = 0;
		motorL.SetSpeed(a*c);
		motorR.SetSpeed(b*c);
	}

	if (box&bit6)
	{
		int i,j,k,maxLux=0;
		for (i = -12; i<=12; i++)
		{
			lightSensor.SetDirection(i*10);
			delay(100);
			j = lightSensor.GetLuxL();
			if (j>maxLux) 
			{
				maxLux = j;
				k = i;
			};
		};

		lightSensor.SetDirection(0);
		if (k<0)
		{
			motorL.SetSpeed(-100);
			motorR.SetSpeed(100);
		};
		if (k>0)
		{
			motorL.SetSpeed(100);
			motorR.SetSpeed(-100);
		};
		while (lightSensor.GetLuxL()<maxLux-10);
		motorL.SetSpeed(200);
		motorR.SetSpeed(200);
	}









}
#endif

void loop()
{
	blueTeeth.SentByte(blueTeeth.GetByte());
}