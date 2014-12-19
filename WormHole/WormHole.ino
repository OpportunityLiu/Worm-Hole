
#include "Wire\Wire.h"

//��Ŷ���
typedef uint8_t pin;

#ifndef _WH_HARDWARE_
#define _WH_HARDWARE_

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

	//��ʼ��
	void Init()
	{
		pinMode(low, OUTPUT);
		pinMode(high, OUTPUT);
		pinMode(speed,INPUT);
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
		if (speed > 0)
		{
			speed += 50;
			motorspeed = (speed > 255) ? 255 : speed;
			digitalWrite(high, HIGH);
			digitalWrite(low, LOW);
			analogWrite(pmw, motorspeed);
		}
		else if (speed<0)
		{
			speed -= 50;
			motorspeed = (speed < -255) ? -255 : speed;
			digitalWrite(low, HIGH);
			digitalWrite(high, LOW);
			analogWrite(pmw, -motorspeed);
		}
		else
		{
			motorspeed = 0;
			analogWrite(pmw, motorspeed);
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
Motor motorL = Motor(5, 4, 2, A1);

//�Ҳ���
Motor motorR = Motor(3, A2, A3, A0);

//�����⴫����
class LightSensor
{
public:
	//���캯��
	LightSensor(pin pin_direction)
	{
        pinDirection = pin_direction;
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

	//��ֻ���ö��������£��ҵ�����ǿ����������Ӧ��ǿ
	uint16_t FindBright()
	{
		int16_t i = 0, j = 0;
		uint16_t maxLux = 0;
		for (i = -12; i <= 12; i++)
		{
			SetDirection(i * 10);
			delay(100);
			j = GetLuxL();
			if (j > maxLux) maxLux = j;
		};
		return maxLux;
	};

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
    pin pinDirection;
	int8_t direction;
};

//�����⴫����
LightSensor lightSensor = LightSensor(6);

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
		pinMode(echo, INPUT);
		analogWrite(trig,128);
	}

	//���Ծ��룬��λΪ um
	unsigned long GetDistance()
	{
        pulseIn(echo, HIGH);
		return pulseIn(echo, HIGH)* 170;
	}

private:
	pin trig, echo;
};

//ǰ�����봫����
DistanceSensor distanceF = DistanceSensor(9, 8);

//�����봫����
DistanceSensor distanceL = DistanceSensor(10, 11);

//�Ҳ���봫����
DistanceSensor distanceR = DistanceSensor(12, 13);

#endif

#ifndef _WH_SUBMODULES_
#define _WH_SUBMODULES_

//������������
void Test(byte message)
{
	char out[20] = {0};
	sprintf(out, "OK\n%#X\n", message);
	blueTeeth.Print(out);
}

//����ң��
class ModeRemoteCtrl
{
public:
    ModeRemoteCtrl()
    {
    }
    void begin(byte message)
    {
        _speed = message << 4;
        speed = 2 * _speed;
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

private:
    int8_t _speed;
    int16_t speed;
};

ModeRemoteCtrl modeRemoteCtrl = ModeRemoteCtrl();

//ֱ��ǰ��
class ModeStraight
{
public:
    void begin(byte message)
    {
        motorL.SetSpeed(100); motorR.SetSpeed(100);
        motorL.WaitDistance(100);
        motorL.SetSpeed(0); motorR.SetSpeed(0);
    }
};

ModeStraight modeStraight = ModeStraight();

#define threshold_dist 100

//Ѱ��ģʽ
class ModeLight
{
public:
    void begin(byte message)
    {
        while (1)
        {
            //׷��ģʽ
            seek_bright();
            go();
            while (if_void(distanceF))
            {
                delay(10);
            }; delay(100); stop();
            //�����ϰ�,�������ģʽ
            while (nb <= 2/*��ֹ���������ѭ��,ǿ�����½���׷��ģʽ*/)
            {
                seek_void(); go();
                changed_l = false; changed_r = false; void_f = true;
                start_l = if_void(distanceL); start_r = if_void(distanceR);
                // go till arriving a new key position
                while ((!changed_l) && (!changed_r) && void_f)
                {
                    changed_l = if_changed(distanceL, start_l);
                    changed_r = if_changed(distanceR, start_r);
                    void_f = if_void(distanceF);
                };
                stop();
                //����ʧ��,��������ģʽ
                if (!void_f) nb++;
                //���ϳɹ�,���½���׷��ģʽ
                if (changed_l || changed_r) break;
            };
        };
    }

private:
    //�泯����ǿ����
    void seek_bright()
    {
        //
        motorL.SetSpeed(-150); motorR.SetSpeed(-150); delay(100); motorL.SetSpeed(0); motorR.SetSpeed(0);
        //360��Ѱ������ǿ
        int box = 0, maxLux = 0;
        maxLux = lightSensor.FindBright();
        //turn  120deg
        motorL.SetSpeed(-150); motorR.SetSpeed(150); delay(3000); motorL.SetSpeed(0); motorR.SetSpeed(0);
        box = lightSensor.FindBright();
        if (box>maxLux) maxLux = box;
        //turn again 120deg
        motorL.SetSpeed(-150); motorR.SetSpeed(150); delay(3000); motorL.SetSpeed(0); motorR.SetSpeed(0);
        box = lightSensor.FindBright();
        if (box>maxLux) maxLux = box;

        //С����������ǿ��
        motorL.SetSpeed(-150); motorR.SetSpeed(150);
        while (lightSensor.GetLuxL() < maxLux - 5)
        {
        };
        motorL.SetSpeed(0); motorR.SetSpeed(0);
    };

    //�Ƿ����ϰ�
    bool if_void(DistanceSensor distance)
    {
        return distance.GetDistance() > threshold_dist;
    }

    bool if_changed(DistanceSensor distance, bool start)
    {
        if (start) return false;
        return if_void(distance);
    }

    //�泯��������ϰ�����
    void seek_void()
    {
        //
        motorL.SetSpeed(-150); motorR.SetSpeed(-150); delay(100); motorL.SetSpeed(0); motorR.SetSpeed(0);
        //
        motorL.SetSpeed(-150); motorR.SetSpeed(150);
        while (!if_void(distanceF))
        {
            delay(10);
        };
        delay(100);	motorL.SetSpeed(0); motorR.SetSpeed(0);
    }

    void go()
    {
        motorL.SetSpeed(150); motorR.SetSpeed(150);
    };

    void stop()
    {
        motorL.SetSpeed(0); motorR.SetSpeed(0);
    };

    int nb = 0;
    bool box, re, changed_l, changed_r, void_f, start_l, start_r;
};

ModeLight modeLight = ModeLight();

//����״̬������
void SendState(byte message)
{
	if (bitRead(message, 5))
	{
		//lightSensor
		char out5[50] = {0};
		sprintf(out5, "L:\n\td: %d\n\tl: %d\n\n", lightSensor.GetDirection(), lightSensor.GetLuxL());
		blueTeeth.Print(out5);
	}
	if (bitRead(message, 4))
	{
		//motorL
		char out4[15] = {0};
		sprintf(out4, "LM: %d\n\n", motorL.GetSpeed());
		blueTeeth.Print(out4);
	}
	if (bitRead(message, 3))
	{
		//motolR
		char out3[15] = {0};
		sprintf(out3, "RM: %d\n\n", motorR.GetSpeed());
		blueTeeth.Print(out3);
	}
	if (bitRead(message, 2))
	{
		//distanceF
		char out2[25] = {0};
		sprintf(out2, "FD: %lu\n\n", distanceF.GetDistance());
		blueTeeth.Print(out2);
	}
	if (bitRead(message, 1))
	{
		//distanceL
		char out1[25] = {0};
		sprintf(out1, "LD: %lu\n\n", distanceL.GetDistance());
		blueTeeth.Print(out1);
	}
	if (bitRead(message, 0))
	{
		//distanceR
		char out0[25] = {0};
		sprintf(out0, "RD: %lu\n\n", distanceR.GetDistance());
		blueTeeth.Print(out0);
	}
}

#endif

#ifndef _WH_MAINMODULE
#define _WH_MAINMODULE

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

//#define DEBUG
#ifndef DEBUG

void setup()
{
	Init();
}

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
						modeRemoteCtrl.begin(message);
						break;
					case 0x02:
						modeStraight.begin(message);
						break;
					case 0x03:
						modeLight.begin(message);
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

void setup()
{
	Init();
	Serial.begin(9600);
}

void loop()
{
	Serial.println(distanceF.GetDistance());
}

#endif

#endif