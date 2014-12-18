
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
//Motor motorL = Motor(6, A4, A5, A1);

//�Ҳ���
//Motor motorR = Motor(3, 2, 4, A0);
Motor motorR = Motor(3, A2, A3, A0);
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
	const pin pinDirection = 6;
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
		pinMode(echo, INPUT);
		analogWrite(trig,128);
	}

	//���Ծ��룬��λΪ um
	unsigned long GetDistance()
	{
		int r;
		digitalWrite(trig, LOW);
		digitalWrite(trig, HIGH);
		r= pulseIn(echo, HIGH) ;
		digitalWrite(trig, LOW);
		return r* 170;
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
	int16_t speed = 2 * _speed;
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

#define threshold_dist 100

//�泯����ǿ����
void seek_bright()
{
	//360��Ѱ������ǿ
	int box = 0, maxLux = 0;
	maxLux = lightSensor.FindBright();
	//turn
	motorL.SetSpeed(-150); motorR.SetSpeed(150); delay(3000); motorL.SetSpeed(0); motorR.SetSpeed(0);
	box = lightSensor.FindBright();
	if (box>maxLux) maxLux = box;
	//turn again
	motorL.SetSpeed(-150); motorR.SetSpeed(150); delay(3000); motorL.SetSpeed(0); motorR.SetSpeed(0);
	box = lightSensor.FindBright();
	if (box>maxLux) maxLux = box;

	//С����������ǿ��
	motorL.SetSpeed(-150); motorR.SetSpeed(150);
	while (lightSensor.GetLuxL() < maxLux - 5) {};
	motorL.SetSpeed(0); motorR.SetSpeed(0);
};

//�泯��������ϰ�����
void seek_void()
{
	motorL.SetSpeed(-150); motorR.SetSpeed(150);
	while (!front_ok()) {};
	motorL.SetSpeed(0); motorR.SetSpeed(0);
}

//ǰ���Ƿ����ϰ�
bool front_ok()
{
	return distanceF.GetDistance() > threshold_dist;
}

//�����Ƿ�
bool side_ok(){};

void go(){};
void stop(){};

//Ѱ�� AI
void ModeFindLight(byte message)
{
	int nb = 0;
	bool box,re,void_l,void_r,void_f;
	while(1)
	{
		//׷��ģʽ
		seek_bright();
		go();
		while (front_ok()) { delay(10); }; stop();
		//�����ϰ�,�������ģʽ
		while (1)
		{
			seek_void(); go();
			void_l = false; void_r = true; void_f = true;
			while((!void_l) && (!void_r) && void_f)
			{
				void_l = side_ok(/*left*/);
				void_r = side_ok(/*right*/);
				void_f = front_ok();
			};
			stop();
			//����ʧ��,��������ģʽ
			if (!void_f) 
			{
				//��ֹ���������ѭ��,ǿ�����½���׷��ģʽ
				if (nb>2) 
				{
					nb = 0;
					break;
				};
				nb++; 
			}
			//���ϳɹ�,���½���׷��ģʽ
			if (void_l||void_r) break;
		};
	};


	//}
	//   
}

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

#define DEBUG
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