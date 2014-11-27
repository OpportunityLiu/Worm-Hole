
#include "Wire\Wire.h"

class Motor
{
public:
    Motor(uint8_t pin_pmw, uint8_t pin_low, uint8_t pin_high)
    {
        pmw = pin_pmw;
        high = pin_high;
        low = pin_low;
        pinMode(pin_low, OUTPUT);
        pinMode(pin_high, OUTPUT);
    }

    //��ȡ�ٶȣ�-255~255
    int getSpeed()
    {
        return speed;
    }

    //�����ٶȣ�-255~255
    void setSpeed(int speed)
    {
        if (speed > 255 || speed < -255)
        {
            return;
        }
        if (speed >= 0)
        {
            digitalWrite(high, HIGH);
            digitalWrite(low, LOW);
            analogWrite(pmw, speed);
        }
        else
        {
            digitalWrite(low, HIGH);
            digitalWrite(high, LOW);
            analogWrite(pmw, -speed);
        }
        speed = this->speed;
    }

private:
    uint8_t pmw, low, high;
	int speed/* = 0*/;
};

Motor* motorL;
Motor* motorR;

void setup()
{
	//�����
	motorL = new Motor(6, 7, 5);
	//�Ҳ���
	motorR = new Motor(3, 4, 2);
    motorL -> setSpeed(100);
    motorR -> setSpeed(200);
}

void loop()
{
}


