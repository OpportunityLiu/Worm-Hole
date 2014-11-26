
#define __BIN_I586__
#define _VMDEBUG 1
#define ARDUINO 153
#define ARDUINO_MAIN
#define printf iprintf
#define __X86__
#define __x86__
#define F_CPU -m32
#define __cplusplus

#include "C:\arduino-1.5.3-Intel.1.0.4\hardware\arduino\x86\cores\arduino\arduino.h"
#include "C:\arduino-1.5.3-Intel.1.0.4\hardware\arduino\x86\variants\galileo_fab_d\pins_arduino.h" 
#include "C:\arduino-1.5.3-Intel.1.0.4\hardware\arduino\x86\variants\galileo_fab_d\variant.h" 

typedef enum
{
      PMW_left = 6,
      Motor_left_H = 5,
      Motor_left_L = 7,
      PMW_right = 3,
      Motor_right_H = 2,
      Motor_right_L = 4
}Pin;

void a()
{
    pinMode(Motor_left_H, OUTPUT);
    pinMode(Motor_left_L, OUTPUT);
    pinMode(Motor_right_H, OUTPUT);
    pinMode(Motor_right_L, OUTPUT);
    analogWrite(PMW_left, 100);
    analogWrite(PMW_right, 200);
    digitalWrite(Motor_left_H, HIGH);
    digitalWrite(Motor_left_L, LOW);
    digitalWrite(Motor_right_H, HIGH);
    digitalWrite(Motor_right_L, LOW);
}

void b()
{

}

