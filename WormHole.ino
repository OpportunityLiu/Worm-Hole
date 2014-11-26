int PMW_left = 6;
int Motor_left_H = 5;
int Motor_left_L = 7;
int PMW_right = 3;
int Motor_right_H = 2;
int Motor_right_L = 4;

void setup() {               
	pinMode(Motor_left_H,OUTPUT);
	pinMode(Motor_left_L,OUTPUT);
	pinMode(Motor_right_H,OUTPUT);
	pinMode(Motor_right_L,OUTPUT);
	analogWrite(PMW_left,100);
	analogWrite(PMW_right,200);
	digitalWrite(Motor_left_H,HIGH);
	digitalWrite(Motor_left_L,LOW);
	digitalWrite(Motor_right_H,HIGH);
	digitalWrite(Motor_right_L,LOW);
}

void loop() {

}

