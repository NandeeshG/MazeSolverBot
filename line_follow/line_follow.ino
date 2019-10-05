//Solve Maze Bot
//TEAM - Aman Jain, Naman Arora, Namit Jain, Nandeesh Gupta

// connect motor controller pins to Arduino digital pins
// motor one
const int enA = 9;  //with PWM.  RIGHT MOTOR
const int inA1 = 5;  //digital
const int inA2 = 6;  //digital
// motor two
const int enB = 10;  //with PWM
const int inB1 = 7; //digital
const int inB2 = 8; //digital
const int lineFollowSensor0 = 2;
const int lineFollowSensor1 = 3;
const int lineFollowSensor2 = 4;
const int lineFollowSensor3 = 11;
const int lineFollowSensor4 = 12;
const int buttonPin = A0;
const int ledPin = 0;
int LFSensor[5]={0, 0, 0, 0, 0};
int error=0,P=0,I=0,D=0,previousError=0,PIDvalue=0;
int iniMotorPower=150;
const int Kp=25, Ki=0, Kd=0;
const int STOPPED = 0;
const int FOLLOWING_LINE = 1;
const int NO_LINE = 2;
int mode = FOLLOWING_LINE;

void motorA(int speed=0)
{
    // turn on motor A
    if(speed>0)
    {
        digitalWrite(inA1, HIGH);
        digitalWrite(inA2, LOW);
    }
    else if(speed<0)
    {
        digitalWrite(inA1, LOW);
        digitalWrite(inA2, HIGH);
    }
    else
    {
        digitalWrite(inA1, LOW);
        digitalWrite(inA2, LOW);
    }
    // set speed to speed out of possible range 0~255
    analogWrite(enA, abs(speed)%256);
}

void motorB(int speed=0)
{
    if(speed>0)
    {
        digitalWrite(inB1, HIGH);
        digitalWrite(inB2, LOW);
    }
    else if(speed<0)
    {
        digitalWrite(inB1, LOW);
        digitalWrite(inB2, HIGH);
    }
    else
    {
        digitalWrite(inB1, LOW);
        digitalWrite(inB2, LOW);
    }
    analogWrite(enB, abs(speed)%256);
}

void botLeft(int time=0)
{
    motorA(-(255));
    motorB(255);
    delay(time);
    motorA();
    motorB();
}

void botRight(int time=0)
{
    motorA((255));
    motorB(-(255));
    delay(time);
    motorA();
    motorB();
}   

void readSensors()
{
    //when bot is in perfect position, array is 00100
    //this means IR returns 1 when line below is WHITE
    // 0 is leftmost for bot
    // 4 is rightmost for bot
    LFSensor[0] = digitalRead(lineFollowSensor0);
    LFSensor[1] = digitalRead(lineFollowSensor1);
    LFSensor[2] = digitalRead(lineFollowSensor2);
    LFSensor[3] = digitalRead(lineFollowSensor3);
    LFSensor[4] = digitalRead(lineFollowSensor4);

    /*
    if((LFSensor[0]== 1 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 1 )
        &&(LFSensor[3]== 1 )&&(LFSensor[4]== 1 )) 
    {
        mode = STOPPED;
        return;
    }
    else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )
        &&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 )) 
    { 
        mode = NO_LINE;
        return;
    }
    */

    if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0)
        &&(LFSensor[3]== 0 )&&(LFSensor[4]== 1 ))
        error = 4;
    else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0)
        &&(LFSensor[3]== 1 )&&(LFSensor[4]== 1 )) 
        error = 3; 
    else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0)
        &&(LFSensor[3]== 1 )&&(LFSensor[4]== 0 )) 
        error = 2;
    else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 1)
        &&(LFSensor[3]== 1 )&&(LFSensor[4]== 0 )) 
        error = 1;
    else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 1)
        &&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 )) 
        error = 0;
    else if((LFSensor[0]== 0 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 1)
        &&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 )) 
        error =- 1;
    else if((LFSensor[0]== 0 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 0)
        &&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 )) 
        error = -2;
    else if((LFSensor[0]== 1 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 0)
        &&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 )) 
        error = -3;
    else if((LFSensor[0]== 1 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0)
        &&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 )) 
        error = -4;
    else if((LFSensor[0]== 0 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 1)
        &&(LFSensor[3]== 1 )&&(LFSensor[4]== 0 ))
        error = 0;
    mode = FOLLOWING_LINE;
}

void calculatePID()
{
    P = error;
    I = I + error;
    D = error-previousError;
    PIDvalue = (Kp*P) + (Ki*I) + (Kd*D);
    previousError = error;
}

void motorPIDcontrol()
{
    int rightMotorSpeed = iniMotorPower - PIDvalue;
    int leftMotorSpeed = iniMotorPower + PIDvalue;
    motorA(rightMotorSpeed);
    motorB(leftMotorSpeed);
}

void ledBlink(int t)
{
  for(int i=0; i<3; ++i)
  {
  digitalWrite(ledPin, HIGH);
  delay(t*100);
  digitalWrite(ledPin, LOW);
  delay(t*100);
  }
}

void setup()
{
    // set all the motor control pins to outputs
    pinMode(enA, OUTPUT);
    pinMode(enB, OUTPUT);
    pinMode(inA1, OUTPUT);
    pinMode(inA2, OUTPUT);
    pinMode(inB1, OUTPUT);
    pinMode(inB2, OUTPUT);
    // led and buttons
    pinMode(ledPin, OUTPUT);
    pinMode(A0, INPUT_PULLUP);

    ledBlink(5);
    //infinite loop until button reads 0
    while(digitalRead(A0))
    {  
      digitalWrite(ledPin, HIGH);
    }
    ledBlink(5);
}

// the loop function runs over and over again forever
void loop()
{
    readSensors();
    calculatePID();
    motorPIDcontrol();
    delay(50);
    /*
    switch (mode)
    {
      case STOPPED:
         motorA();
         motorB();
         break;
      case NO_LINE:
         motorA();
         motorB();
         botLeft(180); 
         break;
      case FOLLOWING_LINE:
         calculatePID();
         motorPIDcontrol();
         break;
    }
    */
}
