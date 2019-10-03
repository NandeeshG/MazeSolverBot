//Solve Maze Bot
//TEAM - Aman Jain, Naman Arora, Namit Jain, Nandeesh Gupta

// connect motor controller pins to Arduino digital pins
// motor one
const int enA = 10;  //with PWM
const int inA1 = 9;  //digital
const int inA2 = 8;  //digital
// motor two
const int enB = 5;  //with PWM
const int inB1 = 7; //digital
const int inB2 = 6; //digital
const int lineFollowSensor0 = 12;
const int lineFollowSensor1 = 18;
const int lineFollowSensor2 = 17;
const int lineFollowSensor3 = 16;
const int lineFollowSensor4 = 19;
const byte buttonPin = 9;
const byte ledPin = 13;
int LFSensor[5]={0, 0, 0, 0, 0};
int error=0,P,I,D,previousError,PIDvalue;
int iniMotorPower;
const int Kp=50, Ki=25, Kd=25;

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

void readSensors()
{
    //when bot is in perfect position, array is 00100
    //this means IR returns 1 when line below is WHITE
    LFSensor[0] = digitalRead(lineFollowSensor0);
    LFSensor[1] = digitalRead(lineFollowSensor1);
    LFSensor[2] = digitalRead(lineFollowSensor2);
    LFSensor[3] = digitalRead(lineFollowSensor3);
    LFSensor[4] = digitalRead(lineFollowSensor4);
    
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
    int leftMotorSpeed = iniMotorPower - PIDvalue;
    int rightMotorSpeed = iniMotorPower - PIDvalue;
    motorA(leftMotorSpeed);
    motorB(rightMotorSpeed);
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
    pinMode(ledPin, OUTPUT);
    pinMode(buttonPin, INPUT_PULLUP);
    
    //infinite loop until button reads 0
    while(digitalRead(buttonPin))
    {  
    }
}

// the loop function runs over and over again forever
void loop()
{
    readLFSsensors(); 
    // read sensors, storage values at Sensor Array and calculate "error"
    calculatePID(); 
    motorPIDcontrol();
}
