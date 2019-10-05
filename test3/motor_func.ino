// motor one
const int enA = 9;  //with PWM.  RIGHT MOTOR
const int inA1 = 5;  //digital
const int inA2 = 6;  //digital
// motor two
const int enB = 10;  //with PWM
const int inB1 = 7; //digital
const int inB2 = 8; //digital

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

void motorStop()
{
  //leftServo.writeMicroseconds(1500);
  //rightServo.writeMicroseconds(1500);
  motorA();
  motorB();
  delay(200);
}

//--------------------------------------------- 
void motorForward()
{
  //leftServo.writeMicroseconds(1500 - (power+adj));
  motorB((power+adj));
  //rightServo.writeMicroseconds(1500 + power);
  motorA(power);
}

//---------------------------------------------
void motorBackward()
{
  //leftServo.writeMicroseconds(1500 + power);
  motorB(-power);
  //rightServo.writeMicroseconds(1500 - power);
  motorA(-power);
}

//---------------------------------------------
void motorFwTime (unsigned int time)
{
  motorForward();
  delay (time);
  motorStop();
}

//---------------------------------------------
void motorBwTime (unsigned int time)
{
  motorBackward();
  delay (time);
  motorStop();
}

//------------------------------------------------
void motorTurn(int direction, int degrees)
{
  int leftMotorSpeed = (iniMotorPower+adj)*direction;
  int rightMotorSpeed = iniMotorPower*direction;
  
  // The motor speed should not exceed the max PWM value
  //constrain(leftMotorSpeed, 0, 255);
  //constrain(rightMotorSpeed, 0, 255);
  
  motorB(leftMotorSpeed);
  motorA(rightMotorSpeed);
  
  delay (round(adjTurn*degrees));
  motorStop();
}

//---------------------------------------------------
void motorPIDcontrol()
{
  
  int leftMotorSpeed = (iniMotorPower+adj) + PIDvalue;
  int rightMotorSpeed = iniMotorPower - PIDvalue;
  
  // The motor speed should not exceed the max PWM value
  //constrain(leftMotorSpeed, 0, 255);
  //constrain(rightMotorSpeed, 0, 255);
  
  motorB(leftMotorSpeed);
  motorA(rightMotorSpeed);
 
  //Serial.print (PIDvalue);
  //Serial.print (" ==> Left, Right:  ");
  //Serial.print (leftMotorSpeed);
  //Serial.print ("   ");
  //Serial.println (rightMotorSpeed);
}

//---------------------------------------------------
void runExtraInch(void)
{
  motorPIDcontrol();
  delay(extraInch);
  motorStop();
}

//---------------------------------------------------
void goAndTurn(int direction, int degrees)
{
  motorPIDcontrol();
  delay(adjGoAndTurn);
  motorTurn(direction, degrees);
}
