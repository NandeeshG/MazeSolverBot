//EXAMPLE To control motor

// connect motor controller pins to Arduino digital pins
// motor one
int enA = 10;  //with PWM
int inA1 = 9;  //digital
int inA2 = 8;  //digital
// motor two
int enB = 5;  //with PWM
int inB1 = 7; //digital
int inB2 = 6; //digital

void setup()
{
  // set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(inA1, OUTPUT);
  pinMode(inA2, OUTPUT);
  pinMode(inB1, OUTPUT);
  pinMode(inB2, OUTPUT);
}

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

void move1()
{
  // this function will run the motors in both directions at a fixed speed
  motorA(200);
  motorB(200);
  delay(2000);

  // now change motor directions
  motorA(-200);
  motorB(-200);
  delay(2000);

  // now turn off motors
  motorA();
  motorB();
}

void move2()
{
  // this function will run the motors across the range of possible speeds
  // note that maximum speed is determined by the motor itself and the operating voltage
  // the PWM values sent by analogWrite() are fractions of the maximum speed possible 
  // by your hardware
  // turn on motors

  // accelerate from zero to maximum speed
  for(int i=0; i<256; ++i)
  {
    motorA(i);
    motorB(i);
    delay(20);
  }
  // decelerate from maximum speed to zero
  for(int i=255; i>=0; --i)
  {
    motorA(i);
    motorB(i);
    delay(20);
  }
  // now turn off motors
  motorA();
  motorB();
}

void loop()
{
  move1();
  delay(1000);
  move2();
  delay(1000);
}


