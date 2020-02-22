/*Title: PWM Speed Control*/
/*Author: https://wiki.dfrobot.com/Arduino_Motor_Shield__L298N___SKU_DRI0009_*/
/*Description: PWM Speed Control*/

/*Arduino PWM Speed Control*/
/*1. Left*/
/*2. Right*/
int E1 = 5;
int M1 = 4;
int E2 = 6;
int M2 = 7;

int value = 130;

/* 
 * ROTARY ENCODER PINS (CHANGE ACCORDINGLY)
 */
#define ENCODER_COUNT_RIGHT_UP 18
#define ENCODER_COUNT_RIGHT_DOWN 19
#define ENCODER_COUNT_LEFT_UP 3
#define ENCODER_COUNT_LEFT_DOWN 2

/*
 * This variable will increase or decrease depending on the 
 * rotation of encoder
 */
volatile signed int TEMP_LEFT, COUNTER_LEFT = 0; 
volatile signed int TEMP_RIGHT, COUNTER_RIGHT = 0;

/*Direction Control*/
/*Backward*/
/*digitalWrite(M1,HIGH);*/
/*digitalWrite(M2, HIGH);*/
/*Forward*/
/*digitalWrite(M1,LOW);*/
/*digitalWrite(M2, LOW);*/
/*Left*/
/*digitalWrite(M1,HIGH);*/
/*digitalWrite(M2, LOW);*/
/*Right*/
/*digitalWrite(M1,LOW);*/
/*digitalWrite(M2, HIGH);*/

void setup()
{
  /*
   * FOR SERIAL DATA TRANSMISSION
   */
  Serial.begin (9600);

  /*
   * SETTING UP PINMODE
   * 1. PWM
   * 2. DIRECTION
   */
  pinMode(ENCODER_COUNT_RIGHT_UP, INPUT_PULLUP);
  pinMode(ENCODER_COUNT_RIGHT_DOWN, INPUT_PULLUP);
  pinMode(ENCODER_COUNT_LEFT_UP, INPUT_PULLUP);
  pinMode(ENCODER_COUNT_LEFT_DOWN, INPUT_PULLUP);
  
  /*
   * SETTING UP INTERRUPT
   */
  attachInterrupt(digitalPinToInterrupt(ENCODER_COUNT_LEFT_UP), 
  COUNT_INTERRUPT_LEFT_CW, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_COUNT_LEFT_DOWN), 
  COUNT_INTERRUPT_LEFT_CCW, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_COUNT_RIGHT_UP), 
  COUNT_INTERRUPT_RIGHT_CW, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_COUNT_RIGHT_DOWN), 
  COUNT_INTERRUPT_RIGHT_CCW, RISING);
  
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
}

void loop()
{
//  int value;
//  for(value = 0 ; value <= 255; value+=5)
  {
/*    Direction Control*/
    digitalWrite(M1,HIGH);
    digitalWrite(M2,HIGH);
/*    PWM Speed Control*/
    analogWrite(E1, value);
    analogWrite(E2, value);

    // Send the value of counter
    Serial.print ("COUNTER_LEFT");
    Serial.print (":");
    Serial.print (COUNTER_LEFT);
    Serial.print (":");
    Serial.print ("COUNTER_RIGHT");
    Serial.print (":");
    Serial.print (COUNTER_RIGHT);
    Serial.println();
    
  }
}

  /*
 * ENCODER COUNTER FUNCTION
 */
void COUNT_INTERRUPT_LEFT_CW() 
{
  if(digitalRead(ENCODER_COUNT_LEFT_DOWN)==LOW) 
  {
    COUNTER_LEFT++;
  }
  else
  {
    COUNTER_LEFT--;
  }
}
   
void COUNT_INTERRUPT_LEFT_CCW()
{
  if(digitalRead(ENCODER_COUNT_LEFT_UP)==LOW) 
  {
    COUNTER_LEFT--;
  }
  else
  {
    COUNTER_LEFT++;
  }
}

void COUNT_INTERRUPT_RIGHT_CW() 
{
  if(digitalRead(ENCODER_COUNT_RIGHT_DOWN)==LOW) 
  {
    COUNTER_RIGHT++;
  }
  else
  {
    COUNTER_RIGHT--;
  }
}
   
void COUNT_INTERRUPT_RIGHT_CCW() 
{
  if(digitalRead(ENCODER_COUNT_RIGHT_UP)==LOW) 
  {
    COUNTER_RIGHT--;
  }
  else
  {
    COUNTER_RIGHT++;
  }
}
