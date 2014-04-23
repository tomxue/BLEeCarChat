/*
 *  When use DUE board, the Arduino IDE should be the version of 1.5.4 or above.
 *  Board       BLEMini(TX, RX) 
 *  DUE          (18, 19)
 *  MEGA         (18, 19)
 *  UNO          (1, 0)
 *  LEONARDO     (1, 0)
 */

#include <Servo.h>
#include "Boards.h"
#include <ble_mini.h>

/* -------------------------------------------display part-------------------------------------------------- */
#include <Arduino.h>
//IO配置
#define LEDARRAY_D 2
#define LEDARRAY_C 3
#define LEDARRAY_B 4
#define LEDARRAY_A 5
#define LEDARRAY_G 6
#define LEDARRAY_DI 7
#define LEDARRAY_CLK 8
#define LEDARRAY_LAT 9

#define led 13


#define Display_Num_Word 2				//液晶能显示的汉字个数

unsigned char Display_Buffer[4];
unsigned char Display_Swap_Buffer[Display_Num_Word][32]={
  0};					//显示缓冲区
bool Shift_Bit = 0;
bool Flag_Shift = 0;
unsigned char Timer0_Count = 0;
unsigned char temp = 0x80;
unsigned char Shift_Count = 0;
unsigned char Display_Word_Count = 0;
unsigned char showContent = 0; // defined by tomxue
#define Num_Of_Word 2
const unsigned char Word[Num_Of_Word][32] = 
{

  0xFE,0xFE,0xFE,0xFE,0xF6,0xF2,0xE6,0xEE,0xDE,0xBE,0x7E,0xFE,0xFE,0xFE,0xFA,0xFD,
  0xFF,0xFF,0xFF,0xFF,0xBF,0xDF,0xEF,0xE7,0xF3,0xF9,0xFB,0xFF,0xFF,0xFF,0xFF,0xFF,/*"小",0*/

  0xFF,0x86,0xF6,0xF6,0x86,0xBF,0xBC,0xBD,0x85,0xF5,0xF4,0xF7,0xF7,0xF7,0xD7,0xEC,
  0xFF,0x07,0xF7,0xF7,0x07,0xBF,0x03,0xBB,0xBB,0xBB,0x03,0xBF,0xB7,0xBB,0x81,0x3B,/*"强",1*/
};

const unsigned char Word2[Num_Of_Word][32] = 
{
  0xEF,0xEF,0xC2,0xDE,0xBE,0x42,0xEE,0xEF,0x03,0xEC,0xED,0xED,0xE9,0xE5,0xEF,0xFF,
  0xDF,0xBF,0x03,0xFB,0x03,0xFB,0x03,0xDF,0xDF,0x01,0xDD,0xDD,0xD5,0xDB,0xDF,0xDF,/*"锦",0*/

  0xEF,0xEF,0xC3,0xDF,0xBE,0x43,0xEF,0xEC,0x03,0xEF,0xEF,0xEF,0xE8,0xE7,0xEF,0xFF,
  0xBF,0xBF,0x03,0x77,0xAF,0xDF,0x27,0xD9,0x07,0xDF,0x07,0xDF,0x01,0xDF,0xDF,0xDF,/*"锋",1*/

};

const unsigned char Word3[Num_Of_Word][32] = 
{
  0xFE,0x06,0xF,0xF6,0xF6,0x86,0xBE,0xB8,0xBE,0x86,0xF6,0xF6,0xF6,0xF6,0xAE,0xDE,
  0xFF,0xF7,0xF7,0xEF,0xDF,0xBF,0xFF,0x01,0xBF,0xDF,0xDF,0xEF,0xF7,0xBB,0x7D,0xFF,/*"张",0*/

  0xFE,0xFE,0x06,0xDE,0xD8,0xDE,0xDE,0x04,0xDC,0xDA,0xDA,0xC6,0x1E,0xBE,0xFE,0xFE,
  0xEF,0xEF,0xEF,0xEF,0x43,0xEF,0xEF,0xC7,0x47,0xAB,0xAB,0x6D,0xEF,0xEF,0xEF,0xEF,/*"琳",1*/

};


/* -------------------------------------------display part-------------------------------------------------- */


#define PROTOCOL_MAJOR_VERSION   0 //
#define PROTOCOL_MINOR_VERSION   0 //
#define PROTOCOL_BUGFIX_VERSION  2 // bugfix

#define PIN_CAPABILITY_NONE      0x00
#define PIN_CAPABILITY_DIGITAL   0x01
#define PIN_CAPABILITY_ANALOG    0x02
#define PIN_CAPABILITY_PWM       0x04
#define PIN_CAPABILITY_SERVO     0x08
#define PIN_CAPABILITY_I2C       0x10

// pin modes
//#define INPUT                 0x00 // defined in wiring.h
//#define OUTPUT                0x01 // defined in wiring.h
#define ANALOG                  0x02 // analog pin in analogInput mode
#define PWM                     0x03 // digital pin in PWM output mode
#define SERVO                   0x04 // digital pin in Servo output mode

byte pin_mode[TOTAL_PINS];
byte pin_state[TOTAL_PINS];
byte pin_pwm[TOTAL_PINS];
byte pin_servo[TOTAL_PINS];

Servo servos[MAX_SERVOS];

/* timer variables */
unsigned long currentMillis;        // store the current value from millis()
unsigned long previousMillis;       // for comparison with currentMillis
int samplingInterval = 5;          // how often to run the main loop (in ms)

void setup()
{
  BLEMini_begin(57600);

#if !defined(__AVR_ATmega328P__)
  Serial.begin(57600);
  //while(!Serial);  // Enable this line if you want to debug on Leonardo
  Serial.println("BLE Arduino Slave ");
#endif

  /* Default all to digital input */
  for (int pin = 0; pin < TOTAL_PINS; pin++)
  {
    // Set pin to input with internal pull up
    if(IS_PIN_DIGITAL(pin))
    {
      pinMode(pin, INPUT);
    }      
    digitalWrite(pin, HIGH);
    // Save pin mode and state
    pin_mode[pin] = INPUT;
    pin_state[pin] = LOW;
  } 

  /* -------------------------------------------display part-------------------------------------------------- */
  pinMode(LEDARRAY_D, OUTPUT); 
  pinMode(LEDARRAY_C, OUTPUT);
  pinMode(LEDARRAY_B, OUTPUT);
  pinMode(LEDARRAY_A, OUTPUT);
  pinMode(LEDARRAY_G, OUTPUT);
  pinMode(LEDARRAY_DI, OUTPUT);
  pinMode(LEDARRAY_CLK, OUTPUT);
  pinMode(LEDARRAY_LAT, OUTPUT);

  Clear_Display();
  /* -------------------------------------------display part-------------------------------------------------- */
}

static byte buf_len = 0;

byte reportDigitalInput()
{
  static byte pin = 0;
  byte report = 0;

  if (!IS_PIN_DIGITAL(pin))
  {
    pin++;
    if (pin >= TOTAL_PINS)
      pin = 0;
    return 0;
  }

  if (pin_mode[pin] == INPUT)
  {
    byte current_state = digitalRead(pin);

    if (pin_state[pin] != current_state)
    {
      pin_state[pin] = current_state;
      byte buf[] = {
        'G', pin, INPUT, current_state                                          };
      BLEMini_write_bytes(buf, 4);

      report = 1;
    }
  }

  pin++;
  if (pin >= TOTAL_PINS)
    pin = 0;

  return report;
}

void reportPinCapability(byte pin)
{
  byte buf[] = {
    'P', pin, 0x00              };
  byte pin_cap = 0;

  if (IS_PIN_DIGITAL(pin))
    pin_cap |= PIN_CAPABILITY_DIGITAL;

  if (IS_PIN_ANALOG(pin))
    pin_cap |= PIN_CAPABILITY_ANALOG;

  if (IS_PIN_PWM(pin))
    pin_cap |= PIN_CAPABILITY_PWM;

  if (IS_PIN_SERVO(pin))
    pin_cap |= PIN_CAPABILITY_SERVO;

  buf[2] = pin_cap;
  BLEMini_write_bytes(buf, 3);
}

void reportPinServoData(byte pin)
{
  //  if (IS_PIN_SERVO(pin))
  //    servos[PIN_TO_SERVO(pin)].write(value);
  //  pin_servo[pin] = value;

  byte value = pin_servo[pin];
  byte mode = pin_mode[pin];
  byte buf[] = {
    'G', pin, mode, value              };         
  BLEMini_write_bytes(buf, 4);
}

byte reportPinAnalogData()
{
  static byte pin = 0;
  byte report = 0;

  if (!IS_PIN_DIGITAL(pin))
  {
    pin++;
    if (pin >= TOTAL_PINS)
      pin = 0;
    return 0;
  }

  if (pin_mode[pin] == ANALOG)
  {
    uint16_t value = analogRead(pin);
    byte value_lo = value;
    byte value_hi = value>>8;

    byte mode = pin_mode[pin];
    mode = (value_hi << 4) | mode;

    byte buf[] = {
      'G', pin, mode, value                            };         
    BLEMini_write_bytes(buf, 4);
  }

  pin++;
  if (pin >= TOTAL_PINS)
    pin = 0;

  return report;
}

void reportPinDigitalData(byte pin)
{
  byte state = digitalRead(pin);
  byte mode = pin_mode[pin];
  byte buf[] = {
    'G', pin, mode, state              };         
  BLEMini_write_bytes(buf, 4);
}

void reportPinPWMData(byte pin)
{
  byte value = pin_pwm[pin];
  byte mode = pin_mode[pin];
  byte buf[] = {
    'G', pin, mode, value              };         
  BLEMini_write_bytes(buf, 4);
}

void sendCustomData(uint8_t *buf, uint8_t len)
{
  uint8_t data[20] = "Z";
  memcpy(&data[1], buf, len);
  BLEMini_write_bytes(data, len+1);
}

byte queryDone = false;

void loop()
{
  /* -------------------------------------------display part-------------------------------------------------- */
  unsigned int i;
  for(i = 0;i<3;i++)    // 调节显示字符的移动速度，by tomxue
  {
    Display(Display_Swap_Buffer);
  }
  Display_Word_Count = Shift_Count/16;				//计算当前显示第几个字
  Calc_Shift();										

  Shift_Count++;

  if(Shift_Count == (Num_Of_Word+Display_Num_Word)*16 )				//移动次数
  {
    Shift_Count = 0;				
  }	
  /* -------------------------------------------display part-------------------------------------------------- */


  while(BLEMini_available())
  {
    byte cmd;
    cmd = BLEMini_read();

#if !defined(__AVR_ATmega328P__) // don't print out on UNO
    Serial.write(cmd);
#endif

    // Parse data here
    switch (cmd)
    {
    case 'V': // query protocol version
      {
        queryDone = false;

        byte buf[] = {
          'V', 0x00, 0x00, 0x01                                                        };
        BLEMini_write_bytes(buf, 4);          
      }
      break;

    case 'C': // query board total pin count
      {
        byte buf[2];
        buf[0] = 'C';
        buf[1] = TOTAL_PINS; 
        BLEMini_write_bytes(buf, 2);
      }        
      break;

    case 'M': // query pin mode
      {  
        byte pin = BLEMini_read();
        byte buf[] = {
          'M', pin, pin_mode[pin]                                                        }; // report pin mode
        BLEMini_write_bytes(buf, 3);
      }  
      break;

    case 'S': // set pin mode
      {
        byte pin = BLEMini_read();
        byte mode = BLEMini_read();

        if (IS_PIN_SERVO(pin) && mode != SERVO && servos[PIN_TO_SERVO(pin)].attached())
          servos[PIN_TO_SERVO(pin)].detach();

        /* ToDo: check the mode is in its capability or not */
        /* assume always ok */
        if (mode != pin_mode[pin])
        {              
          pinMode(pin, mode);
          pin_mode[pin] = mode;

          if (mode == OUTPUT)
          {
            digitalWrite(pin, LOW);
            pin_state[pin] = LOW;
            showContent = 1;
            Clear_All();
          }
          else if (mode == INPUT)
          {
            digitalWrite(pin, HIGH);
            pin_state[pin] = HIGH;
            showContent = 0;
            Clear_All();
          }
          else if (mode == ANALOG)
          {
            if (IS_PIN_ANALOG(pin)) {
              if (IS_PIN_DIGITAL(pin)) {
                pinMode(PIN_TO_DIGITAL(pin), LOW);
              }
            }
            showContent = 2;
            Clear_All();
          }
          else if (mode == PWM)
          {
            if (IS_PIN_PWM(pin))
            {
              pinMode(PIN_TO_PWM(pin), OUTPUT);
              analogWrite(PIN_TO_PWM(pin), 0);
              pin_pwm[pin] = 0;
              pin_mode[pin] = PWM;
            }
          }
          else if (mode == SERVO)
          {
            if (IS_PIN_SERVO(pin))
            {
              pin_servo[pin] = 0;
              pin_mode[pin] = SERVO;
              if (!servos[PIN_TO_SERVO(pin)].attached())
                servos[PIN_TO_SERVO(pin)].attach(PIN_TO_DIGITAL(pin));
            }
          }
        }

        //        if (mode == ANALOG)
        //          reportPinAnalogData(pin);
        if ( (mode == INPUT) || (mode == OUTPUT) )
          reportPinDigitalData(pin);
        else if (mode == PWM)
          reportPinPWMData(pin);
        else if (mode == SERVO)
          reportPinServoData(pin);
      }
      break;

    case 'G': // query pin data
      {
        byte pin = BLEMini_read();
        reportPinDigitalData(pin);
      }
      break;

    case 'T': // set pin digital state
      {
        byte pin = BLEMini_read();
        byte state = BLEMini_read();

        digitalWrite(pin, state);
        reportPinDigitalData(pin);
      }
      break;

    case 'N': // set PWM
      {
        byte pin = BLEMini_read();
        byte value = BLEMini_read();

        analogWrite(PIN_TO_PWM(pin), value);
        pin_pwm[pin] = value;
        reportPinPWMData(pin);
      }
      break;

    case 'O': // set Servo
      {
        byte pin = BLEMini_read();
        byte value = BLEMini_read();

        if (IS_PIN_SERVO(pin))
          servos[PIN_TO_SERVO(pin)].write(value);
        pin_servo[pin] = value;
        reportPinServoData(pin);
      }
      break;

    case 'A': // query all pin status
      for (int pin = 0; pin < TOTAL_PINS; pin++)
      {
        reportPinCapability(pin);
        if ( (pin_mode[pin] == INPUT) || (pin_mode[pin] == OUTPUT) )
          reportPinDigitalData(pin);
        else if (pin_mode[pin] == PWM)
          reportPinPWMData(pin);
        else if (pin_mode[pin] == SERVO)
          reportPinServoData(pin);  
      }

      queryDone = true; 

      {
        uint8_t str[] = "ABC";
        sendCustomData(str, 3);
      }

      break;

    case 'P': // query pin capability
      {
        byte pin = BLEMini_read();
        reportPinCapability(pin);
      }
      break;

    case 'Z':
      {
        byte len = BLEMini_read();
        byte buf[len];
        for (int i=0;i<len;i++)
          buf[i] = BLEMini_read();

#if !defined(__AVR_ATmega328P__)  
        Serial.println("->");
        Serial.print("Received: ");
        Serial.print(len);
        Serial.println(" byte(s)");
#endif          
      }
    }

    return; // only do this task in this loop
  }

  // No input data, no commands, process analog data
  //  if (!ble_connected())
  //    queryDone = false; // reset query state

  if (queryDone) // only report data after the query state
  { 
    byte input_data_pending = reportDigitalInput();  
    if (input_data_pending)  
      return; // only do this task in this loop

    currentMillis = millis();
    if (currentMillis - previousMillis > samplingInterval)
    {
      previousMillis += samplingInterval;

      reportPinAnalogData();
    }
  }  
}

/* -------------------------------------------display part-------------------------------------------------- */
void Clear_All()
{
    Shift_Count = 0;
    Display_Word_Count = 0;
    temp = 0x80;
    Clear_Display();
}

//************************************************************
//清空缓冲区
//************************************************************
void Clear_Display()
{
  unsigned char i,j;
  for(j = 0 ; j < Display_Num_Word; j++)
  {
    for(i = 0 ; i < 32 ;i++)
    {
      Display_Swap_Buffer[j][i] = 0xff;				//0=显示  1=不显示 	
    }
  }
}

//************************************************************
//计算移动数据，存在在缓冲器
//************************************************************
void Calc_Shift()
{
  unsigned char i;

  for(i = 0;i < 16;i++)
  {
    if((Display_Swap_Buffer[0][16+i]&0x80) == 0)									//每行第一个字节移位
    {
      Display_Swap_Buffer[0][i] = (Display_Swap_Buffer[0][i] << 1)&0xfe; 			//最低位清零	
    }
    else
    {
      Display_Swap_Buffer[0][i] = (Display_Swap_Buffer[0][i] << 1)|0x01;			//最低位置一 	
    }

    if((Display_Swap_Buffer[1][i]&0x80) == 0)									//每行第二个字节移位
    {
      Display_Swap_Buffer[0][16+i] = (Display_Swap_Buffer[0][16+i] << 1)&0xfe; 			//最低位清零	
    }
    else
    {
      Display_Swap_Buffer[0][16+i] = (Display_Swap_Buffer[0][16+i] << 1)|0x01;			//最低位置一 	
    }

    if((Display_Swap_Buffer[1][16+i]&0x80) == 0)									//每行第三个字节移位
    {
      Display_Swap_Buffer[1][i] = (Display_Swap_Buffer[1][i] << 1)&0xfe; 			//最低位清零	
    }
    else
    {
      Display_Swap_Buffer[1][i] = (Display_Swap_Buffer[1][i] << 1)|0x01;			//最低位置一 	
    }

    if(Shift_Count%16 < 8 && Display_Word_Count < Num_Of_Word)
    {
      if(showContent == 0)
          Shift_Bit = Word[Display_Word_Count][i]&temp;
      else if(showContent == 1)
          Shift_Bit = Word2[Display_Word_Count][i]&temp;
      else if(showContent == 2)
          Shift_Bit = Word3[Display_Word_Count][i]&temp;
    }
    else if(Shift_Count%16 < 16 && Display_Word_Count < Num_Of_Word)
    {
      if(showContent == 0)
          Shift_Bit = Word[Display_Word_Count][16+i]&temp;
      else if(showContent == 1)
          Shift_Bit = Word2[Display_Word_Count][16+i]&temp;
      else if(showContent == 2)
          Shift_Bit = Word3[Display_Word_Count][16+i]&temp;
    }
    else
    {
      Shift_Bit = 1;						//把字移出缓冲区
    }

    if( Shift_Bit == 0)														//后8位移位
    {
      Display_Swap_Buffer[1][16+i] = (Display_Swap_Buffer[1][16+i] << 1)&0xfe; 		//最低位清零
    }
    else
    {
      Shift_Bit = 1;
      Display_Swap_Buffer[1][16+i] = (Display_Swap_Buffer[1][16+i] << 1)|0x01;		//最低位置一 			
    }
  }

  temp = (temp>>1)&0x7f;
  if(temp == 0x00)
  {
    temp = 0x80;
  }
}
//************************************************************
//num为字数  dat[][32]为字模的名称
//*************************************************************
void Display(const unsigned char dat[][32])					
{
  unsigned char i;

  for( i = 0 ; i < 16 ; i++ )
  {
    digitalWrite(LEDARRAY_G, HIGH);		//更新数据时候关闭显示。等更新完数据，打开138显示行。防止重影。

    Display_Buffer[0] = dat[0][i];		
    Display_Buffer[1] = dat[0][i+16];
    Display_Buffer[2] = dat[1][i];
    Display_Buffer[3] = dat[1][i+16];

    Send(Display_Buffer[3]);
    Send(Display_Buffer[2]);
    Send(Display_Buffer[1]);
    Send(Display_Buffer[0]);

    digitalWrite(LEDARRAY_LAT, HIGH);					//锁存数据
    delayMicroseconds(1);

    digitalWrite(LEDARRAY_LAT, LOW);
    delayMicroseconds(1);

    Scan_Line(i);						//选择第i行

    digitalWrite(LEDARRAY_G, LOW);

    delayMicroseconds(300);
    ;		//延时一段时间，让LED亮起来。				
  }	
}

//****************************************************
//扫描某一行
//****************************************************
void Scan_Line( unsigned char m)
{	
  switch(m)
  {
  case 0:			
    digitalWrite(LEDARRAY_D, LOW);
    digitalWrite(LEDARRAY_C, LOW);
    digitalWrite(LEDARRAY_B, LOW);
    digitalWrite(LEDARRAY_A, LOW); 					
    break;
  case 1:					
    digitalWrite(LEDARRAY_D, LOW);
    digitalWrite(LEDARRAY_C, LOW);
    digitalWrite(LEDARRAY_B, LOW);
    digitalWrite(LEDARRAY_A, HIGH); 		
    break;
  case 2:					
    digitalWrite(LEDARRAY_D, LOW);
    digitalWrite(LEDARRAY_C, LOW);
    digitalWrite(LEDARRAY_B, HIGH);
    digitalWrite(LEDARRAY_A, LOW); 		
    break;
  case 3:					
    digitalWrite(LEDARRAY_D, LOW);
    digitalWrite(LEDARRAY_C, LOW);
    digitalWrite(LEDARRAY_B, HIGH);
    digitalWrite(LEDARRAY_A, HIGH); 		
    break;
  case 4:
    digitalWrite(LEDARRAY_D, LOW);
    digitalWrite(LEDARRAY_C, HIGH);
    digitalWrite(LEDARRAY_B, LOW);
    digitalWrite(LEDARRAY_A, LOW); 		
    break;
  case 5:
    digitalWrite(LEDARRAY_D, LOW);
    digitalWrite(LEDARRAY_C, HIGH);
    digitalWrite(LEDARRAY_B, LOW);
    digitalWrite(LEDARRAY_A, HIGH); 		
    break;
  case 6:
    digitalWrite(LEDARRAY_D, LOW);
    digitalWrite(LEDARRAY_C, HIGH);
    digitalWrite(LEDARRAY_B, HIGH);
    digitalWrite(LEDARRAY_A, LOW); 		
    break;
  case 7:
    digitalWrite(LEDARRAY_D, LOW);
    digitalWrite(LEDARRAY_C, HIGH);
    digitalWrite(LEDARRAY_B, HIGH);
    digitalWrite(LEDARRAY_A, HIGH); 		
    break;
  case 8:
    digitalWrite(LEDARRAY_D, HIGH);
    digitalWrite(LEDARRAY_C, LOW);
    digitalWrite(LEDARRAY_B, LOW);
    digitalWrite(LEDARRAY_A, LOW); 		
    break;
  case 9:
    digitalWrite(LEDARRAY_D, HIGH);
    digitalWrite(LEDARRAY_C, LOW);
    digitalWrite(LEDARRAY_B, LOW);
    digitalWrite(LEDARRAY_A, HIGH); 		
    break;	
  case 10:
    digitalWrite(LEDARRAY_D, HIGH);
    digitalWrite(LEDARRAY_C, LOW);
    digitalWrite(LEDARRAY_B, HIGH);
    digitalWrite(LEDARRAY_A, LOW); 		
    break;
  case 11:
    digitalWrite(LEDARRAY_D, HIGH);
    digitalWrite(LEDARRAY_C, LOW);
    digitalWrite(LEDARRAY_B, HIGH);
    digitalWrite(LEDARRAY_A, HIGH); 		
    break;
  case 12:
    digitalWrite(LEDARRAY_D, HIGH);
    digitalWrite(LEDARRAY_C, HIGH);
    digitalWrite(LEDARRAY_B, LOW);
    digitalWrite(LEDARRAY_A, LOW); 		
    break;
  case 13:
    digitalWrite(LEDARRAY_D, HIGH);
    digitalWrite(LEDARRAY_C, HIGH);
    digitalWrite(LEDARRAY_B, LOW);
    digitalWrite(LEDARRAY_A, HIGH); 		
    break;
  case 14:
    digitalWrite(LEDARRAY_D, HIGH);
    digitalWrite(LEDARRAY_C, HIGH);
    digitalWrite(LEDARRAY_B, HIGH);
    digitalWrite(LEDARRAY_A, LOW); 		
    break;
  case 15:
    digitalWrite(LEDARRAY_D, HIGH);
    digitalWrite(LEDARRAY_C, HIGH);
    digitalWrite(LEDARRAY_B, HIGH);
    digitalWrite(LEDARRAY_A, HIGH); 		
    break;
  default : 
    break;	
  }
}

//****************************************************
//发送数据
//****************************************************
void Send( unsigned char dat)
{
  unsigned char i;
  digitalWrite(LEDARRAY_CLK, LOW);
  delayMicroseconds(1);
  ;	
  digitalWrite(LEDARRAY_LAT, LOW);
  delayMicroseconds(1);
  ;

  for( i = 0 ; i < 8 ; i++ )
  {
    if( dat&0x01 )
    {
      digitalWrite(LEDARRAY_DI, HIGH);	
    }
    else
    {
      digitalWrite(LEDARRAY_DI, LOW);
    }


    digitalWrite(LEDARRAY_CLK, HIGH);				//上升沿发送数据
    delayMicroseconds(1);
    ;
    digitalWrite(LEDARRAY_CLK, LOW);
    delayMicroseconds(1);
    ;		
    dat >>= 1;

  }			
}
