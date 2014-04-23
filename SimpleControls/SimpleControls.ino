/*

Copyright (c) 2012, 2013 RedBearLab

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*/

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

#include <ble_mini.h>
#include <Servo.h> 
 
//#define DIGITAL_OUT_PIN    4
//#define DIGITAL_IN_PIN     5
//#define PWM_PIN            6
//#define SERVO_PIN          7
//#define ANALOG_IN_PIN      A5

Servo myservo;

unsigned long currentMillis;        // store the current value from millis()
unsigned long previousMillis;       // for comparison with currentMillis
int samplingInterval = 250;          // how often to run the main loop (in ms)

void setup()
{
  BLEMini_begin(57600);
  
//  pinMode(DIGITAL_OUT_PIN, OUTPUT);
//  pinMode(DIGITAL_IN_PIN, INPUT);
  
  // Default to internally pull high, change it if you need
//  digitalWrite(DIGITAL_IN_PIN, HIGH);
  //digitalWrite(DIGITAL_IN_PIN, LOW);
  
//  myservo.attach(SERVO_PIN);

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

void loop()
{
  /* -------------------------------------------display part-------------------------------------------------- */
  //if(Shift_Count == (Num_Of_Word+Display_Num_Word)*16 )				//移动次数
  if(Shift_Count == (Display_Num_Word)*16 )				//移动次数
  {
    //Shift_Count = 0;
    Display(Display_Swap_Buffer);
  }	
  else
  {
    unsigned int i;
    //for(i = 0;i<3;i++)    // 调节显示字符的移动速度，by tomxue
    {
      Display(Display_Swap_Buffer);
    }
    Display_Word_Count = Shift_Count/16;				//计算当前显示第几个字
    Calc_Shift();										
  
    Shift_Count++;
  }
  /* -------------------------------------------display part-------------------------------------------------- */

  static boolean analog_enabled = false;
  static byte old_state = LOW;
  
  // If data is ready
  while ( BLEMini_available() == 3 )
  {
    // read out command and data
    byte data0 = BLEMini_read();
    byte data1 = BLEMini_read();
    byte data2 = BLEMini_read();
    
    if (data0 == 0x01)  // Command is to control digital out pin
    {
//      if (data1 == 0x01)
//        digitalWrite(DIGITAL_OUT_PIN, HIGH);
//      else
//        digitalWrite(DIGITAL_OUT_PIN, LOW);
        
      showContent = 1;
      Clear_All();
    }
    else if (data0 == 0xA0) // Command is to enable analog in reading
    {
      if (data1 == 0x01)
        analog_enabled = true;
      else
        analog_enabled = false;

      showContent = 0;
      Clear_All();
    }
    else if (data0 == 0x02) // Command is to control PWM pin
    {
//      analogWrite(PWM_PIN, data1);

      showContent = 2;
      Clear_All();
    }
    else if (data0 == 0x03)  // Command is to control Servo pin
    {
      myservo.write(data1);
    }
    else if (data0 == 0x04) // Command is to reset
    {
      analog_enabled = false;
      myservo.write(0);
//      analogWrite(PWM_PIN, 0);
//      digitalWrite(DIGITAL_OUT_PIN, LOW);
    }
  }
  
//  if (analog_enabled)  // if analog reading enabled
//  {
//    currentMillis = millis();
//    if (currentMillis - previousMillis > samplingInterval)
//    {
//      previousMillis += millis();
//      
//      // Read and send out
//      uint16_t value = analogRead(ANALOG_IN_PIN); 
//      BLEMini_write(0x0B);
//      BLEMini_write(value >> 8);
//      BLEMini_write(value);
//    }
//  }
  
  // If digital in changes, report the state
//  if (digitalRead(DIGITAL_IN_PIN) != old_state)
//  {
//    old_state = digitalRead(DIGITAL_IN_PIN);
//    
//    if (digitalRead(DIGITAL_IN_PIN) == HIGH)
//    {
//      BLEMini_write(0x0A);
//      BLEMini_write(0x01);
//      BLEMini_write(0x00);    
//    }
//    else
//    {
//      BLEMini_write(0x0A);
//      BLEMini_write(0x00);
//      BLEMini_write(0x00);
//    }
//  }
  
//  delay(100);
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
