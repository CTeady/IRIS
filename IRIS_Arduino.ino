// Real Time Clock with LED Display

// Writes data to an I2C/TWI RTC slave device (DS1337) and Display Driver (AS1115)

// Created 04 January 2013


#include <Wire.h>
// I2C Slave Address
#define RTC_Address 0x68
#define LEDDisp_Address 0x00
// Rotary encoder pin definitions
#define ENC_A 9
#define ENC_B 10
#define ENC_PORT PINB

//#define buzzer 11
#define ltgup 7
#define ltgoff 5

#define swint 3 //Switch Interrupt
#define rtcint 2 //RTC Interrupt

const int ltgDelay=100;  // Set delay of 100 milliseconds per lighting pulse which equates to about 48 pulses for full brightness
const int ltgOverallTime=10;  //Light sunrise occurs over a ten minute period
const int snoozeTime=1;  //Time for clock to snooze in minutes
int ltgPulseTime=((ltgOverallTime*60)/48)*1000;
int ltgCounter=0;

int i=0; //Library requires an integer to be sent for a zero address
volatile int ci=0; //Rotary Encoder Flag
int menuLight=0; //Keep track of what you are programming

boolean ut=true; //Update Time Flag
boolean am=false; //Alarm Mode Flag
boolean aam=true; //Audible Alarm Mode Flag
boolean ampm =false; //True=PM False=AM
//volatile boolean pas=HIGH;  //Previous alarm state, set volatile as it will be updated in a ISR
volatile boolean AlarmLED=LOW;
volatile uint8_t AlarmStatus=0;
boolean sn=false;  //Snooze flag, active high

uint8_t mm = 0;
uint8_t hh = 0;
uint8_t smm = 0;  //Snooze minutes

unsigned long menuTimer=millis(); //Keep track of encoder button presses
unsigned long ltgTimer=millis();  //Keep track of when the lights were last incremented
unsigned long alarmTimer=millis(); //Keep track of the alarm


uint8_t bcd2bin (uint8_t val) { return val - 6 * (val >> 4); }
uint8_t bin2bcd (uint8_t val) { return val + 6 * (val / 10); }


void setup()
{
  Wire.begin(); // join i2c bus
  //Serial.begin(9600);

  // Setup encoder pins as inputs
  pinMode(ENC_A, INPUT);
  digitalWrite(ENC_A, HIGH); 
  pinMode(ENC_B, INPUT);
  digitalWrite(ENC_B, HIGH);
  //Allow interrupt to be shutdown and manually controlled
  pinMode(swint, INPUT);
  pinMode(rtcint, INPUT);
  //  Setup pins as high Z for Lutron Remote
  pinMode(ltgoff, INPUT); //OFF
  digitalWrite(ltgoff, HIGH);
  pinMode(ltgup, INPUT); //DIM UP
  digitalWrite(ltgup, HIGH);
  // Send power to rotary encoder and Lutron remote. . .temporary. . .not needed when actual circuit board is developed
  //pinMode(7, OUTPUT);
  //digitalWrite(7, HIGH);
  //pinMode(buzzer, OUTPUT); 
  //digitalWrite(buzzer, LOW);
  //pinMode(13, OUTPUT);  //. . . temporary. . . .not needed when actual circuit board is developed???
  //digitalWrite(13, LOW);
   //pinMode(A2, OUTPUT);
  //digitalWrite(A2, LOW);  //Cap Sensor GND
   //pinMode(A1, OUTPUT);
  //digitalWrite(A1, HIGH);  //Cap Sensor Power
   pinMode(A0, INPUT);
  digitalWrite(A0, LOW);//Cap Sensor OUT
  
  // Initialize the RTC
  // Set time after loss of power
  Wire.beginTransmission(RTC_Address);
  Wire.write(0x0F);	
  Wire.endTransmission();
  Wire.requestFrom(RTC_Address, 1);
  if(Wire.read() & 0x80)//Check and see if the OSF flag is enabled, and if so set the clock to some default value
   {
  
  
        Wire.beginTransmission(RTC_Address); // transmit slave device address
        Wire.write(i);             // send register address for control register
        Wire.write(i);             // Seconds - 00  
        Wire.write(i);             // Minutes - 00
        Wire.write(0x52);          // Hours - 12AM - 12 hour mode  
        Wire.endTransmission();
        
        //Set alarm registers
        Wire.beginTransmission(RTC_Address); // transmit slave device address
        Wire.write(0x07);          // send register address for control register
        //Alarm1 - Whenever hours, minutes, seconds match
        Wire.write(i);             // Alarm1 Seconds - 00  
        Wire.write(i);             // Alarm1 Minutes - 00
        Wire.write(0x47);          // Alarm1 Hours - 7AM - 12 hour mode
        Wire.write(0x80);          // Alarm1 whenever hours, minutes, seconds match
        //Alarm2 - Once a minute, i.e., whenever seconds = 00
        Wire.write(0x80);          // Alarm2 whenever seconds match
        Wire.write(0x80);          // Alarm2 whenever seconds match
        Wire.write(0x80);          // Alarm2 whenever seconds match
        //Control Register
        Wire.write(0x07);          // Enables Alarm1 and Alarm2 and corresponding INT_A and INT_B (Do not connect INT_A to uC)
        //Status Register
        Wire.write(i);             // Ensures Alarm flags and Oscillator flag are reset
        //Trickle Charger Register
        Wire.write(i);             // Ensures trickle charger is disabled
        Wire.endTransmission();
        ci=3;//so we setup time if power was out
   }//End of RTC initialization
   
  // Initialize the LED display driver
  Wire.beginTransmission(LEDDisp_Address); // transmit to device #0 AS1115
  Wire.write(0x09);          // send register address for control register
  Wire.write(0x0F);          // Sets decode mode for first four digits  
  Wire.write(0x0F);          // Global Intensity set to highest
  Wire.write(0x06);          // Scan digits set to six; four digits,colon and alarm PM status 
  Wire.write(0x01);          // Begin normal operation setting feature register to default
  Wire.endTransmission();
  
  //getCurrentTime(0x01);
  //updateDisplay();
  
  //2014.01.10 - Initialize Pin Change Interrupt
  PCICR |= 0x02; //Enable PCIE1 for PCINT 14-8
  PCMSK1 |= 0x01; //Pin Change Mask Register set for PCINT8 (A0)
  
  //Enable the clock interrupt
  attachInterrupt(0, clockInt, FALLING); 
  //Enable the rotary encoder control interrupt
  attachInterrupt(1, controlInt, FALLING);

}

void loop()
{
  //Update Time, ut, Occurs once a minute
  if (ut)
    {
      //Update Display
      getCurrentTime(0x01);
      updateDisplay();
      //Check Alarm Flag 1 as we are only using one interrupt
      if(checkAlarmFlag()) 
        am=true;
      //Reset Alarm Flags after checking alarm 1 status
      resetAlarmFlag();
      //Reset UpdateTime Flag
      ut=false;
    }
    
    while(AlarmStatus>1)
    {
      if(AlarmStatus==2 && !AlarmLED)
      {
        //AlarmLedOn();
        AlarmLED=HIGH;
        getCurrentTime(0x08); //Get Alarm Time
        updateDisplay(); //Display Alarm Time
        delay(2000);
        getCurrentTime(0x01); //Get Time
        updateDisplay(); //Display Time
        AlarmStatus=0;     
      }  
      if(AlarmStatus==2 && AlarmLED)
      {
        AlarmLedOff();
        AlarmLED=LOW;      
        getCurrentTime(0x01); //Get Time
        updateDisplay(); //Display Time
        AlarmStatus=0; 
      }  
      if(AlarmStatus>4)
      {
        AlarmStatus=0;
      }
    }
    
    
  // Enter control interrupt subroutine
  if (ci && (millis()-menuTimer>500))
    {
      startDisplayBlink();
      detachInterrupt(0);//Disable the clock interrupt during programming
      detachInterrupt(1);//Disable the control interrupt during programming
      
      if(ci==1)
      {
      menuLight=1;
      getCurrentTime(0x08); //Get Alarm Time
      updateDisplay(); //Display Alarm Time
      setNewTime(0x08); //Set Alarm Time
      }
      if(ci==3)
      {
      menuLight=2;
      getCurrentTime(0x01); //Get Time
      updateDisplay(); //Display Time
      setNewTime(0x01); //Set Time
      }
      
      menuLight=0;
      endDisplayBlink();
      getCurrentTime(0x01);
      updateDisplay();
      attachInterrupt(0, clockInt, FALLING);//Enable the clock interrupt
      attachInterrupt(1, controlInt, FALLING);//Enable the control interrupt
      ci=0;
      am=false; // Ensure alarm mode is off when programming the time and alarm
      ltgCounter=0; // Ensure the alarm is reset after entering the programming mode
      sn=false;  //Ensure the snooze is reset after programming mode
    }
  
  /*  
    //Check if alarm is on or off
    if(digitalRead(A0) == !pas)
    {
      
      //digitalWrite(13, pas);
      if(am && pas==false)
     {
       alarmTimer=millis();
       while(millis()-alarmTimer<3500) //If second press within 3.5 seconds snooze
       {
         if(digitalRead(A0) == pas)
         {
           //Serial.print("Snooze: ");
           //Chirp for snooze
           pas = !pas;
           //digitalWrite(13, pas);
           //chirp();
           
           pinMode(ltgoff, OUTPUT);
           digitalWrite(ltgoff, LOW);
           delay(ltgDelay);
           pinMode(ltgoff, INPUT); 
           digitalWrite(ltgoff, HIGH);
           ltgCounter=0;
           
           getCurrentTime(0x01); //Get Time
           smm=bcd2bin(mm)+snoozeTime;
           if (smm>59)
             smm-=60;
           smm=bin2bcd(smm);
           //Serial.println(smm, HEX);
           sn=true;
           break;
         }
         delay(100); 
         sn=false;  
       }
       
       if (!sn)
       {
       //Serial.println("Inactivate");
       ltgCounter=0;
       am=false;
       sn=false;
       }
     }
   }
  */  
    //Increment lighting when alarm is on 
    if(am && (millis()-ltgTimer)>ltgPulseTime && AlarmLED)
    {
      if (smm==mm && sn) 
      {
        sn=false;
       // if (ltgCounter==48 && aam)
        //  digitalWrite(buzzer, HIGH); //begin audible alarm again if it was off due to a snooze
      }
      if(ltgCounter<48 && !sn)
     {
       ltgTimer=millis();
       ltgCounter++;
       //Serial.println(ltgCounter);
       
       pinMode(ltgup, OUTPUT);
       digitalWrite(ltgup, LOW);
       delay(ltgDelay);
       pinMode(ltgup, INPUT); 
       digitalWrite(ltgup, HIGH);
       
       if(ltgCounter==48 && aam)
       {
       //digitalWrite(buzzer, HIGH);
       ltgTimer=millis();
       //Serial.println("Audible Alarm on");
       }
     }
     else if (ltgCounter==48 && sn)
       ltgTimer=millis();
     else if (ltgCounter>=48 && (millis()-ltgTimer)>62000)
     {
       //Serial.println("Alarm off");
       //digitalWrite(buzzer, LOW);
       ltgCounter=0;
       am=false;
     }
    }
    
    
    
  
} // End of main loop()


//Interrupt 0
void clockInt()
{
  ut=true;
}

//Interrupt 1
void controlInt()
{
  unsigned long CITimer=(millis()-menuTimer); //Keep track of encoder button presses
  ci++;
  if (CITimer>200 && CITimer<1000)
    ci++;
  menuTimer=millis();
}

// returns change in encoder state (-1,0,1)
int8_t read_encoder()
{
  int8_t enc_states[] = {  0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0  }; 
  static uint8_t old_AB = 0;
  old_AB <<= 2;                   //remember previous state
  old_AB |= ( (ENC_PORT >> 1) & 0x03 );  //add current state
  return ( enc_states[( old_AB & 0x0f )]);  
}

void setNewTime(uint8_t ra)
{
 static int8_t counter = 0;      //this variable will be changed by encoder input      
  int8_t tmpdata;
  while((digitalRead(swint)==HIGH) && (ci==1 || ci==3))
  {
    tmpdata = read_encoder();
    if( tmpdata )
    {
      if( tmpdata == -1 ) 
      {
        counter++;
      }
      if (counter==4)
      {
        hh=bcd2bin(hh);
        hh++; //Increase Hours
        if (hh==12) ampm=!ampm;
        if(hh>=13) hh=1;
        hh=bin2bcd(hh);
        updateDisplay();
        counter=0;
      }
      if( tmpdata == 1 ) {
        counter--;
      }
      if (counter==-4)
      {
        hh=bcd2bin(hh);
        hh--; //Decrease Hours
      if(hh==11) ampm=!ampm;  
      if(hh==0 || hh>12) hh=12;
        hh=bin2bcd(hh);
        updateDisplay();
        counter=0;
      }
    }
  } //End of while loop
  while(digitalRead(swint)==LOW){delay(500);}
  while((digitalRead(swint)==HIGH) && (ci==1 || ci==3))
  {
    tmpdata = read_encoder();
    if( tmpdata )
    {
      if( tmpdata == -1 ) 
      {
        counter++;
      }
      if (counter==4)
      {
        mm=bcd2bin(mm);
        mm++; //Increase Hours
        if(mm>=60) mm=0;
        mm=bin2bcd(mm);
        updateDisplay();
        counter=0;
      }
      if( tmpdata == 1 ) {
        counter--;
      }
      if (counter==-4)
      {
        mm=bcd2bin(mm);
        mm--; //Decrease Hours
        if(mm>59) mm=59;
        mm=bin2bcd(mm);
        updateDisplay();
        counter=0;
      }
    }
  } //End of while loop
  if(ampm) hh|=0x60;
  else hh|=0x40;
  Wire.beginTransmission(RTC_Address); // transmit slave device address
  Wire.write(ra);            // send register address for control register 
  Wire.write(mm);            // Minutes 
  Wire.write(hh);            // Hours - 12 hour mode  
  Wire.endTransmission();
}

void getCurrentTime( uint8_t pa )
{
 //Check current time
  Wire.beginTransmission(RTC_Address);
  Wire.write(pa);	
  Wire.endTransmission();
  
  Wire.requestFrom(RTC_Address, 2);
  mm = Wire.read();
  hh = Wire.read()&0x3F; 
  if(hh & 0x20) 
    ampm=1;
  else ampm=0;
}

void updateDisplay()
{
  uint8_t digit1 = ((hh>>4) & 0x01);
    if(!digit1)
      digit1=0x0F;
  uint8_t digit6;
    if(!AlarmLED)//Alarm state is OFF
      digit6 &= 0xBF;//Set alarm on/off light off
    else
     digit6 |= 0x40;//Set alarm on/off light on
    if(menuLight==1)//i.e. Set Alarm Mode
      digit6|=0x10; //Set "Alarm" light on
  uint8_t digit2 = (hh & 0x0F);
    if(menuLight==2)//i.e. Set Time Mode
      digit6|=0x20;//Set "Time" light on
  uint8_t digit3 = ((mm>>4) & 0x0F);
  uint8_t digit4 = (mm & 0x0F);
  
    if(ampm)
      digit6 |= 0x08;
    else
      digit6 &= 0xF7;
 // Display Time  
  Wire.beginTransmission(LEDDisp_Address); // transmit to device #0 AS1115
  Wire.write(0x01);          // send register address for first digit
  Wire.write(digit1);          // Send  
  Wire.write(digit2);          // Send 
  Wire.write(digit3);          // Send 
  Wire.write(digit4);          // Send 
  Wire.write(0x06);          //Colon always on - No decode mode digits A & B
  Wire.write(digit6);        //Sets on PM, clears on AM
  Wire.endTransmission();    // stop transmitting
  
}

boolean checkAlarmFlag()
{
 //Check Alarm Flag 1
  boolean af;
  Wire.beginTransmission(RTC_Address);
  Wire.write(0x0F);	
  Wire.endTransmission();
  Wire.requestFrom(RTC_Address, 1);
  if(Wire.read() & 0x01)
   af=true;  
  else
   af=false;
  return af;
}

void resetAlarmFlag()
{
  Wire.beginTransmission(RTC_Address); // transmit slave device address
  Wire.write(0x0F);          // send register address for status register
  Wire.write(i);             // Reset Alarm flags
  Wire.endTransmission(); 
}

void startDisplayBlink()
{
  Wire.beginTransmission(LEDDisp_Address); // transmit to device #0 AS1115
  Wire.write(0x0E);          // send register address for feature register
  Wire.write(0x10);          // Send  
  Wire.endTransmission();    // stop transmitting 
}

void endDisplayBlink()
{
  Wire.beginTransmission(LEDDisp_Address); // transmit to device #0 AS1115
  Wire.write(0x0E);          // send register address for feature register
  Wire.write(i);             // Send  
  Wire.endTransmission();    // stop transmitting 
}

/*
void chirp()
{
  digitalWrite(buzzer, HIGH);
  delay(10);
  digitalWrite (buzzer, LOW);
}
*/

//2014.01.10 - ISR uses pas variable to determine 
ISR(SIG_PIN_CHANGE1)
{
  AlarmStatus++;
}

void AlarmLedOn()
{
  uint8_t digit6;
  
  //Get current status from the LED display driver for digit 6
  Wire.beginTransmission(LEDDisp_Address);
  Wire.write(0x07);	
  Wire.endTransmission();
  Wire.requestFrom(LEDDisp_Address, 2);
  digit6 = Wire.read();
   
  digit6 |= 0x40;//Set alarm on/off light on
  Wire.beginTransmission(LEDDisp_Address); // transmit to device #0 AS1115
  Wire.write(0x07);          // send register address for sixth digit
  Wire.write(digit6);        //Sets alarm LED
  Wire.endTransmission();    // stop transmitting
}

void AlarmLedOff()
{
  uint8_t digit6;
  
  //Get current status from the LED display driver for digit 6
  Wire.beginTransmission(LEDDisp_Address);
  Wire.write(0x07);	
  Wire.endTransmission();
  Wire.requestFrom(LEDDisp_Address, 2);
  digit6 = Wire.read();
  
  digit6 &= 0xBF;//Set alarm on/off light off
  Wire.beginTransmission(LEDDisp_Address); // transmit to device #0 AS1115
  Wire.write(0x07);          // send register address for sixth digit
  Wire.write(digit6);        //Sets alarm LED
  Wire.endTransmission();    // stop transmitting
}
