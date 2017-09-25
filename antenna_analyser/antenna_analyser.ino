#include <Adafruit_ST7735.h>
#include <Adafruit_GFX.h>
#include <gfxfont.h>
#include <String.h>

#include "math.h"

/*
Antenna Analyser 
Hardware: 101+EZ-bluefruit+antenna analyzer shield+tft,sd shield
by pavan.inferno
*/

#include <SPI.h>
//#include <TFT.h>


//TFT screen pins
#define cs_tft   10
#define dc   8
#define rst  9
#define mosi 11
#define sclk 13

//SD card pins
#define cs_sd 4

// AD9850 (DDS) pins
#define fq_ud 12
#define sdat 11
#define sclk 13
#define reset -1


#define Neutral 0
#define Press 1
#define Up 2
#define Down 3
#define Right 4
#define Left 5

boolean use_calibration = false;

double swr_results [11][4]={
  
  //SWRmin,Freq(SWRmin)in MHz,correction gradient (*ln(X)), correction intercept
  // separate array from hfbands
  // because this stores floats
  
  {0,0,3.500,0.660},  // correction needs to be measured for 160 m
  {0,0,3.548,0.623},
  {0,0,3.575,0.590},  // ditto for 60 m 
  {0,0,3.609,0.570},                        
  {0,0,3.696,0.500},                        
  {0,0,3.758,0.413},
  {0,0,3.767,0.257},
  {0,0,3.804,0.187},
  {0,0,3.818,0.118},
  {0,0,3.754,0.050},
  {0,0,0,0}                         // this last slot is for arbitrary sweeps
};
// Check the joystick position
int CheckJoystick()
{
  int joystickState = analogRead(2);
  
  if (joystickState < 50) return Left;
  if (joystickState < 150) return Down;
  if (joystickState < 250) return Press;
  if (joystickState < 500) return Right;
  if (joystickState < 650) return Up;
  return Neutral;
}

Adafruit_ST7735 screen = Adafruit_ST7735(cs_tft, dc, mosi, sclk, rst);
String button_status = "";
float x_min = 0,x_max = 90,y_min = -23.12, y_max = 2.34;

// the setup function runs once when you press reset or power the board
void setup() {
  
  //Initialize serial port
  Serial.begin(115200);

  //Configure DDS pins
  pinMode(fq_ud,OUTPUT);
  pinMode(sdat,OUTPUT);
  pinMode(sclk,OUTPUT);
  pinMode(reset,OUTPUT);
  

  //Initialize screen
  screen.initR(INITR_BLACKTAB);   // initialize a ST7735S chip, black tab

  //Set black background
  screen.fillScreen(ST7735_BLACK);
  screen.setRotation(3);

  //Setting text attributes
  screen.setTextSize(0);
  screen.setTextColor(ST7735_YELLOW);

  //Draw the line to separate plot window
  screen.writeFastHLine(0,10,screen.width(),ST7735_BLUE);
  
  //Draw plot border
  screen.drawRect(15,10+2,screen.width()-15,screen.height()-20,ST7735_RED);

  //Plot axes labels 
  screen.setTextColor(ST7735_GREEN);
  
  screen.setCursor(0,screen.height()-20);
  screen.print(y_min,1);

  screen.setCursor(0,screen.height()/2);
  screen.print(0.5*(y_min+y_max),1);
  
  screen.setCursor(0,20);
  screen.print(y_max,1);

  screen.setCursor(10,screen.height()-10);
  screen.print(x_min,1);

  screen.setCursor(screen.width()/2-10,screen.height()-10);
  screen.print(0.5*(x_min+x_max),1);
  
  screen.setCursor(screen.width()-30,screen.height()-10);
  screen.print(x_max,1);
  
}

// the loop function runs over and over again forever
void loop() {
 

// Write someting to the serial port
//Serial.println("Hola");
// Test the button 
int joy = CheckJoystick();
button_status = "";
  switch (joy)
  {
    case Left:
      button_status = "Left";
      break;
    case Right:
      button_status = "Right";
      break;
    case Up:
      button_status = "Up";
      break;
    case Down:
      button_status = "Down";
      break;
    case Press:
      button_status = "Press";
      break;
    default:
      button_status = "";
      break;
  }

//  Serial.println(button_status);
  Serial.println(Get_VSWR());
  
  screen.setCursor(0,0);
  screen.setTextColor(ST7735_YELLOW);
  screen.println(button_status);
  delay(100);
  screen.setCursor(0,0);
  screen.setTextColor(ST7735_BLACK);
  screen.println(button_status);
 
}
double Get_VSWR(){
  double FWD=0;
  double REV=0;
  double VSWR;
  double FWD5=0;  // summing array for 5 point averaging at each frequency
  double REV5=0;
  double temp_VSWR;

  // Read the forward and reverse voltages - always returns nonsense on first iteration after reset (even if I do multiple reads)
    FWD5=0; 
    REV5=0;
    for(int k=0;k<5;k++){
      analogRead(A0);
      delay(10);
      REV5 = REV5 + analogRead(A0);
      analogRead(A1);
      delay(10);
      FWD5 = FWD5 + analogRead(A1);
    }
    FWD=FWD5/5;  //   carry out averaging calc
    REV=REV5/5;
    
    if(REV>=FWD){
      // To avoid a divide by zero or negative VSWR then set to 0
      temp_VSWR = 0;
    }else{
      // Calculate VSWR
      temp_VSWR = (FWD+REV)/(FWD-REV);
    }
    
    return temp_VSWR;
}

void SetDDSFreq(double Freq_Hz){
  // Calculate the DDS word - from AD9850 Datasheet
  int32_t f = Freq_Hz * 4294967295/125000000;
  // Send one byte at a time
  for (int b=0;b<4;b++,f>>=8){
    send_byte(f & 0xFF);
  }
  // 5th byte needs to be zeros
  send_byte(0);
  // Strobe the Update pin to tell DDS to use values
  digitalWrite(fq_ud,HIGH);
  digitalWrite(fq_ud,LOW);
}

void send_byte(byte data_to_send){
  // Bit bang the byte over the SPI bus
  for (int i=0; i<8; i++,data_to_send>>=1){
    // Set Data bit on output pin
    digitalWrite(sdat,data_to_send & 0x01);
    // Strobe the clock pin
    digitalWrite(sclk,HIGH);
    digitalWrite(sclk,LOW);
  }
}
