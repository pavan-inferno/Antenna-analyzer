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
#define cs_tft   7 // 7 not default, this involves some soldering on the TFT shield
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
#define reset 10


#define Neutral 0
#define Press 1
#define Up 2
#define Down 3
#define Right 4
#define Left 5

#define AD9850_CLOCK 125000000         // Module crystal frequency. Tweak here for accuracy.

boolean use_calibration = false;
boolean plot_band = true;

long hfbands [10][4]={
  {160,1800000,2000000,100},
  {80,3500000,4000000,100}, // wavelength,Fstart,Fstop,nsteps
  {60,5330000,5405000,30},
  {40,7000000,7300000,100},
  {30,10100000,10150000,20},
  {20,14000000,14350000,100},
  {17,18068000,18168000,30},
  {15,21000000,21450000,150},
  {12,24890000,24990000,30},
  {10,28000000,29700000,100}
};
long mp_band[10];

double swr_results [11][4]={
  
  //SWRmin,Freq(SWRmin)in MHz,correction gradient (*ln(X)), correction intercept
  // separate array from hfbands
  // because this stores floats
  
  {0,0,4.4167,0.8015},  // correction needs to be measured for 160 m
  {0,0,4.529,0.8599},
  {0,0,4.5922,0.9774},  // ditto for 60 m 
  {0,0,4.9884,1.2783},                        
  {0,0,5.4176,1.7267},                        
  {0,0,6.2202,2.6581},
  {0,0,7.3635,3.8858},
  {0,0,3.804,0.187},
  {0,0,3.818,0.118},
  {0,0,3.754,0.050},
  {0,0,0,0}                         // this last slot is for arbitrary sweeps
};

boolean sweepAbort = LOW;

double temp_swr;

double xmin,xmax,ymin,ymax,swrmin;

double moving_avg_swr [9];        // nine-point moving average for VSWR
double Fstart_MHz = 1;  // Start Frequency for sweep
double Fstop_MHz = 10;  // Stop Frequency for sweep
double Fstep_MHz = 0;
double current_freq_MHz; // Temp variable used during sweep
int num_steps = 100; // Number of steps to use in the sweep
int band_choice = 0; // doesn't need to be zeroed as this happens during initial sweep - used for single band sweeps and graph plotting

int graph_array [100];          // storage array for single graph - try storing vswr values as ints (after multiplying by 100) to save memory
int temp_bandno = 0;
int min_point  = 0;


#define pulseHigh(pin) {digitalWrite(pin, HIGH); digitalWrite(pin, LOW); }



Adafruit_ST7735 screen = Adafruit_ST7735(cs_tft, dc, mosi, sclk, rst);
String button_status = "";
float x_min = 0,x_max = 90,y_min = -23.12, y_max = 2.34;

// the setup function runs once when you press reset or power the board
void setup() {
  
  //Initialize serial port
  Serial.begin(115200);

//  Configure DDS pins
  pinMode(fq_ud,OUTPUT);
  pinMode(sdat,OUTPUT);
  pinMode(sclk,OUTPUT);
  pinMode(reset,OUTPUT);
  
//  Serial.println(hfbands[0][3]);

  // Set up analog inputs on A0 and A1, internal reference voltage
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);
  pinMode(A2,INPUT);

  // Reset the DDS
  // Initialise the AD9850 module. 
  pulseHigh(reset);
  pulseHigh(sclk);
  pulseHigh(fq_ud);    // this pulse enables serial mode - Datasheet page 12 figure 10  


// Sweep all midpoints of bands
//  for (int i=0; i<10; i++)
//  {
//  mp_band[i] = (hfbands[i][1] + hfbands[i][2])/2;
//
//  SetDDSFreq(mp_band[i]);
//  delay(550);
//  temp_swr = Get_VSWR();
//  for (int j = 0; j<9; j++)
//  {
//    temp_swr = (temp_swr + Get_VSWR());
//    delay(50);
//  }
//  Serial.println(temp_swr/10.0,7);
//  }


  
  //Initialize screen
  screen.initR(INITR_BLACKTAB);   // initialize a ST7735S chip, black tab
  screen.fillScreen(ST7735_BLACK);
  screen.setRotation(3);

  screen.print("Do you want to continue with frequency sweep ?");
 
  screen.fillScreen(ST7735_BLACK);
  screen.setCursor(0,0);
  int adjFreq = 1;
  while(CheckJoystick()!= Press)
  {
    
    // Set DDS to current frequency
    SetDDSFreq(adjFreq*1000000);
    delay(50);
    if (CheckJoystick()== Left)
      adjFreq--;
    if (CheckJoystick()== Right)
      adjFreq++;
    // Wait a little for settling - needs to be at least 50 ms (see update info at start)
    //delay();
    screen.fillRect(0,0,100,20,ST7735_BLACK);
    screen.println(Get_VSWR());
    screen.println(adjFreq);
    screen.setCursor(0,0);
  }
  
  screen.fillScreen(ST7735_BLACK);
  screen.setTextSize(1);
  screen.setCursor(0,0);
  screen.print("Scanning Frequency bands");
  //Setting text attributes
  screen.setTextSize(1);
  screen.setTextColor(ST7735_GREEN);

  screen.setCursor(0,10);
  screen.print("Frequency(Hz)");
  screen.setCursor(90,10);
  screen.print("VSWR ");
  screen.setCursor(130,10);
  screen.print("Band");
  
  
  Serial.println("setting dds freq to 7 MHz ");
  SetDDSFreq(7000000);  // start the DDS working at 7 MHz
  delay(1000);  // let the DDS settle and allow time to read splash screen
  // then do a couple of dummy reads on the ADC to flush out the known problem with the first reading after changing the reference
  Serial.println("Performing dummy read on A0 ");
  analogRead(A0);
  delay(100);
  Serial.println("Performing dummy read on A1 ");
  analogRead(A1);
  delay(100);

  // On power-up, sweep Amateur HF bands 80m to 10m
      // as it makes device useful without controls
      // just press reset on Nano to restart measurement and clear minimum values
//  Serial.println("Sweeping bands ");
//  Sweep_bands();
//  Serial.println("Reporting bands ");
//  Sweep_report();
//  Serial.println("Sweeping and reporting done !");
//
//  screen.setCursor(0,0);
//  screen.fillRect(0,0,150,10,ST7735_BLACK);
//  screen.print("Min. VSWR ");
//  for (int j=0; j <1; j++)
//  {
//    screen.setCursor(0,20+10*j);
//    screen.fillRect(0,20+10*j,120,10,ST7735_BLACK);
//    screen.print(swr_results[j][1],3); 
//    screen.setCursor(90,20+10*j);
//    screen.println(swr_results[j][0]);
//
//  }
//  
//  screen.setCursor(0,120);
//  screen.print("Which band ?  ");
//  while(CheckJoystick()!= Press)
//  {
//    screen.fillRect(100,120,40,10,ST7735_BLACK);
//    screen.setCursor(100,120);
//    screen.print(hfbands[temp_bandno][0]);
//    screen.print('m');
//    if (CheckJoystick()== Left)
//      temp_bandno--;
//    if (CheckJoystick()== Right)
//      temp_bandno++;
//    delay(200);
//    if (temp_bandno==8)
//      temp_bandno=0;
//    if (temp_bandno==-1)
//      temp_bandno=7;
//  }

Plot_band(temp_bandno);
  
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
//  Serial.println(Get_VSWR());
  
  screen.setCursor(0,0);
  screen.setTextColor(ST7735_YELLOW);
  screen.println(button_status);
  delay(100);
  screen.setCursor(0,0);
  screen.setTextColor(ST7735_BLACK);
  screen.println(button_status);
 
}

void Plot_band(int j)
{

  Perform_sweep(j);
  plot_template(int(ymin*100), int(ymax*100));
  autoscaled_plot(int(ymin*100), int(ymax*100));
  if (!plot_band)
  {
  plot_template(int(ymin*100), int(ymax*100));
  autoscaled_plot(int(ymin*100), int(ymax*100));
  }
}

void Sweep_bands(){
        
  for(int i=0;i<=0;i++){
        screen.setCursor(130,20+10*i);
        Serial.print(hfbands[i][0]);
        screen.print(hfbands[i][0]);
        Serial.println('m');
        screen.print('m');
        
        

        
        // need to pass current minimum VSWR & freq to sweep routine
        // easiest to use array subscript
        Perform_sweep(i);
        
//        checkButton();  // test for abort (i.e. button pressed)
        if(sweepAbort==HIGH){
          sweepAbort=LOW;
          return;
        }
        
        }
}

void Sweep_report(){
  
  char minvswr[6];
  char minfreqdisp[16];
}

void Perform_sweep(int j){
  char freqdisp[16];
  char vswrdisp[16];
  char vswrchar[6];
  char minvswr[6];
  char minfreqdisp[16];
  int avg_ctr=0;  // counter for moving average (over frequency) array
  int y_min,y_max;

  y_min = 100;
  y_max = 300;

  Fstart_MHz = ((double)hfbands[j][1])/1000000;
  Fstop_MHz = ((double)hfbands[j][2])/1000000;
  num_steps = hfbands[j][3];
  Fstep_MHz = (Fstop_MHz-Fstart_MHz)/num_steps;

  xmax = Fstop_MHz;
  xmin = Fstart_MHz;
  ymax = 0;
  
  double AVG_VSWR;

  
  swr_results[j][0]=0;  // clear stored minimum for this band choice
  swr_results[j][1]=0;
  band_choice=j;  // needed to ensure correct calibration factors are applied for band

  if (plot_band){
    plot_template(y_min,y_max);

  }
  memset(graph_array,0,sizeof(graph_array));  // zero graphng array

  // Start loop 
  for(int i=0;i<=(num_steps+8);i++){  // pad loop out with additional eight steps for 9 pt moving average

//  if(buttonCycled()==HIGH){
//    sweepAbort=HIGH;
//    return;
//  }
        
    // Calculate current frequency
    current_freq_MHz = Fstart_MHz + (i-4)*Fstep_MHz;  // four additional steps prior to Fstart, owing to 9 pt moving average
    // Set DDS to current frequency
    SetDDSFreq(current_freq_MHz*1000000);
    // Wait a little for settling - needs to be at least 50 ms (see update info at start)
    delay(50+100*(i<1));

    moving_avg_swr[avg_ctr]=Get_VSWR(); // load value into next available slot in averaging array
    
    // calculate average of array - crude but effective...
    AVG_VSWR=(moving_avg_swr[0]+moving_avg_swr[1]+moving_avg_swr[2]+moving_avg_swr[3]+moving_avg_swr[4]+moving_avg_swr[5]+moving_avg_swr[6]+moving_avg_swr[7]+moving_avg_swr[8])/9;
 
    // Send current line back to PC over serial bus - this may cause problems for interrupts but there's no reason to be using these to read encoder during a sweep
    AVG_VSWR = swr_results[j][2]*log(AVG_VSWR) - swr_results[j][3];
    if(i>=8){    // VSWR values only valid after 8 measurements have been processed
 
    // load value into graph array if it doesn't overflow
    if(i<108){
      graph_array[i-8]=int(AVG_VSWR*100);
    }

    

    if(!plot_band){
      screen.setCursor(0,20+10*j);
      screen.fillRect(0,20+10*j,120,10,ST7735_BLACK);
      screen.print(long((current_freq_MHz-4*Fstep_MHz)*1000000)); 
      screen.setCursor(90,20+10*j);
      screen.println(AVG_VSWR);
    Serial.print(long((current_freq_MHz-4*Fstep_MHz)*1000000)); // remember that the frequency corresponding to current avg_VSWR was four iterations ago
    Serial.print(",");
    Serial.println(AVG_VSWR,7);
    }
    else
    {
      screen.drawPixel(8 + i, map(graph_array[i-8],y_min,y_max,screen.height()-20,13), ST7735_WHITE);
      screen.fillRect(0,0,40,10,ST7735_BLACK);
      screen.setCursor(0,0);
      screen.print(AVG_VSWR,4);

      
      screen.fillRect(50,0,50,10,ST7735_BLACK);
      screen.setCursor(50,0);
      screen.print(long((current_freq_MHz-4*Fstep_MHz)*1000000));

    }
      // test for minimum VSWR - store and notify if new minimum found
      if((AVG_VSWR<swr_results[j][0])||(swr_results[j][0]==0)){
         swr_results[j][0]=AVG_VSWR;
         swr_results[j][1]=current_freq_MHz-4*Fstep_MHz;  // remember to subtract 4 bin offset for 9 pt moving average
         ymin = AVG_VSWR;
         min_point = i-8;
        }
      // test for maximum VSWR 
      if((AVG_VSWR>ymax)||(ymax==0)){
        ymax=AVG_VSWR;
        }

      // test for minimum VSWR over any sweep - note sure if row [8] of this array is actually used any more - legacy code?
      if((AVG_VSWR<swr_results[8][0])||(swr_results[8][0]==0)){
        swr_results[8][0]=AVG_VSWR;
        swr_results[8][1]=current_freq_MHz-4*Fstep_MHz;  // remember to subtract 4 bin offset for 9 pt moving average
        }
     
    
      // now put it on the OLED screen
      // convert the numbers to strings...
      dtostrf(current_freq_MHz-4*Fstep_MHz,6,3,freqdisp);
      dtostrf(AVG_VSWR,5,2,vswrdisp);
      // min values over entire range / any bands
      dtostrf(swr_results[j][1],6,3,minfreqdisp);
      dtostrf(swr_results[j][0],5,2,minvswr);

    
      }
    
    avg_ctr++;                    // increment moving average array counter to fill next slot
    if(avg_ctr>8) avg_ctr=0;      // wrap back to zero if slot number exceeds 8 (=ninth slot of array)
      
  }
    
}

//Plot template window
void plot_template (int y_min, int y_max){
      screen.fillScreen(ST7735_BLACK);
      //Draw plot border
    screen.drawRect(15,10+2,102,screen.height()-20,ST7735_RED);
      //Setting text attributes
    screen.setTextSize(0);
    screen.setTextColor(ST7735_YELLOW);
  
    //Draw the line to separate plot window
    screen.drawFastHLine(0,10,screen.width(),ST7735_BLUE);
   
    //Plot axes labels 
    screen.setTextColor(ST7735_GREEN);
    screen.setCursor(0,screen.height()-12);
    screen.print(y_min);

    //labels
    screen.setCursor(40,0);
    screen.print('@');
    screen.setCursor(100,0);
    screen.print("Hz");

            //Band name
    screen.setCursor(screen.width()-30, 0);
    screen.print(hfbands[band_choice][0]);
    screen.println('m');
    
    screen.setCursor(0,screen.height()/2-10);
    screen.println("SWR");
    screen.println("x");
    screen.println("100");
    
    screen.setCursor(0,12);
    screen.print(y_max);
  
    screen.setCursor(10,screen.height()-7);
    screen.print(xmin,1);
  
    screen.setCursor(50,screen.height()-7);
    screen.print("MHz");
    
    screen.setCursor(100,screen.height()-7);
    screen.print(xmax,1);
}


void autoscaled_plot(int y_min, int y_max){
      for (int i=0; i<100; i++)
      {
      screen.drawPixel(16+i, map(graph_array[i],y_min,y_max,screen.height()-20,13), ST7735_WHITE);
      }


      
      screen.fillRect(0,0,40,10,ST7735_BLACK);
      screen.setCursor(120,20);
      screen.print("Min:");
      screen.setCursor(120,40);
      screen.print(swr_results[band_choice][0],4);
      
      screen.setCursor(120,50);
      screen.print('@');
      screen.setCursor(120,60);
      screen.print(long(swr_results[band_choice][1]*1000000));

      int i = min_point;
      while(CheckJoystick()!= Press)
        {

          screen.fillRect(50,0,50,10,ST7735_BLACK);
          screen.fillRect(0,0,40,10,ST7735_BLACK);
          screen.drawLine(16+i,13,16+i,17,ST7735_BLACK);
          
          delay(50);
          if ((CheckJoystick()== Left)&&(i>0))
            i--;
          if ((CheckJoystick()== Right)&&(i<num_steps))
            i++;
          // Wait a little for settling - needs to be at least 50 ms (see update info at start)
          //delay();
          screen.drawLine(16+i,13,16+i,17,ST7735_YELLOW);
          
          screen.setCursor(0,0);
          screen.print(graph_array[i]/100.0,4);

          screen.setCursor(50,0);
          screen.print(long((Fstart_MHz + i*Fstep_MHz)*1000000));
       
        }
}
// Check the joystick position
int CheckJoystick()
{
  int joystickState = analogRead(3);
//  Serial.println(joystickState);
  if (joystickState < 50) return Left;
  if (joystickState < 150) return Down;
  if (joystickState < 250) return Press;
  if (joystickState < 500) return Right;
  if (joystickState < 650) return Up;
    
//  if (joystickState < 50) return Left;
//  if (joystickState < 200) return Down;
//  if (joystickState < 500) return Press;
//  if (joystickState < 600) return Right;
//  if (joystickState < 950) return Up;
  return Neutral;
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

//    Serial.print(FWD);
//    Serial.print(',');
//    Serial.print(REV);
//    Serial.print(',');
   
    if(REV>=FWD){
      // To avoid a divide by zero or negative VSWR then set to 0
      temp_VSWR = 0;
    }else{
      // Calculate VSWR
      temp_VSWR = (FWD+REV)/(FWD-REV);
    }
    
    return temp_VSWR;
}


 // transfers a byte, a bit at a time, LSB first to the 9850 via serial DATA line
void tfr_byte(byte data) {
  for (int i = 0; i < 8; i++, data >>= 1) {
    digitalWrite(sdat, data & 0x01);
    pulseHigh(sclk);   //after each bit sent, CLK is pulsed high
  }
}

void SetDDSFreq(double frequency) {
  int32_t freq1 = frequency * 4294967295/AD9850_CLOCK;  // note 125 MHz clock on 9850
  for (int b = 0; b < 4; b++, freq1 >>= 8) {
    tfr_byte(freq1 & 0xFF);
  }
  tfr_byte(0x000);                     // Final control byte, all 0 for 9850 chip
  pulseHigh(fq_ud);                    // Done!  Should see output
}
