// aJava
//R. Tsuchiyama

// This code was adapted from the a_logger.pde file provided
// by Bill Welch.

// this library included with the arduino distribution
#include <Wire.h>
#include <LiquidCrystal.h>

// these "contributed" libraries must be installed in your sketchbook's arduino/libraries folder
#include <TypeK.h>
#include <cADC.h>

//user.h is in same directory as this file
#include "user.h"
#define BANNER_BRBN "Java beta 01"

int get_button();          //routine in button.pde

// *************************************************************************************
// NOTE TO USERS: the following parameters should be
// be reviewed to suit your preferences and hardware setup.  
// First, load and edit this sketch in the Arduino IDE.
// Next compile the sketch and upload it to the Duemilanove.

// ------------------ optionally, use I2C port expander for LCD interface
//#define I2C_LCD //comment out to use the standard parallel LCD 4-bit interface
//#define EEPROM_BRBN // comment out if no calibration information stored in 64K EEPROM

#define BAUD 57600  // serial baud rate
#define BT_FILTER 10 // filtering level (percent) for displayed BT
#define ET_FILTER 10 // filtering level (percent) for displayed ET

// needed for usesr without calibration values stored in EEPROM
#define CAL_GAIN 1.00 // substitute known gain adjustment from calibration
#define UV_OFFSET 0 // subsitute known value for uV offset in ADC
#define AMB_OFFSET 0.0 // substitute known value for amb temp offset (Celsius)

// ambient sensor should be stable, so quick variations are probably noise -- filter heavily
#define AMB_FILTER 70 // 70% filtering on ambient sensor readings

// *************************************************************************************

// ------------------------ other compile directives
#define MIN_DELAY 270   // ms between ADC samples (tested OK at 270)
#define NCHAN 2   // number of TC input channels
#define TC_TYPE TypeK  // thermocouple type / library
#define DP 1  // decimal places for output on serial port
#define D_MULT 0.001 // multiplier to convert temperatures from int to float

// --------------------------------------------------------------
// global variables

//#ifdef EEPROM_BRBN // optional code if EEPROM flag is active
//#include <mcEEPROM.h>
// eeprom calibration data structure
/*struct infoBlock {
  char PCB[40]; // identifying information for the board
  char version[16];
  float cal_gain;  // calibration factor of ADC at 50000 uV
  int16_t cal_offset; // uV, probably small in most cases
  float T_offset; // temperature offset (Celsius) at 0.0C (type T)
  float K_offset; // same for type K
}; */
//mcEEPROM eeprom;
//infoBlock caldata;
//#endif

// class objects
cADC adc( A_ADC ); // MCP3424
ambSensor amb( A_AMB ); // MCP9800
filterRC fT[NCHAN]; // filter for displayed/logged ET, BT
filterRC fRise[NCHAN]; // heavily filtered for calculating RoR
filterRC fRoR[NCHAN]; // post-filtering on RoR values


int32_t temps[NCHAN]; //  stored temperatures are divided by D_MULT
int32_t ftemps[NCHAN]; // heavily filtered temps
int32_t ftimes[NCHAN]; // filtered sample timestamps
int32_t flast[NCHAN]; // for calculating derivative
int32_t lasttimes[NCHAN]; // for calculating derivative

// LCD output strings
char smin[3],ssec[3],st1[6],st2[6],smilli[6];

// ---------------------------------- LCD interface definition
#ifdef I2C_LCD
#define BACKLIGHT lcd.backlight();
cLCD lcd; // I2C LCD interface
#else // equivalent to standard LiquidCrystal interface
#define BACKLIGHT_PIN 10
#define RS 2
#define ENABLE 4
#define D4 7
#define D5 8
#define D6 12
#define D7 13
LiquidCrystal lcd( RS, ENABLE, D4, D5, D6, D7 ); // standard 4-bit parallel interface
#endif

//define digital inputs to read push buttons
#define PUSH01 3
#define PUSH02 5
#define PUSH03 6
#define PUSH04 11

//define Loop Time
//#define LoopTime 1000
#define LoopTime 500

// used in main loop
float timestamp = 0;
//boolean first;
uint32_t nextLoop;
float start_shot = 0;
float shot_time = 0;
boolean shot_flag = false;
boolean back_flag = true;
boolean button_pushed = false;

// -------------------------- Settings and Values for Button switches and button wiring
//#define ANYKEY       500       // value to decide if anykey is pushed
//#define DEBOUNCE     350       // debounce time

#define NOBUTTON 0       // set to this value if no valid switch push detected
#define BUTTON_1 1       // set to this value if Button 1 was pushed
#define BUTTON_2 2       // set to this value if Button 2 was pushed
#define BUTTON_3 3       // set to this value if Button 3 was pushed
#define BUTTON_4 4       // set to this value if Button 4 was pushed

int buttonValue = 0;  // variable to store the analog input value coming from the switches



//void updateLCD( float t1, float t2, float t3, float t4 );
void updateLCD( float t1, float t2, float t3);

// ------------------------------------------------------------------
void logger()      //write temps to serial port and to LCD
{
  int i;
  float t1,t2,t3, t_amb;
  float rx;

  // print timestamp from when samples were taken
  Serial.print( timestamp, DP );

  // print ambient
//  Serial.print(",");
  t_amb = amb.getAmbF();
  Serial.print( t_amb, DP );
   
  // print temperature, rate for each channel
  i = 0;
  if( NCHAN >= 1 ) {
    Serial.print(",");
    Serial.print( t1 = D_MULT*temps[i], DP );
    i++;
  };
  
  if( NCHAN >= 2 ) {
    Serial.print(",");
    Serial.print( t2 = D_MULT * temps[i], DP );
    i++;
  };
  
//  if( NCHAN >= 3 ) {
//    Serial.print(",");
//    Serial.print( t3 = D_MULT * temps[i], DP );
//    i++;
//  };
  Serial.print(",");
  Serial.println(millis());
// log the placeholder to serial port
//  Serial.println("");
  
  updateLCD( t1, t2, t3  );  
};

// --------------------------------------------
void updateLCD( float t1, float t2, float t3 ) {
  // form the timer output string in min:sec format
  int itod = round( timestamp );
  if( itod > 3599 ) itod = 3599;
  sprintf( smin, "%02u", itod / 60 );
  sprintf( ssec, "%02u", itod % 60 );
  //start of bottom row
  lcd.setCursor(0,1);
  lcd.print( smin );
  lcd.print( ":" );
  lcd.print( ssec );

  sprintf( smilli, "%06u", millis() );
  lcd.setCursor(4,0); //top row, 4 chars over
  lcd.print( smilli );

if (shot_flag == true) {
   shot_time = timestamp - start_shot;
   }
else
   { shot_time = 0 ;}   
  int ishot = round( shot_time );
  sprintf( ssec, "%02u", ishot % 60 );
  //start of top row
  lcd.setCursor(0,0);
  lcd.print( ssec );

 
  // channel 1 temperature
  int it01 = round( t1 );
  if( it01 > 999 ) 
    it01 = 999;
  else
    if( it01 < -999 ) it01 = -999;
  sprintf( st1, "%3d", it01 );
  lcd.setCursor( 11, 0 );  //right side of top row
  lcd.print("B:");//0
  lcd.print(st1);//1, 2,3 and 4 for a space

  // channel 2 temperature 
  int it02 = round( t2 );
  if( it02 > 999 ) it02 = 999;
  else if( it02 < -999 ) it02 = -999;
  sprintf( st2, "%3d", it02 );
  lcd.setCursor( 11, 1 );  //right side of bottom row
  lcd.print( "G:" ); //5
  lcd.print( st2 ); //6, 7, 8, and 9 for a space
  
}

// --------------------------------------------------------------------------
void get_samples() // this function talks to the amb sensor and ADC via I2C
{
  int32_t v;
  TC_TYPE tc;
  float tempC;
  
  for( int j = 0; j < NCHAN; j++ ) { // one-shot conversions on both chips
    adc.nextConversion( j ); // start ADC conversion on channel j
    amb.nextConversion(); // start ambient sensor conversion
    delay( MIN_DELAY ); // give the chips time to perform the conversions
    ftimes[j] = millis(); // record timestamp for RoR calculations
    amb.readSensor(); // retrieve value from ambient temp register
    v = adc.readuV(); // retrieve microvolt sample from MCP3424
    tempC = tc.Temp_C( 0.001 * v, amb.getAmbC() ); // convert to Celsius
    v = round( C_TO_F( tempC ) / D_MULT ); // store results as integers
    temps[j] = fT[j].doFilter( v ); // apply digital filtering for display/logging
//    ftemps[j] =fRise[j].doFilter( v ); // heavier filtering for RoR
  }
};
  
// ------------------------------------------------------------------------
// MAIN
//
void setup()
{
  delay(100);
  Wire.begin(); 
  pinMode (BACKLIGHT_PIN, OUTPUT);
  pinMode (PUSH01, INPUT);
  pinMode (PUSH02, INPUT);
  pinMode (PUSH03, INPUT);
  pinMode (PUSH04, INPUT);
  
  lcd.begin(16, 2);
//  BACKLIGHT;
  lcd.setCursor( 0, 0 );
  lcd.print( BANNER_BRBN ); // display version banner
  Serial.begin(BAUD);
  amb.init( AMB_FILTER );  // initialize ambient temp filtering

#ifdef EEPROM_BRBN
  // read calibration and identification data from eeprom
  if( eeprom.read( 0, (uint8_t*) &caldata, sizeof( caldata) ) == sizeof( caldata ) ) {
    Serial.println("# EEPROM data read: ");
    Serial.print("# ");
    Serial.print( caldata.PCB); Serial.print("  ");
    Serial.println( caldata.version );
    Serial.print("# ");
    Serial.print( caldata.cal_gain, 4 ); Serial.print("  ");
    Serial.println( caldata.K_offset, 2 );
    lcd.setCursor( 0, 1 ); // echo EEPROM data to LCD
    lcd.print( caldata.PCB );
    adc.setCal( caldata.cal_gain, caldata.cal_offset );
    amb.setOffset( caldata.K_offset );
  }
  else { // if there was a problem with EEPROM read, then use default values
    Serial.println("no EEPROM data read");
    adc.setCal( CAL_GAIN, UV_OFFSET );
    amb.setOffset( AMB_OFFSET );
  }   
#else
  adc.setCal( CAL_GAIN, UV_OFFSET );
  amb.setOffset( AMB_OFFSET );
#endif

  // write header to serial port
  Serial.print("# time,ambient,T0,rate0");
  if( NCHAN >= 2 ) Serial.println(",T1,rate1");
  
  fT[0].init( BT_FILTER ); // digital filtering on BT
  fT[1].init( ET_FILTER ); // digital filtering on ET
  
  digitalWrite(BACKLIGHT_PIN, 1);  //turn on backlight
  delay( 500 );
  nextLoop = 1000;
//  first = true;
  lcd.clear();
}

// -----------------------------------------------------------------
void loop()
{
  float idletime;

//  buttonValue = NOBUTTON;
//  button_pushed = false;  
 // timestamp = float( millis() ) * 0.01;
  // update on even 1 second boundaries
 // while ( millis() < nextLoop ) { // delay until time for next loop
  if ( millis() > nextLoop ) { // delay until time for next loop
 //   if (button_pushed == false) { 
 //     Serial.println("loop");
      buttonValue = get_button();       
      if (buttonValue == BUTTON_1) {
        shot_flag = true;
        start_shot = timestamp;
        button_pushed == true;
        Serial.println("push 1a");
        }
      else if (buttonValue == BUTTON_2) {
        shot_flag = false;
        button_pushed == true;
        Serial.println("push 2a");
        }
      else if (buttonValue == BUTTON_3) {
        button_pushed == true;
        Serial.println("push 3a");
        if (back_flag == true) {
          back_flag = false;
          }
        else if (back_flag == false) {
          back_flag = true;
          }
        }
      else if (buttonValue == BUTTON_4) {
        button_pushed == true;
        if (back_flag == true) {
  //        back_flag = false;
          }
        else if (back_flag == false) {
  //        back_flag = true;
          }
        }
 //     }
    

   
  //nextLoop += 1000; // time mark for start of next update 
  nextLoop += 100; // time mark for start of next update 
  timestamp = float( millis() ) * 0.001;
  get_samples(); // retrieve values from MCP9800 and MCP3424

  logger(); // output results to serial port

  }
//if (back_flag == true) {
//  }
//else if (back_flag == false) {
//  digitalWrite(BACKLIGHT_PIN, 0);
//  }
//delay (50);

}
