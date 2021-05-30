
// *********************************************************************************************************
// Astrophotography Barn Door (dual axis) and Camera controller with ASCOM autoguide interface
// Author: Craig Hoffmann  Jan 2021
//
// For more details refer to...   https://github.com/CraigHoffmann
//
// Driver Commands:
//  HI         Return OK to indicate controller is alive
//  G?         Return 0 if not currently pulse guiding or 1 if pulse guiding in progress
//  GN xxxxx   Guide North for xxxxx mS - Return OK immediately command received
//  GS, GE, GW As per GN above
//
// All commands sent by driver will be terminated by CR (should strip any LF sent from terminal interfaces)
// All command responses should be terminated by CR only
//
// *********************************************************************************************************

#include "src/myCmdArduino/myCmdArduino.h"    // Modified version of cmdArduino installed by library manager
#include <math.h>
#include <EEPROM.h>
#define EEPROM_HAS_DATA 888          // Used to flag s/w that EEPROM has been written data before


// *************************** USER CONFIGURATION FOR (MAIN) RA AXIS ***************************************
//                      Ensure all values to include decimal so calcs as double

#define RA_BARN_DOOR_RADIUS_mm         307.0        // distance from hinge point to thread (center of each)
#define RA_THREAD_PITCH_mm             1.05833      
#define RA_MOTOR_STEPS_PER_360deg      4076.0       // See note below - you may need to experiment
#define RA_SMALL_GEAR_TEETH            10.0
#define RA_BIG_GEAR_TEETH              55.0 
#define DEFAULT_GUIDE_RATE             0.5         // How much faster/slower than the sidereal rate (both axis)

// *************************** USER CONFIGURATION FOR DEC (ALT) AXIS ***************************************

#define ALT_AXIS_RADIUS_mm             260.0
#define ALT_THREAD_PITCH_mm            1.05833
#define ALT_MOTOR_STEPS_PER_360deg     4076.0       // See note below - you may need to experiment
#define ALT_SMALL_GEAR_TEETH           10.0
#define ALT_BIG_GEAR_TEETH             55.0 

// NOTE: In halfsetp mode one revolution of the output shaft of a 28BYJ-48 stepper takes 4096 steps, so 250 steps/sec
// results in roughly 3.5 rpm.  Important note: Manufacturers usually specify that the motors have a 64:1 gear reduction. 
// Some members of the Arduino Forums noticed that this wasn’t correct and so they took some motors apart to check 
// the actual gear ratio. They determined that the exact gear ratio is in fact 63.68395:1, which results in 
// approximately 4076 steps per full revolution of the output shaft (in half step mode).
//
// ********************************** END USER CONFIGURATION ************************************************



#define SHUTTER_OPEN 60              // Default Number Seconds Shutter is open
#define SHUTTER_DELAY 3              // Number Seconds before next Shutter trigger
#define PROCESS_PERIOD_mS 1000       // 1 second

#define TWOPI   6.28318530717959
#define SECONDS_IN_SIDEREAL_DAY   86164.0916
#define RA_TRACK_SPEED ((((TWOPI * RA_BARN_DOOR_RADIUS_mm) / RA_THREAD_PITCH_mm) * (RA_BIG_GEAR_TEETH / RA_SMALL_GEAR_TEETH)) / SECONDS_IN_SIDEREAL_DAY * RA_MOTOR_STEPS_PER_360deg)
#define RA_MAX_ADJUSTMENT_nS 10000
#define MAX_GUIDE_TIME_mS 60000

#define ALT_EQUIV_RA_SPEED ((((TWOPI * ALT_AXIS_RADIUS_mm) / ALT_THREAD_PITCH_mm) * (ALT_BIG_GEAR_TEETH / ALT_SMALL_GEAR_TEETH)) / SECONDS_IN_SIDEREAL_DAY * ALT_MOTOR_STEPS_PER_360deg)

#define RA_MOTOR 0
#define DEC_MOTOR 1

#define RA_GUIDE_BIT 0x01
#define DEC_GUIDE_BIT 0x02

#define RA_MotorPin1  6     // IN1 on the ULN2003 driver 1
#define RA_MotorPin2  5     // IN2 on the ULN2003 driver 1
#define RA_MotorPin3  4     // IN3 on the ULN2003 driver 1
#define RA_MotorPin4  3     // IN4 on the ULN2003 driver 1

#define DEC_MotorPin1  10    // IN1 on the ULN2003 driver 2
#define DEC_MotorPin2  9     // IN2 on the ULN2003 driver 2
#define DEC_MotorPin3  8     // IN3 on the ULN2003 driver 2
#define DEC_MotorPin4  7     // IN4 on the ULN2003 driver 2



// EEPROM stuff
struct EEPROM_Data_Object {
  unsigned long RA_StepAdjust_nS;
  unsigned int ShutterSpeed_S;
};

EEPROM_Data_Object MySaveData = {
  0,
  SHUTTER_OPEN
};


// Camera shutter timing variables
const int ShutterPin = A0;          // pin used for shutter control
int ShutterSeconds = 0;
unsigned int ShutterSpeed_S = SHUTTER_OPEN;
unsigned long PreviousMillis;

byte isGuiding = 0;

// RA Axis variables
unsigned long RA_PreviousStep_nS;
unsigned long RA_PreviousStep_uS;
unsigned long RA_CurrentStepPeriod_uS;
unsigned long RA_GuideTimeout_mS;
unsigned long RA_GuideStart_mS;
long RA_StepAdjust_nS = 0;  
byte RA_StepCount = 0;
float RA_guide = 0.0;


// DEC Axis variables
float DEC_Guide = 0;
unsigned long DEC_PreviousStep_uS;
unsigned long DEC_CurrentStepPeriod_uS;
unsigned long DEC_GuideTimeout_mS;
unsigned long DEC_GuideStart_mS;
byte DEC_StepCount = 0;



void setup() 
{
  // init the command line and set it for a speed of 57600
  cmd.begin(115200);

  // Add functions for dealing with command line input
  cmd.add("shutter", shutter);
  cmd.add("speed", shutter_speed);
  cmd.add("track", tracking);
  cmd.add("help", help);
  cmd.add("echo", echo);
  cmd.add("CONNECT", ascom_connect);
  cmd.add("DISCONNECT", ascom_disconnect);
  cmd.add("RA0", ascom_RA_reset);
  cmd.add("RA+", ascom_RA_inc);
  cmd.add("RA-", ascom_RA_dec);
  cmd.add("DEC0", ascom_DEC_reset);
  cmd.add("DEC+", ascom_DEC_inc);
  cmd.add("DEC-", ascom_DEC_dec);

  cmd.add("HI", ascom_HI);
  cmd.add("GQ", ascom_Guide_Query);
  cmd.add("GN", ascom_Guide_North);
  cmd.add("GS", ascom_Guide_South);
  cmd.add("GE", ascom_Guide_East);
  cmd.add("GW", ascom_Guide_West);
  cmd.add("GR", ascom_Guide_Rate_Query);

  ReadEEPROMData();
    
  pinMode(ShutterPin, OUTPUT);

  pinMode(RA_MotorPin1, OUTPUT);
  pinMode(RA_MotorPin2, OUTPUT);
  pinMode(RA_MotorPin3, OUTPUT);
  pinMode(RA_MotorPin4, OUTPUT);

  pinMode(DEC_MotorPin1, OUTPUT);
  pinMode(DEC_MotorPin2, OUTPUT);
  pinMode(DEC_MotorPin3, OUTPUT);
  pinMode(DEC_MotorPin4, OUTPUT);

  digitalWrite(ShutterPin, LOW);   // Shutter closed
  
  ShutterSeconds=-1;
  PreviousMillis = millis();
  RA_PreviousStep_uS = micros();
  RA_PreviousStep_nS = 0;

  RA_CurrentStepPeriod_uS = 1000000.0 / RA_TRACK_SPEED;        // Reset to default step period (in uS)
  RA_StepAdjust_nS = GetRemaining_nS(1000000.0 / RA_TRACK_SPEED);
  //Serial.println("calculations:");
  //Serial.println(RA_CurrentStepPeriod_uS);
  //Serial.println(RA_StepAdjust_nS);
  //Serial.println(RA_TRACK_SPEED);
  halfStep(RA_StepCount, RA_MOTOR);

  DEC_PreviousStep_uS = micros();
  DEC_CurrentStepPeriod_uS = (1000000.0 / ALT_EQUIV_RA_SPEED) / DEFAULT_GUIDE_RATE;       // Reset to default step period (in uS)
  halfStep(DEC_StepCount, DEC_MOTOR);

  RA_GuideTimeout_mS = MAX_GUIDE_TIME_mS;
  RA_GuideStart_mS = millis();
  DEC_GuideTimeout_mS = MAX_GUIDE_TIME_mS;
  DEC_GuideStart_mS = millis();

  while (!Serial);
  //Serial.println("\nINITIALIZED#");
}


void loop() 
{


  // Process every step period
  if ((micros() - RA_PreviousStep_uS) >= RA_CurrentStepPeriod_uS ) 
  {
    RA_PreviousStep_uS = RA_PreviousStep_uS + RA_CurrentStepPeriod_uS;                              // Update the last period
//    RA_CurrentStepPeriod_uS = (1000000.0 / (RA_TRACK_SPEED + (RA_TRACK_SPEED * RA_guide)));         // Update the tracking step period

    RA_CurrentStepPeriod_uS = (1000000.0 / RA_TRACK_SPEED) - ((1000000.0 / RA_TRACK_SPEED) * RA_guide);         // Update the tracking step period
    
//Serial.println(RA_CurrentStepPeriod_uS);
    // Fine tuning the step period (StepAdust_nS should always be positive)
    if (RA_guide == 0.0)
    {
      RA_PreviousStep_nS = RA_PreviousStep_nS + RA_StepAdjust_nS;       
      while (RA_PreviousStep_nS >= 1000)
      {
        RA_PreviousStep_nS = RA_PreviousStep_nS - 1000;
        RA_CurrentStepPeriod_uS = RA_CurrentStepPeriod_uS + 1;
      }
    }
    
    halfStep(RA_StepCount++, RA_MOTOR);
  }

  // Check if the current RA axis guide pulse has ended
  if ((millis() - RA_GuideStart_mS) >= RA_GuideTimeout_mS)
  {
    isGuiding = isGuiding & (!RA_GUIDE_BIT);
    RA_guide = 0.0;
    RA_GuideTimeout_mS = MAX_GUIDE_TIME_mS;   // Don't worry about checking again for a while
    RA_GuideStart_mS = millis();
  }


  // Process every DEC step period
  if ((micros() - DEC_PreviousStep_uS) >= DEC_CurrentStepPeriod_uS ) 
  {
    DEC_PreviousStep_uS = DEC_PreviousStep_uS + DEC_CurrentStepPeriod_uS;                // Update the last period
//    DEC_CurrentStepPeriod_uS = 1000000.0 / (ALT_EQUIV_RA_SPEED * DEFAULT_GUIDE_RATE);        // For simplicity just use the default RA guide - update this!!
//    DEC_CurrentStepPeriod_uS = (1000000.0 / ALT_EQUIV_RA_SPEED) - ((1000000.0 / ALT_EQUIV_RA_SPEED) * DEFAULT_GUIDE_RATE);        // For simplicity just use the default RA guide - update this!!

    if (DEC_Guide < 0.0)
    {
      DEC_CurrentStepPeriod_uS = ((1000000.0 / ALT_EQUIV_RA_SPEED) / abs(DEC_Guide));        // For simplicity just use the default RA guide - update this!!
      halfStep(DEC_StepCount--, DEC_MOTOR);
    }
    else if (DEC_Guide > 0.0)
    {
      DEC_CurrentStepPeriod_uS = ((1000000.0 / ALT_EQUIV_RA_SPEED) / abs(DEC_Guide));        // For simplicity just use the default RA guide - update this!!
      halfStep(DEC_StepCount++, DEC_MOTOR);
    }

  }

  // Check if the current DEC axis guide pulse has ended
  if ((millis() - DEC_GuideStart_mS) >= DEC_GuideTimeout_mS)
  {
    isGuiding = isGuiding & (!DEC_GUIDE_BIT);
    DEC_Guide = 0.0;
    DEC_GuideTimeout_mS = MAX_GUIDE_TIME_mS;   // Don't worry about checking again for a while
    DEC_GuideStart_mS = millis();
  }




  // Process every second
  if ((millis() - PreviousMillis) >= PROCESS_PERIOD_mS) 
  {
    PreviousMillis += PROCESS_PERIOD_mS;
    // Do stuff here - currently not required

    ShutterSeconds++;
    if (ShutterSeconds <= ShutterSpeed_S)
    {
      // Shutter Open
      digitalWrite(ShutterPin, HIGH);   // Shutter closed
    }
    else
    {
      // Shutter Closed
      digitalWrite(ShutterPin, LOW);   // Shutter closed
    }

    if (ShutterSeconds >= (ShutterSpeed_S + SHUTTER_DELAY))
    {
      ShutterSeconds = 0;
    }
  
  }

  // Check if there is any serial port activity and process
  cmd.poll();

}


void shutter(int argCnt, char **args)
{
  // Serial.println("rcvd cmd");

  if (argCnt>1)     // Must send "shutter close" or "shutter open"
  {
    //  Serial.println("have args");
    //  Serial.println(cmd.conv(args[1]));
    if (strcmp(args[1],"open")==0)   // if match then =0
    {
      Serial.println("**open shutter**");
    }
    else if (strcmp(args[1],"close")==0)     // if match then =0
    {
      Serial.println("**close shutter**");
    }
    
  }
}

void shutter_speed(int argCnt, char **args)
{
  unsigned int getSpeed = 0;
  getSpeed = ShutterSpeed_S;

  if (argCnt>1)     // Must send "track xxxxx" where xxxxx is a positive integer to increase step period or negative to decrease period (nano seconds per step)
  {
    getSpeed = cmd.conv(args[1]);
    if (getSpeed < 1)    // minimum 1 second
    {
      getSpeed = 1;
    }
    else if (getSpeed > 3600)    // maximum 1hr
    {
      getSpeed = 3600;
    }
    ShutterSpeed_S = getSpeed;
    WriteEEPROMData();
  }

  Serial.print("Shutter Speed set to ");    
  Serial.println(ShutterSpeed_S);    
}

void tracking(int argCnt, char **args)
{
  long adjustBy = 0;
  // Serial.println("rcvd cmd");
  adjustBy = RA_StepAdjust_nS;
  //Serial.println(adjustBy);
  
  if (argCnt>1)     // Must send "track xxxxx" where xxxxx is a positive integer to increase step period or negative to decrease period (nano seconds per step)
  {
    adjustBy = cmd.conv(args[1]);
    if (adjustBy > (RA_MAX_ADJUSTMENT_nS))
    {
      adjustBy = RA_MAX_ADJUSTMENT_nS;
    }
    else if (adjustBy < (-RA_MAX_ADJUSTMENT_nS))
    {
      adjustBy = (-RA_MAX_ADJUSTMENT_nS);
    }
    RA_StepAdjust_nS = adjustBy;
    WriteEEPROMData();
  }
  
  Serial.print("Tracking compensation set to ");    
  Serial.println(adjustBy);    
}

void help(int argCnt, char **args)
{
    Serial.println("------------------commands------------------");    
    Serial.println("track +/-xxxxxx to adjust tracking step period in nS");    
    Serial.println("shutter open|close|auto|single");    
    Serial.println("speed xxxx sets shutter speed in seconds (1 to 3600)");    
    Serial.println("echo on|off");    

}

// Turn command line echo on or off
void echo(int argCnt, char **args)
{
  if (argCnt>1)     
  {
    if (strcmp(args[1],"on")==0)   // if match then =0
    {
      cmd.echo(1);   
      Serial.println("echo turned on");    
    }
    else if (strcmp(args[1],"off")==0)   // if match then =0
    {
      cmd.echo(0);   
      Serial.println("echo turned off");    
    }
  }
}

// ********************************** NEW ASCOM INTERFACE ****************************

void ascom_HI(int argCnt, char **args)
{
  if (argCnt==1)       // The first arg is the command arg[0]     
  {
      cmd.echo(0);     // Ensure echo is off for direct ascom connection
      Serial.println("OK");    
  }
  else
  {
    Serial.println("ERR");
  }
}

void ascom_Guide_Rate_Query(int argCnt, char **args)
{
  if (argCnt>1)     
  {
    if (strcmp(args[1],"RA")==0)   // if match then =0
    {
      Serial.println(DEFAULT_GUIDE_RATE);    
    }
    else if (strcmp(args[1],"DEC")==0)   // if match then =0
    {
      Serial.println(DEFAULT_GUIDE_RATE);    
    }
  }
}


void ascom_Guide_Query(int argCnt, char **args)
{
  if (argCnt==1)       // The first arg is the command arg[0] so this checks there are no Args     
  {
    if (isGuiding != 0)
    {
      Serial.println("1");    // Guiding pulse in progress
    }
    else
    {
      Serial.println("0");    // Guiding pulse in progress
    }
  }
  else
  {
    Serial.println("ERR");
  }
}

void ascom_Guide_West(int argCnt, char **args)
{
  unsigned int Duration_mS = 0;
  
  if (argCnt>1)       // The first arg is the command arg[0] so this checks there is at least one parameter  
  {
    Duration_mS = cmd.conv(args[1]);
    if (Duration_mS < 0)
    {
      Duration_mS = 0;
    }
    else if (Duration_mS > MAX_GUIDE_TIME_mS)
    {
      Duration_mS = MAX_GUIDE_TIME_mS;
    }
    RA_guide = DEFAULT_GUIDE_RATE;
    RA_GuideTimeout_mS = Duration_mS;
    RA_GuideStart_mS = millis();
    isGuiding = isGuiding | RA_GUIDE_BIT;
  }
  else
  {
    RA_guide = 0.0;          // if no duration parameter default to 0 - stop N/S guiding 
    RA_GuideTimeout_mS = 0;
    isGuiding = isGuiding & (!RA_GUIDE_BIT);
  }
  Serial.println("OK");
}

void ascom_Guide_East(int argCnt, char **args)
{
  unsigned int Duration_mS = 0;
  
  if (argCnt>1)       // The first arg is the command arg[0] so this checks there is at least one parameter  
  {
    Duration_mS = cmd.conv(args[1]);
    if (Duration_mS < 0)
    {
      Duration_mS = 0;
    }
    else if (Duration_mS > MAX_GUIDE_TIME_mS)
    {
      Duration_mS = MAX_GUIDE_TIME_mS;
    }
    RA_guide = -DEFAULT_GUIDE_RATE;
    RA_GuideTimeout_mS = Duration_mS;
    RA_GuideStart_mS = millis();
    isGuiding = isGuiding | RA_GUIDE_BIT;
  }
  else
  {
    RA_guide = 0.0;          // if no duration parameter default to 0 - stop N/S guiding 
    RA_GuideTimeout_mS = 0;
    isGuiding = isGuiding & (!RA_GUIDE_BIT);
  }
  Serial.println("OK");
}


void ascom_Guide_North(int argCnt, char **args)
{
  unsigned int Duration_mS = 0;
  
  if (argCnt>1)       // The first arg is the command arg[0] so this checks there is at least one parameter  
  {
    Duration_mS = cmd.conv(args[1]);
    if (Duration_mS < 0)
    {
      Duration_mS = 0;
    }
    else if (Duration_mS > MAX_GUIDE_TIME_mS)
    {
      Duration_mS = MAX_GUIDE_TIME_mS;
    }
    DEC_Guide = -DEFAULT_GUIDE_RATE;
    DEC_GuideTimeout_mS = Duration_mS;
    DEC_GuideStart_mS = millis();
    isGuiding = isGuiding | DEC_GUIDE_BIT;
  }
  else
  {
    DEC_Guide = 0.0;          // if no duration parameter default to 0 - stop N/S guiding 
    DEC_GuideTimeout_mS = 0;
    isGuiding = isGuiding & (!DEC_GUIDE_BIT);
  }
  Serial.println("OK");
}


void ascom_Guide_South(int argCnt, char **args)
{
  unsigned int Duration_mS = 0;
  
  if (argCnt>1)       // The first arg is the command arg[0] so this checks there is at least one parameter  
  {
    Duration_mS = cmd.conv(args[1]);
    if (Duration_mS < 0)
    {
      Duration_mS = 0;
    }
    else if (Duration_mS > MAX_GUIDE_TIME_mS)
    {
      Duration_mS = MAX_GUIDE_TIME_mS;
    }
    DEC_Guide = DEFAULT_GUIDE_RATE;
    DEC_GuideTimeout_mS = Duration_mS;
    DEC_GuideStart_mS = millis();
    isGuiding = isGuiding | DEC_GUIDE_BIT;
  }
  else
  {
    DEC_Guide = 0.0;          // if no duration parameter default to 0 - stop N/S guiding 
    DEC_GuideTimeout_mS = 0;
    isGuiding = isGuiding & (!DEC_GUIDE_BIT);
  }
  Serial.println("OK");
}





// ******************************* OLD INTERFACE *******************************

void ascom_connect(int argCnt, char **args)
{
  if (argCnt==1)       // The first arg is the command arg[0]     
  {
      cmd.echo(0);
      Serial.println("OK#");    
  }
}

void ascom_disconnect(int argCnt, char **args)
{
  if (argCnt==1)       // The first arg is the command arg[0]     
  {
      Serial.println("OK#");    
  }
}

void ascom_RA_reset(int argCnt, char **args)
{
  if (argCnt==1)       // The first arg is the command arg[0]
  {
      RA_guide = 0.0;
      Serial.println("OK#");    
  }
}

void ascom_RA_inc(int argCnt, char **args)
{
  if (argCnt==1)       // The first arg is the command arg[0]    
  {
      RA_guide = DEFAULT_GUIDE_RATE;   // increase speed
      Serial.println("OK#");    
  }
}

void ascom_RA_dec(int argCnt, char **args)
{
  if (argCnt==1)       // The first arg is the command arg[0]  
  {
      RA_guide = -DEFAULT_GUIDE_RATE;    // decrease speed
      Serial.println("OK#");    
  }
}

void ascom_DEC_reset(int argCnt, char **args)
{
  if (argCnt==1)       // The first arg is the command arg[0]    
  {
      DEC_Guide = 0;
      Serial.println("OK#");    
  }
}

void ascom_DEC_inc(int argCnt, char **args)
{
  if (argCnt==1)       // The first arg is the command arg[0]   
  {
      DEC_Guide = 1;
      Serial.println("OK#");    
  }
}

void ascom_DEC_dec(int argCnt, char **args)
{
  if (argCnt==1)       // The first arg is the command arg[0]  
  {
      DEC_Guide = -1;
      Serial.println("OK#");    
  }
}

// *************************** END OLD INTERFACE *******************************

// 4 pin half step function
void halfStep(byte step, byte Motor)
{
  switch (step & 0x7)
  {
    case 0:    // 1000
      setOutputPins(0b1000,Motor);
      break;
      
    case 1:    // 1100
      setOutputPins(0b1100,Motor);
      break;
      
    case 2:    // 0100
      setOutputPins(0b0100,Motor);
      break;
      
    case 3:    // 0110
      setOutputPins(0b0110,Motor);
      break;
      
    case 4:    // 0010
      setOutputPins(0b0010,Motor);
      break;
      
    case 5:    // 0011
      setOutputPins(0b0011,Motor);
      break;
      
    case 6:    // 0001
      setOutputPins(0b0001,Motor);
      break;
      
    case 7:    //1001
      setOutputPins(0b1001,Motor);
      break;
  }
}


// Set the motor drive output pins
//
void setOutputPins(uint8_t mask, byte Motor)
{
  if (Motor == RA_MOTOR)
  {
    digitalWrite(RA_MotorPin1, (mask & 0x01) );        // Active High drive
    digitalWrite(RA_MotorPin2, ((mask>>1) & 0x01) );
    digitalWrite(RA_MotorPin3, ((mask>>2) & 0x01) );
    digitalWrite(RA_MotorPin4, ((mask>>3) & 0x01) );
  }
  else if (Motor == DEC_MOTOR)
  {
    digitalWrite(DEC_MotorPin1, (mask & 0x01) );        // Active High drive
    digitalWrite(DEC_MotorPin2, ((mask>>1) & 0x01) );
    digitalWrite(DEC_MotorPin3, ((mask>>2) & 0x01) );
    digitalWrite(DEC_MotorPin4, ((mask>>3) & 0x01) );
  }
}


void ReadEEPROMData(void)
{
  unsigned int CheckValue=0;
  EEPROM.get(0,CheckValue);
  if (CheckValue == EEPROM_HAS_DATA)
  {
    EEPROM.get(8 ,MySaveData);
    RA_StepAdjust_nS = MySaveData.RA_StepAdjust_nS;
    ShutterSpeed_S = MySaveData.ShutterSpeed_S;
  }
  else
  {
    WriteEEPROMData();
  }
  
}


void WriteEEPROMData(void)
{
  unsigned int CheckValue;
  CheckValue = EEPROM_HAS_DATA;
  MySaveData.RA_StepAdjust_nS = RA_StepAdjust_nS;
  MySaveData.ShutterSpeed_S = ShutterSpeed_S;
  EEPROM.put(0,CheckValue);
  EEPROM.put(8 ,MySaveData);
}



double GetRemaining_nS(double number)
{
    double fraction, integer;

    fraction = modf(number, &integer);
    // Serial.println("modf:");
    //Serial.println(number);
    //Serial.println(integer);
    //Serial.println(fraction);
    
    return (fraction * 1000.0);
}
