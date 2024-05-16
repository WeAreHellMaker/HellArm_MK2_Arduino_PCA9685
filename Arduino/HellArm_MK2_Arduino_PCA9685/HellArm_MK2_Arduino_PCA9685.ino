/*
  ~~~~~Prerequisites~~~~~
  ServoEasing.hpp   https://github.com/ArminJo/ServoEasing
                    Important: You must uncomment the line #define USE_PCA9685_SERVO_EXPANDER in the file ServoEasing.h

  PS2X_lib.h        https://github.com/madsci1016/Arduino-PS2X

  ~~~~~Details~~~~~~~~~~~~

  Program title : HellArm_MK2_Arduino_PCA9685  EEZYbotARM control
  Author        : Duke Kim (nicebug@naver.com)
  Date          : 2024-05-15
  Purpose       : Control the EEZYbotARM with a PS2 Controller

  Not work      : Communication with the PC is still incomplete
   
  ~~~~~Licence~~~~~~~~~~~~
   MIT License

  ~~~~~Code From~~~~~~~~~~~~

  https://github.com/meisben/easyEEZYbotARM
  https://github.com/marcelolarios/R2D2-Sound-Generator

   ~~~~~Version Control~~~
   v1.00 -  Control with a PS@ Controller
            Add R2D2 start sound
            Add a warning sound when the robotic arm exceeds its range of motion   
*/

//********************************************************************************
//  Library Includes.
//********************************************************************************
#include <Arduino.h>

#include <PS2X_lib.h>
#include <ServoEasing.hpp>

//********************************************************************************
//  Definitions
//********************************************************************************
#define VERSION           "1.0"


//--------------------------------------------------------------------------------
//  IO Pins
//--------------------------------------------------------------------------------
#define PIN_LED             4
#define PIN_BUZZER          5     //  buzzer to featherwing pin 5, 470 ohm resistor  


//  Set the connection pins for the PS2 controller
#define PIN_PS2_DAT        13
#define PIN_PS2_CMD        11
#define PIN_PS2_SEL        10
#define PIN_PS2_CLK        12

#define pressures         false
#define rumble            false

#define ACTION_TIME_PERIOD 1000

//--------------------------------------------------------------------------------
//  Servo
//--------------------------------------------------------------------------------
const int SERVO1_PIN = 0; // servo pin for joint 1
const int SERVO2_PIN = 1; // servo pin for joint 2
const int SERVO3_PIN = 2; // servo pin for joint 3

const int SERVO_EE_PIN = 3; // servo pin for end effector

#define LINK_Q1_ANGLE_MIN 0
#define LINK_Q1_ANGLE_MAX 180

#define LINK_Q2_ANGLE_MIN 60
#define LINK_Q2_ANGLE_MAX 160

#define LINK_Q3_ANGLE_MIN 60
#define LINK_Q3_ANGLE_MAX 120

#define LINK_EE_ANGLE_MIN 0
#define LINK_EE_ANGLE_MAX 60

#define LINK_Q1_ANGLE_DEFAULT     90.0
#define LINK_Q2_ANGLE_DEFAULT     90.0
#define LINK_Q3_ANGLE_DEFAULT     90.0

#define LINK_EE_ANGLE_DEFAULT     0.0

//********************************************************************************
//  Variables
//********************************************************************************

//-------- Variables for receiving serial data -------------
const byte buffSize = 40;
char inputBuffer[buffSize];

const char startMarker = '<';
const char endMarker = '>';

byte bytesRecvd = 0;
boolean readInProgress = false;
boolean newDataFromPC = false;

char messageFromPC[buffSize] = {0};

// -------- Variables to hold time -------------
unsigned long curMillis; // Variable for current time

//--------------------------------------------------------------------------------
//  Variables to hold the parsed data
//--------------------------------------------------------------------------------
float g_fAngleFromPC_Q1 = LINK_Q1_ANGLE_DEFAULT;      //  initial values are mid range for joint angles
float g_fAngleFromPC_Q2 = LINK_Q2_ANGLE_DEFAULT;
float g_fAngleFromPC_Q3 = LINK_Q3_ANGLE_DEFAULT;

float g_fAngleFromPC_EE = LINK_EE_ANGLE_DEFAULT;                 

float g_fLast_servoAngle_q1 = g_fAngleFromPC_Q1;      //  initial values are mid range for joint angles
float g_fLast_servoAngle_q2 = g_fAngleFromPC_Q2;
float g_fLast_servoAngle_q3 = g_fAngleFromPC_Q3;

float g_fLast_servoAngle_EE = g_fAngleFromPC_EE;

//********************************************************************************
//  Instatiate clasess for libraries
//********************************************************************************
PS2X ps2x;

ServoEasing Servo1(PCA9685_DEFAULT_ADDRESS, &Wire);
ServoEasing Servo2(PCA9685_DEFAULT_ADDRESS, &Wire);
ServoEasing Servo3(PCA9685_DEFAULT_ADDRESS, &Wire);

ServoEasing Servo0(PCA9685_DEFAULT_ADDRESS, &Wire);


int   g_nPS2Error = 0;
byte  g_bPS2Vibrate = 0;

//********************************************************************************
//
//********************************************************************************
void config_gamepad()
{
  //
  //  GamePad(clock,command,attention,data,Pressures?, Rumble?) check for error
  //
  g_nPS2Error = ps2x.config_gamepad( PIN_PS2_CLK, PIN_PS2_CMD, PIN_PS2_SEL, PIN_PS2_DAT, pressures, rumble);

 if(g_nPS2Error == 0)
 {
    Serial.print("Found Controller, configured successful ");
    Serial.print("pressures = ");

    if (pressures)
      Serial.println("true ");
    else
      Serial.println("false");
    Serial.print("rumble = ");
    if (rumble)
      Serial.println("true)");
    else
      Serial.println("false");

    Serial.println("Connected Game Pad");

    PlayR2D2();
  }  

}


//********************************************************************************
//  START OF PROGRAM (Setup)
//********************************************************************************
void setup()
{
  //
  //  Setup pins
  //
  pinMode(PIN_LED, OUTPUT); 
  pinMode(PIN_BUZZER, OUTPUT);

  blinkLED();

  // Play tone so we can tell if the board accidently reset
  playTone();

  // Begin serial communications
  Serial.begin(115200);

  
  // Wait for serial communications to start before continuing
  while (!Serial)
    ; // delay for Leonardo
  

  config_gamepad();

  //
  //  Just to know which program is running on my Arduino
  //
  Serial.println(F("START Version " VERSION " from " __DATE__));

  //
  //  Attach servo to pin
  //
  Servo1.attach(SERVO1_PIN);
  Servo2.attach(SERVO2_PIN);
  Servo3.attach(SERVO3_PIN);

  Servo0.attach(SERVO_EE_PIN);

  //
  //  Set servo to start position.
  //
  Servo1.setEasingType(EASE_CUBIC_IN_OUT);
  Servo2.setEasingType(EASE_CUBIC_IN_OUT);
  Servo3.setEasingType(EASE_CUBIC_IN_OUT);

  Servo0.setEasingType(EASE_CUBIC_IN_OUT);  // end effector


  Servo1.write(g_fLast_servoAngle_q1);
  Servo2.write(g_fLast_servoAngle_q2);
  Servo3.write(g_fLast_servoAngle_q3);

  Servo0.write(g_fLast_servoAngle_EE);         // end effector


  // Just wait for servos to reach position
  delay(500); // delay() is OK in setup as it only happens once


  randomSeed(analogRead(0));

  // tell the PC we are ready
  Serial.println("<Hell ARM is ready>");
}


//********************************************************************************
//  MAIN PROGRAM (Loop)
//********************************************************************************
void loop()
{

  // This part of the loop for the serial communication is not inside a timer -> it happens very quickly
  curMillis = millis(); // get current time
  getDataFromPC();      // receive data from PC and save it into inputBuffer

  // need if statement -> flag to say if new data is available
  if (newDataFromPC == true)
  {
    processMessageFromPC();     // Processes text message from PC

    ActionInstructionsFromPC(); // Arrange for things to move, beep, light up

    replyToPC();                // Reply to PC
  }

  //------------------------------------------------------------------------------
  //  Check PS2 Controller
  //------------------------------------------------------------------------------  
  if( g_nPS2Error == 0 ) 
  {

    // read controller and set large motor to spin at 'vibrate' speed
    ps2x.read_gamepad(false, g_bPS2Vibrate);

    if(ps2x.ButtonReleased(PSB_START))
    { 
      Serial.println("Start is being held");
    }

    if(ps2x.ButtonReleased(PSB_L1))
    { 
      Serial.println("PSB_L1");      
    }

    if(ps2x.ButtonReleased(PSB_SELECT))
    {
      g_fAngleFromPC_Q1 = LINK_Q1_ANGLE_DEFAULT;
      g_fAngleFromPC_Q2 = LINK_Q2_ANGLE_DEFAULT;
      g_fAngleFromPC_Q3 = LINK_Q3_ANGLE_DEFAULT;

      g_fAngleFromPC_EE = LINK_EE_ANGLE_DEFAULT; 
    }

    //
    //  Link Servo Q1
    //    
    if(ps2x.Button(PSB_PAD_LEFT))
    {
      g_fAngleFromPC_Q1 --;

    }
    
    if(ps2x.Button(PSB_PAD_RIGHT))
    {
      g_fAngleFromPC_Q1 ++;

    }

    //
    //  Link Servo Q2
    // 
    if(ps2x.Button(PSB_PAD_UP)) 
    {      
      g_fAngleFromPC_Q2 --;
      delay(50);
    }

    if(ps2x.Button(PSB_PAD_DOWN))
    {
      g_fAngleFromPC_Q2 ++;
      delay(50);      
    }  

    //
    //  Link Servo Q3
    // 
    if(ps2x.Button(PSB_TRIANGLE))
    {
      g_fAngleFromPC_Q3 ++;
      delay(50);            
    }

    if(ps2x.Button(PSB_CROSS))
    {
      g_fAngleFromPC_Q3 --;    
      delay(50);
    }

    //
    //  Link Servo EE
    //
    if(ps2x.Button(PSB_SQUARE))
    {
      g_fAngleFromPC_EE --;
      delay(10);
    }

    if(ps2x.Button(PSB_CIRCLE))
    {
      g_fAngleFromPC_EE ++;
      delay(10);      
    }
    
    ActionInstructionsFromPC();      

    delay(20); 
  }
  else
  {
    //
    //  Setting PS2 Controller
    //
    config_gamepad();
    delay(100);
  }

}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
              FUNCTIONS FOR MAKING THINGS MOVE OR LIGHT UP OR BEEP!              *
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

//~~~~~~~~~~~~~Fuction: Blink LED~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void blinkLED()
{
  digitalWrite( PIN_LED, HIGH);
  delay(100);
  digitalWrite( PIN_LED, LOW);
  delay(100);
}


//********************************************************************************
//  Sound
//********************************************************************************
void playTone()
{
  tone(PIN_BUZZER, 650, 300);
}


//--------------------------------------------------------------------------------
//  R2DR Sound
//--------------------------------------------------------------------------------
void phrase1() 
{
    
    int k = random(1000,2000);
    digitalWrite(PIN_LED, HIGH);

    for (int i = 0; i <=  random(100,2000); i++)
    {       
        tone(PIN_BUZZER, k+(-i*2));          
        delay(random(.9,2));             
    } 

    digitalWrite(PIN_LED, LOW);   

    for (int i = 0; i <= random(100,1000); i++)
    {
        
        tone(PIN_BUZZER, k + (i * 10));          
        delay(random(.9,2));             
    } 
}

//--------------------------------------------------------------------------------
//
//--------------------------------------------------------------------------------
void phrase2() 
{
    
    int k = random(1000,2000);

    digitalWrite(PIN_LED, HIGH);  

    for (int i = 0; i <= random(100,2000); i++){
        
        tone(PIN_BUZZER, k+(i*2));          
        delay(random(.9,2));             
    } 
    digitalWrite(PIN_LED, LOW);   
    for (int i = 0; i <= random(100,1000); i++){
        
        tone(PIN_BUZZER, k + (-i * 10));          
        delay(random(.9,2));             
    } 
}

//--------------------------------------------------------------------------------
//
//--------------------------------------------------------------------------------
void PlayR2D2() 
{    
    int K = 2000;

    switch (random(1,7)) 
    {        
    case 1:phrase1(); break;
    case 2:phrase2(); break;
    case 3:phrase1(); phrase2(); break;
    case 4:phrase1(); phrase2(); phrase1();break;
    case 5:phrase1(); phrase2(); phrase1(); phrase2(); phrase1();break;
    case 6:phrase2(); phrase1(); phrase2(); break;
    }

    for (int i = 0; i <= random(3, 9); i++){
        
        digitalWrite(PIN_LED, HIGH);  
        tone(PIN_BUZZER, K + random(-1700, 2000));          
        delay(random(70, 170));  
        digitalWrite(PIN_LED, LOW);           
        noTone(PIN_BUZZER);         
        delay(random(0, 30));             
    } 
    noTone(PIN_BUZZER);    

    delay(random(2000, 4000));             
}


//********************************************************************************
//  Fuction: Action the instructions from the PC
//********************************************************************************
void ActionInstructionsFromPC()
{
  //
  //  Check the Servo move range : Min Max Angle
  //
  if( g_fAngleFromPC_Q1 < LINK_Q1_ANGLE_MIN || g_fAngleFromPC_Q1 >  LINK_Q1_ANGLE_MAX )
  {
    g_fAngleFromPC_Q1 = min( max( g_fAngleFromPC_Q1, LINK_Q1_ANGLE_MIN ), LINK_Q1_ANGLE_MAX );  

    playTone();
  }

  if( g_fAngleFromPC_Q2 < LINK_Q2_ANGLE_MIN || g_fAngleFromPC_Q2 > LINK_Q2_ANGLE_MAX )
  {
    g_fAngleFromPC_Q2 = min( max( g_fAngleFromPC_Q2, LINK_Q2_ANGLE_MIN ), LINK_Q2_ANGLE_MAX );

    playTone();
  }

  if( g_fAngleFromPC_Q3 < LINK_Q3_ANGLE_MIN || g_fAngleFromPC_Q3 > LINK_Q3_ANGLE_MAX )
  {
    g_fAngleFromPC_Q3 = min( max( g_fAngleFromPC_Q3, LINK_Q3_ANGLE_MIN ), LINK_Q3_ANGLE_MAX );

    playTone();
  }
  
  if( g_fAngleFromPC_EE < LINK_EE_ANGLE_MIN || g_fAngleFromPC_EE > LINK_EE_ANGLE_MAX )
  {
    g_fAngleFromPC_EE = min( max( g_fAngleFromPC_EE, LINK_EE_ANGLE_MIN ), LINK_EE_ANGLE_MAX );

    playTone();
  }

  //  
  //  Check if the joint angle has changed!
  //
  if (g_fAngleFromPC_Q1 != g_fLast_servoAngle_q1)
  { 
    int nMoveTime = abs( g_fAngleFromPC_Q1 - g_fLast_servoAngle_q1 );

    Servo1.startEaseToD(g_fAngleFromPC_Q1, nMoveTime);

    //    while (Servo3.isMovingAndCallYield()) {
    //      ; // no delays here to avoid break between forth and back movement
    //    }
  }

  if (g_fAngleFromPC_Q2 != g_fLast_servoAngle_q2)
  {
    int nMoveTime = abs( g_fAngleFromPC_Q2 - g_fLast_servoAngle_q2 );

    Servo2.startEaseToD(g_fAngleFromPC_Q2, nMoveTime);

    //    while (Servo3.isMovingAndCallYield()) {
    //      ; // no delays here to avoid break between forth and back movement
    //    }
  }

  if (g_fAngleFromPC_Q3 != g_fLast_servoAngle_q3)
  {
    int nMoveTime = abs( g_fAngleFromPC_Q3 - g_fLast_servoAngle_q3 );

    Servo3.startEaseToD(g_fAngleFromPC_Q3, nMoveTime);

    //    while (Servo3.isMovingAndCallYield()) {
    //      ; // no delays here to avoid break between forth and back movement
    //    }
  }

  if (g_fAngleFromPC_EE != g_fLast_servoAngle_EE)
  {
    int nMoveTime = abs( g_fAngleFromPC_EE - g_fLast_servoAngle_EE ) * 10;

    Servo0.startEaseToD(g_fAngleFromPC_EE, nMoveTime);

    //    while (Servo3.isMovingAndCallYield()) {
    //      ; // no delays here to avoid break between forth and back movement
    //    }
  }

  //
  //  Store current joint angle
  //
  g_fLast_servoAngle_q1 = g_fAngleFromPC_Q1;
  g_fLast_servoAngle_q2 = g_fAngleFromPC_Q2;
  g_fLast_servoAngle_q3 = g_fAngleFromPC_Q3;
  g_fLast_servoAngle_EE = g_fAngleFromPC_EE;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
              FUNCTIONS FOR RECEIVING DATA VIA SERIAL MONITOR                    *
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

//~~~~~~~~~~~~~Fuction: Receive data with start and end markers~~~~~~~~~~~~~~~~~~

void getDataFromPC()
{  
  // This function receives data from PC and saves it into inputBuffer

  if (Serial.available() > 0 && newDataFromPC == false)
  {

    char x = Serial.read();

    // the order of these IF clauses is significant

    if (x == endMarker)
    {
      readInProgress = false;
      newDataFromPC = true;
      inputBuffer[bytesRecvd] = 0;

      parseData();
    }

    if (readInProgress)
    {
      inputBuffer[bytesRecvd] = x;
      bytesRecvd++;
      if (bytesRecvd == buffSize)
      {
        bytesRecvd = buffSize - 1;
      }
    }

    if (x == startMarker)
    {
      bytesRecvd = 0;
      readInProgress = true;
    }
  }
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~Fuction: Split data into known component parts~~~~~~~~~~~~~~~~~~

void parseData()
{

  // split the data into its parts

  char *strtokIndx; // this is used by strtok() as an index

  strtokIndx = strtok(inputBuffer, ","); // get the first part - the string
  strcpy(messageFromPC, strtokIndx);     // copy it to messageFromPC

  strtokIndx = strtok(NULL, ",");  // this continues where the previous call left off
  g_fAngleFromPC_EE = atof(strtokIndx); // convert this part to a float -> to convert to an integer we would use atoi

  strtokIndx = strtok(NULL, ",");
  g_fAngleFromPC_Q1 = atof(strtokIndx); // convert this part to a float

  strtokIndx = strtok(NULL, ",");
  g_fAngleFromPC_Q2 = atof(strtokIndx); // convert this part to a float

  strtokIndx = strtok(NULL, ",");
  g_fAngleFromPC_Q3 = atof(strtokIndx); // convert this part to a float
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~Fuction: Send message back to PC~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void replyToPC()
{

  if (newDataFromPC)
  {
    newDataFromPC = false;

    Serial.print(F("<Msg "));
    Serial.print(messageFromPC);
    Serial.print(F(" g_fAngleFromPC_EE "));
    Serial.print(g_fAngleFromPC_EE);

    Serial.print(F(" g_fAngleFromPC_Q1 "));
    Serial.print(g_fAngleFromPC_Q1);

    Serial.print(F(" g_fAngleFromPC_Q2 "));
    Serial.print(g_fAngleFromPC_Q2);

    Serial.print(F(" g_fAngleFromPC_Q3 "));
    Serial.print(g_fAngleFromPC_Q3);

    Serial.print(F(" Time "));

    Serial.print(curMillis / 1000); // divide by 512 is approx = half-seconds
    Serial.println(F(">"));
  }
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~Fuction: Process the string message from the PC~~~~~~~~~~~~~~~~~~~~

void processMessageFromPC()
{

  // this illustrates using different inputs to call different functions
  // strcmp compares two strings and returns zero if the strings are equal

  if (strcmp(messageFromPC, "LED") == 0)
  {
    blinkLED();
  }

  if (strcmp(messageFromPC, "BUZZ") == 0)
  {
    playTone();
  }

  if (strcmp(messageFromPC, "BUZZ") == 0)
  {
    playTone();
  }
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
