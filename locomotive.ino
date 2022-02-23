// MPED DIY: Basic Multifunction Locomotive Decoder
//
// Version: 1.0
// Author: Marko Pinteric 2021-08-30.
// Based on the work by Alex Shepherd.
// 
// This sketch requires the NmraDcc Library, which can be found and installed via the Arduino IDE Library Manager.
//
// This is a simple sketch for controlling motor speed and direction according to NMRA recommendations using PWM and a DRV8870 type H-bridge.
// It uses the values vStart vHigh, Acc and Dec CV, to regulate the PWM values to the motor, and the value Timout CV, to stop the motor when the signal is lost.
// It also uses the Headling Function to drive two LEDs for Directional Headlights.
// Other than that, there's nothing fancy like Lighting Effects or a function matrix or Speed Tables.

#include <NmraDcc.h>

// Uncomment any of the lines below to enable debug messages for different parts of the code
//#define DEBUG_FUNCTIONS
//#define DEBUG_SPEED
//#define DEBUG_PWM
//#define DEBUG_DCC_ACK
//#define DEBUG_DCC_MSG
//#define DEBUG_DEACCELERATION
//#define DEBUG_PRINT

#if defined(DEBUG_FUNCTIONS) or defined(DEBUG_SPEED) or defined(DEBUG_PWM) or defined(DEBUG_DCC_ACK) or defined(DEBUG_DCC_MSG) or defined(DEBUG_DEACCELERATION)
#define DEBUG_PRINT
#endif

// Actual number of speed steps in one direction 9 * 28 = 2 * 126
#define SPEED_STEPS 252

// This is the default DCC Address
#define DEFAULT_DECODER_ADDRESS 3

// This section defines the Arduino UNO Pins to use 
#if defined(__AVR_ATmega328__) or defined(__AVR_ATmega328P__)

#define DCC_PIN       2

#define LED_PIN_FWD   9
#define LED_PIN_REV  10
#define MOTOR_PIN_FWD 5
#define MOTOR_PIN_REV 6

// This section defines the Arduino ATTiny85 Pins to use 
#elif defined(ARDUINO_AVR_ATTINYX5) 

#define DCC_PIN       2

#define LED_PIN_FWD   0
#define LED_PIN_REV   1
#define MOTOR_PIN_FWD 3
#define MOTOR_PIN_REV 4

#else
#error "Unsupported CPU, you need to add another configuration section for your CPU"
#endif 

// Some global state variables
uint8_t newLedState;
uint8_t lastLedState = 0;
uint8_t lastDirection = 1;
uint8_t newSpeed;
uint8_t lastSpeed = 0;
uint8_t numSpeedSteps = SPEED_STEP_128;
int16_t currentStep = 0;
int16_t targetStep = 0;
uint8_t absStep;
uint8_t tableElement;
uint8_t motorPwm;
uint16_t periodAcc;                // time in ms between two PWM changes when accelerating 
uint16_t periodDec;                // time in ms between two PWM changes when decelerating 
uint32_t timePwm = 0;              // time in ms for next PWM change
bool changeLight = false;          // change FWD/REV light
bool speedTableSelect = false;     // use speed table
uint32_t timeSignal = 0xFFFFFFFF;  // time in ms to expect DCC Signal (speed packet), 0xFFFFFFFF means no Signal

uint8_t vStart;
uint8_t vHigh;
uint8_t vMid;
uint32_t Timeout;                   // large number for calculation purposes
uint8_t speedTable[29];

// Structure for CV Values Table
struct CVPair
{
  uint16_t  CV;
  uint8_t   Value;
};

// CV Addresses we will be using
#define CV_VSTART               2
#define CV_ACCELERATION_RATE    3
#define CV_DECELERATION_RATE    4
#define CV_VHIGH                5
#define CV_VMID                 6
#define CV_SIGNAL_TIMEOUT       11
#define CV_SPEED_TABLE          67
// #define CV_DECODER_MASTER_RESET 120

// Default CV Values Table
CVPair FactoryDefaultCVs [] =
{
  {CV_MULTIFUNCTION_PRIMARY_ADDRESS, DEFAULT_DECODER_ADDRESS},
  {CV_MULTIFUNCTION_EXTENDED_ADDRESS_MSB, 0},
  {CV_MULTIFUNCTION_EXTENDED_ADDRESS_LSB, 0},

  {CV_VSTART, 3},                   // Zimo=1 Lais=0(9)
  {CV_ACCELERATION_RATE, 5},        // Zimo=2 Lais=1(3)
  {CV_DECELERATION_RATE, 2},        // Zimo=1 Lais=1(3)
  {CV_VHIGH, 0},                    // Zimo=1 Lais=0(0)
  {CV_VMID, 0},                     // Zimo=1 Lais=0(-)
  {CV_VERSION_ID, 1},
  {CV_MANUFACTURER_ID, MAN_ID_DIY},
  {CV_SIGNAL_TIMEOUT, 5},

// ONLY uncomment one CV_29_CONFIG line below as approprate
//  {CV_29_CONFIG,                                      0}, // Short Address 14 Speed Steps
  {CV_29_CONFIG,                       CV29_F0_LOCATION}, // Short Address 28/128 Speed Steps
//  {CV_29_CONFIG, CV29_EXT_ADDRESSING | CV29_F0_LOCATION}, // Long  Address 28/128 Speed Steps  

  {CV_SPEED_TABLE, 12},
  {CV_SPEED_TABLE+1, 21},
  {CV_SPEED_TABLE+2, 30},
  {CV_SPEED_TABLE+3, 39},
  {CV_SPEED_TABLE+4, 48},
  {CV_SPEED_TABLE+5, 57},
  {CV_SPEED_TABLE+6, 66},
  {CV_SPEED_TABLE+7, 75},
  {CV_SPEED_TABLE+8, 84},
  {CV_SPEED_TABLE+9, 93},
  {CV_SPEED_TABLE+10, 102},
  {CV_SPEED_TABLE+11, 111},
  {CV_SPEED_TABLE+12, 120},
  {CV_SPEED_TABLE+13, 129},
  {CV_SPEED_TABLE+14, 138},
  {CV_SPEED_TABLE+15, 147},
  {CV_SPEED_TABLE+16, 156},
  {CV_SPEED_TABLE+17, 165},
  {CV_SPEED_TABLE+18, 174},
  {CV_SPEED_TABLE+19, 183},
  {CV_SPEED_TABLE+20, 192},
  {CV_SPEED_TABLE+21, 201},
  {CV_SPEED_TABLE+22, 210},
  {CV_SPEED_TABLE+23, 219},
  {CV_SPEED_TABLE+24, 228},
  {CV_SPEED_TABLE+25, 237},
  {CV_SPEED_TABLE+26, 246},
  {CV_SPEED_TABLE+27, 255},
};

NmraDcc  Dcc ;

// This call-back function is called when a CV Value changes so we can update CVs we're using
void notifyCVChange( uint16_t CV, uint8_t Value)
{
  switch(CV)
  {
    case CV_VSTART:
      vStart = Value;
      speedTable[0] = Value;
      break;
      
    case CV_ACCELERATION_RATE:
      periodAcc = (uint16_t) Value*896/SPEED_STEPS;

      break;

    case CV_DECELERATION_RATE:
      periodDec = (uint16_t) Value*896/SPEED_STEPS;
      break;

    case CV_VHIGH:
      vHigh = Value;
      break;

    case CV_VMID:
      vMid = Value;
      break;

    case CV_29_CONFIG:
      if((Value & 0b00010000) == 0) speedTableSelect = false;
      else speedTableSelect = true;

    case CV_SIGNAL_TIMEOUT:
      Timeout = Value;
      timeSignal = 0xFFFFFFFF;
      break;
  }
  if ((CV>=67) and (CV<=94)) speedTable[CV-66] = Value;
}

uint8_t FactoryDefaultCVIndex = 0;
void notifyCVResetFactoryDefault()
{
  // Make FactoryDefaultCVIndex non-zero and equal to num CV's to be reset 
  // to flag to the loop() function that a reset to Factory Defaults needs to be done
  if (FactoryDefaultCVIndex == 0) FactoryDefaultCVIndex = sizeof(FactoryDefaultCVs)/sizeof(CVPair);
};

// This call-back function is called whenever we receive a DCC Speed packet for our address 
void notifyDccSpeed( uint16_t Addr, DCC_ADDR_TYPE AddrType, uint8_t newSpeed, DCC_DIRECTION newDirection, DCC_SPEED_STEPS numSpeedSteps )
{
  if (Timeout != 0) timeSignal = millis() + 1000 * Timeout;

  #ifdef DEBUG_SPEED
  Serial.print("notifyDccSpeed: Addr: ");
  Serial.print(Addr,DEC);
  Serial.print( (AddrType == DCC_ADDR_SHORT) ? "-S" : "-L" );
  Serial.print(" Speed: ");
  Serial.print(newSpeed,DEC);
  Serial.print(" Steps: ");
  Serial.print(numSpeedSteps,DEC);
  Serial.print(" Dir: ");
  Serial.println( (newDirection == DCC_DIR_FWD) ? "Forward" : "Reverse" );
  #endif

  if((lastSpeed != newSpeed) || (lastDirection != newDirection))
  {
    uint8_t vScaleFactor;

    if (newSpeed == 0)
    {
      // changing direction at standstill
      if (currentStep == 0) changeLight = true;

      // emergency stop
      currentStep = 0;
      targetStep = 0;

      #ifdef DEBUG_DEACCELERATION
      Serial.print(" motorPwm: ");
      Serial.println(0,DEC);
      #endif

      digitalWrite(MOTOR_PIN_REV, LOW);
      digitalWrite(MOTOR_PIN_FWD, LOW);
    }
    else if (newSpeed == 1)
    {
      // changing direction at standstill
      if (currentStep == 0) changeLight = true;

      targetStep = 0;
    }
    else
    {
      targetStep=(newSpeed-1)*SPEED_STEPS/(numSpeedSteps-1);
      if (newDirection == 0) targetStep = -targetStep;

      #ifdef DEBUG_PWM
      Serial.print("New Speed: vStart: ");
      Serial.print(vStart);
      Serial.print(" vHigh: ");
      Serial.print(vHigh);
      Serial.print(" modSpeed: ");
      Serial.print(modSpeed);
      Serial.print(" vScaleFactor: ");
      Serial.print(vScaleFactor);
      Serial.print(" modSteps: ");
      Serial.print(modSteps);
      Serial.print(" targetStep: ");
      Serial.println(targetStep);
      #endif

    }
    // change PWM immediately
    timePwm = millis();

    lastSpeed = newSpeed;
    lastDirection = newDirection;
  }
};

// This call-back function is called whenever we receive a DCC Function packet for our address 
void notifyDccFunc(uint16_t Addr, DCC_ADDR_TYPE AddrType, FN_GROUP FuncGrp, uint8_t FuncState)
{
  #ifdef DEBUG_FUNCTIONS
  Serial.print("notifyDccFunc: Addr: ");
  Serial.print(Addr,DEC);
  Serial.print( (AddrType == DCC_ADDR_SHORT) ? 'S' : 'L' );
  Serial.print("  Function Group: ");
  Serial.print(FuncGrp,DEC);
  #endif

  if(FuncGrp == FN_0_4)
  {
    newLedState = (FuncState & FN_BIT_00) ? 1 : 0;
    #ifdef DEBUG_FUNCTIONS
    Serial.print(" FN 0: ");
    Serial.print(newLedState);
    #endif
  }
  #ifdef DEBUG_FUNCTIONS
  Serial.println();
  #endif
}

// This call-back function is called whenever we receive a DCC Packet
#ifdef  DEBUG_DCC_MSG
void notifyDccMsg( DCC_MSG * Msg)
{
  Serial.print("notifyDccMsg: ") ;
  for(uint8_t i = 0; i < Msg->Size; i++)
  {
    Serial.print(Msg->Data[i], HEX);
    Serial.write(' ');
  }
  Serial.println();
}
#endif

// This call-back function is called by the NmraDcc library when a DCC ACK needs to be sent
// Calling this function should cause an increased 60ma current drain on the power supply for 6ms to ACK a CV Read
// So we will just turn the motor on for 8ms and then turn it off again.
void notifyCVAck(void)
{
  #ifdef DEBUG_DCC_ACK
  Serial.println("notifyCVAck") ;
  #endif

  digitalWrite(MOTOR_PIN_FWD, HIGH);

  delay( 8 );  

  digitalWrite(MOTOR_PIN_FWD, LOW);
}

void setup()
{
  #ifdef DEBUG_PRINT
  Serial.begin(115200);
  Serial.println("NMRA Dcc Multifunction Motor Decoder");
  #endif

  // Setup the Pins for the Fwd/Rev LED for Function 0 Headlight
  pinMode(LED_PIN_FWD, OUTPUT);
  pinMode(LED_PIN_REV, OUTPUT);

  // Setup the Pins for the Motor H-Bridge Driver
  pinMode(MOTOR_PIN_FWD, OUTPUT);
  pinMode(MOTOR_PIN_REV, OUTPUT);

  // Setup which External Interrupt, the Pin it's associated with that we're using and enable the Pull-Up 
  Dcc.pin(DCC_PIN, 0);

  // Call the main DCC Init function to enable the DCC Receiver
  //!!!!!!!!!!!!!!!!!! FLAGS_AUTO_FACTORY_DEFAULT: Call notifyCVResetFactoryDefault() if CV 7 & 8 == 255
  Dcc.init( MAN_ID_DIY, 1, FLAGS_MY_ADDRESS_ONLY | FLAGS_AUTO_FACTORY_DEFAULT, 0 );

  // trigger decoder master reset
//  if(Dcc.getCV(CV_DECODER_MASTER_RESET) == CV_DECODER_MASTER_RESET) notifyCVResetFactoryDefault();
  
  // Read the current CV values
  vStart = Dcc.getCV(CV_VSTART);
  speedTable[0] = Dcc.getCV(CV_VSTART);
  periodAcc = (uint16_t) Dcc.getCV(CV_ACCELERATION_RATE)*896/SPEED_STEPS;
  periodDec = (uint16_t) Dcc.getCV(CV_DECELERATION_RATE)*896/SPEED_STEPS;
  vHigh = Dcc.getCV(CV_VHIGH);
  vMid = Dcc.getCV(CV_VMID);
  Timeout = Dcc.getCV(CV_SIGNAL_TIMEOUT);
  if((Dcc.getCV(CV_29_CONFIG) & 0b00010000) == 0) speedTableSelect = false;
  else speedTableSelect = true;

  for(uint8_t i = 0; i < 28; i++) speedTable[i+1] = Dcc.getCV(CV_SPEED_TABLE + i);
}

void loop()
{
  // You MUST call the NmraDcc.process() method frequently from the Arduino loop() function for correct library operation
  Dcc.process();

  // Handle lost Signal
  if (millis() > timeSignal)
  {
  // emergency stop
  currentStep = 0;
  targetStep = 0;
  lastSpeed = 0;

  #ifdef DEBUG_DEACCELERATION
  Serial.print(" motorPwm: ");
  Serial.println(0,DEC);
  #endif

  digitalWrite(MOTOR_PIN_REV, LOW);
  digitalWrite(MOTOR_PIN_FWD, LOW);

  timeSignal = 0xFFFFFFFF;  
  }

  // Handle Speed changes
  if((currentStep != targetStep) && (millis() >= timePwm))
  {
    if (targetStep > currentStep)
    {
      currentStep++;
      if (currentStep > 0)
      {
        timePwm = millis() + periodAcc;
      }
      else
      {
        timePwm = millis() + periodDec;
      }
    }
    else // targetStep < currentStep
    {
      currentStep--;
      if (currentStep < 0)
      {
        timePwm = millis() + periodAcc;
      }
      else
      {
        timePwm = millis() + periodDec;
      }
    }

    absStep = abs(currentStep);
    if(absStep == 0)
    {
      motorPwm = 0;
      changeLight = true;
    }
    else if(speedTableSelect)
    {
      tableElement = trunc(currentStep/9);
      motorPwm = (uint8_t) speedTable[tableElement] + (absStep - 9 * tableElement) * (speedTable[tableElement+1] - speedTable[tableElement]) / 9;
    }
    else if ((vMid > 0) && (vMid > vStart) && (vHigh > vMid))
    {
      if (absStep < SPEED_STEPS/2)
      {
        motorPwm = (uint8_t) vStart + (uint16_t) (vMid - vStart) * absStep/(SPEED_STEPS/2);
      }
      else
      {
        motorPwm = (uint8_t) vMid + (uint16_t) (vHigh - vMid) * (absStep-SPEED_STEPS/2)/(SPEED_STEPS/2);
      }
    }
    else if ((vHigh > 0) && (vHigh > vStart))
    {
      motorPwm = (uint8_t) vStart + (uint16_t) (vHigh - vStart) * absStep/SPEED_STEPS;
    }
    else
    {
      motorPwm = (uint8_t) vStart + (255 - vStart) * absStep/SPEED_STEPS;
    }

    #ifdef DEBUG_DEACCELERATION
    Serial.print(" vMid: ");
    Serial.print(vMid,DEC);
    Serial.print(" vHigh: ");
    Serial.print(vHigh,DEC);
    Serial.print(" motorPwm: ");
    Serial.print(motorPwm,DEC);
    Serial.print(" currentStep: ");
    Serial.println(currentStep,DEC);
    #endif

    if (currentStep > 0)
    {
      digitalWrite(MOTOR_PIN_REV, LOW);
      analogWrite(MOTOR_PIN_FWD, motorPwm);
    }
    else if (currentStep < 0)
    {
      digitalWrite(MOTOR_PIN_FWD, LOW);
      analogWrite(MOTOR_PIN_REV, motorPwm);    
    }
    else
    {
      digitalWrite(MOTOR_PIN_REV, LOW);
      digitalWrite(MOTOR_PIN_FWD, LOW);
    }
  }

  // Handle Direction and Headlight changes
  if((changeLight) || (lastLedState != newLedState))
  {
    if(newLedState)
    {
      #ifdef DEBUG_FUNCTIONS
      Serial.println("LED On");
      #endif
      if(currentStep == 0)
      {
        digitalWrite(LED_PIN_FWD, lastDirection ? HIGH : LOW);
        digitalWrite(LED_PIN_REV, lastDirection ? LOW : HIGH);
      }
      else if(currentStep > 0)
      {
        digitalWrite(LED_PIN_FWD, HIGH);
        digitalWrite(LED_PIN_REV, LOW);
      }
      else
      {
        digitalWrite(LED_PIN_FWD, LOW);
        digitalWrite(LED_PIN_REV, HIGH);
      }
    }
    else
    {
      #ifdef DEBUG_FUNCTIONS
      Serial.println("LED Off");
      #endif
      digitalWrite(LED_PIN_FWD, LOW);
      digitalWrite(LED_PIN_REV, LOW);
    }

    changeLight = false;
    lastLedState = newLedState;
  }

  // Handle resetting CVs back to Factory Defaults
  if( FactoryDefaultCVIndex && Dcc.isSetCVReady())
  {
    FactoryDefaultCVIndex--; // Decrement first as initially it is the size of the array 
    Dcc.setCV( FactoryDefaultCVs[FactoryDefaultCVIndex].CV, FactoryDefaultCVs[FactoryDefaultCVIndex].Value);
  }
}
