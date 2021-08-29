// MPED DIY: Basic Multifunction Motor Decoder
//
// Author: Marko Pinteric 2019-03-30.
// Based on the work by Alex Shepherd.
// 
// This example requires these Arduino Libraries:
//
// 1) The NmraDcc Library from: http://mrrwa.org/download/
//
// These libraries can be found and installed via the Arduino IDE Library Manager
//
// This is a simple sketch for controlling motor speed and direction according to NMRA recommendations using PWM and a DRV8870 type H-bridge.
// It uses the values vStart vHigh, Acc and Dec CV, to regulate the PWM values to the motor, and the value Timout CV, to stop the motor when the signal is lost.
// It also uses the Headling Function to drive two LEDs for Directional Headlights.
// Other than that, there's nothing fancy like Lighting Effects or a function matrix or Speed Tables - it's just the basics...
//

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

// This is the default DCC Address
#define DEFAULT_DECODER_ADDRESS 3

// This section defines the Arduino UNO Pins to use 
#ifdef __AVR_ATmega328P__ 

#define DCC_PIN     2

#define LED_PIN_FWD 9
#define LED_PIN_REV 10
#define MOTOR_PIN_FWD 5
#define MOTOR_PIN_REV 6

// This section defines the Arduino ATTiny85 Pins to use 
#elif ARDUINO_AVR_ATTINYX5 

#define DCC_PIN     2

#define LED_PIN_FWD 0
#define LED_PIN_REV 1
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
int16_t currentPwm = 0;
int16_t targetPwm = 0;
uint16_t periodAcc;                // time in ms between two PWM changes when accelerating 
uint16_t periodDec;                // time in ms between two PWM changes when decelerating 
uint32_t timePwm = 0;              // time in ms for next PWM change
bool changeLight = false;          // change FWD/REV light
uint32_t timeSignal = 0xFFFFFFFF;  // time in ms to expect DCC Signal (speed packet), 0xFFFFFFFF means no Signal

uint8_t vStart;
uint8_t Dec;
uint8_t Acc;
uint8_t vHigh;
uint32_t Timeout;       // large number for calculation purposes

// Structure for CV Values Table
struct CVPair
{
  uint16_t  CV;
  uint8_t   Value;
};

// CV Addresses we will be using
#define CV_VSTART  2
#define CV_ACC     3
#define CV_DEC     4
#define CV_VHIGH   5
#define CV_MAN_ID  8
#define CV_TIMEOUT 11
#define CV_RESET   120  // master reset

// Default CV Values Table
CVPair FactoryDefaultCVs [] =
{
	// The CV Below defines the Short DCC Address
  {CV_MULTIFUNCTION_PRIMARY_ADDRESS, DEFAULT_DECODER_ADDRESS},

  // Three Step Speed Table
  {CV_VSTART, 9},
  {CV_ACC, 5},
  {CV_DEC, 2},
  {CV_VHIGH, 255},
  {CV_MAN_ID, 13},
  {CV_TIMEOUT, 5},
  {CV_RESET, 0},

  // These two CVs define the Long DCC Address
  {CV_MULTIFUNCTION_EXTENDED_ADDRESS_MSB, 0},
  {CV_MULTIFUNCTION_EXTENDED_ADDRESS_LSB, DEFAULT_DECODER_ADDRESS},

// ONLY uncomment 1 CV_29_CONFIG line below as approprate
//  {CV_29_CONFIG,                                      0}, // Short Address 14 Speed Steps
  {CV_29_CONFIG,                       CV29_F0_LOCATION}, // Short Address 28/128 Speed Steps
//  {CV_29_CONFIG, CV29_EXT_ADDRESSING | CV29_F0_LOCATION}, // Long  Address 28/128 Speed Steps  
};

NmraDcc  Dcc ;

uint8_t FactoryDefaultCVIndex = 0;

// This call-back function is called when a CV Value changes so we can update CVs we're using
void notifyCVChange( uint16_t CV, uint8_t Value)
{
  switch(CV)
  {
    case CV_VSTART:
      vStart = Value;
      break;
      
    case CV_ACC:
      Acc = Value;
      break;

    case CV_DEC:
      Dec = Value;
      break;

    case CV_VHIGH:
      vHigh = Value;
      break;

    case CV_TIMEOUT:
      Timeout = Value;
      break;

    timeSignal = 0xFFFFFFFF;
  }
}

void notifyCVResetFactoryDefault()
{
  // Make FactoryDefaultCVIndex non-zero and equal to num CV's to be reset 
  // to flag to the loop() function that a reset to Factory Defaults needs to be done
  FactoryDefaultCVIndex = sizeof(FactoryDefaultCVs)/sizeof(CVPair);
};

// This call-back function is called whenever we receive a DCC Speed packet for our address 
void notifyDccSpeed( uint16_t Addr, DCC_ADDR_TYPE AddrType, uint8_t newSpeed, DCC_DIRECTION newDirection, DCC_SPEED_STEPS numSpeedSteps )
{
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

  if (Timeout != 0) timeSignal = millis() + 1000 * Timeout;

  if((lastSpeed != newSpeed) || (lastDirection != newDirection))
  {
    // Calculate target PWM value in the range -255 ... 255   
    uint8_t vScaleFactor;

    if (newSpeed == 0)
    {
      // changing direction at standstill
      if (currentPwm == 0) changeLight = true;

      // emergency stop
      currentPwm = 0;
      targetPwm = 0;

      #ifdef DEBUG_DEACCELERATION
      Serial.print(" currentPwm: ");
      Serial.println(currentPwm,DEC);
      #endif

      digitalWrite(MOTOR_PIN_REV, LOW);
      digitalWrite(MOTOR_PIN_FWD, LOW);
    }
    else if (newSpeed == 1)
    {
      // changing direction at standstill
      if (currentPwm == 0) changeLight = true;

      targetPwm = 0;
    }
    else
    {
      if((vHigh > 1) && (vHigh > vStart))
        vScaleFactor = vHigh - vStart;
      else
        vScaleFactor = 255 - vStart;

      uint8_t modSpeed = newSpeed - 1;
      uint8_t modSteps = numSpeedSteps - 1;

      targetPwm = (int16_t) vStart + modSpeed * vScaleFactor / modSteps;
      if (newDirection == 0) targetPwm = -targetPwm;

      periodAcc = (uint16_t) Acc*896/vScaleFactor;
      periodDec = (uint16_t) Dec*896/vScaleFactor;

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
      Serial.print(" newPwm: ");
      Serial.println(targetPwm);
      #endif

    }
    // change immediately
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
  Serial.println("NMRA Dcc Multifunction Motor Decoder Demo");
  #endif

  // Setup the Pins for the Fwd/Rev LED for Function 0 Headlight
  pinMode(LED_PIN_FWD, OUTPUT);
  pinMode(LED_PIN_REV, OUTPUT);

  // Setup the Pins for the Motor H-Bridge Driver
  pinMode(MOTOR_PIN_FWD, OUTPUT);
  pinMode(MOTOR_PIN_REV, OUTPUT);

  
  // Setup which External Interrupt, the Pin it's associated with that we're using and enable the Pull-Up 
  Dcc.pin(DCC_PIN, 0);
  
  Dcc.init( MAN_ID_DIY, 10, FLAGS_MY_ADDRESS_ONLY | FLAGS_AUTO_FACTORY_DEFAULT, 0 );

  // Uncomment to force CV Reset to Factory Defaults
//  notifyCVResetFactoryDefault();

  // master reset
  if(Dcc.getCV(CV_RESET) == CV_RESET) notifyCVResetFactoryDefault();
  
  // Read the current CV values
  vStart = Dcc.getCV(CV_VSTART);
  Acc = Dcc.getCV(CV_ACC);
  Dec = Dcc.getCV(CV_DEC);
  vHigh = Dcc.getCV(CV_VHIGH);
  Timeout = Dcc.getCV(CV_TIMEOUT);
}

void loop()
{
  // You MUST call the NmraDcc.process() method frequently from the Arduino loop() function for correct library operation
  Dcc.process();

  // Handle lost Signal
  if (millis() > timeSignal)
  {
  // emergency stop
  currentPwm = 0;
  targetPwm = 0;
  lastSpeed = 0;

  #ifdef DEBUG_DEACCELERATION
  Serial.print(" currentPwm: ");
  Serial.println(currentPwm,DEC);
  #endif

  digitalWrite(MOTOR_PIN_REV, LOW);
  digitalWrite(MOTOR_PIN_FWD, LOW);

  timeSignal = 0xFFFFFFFF;  
  }

  // Handle Speed changes
  if((currentPwm != targetPwm) && (millis() >= timePwm))
  {
    // by default acclerating
    timePwm = millis() + periodAcc;
    if (targetPwm > currentPwm)
    {
      if(currentPwm == -vStart) currentPwm=0;
      else if (currentPwm == 0)
      {
        currentPwm = vStart;
        // changing direction at standstill
        changeLight = true;
      }
      else
      {
        currentPwm++;
        // actually decelerating
        if (currentPwm < 0) timePwm = millis() + periodDec;
      }
    }
    else // targetPwm < currentPwm
    {
      if(currentPwm == vStart) currentPwm=0;
      else if (currentPwm == 0)
      {
        currentPwm = -vStart;
        // changing direction at standstill
        changeLight = true;
      }
      else
      {
        currentPwm--;
        // actually decelerating
        if (currentPwm > 0) timePwm = millis() + periodDec;
      }
    }
    #ifdef DEBUG_DEACCELERATION
    Serial.print(" currentPwm: ");
    Serial.println(currentPwm,DEC);
    #endif

    if (currentPwm > 0)
    {
      digitalWrite(MOTOR_PIN_REV, LOW);
      analogWrite(MOTOR_PIN_FWD, currentPwm);
    }
    else if (currentPwm < 0)
    {
      digitalWrite(MOTOR_PIN_FWD, LOW);
      analogWrite(MOTOR_PIN_REV, -currentPwm);    
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
      if(currentPwm == 0)
      {
        digitalWrite(LED_PIN_FWD, lastDirection ? HIGH : LOW);
        digitalWrite(LED_PIN_REV, lastDirection ? LOW : HIGH);
      }
      else if(currentPwm > 0)
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
