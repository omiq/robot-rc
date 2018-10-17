// Multi Channel Remote using Interrupts
// -Heavily- Based on code from rcarduino.blogspot.com

// include the pinchangeint library from RCArduino
#include <PinChangeInt.h>

// Arduino Servo library for output
#include <Servo.h>

// Assign your channels to pins
// I am using Arduino Mega
// check the site listed above for Uno etc
#define AUX_IN_PIN 10
#define THROTTLE_IN_PIN 11
#define STEERING_IN_PIN 12

// Assign your channel out pins
#define THROTTLE_OUT_PIN 5
#define AUX_OUT_PIN 7
#define STEERING_OUT_PIN 9

// Servo objects for output
Servo servoThrottle;
Servo servoSteering;
Servo servoAux;

// Bit flags for channel changes
#define THROTTLE_FLAG 1
#define STEERING_FLAG 2
#define AUX_FLAG 4

// Update flags defined above
volatile uint8_t bUpdateFlagsShared;

// shared variables updated by the ISR (Interrupt Service Routine)
volatile uint16_t unThrottleInShared;
volatile uint16_t unSteeringInShared;
volatile uint16_t unAuxInShared;

// Used to record the rising edge of a pulse
uint32_t ulThrottleStart;
uint32_t ulSteeringStart;
uint32_t ulAuxStart;

void setup()
{
  // Start serial output
  Serial.begin(115200);
  
  // Attach servo objects to pins  
  servoThrottle.attach(THROTTLE_OUT_PIN);
  servoSteering.attach(STEERING_OUT_PIN);
  servoAux.attach(AUX_OUT_PIN);

  // using the PinChangeInt library, attach the interrupts
  // used to read the channels
  PCintPort::attachInterrupt(THROTTLE_IN_PIN, calcThrottle,CHANGE); 
  PCintPort::attachInterrupt(STEERING_IN_PIN, calcSteering,CHANGE); 
  PCintPort::attachInterrupt(AUX_IN_PIN, calcAux,CHANGE); 
}

void loop()
{
  // Local copies of the channel inputs
  static uint16_t unThrottleIn;
  static uint16_t unSteeringIn;
  static uint16_t unAuxIn;
  
  // Local copy of update flags
  static uint8_t bUpdateFlags;

  // Check shared update flags
  if(bUpdateFlagsShared)
  {
    // turn interrupts off temporarily
    noInterrupts(); 

    // take a local copy of which channels were updated in case we need to use this in the rest of loop
    bUpdateFlags = bUpdateFlagsShared;
    
    // only copy when the flags tell us we can.    
    if(bUpdateFlags & THROTTLE_FLAG)
    {
      unThrottleIn = unThrottleInShared;
    }
    
    if(bUpdateFlags & STEERING_FLAG)
    {
      unSteeringIn = unSteeringInShared;
    }
    
    if(bUpdateFlags & AUX_FLAG)
    {
      unAuxIn = unAuxInShared;
    }
     
    // clear shared copy of updated flags
    bUpdateFlagsShared = 0;
    
    // turn interrupts back on
    interrupts(); 

  }
  
  // Do processing here
  if(bUpdateFlags & THROTTLE_FLAG)
  {
    if(servoThrottle.readMicroseconds() != unThrottleIn)
    {
      servoThrottle.writeMicroseconds(unThrottleIn);
      Serial.println("Throttle: " + String(unThrottleIn));
    }
  }
  
  if(bUpdateFlags & STEERING_FLAG)
  {
    if(servoSteering.readMicroseconds() != unSteeringIn)
    {
      int steer = map(unSteeringIn, 1052,1876,0,180);
      servoSteering.write(steer);
      Serial.println("Steer: " + String(unSteeringIn));
    }
  }
  
  if(bUpdateFlags & AUX_FLAG)
  {
    if(servoAux.readMicroseconds() != unAuxIn)
    {
      servoAux.writeMicroseconds(unAuxIn);
      Serial.println("Aux: " + String(unAuxIn));
    }
  }
  
  // Clear flags
  bUpdateFlags = 0;
}


// simple interrupt service routine
void calcThrottle()
{
  // if the pin is high, its a rising edge of the signal pulse, so lets record its value
  if(digitalRead(THROTTLE_IN_PIN) == HIGH)
  { 
    ulThrottleStart = micros();
  }
  else
  {
    // else it must be a falling edge, so lets get the time and subtract the time of the rising edge
    // this gives use the time between the rising and falling edges i.e. the pulse duration.
    unThrottleInShared = (uint16_t)(micros() - ulThrottleStart);
    // use set the throttle flag to indicate that a new throttle signal has been received
    bUpdateFlagsShared |= THROTTLE_FLAG;
  }
}

void calcSteering()
{
  if(digitalRead(STEERING_IN_PIN) == HIGH)
  { 
    ulSteeringStart = micros();
  }
  else
  {
    unSteeringInShared = (uint16_t)(micros() - ulSteeringStart);
    bUpdateFlagsShared |= STEERING_FLAG;
  }
}

void calcAux()
{
  if(digitalRead(AUX_IN_PIN) == HIGH)
  { 
    ulAuxStart = micros();
  }
  else
  {
    unAuxInShared = (uint16_t)(micros() - ulAuxStart);
    bUpdateFlagsShared |= AUX_FLAG;
  }
}
