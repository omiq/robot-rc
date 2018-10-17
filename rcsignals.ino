// Simple example to read the signals
// from RC receiver and output to serial
// also passes through to a servo to
// show how you can intercept then control
// devices using this method

#include <Servo.h>

// Servo object
Servo myservo;  

// Channels
int ch1; 
int ch2;
int ch3;
int ch4;
int ch5;

// Set up variables
void setup() {

// Input Pins
pinMode(3, INPUT);
pinMode(4, INPUT);
pinMode(5, INPUT); 
pinMode(6, INPUT);
pinMode(7, INPUT);

// Servo on pin 9
myservo.attach(9);  

// Start serial output
Serial.begin(115200); 

}

void loop() {
  
  // Read the pulse width of
  // the channels we care about
  // X/Y and Throttle
  ch1 = pulseIn(5, HIGH, 25000);  
  ch2 = pulseIn(6, HIGH, 25000); 
  ch3 = pulseIn(7, HIGH, 25000);

  // Output the value of Ch1 to Servo
  // Mapped to convert from pulse to degrees
  myservo.write(map(ch1, 1040,1866,0,180));

  // Print the values to serial so you can plot
  Serial.println(ch1); 
  Serial.println(ch2); 
  Serial.println(ch3); 
  delay(100);
}
