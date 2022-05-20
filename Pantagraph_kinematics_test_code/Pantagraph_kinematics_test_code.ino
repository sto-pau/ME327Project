//--------------------------------------------------------------------------
// Code to test basic Hapkit functionality (sensing and force output)
// Updated by Allison Okamura 1.17.2018
//--------------------------------------------------------------------------

// Includes
#include <math.h>
#include <AS5048A.h> 
// the sensor CSn pin is connected to pin 10 
AS5048A angleSensor(10); 


// Pin declares
int pwmPinR = 5; // PWM output pin for motor 
int dirPinR = 8; // direction output pin for motor 
int pwmPinL = 6; // PWM output pin for motor 
int dirPinL = 7; // direction output pin for motor 
int sensorPosPin = A4; // input pin for MR sensor
int fsrPin = A3; // input pin for FSR sensor

// Position tracking variables
int updatedPos = 0;     // keeps track of the latest updated value of the MR sensor reading
int rawPos = 0;         // current raw reading from MR sensor
int lastRawPos = 0;     // last raw reading from MR sensor
int lastLastRawPos = 0; // last last raw reading from MR sensor
int flipNumber = 0;     // keeps track of the number of flips over the 180deg mark
int tempOffset = 0;
int rawDiff = 0;
int lastRawDiff = 0;
int rawOffset = 0;
int lastRawOffset = 0;
const int flipThresh = 700;  // threshold to determine whether or not a flip over the 180 degree mark occurred
boolean flipped = false;
double OFFSET = 980;
double OFFSET_NEG = 15;

// Position tracking variables Hall effect sensor
long updatedPosHE = 0; // latest processed value of the Hall Effect sensor
int rawPosHE = 0;         // current raw reading from MR sensor
int lastRawPosHE = 0;     // last raw reading from MR sensor
int lastLastRawPosHE = 0; // last last raw reading from MR sensor
int flipNumberHE = 0;     // keeps track of the number of flips over the 180deg mark
int tempOffsetHE = 0;
int rawDiffHE = 0;
int lastRawDiffHE = 0;
int rawOffsetHE = 0;
int lastRawOffsetHE = 0;
const int flipThreshHE = 16000;  // threshold to determine whether or not a flip over the 180 degree mark occurred
boolean flippedHE = false;
double OFFSETHE = 16000;
double OFFSET_NEGHE = 15;

// Kinematics variables
double xh = 0;           // position of the handle [m]

// Force output variables
double force = 0;           // force at the handle
double Tp = 0;              // torque of the motor pulley
double duty = 0;            // duty cylce (between 0 and 255)
unsigned int output = 0;    // output command to the motor


// --------------------------------------------------------------
// Setup function -- NO NEED TO EDIT
// --------------------------------------------------------------
void setup() 
{

  angleSensor.init(); 
  
  // Set up serial communication
  Serial.begin(115200);
  
  // Set PWM frequency 
  setPwmFrequency(pwmPinR,1); 
  
  // Input pins
  pinMode(sensorPosPin, INPUT); // set MR sensor pin to be an input
  pinMode(fsrPin, INPUT);       // set FSR sensor pin to be an input

  // Output pins
  pinMode(pwmPinR, OUTPUT);  // PWM pin for motor 
  pinMode(dirPinR, OUTPUT);  // dir pin for motor 

    // Output pins
  pinMode(pwmPinL, OUTPUT);  // PWM pin for motor 
  pinMode(dirPinL, OUTPUT);  // dir pin for motor 
  
  // Initialize motor 
  analogWrite(pwmPinR, 0);     // set to not be spinning (0/255)
  digitalWrite(dirPinR, LOW);  // set direction

  // Initialize motor 
  analogWrite(pwmPinL, 0);     // set to not be spinning (0/255)
  digitalWrite(dirPinL, LOW);  // set direction
  
  // Initialize position valiables
  lastLastRawPos = analogRead(sensorPosPin);
  lastRawPos = analogRead(sensorPosPin);
  flipNumber = 0;

  // Initialize position variables 
  angleSensor.getRawRotation(); // take reading in case first reading is spurious 
  lastLastRawPosHE = angleSensor.getRawRotation(); //position from Hall Effect sensor 
  lastRawPosHE = angleSensor.getRawRotation(); //position from Hall Effect sensor
}


// --------------------------------------------------------------
// Main Loop
// --------------------------------------------------------------
void loop()
{
  
  //*************************************************************
  //*** Section 1. Compute position in counts (do not change) ***  
  //*************************************************************

  // Get voltage output by MR sensor
  rawPos = analogRead(sensorPosPin);  //current raw position from MR sensor

  // Calculate differences between subsequent MR sensor readings
  rawDiff = rawPos - lastRawPos;          //difference btwn current raw position and last raw position
  lastRawDiff = rawPos - lastLastRawPos;  //difference btwn current raw position and last last raw position
  rawOffset = abs(rawDiff);
  lastRawOffset = abs(lastRawDiff);
  
  // Update position record-keeping vairables
  lastLastRawPos = lastRawPos;
  lastRawPos = rawPos;
  
  // Keep track of flips over 180 degrees
  if((lastRawOffset > flipThresh) && (!flipped)) { // enter this anytime the last offset is greater than the flip threshold AND it has not just flipped
    if(lastRawDiff > 0) {        // check to see which direction the drive wheel was turning
      flipNumber--;              // cw rotation 
    } else {                     // if(rawDiff < 0)
      flipNumber++;              // ccw rotation
    }
    flipped = true;            // set boolean so that the next time through the loop won't trigger a flip
  } else {                        // anytime no flip has occurred
    flipped = false;
  }
   updatedPos = rawPos + flipNumber*OFFSET; // need to update pos based on what most recent offset is 


  //*************************************************************
  //*** Section 1. Compute position in counts (do not change) ***  
  //*************************************************************
  
  // Get angle output by Hall Effect sensor 
  rawPosHE = int(angleSensor.getRawRotation()); //position from Hall Effect sensor 

    // Calculate differences between subsequent MR sensor readings
  rawDiffHE = rawPosHE - lastRawPosHE;          //difference btwn current raw position and last raw position
  lastRawDiffHE = rawPosHE - lastLastRawPosHE;  //difference btwn current raw position and last last raw position
  rawOffsetHE = abs(rawDiffHE);
  lastRawOffsetHE = abs(lastRawDiffHE);
  
  // Update position record-keeping vairables
  lastLastRawPosHE = lastRawPosHE;
  lastRawPosHE = rawPosHE;
  
  // Keep track of flips over 180 degrees
  if((lastRawOffsetHE > flipThreshHE) && (!flippedHE)) { // enter this anytime the last offset is greater than the flip threshold AND it has not just flipped
    if(lastRawDiffHE > 0) {        // check to see which direction the drive wheel was turning
      flipNumberHE--;              // cw rotation 
    } else {                     // if(rawDiff < 0)
      flipNumberHE++;              // ccw rotation
    }
    flippedHE = true;            // set boolean so that the next time through the loop won't trigger a flip
  } else {                        // anytime no flip has occurred
    flippedHE = false;
  }
   updatedPosHE = rawPosHE + flipNumberHE*OFFSETHE; // need to update pos based on what most recent offset is 

 
  //*************************************************************
  //*** Section 2. Compute position in meters *******************
  //*************************************************************

  // ADD YOUR CODE HERE
  // Define kinematic parameters you may need
     //double rh = ?;   //[m]
  // Step B.1: print updatedPos via serial monitor
  //Serial.println((float)updatedPosHE,5);
  //Serial.println((float)updatedPos,5);
  // Step B.6: double ts = ?; // Compute the angle of the sector pulley (ts) in degrees based on updatedPos
  double ts5 = (PI/180)*(updatedPos*-0.0129+155.5);
  double ts1 = (PI/180)*(updatedPosHE*0.001470+38.09);
//  Serial.print((float)ts5,5);
//  Serial.print("\t");
//  Serial.println((float)ts1,5);
  // Step B.7: xh = ?;       // Compute the position of the handle (in meters) based on ts (in radians)
  // Step B.8: print xh via serial monitor
    
  
  //*************************************************************
  //*** Section 3. Assign a motor output force in Newtons *******  
  //*************************************************************
 
  // ADD YOUR CODE HERE
  // Define kinematic parameters you may need
  
  // Step C.1: force = ?; // You can  generate a force by assigning this to a constant number (in Newtons) or use a haptic rendering / virtual environment
  // Step C.2: Tp = ?;    // Compute the require motor pulley torque (Tp) to generate that force using kinematics
 
  //*************************************************************
  //*** Section 4. Force output (do not change) *****************
  //*************************************************************
  
  // Determine correct direction for motor torque
  if(force > 0) { 
    digitalWrite(dirPinR, HIGH);
  } else {
    digitalWrite(dirPinR, LOW);
  }

  // Compute the duty cycle required to generate Tp (torque at the motor pulley)
  duty = sqrt(abs(Tp)/0.0183);

  // Make sure the duty cycle is between 0 and 100%
  if (duty > 1) {            
    duty = 1;
  } else if (duty < 0) { 
    duty = 0;
  }  
  output = (int)(duty* 255);   // convert duty cycle to output signal
  analogWrite(pwmPinR,output);  // output the signal
}

// --------------------------------------------------------------
// Function to set PWM Freq -- DO NOT EDIT
// --------------------------------------------------------------
void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}
