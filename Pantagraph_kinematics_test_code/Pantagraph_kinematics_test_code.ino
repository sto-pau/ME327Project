//--------------------------------------------------------------------------
// Code to test basic Hapkit functionality (sensing and force output)
// Updated by Allison Okamura 1.17.2018
//--------------------------------------------------------------------------

// Includes
#include <math.h>
#include <AS5048A.h> 
// the sensor CSn pin is connected to pin 10 
AS5048A angleSensor(10); 

// generate constants
double a1 = 129; //mm
double a2 = 152; //mm
double a3 = 152; //mm
double a4 = 129; //mm
double a5 = 82; //mm

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

// Kinematics 
//variables 
double xh = 0;
double yh = 0;
double dxh = 0;
double dyh = 0;

//theta 1
double theta1 = 0;
double theta1_prev;
double theta1_prev2;
double dtheta1;
double dtheta1_prev;
double dtheta1_prev2;
// theta 5
double theta5 = 0;
double theta5_prev;
double theta5_prev2;
double dtheta5;
double dtheta5_prev;
double dtheta5_prev2;

// Force output variables
double force = 0;           // force at the handle
double forcey = 0;

double Tp = 0;              // torque of the motor pulley
double TpL = 0;              // torque of the motor pulley

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
  double theta5 = (PI/180)*(updatedPos*-0.0129+155.5);
  double theta1 = (PI/180)*(updatedPosHE*0.001470+38.09);
//  Serial.print((float)theta5,5);
//  Serial.print("\t");
//  Serial.print((float)theta1,5);
  // Step B.7: xh = ?;       // Compute the position of the handle (in meters) based on ts (in radians)
  // Step B.8: print xh via serial monitor

  // Calculate velocity with loop time estimation
  dtheta1 = (double)(theta1 - theta1_prev) / 0.001;
  // Record the position and velocity
  theta1_prev2 = theta1_prev;
  theta1_prev = theta1;
  theta1_prev2 = theta1_prev;
  theta1_prev = theta1;

  // Calculate velocity with loop time estimation
  dtheta5 = (double)(theta5 - theta5_prev) / 0.001;
  // Record the position and velocity
  theta5_prev2 = theta5_prev;
  theta5_prev = theta5;
  theta5_prev2 = theta5_prev;
  theta5_prev = theta5;

//  Serial.print("\t");
//  Serial.print((float)dtheta5,5);
//  Serial.print("\t");
//  Serial.println((float)dtheta1,5);
  
  //calculate positions 2 and 4
  double P2x = a1*cos(theta1);
  double P2y = a1*sin(theta1);
  double P4x = a4*cos(theta5) - a5;
  double P4y = a4*sin(theta5);


  
  // calculate norms
  double P42_norm = sqrt((P2x-P4x)*(P2x-P4x)+(P2y-P4y)*(P2y-P4y));
  double P2h_norm = (a2*a2 - a3*a3 + P42_norm*P42_norm)/(2*P42_norm);
  double P3h_norm = sqrt(a2*a2 - P2h_norm*P2h_norm);

  // calculate position h
  double Phx = P2x + (P2h_norm/P42_norm)*(P4x-P2x);
  double Phy = P2y + (P2h_norm/P42_norm)*(P4y-P2y);
  // calculate position 3
  double P3x = Phx + (P3h_norm/P42_norm)*(P4y-P2y);
  double P3y = Phy - (P3h_norm/P42_norm)*(P4x-P2x);

  xh = P3x;
  yh = P3y;

  //rename define for convenience
  double d = P42_norm;
  double b = P2h_norm;
  double h = P3h_norm;
  // calculate partial derivatives
  // d1 partial derivatives
  double d1x2 = -a1*sin(theta1);
  double d1y2 = a1*cos(theta1);
  double d1y4 = 0;
  double d1x4 = 0;
  double d1d = ( (P4x-P2x)*(d1x4-d1x2) + (P4y-P2y)*(d1y4-d1y2 ))/d;
  double d1b = d1d - ( d1d*(a2*a2 - a3*a3 + d*d))/(2*d*d);
  double d1h = -b*d1b/h;
  double d1yh = d1y2 + ( (d1b*d - d1d*b)/(d*d) )*(P4y-P2y)+(b/d)*(d1y4-d1y2);
  double d1xh = d1x2+( (d1b*d-d1b*d)/(d*d) )*(P4x-P2x)+(b/d)*(d1x4-d1x2);
  double d1x3 = d1xh + (h/d)*(d1y4 - d1y2) + (d1h*d - d1d*h)/(d*d)*(P4y - P2y);
  double d1y3 = d1yh + (h/d)*(d1x4 - d1x2) + (d1h*d - d1d*h)/(d*d)*(P4x - P2x);
  // d5 partial derivatives
  double d5x4 = -a4*sin(theta5);
  double d5y4 = a4*cos(theta5);
  double d5y2 = 0;
  double d5x2 = 0;
  double d5d = ( (P4x-P2x)*(d5x4-d5x2) + (P4y-P2y)*(d5y4-d5y2) )/d;
  double d5b = d5d - ( d5d*(a2*a2 - a3*a3 + d*d))/(2*d*d);
  double d5h = -b*d1b/h;
  double d5yh = d5y2 + ( (d5b*d - d5d*b)/(d*d) )*(P4y-P2y)+(b/d)*(d5y4-d5y2);
  double d5xh = d5x2+( (d5b*d-d5b*d)/(d*d) )*(P4x-P2x)+(b/d)*(d5x4-d5x2);;
  double d5x3 = d5xh + (h/d)*(d5y4 - d5y2) + (d5h*d - d5d*h)/(d*d)*(P4y - P2y);
  double d5y3 = d5yh + (h/d)*(d5x4 - d5x2) + (d5h*d - d5d*h)/(d*d)*(P4x - P2x);
  //calculate velocity from omega (dTheta{i})
  double dxh = d1x3*dtheta1 + d5x3*dtheta5;
  double dyh = d1y3*dtheta1 + d5x3*dtheta5; 
  //calculate torque from force 
  Serial.print((float)dxh,3);
  Serial.print(" : ");
  Serial.println((float)dyh,3);

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
