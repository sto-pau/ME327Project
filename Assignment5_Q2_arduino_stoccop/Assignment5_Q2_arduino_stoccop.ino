//--------------------------------------------------------------------------
// Code to test basic Hapkit functionality (sensing and force output)
// Code updated by Cara Nunez 4.17.19
//--------------------------------------------------------------------------
// Parameters that define what environment to render
//#define ENABLE_VIRTUAL_WALL
//#define ENABLE_LINEAR_DAMPING
//#define ENABLE_NONLINEAR_FRICTION
//#define ENABLE_HARD_SURFACE
//#define ENABLE_BUMP_VALLEY
//#define ENABLE_TEXTURE
#define ENABLE_MASS_SPRING_DAMP

// Includes
#include <math.h>

//**
#include <AS5048A.h>
// the sensor CSn pin is connected to pin 10
AS5048A angleSensor(10);

// Pin declares
int pwmPin = 5; // PWM output pin for motor 1
int dirPin = 8; // direction output pin for motor 1
int sensorPosPin = A2; // input pin for MR sensor
int fsrPin = A3; // input pin for FSR sensor

// Position tracking variables
//**
//int updatedPos = 0;     // keeps track of the latest updated value of the MR sensor reading
long updatedPos = 0; // latest processed value of the Hall Effect sensor
int rawPos = 0;         // current raw reading from MR sensor
int lastRawPos = 0;     // last raw reading from MR sensor
int lastLastRawPos = 0; // last last raw reading from MR sensor
int flipNumber = 0;     // keeps track of the number of flips over the 180deg mark
int tempOffset = 0;
int rawDiff = 0;
int lastRawDiff = 0;
int rawOffset = 0;
int lastRawOffset = 0;
//**
//const int flipThresh = 700;  // threshold to determine whether or not a flip over the 180 degree mark occurred
const double flipThresh = 10000; // threshold to determine whether or not a flip over the 360 degree mark occurred, set low or missed at high speeds
boolean flipped = false;
double OFFSET = 16363; //measured max value before hall sensor flips, this is added and must be close or will create discontinuities in pos
double OFFSET_NEG = 15;

// Kinematics variables
double xh = 0;           // position of the handle [m]
double theta_s = 0;      // Angle of the sector pulley in deg
double xh_prev;          // Distance of the handle at previous time step
double xh_prev2;
double dxh;              // Velocity of the handle
double dxh_prev;
double dxh_prev2;
double dxh_filt;         // Filtered velocity of the handle
double dxh_filt_prev;
double dxh_filt_prev2;

// Force output variables
double force = 0;           // force at the handle
double Tp = 0;              // torque of the motor pulley
double duty = 0;            // duty cylce (between 0 and 255)
unsigned int output = 0;    // output command to the motor

//mass spring dampner
float xmass = 0.5 / 100; 
float velMass = 0.0;
float accMass = 0.0;
float velMassPrev = 0.0;
float accMassPrev = 0.0;

float xwall = 45.0 / 1000; 

float userForce = 0.0;
float springForce = 0.0;
float springEqX = 0.5 / 100.0;
float dampForce = 0.0;

float mass = 2.0;
float b = 3;
float k = 300.0;
float kUser = 1000.0;

int mSStart = 0;


// --------------------------------------------------------------
// Setup function -- NO NEED TO EDIT
// --------------------------------------------------------------
void setup() 
{
  //**
  angleSensor.init();
  
  // Set up serial communication
  Serial.begin(38400);
  
  // Set PWM frequency 
  setPwmFrequency(pwmPin,1); 
  
  // Input pins
  pinMode(sensorPosPin, INPUT); // set MR sensor pin to be an input
  pinMode(fsrPin, INPUT);       // set FSR sensor pin to be an input

  // Output pins
  pinMode(pwmPin, OUTPUT);  // PWM pin for motor A
  pinMode(dirPin, OUTPUT);  // dir pin for motor A
  
  // Initialize motor 
  analogWrite(pwmPin, 0);     // set to not be spinning (0/255)
  digitalWrite(dirPin, LOW);  // set direction
  
  //**
  // Initialize position valiables
  //lastLastRawPos = analogRead(sensorPosPin);
  //lastRawPos = analogRead(sensorPosPin);

  // Initialize position variables
  angleSensor.getRawRotation(); // take reading in case first reading is spurious
  lastLastRawPos = angleSensor.getRawRotation(); //position from Hall Effect sensor
  lastRawPos = angleSensor.getRawRotation(); //position from Hall Effect sensor
  
  flipNumber = 0;
}

// --------------------------------------------------------------
// Main Loop
// --------------------------------------------------------------
void loop()
{
  
  //*************************************************************
  //*** Section 1. Compute position in counts (do not change) ***  
  //*************************************************************

  //**
  // Get voltage output by MR sensor
  //rawPos = analogRead(sensorPosPin);  //current raw position from MR sensor

  // Get angle output by Hall Effect sensor
  rawPos = int(angleSensor.getRawRotation()); //position from Hall Effect sensor

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
  //*** Section 2. Compute position in meters *******************
  //*************************************************************

  // ADD YOUR CODE HERE
  // Define kinematic parameters you may need
     //double rh = ?;   //[m]
  // Step B.1: print updatedPos via serial monitor
  
  //Serial.println((String) rawPos + " " + updatedPos);
  
  // Step B.6: double ts = ?; // Compute the angle of the sector pulley (ts) in degrees based on updatedPos IN RADIANS
  double ts = 0.0015 * updatedPos - 6.6185; //this is IN DEGREES
  ts = ts * M_PI / 180;    //now in radians
  
  // Step B.7: xh = ?;       // Compute the position of the handle (in meters) based on ts (in radians)
  float rhandle = 90.0 / 1000.0; //don't forget the decimal point and zero or will not be decimal arithmatic
  xh =  - rhandle * ts; //for some reason, the default for me is handle to the right makes a negative number, and handle to the right is a positive numbers
  
  // Step B.8: print xh via serial monitor
  //Serial.println(xh, 5);

  // Calculate velocity with loop time estimation
  dxh = (double)(xh - xh_prev) / 0.001;

  // Calculate the filtered velocity of the handle using an infinite impulse response filter
  dxh_filt = .9*dxh + 0.1*dxh_prev; 
    
  // Record the position and velocity
  xh_prev2 = xh_prev;
  xh_prev = xh;
  
  dxh_prev2 = dxh_prev;
  dxh_prev = dxh;
  
  dxh_filt_prev2 = dxh_filt_prev;
  dxh_filt_prev = dxh_filt;
  
  //*************************************************************
  //*** Section 3. Assign a motor output force in Newtons *******  
  //*************************************************************
  //*************************************************************
  //******************* Rendering Algorithms ********************
  //*************************************************************
  
  // ADD YOUR CODE HERE
  // Define kinematic parameters you may need
     double rpulley = 5.0 / 1000.0;   //[m] from HW2 and physical measurement
     double rsector = 74.0 / 1000.0;   //[m] //from HW2 and measured from STL file 66.9 + 5 mm

#ifdef ENABLE_VIRTUAL_WALL
double kWall = 250;
double xWall = 5.0 / 1000.0; //[m] wall is 0.5cm

if (xh > xWall){
  force = kWall * (xWall - xh);
}
else{
  force = 0;
}

#endif

#ifdef ENABLE_LINEAR_DAMPING
double b = 25.0;

if (dxh_filt > 0.01 || dxh_filt < -0.01){ //some noise at no motion even after low pass filter
  force = -dxh_filt * b;
}
else{
  force = 0;
}

#endif

#ifdef ENABLE_NONLINEAR_FRICTION
#endif

#ifdef ENABLE_HARD_SURFACE
#endif

#ifdef ENABLE_BUMP_VALLEY  
//force constant
double kfeature = 100;

//valley location
double xvalley = 25.0 / 1000.0; //25mm to the right
//valley width
double wvalley = 5.0 / 1000.0; // 10mm wide

//bump location
double xbump = - 25.0 / 1000.0; //25mm to the left
//bump width
double wbump = 5.0 / 1000.0; // 10mm wide

if((xvalley - wvalley) < xh && xh < xvalley){ //if approaching valley center
  force = kfeature * (xh - (xvalley - wvalley));//force pulls in
}
else if ((xvalley + wvalley) > xh && xh > xvalley){ //if leaving valley
  force = -kfeature * (xh - (xvalley - wvalley));//force pushes out
}

else if( (xbump + wbump) > xh && xh > xbump){//if approaching bump center
  force = kfeature * (xh - xbump);//force pushed out
}
else if( xh < xbump && xh > (xbump - wbump) ){ //if leaving bump
  force = kfeature * (xh - xbump);//force pushes in
}

else{ //keep freespace free
  force = 0;
}

#endif

#ifdef ENABLE_TEXTURE
#endif

#ifdef ENABLE_MASS_SPRING_DAMP

//calculate current xmass position
  
  //forces
  //Serial.println((xh - xmass), 5);
  if( (xh - xmass) > 0 ){//user applied force
    userForce = kUser * ((float)xh - xmass);
  }
  else{
  userForce = 0;
  }

  force = - userForce * 0.25; //add Haptic feedback
//  if (force > 2.5){
//    force = 2.5;
//  }
//  if (force < -2.5){
//    force = -2.5;
//  }

  //Serial.println((String) userForce + "," + force);
  
  springForce = -k * (xmass - springEqX); //spring force
  //println(springForce);
    
  dampForce = -b * velMass; //dampner force  
  
  accMass = (userForce + springForce + dampForce) / mass; // calculate accleration from sum forces and mass
  //println(accMass);
  
  //change in time
  
  //float seconds = ( millis() - mSStart ) / (64 * 1000.0); // ; //divide by 64 to account for motor prescalar 
  //Serial.print(seconds,5);
  //Serial.print(",");
//  Serial.print(millis()/64,5);
//  Serial.print(",");
//  Serial.println(mSStart/64,5);

  //mSStart = millis(); //for next time
//  if (seconds <= 0.01){
//    seconds = seconds;
//  }
//  else{
//    seconds = 0.01;
//  }
    
  velMass = velMass + ( 0.5 * (accMassPrev + accMass) * (1.0 / 1000.0) ); //NOTE loop time estimation taken from arduino starter code 0.001;
  xmass = xmass + ( 0.5 * (velMassPrev + velMass) * (1.0 / 1000.0));
  
  //store acceleration and velocity from last time for use this time
  accMassPrev = accMass;
  velMassPrev = velMass;

  Serial.print(xh,5);
  Serial.print(",");
  Serial.print(xmass,5);
  Serial.print(",");
  Serial.println(force,5);
  //Serial.println(xmass,5);

#endif


  //Step C.1: this force is AT THE HANDLE
  //double kspring = 5.0;
  //force = kspring * (0 - xh); // You can  generate a force by assigning this to a constant number (in Newtons) or use a haptic rendering / virtual environment
  
  //Step C.2: // Compute the require motor pulley torque (Tp) to generate that force using kinematics
  Tp =  rhandle * rpulley * force / rsector;  

  //*************************************************************
  //*** Section 4. Force output (do not change) *****************
  //*************************************************************
  
  // Determine correct direction for motor torque
  if(force > 0) { 
    digitalWrite(dirPin, HIGH);
  } else {
    digitalWrite(dirPin, LOW);
  }

  // Compute the duty cycle required to generate Tp (torque at the motor pulley)
  duty = sqrt(abs(Tp)/0.03);

  // Make sure the duty cycle is between 0 and 100%
  if (duty > 1) {            
    duty = 1;
  } else if (duty < 0) { 
    duty = 0;
  }  
  output = (int)(duty* 255);   // convert duty cycle to output signal
  analogWrite(pwmPin,output);  // output the signal

  //for checking duty cycle and force before plugging in motor
  double xhMilli = xh * 1000.0;
  //Serial.println((String) force + " " + xhMilli + " " + duty);
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
