//--------------------------------------------------------------------------
// Code to start Processing Code input setup
//--------------------------------------------------------------------------
// Parameters that define what environment to render
#define ENABLE_MASS_SPRING_DAMP

// Includes
#include <math.h>

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

}

// --------------------------------------------------------------
// Main Loop
// --------------------------------------------------------------
void loop()
{
  //*************************************************************
  //******************* Rendering Algorithms ********************
  //*************************************************************
  
// ADD YOUR CODE HERE
// Define kinematic parameters you may need
   double rpulley = 5.0 / 1000.0;   //[m] from HW2 and physical measurement
   double rsector = 74.0 / 1000.0;   //[m] //from HW2 and measured from STL file 66.9 + 5 mm

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

  //calculate torque output here
  //T[T1 T2] =  J.transpose(dx3, dy3, dth1, dth4) * Force[Fx Fy];
 
}
