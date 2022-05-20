//--------------------------------------------------------------------------
// Code to start Processing Code input setup
//--------------------------------------------------------------------------
// Parameters that define what environment to render
#define ENABLE_MASS_SPRING_DAMP 
//#define DEBUGGING 

// Includes
#include <math.h>


//constants here
float unitsDivisor = 1000.0; 

//workspace setup
const float lengthWorkspace = 198.0 / unitsDivisor; //usable work length
const int points = 10; //number of points to be used (needs to be constant to initialize arrays)
const float lengthBetween = lengthWorkspace / (points - 2); //distance between points
const float startingDepth = 125 / unitsDivisor; //thickness of clay block when starting

//function prototypes here (not needed for arduino?)
//float xmass[points] InitializeXmass(float xmass[points]);

// Kinematics variables
float yUser = 0;
float xUser = 0;
float dyUser = 0;

// Force output variables
double force = 0;           // force at the handle
double Tp = 0;              // torque of the motor pulley

//mass spring dampners system 
//position variables
float ymass[points] = {0}; //based on x & y axis of pantograph
float xmass[points] = {0}; //based on x & y axis of pantograph
  //use for loop to fill array with points distance lengthBetween IN MAIN     
  
//change in position variables
float velMass[points] = {0};
float accMass[points] = {0};
float velMassPrev[points] =  {0};
float accMassPrev[points] =  {0};
//force calculation variables
float userForceX = 0.0;
float userForceY = 0.0;

//calculation variables
float xdiffUserMass[points] = {0};
int clayIndexClosest = points + 1; //should start as impossible index
int clayIndexNext = points + 1; //should start as impossible index

int mSStart = 0;

// --------------------------------------------------------------
// Setup function -- NO NEED TO EDIT
// --------------------------------------------------------------
void setup() 
{
  
  InitializeYmass(&ymass[0],startingDepth);
  InitializeXmass(&xmass[0]);
  
  Serial.begin(9600);


  xUser = lengthWorkspace / 2;
  Serial.println(xUser,3);
  PrintArray(ymass);
  PrintArray(xmass);
  Serial.println(xUser); 
  memcpy(xdiffUserMass, xmass, sizeof(xdiffUserMass));
  AddValue(xdiffUserMass, -xUser);
  PrintArray(xdiffUserMass);

  clayIndexClosest = indexMin(xdiffUserMass);
  Serial.println(clayIndexClosest);

}

// --------------------------------------------------------------
// Main Loop
// --------------------------------------------------------------
void loop()
{
  
  //*************************************************************
  //******************* Rendering Algorithms ********************
  //*************************************************************
  
#ifdef ENABLE_MASS_SPRING_DAMP

#ifdef DEBUGGING

//find which two points will be affected
    //array containing difference between user position and all clay positions

    //choose the clay element to interact with as
    //the element with minimum xdiffUserMass //function that gets minimum value

    //find 2nd element depending on if  xUser  > or < xMass


//calculate user applied force NOTE: There are no Fy forces on the MASSES because they do not translate up and down
//the reaction force on the user WILL have Fy forces
    //FUserx1 = k * d * abs( xUser - x1 ) / lengthBetween) ) + b * vXUser * abs( xUser - x1 ) / lengthBetween) )
    //FUserx2 = k * d * abs( xUser - x2 ) / lengthBetween) ) + b * vXUser * abs( xUser - x1 ) / lengthBetween) )


//calculate current xmass position

//find penetration distance 
    //calculate line equation
        //ax + by + c = 0
            //a = y1 - y2
            //b = x2 - x1
            //c = (x1 - x2) * y1 + (y2 - y1) * x1
    //calculate normal distance
        //d = abs ( a * xuser + b * yuser + c) / sqrt( a^2 + b^2 )

  
  //forces
  //Serial.println((xh - xmass), 5);
  if( (xh - xmass) > 0 ){//user applied force
    userForce = kUser * ((float)xh - xmass);
  }
  else{
  userForce = 0;
  }
  
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

#endif

  //calculate torque output here
  //T[T1 T2] =  J.transpose(dx3, dy3, dth1, dth4) * Force[Fx Fy];
 
}

void InitializeXmass(float xmass[]){
  for (int index = 1; index < points - 1; xmass[index] += xmass[index-1] + lengthBetween, index++){}
  return;
}


void InitializeYmass(float ymass[], const float startingDepth){
  for (int index = 0; index < points - 1; ymass[index] = startingDepth, index++){}
  return;
}

void AddValue(float targetArray[], float val2add){
  for (int index = 0; index < points - 1; targetArray[index] += val2add, index++){}
  return;
}

void PrintArray(float printArray[]){  
  Serial.print("\n");
  for (int index = 0; index < points - 1 ; index++){
    Serial.print(printArray[index],3);
    Serial.print(" ");
    }
  Serial.print("\n");
  Serial.println();  
  return;  
}

int indexMin(float targetArray[]){
  
    int minIndex = 0; //have to create outside forloop to pass it out
    
    for(int currentIndex = 1; currentIndex < points; currentIndex++){        
        if(abs(targetArray[currentIndex]) < abs(targetArray[minIndex]))
          minIndex = currentIndex;           
    }
    
    return minIndex;
}
