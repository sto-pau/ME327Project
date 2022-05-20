//--------------------------------------------------------------------------
// Code to start Processing Code input setup
//--------------------------------------------------------------------------
// Parameters that define what environment to render
#define ENABLE_MASS_SPRING_DAMP 

//#define DEBUGGING 

#define ARDBUFFER 16 //for serial line printing

// Includes
#include <math.h>


//constants here
float unitsDivisor = 1000.0; 

//workspace setup
const float lengthWorkspace = 193.0 / unitsDivisor; //usable work length
const int points = 10; //number of points to be used (needs to be constant to initialize arrays)
const float lengthBetween = lengthWorkspace / (points - 1); //distance between points
const float startingDepth = 125.0 / unitsDivisor; //thickness of clay block when starting

//function prototypes here (not needed for arduino?)
//float xmass[points] InitializeXmass(float xmass[points]);

// Kinematics variables
float yUser = 0.0;
float xUser = 0.0;
float dyUser = 0.0;

// Force output variables
double force = 0.0;           // force at the handle
double Tp = 0.0;              // torque of the motor pulley

//mass spring dampners system 
//position variables
float ymass[points] = {0.0}; //based on x & y axis of pantograph
float xmass[points] = {0.0}; //based on x & y axis of pantograph
  //use for loop to fill array with points distance lengthBetween IN MAIN     
  
//change in position variables
float velMass[points] = {0.0};
float accMass[points] = {0.0};
float velMassPrev[points] =  {0.0};
float accMassPrev[points] =  {0.0};

//force calculation variables
  //clay variables refer to each individual block, not the total mass
float kClay = 100.0;
float dClay = 10.0;
float massClay = 2.0;
float kUser = 1000.0;

int mSStart = 0;

// --------------------------------------------------------------
// Setup function -- NO NEED TO EDIT
// --------------------------------------------------------------
void setup() 
{  

  ///****setup, first starting****///
  //InitializeYmass(&ymass[0],startingDepth);
  InitializeXmass(&ymass[0]);
  InitializeXmass(&xmass[0]);
  
  Serial.begin(9600);
  
  PrintArray(ymass);
  PrintArray(xmass);
  
  ///****Recieve User Information****///
  
  //xUser = xmass[0] - 0.05;
  xUser = xmass[points - 1] + 0.05; //<xUser, yUser> = < -(xPos - 55), ypos + 89.8 > 
  Serial.println(xUser,6);

  if (xUser <= xmass[0]){
    xUser = xmass[0];
  }
  else if (xUser >= xmass[points - 1]){
      xUser = xmass[points - 1];
  } 
  
  Serial.println(xUser,6);

  ///****Collision Detection Setup****///
  
  float xdiffUserMass[points] = {0};
  
  memcpy(xdiffUserMass, xmass, sizeof(xdiffUserMass));
  
  AddValue(xdiffUserMass, -xUser);
  PrintArray(xdiffUserMass);

  int clayIndexClosest = points + 1; //should start as impossible index

  clayIndexClosest = indexMin(xdiffUserMass);
  Serial.println(clayIndexClosest);

  int clayIndexNext = points + 1; //should start as impossible index
  
  if (xUser > xmass[clayIndexClosest]){ // inequality could be if xdiffUserMass[clayIndexClosest] <= 0 
    clayIndexNext = clayIndexClosest + 1;
  }
  else if (xUser < xmass[clayIndexClosest]){
    clayIndexNext = clayIndexClosest - 1;
  }
  else if (xUser == xmass[clayIndexClosest]){//if equal, check if at top or bottom
    if(clayIndexClosest == 0){
      clayIndexNext = clayIndexClosest + 1;
    }
    else if( clayIndexClosest == (points - 1) ){
        clayIndexNext = clayIndexClosest - 1;
    }
  }

  Serial.println(clayIndexNext);

  xmass[clayIndexClosest] = 50; //150
  ymass[clayIndexClosest] = 50; //0

  xmass[clayIndexNext] = 150; //50
  ymass[clayIndexNext] = 50; //100 

  //17.68 //17.68

  Serial.println(clayIndexNext);

  //line equations
  int slopeLowerIndex = min(clayIndexClosest, clayIndexNext);
  Serial.println(slopeLowerIndex);
  
  int slopeHigherIndex = max(clayIndexClosest, clayIndexNext);
  Serial.println(slopeHigherIndex);

  float xLineWeight = 0.0;
  float yLineWeight = 0.0;
  float lineConstant = 0.0;
  
  yLineWeight = xmass[slopeHigherIndex] - xmass[slopeLowerIndex]; //b 
  xLineWeight = ymass[slopeLowerIndex] - ymass[slopeHigherIndex]; //a
  lineConstant = xmass[slopeLowerIndex] * ymass[slopeHigherIndex] - ymass[slopeLowerIndex] * xmass[slopeHigherIndex]; //c

  ardprintf("%f, %f, %f", xLineWeight, yLineWeight, lineConstant);

  xUser = 100;
  yUser = 51;

  float yOnLineUser = - ( (xLineWeight * xUser + lineConstant) / yLineWeight );
  Serial.println(yOnLineUser);

  float d = abs ( xLineWeight * xUser + yLineWeight * yUser  + lineConstant ) / sqrt(xLineWeight*xLineWeight + yLineWeight*yLineWeight); //penetration distance
  Serial.println(d);

  ///****Test Collision and Set userForce onto the clay accordingly****///

  //initialize userForce as all zeroes before proving contact was made
  float userForce[points] = {0};

//  if (yUser < yOnLineUser){ //if there is contact, set the adjacent clay userForce not to zero  
//    
//      float d = abs ( xLineWeight * xUser + yLineWeight * yUser  + lineConstant ) / sqrt(xLineWeight*xLineWeight + yLineWeight*yLineWeight); //penetration distance
//      Serial.println(d);
//      userForce[clayIndexClosest] = kUser* d * abs( ( xUser - xmass[clayIndexClosest] ) / lengthBetween );
//      userForce[clayIndexNext] = kUser * d * abs( ( xUser - xmass[clayIndexNext] ) / lengthBetween );
//            
//  }

 ///****Calculate total force on clay****///

 ///***Calculate resulting motion****///
  

  
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
  for (int index = 1; index < points; xmass[index] += xmass[index-1] + lengthBetween, index++){}
  return;
}


void InitializeYmass(float ymass[], const float startingDepth){
  for (int index = 0; index < points; ymass[index] = startingDepth, index++){}
  return;
}

void AddValue(float targetArray[], float val2add){
  for (int index = 0; index < points; targetArray[index] += val2add, index++){}
  return;
}

void PrintArray(float printArray[]){  
  Serial.print("\n");
  for (int index = 0; index < points; index++){
    Serial.print(printArray[index],6);
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

int ardprintf(char *str, ...)
{
  int i, count=0, j=0, flag=0;
  char temp[ARDBUFFER+1];
  for(i=0; str[i]!='\0';i++)  if(str[i]=='%')  count++;

  va_list argv;
  va_start(argv, count);
  for(i=0,j=0; str[i]!='\0';i++)
  {
    if(str[i]=='%')
    {
      temp[j] = '\0';
      Serial.print(temp);
      j=0;
      temp[0] = '\0';

      switch(str[++i])
      {
        case 'd': Serial.print(va_arg(argv, int));
                  break;
        case 'l': Serial.print(va_arg(argv, long));
                  break;
        case 'f': Serial.print(va_arg(argv, double));
                  break;
        case 'c': Serial.print((char)va_arg(argv, int));
                  break;
        case 's': Serial.print(va_arg(argv, char *));
                  break;
        default:  ;
      };
    }
    else 
    {
      temp[j] = str[i];
      j = (j+1)%ARDBUFFER;
      if(j==0) 
      {
        temp[ARDBUFFER] = '\0';
        Serial.print(temp);
        temp[0]='\0';
      }
    }
  };
  Serial.println();
  return count + 1;
}
