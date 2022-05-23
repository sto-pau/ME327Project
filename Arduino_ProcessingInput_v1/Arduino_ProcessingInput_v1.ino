//--------------------------------------------------------------------------
// Code to start Processing Code input setup
//--------------------------------------------------------------------------
// Parameters that define what environment to render
#define ENABLE_MASS_SPRING_DAMP 

#define DEBUGGING 
#define TESTING 

#define ARDBUFFER 16 //for serial line printing

// Includes
#include <math.h>


//constants here
float unitsDivisor = 1000.0; 

//workspace setup
const float lengthWorkspace = 193.0 / unitsDivisor; //usable work length
const int points = 4; //number of points to be used (needs to be constant to initialize arrays)
const float lengthBetween = lengthWorkspace / (points - 1); //distance between points
const float startingDepth = 125.0 / unitsDivisor; //thickness of clay block when starting

//function prototypes here (not needed for arduino?)
//float xmass[points] InitializeXmass(float xmass[points]);

// Kinematics variables
float yUser = 0.0;
float xUser = 0.0;
float dyUser = 0.0;

// Force & Torque output variables
double Tp = 0.0;              // torque of the motor pulley

//mass spring dampners system 
//position variables
float ymass[points] = {0.0}; //based on x & y axis of pantograph
float xmass[points] = {0.0}; //based on x & y axis of pantograph
  //use for loop to fill array with points distance lengthBetween IN MAIN  

//force calculation
float claySpringForce[points] = {0.0};
float clayDampForce[points] = {0.0};
float clayTotalForce[points] = {0.0};

float kUser = 1000.0;
float bClay = 150.0;
float kClay = 0.0;
float massClay = 2.0;
  
//change in position variables
float velMass[points] = {0.0};
float accMass[points] = {0.0};
float velMassPrev[points] =  {0.0};
float accMassPrev[points] =  {0.0};

int mSStart = 0;

// --------------------------------------------------------------
// Setup function -- NO NEED TO EDIT
// --------------------------------------------------------------
void setup() 
{  
  ///****setup, first starting****///
  
  Serial.begin(9600);
 
  SetAllElements(&ymass[0],startingDepth);
  //InitializeXmass(&ymass[0]);
  InitializeXmass(&xmass[0]);
//
//  Serial.println("Starting Arrays ymass xmass");
//  PrintArray(ymass);
//  PrintArray(xmass);
   
} // end of setup loop

// --------------------------------------------------------------
// Main Loop
// --------------------------------------------------------------

int loopNumber = 0;

void loop()
{
  
  //*************************************************************
  //******************* Rendering Algorithms ********************
  //*************************************************************
  
#ifdef ENABLE_MASS_SPRING_DAMP

#ifdef DEBUGGING

///****Recieve User Information****///
  
  xUser = xmass[1] - 0.05;
  //xUser = xmass[points - 1] + 0.05; //<xUser, yUser> = < -(xPos - 55), ypos + 89.8 > 

  yUser = startingDepth;

#ifdef TESTING

  int loopingRate = 500;

  if (loopNumber <= loopingRate){

    xUser = xmass[1] - 0.05;
    yUser = ymass[1] - (25.5 / unitsDivisor);
    loopNumber++;
    
  }  
//  else if (loopNumber <= 2 * loopingRate){
//
//    xUser = xmass[2] - 0.02;
//    yUser = ymass[2] - (12.5 / unitsDivisor);
//    loopNumber++;
//    
//  }
//   else if (loopNumber <= 3 * loopingRate){
//
//    xUser = xmass[3] - 0.01;
//    yUser = ymass[3] - (12.5 / unitsDivisor);
//    loopNumber++;
//    
//  }
  
#endif //TESTING

  if (xUser <= xmass[0]){//handle case if xUser is = or surpasses min/max xMass
    xUser = xmass[0];
  }
  else if (xUser >= xmass[points - 1]){
      xUser = xmass[points - 1];
  }   

  ///****Send User Information and Clay Information over Serial to Processing****///

  SendArrayOverSerial(xmass);
  SendArrayOverSerial(ymass);
  Serial.print(xUser,3);
  Serial.print(",");
  Serial.print(yUser,3);
  Serial.println();  

  ///****Collision Detection Setup****///
  
  //find which two points will be affected
  float xdiffUserMass[points] = {0}; //array containing difference between user position and all clay positions
  
  memcpy(xdiffUserMass, xmass, sizeof(xdiffUserMass));
  
  AddValue(xdiffUserMass, -xUser);

  int clayIndexClosest = points + 1; //should start as impossible index

  clayIndexClosest = indexMin(xdiffUserMass); //choose the clay element to interact with as the element with minimum xdiffUserMass 

  int clayIndexNext = points + 1; //should start as impossible index

  //find 2nd element depending on if  xUser  > or < xMass
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
  
  //find penetration distance
  
  //line equations
  int slopeLowerIndex = min(clayIndexClosest, clayIndexNext);
  
  int slopeHigherIndex = max(clayIndexClosest, clayIndexNext);

  float xLineWeight = 0.0;
  float yLineWeight = 0.0;
  float lineConstant = 0.0;
  
  yLineWeight = xmass[slopeHigherIndex] - xmass[slopeLowerIndex]; //b 
  xLineWeight = ymass[slopeLowerIndex] - ymass[slopeHigherIndex]; //a
  lineConstant = xmass[slopeLowerIndex] * ymass[slopeHigherIndex] - ymass[slopeLowerIndex] * xmass[slopeHigherIndex]; //c

  //determine if there is penetration by finding the depth at the user height to be on the line
  float yOnLineUser = - ( (xLineWeight * xUser + lineConstant) / yLineWeight ); //if depth is not less than this, then not inside the line

  ///****Test Collision and Set userForce onto the clay accordingly****///

  //initialize userForce as all zeroes before proving contact was made
  float userForce[points] = {0.0};
  float userForceMag = 0.0; //if no contact found forceX and forceY should be 0

  if (yUser < yOnLineUser){ //if there is contact, set the adjacent clay userForce not to zero  

    //force calculation variables 
      
      float d = abs ( xLineWeight * xUser + yLineWeight * yUser  + lineConstant ) / sqrt(xLineWeight*xLineWeight + yLineWeight*yLineWeight); //penetration distance
      
      userForceMag = kUser * d; 
      
      userForce[clayIndexClosest] = -userForceMag * abs( ( xUser - xmass[clayIndexClosest] ) / lengthBetween ); //need to add negative such that clay is being pushed inwards
      userForce[clayIndexNext] = -userForceMag * d * abs( ( xUser - xmass[clayIndexNext] ) / lengthBetween ); //see above
            
  }  

 ///****Calculate force on the user****///

  //unit vector perpendicular to line components
  float unitDirectionY = ( ymass[slopeHigherIndex] - ymass[slopeLowerIndex] ) / sqrt(xLineWeight*xLineWeight + yLineWeight*yLineWeight); 
  float unitDirectionX = ( xmass[slopeHigherIndex] - xmass[slopeLowerIndex] ) / sqrt(xLineWeight*xLineWeight + yLineWeight*yLineWeight);

  //force at handle, if no contact found userForceMag will be 0 
  //force applied to the User should be perpendicular to the line
  double forceX = - userForceMag * unitDirectionY; //need multiply by negative unitDirectionY weight to have the user direction point OUT of the clay
  double forceY = userForceMag * unitDirectionX;
 
 ///****Calculate total force on clay****///
 
    UpdateClaySpringForce(&claySpringForce[0], &ymass[0]); //spring force

    UpdateClayDampForce(&clayDampForce[0], &velMass[0]);//dampner force

    UpdateTotalForce(clayTotalForce,clayDampForce,claySpringForce,userForce); //total force  

 ///****Calculate resulting motion****///

 //update all the masses velocities, accelerations, and forces
    //calculate acceleration from F = ma
    UpdateClayAccelerations(clayTotalForce, accMass);   

    //integrate to find velocity and postion

    //change in time
    float loopTime = (1.0 / 1000.0);

    //calculated based on actual loop time (may need to use if becomes very slow)
    float mSTemp = millis();
    float seconds = ( mSTemp - mSStart ) / (64 * 1000.0); //divide by 64 to account for motor prescalar
    mSStart = mSTemp;

    
    IntegratePrevious(velMass,accMass,accMassPrev, loopTime); //integrate for velocity
    IntegratePrevious(ymass,velMass,velMassPrev, loopTime); //integrate for position  


    //store acceleration and velocity from last time for use this time
    memcpy(accMassPrev, accMass, sizeof(accMassPrev));
    memcpy(velMassPrev, velMass, sizeof(velMassPrev));   

#endif //DEBUGGING

#endif //ENABLE_MASS_SPRING_DAMP

  //calculate torque output here
  //T[T1 T2] =  J.transpose(dx3, dy3, dth1, dth4) * Force[Fx Fy];
 
} //end of main loop

///****Function****///

void InitializeXmass(float xmass[]){
  for (int index = 1; index < points; xmass[index] += xmass[index-1] + lengthBetween, index++){}
  return;
}

void SetAllElements(float ymass[], const float startingDepth){
  for (int index = 0; index < points; ymass[index] = startingDepth, index++){}
  return;
}

void AddValue(float targetArray[], float val2add){
  for (int index = 0; index < points; targetArray[index] += val2add, index++){}
  return;
}

void UpdateClaySpringForce(float claySpringForce[], float ymass[]){
  for (int index = 0; index < points; index++){    
    claySpringForce[index] = kClay * (ymass[index] - startingDepth);  //no negative sign so that the clay resists OUTWARDS, positive Y direction     
  }  
 return;
}

void UpdateClayDampForce(float clayDampForce[], float velMass[]){
  for (int index = 0; index < points; index++){    
    clayDampForce[index] = -bClay * velMass[index];          
  }  
 return;
}

void UpdateTotalForce(float clayTotalForce[], float clayDampForce[], float claySpringForce[], float userForce[]){
  for (int index = 0; index < points; index++){    
    clayTotalForce[index] = clayDampForce[index] + claySpringForce[index] + userForce[index];           
  }  
 return;
}

void UpdateClayAccelerations(float clayTotalForce[], float accMass[]){
  
  for (int index = 0; index < points; index++){    
    accMass[index] = clayTotalForce[index] / massClay;           
  }  
 return;
 
}

void IntegratePrevious(float velMass[], float accMass[], float accMassPrev[], float loopTime){
  //for position integration yMass, velMass, vellMassPrev
  
  for (int index = 0; index < points; index++){    
    velMass[index] = velMass[index] + ( 0.5 * (accMassPrev[index] + accMass[index]) * (loopTime) ); //NOTE loop time estimation taken from arduino starter code 0.001;           
  } 
   
 return;
 
}

void PrintArray(float printArray[]){  
  for (int index = 0; index < points; index++){
    Serial.print(printArray[index],3);
    Serial.print(", ");
    }   
  Serial.println();
  return;    
}

void SendArrayOverSerial(float printArray[]){  
  for (int index = 0; index < points; index++){
    Serial.print(printArray[index],3);
    Serial.print(",");
    }   
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
