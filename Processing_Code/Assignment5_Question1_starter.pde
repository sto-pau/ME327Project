// Pro_Graph2.pde
/*
 Based on the Arduining example which is based on the Tom Igoe example.
 Mofified by Cara Nunez 5/1/2019:
  -A wider line was used. strokeWeight(4);
  -Continuous line instead of vertical lines.
  -Bigger Window size 600x400.
-------------------------------------------------------------------------------
This program takes ASCII-encoded strings
from the serial port at 9600 baud and graphs them. It expects values in the
range 0 to 1023, followed by a newline, or newline and carriage return


*/

import processing.serial.*;

Serial myPort;        // The serial port

//initialize variables
float inByte = 0;
float lastByte = 0;

// self initialized variables
//String rxString;
String[] rxArray;
int idx1;
float newX1; // x and y are flipped in the serial
float newY1;
int idx2;
float newX2;
float newY2;

// constant values for graphics
float buffer;
float xMin = 0;
float xMax = 0.085;
float yMin = 0;
float yMax = 0.075;
float xMapMin; // defines region for actual movement in display
float xMapMax;
float yMapMin;
float yMapMax;
float clayWidth = 0.040; //initial width of the clay (in real world)
float clayMapWidth;

// array to store point positions
int numPoints = 10;
float[][] points;
// handle coordinates
float rawXh;
float rawYh;
float xh;
float yh;

// font setups
PFont font;
int opacity = 0;
int direction = 1;

// colors
color color1 = #E5D3B3;
color color2 = #432616;
color color3 = #FFFDFA;

// rotating arc parameters
float AngleStart = 0;  // where outer ring starts
float Speed = 30;       // how fast everything spins
float AngleBump = Speed*0.1;   // added rotation of each ring

// other graphing setup
int curvature = 50; // parameter for how much curvature the lines along the surface have

void setup () {
  // set the window size:
  //size(600, 400);
  fullScreen();
  // set region for actual movement in display
  buffer = height/12;
  xMapMin = width/2; 
  xMapMax = width/2+width/3;
  yMapMin = buffer;
  yMapMax = height-buffer*2;
  
  // font setup
  font = createFont("TropicalAsianDemoRegular-11V0.ttf", 24);
  textFont(font);

  // initialize points
  points = new float[numPoints][2];
  // map clay block width
  clayMapWidth = map(clayWidth, xMin, xMax, xMapMin, xMapMax);
  // initialize point positions
  for (int i = 0; i < numPoints; i += 1){
    points[i][0] = clayMapWidth;
    //points[i][1] = (i+1) * height/(numPoints+1);
    points[i][1] = i * (yMapMax-yMapMin)/(numPoints-1) + yMapMin;
  }

  // List all the available serial ports
  println(Serial.list());
  // Check the listed serial ports in your machine
  // and use the correct index number in Serial.list()[  ].
  myPort = new Serial(this, Serial.list()[0], 38400);//make sure baud rate matches Arduino
  
  // A serialEvent() is generated when a newline character is received :
  myPort.bufferUntil('\n');
  background(color1);      // set inital background:
}

void draw () {
  background(color1);
  // draw range for x and y
  //strokeWeight(2);
  //noFill();
  //rect(xMapMin, yMapMin, xMapMax-xMapMin, yMapMax-yMapMin);
    
  // set initial stroke color and weight
  stroke(color2);     //stroke color
  strokeWeight(2);        //stroke width
  
  // draw center line
  //line(xMapMin, 0, xMapMin, height);
  
  // draw title
  textSize(height/5);
  fill(color2);
  textLeading(height*3/20);
  text("Virtual \nPottery \nMaking", width/25, height/5);
  
  // draw rotating arc
  noFill();
  drawArc(width/2, height-buffer*2, width/2, curvature*2); // rotating arc
  ellipse(width/2, height-buffer*2, width*2/3, curvature*3);
  noStroke();
  fill(color1);
  rect(width-points[8][0]+2, points[8][1]-curvature/2-60, 2*points[8][0]-width-4, curvature+80); // cover arc when behind pottery
  
  // draw points and contour for pottery
  for (int i = 0; i < numPoints; i += 1) {
    // draw points on the pottery
    //stroke(color2);
    //strokeWeight(6);
    //point(points[i][0], points[i][1]); 
    // curves for displaying volume
    stroke(color2);
    strokeWeight(4);
    noFill();
    ellipse(width/2, points[i][1], 2*points[i][0]-width, curvature);
    // lines along side & and cover volume ellipse
    if (i>=1){ 
      // rect for covering top of the ellipse
      noStroke();
      fill(color1);
      rect(width-points[i][0]+2, points[i][1]-curvature/2-10, 2*points[i][0]-width-4, curvature/2+10);
      // lines along ledge
      stroke(color2);
      strokeWeight(4);
      line(points[i-1][0], points[i-1][1], points[i][0], points[i][1]);
      line(width-points[i-1][0], points[i-1][1], width-points[i][0], points[i][1]);
    }
    // lines on top and bottom
    //if (i==0 || i==numPoints-1){ 
    //  line(width-points[i][0], points[i][1], points[i][0], points[i][1]);
    //}
  }
  // draw user position
  stroke(color2);
  strokeWeight(4);
  ellipse(xh, yh, 20, 20);
}

void serialEvent (Serial myPort) {
  // read input string
  String rxString = myPort.readStringUntil('\n');
  println("rxString", rxString);
  // process received data
  if (rxString != null){
    // split into array of strings
    rxArray = split(rxString, ',');
    // convert values to ints/floats and update values
    idx1 = int(rxArray[0].trim());
    newX1 = float(rxArray[1].trim());
    newY1 = float(rxArray[2].trim());
    idx2 = int(rxArray[3].trim());
    newX2 = float(rxArray[4].trim());
    newY2 = float(rxArray[5].trim());
    rawXh = float(rxArray[6].trim());
    rawYh = float(rxArray[7].trim()); // TODO: check this if added extra elements
    float forceX;
    float forceY;
    forceX = float(rxArray[8].trim());
    forceY = float(rxArray[9]); // TODO: check this if added extra elements
    
    
    println("xUser", rawXh*1000, "yUser", rawYh*1000);
    // reset points for reset message
    if (idx1 == 327){
      for (int i = 0; i < numPoints; i += 1){
        points[i][0] = clayMapWidth;
        //points[i][1] = (i+1) * height/(numPoints+1);
        points[i][1] = i * (yMapMax-yMapMin)/(numPoints-1) + yMapMin;
      }
    }
    else{
      // map and update values
      xh = map(rawXh, xMin, xMax, xMapMin, xMapMax);  // x handle position
      yh = map(rawYh, yMin, yMax, yMapMax, yMapMin);  // y handle position, flipping y max and min to match coordinates
      points[numPoints-idx1-1][0] = map(newX1, xMin, xMax, xMapMin, xMapMax);  // first point x, mapping to numPoints-idx-1 to index from top
      //points[idx1][1] = map(newY1, yMin, yMax, yMapMin, yMapMax);  // first point y
      points[numPoints-idx2-1][0] = map(newX2, xMin, xMax, xMapMin, xMapMax);  // second point x
      //points[idx2][1] = map(newY2, yMin, yMax, yMapMin, yMapMax);  // second point y
    }
  }
}

/*
Description:
draw rotating arcs, rotating as time goes
partially adapted from https://imaginary-institute.com/previews/preview-wheels.html
Input:
float centerX - center of arc, x
float centerY - center of arc, y
float xWidth - length of arc in x direction
float yWidth - length of arc in y direction
*/
void drawArc(float centerX, float centerY, float xWidth, float yWidth){
  stroke(color2);
  strokeWeight(4);
  float angle = AngleStart;  // starting angle
  arc(centerX, centerY, xWidth, yWidth, angle, angle+PI/6);
  AngleStart += Speed*.01;  // next frame, start here
}
