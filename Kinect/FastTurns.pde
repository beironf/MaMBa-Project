/// Daniel Shiffman
// All features test

// https://github.com/shiffman/OpenKinect-for-Processing
// http://shiffman.net/p5/kinect/

import org.openkinect.freenect.*;
import org.openkinect.freenect2.*;
import org.openkinect.processing.*;
import blobDetection.*;
import java.awt.*;
import java.util.*;
import processing.net.*;
import java.net.InetAddress;

// Server variables
Server myServer;
int port = 3233;
Client c;
String msg;
String myIP;
InetAddress inet;

Kinect2 kinect2;
boolean showImage = true;
float distanceX;
float distanceY;

// Variabler för boll
float redThreshold = 0.3;
int redCount = 0;
int sumX = 0;
int sumY = 0;
float meanDepth;
int meanX;
int meanY;

// Variabler för val av övning
boolean rollToTarget = false;
boolean throwAgainstTarget = false;
boolean moveToCircle = false;
boolean fastTurns = false;
boolean doneWithExercise = false;




// Fast Turns Variables 
int greenCountLeft;
int greenCountRight;
int sumGoalXLeft;
int sumGoalXRight;
int sumGoalYLeft;
int sumGoalYRight;
float goalDepthLeft;
float goalDepthRight;
int meanGoalXLeft;
int meanGoalXRight;
int meanGoalYLeft;
int meanGoalYRight;
float greenThreshold = 0.1;
int startTime;
int nrOfTotalTurns = 10;
float redXPos;
float rightXPos;
float leftXPos;
float tStart;
float tid = 0;
boolean shouldPassRight;
boolean isDone;




// ----------- Methods -------------
float calculateRedValue (color c) {
    return red(c)/255 - (green(c)/255 + blue(c)/255)/2;
}

float calculateGreenValue (color c) {
    return green(c)/255 - (red(c)/255+blue(c)/255)/2;
}

float calculateBlueValue (color c) {
    return blue(c)/255 - (red(c)/255+green(c)/255)/2;
}

/*float distanceBetween (float rX, float rY, float rZ, float gX, float gY, float gZ) {
  float gXLengthPerPixel = tan(35.3*PI/180)*gZ/256;
  float rXLengthPerPixel = tan(35.3*PI/180)*rZ/256;
  float gYLengthPerPixel = tan(30*PI/180)*gZ/212;
  float rYLengthPerPixel = tan(30*PI/180)*rZ/212;
  rX = (rX-256)*rXLengthPerPixel*(-1);
  rY = (rY-212)*rYLengthPerPixel*(-1);
  gX = (gX-256)*gXLengthPerPixel*(-1);
  gY = (gY-212)*gYLengthPerPixel*(-1);
  //println("rx: " + rX + " ry: " + rY);
  return sqrt((rX-gX)*(rX-gX) + (rY-gY)*(rY-gY) + (rZ-gZ)*(rZ-gZ));
}

float distanceBetweenX (float rX, float rZ, float gX, float gZ) {
  float gXLengthPerPixel = tan(35.3*PI/180)*gZ/256;
  float rXLengthPerPixel = tan(35.3*PI/180)*rZ/256;
  rX = (rX-256)*rXLengthPerPixel*(-1);
  gX = (gX-256)*gXLengthPerPixel*(-1);
  return abs(rX - gX);
}

float distanceBetweenY (float rY, float rZ, float gY, float gZ) {
  float gYLengthPerPixel = tan(30*PI/180)*gZ/212;
  float rYLengthPerPixel = tan(30*PI/180)*rZ/212;
  rY = (rY-212)*rYLengthPerPixel*(-1);
  gY = (gY-212)*gYLengthPerPixel*(-1);
  return abs(rY - gY);
}*/


float calculateXPosition (float rX, float rZ) {
  float lengthPerPixelX = tan(35.3*PI/180)*rZ/256;
  rX = (rX-256)*lengthPerPixelX;
  return rX;
}

void keyPressed() {
  if (key == CODED) {
    // Increase of decrease the threshold for green or red with the arrows
    if (keyCode == RIGHT) {
      redThreshold += 0.03;
    } else if (keyCode == LEFT) {
      redThreshold -= 0.03;
    } else if (keyCode == UP) {
      greenThreshold += 0.03;
    } else if (keyCode == DOWN) {
      greenThreshold -= 0.03;
    }
  }
  // Show image if 'i' is being pressed
  if (key == 'i') {
    if (!showImage) {
      showImage = true;
    } else {
      showImage = false;
    }
    // Start the time for Move To Circle if 
  } else if (key == 'a') {
      startTime = millis();
      isDone = false;     
  } else if (key == 'f') { // Start the excercies Fast Turns
    fastTurns = true;
  } 
} 

void setup() {
  size(512, 424);
  kinect2 = new Kinect2(this);
  kinect2.initDepth();
  kinect2.initVideo();
  kinect2.initIR();
  kinect2.initRegistered();
  // Start all data
  kinect2.initDevice();
  frameRate(30); // camera limit
  
  // Server
  myServer = new Server(this, port);
  try {
    inet = InetAddress.getLocalHost();
    myIP = inet.getHostAddress();
  }
  catch (Exception e) {
    e.printStackTrace();
    myIP = "couldnt get IP";
  }
  println(myIP);
} 
 
void draw() {
  background(0);
  PImage img = kinect2.getRegisteredImage();
  
  int[] depthRaw = kinect2.getRawDepth();
  img.loadPixels();
  // check distance to some points on the wall
  int HU = 512*50+256+100;
  int HN = 512*250+256+100;
  int VU = 512*50+256-100;
  int VN = 512*250+256-100;
  float precision = 5.0;
  
  // Adjust the camera up or down
  fill(255);
  if ((depthRaw[VN]+depthRaw[HN])/2 > (depthRaw[VU]+depthRaw[HU])/2 + precision) {
    text("Vinkla upp kameran", 10, 405);
  } else if ((depthRaw[VN]+depthRaw[HN])/2 < (depthRaw[VU]+depthRaw[HU])/2 - precision) {
    text("Vinkla ner kameran", 10, 405);
  } else {
    text("", 10, 405);
  }
  // Adjust the camera left of right
  if ((depthRaw[VN]+depthRaw[VU])/2 > (depthRaw[HU]+depthRaw[HN])/2 + precision) {
    text("Vinkla kameran till vänster", 200, 405);
  } else if ((depthRaw[VN]+depthRaw[VU])/2 < (depthRaw[VU]+depthRaw[HU])/2 - precision) {
    text("Vinkla kameran till höger", 200, 405);
  } else {
    text("", 200, 405);
  }
  for (int i = 0; i < 512*424; i += 1) {
    if (calculateGreenValue(img.pixels[i]) >= greenThreshold) { 

      // Set pixel to green
      img.pixels[i] = color(0, 255, 0);
    }
    if (calculateRedValue(img.pixels[i]) >= redThreshold) { 

      // Set pixel to red
      img.pixels[i] = color(255, 0, 0);
    }
  }
  text("Green Threshold: " + greenThreshold + "     Red Threshold: " + redThreshold + "            Host IP: " + myIP, 10, 420);
  
  img.updatePixels();
  image(img, 0, 0, kinect2.depthWidth, kinect2.depthHeight);
  c = myServer.available();      // Get the next available client
  if (c != null) {
    if (c.available() > 0) {    // Are there any data incoming?
      msg = c.readString();     // Read the data
      if (msg != "") {
        msg = msg.trim();       // Get rid of blank spaces
        println(msg);
        
        
        // Check wich exercise the app wants to run: 
        
        
          
         if (msg.equals("1")) {  // ------------------- Fast Turns
          doneWithExercise = false;
          println("Starting Fast Turns!\n");
          // Clear variables
          int nrOfTurns = 0;
          boolean passed = false; // nrOfTurns can increase if true
          boolean haveStarted = false;
          while (true) {
            redCount = 0;
            greenCountLeft = 0;
            greenCountRight = 0;
            sumGoalXLeft = 0;
            sumGoalYRight = 0;
            sumGoalYLeft = 0;
            sumGoalXRight = 0;
            meanX = 0;
            meanY = 0;
            sumX = 0;
            sumY = 0;
            meanGoalXLeft = 0;
            meanGoalXRight = 0;
            meanGoalYLeft = 0;
            meanGoalXRight = 0;
           
            // Get video and depth values from Kinect
            img = kinect2.getRegisteredImage();
            depthRaw = kinect2.getRawDepth();
            img.loadPixels();
            
            // Step through every pixel and see if the redness > threshold or greeness > threshold
            for (int i = 0; i < 512*424; i++) {
              if (calculateGreenValue(img.pixels[i]) >= greenThreshold) { 
                // Save values
                if (i%512 > 256) {
                  sumGoalXRight += i%512;
                  sumGoalYRight += i/512;
                  greenCountRight++;
                } else {
                  sumGoalXLeft += i%512;
                  sumGoalYLeft += i/512;
                  greenCountLeft++;
                }
                
                // Set pixel to green
                img.pixels[i] = color(0, 255, 0);
              }
              if (calculateRedValue(img.pixels[i]) >= redThreshold) { 
                // Save values
                sumX += i%512;
                sumY += i/512;
                      
                redCount++;
                // Set pixel to red
                img.pixels[i] = color(255, 0, 0);
              }
            }
            
            // get depth
            if (redCount > 0) {
              meanY = sumY/redCount;
              meanX = sumX/redCount;
              
              meanDepth = depthRaw[round(floor(meanY))*512 + round(meanX)];
            }
            if (greenCountLeft > 0) {
              meanGoalXLeft = sumGoalXLeft/greenCountLeft;
              meanGoalYLeft = sumGoalYLeft/greenCountLeft;
              
              goalDepthLeft = depthRaw[round(floor(meanGoalYLeft))*512 + round(meanGoalXLeft)];
            }
            if (greenCountRight > 0) {
              meanGoalXRight = sumGoalXRight/greenCountRight;
              meanGoalYRight = sumGoalYRight/greenCountRight;
              
              goalDepthRight = depthRaw[round(floor(meanGoalYRight))*512 + round(meanGoalXRight)];
            }
            
            
            
            
            // Ovning
            if (greenCountLeft > 0 && greenCountRight > 0 && redCount > 0) {
              redXPos = calculateXPosition(meanX, meanDepth);
              leftXPos = calculateXPosition(meanGoalXLeft, goalDepthLeft);
              rightXPos = calculateXPosition(meanGoalXRight, goalDepthRight);
              
              if (nrOfTurns == 0 && redXPos < rightXPos && redXPos > leftXPos && haveStarted == false) {
                tStart = millis();
                haveStarted = true;
                shouldPassRight = true;
              }
              if (haveStarted == true && shouldPassRight == true && redXPos > rightXPos) {
                nrOfTurns++;
                tid = (millis() - tStart)/1000;
                tStart = millis();
                shouldPassRight = false;
                print("send data\n");
                String msgToSend = Float.toString(tid);
                c.write(msgToSend);
              }
               else if (haveStarted == true && shouldPassRight == false && redXPos < leftXPos) {
                  nrOfTurns++;
                  tid = (millis() - tStart)/1000;
                  tStart = millis();
                  shouldPassRight = true;
                  print("send data\n");
                  String msgToSend = Float.toString(tid);
                  c.write(msgToSend);
              }
            }
           
            img.updatePixels();
         
  
            if (nrOfTurns == 10) {
              break;
            }
          }        
        } else if (msg.equals("stop")) { // ------------ Stop server and end program
          println("Server ended.");
          println("Program ended.");
          myServer.stop();
          System.exit(0);
        }
      }
    }
  }
}