/*
 * Listening OSC
 * Stejara Dinulescu
 * Program listens for OSC messages from motion capture webcam input and
 * uses it to drive sound
 */


/* CITATIONS: 
 * oscP5sendreceive by andreas schlegel
 * example shows how to send and receive osc messages.
 * oscP5 website at http://www.sojamo.de/oscP5
 *
 * sound example taken from processing.org 
 * examples show how to create sine waves, envelopes, tri oscillator
 * sound website at https://processing.org/tutorials/sound/
 */

import oscP5.*;
import netP5.*;
import processing.sound.*;

OscP5 oscP5;

int DEST_PORT = 8888;
String DEST_HOST = "127.0.0.1";

//Values for Osc

float angle = 0; 
float xAccel = 0;
float prevXAccel = 0;
float yAccel = 0;
float prevYAccel = 0;
float der1x =0;
float prevDer1x = 0;
float der1y = 0; 
float prevDer1y = 0;
float der2x =0;
float prevDer2x = 0;
float der2y = 0; 
float prevDer2y = 0;
float dist = 0;

float xPos = 0;
float prevXPos = 0;
float yPos = 0;
float prevYPos = 0;
float maxMotion = 0;
float prevMaxMotion = 0;


//Tri oscillator and envelope -> max motion value is passed in 
TriOsc tri;
TriOsc tri2;
Env env;
Env env2;

int note = 0;

SoundFile file;

void setup() {
  fullScreen();
  frameRate(20);
  //background(50, 50, 150);
  background(20);
  /* start oscP5, listening for incoming messages at destination port */
  oscP5 = new OscP5(this, DEST_PORT);
  
   //set up tri oscillator and envelope
  tri = new TriOsc(this);
  tri2 = new TriOsc(this);
  env = new Env(this);
  env2 = new Env(this);

  /* myRemoteLocation is a NetAddress. a NetAddress takes 2 parameters,
   * an ip address and a port number. myRemoteLocation is used as parameter in
   * oscP5.send() when sending osc packets to another computer, device, 
   * application. usage see below. for testing purposes the listening port
   * and the port of the remote location address are the same, hence you will
   * send messages back to this sketch.
   */
   
   file = new SoundFile(this, "music.wav");
   file.amp(0.7);
   file.play();
   file.loop();
}

void oscEvent(OscMessage msg) { //checks for incoming osc messages
  String addr = msg.addrPattern();
  //set previous values?
  prevXAccel = xAccel;
  prevYAccel = yAccel;
  prevDer1x = der1x;
  prevDer1y = der1y;
  prevDer2x = der2x;
  prevDer2y = der2y;

  prevXPos = xPos;
  prevYPos = yPos;
  prevMaxMotion = maxMotion;
  
  //for some reason this stops sometimes because it receives something null?
  //if (frameCount % 5 == 0 ) {
    if ( addr.contains("mocap/points") ) {
      //set current values
      xAccel = msg.get(0).floatValue();
      yAccel = msg.get(1).floatValue();
    } else if ( addr.contains("mocap/derivative") ) {
      der1x = msg.get(0).floatValue();
      der1y = msg.get(1).floatValue();
    } else if ( addr.contains("mocap/square") ) {
      maxMotion = msg.get(0).floatValue();
      xPos = msg.get(1).floatValue();
      yPos = msg.get(2).floatValue();
    }
  //}
}

void handleTri() {
  //map values
  float mappedX = map(xPos, 0, 640, 0, 1);
  float mappedY = map(yPos, 0, 480, 0, 1);
  float midiX = map(xPos, 0, 640, 49, 84);
  float midiY = map(yPos, 0, 480, 49, 84);
  
  float mappedMotion = map(maxMotion, 0, 90000, 0, 0.007);
  
  //sustainLevel = mappedY; //square y values change the sustainTime
  //attackTime = sqrt(mappedX*mappedX + mappedY*mappedY); //distance formula between square x and y values changes the sustain time of tri oscillator

  //println(maxX);
  //"melody" tri oscillator that plays notes based on the max square motion that is passed in
  tri.play( midiToFreq( (int)midiX ), mappedMotion  ); //square X values change amplitude
  env.play( tri, 0.5, 1, 1, 1 );
  tri2.play( midiToFreq( (int)midiX + 7 ), mappedMotion ); //square Y values change amplitude
  env2.play( tri2, 0.5, 1, 1, 1 );
}

float midiToFreq(int note) { //translates to notes (from processing documentation)
  return (pow(2, ((note-69)/12.0)))*440;
}

void handleViz() {
  //mapping incoming blob values
  float xA = map(xAccel, 0, 1, 0, 10);
  float yA = map(yAccel, 0, 1, 0, 10);
  float d1x = map(der1x, 0, 1000, 0, width);
  float d1y = map(der1y, 0, 1000, 0, height);
  float xP = map(xPos, 0, 640, 100, width);
  float yP = map(yPos, 0, 480, 100, height);
  
  dist = sqrt(der1y*der1y - der1x*der1x);
  
  float colRed = map(xAccel, 0, 1, 0, 255);
  float colGreen = map(yAccel, 0, 1, 0, 255);
  float colBlue = map(dist, 0, 80, 0, 255);
  float colAlpha = map(maxMotion, 0, 300000, 0, 255);
  println(colRed + " " + colGreen + " " + colBlue);
  //println("maxM: " + maxMotion + " colAlpha: " + colAlpha);
  
  //pushMatrix();
  translate(xP, yP);
  rotate(angle);

  stroke(colRed, colGreen - 50, colBlue, colAlpha);
  noFill();
  bezier(0, 0, xP, yP, xA, yA, d1x, d1y);
  noStroke();
  fill(255, 20);
  ellipse(xA, yA, d1x, d1y);
  //popMatrix();
  
  //println(xP + " " + yP + " " + xA + " " + yA + " " + d1x + " " + d1y);

  angle += der1x;
}

void draw() {
  handleTri();
  handleViz();
}
