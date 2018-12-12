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

Float angle = 0.; 
Float xAccel = 0.;
Float prevXAccel = 0.;
Float yAccel = 0.;
Float prevYAccel = 0.;
Float der1x = 0.;
Float prevDer1x = 0.;
Float der1y = 0.; 
Float prevDer1y = 0.;
Float dist = 0.;

Float xPos = 0.;
Float prevXPos = 0.;
Float yPos = 0.;
Float prevYPos = 0.;
Float maxMotion = 0.;
Float prevMaxMotion = 0.;


//Tri oscillator and envelope -> max motion value is passed in 
TriOsc tri;
TriOsc tri2;
Env env;
Env env2;
//For drum sound file
Delay delay;
Reverb reverb;
//AudioIn in;
SoundFile file;
//used to map midi notes
int note = 0;

void setup() {
  fullScreen();
  frameRate(20);
  background(0xF5, 0xEF, 0xED);
  /* start oscP5, listening for incoming messages at destination port */
  oscP5 = new OscP5(this, DEST_PORT);
  
   //set up tri oscillator, envelope, delay, reverb, and audio input stream
  tri = new TriOsc(this);
  tri2 = new TriOsc(this);
  env = new Env(this);
  env2 = new Env(this);
  delay = new Delay(this);
  reverb = new Reverb(this);
  //in = new AudioIn(this, 0);
  
  //set up sound file
  file = new SoundFile(this, "finalmusic.wav");
  //in.play();

  /* myRemoteLocation is a NetAddress. a NetAddress takes 2 parameters,
   * an ip address and a port number. myRemoteLocation is used as parameter in
   * oscP5.send() when sending osc packets to another computer, device, 
   * application. usage see below. for testing purposes the listening port
   * and the port of the remote location address are the same, hence you will
   * send messages back to this sketch.
   */
   

}

void oscEvent(OscMessage msg) { //checks for incoming osc messages
  String addr = msg.addrPattern();
  //set previous values?
  prevXAccel = xAccel;
  prevYAccel = yAccel;
  prevDer1x = der1x;
  prevDer1y = der1y;
  prevXPos = xPos;
  prevYPos = yPos;
  prevMaxMotion = maxMotion;
  
  //for some reason this stops sometimes because it receives something null?
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
    //println("xp: " + xPos + " yp: " + yPos + " maxM: " + maxMotion + " der1x: " + der1x + " der1y: " + der1y + " xAccel: " + xAccel + " yAccel: " + yAccel);
}

void handleTri() {
  //map values
  Float midiX = map(xPos, 0, 640, 49, 84);
  Float mappedMotion = map(maxMotion, 0, 300000, 0, 0.05);
  //Float xA = map(xAccel, 0, 10, 0, 1);
  //Float yA = map(yAccel, 0, 10, 0, 1);
  if (midiX == null) {
    midiX = 0.;
  }
  if (mappedMotion == null) {
    mappedMotion = 0.;
  }
  
  //sustainLevel = mappedY; //square y values change the sustainTime
  //attackTime = sqrt(mappedX*mappedX + mappedY*mappedY); //distance formula between square x and y values changes the sustain time of tri oscillator

  //println(maxX);
  //"melody" tri oscillator that plays notes based on the max square motion that is passed in
  //println(mappedMotion);
  tri.play( midiToFreq( (int)(float)midiX ), mappedMotion  ); //square X values change amplitude
  println("note1: " + midiToFreq( (int)(float)midiX ));
  env.play( tri, 0.001, 0.5, xAccel, yAccel );
  tri2.play( midiToFreq( (int)(float)midiX + 7), mappedMotion ); //square Y values change amplitude
  println("note2: " + midiToFreq( (int)(float)midiX  + 7));
  env2.play( tri2, 0.001, 0.5, xAccel, yAccel );
}

Float midiToFreq(int note) { //translates to freq (from processing documentation)
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
  //println(colRed + " " + colGreen + " " + colBlue);
  //println("maxM: " + maxMotion + " colAlpha: " + colAlpha);
  
  translate(xP, yP);
  rotate(angle);

  stroke(colRed, colGreen - 50, colBlue, colAlpha);
  noFill();
  bezier(0, 0, xP, yP, xA, yA, d1x, d1y);
  noStroke();
  fill(255, 20);
  ellipse(xA, yA, d1x, d1y);
  
  //println(xP + " " + yP + " " + xA + " " + yA + " " + d1x + " " + d1y);

  angle += der1x;
}

void handleDrums() {
  Float xSound = map(xPos, 0, 640, 0, 1);
  Float ySound = map(der1y, 0, 100, 0, 1);
  //dead code prevents null exception from sound library
  if (xSound == null) {
    xSound = 0.;
  }
  if (ySound == null) {
    ySound = 0.;
  }
  if (der1x == null) {
    der1x = 0.;
  }
  if (der1y == null) {
    der1y = 0.;
  }
  
  file.amp(xSound);
  if (der1x > 50) {
    if (!file.isPlaying()) {
      file.stop();
      file.play();
      //in.play();
    } else { 
      //delay.process(in, 0.2);  
    } 
  }
 
  //println(der1x + " " + der1y);
}

void draw() {
  if (frameCount <= 2400) {
    //if at least one value is changing...
     if (xPos != prevXPos || yPos != prevYPos || maxMotion != prevMaxMotion || der1x != prevDer1x || der1y != prevDer1y) {
      handleTri();
      handleViz();
      handleDrums();
     }
  } else if (frameCount > 2400 && frameCount < 2500) {
    //5 second delay where no sound is being processed, background is black
    background(0);
  } else if (frameCount >= 2500) {
    exit();
  }
 
}
