// Based on example: https://forum.processing.org/one/topic/how-to-implement-a-interactive-3d-camera-on-a-2d-world-map.html

// left mouse button + mouse drag = rotate
// right mouse button + mouse drag = translate
// mouse scroll wheel = scale


 
import processing.opengl.*;

String baseaddress="/oscplayer/";
 
import oscP5.*;
import netP5.*;

OscP5 oscP5;
NetAddress myRemoteLocation;
 
PVector position = new PVector(450, 450);
PVector movement = new PVector();
PVector rotation = new PVector();
PVector velocity = new PVector();
float rotationSpeed = 0.035;
float movementSpeed = 0.05;
float scaleSpeed = 0.15; //0.25;
float fScale = 2;
PVector axisScale=new PVector(1,-1,1);

ObjectPoint objectPoints[]=new ObjectPoint[100];

PVector measurementPointA;
PVector measurementPointB;

class ObjectPoint { 
  float x,y,z;
  int lastUpdated=0; 
  ObjectPoint (){
    x=0;
    y=0;
    z=0;
  }
  ObjectPoint (float px, float py, float pz) {  
    x=px;
    y=py;
    z=pz;
  } 
  void setPosition(float px, float py, float pz) {  
    x=px;
    y=py;
    z=pz;
    lastUpdated=millis();
  }
  
  PVector getPosition() {
    return new PVector(x,y,z);
  }
  
}

class TrackedObject { 
  float x,y,z;
  float u,v,w;
  TrackedObject (){
    x=0;
    y=0;
    z=0;
    u=0;
    v=0;
    w=0;
  }
  TrackedObject (float px, float py, float pz, float pu,float pv, float pw) {  
    x=px;
    y=py;
    z=pz;
    u=pu;
    v=pv;
    w=pw;
  } 
  
}
 
void setup() {
  size(900, 900, OPENGL);
  stroke(255, 255, 0);
  strokeWeight(2);
  fill(150, 200, 250);
  
  
  oscP5 = new OscP5(this,12000); //Port for input OSC Messages
  myRemoteLocation = new NetAddress("127.0.0.1",7000); //connect to remote, IP, Port
  
  
}
 
void draw() {
  if (mousePressed) {
    if (mouseButton==LEFT) velocity.add( (pmouseY-mouseY) * 0.01, 0, (mouseX-pmouseX) * 0.01);
    if (mouseButton==RIGHT) movement.add( (mouseX-pmouseX) * movementSpeed, (mouseY-pmouseY) * movementSpeed,0);
  }
  velocity.mult(0.95);
  rotation.add(velocity);
  movement.mult(0.95);
  position.add(movement);
 
  background(0);
  lights();
 
  pushMatrix();
  translate(position.x, position.y, position.z);
  rotateX(rotation.x*rotationSpeed);
  rotateZ(rotation.z*rotationSpeed*-1);
  scale(fScale);
  
  drawOrigin(100);
  drawObjectPoints(100);
  
  popMatrix();
  
  textSize(30);
  fill(#ffffff);
  if (measurementPointA != null) {
    
    text("A="+measurementPointA.x+", "+measurementPointA.y+", "+measurementPointA.z, 20, 30*1, -10);
  }
  if (measurementPointB != null) {
    text("B="+measurementPointB.x+", "+measurementPointB.y+", "+measurementPointB.z, 20, 30*2, -10);
  }
  if (measurementPointA != null && measurementPointB != null) {
     float measurementDist=measurementPointA.dist(measurementPointB);
      text("D="+measurementDist, 20, 30*3, -10);
  }
  
  ObjectPoint objectPointA=objectPoints[0];
  ObjectPoint objectPointB=objectPoints[1];
  
  if (objectPointA != null && objectPointB != null) {
      if ((millis()-objectPointA.lastUpdated < 1000) && (millis()-objectPointB.lastUpdated < 1000) ) {
        float measurementDist=objectPointA.getPosition().dist(objectPointB.getPosition());
        text("Dlive="+measurementDist, 20, 30*5, -10);
      }
  }
}
 
void mouseWheel(MouseEvent event) {
  float delta = event.getCount();
  
  fScale -= delta * scaleSpeed;
  fScale = max(0.5, fScale);
}



void oscEvent( OscMessage m ) {
  //print( "Received an osc message" );
  //print( ", address pattern: " + m.addrPattern( ) );
  //print( ", typetag: " + m.typetag( ) );
  if(m.addrPattern( ).equals("/objectPoint") && m.typetag().equals("ifff")) {
    /* transfer receivd values to local variables */
    int id=m.get(0).intValue();
    float x0 = m.get(1).floatValue();
    float y0 = m.get(2).floatValue();
    float z0 = m.get(3).floatValue();
    //print("id="+str(id)+" xyz="+str(x0)+","+str(y0)+","+str(z0));
    if (objectPoints[id]==null){
      objectPoints[id]=new ObjectPoint();
    }
    
    objectPoints[id].setPosition(x0,y0,z0);
  }
  println();
}

void drawOrigin(float scale)
{
  stroke(#ffffff);
  fill(#555555);
  box(scale/10);
  stroke(#ff0000);
  line(-scale, 0, 0, scale, 0, 0);
  line(scale, 0, 0, scale-scale/10, scale/10, 0);
  line(scale, 0, 0, scale-scale/10, -scale/10, 0);
  
  stroke(#00ff00);
  line(0, (-scale)*axisScale.y, 0, 0, scale*axisScale.y, 0);
  line(0, (scale)*axisScale.y, 0, scale/10, (scale-scale/10)*axisScale.y, 0);
  line(0, (scale)*axisScale.y, 0, -scale/10, (scale-scale/10)*axisScale.y, 0);
  
  stroke(#0000ff);
  line(0, 0, -scale, 0, 0, scale);
  line(0, 0, scale, scale/10, 0, scale-scale/10);
  line(0, 0, scale, -scale/10, 0, scale-scale/10);
}


void drawObjectPoints(float scale)
{
  
  
  //for (int i=0;i<100;i++)
  for (ObjectPoint objectPoint : objectPoints)
  {
    //ObjectPoint objectPoint=objectPoints[i];
    if (objectPoint != null) {
      if (millis()-objectPoint.lastUpdated < 5000) {
        pushMatrix();
        translate(objectPoint.x*axisScale.x*scale,objectPoint.y*axisScale.y*scale,objectPoint.z*axisScale.z*scale);
        stroke(#ff00ff);
        fill(#aaffaa);
        box(1);
        popMatrix();
      }
    }
  }
   
}

void keyPressed() {
  if (key=='a'){
    measurementPointA=objectPoints[0].getPosition();
  }
  if (key=='b'){
    measurementPointB=objectPoints[0].getPosition();
  }
  /*if (key == CODED) {
    if (keyCode == UP) {
      fillVal = 255;
    } else if (keyCode == DOWN) {
      fillVal = 0;
    } 
  } else {
    fillVal = 126;
  }*/
}
