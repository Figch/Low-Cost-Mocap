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
float scaleSpeed = 15;
float fScale = 200;
PVector axisScale=new PVector(1,-1,1);

ObjectPoint objectPoints[]=new ObjectPoint[100];

ObjectPoint trailPoints[] = new ObjectPoint[200];
int trailPoints_pos=0;
float trailPoints_interval=100; //ms
boolean trailPoints_enabled=false;


ObjectPoint trailPoints2[] = new ObjectPoint[200];
int trailPoints2_pos=0;
float trailPoints2_interval=100; //ms
boolean trailPoints2_enabled=false;

boolean trailConnect_enabled=false;


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
  
  
  oscP5 = new OscP5(this,3002); //Port for input OSC Messages
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
  
  drawOrigin(1);
  drawObjectPoints(objectPoints,color(255,255,0), 0.05, 2000);
  if (!trailConnect_enabled){
    drawObjectPoints(trailPoints,color(0,255,100), 0.01, 0);
    drawObjectPoints(trailPoints2,color(0,100,255), 0.01, 0);
  }else{
    drawObjectPointsConnected(trailPoints,color(0,255,100), 0.01, 0);
    drawObjectPointsConnected(trailPoints2,color(0,100,255), 0.01, 0); 
  }
  
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
  
  if (trailPoints_enabled){
    text("Trail="+trailPoints_enabled, 20, 30*7, -10);
  }
  if (trailPoints2_enabled){
    text("Trail2="+trailPoints2_enabled, 20, 30*8, -10);
  }
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
    if (trailPoints_enabled){
      if (trailPoints[trailPoints_pos]==null){
        trailPoints[trailPoints_pos]=new ObjectPoint();
      }
      trailPoints[trailPoints_pos].setPosition(x0,y0,z0);
      trailPoints_pos++;
      trailPoints_pos%=trailPoints.length;
    }
    
    if (trailPoints2_enabled){
      if (trailPoints2[trailPoints2_pos]==null){
        trailPoints2[trailPoints2_pos]=new ObjectPoint();
      }
      trailPoints2[trailPoints2_pos].setPosition(x0,y0,z0);
      trailPoints2_pos++;
      trailPoints2_pos%=trailPoints2.length;
    }
      
  }
  println();
}

void drawOrigin(float scale)
{
  strokeWeight(scale/100.0);
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


void drawObjectPoints(ObjectPoint[] objectPoints, color c,float size, int timeout)
{
  //timeout=0 -> show forever
  
  noStroke();
  //for (int i=0;i<100;i++)
  for (ObjectPoint objectPoint : objectPoints)
  {
    //ObjectPoint objectPoint=objectPoints[i];
    if (objectPoint != null) {
      if (timeout==0 || millis()-objectPoint.lastUpdated < timeout) {
        pushMatrix();
        translate(objectPoint.x*axisScale.x,objectPoint.y*axisScale.y,objectPoint.z*axisScale.z);
        
        fill(c);
        box(size);
        popMatrix();
      }
    }
  }
   
}


void drawObjectPointsConnected(ObjectPoint[] objectPoints, color c,float size, int timeout)
{
  //timeout=0 -> show forever
  
  stroke(c);
  strokeWeight(size);
  //for (int i=0;i<100;i++)
  ObjectPoint lastPoint=null;
  for (ObjectPoint objectPoint : objectPoints)
  {
    //ObjectPoint objectPoint=objectPoints[i];
    if (objectPoint != null && lastPoint!=null) {
      if (timeout==0 || millis()-objectPoint.lastUpdated < timeout) {
        pushMatrix();
        //translate(objectPoint.x*axisScale.x,objectPoint.y*axisScale.y,objectPoint.z*axisScale.z);
        PVector start=new PVector(lastPoint.x*axisScale.x,lastPoint.y*axisScale.y,lastPoint.z*axisScale.z);
        PVector end=new PVector(objectPoint.x*axisScale.x,objectPoint.y*axisScale.y,objectPoint.z*axisScale.z);
        line(start.x,start.y,start.z,end.x,end.y,end.z);
        
        
        box(size);
        popMatrix();
      }
    }
    lastPoint=objectPoint;
  }
   
}


void mouseWheel(MouseEvent event) {
  float delta = event.getCount();
  
  fScale -= delta * scaleSpeed;
  fScale = max(0.005, fScale);
}


void keyPressed() {
  if (key=='a'){
    measurementPointA=objectPoints[0].getPosition();
  }
  if (key=='b'){
    measurementPointB=objectPoints[0].getPosition();
  }
  if (key=='t'){
    trailPoints_enabled=!trailPoints_enabled;
    if (trailPoints_enabled) {
      emptyArray(trailPoints);
    }
    print("trailPoints_enabled="+str(trailPoints_enabled));
  }
  if (key=='z'){
    trailPoints2_enabled=!trailPoints2_enabled;
    if (trailPoints2_enabled) {
      emptyArray(trailPoints2);
    }
    print("trailPoints2_enabled="+str(trailPoints2_enabled));
  }
  
  if (key=='-'){
    trailConnect_enabled=!trailConnect_enabled;
  }
  
}

void emptyArray(ObjectPoint[] arr){
  for (int i=0;i<arr.length;i++){
    arr[i]=null;
  }
}
