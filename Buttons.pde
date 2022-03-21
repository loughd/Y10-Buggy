import controlP5.*;
import processing.net.*; //importing necessary libraries

Client myClient; //declaring our client class
String data = "null";
int distance = 0; //declaring our variables
ControlP5 p5; //declaring our p5 class
Button GoButton;
Button StopButton; //declaring our buttons
PFont f;
ControlFont font;

void setup() {
  size(500,400); //output window size (length, width)
  noStroke();
  background(140, 130, 260);
  f = createFont("Calibri", 30, true);
  font = new ControlFont(f);
  p5 = new ControlP5(this); //creates p5 class
  Button GoButton = p5.addButton("Go");
  Button StopButton = p5.addButton("Stop"); //creates our buttons
  GoButton.setPosition(width * 0.3,150).setSize(200,50).setFont(font);
  StopButton.setPosition(width * 0.3,300).setSize(200,50).setFont(font); //sets our buttons locations and sizes
  myClient = new Client(this, "192.168.4.1", 80); //connects our client to the specified ip address in the wifi access point this machine is connected to
  myClient.write("I am a new client"); //required as the client must speak to the server first to initialise the connection
}

void draw() {
  background(140, 130, 255);
  textFont(f);
  textAlign(CENTER);
  drawType(width * 0.5);
  if (myClient.available() > 0) { //only reads in data while the server has data to give out
    distance = myClient.read(); //reads in the distance measured by the ultrasonic sensor
    if (distance <= 15) {
       print("Stopping for object at distance ");
       println(distance); //outputs the distance to the console if the object is close enough to require it to stop
    }
  }
}

public void controlEvent(ControlEvent ev) {
  myClient.write(ev.getController().getName()); //writes to the server the name of the button that was just pressed
  
  /*if (ev.getController().getName() == "Go") {
     distance = 20;
   }
   else if (ev.getController().getName() == "Stop") {
     distance = 10;
   }*/
}

void drawType(float x) {
  text("Free space ahead: ", x, 30);
  text(distance, x, 55);
  fill(51);
  
  if (distance < 20) {
    text("Obstacle detected at: ", x, 80);
    text(distance, x + 140 , 80);
    fill(51);
  } else {
    text("No obstacle detected.", x, 80);
  }
}
