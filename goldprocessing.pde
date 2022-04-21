import processing.net.*; //networking library
int roll = 0; //declares roll value
Client myClient; //initialises our client
void setup() {
  size(600, 600, P2D); //creates window size and tells it to draw a 2d object
  myClient = new Client(this, "192.168.4.1", 80); //connects to arduino
  myClient.write("I am a new client"); //tells the arduino it is connected
}

void draw() {
  if (myClient.available() > 0) { //only reads in data while the server has data to give out
    roll = myClient.read(); //reads in the angle measured 
    println(roll); //prints this value for debugging
    background(100, 120, 200); //refreshes output window each loop
    pushMatrix(); //starts object
    translate(300, 300); //moves buggy to centre of screen
    rotate(radians(roll + 90)); //orients the 2d buggy to be at the same angle as the real one
    drawArduino(); //calls the function drawing the buggy
    popMatrix(); //finishes object
  }
  
}

void drawArduino() {
  stroke(90, 90, 90);
  fill(200, 200, 200);
  rect(-50, -50, 100, 280); //creates grey box with darker grey outline
  
  stroke(0, 0, 0);
  fill(0, 0, 0);
  circle(0, 0, 140); //creates black circle for rim of wheel
  
  stroke(255, 255, 0);
  fill(255,255,0);
  circle(0,0,120); //creates yellow circle for middle of wheel
}
