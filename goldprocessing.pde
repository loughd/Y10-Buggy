float roll = 90;

void setup() {
  size(600, 600, P2D); 
  //rectMode(CENTER);
}

void draw() {
  background(140, 130, 260);
  pushMatrix();
  translate(300, 300);
  rotate(radians(roll));
  //translate(300, 300);
  drawArduino();
  
  popMatrix();
  if (keyPressed == true) {
    if (key == 'h') roll = roll + 1.5; 
    if (key == 'g') roll = roll - 1.5;
  }
  
}

void drawArduino() {
  stroke(0, 90, 90);
  fill(200, 200, 200);
  rect(-50, -50, 100, 280);
  
  stroke(0, 0, 0);
  fill(0, 0, 0);
  circle(0, 0, 140);
  
  stroke(255, 255, 0);
  fill(255,255,0);
  circle(0,0,120);
}
