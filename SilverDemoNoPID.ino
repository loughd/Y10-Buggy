#include <WiFiNINA.h>
char ssid[] = "teamY10";
char pass[] = "2E10Project"; //access point name and password
const int L_Motor_Fwd = 2;
const int L_Motor_Bwd = 3;
const int R_Motor_Fwd = 5;
const int R_Motor_Bwd = 6;
const int LEYE = 10;
const int REYE = 9;
const int US_TRIG = 7;
const int US_ECHO = 8; //declaring the pins that our components are connected to
int distance2 = 0;
int myTime = 0;
int total = 0;
int decide = 0;
int connection;
double spd = 130;
int spdtime = 0;
int spdtimeconst = 0;
int distcheck1 = 0;
int distcheck2 = 0;//declaring the other variables that will be used throughout our code

WiFiServer server(80); //creating server object
String input;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  WiFi.beginAP(ssid, pass); //starts access point
  IPAddress ip = WiFi.localIP(); //finds local ip address of access point
  //Serial.print("IP Address: "); 
  //Serial.println(ip); //prints the local ip address to the arduino serial monitor, useful when debugging
  server.begin(); //starts server
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(R_Motor_Fwd, OUTPUT);
  pinMode(R_Motor_Bwd, OUTPUT);
  pinMode(L_Motor_Fwd, OUTPUT);
  pinMode(L_Motor_Bwd, OUTPUT);
  pinMode(LEYE, INPUT );
  pinMode(REYE, INPUT );
  pinMode(US_TRIG, OUTPUT);
  pinMode(US_ECHO, INPUT); //declaring our pins and how they will work
  connection = 0; //variable used in determining input from client
}

bool L_EYE() {
  if (digitalRead(LEYE) == HIGH) return true;
  else return false;
}
bool R_EYE() {
  if (digitalRead(REYE) == HIGH) return true;
  else return false;
}
//functions used for checking whether or not the infrared sensors detect anything

int UltraSonic(){
  int distance;
  long duration; //declaring function variables
  digitalWrite( US_TRIG, LOW );
  delayMicroseconds(2);
  digitalWrite( US_TRIG, HIGH );
  delayMicroseconds( 10 );
  digitalWrite( US_TRIG, LOW ); //toggling the ultrasonic trigger rapidly
  duration = pulseIn( US_ECHO, HIGH ); //finds how long it takes for the ultrasonic pulse to return to the sensor
  distance = duration/58; //converts time taken to distance
  //Serial.print("Distance detected: ");
  //Serial.print( distance );
  //Serial.println(" cm");
  return distance; //makes the function give us the distance it has measured
}

void FullForward() {
  analogWrite(R_Motor_Fwd, spd);
  analogWrite(L_Motor_Fwd, spd);
  analogWrite(R_Motor_Bwd, 0);
  analogWrite(L_Motor_Bwd, 0);
}
void FullLeft() {
  analogWrite(R_Motor_Fwd, spd);
  analogWrite(L_Motor_Fwd, 0);
  analogWrite(R_Motor_Bwd, 0);
  analogWrite(L_Motor_Bwd, 0);
}
void FullRight() {
  analogWrite(R_Motor_Fwd, 0);
  analogWrite(L_Motor_Fwd, spd);
  analogWrite(R_Motor_Bwd, 0);
  analogWrite(L_Motor_Bwd, 0);
}
void Stop() {
  analogWrite(R_Motor_Fwd, 0);
  analogWrite(L_Motor_Fwd, 0);
  analogWrite(R_Motor_Bwd, 0);
  analogWrite(L_Motor_Bwd, 0);
}
void Back() {
  analogWrite(R_Motor_Fwd, 0);
  analogWrite(L_Motor_Fwd, 0);
  analogWrite(R_Motor_Bwd, spd);
  analogWrite(L_Motor_Bwd, spd);
}
//the above 5 functions are used to make the motor go forward, stop, turn and reverse

void LineFollowing() {
  myTime = millis() - total; //time variable for polling the ultrasonic sensor every 250ms without using delay();
  if (L_EYE() && !R_EYE()) FullRight(); //White line on black surface
  else if (R_EYE() && !L_EYE()) FullLeft();
  else if (R_EYE() && L_EYE()) FullForward();
  else if (!R_EYE() && !L_EYE()) FullForward();
  else Stop(); //above if else statements determine where the line is and use this information to decide if it should turn or keep going straight.
  if (myTime >250) {
    distance2 = UltraSonic(); //calls the ultrasonic function every 250 ms and finds the distance returned by it
    //Serial.println(distance2);
    total = myTime + total;  //updates the running time counter
    //Serial.println(myTime);
    if (distance2 < 10) {
      spd = 127;
      Back();
      server.write(distance2); //outputs the last distance measured by the ultrasonic sensors in our line following code.
      delay(250); //reverses if distance is less than 5 cm
    }
    else if (distance2 <13) {
      spd = 127;
      Stop();
      server.write(distance2); //outputs the last distance measured by the ultrasonic sensors in our line following code.
      delay(250); //stops if distance is less that 15 cm
    }
  }
}

void loop() {
 WiFiClient client = server.available(); //finds a client connected to the server with data available for the server to read
 if (client.connected()) {
  //Serial.println("Client Connected");
  connection = 1; //sets the connection variable to avoid getting trapped in a while loop
 }
 if (connection == 1) {
  String S = client.readString(); //takes the button name string that was pressed in processing and makes it a string here. 
  //String is only sent through once, or while button is pressed, must have a deciding variable to tell us what state we are in
  if (S == "Go") {
    digitalWrite(LED_BUILTIN, HIGH);
    decide = 1; //tells us that the last time a button was pressed, it was the "go" button and stores this for later
  }
  else if (S == "Stop") {
    digitalWrite(LED_BUILTIN, LOW);
    decide = 0; //tells us that the last time a button was pressed, it was the "stop" button and stores this for later
  } 
  while (server.available() == false) {
    //stays in this loop as long as the client has no new data to give the server
    if (decide == 1) {
      LineFollowing(); //the "go" button was the last button pressed and we can now begin our line following function.
    }
    else if (decide == 0) {
      Stop(); //the "stop" button was the last button pressed and our buggy stops.
    }
    distcheck1 = distance2;
    spdtime = millis() - spdtimeconst;
    if (spdtime >= 250) {
      if (distcheck1 > distcheck2) {
        if (spd <= 240){
          spd = spd + 10;
        }
      }
      else if (distcheck1 < distcheck2) {
        if (spd >= 71) {
          spd = spd - 10;
        }
      }
      Serial.print("speed");
      Serial.println(spd);
      spdtimeconst = spdtimeconst + 250;
      distcheck2 = distcheck1;
    }
  }
 }
}
