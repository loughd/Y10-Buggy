#include <WiFiNINA.h>
#include <SPI.h>


char ssid[] = "TeamY10";        
char pass[] = "2E10Project";

WiFiServer server(80);

const int US_TRIG = 8;
const int US_ECHO = 9;

const int LEYE = 10;
const int REYE = 11;

const int LANA = 16;
const int RANA = 17;

const int LDIG1 = 2;
const int LDIG2 = 3;
const int RDIG1 = 5;
const int RDIG2 = 6;

char x = 'a';

bool L_EYE() {
  if (digitalRead(LEYE) == HIGH) return true;
  else return false;
}

bool R_EYE() {
  if (digitalRead(REYE) == HIGH) return true;
  else return false;
}

void Stop(){
  digitalWrite(LDIG1, LOW);
  digitalWrite(LDIG2, LOW);
  digitalWrite(RDIG1, LOW);
  digitalWrite(RDIG2, LOW);
}

void Right(){
  analogWrite(LANA, 169);
  analogWrite(RANA, 69);
  
  digitalWrite(LDIG1, LOW);
  digitalWrite(LDIG2, HIGH);
  digitalWrite(RDIG1, HIGH);
  digitalWrite(RDIG2, LOW);  
}

void Left(){
  analogWrite(LANA, 69);
  analogWrite(RANA, 169);
  
  digitalWrite(LDIG1, LOW);
  digitalWrite(LDIG2, HIGH);
  digitalWrite(RDIG1, HIGH);
  digitalWrite(RDIG2, LOW);  
}

void Forward(int x){
  analogWrite(LANA, 100+(10*x));
  analogWrite(RANA, 100+(10*x));
  
  digitalWrite(LDIG1, LOW);
  digitalWrite(LDIG2, HIGH);
  digitalWrite(RDIG1, HIGH);
  digitalWrite(RDIG2, LOW);  
}

void Backward(){
  analogWrite(LANA, 100);
  analogWrite(RANA, 100);
  
  digitalWrite(LDIG1, HIGH);
  digitalWrite(LDIG2, LOW);
  digitalWrite(RDIG1, LOW);
  digitalWrite(RDIG2, HIGH);
}



void setup() {
  Serial.begin( 9600 );
  pinMode(US_TRIG, OUTPUT);
  pinMode(US_ECHO, INPUT);
  
  pinMode(LDIG1, OUTPUT);
  pinMode(LDIG2, OUTPUT);
  pinMode(RDIG1, OUTPUT);
  pinMode(RDIG2, OUTPUT);
  pinMode(LANA, OUTPUT);
  pinMode(RANA, OUTPUT);
  
  WiFi.beginAP(ssid, pass);
  delay(10000);
  server.begin();
  IPAddress ip = WiFi.localIP();
  Serial.println(ip);
  
  //attachInterrupt( digitalPinToInterrupt(LEYE), ir_isrL, FALLING);
  pinMode(LEYE, INPUT);
  //attachInterrupt( digitalPinToInterrupt(REYE), ir_isrR, FALLING);
  pinMode(REYE, INPUT);
}
//void ir_isrL(){
 
  //analogWrite(LANA, 0);
//}

//void ir_isrR(){

  //analogWrite(RANA, 0);
//}


void loop(){
  //int RIR = digitalRead(REYE);
  //int LIR = digitalRead(LEYE); 
  //int RSPEED = 150/(1+exp(-3*RIR));
  //int LSPEED = 150/(1+exp(-3*LIR));
  
  
  WiFiClient client = server.available();
  if(client.connected()){
    x = client.read();
  }
  
  delay(100);   
  if (x == 'b'){
 
  int distance;
  long duration;
 
  digitalWrite( US_TRIG, LOW );
  delayMicroseconds(10);
  digitalWrite( US_TRIG, HIGH );
  delayMicroseconds( 50 );
  digitalWrite( US_TRIG, LOW );

  duration = pulseIn( US_ECHO, HIGH, 5600);

  distance = duration/58;

  WiFiClient client = server.available();
  if(client)
  if(client.connected()){
    client.println(distance);
  }
  
  if (L_EYE() && R_EYE()) Forward(distance-15);
  
  else if (R_EYE() && !L_EYE()) Left();
  
  else if (!R_EYE() && L_EYE()) Right();
  
  else Stop();  

  Serial.println(distance);

}
  else if (x == 'a') Stop();

 }
    
