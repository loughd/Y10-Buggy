#include <MadgwickAHRS.h>
#include <Arduino_LSM6DS3.h>
#include <WiFiNINA.h>
char ssid[] = "teamY10";
char pass[] = "2E10Project";
int connection = 0;
const int LANA = 16;
const int RANA = 17;

const int LDIG1 = 2;
const int LDIG2 = 3;
const int RDIG1 = 5;
const int RDIG2 = 6;
int nearRoll;

double kp = 8;
double ki = 0.0001;
double kd = 0.01;

int currentTime = 0;
int elapsedTime = 0;
int previousTime = 0;
int error = 0;
int setPoint = 73;
int cumError = 0;
int rateError = 0;
int lastError = 0;
int sped = 0;
int uploadTimer = 0;
int prevUploadTime = 0;

WiFiServer server(80); //creating server object
String input;

Madgwick filter;
unsigned long millisPerReading, millisPrevious;
float accelScale, gyroScale;

void Backward(){
  //analogWrite(LANA, );
  //analogWrite(RANA, gX);
 
  analogWrite(LDIG1, 0);
  analogWrite(LDIG2, sped);
  analogWrite(RDIG1, sped);
  analogWrite(RDIG2, 0);
}

void Forward(){
  //analogWrite(LANA, gX);
  //analogWrite(RANA, gX);
 
  analogWrite(LDIG1, abs(sped));
  analogWrite(LDIG2, 0);
  analogWrite(RDIG1, 0);
  analogWrite(RDIG2, abs(sped));
}

void setup() {
  Serial.begin(9600);
  Serial.begin(9600);
  WiFi.beginAP(ssid, pass); //starts access point
  IPAddress ip = WiFi.localIP(); //finds local ip address of access point
  //Serial.print("IP Address: "); 
  //Serial.println(ip); //prints the local ip address to the arduino serial monitor, useful when debugging
  server.begin(); //starts server
 
  pinMode(LDIG1, OUTPUT);
  pinMode(LDIG2, OUTPUT);
  pinMode(RDIG1, OUTPUT);
  pinMode(RDIG2, OUTPUT);
  pinMode(LANA, OUTPUT);
  pinMode(RANA, OUTPUT);
 
  // start the IMU and filter
  IMU.begin();
  //IMU.setGyroRate(25);
  //IMU.setAccelerometerRate(25);
  filter.begin(1);

  // Set the accelerometer range to 2 g
  //IMU.setAccelerometerRange(2);
  // Set the gyroscope range to 250 degrees/second
  //IMU.setGyroRange(250);

  // initialize variables to pace updates to correct rate
  millisPerReading = 16;
  millisPrevious = millis();
}

void loop() {
  
  float aX, aY, aZ;
  float gX, gY, gZ;
  float ax, ay, az;
  float gx, gy, gz;
  float roll, pitch, heading;
  unsigned long millisNow;

  // check if it's time to read data and update the filter
  millisNow = millis();
  
  if (millisNow - millisPrevious >= 12) {
 
  // read raw data from CurieIMU
     if(IMU.accelerationAvailable() && IMU.gyroscopeAvailable()){
  IMU.readAcceleration(aX, aY, aZ);
  IMU.readGyroscope(gX, gY, gZ);
     }
    // convert from raw data to gravity and degrees/second units
    ax = convertRawAcceleration(aX);
    ay = convertRawAcceleration(aY);
    az = convertRawAcceleration(aZ);
    gx = convertRawGyro(gX);
    gy = convertRawGyro(gY);
    gz = convertRawGyro(gZ);
 
    // update the filter, which computes orientation
    filter.updateIMU(gx, gy, gz, ax, ay, az);

    // print the heading, pitch and roll
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();
    Serial.print("Orientation: ");
    Serial.print(heading);
    Serial.print(" ");
    Serial.print(pitch);
    Serial.print(" ");
    Serial.println(roll);
    nearRoll = round(roll);
    Serial.println(nearRoll);

    sped = computePID(roll);

    if (sped > 0) Forward();
    else Backward();

    // increment previous time, so we keep proper pace
    millisPrevious = millisPrevious + millisPerReading;
  } 
  WiFiClient client = server.available(); //finds a client connected to the server with data available for the server to read
  if (client.connected()) {
  //Serial.println("Client Connected");
    connection = 1; //sets the connection variable to avoid getting trapped in a while loop
  }
  uploadTimer = millis() - prevUploadTime;
  if (uploadTimer >= 250) {
    if (connection == 1) {
      server.write(nearRoll);
    }
    prevUploadTime = prevUploadTime + uploadTimer;
  }
}

float convertRawAcceleration(float aRaw) {
  // since we are using 2 g range
  // -2 g maps to a raw value of -32768
  // +2 g maps to a raw value of 32767
 
  float a = (aRaw * 2.0) / 32768.0;
  return a;
}

float convertRawGyro(float gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767
 
  float g = (gRaw * 500.0) / 32768.0;
  return g;
}

double computePID(double inp){    
  currentTime = millis();                //get current time
  elapsedTime = (double)(currentTime - previousTime);        //compute time elapsed from previous computation
       
  error = setPoint - inp;                                // determine error
  cumError += error * elapsedTime;                // compute integral
  rateError = (error - lastError)/elapsedTime;   // compute derivative
 
  double out = (kp*error + ki*cumError + kd*rateError);                //PID output              
  Serial.println(out);
  lastError = error;                                //remember current error
  previousTime = currentTime;                        //remember current time
  return out;
}
