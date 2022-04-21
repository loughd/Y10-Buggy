#include <MadgwickAHRS.h> //library for changing degress per second value to degrees
#include <Arduino_LSM6DS3.h> //IMU library
#include <WiFiNINA.h> //WiFi access point library
char ssid[] = "teamY10";
char pass[] = "2E10Project";
int connection = 0;

const int LDIG1 = 2;
const int LDIG2 = 3;
const int RDIG1 = 5;
const int RDIG2 = 6; //declare motor pins
int nearRoll;

double kp = 8; //proportional constant for PID
double ki = 0.0001; //integral constant for PID
double kd = 0.01; //derivative constant for PID

int currentTime = 0;
int elapsedTime = 0;
int previousTime = 0; //time variables for integral and derivative parts of PID
int error = 0; //proportional error of PID
int setPoint = 73; //desired degree value for balance with PID
int cumError = 0; //integral error of PID
int rateError = 0; //derivative error of PID
int lastError = 0; //last error value for computing derivative
int sped = 0; //motor speed
int uploadTimer = 0; //timer for uploading roll value to processing
int prevUploadTime = 0; 

WiFiServer server(80); //creating server object

Madgwick filter; 
unsigned long millisPerReading, millisPrevious;
float accelScale, gyroScale; //all variables needed for madgwick to convert properly

void Backward(){
  analogWrite(LDIG1, 0);
  analogWrite(LDIG2, sped);
  analogWrite(RDIG1, sped);
  analogWrite(RDIG2, 0); //sends PWM speed value to the motors to make them go backward
}

void Forward(){
  analogWrite(LDIG1, abs(sped));
  analogWrite(LDIG2, 0);
  analogWrite(RDIG1, 0);
  analogWrite(RDIG2, abs(sped)); //sends the PWM speed value to the motors to make them go forward
}

void setup() {
  Serial.begin(9600); //starts serial port for debugging
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
  pinMode(RANA, OUTPUT); //declares pins
 
  // start the IMU and filter
  IMU.begin();
  filter.begin(1);

  // initialize variables to pace updates to correct rate
  millisPerReading = 16;
  millisPrevious = millis();
}

void loop() {
  
  float aX, aY, aZ; //raw accelerometer values
  float gX, gY, gZ; //raw gyroscope values
  float ax, ay, az; //converted accelerometer values
  float gx, gy, gz; //converted gyroscope values
  float roll, pitch, heading; //orientation of buggy in degrees
  unsigned long millisNow; //timer for updating madgwick filter

  // check if it's time to read data and update the filter
  millisNow = millis();
  
  if (millisNow - millisPrevious >= 12) {
 
  // read raw data from IMU
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

    // sets the heading, pitch and roll
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();
    //prints heading, pitch and roll for debugging
    /*Serial.print("Orientation: ");
    Serial.print(heading);
    Serial.print(" ");
    Serial.print(pitch);
    Serial.print(" ");
    Serial.println(roll);*/
    nearRoll = round(roll); //converts this value to an int so it can be uploaded
    //Serial.println(nearRoll); //checking this value for debugging
    sped = computePID(roll); //sets speed PWM from the PID function

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
      server.write(nearRoll); //sends the roll in degrees as an int to processing
    }
    prevUploadTime = prevUploadTime + uploadTimer; //sets value for upload timer again
  }
}

float convertRawAcceleration(float aRaw)
  // -2 g maps to a raw value of -32768
  // +2 g maps to a raw value of 32767
  float a = (aRaw * 2.0) / 32768.0;
  return a;
}

float convertRawGyro(float gRaw) {
  // -500 maps to a raw value of -32768
  // +500 maps to a raw value of 32767
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
