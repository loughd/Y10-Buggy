#include <MadgwickAHRS.h>
#include <Arduino_LSM6DS3.h>

const int LANA = 16;
const int RANA = 17;

const int LDIG1 = 2;
const int LDIG2 = 3;
const int RDIG1 = 5;
const int RDIG2 = 6;

double kp = 1;
double ki = 0.00001;
double kd = 0.1;

int currentTime = 0;
int elapsedTime = 0;
int previousTime = 0;
int error = 0;
int setPoint = 80;
int cumError = 0;
int rateError = 0;
int lastError = 0;
int sped = 0;

Madgwick filter;
unsigned long microsPerReading, microsPrevious;
float accelScale, gyroScale;

void Forward(int gX){
  analogWrite(LANA, gX);
  analogWrite(RANA, gX);
 
  digitalWrite(LDIG1, LOW);
  digitalWrite(LDIG2, HIGH);
  digitalWrite(RDIG1, HIGH);
  digitalWrite(RDIG2, LOW);
}

void Backward(int gX){
  analogWrite(LANA, gX);
  analogWrite(RANA, gX);
 
  digitalWrite(LDIG1, HIGH);
  digitalWrite(LDIG2, LOW);
  digitalWrite(RDIG1, LOW);
  digitalWrite(RDIG2, HIGH);
}

void setup() {
  Serial.begin(9600);
 
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
  filter.begin(25);

  // Set the accelerometer range to 2 g
  //IMU.setAccelerometerRange(2);
  // Set the gyroscope range to 250 degrees/second
  //IMU.setGyroRange(250);

  // initialize variables to pace updates to correct rate
  microsPerReading = 1000000 / 25;
  microsPrevious = micros();
}

void loop() {
 
  float aX, aY, aZ;
  float gX, gY, gZ;
  float ax, ay, az;
  float gx, gy, gz;
  float roll, pitch, heading;
  unsigned long microsNow;

  // check if it's time to read data and update the filter
  microsNow = micros();
 
  if (microsNow - microsPrevious >= microsPerReading) {
 
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
    //Serial.print("Orientation: ");
    //Serial.print(heading);
    //Serial.print(" ");
    //Serial.print(pitch);
    //Serial.print(" ");
    Serial.println(roll);

    sped = computePID(roll);

    if (sped > 0)
     Forward(sped);
    else Backward(abs(sped));

    // increment previous time, so we keep proper pace
    microsPrevious = microsPrevious + microsPerReading;
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
 
  float g = (gRaw * 250.0) / 32768.0;
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
