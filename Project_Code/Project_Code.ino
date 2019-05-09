#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <Wire.h>
#include <ESP8266HTTPClient.h>
#include <time.h>

#define PI 3.1415926535897932384626433832795

//PIN LAYOUT
//EN1 --> D3
//IN1 --> RX
//IN2 --> TX
//IN3 --> D0
//IN4 --> D7
//EN2 --> D4

//ENC1 -->D1
//ENC2 --> D6

// D8 IS BAD. DO NOT USE!

//Pin Definitions
#define ENCODER1 5 //[D1]
#define ENCODER2 12 //[D6]

#define EN1  0  //[D3]
#define IN1  3  //[RX]
#define IN2  1 //[TX]

#define EN2  2 //[D4]
#define IN3  16 //[D0]
#define IN4  13 //[D7] 

#define PI 3.1415926535897932384626433832795
#define radius 0.032955

//requirements to run robot
float count1 = 0;
float count2 = 0;
float rev1 = 0;
float rev2 = 0;
float v1, v2, v_actual;
float ts = 0;
float tremove = 0;

//requirements to run ackremenas steering
#define h 0.5
#define k1 0.5
#define k2 0.5
#define l 0.157

//initial conditions
double x = 0;
double y = 0;
double theta = 0;
double w = 0;
double v = 0;
double xh = 0;
double yh = 0;
double e1 = 0;
double e2 = 0;
double vl = 0;
double vr = 0;
double pwm_l = 0;
double pwm_r = 0;


//final point
double xr = 2;
double yr = 2;


// Connection credentials
const char* WIFI_NAME = "Daedalus";
const char* WIFI_PASS = "flightoficarus";
const char* host = "192.168.43.177";

// Select SDA and SCL pins for I2C communication 
const uint8_t scl = 4; //[D2]
const uint8_t sda = 14; //[D5]

// MPU6050 Slave Device Address
const uint8_t MPU6050SlaveAddress = 0x68;

// sensitivity scale factor respective to full scale setting provided in datasheet 
const uint16_t AccelScaleFactor = 16384;
const uint16_t GyroScaleFactor = 131;

// MPU6050 few configuration register addresses
const uint8_t MPU6050_REGISTER_SMPLRT_DIV   =  0x19;
const uint8_t MPU6050_REGISTER_USER_CTRL    =  0x6A;
const uint8_t MPU6050_REGISTER_PWR_MGMT_1   =  0x6B;
const uint8_t MPU6050_REGISTER_PWR_MGMT_2   =  0x6C;
const uint8_t MPU6050_REGISTER_CONFIG       =  0x1A;
const uint8_t MPU6050_REGISTER_GYRO_CONFIG  =  0x1B;
const uint8_t MPU6050_REGISTER_ACCEL_CONFIG =  0x1C;
const uint8_t MPU6050_REGISTER_FIFO_EN      =  0x23;
const uint8_t MPU6050_REGISTER_INT_ENABLE   =  0x38;
const uint8_t MPU6050_REGISTER_ACCEL_XOUT_H =  0x3B;
const uint8_t MPU6050_REGISTER_SIGNAL_PATH_RESET  = 0x68;

int16_t AccelX, AccelY, AccelZ, Temperature, GyroX, GyroY, GyroZ;
double Ax, Ay, Az, T, Gx, Gy, Gz;

void High_Callback() {
  count1 += 1;
}
void Low_Callback() {
  count2 += 1;
}

void setup() {
  
  Serial.begin(115200);
  Serial.print("STARTED ROBOT");
  
  //Connect to WiFi
  Connect2Wifi();
  
  // Initialize MPU
  Wire.begin(sda, scl);
  MPU6050_Init();

  //Defining PIN directions
  pinMode(EN1, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(EN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENCODER1, INPUT);
  pinMode(ENCODER2, INPUT);
  
  delay(1000);

  analogWrite(EN1, 1024*0.9);
  analogWrite(EN2, 1024);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  
  attachInterrupt(digitalPinToInterrupt(ENCODER1), High_Callback, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER2), Low_Callback, RISING);
}

void loop(){

  if(WiFi.status() != WL_CONNECTED){
    Connect2Wifi();
  }

  //obtaining the timestep between loops
  ts = (millis() - tremove)/1000;
  tremove = millis();
  
  //obtain the speed of wheels in revolutions/sec using the count values from the interrupt functions 
  rev1 = count1/(20*ts);
  rev2 = count2/(20*ts);

  //change these speeds into m/s for each wheel
  v1 = rev1*2*PI*radius;
  v2 = rev2*2*PI*radius;

  //obtain actual robot speed
  v_actual = (v1 + v2)/2;

  
  //reset counter values in preperation for next v measurement
  count1 = 0;
  count2 = 0;
  
  //obtain accelerometer and gyroscope values 
  Read_RawValue(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_XOUT_H);
  //divide each with their sensitivity scale factor
  Ax = (double)AccelX*9.81/AccelScaleFactor;
  Ay = (double)AccelY*9.81/AccelScaleFactor;
  Az = (double)AccelZ*9.81/AccelScaleFactor;
  T =  (double)Temperature/340+36.53;         //temperature formula
  Gx = (double)GyroX/GyroScaleFactor;
  Gy = (double)GyroY/GyroScaleFactor;
  Gz = (double)GyroZ/GyroScaleFactor;

  
  //implementing ackermans seering
  x = x + ts*v_actual*cos(theta);
  y = y + ts*v_actual*sin(theta);
  theta = theta + ts*w;

  xh = x + h*cos(theta);
  yh = y + h*sin(theta);

  e1 = xh - xr;
  e2 = yh - yr;

  v = (k1*cos(theta) + k2*sin(theta))*e1;
  w = ((-k1*sin(theta)/h) + (-k2*cos(theta)/h))*e2;


  //obtain required speed for each motor
  vl = v + (l*w/2);
  vr = v - (l*w/2);

  
  //scaling the voltage values between 600 - 1024
  pwm_l = ((1024 - 600) * (vl/0.7)) + 600;
  pwm_r = ((1024 - 600) * (vr/0.7)) + 600; 

  //apply voltages to wheels
  if (pwm_l > 0){

    //must switch off mototrs before switching direction
    if (flag_p_l != 1){
      analogWrite(EN1, 0);  
      flag_p_l = 1;
      delay(100);
    }
    
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);

     
  }else{

    //must switch off mototrs before switching direction
    if (flag_p_L == 1){
      analogWrite(EN1, 0);  
      flag_p_r = 0;
      delay(100);
    }

    
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    pwm_l = abs(pwm_l);

  }

  if (pwm_r > 0){

    //must switch off mototrs before switching direction
    if (flag_p_r != 1){
      analogWrite(EN2, 0);  
      flag_p_r = 1;
      delay(100);
    }

    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  
     
  }else{

    //must switch off mototrs before switching direction
    if (flag_p_r == 1){
      analogWrite(EN2, 0);  
      flag_p_r = 0;
      delay(100);
    }

    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    pwm_r = abs(pwm_r);

  }
  
  //apply required speed for each motor 
  //note: the 0.9 factor is just used for calibration since the motors aren't identical
  analogWrite(EN1, pwm_l*0.9);  
  analogWrite(EN2, pwm_r);

  SendData(x, y, vl, vr, pwm_l, pwm_r, ts, count1, count2, rev1, rev2, v1, v2, v_actual, Ax, Ay, Az, T, Gx, Gy, Gz);
  delay(100);
}

// --------------------  ACCELEROMETER FUNCTIONS --------------------
void I2C_Write(uint8_t deviceAddress, uint8_t regAddress, uint8_t data){
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.write(data);
  Wire.endTransmission();
}

// read all 14 register
void Read_RawValue(uint8_t deviceAddress, uint8_t regAddress){
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, (uint8_t)14);
  AccelX = (((int16_t)Wire.read()<<8) | Wire.read());
  AccelY = (((int16_t)Wire.read()<<8) | Wire.read());
  AccelZ = (((int16_t)Wire.read()<<8) | Wire.read());
  Temperature = (((int16_t)Wire.read()<<8) | Wire.read());
  GyroX = (((int16_t)Wire.read()<<8) | Wire.read());
  GyroY = (((int16_t)Wire.read()<<8) | Wire.read());
  GyroZ = (((int16_t)Wire.read()<<8) | Wire.read());
}

//configure MPU6050
void MPU6050_Init(){
  delay(150);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SMPLRT_DIV, 0x07);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_1, 0x01);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_2, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_CONFIG, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_GYRO_CONFIG, 0x00);//set +/-250 degree/second full scale
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_CONFIG, 0x00);// set +/- 2g full scale
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_FIFO_EN, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_INT_ENABLE, 0x01);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SIGNAL_PATH_RESET, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_USER_CTRL, 0x00);
}
// --------------------------------------------------------------------------------

void Connect2Wifi(){
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  WiFi.begin(WIFI_NAME, WIFI_PASS);
  delay(4000);
}

void SendData(double X,double Y, double VL,double VR,double PWM_L,double PWM_R,double TS, double E1, double E2, double REV1, double REV2, double V1, double V2, double V_ACTUAL, double AX, double AY, double AZ, double T, double OX, double OY, double OZ){
  HTTPClient http;
  http.begin("http://" + String(host) + ":5000/data");
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  http.POST(
    "x=" + String(X) + "&"
    "y=" + String(Y) + "&"
    "vl=" + String(VL) + "&"
    "vr=" + String(VR) + "&"
    "pwm_l=" + String(PWM_L) + "&"
    "pwm_r=" + String(PWM_R) + "&"
    "TS=" + String(TS) + "&"
    "E1=" + String(E1) + "&"
    "E2=" + String(E2) + "&"
    "REV1=" + String(REV1) + "&"
    "REV2=" + String(REV2) + "&"
    "V1=" + String(V1) + "&"
    "V2=" + String(V2) + "&"
    "V Actual=" + String(V_ACTUAL) + "&"
    "AX=" + String(AX) + "&"
    "AY=" + String(AY) + "&"
    "AZ=" + String(AZ) + "&"
    "T=" + String(T) + "&"
    "OX=" + String(OX) + "&"
    "OY=" + String(OY) + "&"
    "OZ=" + String(OZ)
    );
  http.writeToStream(&Serial);
  http.end();
} 
