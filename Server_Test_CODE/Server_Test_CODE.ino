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


double h = 0.5;
double k1 = 0.5;
double k2 = 0.5;
double w = 0;
double v = 0;
double l = 0.157;
double x = 0;
double y = 0;
double theta = 0;
unsigned long ts = 0;
double xr = -5;
double yr = 10;
double vl = 0;
double vr = 0;
double v_measured = 0;
double xh;
double yh;
double e1;
double e2;

// Select SDA and SCL pins for I2C communication 
const uint8_t scl = 4; //[D2]
const uint8_t sda = 14; //[D5]

// Connection credentials
const char* WIFI_NAME = "Daedalus";
const char* WIFI_PASS = "flightoficarus";
const char* host = "192.168.43.177";

// Wheel Variables
float count1;
float count2;
float rev1;
float rev2;
float rev1_f;
float rev2_f;
String message;
float radius = 0.032955;

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

void setup() {
  Serial.begin(9600);

  //Connect to WiFi
//  Connect2Wifi();
  
  // Initialize MPU
//  Wire.begin(sda, scl);
//  MPU6050_Init();
//  
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

  analogWrite(EN1, LOW);
  analogWrite(EN2, LOW);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  
  attachInterrupt(digitalPinToInterrupt(ENCODER1), High_Callback, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER2), Low_Callback, RISING);
}

void loop() {
  count1 = 0;
  count2 = 0;
//  if(WiFi.status() != WL_CONNECTED){
//    Connect2Wifi();
//  }

//  Read_RawValue(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_XOUT_H);
// 
//  //divide each with their sensitivity scale factor
//  Ax = (double)AccelX*9.81/AccelScaleFactor;
//  Ay = (double)AccelY*9.81/AccelScaleFactor;
//  Az = (double)AccelZ*9.81/AccelScaleFactor;
//  T = (double)Temperature/340+36.53; //temperature formula
//  Gx = (double)GyroX/GyroScaleFactor;
//  Gy = (double)GyroY/GyroScaleFactor;
//  Gz = (double)GyroZ/GyroScaleFactor;
//
//  Serial.print("Ax: "); Serial.print(Ax);
//  Serial.print(" Ay: "); Serial.print(Ay);
//  Serial.print(" Az: "); Serial.print(Az);
//  Serial.print(" T: "); Serial.print(T);
//  Serial.print(" Gx: "); Serial.print(Gx);
//  Serial.print(" Gy: "); Serial.print(Gy);
//  Serial.print(" Gz: "); Serial.println(Gz);
  
  Ackerman_Steering(x, y, theta, xr, yr);
  vl = v + (l*w/2);
  vr = v - (l*w/2);
  Change_Speed(vl, vr);

//  rev1 = 0;
//  rev2 = 0;
//  for (int j = 1; j < 11; j++) {
//    count1 = 0;
//    count2 = 0;
//    delay(100);
//
//    rev1 += count1; //number of revolutions
//    rev2 += count2; //number of revolutions
//  }
//  rev1_f = (rev1 * 2 * PI * radius)/20;
//  rev2_f = (rev2 * 2 * PI * radius)/20;

  ts = millis() - ts;
  ts = ts/1000;
  rev1_f = count1/(20*ts);
  rev2_f = count2/(20*ts);
  
  v_measured = (rev1_f + rev2_f)/2;
  
  x = x + ts*v_measured*cos(theta);
  y = y + ts*v_measured*sin(theta);
  theta = theta + ts*w;

  Serial.println("X = " + String(x));
  Serial.println("Y = " + String(y));
  Serial.println("Theta = " + String(theta));
  Serial.println("Ts = " + String(ts));
  Serial.println("Vl = " + String(vl));
  Serial.println("Vr = " + String(vr));
  Serial.println("V_measured = " + String(v_measured));
  
//  SendData(rev1_f, rev2_f, Ax, Ay, Az, T, Gx, Gy, Gz);
}

void Change_Speed(double vl, double vr){
  analogWrite(EN1, vl);
  analogWrite(EN2, vr);
}
void Ackerman_Steering(double x, double y, double theta, double xr, double yr){
  
  xh = x + h*cos(theta);
  yh = y + h*sin(theta);
  
  e1 = xh - xr;
  e2 = yh - yr;

  v = (k1*cos(theta) + k2*sin(theta))*e1;
  w = ((-k1*sin(theta)/h) + (-k2*cos(theta)/h))*e2;
}

void High_Callback() {
  count1 += 1;
}
void Low_Callback() {
  count2 += 1;
}

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

void Connect2Wifi(){
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  WiFi.begin(WIFI_NAME, WIFI_PASS);
  delay(8000);
}

void SendData(double E1, double E2, double AX, double AY, double AZ, double T, double OX, double OY, double OZ){
  HTTPClient http;
  http.begin("http://" + String(host) + ":5000/data");
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  http.POST(
    "E1=" + String(E1) + "&"
    "E2=" + String(E2) + "&"
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
