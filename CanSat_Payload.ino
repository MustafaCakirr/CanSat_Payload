// MC///////////////////////////////////////////////////////
#include <SD.h>
#include <PID_v1.h>
#include <EEPROM.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_BMP280.h>
#include <Time.h>
#include <DS1307RTC.h>
/////////////////////////////////////////////////////////
double degree = 0;
String Telemetry;
String com[6];
int packet_count = 1;
int team_id = 6101;
int Alti_Ref = 0;
int al;
char calib_;
String telemetry_onoff = "OFF";
int hh, mm, ss;
char tether = 'T';
int Altitude;
///////////////////MILLIS/////////////////
unsigned long ilk_zaman;
unsigned long son_zaman = 0;
unsigned long prev_time = 0;
unsigned long current_time;
//////////////Camera//////////////////
#define camera 14
unsigned long cam_current;
unsigned long cam_prev = 0;
int cam_a = 0;
int eq = 0;
//////////////////////////////////////
byte milli = 0;
///////////////////SD//////////////////
File my_file;
const int chipSelect = BUILTIN_SDCARD;
///////////////////EEPROM//////////////
int Software_State = 0;
byte MSB;
byte LSB;
int E_pack;
int E_ref;
///////////////////BMP&BNO_ADRESS/////////////////////
Adafruit_BMP280 bmp;
Adafruit_BNO055 myIMU = Adafruit_BNO055(-1, 0x28);
float gyro_x;
float gyro_z;
float gyro_y;
float acc_x;
float acc_y;
float acc_z;
float mag_x;
float mag_y;
float mag_z;
float temp;
float Alti;
///////////////////BN0055////////////////////////
imu::Vector<3> euler;
imu::Vector<3> acc;
imu::Vector<3> gyro;
imu::Vector<3> mag;
imu::Vector<3> lineer;
////////////////////BUZZER/////////////////////////
int buzzer_pin = 6;
///////////////VOLTAGE DIVIDER////////////////////
float vout;
float vin;
///////////////////ENCODER//////////////////
#define encodPinA1 34 // Quadrature encoder A pin
#define encodPinB1 35 // Quadrature encoder B pin
#define M1 37         // PWM outputs to L298N H-Bridge motor driver module
#define M2 36
/////////////////PID///////////////////////////
double pitchy;
double kp = 15, ki = 1.25, kd = 0.025; // modify for optimal performance
double input = 0, outputt = 0, setpoint = 0;
volatile long encoderPos = 0;
PID myPID(&input, &outputt, &setpoint, kp, ki, kd, DIRECT); // if motor will only run at full speed try 'REVERSE' instead of 'DIRECT'
int gyrooperational;
int prevgyro = 0;
int counter = 0;
float a;
float error_point;

void setup(void)
{
  Serial1.begin(19200);
  pinMode(A12, INPUT);
  pinMode(buzzer_pin, OUTPUT);
  bmp.begin(0x76, BMP280_CHIPID);
  myIMU.begin(0x0C);
  myIMU.setExtCrystalUse(true);
  ////////////////////////////////////////////////
  pinMode(encodPinA1, INPUT_PULLUP);     // quadrature encoder input A
  pinMode(encodPinB1, INPUT_PULLUP);     // quadrature encoder input B
  attachInterrupt(34, encoder, FALLING); // update encoder position
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);
  myPID.SetOutputLimits(-255, 255);
  ////////////////////////////////////////////////////
  SD.begin(chipSelect);
  /////////////////BMP280_FILTER////////////////////////////////////
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);

  digitalWrite(buzzer_pin, HIGH);
  delay(1000);
  digitalWrite(buzzer_pin, LOW);

  pinMode(camera, OUTPUT);

  packet_count = eeprom_read(10, 11);
  Alti_Ref = eeprom_read(20, 21);
  Software_State = eeprom_read(30, 31);

  if (Software_State == 1)
  {
    telemetry_onoff = "ON";
  }
  else if (Software_State == 0)
  {
    telemetry_onoff = "OFF";
  }
}

void loop()
{
  son_zaman = millis();
  cam_current = millis();
  current_time = millis();
  imu_();

  if (son_zaman - ilk_zaman >= 245)
  {
    telemetry_fonk();
    ilk_zaman = son_zaman;
  }
  if (cam_a == 1)
  {
    camera_func();
  }
}
void camera_func()
{
  if (eq == 0)
  {
    digitalWrite(camera, LOW);
    cam_prev = cam_current;
    eq = 1;
  }

  if (cam_current - cam_prev >= 750)
  {

    digitalWrite(camera, HIGH);
    cam_a = 0;
    eq = 0;
  }
}

void telemetry_fonk()
{
  gerilim();
  CMD();
  milli_();

  if (telemetry_onoff == "ON")
  {
    Telemetry = "";
    Telemetry += (team_id);
    Telemetry += ",";
    if (hour() < 10)
    {
      Telemetry += "0";
      Telemetry += hour();
    }
    else
    {
      Telemetry += hour();
    }
    Telemetry += ":";
    if (minute() < 10)
    {
      Telemetry += "0";
      Telemetry += minute();
    }
    else
    {
      Telemetry += minute();
    }
    Telemetry += ":";

    if (second() < 10)
    {
      Telemetry += "0";
      Telemetry += second();
    }
    else
    {
      Telemetry += second();
    }
    Telemetry += ":";
    Telemetry += milli;
    Telemetry += ",";
    Telemetry += (packet_count);
    Telemetry += ",";
    Telemetry += (tether);
    Telemetry += ",";
    Telemetry += (Alti - Alti_Ref);
    Telemetry += ",";
    Telemetry += temp;
    Telemetry += ",";
    Telemetry += vout;
    Telemetry += ",";
    Telemetry += gyro_x;
    Telemetry += ",";
    Telemetry += gyro_y;
    Telemetry += ",";
    Telemetry += gyro_z;
    Telemetry += ",";
    Telemetry += acc_x;
    Telemetry += ",";
    Telemetry += acc_y;
    Telemetry += ",";
    Telemetry += acc_z;
    Telemetry += ",";
    Telemetry += mag_x;
    Telemetry += ",";
    Telemetry += mag_y;
    Telemetry += ",";
    Telemetry += mag_z;
    Telemetry += ",";
    Telemetry += error_point;
    Telemetry += ",";
    Telemetry += "TELEMETRY_ON";
    Telemetry += ",";
    Serial1.println(Telemetry);
    Serial1.flush();
    packet_count++;

    my_file = SD.open("CanSat_Flight_1101_T", FILE_WRITE);
    my_file.println(Telemetry);
    my_file.close();
    eeprom_write(10, 11, packet_count);
  }
}

void imu_()
{
  acc = myIMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  gyro = myIMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  mag = myIMU.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  euler = myIMU.getVector(Adafruit_BNO055::VECTOR_EULER);
  // lineer = myIMU.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  gyro_x = gyro.x();
  gyro_y = gyro.y();
  gyro_z = gyro.z();
  acc_x = acc.x();
  acc_y = acc.y();
  acc_z = acc.z();
  mag_x = mag.x();
  mag_y = mag.y();
  mag_z = mag.z();
  temp = bmp.readTemperature();
  Alti = bmp.readAltitude(1013.25);
  PID_();
}

int sensor_rtc(int hh, int mm, int ss)
{
  setTime(hh, mm, ss, 10, 6, 2021);
}

void gerilim()
{

  vin = (analogRead(A12) * 4.2000) / 1024;
  vout = ((vin) * (0.89));
}

void CMD()
{
  if (Serial1.available() > 0)
  {
    Serial1.setTimeout(20);
    for (int i = 0; i <= 5; i++)
    {
      com[i] = Serial1.readStringUntil(',');
    }
  }

  if (com[0] == "CMD" && com[1] == "6101" && com[2] == "ST")
  {
    // RTC komutu
    // CMD,6101,ST,11,22,33,
    int hh = com[3].toInt();
    int mm = com[4].toInt();
    int ss = com[5].toInt();

    sensor_rtc(hh, mm, ss);
    // cam_a = 1;

    com[0] = '\0';
  }

  if (com[0] == "CMD" && com[1] == "6101" && com[2] == "CBN")
  {
    // KALİBRASYON KOMUTU
    //   CMD,6101,CBN,
    calibration();

    int j = 0;
    if (j == 0)
    {
      Alti_Ref = bmp.readAltitude(1013.25);
      eeprom_write(20, 21, Alti_Ref);
      j = 1;
    }
    digitalWrite(buzzer_pin, HIGH);
    delay(500);
    digitalWrite(buzzer_pin, LOW);

    com[0] = '\0';
  }

  if (com[0] == "CMD" && com[1] == "6101" && com[2] == "TPX" && com[3] == "ON")
  {
    // Telemetry komutu
    // CMD,6101,TPX,ON,
    cam_a = 1;
    telemetry_onoff = "ON";
    Software_State = 1;
    eeprom_write(30, 31, Software_State);

    com[0] = '\0';
  }

  if (com[0] == "CMD" && com[1] == "6101" && com[2] == "TPX" && com[3] == "OFF")
  {
    // Telemetry komutu
    // CMD,6101,TPX,OFF,
    telemetry_onoff = "OFF";
    Software_State = 0;
    eeprom_write(30, 31, Software_State);
    cam_a = 1;
    digitalWrite(buzzer_pin, HIGH);
    com[0] = '\0';
  }
  /*
    if (com[0] == "Buzzer")
    {
      com[0] = '\0';
    }*/
}

void milli_()
{

  milli += 25;
  if (milli > 75)
    milli = 0;
}

void calibration()
{

  packet_count = 0;
  Alti_Ref = 0;
  Software_State = 0;
  telemetry_onoff = "OFF";
  eeprom_clear();

  eeprom_write(10, 11, packet_count);
  eeprom_write(20, 21, Alti_Ref);
  eeprom_write(30, 31, Software_State);

  SD.remove("CanSat_Flight_1101_T");
}

void eeprom_clear()
{
  for (int i = 0; i < 100; i++)
  {
    EEPROM.update(i, 0);
  }
}

int eeprom_write(int x, int y, int kaydet)
{
  byte dusukbyte = lowByte(kaydet);
  byte buyukbyte = highByte(kaydet);

  EEPROM.update(x, dusukbyte);
  EEPROM.update(y, buyukbyte);
}
int eeprom_read(int x, int y)
{

  MSB = EEPROM.read(x);
  LSB = EEPROM.read(y);
  E_pack = MSB + (LSB << 8);
  return E_pack;
}

void PID_()
{

  if (current_time - prev_time > 25)
  {
    gyrooperational = euler.x();
    if (prevgyro - gyrooperational > 200)
    {
      counter++;
    }
    if (prevgyro - gyrooperational < -200)
    {
      counter--;
    }
    a = (counter * 360 + gyrooperational);

    prevgyro = euler.x();
    prev_time = current_time;
  }
  error_point = outputt / kp;

  setpoint = (a) / 3; //(analogRead(A0)*0.3515625)/3;

  input = encoderPos; // data from encoder

  myPID.Compute(); // calculate new output

  pwmOut(outputt);
}

void pwmOut(int out)
{

  if (out > 0)
  {
    analogWrite(M1, 0); // drive motor CW
    analogWrite(M2, out);
  }
  else if (out < 0)
  {
    analogWrite(M1, abs(out));
    analogWrite(M2, 0); // drive motor CCW
  }
  else
  {
    analogWrite(M1, 0);
    analogWrite(M2, 0);
  }
}

void encoder()
{ // pulse and direction, direct port reading to save cycles
  if (digitalRead(encodPinB1) == HIGH)
  {
    encoderPos++;
    // if(digitalRead(encodPinB1)==HIGH)   count ++;
  }
  else
  {
    encoderPos--; // if(digitalRead(encodPinB1)==LOW)   count --;
  }
}
