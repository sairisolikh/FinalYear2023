// LIBRARY WIFI + FIREBASE
#include <WiFi.h>
#include <HTTPClient.h>
#include <IOXhop_FirebaseESP32.h>
#include "Arduino.h"
//...//

#define ssid "WIFI"
#define password "paswordnyaaa123"
#define FIREBASE_HOST "https://gstick-c1038-default-rtdb.firebaseio.com/"
#define FIREBASE_AUTH "q7imS0wy1XrKqYOFdzUH4uEV2VbCfIv8FjkTqonS"
String GOOGLE_SCRIPT_ID = "AKfycbxh1-flJ04WD7OOpqyfxZ0jMG85iyJ256XHC3eg63-aPlBmsjrfddpi5udmxQaqxWI";
bool flagKonekWifi = false;


#include <analogWrite.h>

#include <Wire.h>
#include <VL53L0X.h>
#include <VL53L1X.h>
#include <math.h>

// LIBRARY MP3 OPENSMART UART
#include <SoftwareSerial.h>
#include "RedMP3.h" //library lagu

// LIBRARY WIFI + FIREBASE
#include <WiFi.h>
#include <HTTPClient.h>
#include <IOXhop_FirebaseESP32.h>

// LIBRARY MOTOR GETAR
#include <loopTimer.h>
#include <millisDelay.h>
#include <BufferedOutput.h>
bool motorOn = false;
millisDelay motorDelay;
millisDelay printDelay;

// Inisialisasi sensor jarak VL53L0X dan VL53L1X
VL53L0X sensor1;
VL53L1X sensor2;
#define SDA 33
#define SCL 32

// Inisialisasi pin motor getar
const int motorPin = 15;

//KONFIGURASI MODUL MP3 OPENSMART UART
#define MP3_RX 4//RX D4
#define MP3_TX 2//TX D2
MP3 mp3(MP3_RX, MP3_TX);
int8_t volume = 0x1e;//0~0x1e (30 adjustable level speaker)
int8_t folderName;//folder name must be 01 02 03 04 ...
int8_t fileName; // prefix of file name must be 001xxx 002xxx

// Definisi variabel fuzzy input dan output
float distance0, distance1;
float fuzzyValue0, fuzzyValue1;

// Fungsi untuk menghitung nilai fuzzifikasi
float calculateFuzzyValue(float value, float mean, float deviation)
{
  return exp(-pow((value - mean) / deviation, 2));
}

// Fungsi untuk melakukan defuzzifikasi centroid
int centroidDefuzzification(float lowValue, float midValue, float highValue)
{
  // Menghitung centroid (titik pusat) dari area fuzzy
  float centroid = (lowValue + midValue + highValue) / 3.0;

  // Defuzzifikasi centroid dengan skala nilai 0-255
  int defuzzifiedValue = static_cast<int>(centroid * 255.0);
  
  // Batasi nilai defuzzifikasi agar tidak melebihi batas maksimum
  if (defuzzifiedValue > 255) {
    defuzzifiedValue = 255;
  }

  return defuzzifiedValue;
}

void setup()
{
  // Menginisialisasi komunikasi serial
  Serial.begin(115200);
  WiFi.begin(ssid,password);
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Wire.begin();

  Wire.begin();
  Wire1.begin(SDA, SCL);
  sensor1.setAddress(0x29);
  sensor1.setBus(&Wire);
  sensor1.setTimeout(500);
  if (!sensor1.init())
  {
    Serial.println("VL53L0X FAILED!");
    while (1) {}
  }
  sensor1.startContinuous();
  delay(500);
  
  sensor2.setBus(&Wire1);
  sensor2.setAddress(0x29);
  sensor2.setTimeout(500);
  if (!sensor2.init())
  {
    Serial.println("VL53L1X FAILED!");
    while (1);
  }
  sensor2.setDistanceMode(VL53L1X::Long);
  sensor2.setMeasurementTimingBudget(50000);
  sensor2.startContinuous(50); 

  // Inisialisasi motor getaran
  pinMode(motorPin, OUTPUT);

}

void loop()
{
      //wifi non-blocking
  if (WiFi.status() == WL_CONNECTED && !flagKonekWifi){
    Serial.println("Connected");
    flagKonekWifi = true;
//    suaraWifiTerhubung();
  }
  if (WiFi.status() != WL_CONNECTED){
    Serial.println(".");
    delay(1000);
    flagKonekWifi = false;
  }

  // Mengukur jarak menggunakan sensor VL53L0X dan VL53L1X
  distance0 = sensor1.readRangeContinuousMillimeters();
  distance1 = sensor2.read();
  float outputBawah, outputAtas;
  outputBawah = (float)(sensor1.readRangeContinuousMillimeters()/10.0);
  outputAtas = (float)(sensor2.read()/10.0);

  // Menghitung nilai fuzzifikasi untuk setiap sensor
  fuzzyValue0 = calculateFuzzyValue(distance0, 0, 1000); // Mean = 150, Deviation = 50
  fuzzyValue1 = calculateFuzzyValue(distance1, 0, 1000); // Mean = 200, Deviation = 70

  // Menentukan kecepatan motor getar berdasarkan logika fuzzy
  int motorSpeed = 0;

  // Menentukan nilai fuzzifikasi untuk setiap fuzzy set output
  float lowValue = fuzzyValue0;
  float midValue = max(fuzzyValue0, fuzzyValue1);
  float highValue = fuzzyValue1;

  // Melakukan defuzzifikasi centroid untuk mendapatkan kecepatan motor getar
  motorSpeed = centroidDefuzzification(lowValue, midValue, highValue);

  // Mengendalikan motor getar berdasarkan kecepatan
  analogWrite(motorPin, motorSpeed);

  // Menampilkan hasil ke serial monitor
  Serial.print("Distance 0: ");
  Serial.print(distance0);
  Serial.print(" mm, Distance 1: ");
  Serial.print(distance1);
  Serial.print(" mm, Motor Speed: ");
  Serial.println(motorSpeed);

  if (outputBawah <= 100){
    suaraHalangan();
  }
  if (outputAtas <= 100){
    suaraHalangan();
  }

  delay(100);
}

//OUTPUT SUARA OPENSMART
void startupsuara(){
   mp3.setVolume(volume);
   mp3.playWithFileName(01,001);
   delay(2000);
}
void suaraHalangan(){
  mp3.setVolume(volume);
  mp3.playWithFileName(01,003);
  delay(2000);
}

void suaraWifiTerhubung(){
  mp3.setVolume(volume);
  mp3.playWithFileName(01,002);
  delay(2000);
}
