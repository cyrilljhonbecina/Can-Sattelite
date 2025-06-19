#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_MPL3115A2.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TinyGPS++.h>

// Pin definitions
#define SD_CS 5     // SD card module Chip Select pin
#define MIC_PIN 27  // MAX4466 microphone pin
#define BUZZER_PIN 26
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define GPS_BAUDRATE 9600
#define HC12_BAUDRATE 9600

// Thermistor Constants
#define VCC 3.3        // ESP32 operates at 3.3V
#define ADC_MAX 4095   // 12-bit ADC resolution
#define R_FIXED 10000  // Fixed resistor value in series with thermistor

float To = 298.15;   // Reference temperature in Kelvin (25°C)
float Ro = 10000.0;  // Resistance of thermistor at To (10kΩ)
float B = 3435.0;    // Beta coefficient

int thermistorPin1 = 12;  // First thermistor
int thermistorPin2 = 13;  // Second thermistor
int thermistorPin3 = 14;  // Third thermistor

// Sensor objects
Adafruit_MPU6050 mpu;
Adafruit_MPL3115A2 baro;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
TinyGPSPlus gps;

// Microphone variables
const int baseline = 337;
int mic, amplitude;

void setup() {
  Serial.begin(115200);
  Serial2.begin(GPS_BAUDRATE, SERIAL_8N1, 16, 17);
  Serial1.begin(HC12_BAUDRATE, SERIAL_8N1, 15, 2);  // HC-12 connected
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  // Initialize OLED display
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 OLED initialization failed!");
    while (1)
      ;
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("Initializing...");
  display.display();

  // Initialize SD card
  if (!SD.begin(SD_CS)) {
    Serial.println("SD card initialization failed!");
    display.println("SD Error!");
    display.display();
    return;
  }
  display.println("SD Ready!");
  display.display();

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1)
      ;
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Initialize MPL3115A2
  if (!baro.begin()) {
    Serial.println("Could not find MPL3115A2 sensor");
    while (1)
      ;
  }
  baro.setSeaPressure(1013.26);

  display.println("Sensors Ready!");
  display.display();
  delay(1000);
}
void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Read microphone (MAX4466)
  mic = analogRead(MIC_PIN);
  amplitude = abs(mic - baseline);

  // Read MPL3115A2
  float pressure = baro.getPressure();
  float altitude = baro.getAltitude();
  float mpl_temp = baro.getTemperature();

  // Read Thermistors
  float temp1 = readTemperature(thermistorPin1);
  float temp2 = readTemperature(thermistorPin2);
  float temp3 = readTemperature(thermistorPin3);

  display.clearDisplay();
  display.setCursor(0, 0);

  String gpsData = "Location: INVALID";

  if (Serial2.available() > 0) {
    if (gps.encode(Serial2.read())) {
      if (gps.location.isValid()) {
        gpsData = "Lat: " + String(gps.location.lat(), 6) + ", Lon: " + String(gps.location.lng(), 6);
      }
      gpsData += ", Speed: " + (gps.speed.isValid() ? String(gps.speed.kmph()) : "INVALID") + " km/h";
      gpsData += ", Satellites: " + (gps.satellites.isValid() ? String(gps.satellites.value()) : "INVALID");
      if (gps.date.isValid() && gps.time.isValid()) {
        gpsData += ", Date: " + String(gps.date.day()) + "/" + String(gps.date.month()) + "/" + String(gps.date.year());
        gpsData += ", Time: " + String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second());
      } else {
        gpsData += ", Date/Time: INVALID";
      }
    }
  }

  display.println(gpsData);
  display.println("NTC1: " + String(temp1) + " C");
  display.println("NTC2: " + String(temp2) + " C");
  display.println("NTC3: " + String(temp3) + " C");
  display.println("Pressure: " + String(pressure) + " hPa");
  display.println("Alt: " + String(altitude) + " m");
  display.println("Sound: " + String(amplitude));
  display.println("MPL T: " + String(mpl_temp) + " C");
  display.println("X: " + String(a.acceleration.x) + " Y: " + String(a.acceleration.y) + " Z: " + String(a.acceleration.z));
  display.display();

  String dataString = gpsData + " ";
  dataString += "Acceleration X: " + String(a.acceleration.x) + " Y: " + String(a.acceleration.y) + " Z: " + String(a.acceleration.z) + " ";
  dataString += "Gyro X: " + String(g.gyro.x) + " Y: " + String(g.gyro.y) + " Z: " + String(g.gyro.z) + " ";
  dataString += "Temp1: " + String(temp1) + " C, Temp2: " + String(temp2) + " C, Temp3: " + String(temp3) + " C ";
  dataString += "Sound Amplitude: " + String(amplitude) + " ";
  dataString += "Pressure: " + String(pressure) + " hPa, Altitude: " + String(altitude) + " m, " + "MPL Temp: " + String(mpl_temp) + " C";

  Serial.println(dataString);
  Serial1.println(dataString);
  // Write data to SD card
  File file = SD.open("/sensor_data.txt", FILE_APPEND);
  if (file) {
    file.print(dataString);
    file.close();
    Serial.println("Data written to SD card.");
  } else {
    Serial.println("Error opening file!");
  }
  digitalWrite(BUZZER_PIN, HIGH);
  delay(100);
  digitalWrite(BUZZER_PIN, LOW);
  delay(2000);
}


float readTemperature(int pin) {
  int Dmeasured = analogRead(pin);
  float voltage = Dmeasured * (VCC / ADC_MAX);
  float Rthermistor = (R_FIXED * voltage) / (VCC - voltage);
  float TempK = (To * B) / (B + (To * log(Rthermistor / Ro)));
  return TempK - 273.15;  // Convert to Celsius
}