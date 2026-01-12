#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <LittleFS.h>
#include <WiFiManager.h> 
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <time.h> 
#include "esp_adc_cal.h" 
#include <ElegantOTA.h>
#include <ESP32Servo.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// --- HARDWARE PIN DEFINITIONS ---
#define I2C_SDA 21
#define I2C_SCL 22
#define ONE_WIRE_BUS 4
#define servoPinX 25
#define servoPinY 26

// --- LOGGING CONFIGURATION ---
const long LOG_INTERVAL = 60000; // 1 Minute

// --- TIME CONFIGURATION ---
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 28800; 
const int   daylightOffset_sec = 0;

struct LDRSensor {
  int pin;          // GPIO Pin
  int minVal;       // Raw value in complete darkness (Calibrated)
  int maxVal;       // Raw value in full saturation (Calibrated)
};

LDRSensor sensorsPIN[4] = {
  {34, 50, 3865},  // Top Left (LDR1)
  {35, 280, 4095},  // Top Right (LDR2)
  {32, 180, 4095},  // Bottom Left (LDR3)
  {33, 240, 4095}   // Bottom Right (LDR4)
};

const int Dmin = 20;
const int Dmax = 160;

float tolerance = 0.1; 
int CurX = 90;
int CurY = 90;
int speed = 50;

// --- GLOBAL OBJECTS ---

Servo myServoX;
Servo myServoY;
Adafruit_INA219 ina219;
WebServer server(80);
unsigned long lastLogTime = 0;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// --- HELPER FUNCTIONS ---
String getLocalTime() {
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)) return "00:00";
  char timeStringBuff[10];
  strftime(timeStringBuff, sizeof(timeStringBuff), "%H:%M", &timeinfo);
  return String(timeStringBuff);
}

float readNormalizedLDR(int sensorIndex) {
  int raw = analogRead(sensorsPIN[sensorIndex].pin);
  
  raw = constrain(raw, sensorsPIN[sensorIndex].minVal, sensorsPIN[sensorIndex].maxVal);

  // Map raw value to 0.0 - 1.0 (Float precision for better PID control)
  float normalized = (float)(raw - sensorsPIN[sensorIndex].minVal) / (float)(sensorsPIN[sensorIndex].maxVal - sensorsPIN[sensorIndex].minVal);
                     
  return normalized;
}


// --- WEB HANDLERS ---
void handleRoot() {
  File file = LittleFS.open("/index.html", "r");
  if (!file) { server.send(404, "text/plain", "Error: Upload index.html!"); return; }
  server.streamFile(file, "text/html");
  file.close();
}

void handleChartJS() {
  File file = LittleFS.open("/chart.min.js", "r");
  if (!file) { server.send(404, "text/plain", "Error: Upload chart.min.js!"); return; }
  server.streamFile(file, "application/javascript");
  file.close();
}

void handleStatus() {
  float busvoltage = ina219.getBusVoltage_V();
  float current_mA = ina219.getCurrent_mA();
  float power_mW = ina219.getPower_mW();
  float temp_c = sensors.getTempCByIndex(0);
  
  StaticJsonDocument<300> doc;
  doc["voltage"] = busvoltage;
  doc["current_mA"] = current_mA;
  doc["power_mW"] = power_mW;
  doc["temp_c"] = temp_c;
  doc["irradiance"] = 0.0;
  doc["mode"] = "AUTO"; 

  String jsonString;
  serializeJson(doc, jsonString);
  server.send(200, "application/json", jsonString);
}

void logDataToFile() {
  float power_mW = ina219.getPower_mW();
  float temp_c = sensors.getTempCByIndex(0);
  String timeStr = getLocalTime();

  File file = LittleFS.open("/log_full.txt", "a");
  if (!file) return;

  file.print(timeStr);
  file.print(",");
  file.print(temp_c, 1); 
  file.print(",");
  file.println(power_mW, 0); 
  file.close();
}

void track(){
  float val_TL = readNormalizedLDR(0);
  float val_TR = readNormalizedLDR(1);
  float val_BL = readNormalizedLDR(2);
  float val_BR = readNormalizedLDR(3);

  float avg_top = (val_TL + val_TR) / 2;
  float avg_bottom = (val_BL + val_BR) / 2;
  float avg_left = (val_TL + val_BL) / 2;
  float avg_right = (val_TR + val_BR) / 2;

  float vertical_error = avg_top - avg_bottom;
  float horizontal_error = avg_left - avg_right;

  if (vertical_error > tolerance) {
    Serial.println("ACTION: Move DOWN (Top is brighter)");
    CurX++; 
  } 
  else if (vertical_error < -tolerance) {
    Serial.println("ACTION: Move UP (Bottom is brighter)");
    CurX--;
  } 
  else {
    Serial.println("ACTION: (Vertical OK)");
  }

  if(CurX > Dmax) CurX = Dmax;
  if(CurX < Dmin) CurX = Dmin;
  myServoX.write(CurX);

  // 2. Horizontal (Pan) Motor
  if (horizontal_error > tolerance) {
    Serial.println("ACTION: Move RIGHT (Left is brighter)");
    CurY--;
  } 
  else if (horizontal_error < -tolerance) {
    Serial.println("ACTION: Move LEFT (Right is brighter)");
    CurY++;   
  } 
  else {
    Serial.println("ACTION: (Horizontal OK)");
  }
  
  if(CurX > Dmax) CurX = Dmax;
  if(CurX < Dmin) CurX = Dmin;
  myServoY.write(CurY);
}

// --- SETUP ---
void setup() {
  Serial.begin(115200);
  Serial.println("\n--- BOOTING SOLAR TRACKER ---");
  

  // 1. Filesystem
  if (!LittleFS.begin(false)) LittleFS.begin(true);

  // 3. Sensors
  sensors.begin();
  Wire.begin(I2C_SDA, I2C_SCL);
  if (!ina219.begin()) Serial.println("INA219 Not Found");

  // 4. Log File
  if (!LittleFS.exists("/log_full.txt")) {
    File file = LittleFS.open("/log_full.txt", "w");
    file.println("Time,Power(mW)");
    file.close();
  }

  myServoX.attach(servoPinX);
  myServoY.attach(servoPinY);
  myServoX.write(CurX); 
  myServoY.write(CurY);

  // 5. WiFi
  WiFiManager wifiManager;
  wifiManager.setConnectTimeout(8);
  if (!wifiManager.autoConnect("JejakSolar-Setup")) ESP.restart();
  Serial.println("WiFi Connected: " + WiFi.localIP().toString());

  // 6. mDNS
  if (MDNS.begin("jejaksolar")) Serial.println("mDNS started");

  // 7. Time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  // 8. Server Routes
  server.on("/", HTTP_GET, handleRoot);
  server.on("/index.html", HTTP_GET, handleRoot);
  server.on("/chart.min.js", HTTP_GET, handleChartJS);
  server.on("/status", HTTP_GET, handleStatus);
  server.serveStatic("/log_full.txt", LittleFS, "/log_full.txt");

  // --- START ELEGANT OTA ---
  ElegantOTA.begin(&server);    

  server.begin();
  Serial.println("Web Interface Ready!");
}

void loop() {
  sensors.requestTemperatures(); 
  server.handleClient();
  
  unsigned long currentMillis = millis();
  if (currentMillis - lastLogTime >= LOG_INTERVAL) {
    lastLogTime = currentMillis;
    if (WiFi.status() == WL_CONNECTED) logDataToFile();
  }

  track();
  delay(speed); 
}