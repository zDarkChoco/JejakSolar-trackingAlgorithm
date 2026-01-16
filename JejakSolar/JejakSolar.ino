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

// --- PIN DEFINITIONS ---
#define I2C_SDA 21
#define I2C_SCL 22
#define ONE_WIRE_BUS 4
#define servoPinX 26
#define servoPinY 25

// --- CONFIGURATION ---
const long LOG_INTERVAL = 60000;  // 1 Minute
const int NIGHT_THRESHOLD = 300;  // Threshold for Night Mode
const int PARK_AZIMUTH = 0;       // Angle to face East
const int PARK_ELEVATION = 45;    // Safe angle

const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 28800;
const int daylightOffset_sec = 0;

// --- LDR SENSOR SETUP ---
struct LDRSensor {
  int pin;     
  int minVal;  
  int maxVal;  
};

LDRSensor sensorsPIN[4] = {
  { 34, 40, 3095 },   // Top Left (LDR1)
  { 32, 280, 4095 },  // Top Right (LDR2)
  { 35, 180, 4095 },  // Bottom Left (LDR3)
  { 33, 240, 4095 }   // Bottom Right (LDR4)
};

// --- SERVO SETTINGS ---
const int Dmin = 0;  
const int Dmax = 180;
float tolerance = 0.03;
int CurX = 90;
int CurY = 90;
int speed = 20;

// --- STATE VARIABLES ---
bool isManualMode = false;  
bool isNightMode = false;
bool isSystemOn = false;   // <--- DEFAULT OFF
float currentAvgLight = 0; // Store light level for logic checks

// --- OBJECTS ---
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
  if (!getLocalTime(&timeinfo)) return "00:00";
  char timeStringBuff[10];
  strftime(timeStringBuff, sizeof(timeStringBuff), "%H:%M", &timeinfo);
  return String(timeStringBuff);
}

float readNormalizedLDR(int sensorIndex) {
  int raw = analogRead(sensorsPIN[sensorIndex].pin);
  raw = constrain(raw, sensorsPIN[sensorIndex].minVal, sensorsPIN[sensorIndex].maxVal);
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

void handleSetMode() {
  if (server.hasArg("toggle")) {
    isManualMode = !isManualMode;
  }
  server.send(200, "text/plain", isManualMode ? "MANUAL" : "AUTO");
}

// --- NEW: SYSTEM TOGGLE HANDLER ---
void handleToggleSystem() {
  if (!isSystemOn) {
    // TRYING TO TURN ON
    if (currentAvgLight < NIGHT_THRESHOLD || isNightMode) {
      // Reject if it is night
      server.send(403, "text/plain", "Error: Cannot start in Night Mode (Too Dark)");
      return;
    }
    // Turn ON
    isSystemOn = true;
    isManualMode = false; // Default to AUTO when turned on
    
    // Attach Servos
    if (!myServoX.attached()) myServoX.attach(servoPinX);
    if (!myServoY.attached()) myServoY.attach(servoPinY);
    Serial.println("System Turned ON (AUTO Mode)");
    
  } else {
    // TRYING TO TURN OFF
    isSystemOn = false;
    // Detach Servos to stop movement/save power
    myServoX.detach();
    myServoY.detach();
    Serial.println("System Turned OFF");
  }
  
  server.send(200, "text/plain", isSystemOn ? "ON" : "OFF");
}

void handleSetServo() {
  if (!isManualMode || !isSystemOn) { // Cannot move if System OFF or Auto
    server.send(403, "text/plain", "Denied: System OFF or in AUTO");
    return;
  }

  if (server.hasArg("axis") && server.hasArg("val")) {
    String axis = server.arg("axis");
    int val = server.arg("val").toInt();
    val = constrain(val, Dmin, Dmax);

    if (axis == "pan") {
      CurX = val;
      myServoX.write(CurX);
    } else if (axis == "tilt") {
      CurY = val;
      myServoY.write(CurY);
    }
    server.send(200, "text/plain", "OK");
  } else {
    server.send(400, "text/plain", "Bad Args");
  }
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
  doc["irradiance"] = currentAvgLight; // Send raw light avg
  doc["mode"] = isManualMode ? "MANUAL" : "AUTO";
  doc["system_on"] = isSystemOn; // <--- Send Status to UI

  String jsonString;
  serializeJson(doc, jsonString);
  server.send(200, "application/json", jsonString);
}

void logDataToFile() {
  float power_mW = ina219.getPower_mW();
  float temp_c = sensors.getTempCByIndex(0);
  String timeStr;
  
  if (WiFi.status() == WL_CONNECTED) {
    timeStr = getLocalTime();
    if (timeStr == "00:00") timeStr = "S:" + String(millis() / 1000);
  } else {
    timeStr = "S:" + String(millis() / 1000);
  }

  File file = LittleFS.open("/log_full.txt", "a");
  if (!file) return;

  file.print(timeStr);
  file.print(",");
  file.print(temp_c, 1);
  file.print(",");
  file.println(power_mW, 0);
  file.close();
}

void checkNightMode(float avgLight) {
  // 1. Enter Night Mode
  if (avgLight < NIGHT_THRESHOLD) {
    if (!isNightMode) {
      Serial.println("--- NIGHT MODE ACTIVATED ---");
      Serial.println("Parking Panel to East...");
      
      // Only move if System is ON
      if (isSystemOn) {
        if (!myServoX.attached()) myServoX.attach(servoPinX);
        if (!myServoY.attached()) myServoY.attach(servoPinY);
        
        myServoX.write(PARK_AZIMUTH);
        myServoY.write(PARK_ELEVATION);
        delay(2000); // Give time to move
        
        // Detach to save power
        myServoX.detach();
        myServoY.detach();
      }
      isNightMode = true;
    }
    return;
  }

  // 2. Exit Night Mode
  if (isNightMode && avgLight > (NIGHT_THRESHOLD + 100)) { 
    Serial.println("--- MORNING DETECTED ---");
    isNightMode = false;
    // If system is ON, re-attach servos to start tracking
    if (isSystemOn) {
        myServoX.attach(servoPinX);
        myServoY.attach(servoPinY);
    }
  }
}

// Separate Tracking Logic
void performTracking(float valTL, float valTR, float valBL, float valBR) {
  float avgTop = (valTL + valTR) / 2;
  float avgBot = (valBL + valBR) / 2;
  float avgLeft = (valTL + valBL) / 2;
  float avgRight = (valTR + valBR) / 2;

  // --- ELEVATION (Up/Down) ---
  float diffY = avgTop - avgBot;
  if (abs(diffY) > tolerance) {
    if (avgTop > avgBot) CurY++;
    else CurY--;
    
    CurY = constrain(CurY, Dmin, Dmax);
    myServoY.write(CurY);
  }

  // --- AZIMUTH (Left/Right) ---
  float diffX = avgLeft - avgRight;
  if (abs(diffY) <= tolerance && abs(diffX) > tolerance) {
    bool moveLeft = (avgLeft > avgRight);
    bool moveRight = (avgRight > avgLeft);
    bool isFlipped = (CurY > 90);

    if (moveLeft) {
      if (isFlipped) CurX++; else CurX--;
    } else if (moveRight) {
      if (isFlipped) CurX--; else CurX++;
    }

    CurX = constrain(CurX, Dmin, Dmax);
    myServoX.write(CurX);
  }
}

// --- SETUP ---
void setup() {
  Serial.begin(115200);
  Serial.println("\n--- BOOTING SOLAR TRACKER ---");

  if (!LittleFS.begin(false)) LittleFS.begin(true);

  sensors.begin();
  Wire.begin(I2C_SDA, I2C_SCL);
  if (!ina219.begin()) Serial.println("INA219 Not Found");

  if (!LittleFS.exists("/log_full.txt")) {
    File file = LittleFS.open("/log_full.txt", "w");
    file.println("Time,Temp,Power(mW)");
    file.close();
  }

  // DO NOT Attach Servos yet. System starts OFF.
  // myServoX.attach(servoPinX); 
  // myServoY.attach(servoPinY);

  WiFiManager wifiManager;
  wifiManager.setConfigPortalTimeout(60); 
  if (!wifiManager.autoConnect("JejakSolar-Setup")) {
     Serial.println("Offline Mode...");
  } else {
     Serial.println("WiFi Connected: " + WiFi.localIP().toString());
  }

  if (MDNS.begin("jejaksolar")) Serial.println("mDNS started");
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  // Web Routes
  server.on("/", HTTP_GET, handleRoot);
  server.on("/index.html", HTTP_GET, handleRoot);
  server.on("/chart.min.js", HTTP_GET, handleChartJS);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/setMode", HTTP_GET, handleSetMode);
  server.on("/setServo", HTTP_GET, handleSetServo);
  
  // NEW ROUTE
  server.on("/toggleSystem", HTTP_GET, handleToggleSystem);

  server.serveStatic("/log_full.txt", LittleFS, "/log_full.txt");
  ElegantOTA.begin(&server);
  server.begin();
  Serial.println("Web Interface Ready!");
}

void loop() {
  sensors.requestTemperatures();
  server.handleClient();
  ElegantOTA.loop(); 

  // 1. Read Sensors Always (Needed for Night Mode & Turn ON check)
  float valTL = readNormalizedLDR(0);
  float valTR = readNormalizedLDR(1);
  float valBL = readNormalizedLDR(2);
  float valBR = readNormalizedLDR(3);
  
  // Update Global Light Level
  currentAvgLight = (valTR + valBR + valTL + valBL) / 4;
  
  // 2. Check Night Mode (Updates isNightMode flag)
  checkNightMode(currentAvgLight);

  // 3. Logging
  unsigned long currentMillis = millis();
  if (currentMillis - lastLogTime >= LOG_INTERVAL) {
    lastLogTime = currentMillis;
    logDataToFile(); 
  }

  // 4. Motor Logic
  if (isSystemOn) {
    if (!isNightMode) {
      if (!isManualMode) {
        performTracking(valTL, valTR, valBL, valBR);
        delay(speed);
      } else {
        delay(50); // Manual Mode Idle
      }
    } else {
       // Night Mode (System ON but sleeping)
       delay(200);
    }
  } else {
    // System OFF (Servos Detached)
    delay(200); 
  }
}
