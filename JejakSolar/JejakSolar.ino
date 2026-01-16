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

#define I2C_SDA 21
#define I2C_SCL 22
#define ONE_WIRE_BUS 4
#define servoPinX 26
#define servoPinY 25

const long LOG_INTERVAL = 60000;  // 1 Minute
const int NIGHT_THRESHOLD = 300; // Adjust: Lower = darker required to sleep
const int PARK_AZIMUTH = 0;      // Angle to face East (Sunrise)
const int PARK_ELEVATION = 45;   // Safe angle for wind

const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 28800;
const int daylightOffset_sec = 0;

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

const int Dmin = 0;  
const int Dmax = 180;

float tolerance = 0.03;
int CurX = 90;
int CurY = 90;
int speed = 20;
bool isManualMode = false;  
bool isNightMode = false;

Servo myServoX;
Servo myServoY;
Adafruit_INA219 ina219;
WebServer server(80);
unsigned long lastLogTime = 0;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

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


void handleRoot() {
  File file = LittleFS.open("/index.html", "r");
  if (!file) {
    server.send(404, "text/plain", "Error: Upload index.html!");
    return;
  }
  server.streamFile(file, "text/html");
  file.close();
}

void handleChartJS() {
  File file = LittleFS.open("/chart.min.js", "r");
  if (!file) {
    server.send(404, "text/plain", "Error: Upload chart.min.js!");
    return;
  }
  server.streamFile(file, "application/javascript");
  file.close();
}


void handleSetMode() {
  if (server.hasArg("toggle")) {
    isManualMode = !isManualMode;
    Serial.println(isManualMode ? "Switched to MANUAL" : "Switched to AUTO");
  }
  server.send(200, "text/plain", isManualMode ? "MANUAL" : "AUTO");
}


void handleSetServo() {
  if (!isManualMode) {
    server.send(403, "text/plain", "Denied: System is in AUTO mode");
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
  doc["irradiance"] = 0.0;


  doc["mode"] = isManualMode ? "MANUAL" : "AUTO";

  String jsonString;
  serializeJson(doc, jsonString);
  server.send(200, "application/json", jsonString);
}

void logDataToFile() {
  float power_mW = ina219.getPower_mW();
  float temp_c = sensors.getTempCByIndex(0);
  String timeStr;
  
  // LOGIC: Check WiFi first.
  if (WiFi.status() == WL_CONNECTED) {
    timeStr = getLocalTime();
    // Fallback: If NTP hasn't updated yet, use Seconds
    if (timeStr == "00:00") {
      timeStr = "S:" + String(millis() / 1000);
    }
  } else {
    // Offline Mode: Use Seconds since boot
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

void checkNightMode(int avgLight) {
  // 1. Enter Night Mode
  if (avgLight < NIGHT_THRESHOLD) {
    if (!isNightMode) {
      Serial.println("--- NIGHT MODE ACTIVATED ---");
      Serial.println("Parking Panel to East...");
      
      // Move to Park Position
      servoHorizontal.attach(PIN_SERVO_H); // Re-attach briefly to move
      servoVertical.attach(PIN_SERVO_V);
      
      servoHorizontal.write(PARK_AZIMUTH);
      servoVertical.write(PARK_ELEVATION);
      delay(2000); // Give time to move
      
      // Detach to save power and stop jitter
      servoHorizontal.detach();
      servoVertical.detach();
      
      isNightMode = true;
    }
    // While in Night Mode, do nothing. Just wait.
    return;
  }

  // 2. Exit Night Mode (Morning)
  if (isNightMode && avgLight > (NIGHT_THRESHOLD + 100)) { // Hysteresis to prevent flipping
    Serial.println("--- MORNING DETECTED ---");
    Serial.println("Waking up Servos...");
    
    servoHorizontal.attach(PIN_SERVO_H);
    servoVertical.attach(PIN_SERVO_V);
    
    isNightMode = false;
  }
}

void track() {
  float valTL = readNormalizedLDR(0);
  float valTR = readNormalizedLDR(1);
  float valBL = readNormalizedLDR(2);
  float valBR = readNormalizedLDR(3);


  float avgTop = (valTL + valTR) / 2;
  float avgBot = (valBL + valBR) / 2;
  float avgLeft = (valTL + valBL) / 2;
  float avgRight = (valTR + valBR) / 2;
  float avgLight = (valTR + valBR + valTL + valBL) / 4;

  checkNightMode(avgLight);

  Serial.print("avgTop: ");
  Serial.print(avgTop);
  Serial.print(" | avgBot: ");
  Serial.print(avgBot);
  Serial.print(" | avgLeft: ");
  Serial.print(avgLeft);
  Serial.print(" | avgRight: ");
  Serial.println(avgRight);

  float diffY = avgTop - avgBot;
  Serial.print("diffY: ");
  Serial.print(diffY);

  if (abs(diffY) > tolerance) {
    if (avgTop > avgBot) {
      CurY++;
    } else {
      CurY--;
    }
    if (CurY > Dmax) CurY = Dmax;
    if (CurY < Dmin) CurY = Dmin;
    myServoY.write(CurY);
  }

  float diffX = avgLeft - avgRight;
  Serial.print(" | diffX: ");
  Serial.println(diffX);

  if (abs(diffY) <= tolerance && abs(diffX) > tolerance) {
    bool moveLeft = (avgLeft > avgRight);
    bool moveRight = (avgRight > avgLeft);

    bool isFlipped = (CurY > 90);

    if (moveLeft) {
      if (isFlipped) CurX++;  
      else CurX--;            
    } else if (moveRight) {
      if (isFlipped) CurX--;  
      else CurX++;            
    }

    if (CurX > Dmax) CurX = Dmax;
    if (CurX < Dmin) CurX = Dmin;
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
    file.println("Time,Power(mW)");
    file.close();
  }

  myServoX.attach(servoPinX);
  myServoY.attach(servoPinY);
  myServoX.write(CurX);
  myServoY.write(CurY);

  // 5. WiFi
  WiFiManager wifiManager;
  wifiManager.setConfigPortalTimeout(60); // Wait 1 mins for user, then quit
  if (!wifiManager.autoConnect("JejakSolar-Setup")) {
     Serial.println("Offline Mode: Logging initiated...");
     // We do NOT restart. We let it continue to loop().
  } else {
     Serial.println("WiFi Connected: " + WiFi.localIP().toString());
  }


  if (MDNS.begin("jejaksolar")) Serial.println("mDNS started");
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);


  server.on("/", HTTP_GET, handleRoot);
  server.on("/index.html", HTTP_GET, handleRoot);
  server.on("/chart.min.js", HTTP_GET, handleChartJS);
  server.on("/status", HTTP_GET, handleStatus);


  server.on("/setMode", HTTP_GET, handleSetMode);
  server.on("/setServo", HTTP_GET, handleSetServo);

  server.serveStatic("/log_full.txt", LittleFS, "/log_full.txt");


  ElegantOTA.begin(&server);

  server.begin();
  Serial.println("Web Interface Ready!");
}

void loop() {
  sensors.requestTemperatures();
  server.handleClient();
  ElegantOTA.loop(); 

  unsigned long currentMillis = millis();
  if (currentMillis - lastLogTime >= LOG_INTERVAL) {
    lastLogTime = currentMillis;
    logDataToFile(); 
  }

  if (!isManualMode) {
    track();
    delay(speed);
  } else {

    delay(50);
  }
}
