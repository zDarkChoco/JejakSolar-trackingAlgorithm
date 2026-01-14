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
#define servoPinX 26
#define servoPinY 25

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
  {34, 40, 3095},   // Top Left (LDR1)
  {32, 280, 4095},  // Top Right (LDR2)
  {35, 180, 4095},  // Bottom Left (LDR3)
  {33, 240, 4095}   // Bottom Right (LDR4)
};

const int Dmin = 0;   // Expanded range for manual control (0-180)
const int Dmax = 180;

float tolerance = 0.03; 
int CurX = 90;
int CurY = 90;
int speed = 100;
bool isManualMode = false; // [NEW] Flag for Manual Mode

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
  // Map raw value to 0.0 - 1.0
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

// [NEW] Handle Mode Switching (Auto <-> Manual)
void handleSetMode() {
  if (server.hasArg("toggle")) {
    isManualMode = !isManualMode;
    Serial.println(isManualMode ? "Switched to MANUAL" : "Switched to AUTO");
  }
  server.send(200, "text/plain", isManualMode ? "MANUAL" : "AUTO");
}

// [NEW] Handle Manual Servo Control
void handleSetServo() {
  if (!isManualMode) {
    server.send(403, "text/plain", "Denied: System is in AUTO mode");
    return;
  }
  
  if (server.hasArg("axis") && server.hasArg("val")) {
    String axis = server.arg("axis");
    int val = server.arg("val").toInt();
    
    // Safety Constrain
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
  
  // [UPDATED] Send actual mode status
  doc["mode"] = isManualMode ? "MANUAL" : "AUTO"; 

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
  float valTL = readNormalizedLDR(0);
  float valTR = readNormalizedLDR(1);
  float valBL = readNormalizedLDR(2);
  float valBR = readNormalizedLDR(3);

  // --- B. Calculate Averages ---
  float avgTop   = (valTL + valTR) / 2;
  float avgBot   = (valBL + valBR) / 2;
  float avgLeft  = (valTL + valBL) / 2;
  float avgRight = (valTR + valBR) / 2;

  Serial.print("avgTop: "); Serial.print(avgTop);
  Serial.print(" | avgBot: "); Serial.print(avgBot);
  Serial.print(" | avgLeft: "); Serial.print(avgLeft);
  Serial.print(" | avgRight: "); Serial.println(avgRight);

  // --- C. Logic for Y-Axis (Tilt/Elevation) ---
  float diffY = avgTop - avgBot;
  Serial.print("diffY: "); Serial.print(diffY);
  
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

  // --- D. Logic for X-Axis (Pan) WITH INVERSION ---
  float diffX = avgLeft - avgRight;
  Serial.print(" | diffX: "); Serial.println(diffX);

  if (abs(diffY) <= tolerance && abs(diffX) > tolerance) {
    bool moveLeft = (avgLeft > avgRight); 
    bool moveRight = (avgRight > avgLeft); 
    
    // Invert Pan controls if we are tilted "over the back" (>90)
    bool isFlipped = (CurY > 90); 

    if (moveLeft) {
        if (isFlipped) CurX++; // Inverted
        else CurX--; // Normal
    } 
    else if (moveRight) {
       if (isFlipped) CurX--; // Inverted
       else CurX++; // Normal
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

  // 1. Filesystem
  if (!LittleFS.begin(false)) LittleFS.begin(true);

  // 2. Sensors
  sensors.begin();
  Wire.begin(I2C_SDA, I2C_SCL);
  if (!ina219.begin()) Serial.println("INA219 Not Found");

  // 3. Log File
  if (!LittleFS.exists("/log_full.txt")) {
    File file = LittleFS.open("/log_full.txt", "w");
    file.println("Time,Power(mW)");
    file.close();
  }

  // 4. Servos
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
  
  // [NEW] Register Manual Control Handlers
  server.on("/setMode", HTTP_GET, handleSetMode);
  server.on("/setServo", HTTP_GET, handleSetServo);

  server.serveStatic("/log_full.txt", LittleFS, "/log_full.txt");

  // --- START ELEGANT OTA ---
  ElegantOTA.begin(&server);    

  server.begin();
  Serial.println("Web Interface Ready!");
}

void loop() {
  sensors.requestTemperatures(); 
  server.handleClient();
  ElegantOTA.loop(); // Don't forget OTA loop if you use it!
  
  unsigned long currentMillis = millis();
  if (currentMillis - lastLogTime >= LOG_INTERVAL) {
    lastLogTime = currentMillis;
    if (WiFi.status() == WL_CONNECTED) logDataToFile();
  }

  // [UPDATED] Only track automatically if NOT in manual mode
  if (!isManualMode) {
    track();
    delay(speed);
  } else {
    // In manual mode, we just wait for web commands. 
    // Small delay to prevent CPU hogging.
    delay(50); 
  }
}
