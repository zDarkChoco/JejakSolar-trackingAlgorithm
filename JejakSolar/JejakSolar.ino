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
  {34, 40, 3095},  // Top Left (LDR1)
  {32, 280, 4095},  // Top Right (LDR2)
  {35, 180, 4095},  // Bottom Left (LDR3)
  {33, 240, 4095}   // Bottom Right (LDR4)
};

const int Dmin = 20;
const int Dmax = 160;

float tolerance = 0.03; 
int CurX = 90;
int CurY = 90;
int speed = 100;

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
  // Assuming logic: If Top is brighter than Bottom, we need to tilt towards Top.
  // Check your specific servo mechanics: Does angle++ move Up or Down?
  // Current assumption: Angle++ moves towards East/Down (away from West).
  // Adjust the '++' and '--' below if it moves the wrong way.
  
  float diffY = avgTop - avgBot;

  Serial.print("diffY: "); Serial.print(diffY);
  //bool isFlipped = (posY > 90);
  
  if (abs(diffY) > tolerance) {
    if (avgTop > avgBot) {
      // Top is brighter. If 90 is UP and 20 is WEST, we usually need to move closer to 90?
      // You may need to SWAP these signs based on your physical motor mounting.
      CurY++; 
    } else {
      // Bottom is brighter
      CurY--;
    }
    if (CurY > Dmax) CurY = Dmax;
    if (CurY < Dmin) CurY = Dmin;
    myServoY.write(CurY);
  }

  // Constrain Y to user limits
  

  // --- D. Logic for X-Axis (Pan) WITH INVERSION ---
  float diffX = avgLeft - avgRight;

  Serial.print(" | diffX: "); Serial.println(diffX);

  if (abs(diffY) <= tolerance && abs(diffX) > tolerance) {
    
    // Determine Tracking Direction
    bool moveLeft = (avgLeft > avgRight); // Light is on Left
    bool moveRight = (avgRight > avgLeft); // Light is on Right
    
    // CHECK ORIENTATION: Is the panel flipped over the top?
    // If Y > 90, we are "looking back" (East), so Left/Right commands must swap.
    bool isFlipped = (CurY > 90); 

    if (moveLeft) {
      
        if (isFlipped) {
        // Inverted: Light is Left (sensor), but physically we must turn Right
        CurX++; 
      } else {
        // Normal: Light is Left, turn Left
        CurX--; 
      } 
      
    } 
    else if (moveRight) {
      
        // Normal: Light is Right, turn Right
       if (isFlipped) {
        // Inverted: Light is Right (sensor), but physically we must turn Left
        CurX--; 
      } else {
        // Normal: Light is Right, turn Right
        CurX++; 
      } 
      
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
