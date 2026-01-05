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

// --- HARDWARE PIN DEFINITIONS ---
#define I2C_SDA 21
#define I2C_SCL 22
//#define PIN_LM35 35 

// --- CALIBRATION SETTINGS ---
// We removed the 2.0 multiplier.
// If the temp is consistently off by a few degrees, change this offset.
// Example: If real is 26 and screen is 28, set this to -2.0.
//#define TEMP_OFFSET 0.0 

// --- LOGGING CONFIGURATION ---
const long LOG_INTERVAL = 60000; // 1 Minute

// --- TIME CONFIGURATION ---
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 28800; 
const int   daylightOffset_sec = 0;

// --- GLOBAL OBJECTS ---
Adafruit_INA219 ina219;
WebServer server(80);
unsigned long lastLogTime = 0;

// --- ADC CALIBRATION ---
//esp_adc_cal_characteristics_t *adc_chars;
//#define DEFAULT_VREF 1100 

// --- HELPER FUNCTIONS ---
String getLocalTime() {
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)) return "00:00";
  char timeStringBuff[10];
  strftime(timeStringBuff, sizeof(timeStringBuff), "%H:%M", &timeinfo);
  return String(timeStringBuff);
}

/*float readCalibratedTemp() {
  unsigned long totalMilliVolts = 0;
  int samples = 60; // Increased samples for 11db stability

  for (int i = 0; i < samples; i++) {
    int rawValue = analogRead(PIN_LM35);
    // Convert raw using the 11dB characteristics
    totalMilliVolts += esp_adc_cal_raw_to_voltage(rawValue, adc_chars);
    delay(1);
  }

  float avgMilliVolts = totalMilliVolts / (float)samples;

  // 1. Convert Voltage to Base Temp (10mV = 1C)
  float baseTemp = avgMilliVolts / 10.0;

  // 2. Apply Offset (No multiplier anymore)
  float finalTemp = baseTemp + TEMP_OFFSET;
  
  return finalTemp;
}*/

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
  //float temp_c = readCalibratedTemp();
  
  StaticJsonDocument<300> doc;
  doc["voltage"] = busvoltage;
  doc["current_mA"] = current_mA;
  doc["power_mW"] = power_mW;
  doc["temp_c"] = "unavalilable";
  doc["irradiance"] = 0.0;
  doc["mode"] = "AUTO"; 

  String jsonString;
  serializeJson(doc, jsonString);
  server.send(200, "application/json", jsonString);
}

void logDataToFile() {
  float power_mW = ina219.getPower_mW();
  //float temp_c = readCalibratedTemp();
  String timeStr = getLocalTime();

  File file = LittleFS.open("/log_full.txt", "a");
  if (!file) return;

  file.print(timeStr);
  file.print(",");
  //file.print(temp_c, 1); 
  //file.print(",");
  file.println(power_mW, 0); 
  file.close();
}

// --- SETUP ---
void setup() {
  Serial.begin(115200);
  Serial.println("\n--- BOOTING SOLAR TRACKER ---");

  // 1. Filesystem
  if (!LittleFS.begin(false)) LittleFS.begin(true);

  // 2. ADC Calibration (SWITCHED TO 11dB)
  //analogReadResolution(12);
  
  // STRATEGY: Use the widest range (11dB = 0-3.3V)
  // This matches the default ESP32 behavior and prevents "Half Readings"
  //analogSetPinAttenuation(PIN_LM35, ADC_11db); 
  
  //adc_chars = (esp_adc_cal_characteristics_t *)calloc(1, sizeof(esp_adc_cal_characteristics_t));
  
  // Calibrate for 11dB
  //esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);

  // 3. Sensors
  Wire.begin(I2C_SDA, I2C_SCL);
  if (!ina219.begin()) Serial.println("INA219 Not Found");

  // 4. Log File
  if (!LittleFS.exists("/log_full.txt")) {
    File file = LittleFS.open("/log_full.txt", "w");
    file.println("Time,Power(mW)");
    file.close();
  }

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
  server.handleClient();
  
  unsigned long currentMillis = millis();
  if (currentMillis - lastLogTime >= LOG_INTERVAL) {
    lastLogTime = currentMillis;
    if (WiFi.status() == WL_CONNECTED) logDataToFile();
  }
}