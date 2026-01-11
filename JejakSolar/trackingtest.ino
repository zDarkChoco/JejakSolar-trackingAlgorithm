/*
 * Dual-Axis LDR Tracker - Logic Test
 * Reads 4 LDRs and prints intended actions to the Serial Monitor.
 */

// === 1. Define Pins ===
// Use ADC1 pins (GPIOs 32-39) for best results

#include <ESP32Servo.h>

Servo myServoX;
Servo myServoY;

const int LDR_TL_PIN = 33; // Top-Left LDR
const int LDR_TR_PIN = 32; // Top-Right LDR
const int LDR_BL_PIN = 35; // Bottom-Left LDR
const int LDR_BR_PIN = 34; // Bottom-Right LDR
const int servoPinX = 25; 
const int servoPinY = 26;



// === 2. Define Tracking Parameters ===
// This "dead-zone" stops the servos from "hunting" or twitching for
// tiny light changes. Increase it if the system is too sensitive.
int tolerance = 150; 
int CurX = 90;
int CurY = 90;
int rate = 5;

// === 3. Setup Function ===
void setup() {
  Serial.begin(115200); // Start serial comms for debugging
  Serial.println("\n--- Dual-Axis Tracker Logic Test ---");
  Serial.println("Aim a light source at the sensors.");
  myServoX.attach(servoPinX);
  myServoY.attach(servoPinY);
  myServoX.write(CurX); 
  myServoY.write(CurY);
}

// === 4. Main Loop ===
void loop() {
  // Call the main tracking function
  trackSun();
  
  // Wait a moment so we don't spam the Serial Monitor
  delay(200); 
}

// === 5. Tracking Logic Function ===
void trackSun() {
  // --- A. Read Sensor Values ---
  int val_TL = analogRead(LDR_TL_PIN);
  int val_TR = analogRead(LDR_TR_PIN);
  int val_BL = analogRead(LDR_BL_PIN);
  int val_BR = analogRead(LDR_BR_PIN);

  // --- B. Print Raw Values (for testing) ---
  // This is how you confirm your sensors are working!
  Serial.println("---");
  Serial.print("LDR_TL: "); Serial.print(val_TL);
  Serial.print("\t LDR_TR: "); Serial.println(val_TR);
  Serial.print("LDR_BL: "); Serial.print(val_BL);
  Serial.print("\t LDR_BR: "); Serial.println(val_BR);
  Serial.println("---");

  // --- C. Calculate Averages ---
  int avg_top = (val_TL + val_TR) / 2;
  int avg_bottom = (val_BL + val_BR) / 2;
  int avg_left = (val_TL + val_BL) / 2;
  int avg_right = (val_TR + val_BR) / 2;

  // --- D. Calculate Error ---
  // Positive error means top/left is brighter
  // Negative error means bottom/right is brighter
  int vertical_error = avg_top - avg_bottom;
  int horizontal_error = avg_left - avg_right;

  // --- E. Make Decisions & Print Actions ---
  // This replaces the servo-moving code for now.
  myServoX.write(CurX); 
  myServoY.write(CurY);

  // 1. Vertical (Tilt) Motor
  if (vertical_error > tolerance) {
    Serial.println("ACTION: Move DOWN (Top is brighter)");
    CurX++;
    myServoX.write(CurX); 
    Serial.println(CurX);
  } 
  else if (vertical_error < -tolerance) {
    
    CurX--;
    myServoX.write(CurX);
    Serial.println("ACTION: Move UP (Bottom is brighter)");
    Serial.println(CurX);
  } 
  else {
    Serial.println("ACTION: (Vertical OK)");
  }

  // 2. Horizontal (Pan) Motor
  if (horizontal_error > tolerance) {
    Serial.println("ACTION: Move RIGHT (Left is brighter)");
    CurY--;
    myServoY.write(CurY); 
    Serial.println(CurY);
  } 
  else if (horizontal_error < -tolerance) {
    Serial.println("ACTION: Move LEFT (Right is brighter)");
    CurY++;
    myServoY.write(CurY);
    Serial.println(CurY);
  } 
  else {
    Serial.println("ACTION: (Horizontal OK)");
  }
}