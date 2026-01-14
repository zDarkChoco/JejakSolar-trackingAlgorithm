/*
 * Dual-Axis Solar Tracker with Zenith Inversion Logic
 * Platform: ESP32
 * * Logic:
 * - Reads 4 LDRs (Top-Left, Top-Right, Bottom-Left, Bottom-Right).
 * - Y-Axis (Tilt): Moves 0-180. 90 is Up. Handles East-West arc.
 * - X-Axis (Pan): Handles North-South correction.
 * - INVERSION: When Y-Axis > 90 (facing East), X-Axis controls flip.
 */

#include <ESP32Servo.h>

// --- 1. Pin Definitions ---
// LDR Analog Inputs


// Servo Outputs
const int PIN_SERVO_Y = 25; // Tilt (Elevation/East-West)
const int PIN_SERVO_X = 26; // Pan (Azimuth/North-South)

// --- 2. Configuration Parameters ---
// Servo Limits (User defined: 20 to 160)
const int LIM_MIN = 20;
const int LIM_MAX = 160;

// Movement Speed (Delay in ms between updates)
const int SPEED_DELAY = 200; 

// Sensitivity (Deadband) - prevents jitter
const float TOLERANCE = 0.03; 

// --- 3. Global Variables ---
Servo servoX;
Servo servoY;

// Initial positions (Start at Zenith/West-ish)
int posX = 90; 
int posY = 90;

struct LDRSensor {
  int pin;          // GPIO Pin
  int minVal;       // Raw value in complete darkness (Calibrated)
  int maxVal;       // Raw value in full saturation (Calibrated)
};

// INITIALIZE SENSORS (Update these values after your Calibration Test)
// Format: {Pin, Min_Dark_Value, Max_Bright_Value}
LDRSensor sensors[4] = {
  {34, 50, 3100},  // Top Left (LDR1)
  {32, 280, 4095},  // Top Right (LDR2)
  {35, 180, 4095},  // Bottom Left (LDR3 - The Mismatched One)
  {33, 240, 4095}   // Bottom Right (LDR4)
};

// --- NORMALIZED READING FUNCTION ---

float readNormalizedLDR(int sensorIndex) {
  int raw = analogRead(sensors[sensorIndex].pin);
  
  // Constrain values to prevent math errors if raw data exceeds expected range
  raw = constrain(raw, sensors[sensorIndex].minVal, sensors[sensorIndex].maxVal);

  // Map raw value to 0.0 - 1.0 (Float precision for better PID control)
  // Formula: (Raw - Min) / (Max - Min)
  float normalized = (float)(raw - sensors[sensorIndex].minVal) / (float)(sensors[sensorIndex].maxVal - sensors[sensorIndex].minVal);
                     
  return normalized;
}

void setup() {
  Serial.begin(115200);
  Serial.println("--- Starting Solar Tracker ---");

  // Attach Servos
  servoX.attach(PIN_SERVO_X);
  servoY.attach(PIN_SERVO_Y);

  // Move to initial position
  servoX.write(posX);
  servoY.write(posY);
  
  
  delay(1000); // Startup delay
}

void loop() {
  // --- A. Read Sensors ---
  // Note: ESP32 ADC is non-linear, but sufficient for relative tracking.
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
  
  if (abs(diffY) > TOLERANCE) {
    if (avgTop > avgBot) {
      // Top is brighter. If 90 is UP and 20 is WEST, we usually need to move closer to 90?
      // You may need to SWAP these signs based on your physical motor mounting.
      posY++; 
    } else {
      // Bottom is brighter
      posY--;
    }
    if (posY > LIM_MAX) posY = LIM_MAX;
    if (posY < LIM_MIN) posY = LIM_MIN;
    servoY.write(posY);
  }

  // Constrain Y to user limits
  

  // --- D. Logic for X-Axis (Pan) WITH INVERSION ---
  float diffX = avgLeft - avgRight;

  Serial.print(" | diffX: "); Serial.println(diffX);

  if (abs(diffY) <= TOLERANCE && abs(diffX) > TOLERANCE) {
    
    // Determine Tracking Direction
    bool moveLeft = (avgLeft > avgRight); // Light is on Left
    bool moveRight = (avgRight > avgLeft); // Light is on Right
    
    // CHECK ORIENTATION: Is the panel flipped over the top?
    // If Y > 90, we are "looking back" (East), so Left/Right commands must swap.
    bool isFlipped = (posY > 90); 

    if (moveLeft) {
      
        if (isFlipped) {
        // Inverted: Light is Left (sensor), but physically we must turn Right
        posX++; 
      } else {
        // Normal: Light is Left, turn Left
        posX--; 
      } 
      
    } 
    else if (moveRight) {
      
        // Normal: Light is Right, turn Right
       if (isFlipped) {
        // Inverted: Light is Right (sensor), but physically we must turn Left
        posX--; 
      } else {
        // Normal: Light is Right, turn Right
        posX++; 
      } 
      
    }
    if (posX > LIM_MAX) posX = LIM_MAX;
    if (posX < LIM_MIN) posX = LIM_MIN;
    servoX.write(posX);
  }

  // Constrain X to user limits
  

  // --- E. Debugging (Optional) ---
   Serial.println("Y: "); Serial.print(posY);
   Serial.print(" | X: "); Serial.print(posX);
   Serial.print(" | Flipped: "); Serial.println(posY > 90);

  delay(SPEED_DELAY);
}
