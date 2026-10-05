#include <Wire.h>
#include <ESP32Servo.h>
#include <math.h>
#include <Preferences.h>

Preferences preferences;

// --- Hardware Pins ---
const int PIN_PROXIMITY = 27; // IR Sensor OUT
const int PIN_IMPACT = 26;    // Microswitch NO
const int PIN_LED_GREEN = 12; // Armed Indicator
const int PIN_LED_RED = 14;   // Detonation Indicator

// --- I2C Addresses ---
const int MPU_ADDR = 0x68;
const int QMC_ADDR = 0x0D;

// --- Servo Objects ---
Servo canardTop, canardRight, canardBottom, canardLeft;

// --- PID Configuration ---
float Kp = 1.3;  
float Ki = 0.0;  
float Kd = 0.0;  

// Target hovering angles 
float targetPitch;
float targetRoll;

// PID memory variables
float pitchErrorSum = 0, rollErrorSum = 0;
float lastPitchError = 0, lastRollError = 0;
unsigned long lastTime = 0;

// --- System States & Modes ---
enum FuzeState { SETUP, ARMED, DELAYING, DETONATED };
enum OperatingMode { NONE, PROXIMITY, IMPACT, DELAY };

volatile FuzeState currentState = SETUP;
OperatingMode currentMode = NONE;

// Flags for interrupt triggers
volatile bool irTriggered = false;
volatile bool switchTriggered = false;

// Variables for Delay Mode
unsigned long impactTime = 0;
const unsigned long delayDuration = 3000; 

// --- Interrupt Service Routines ---
void IRAM_ATTR isrAirburst() {
  if (currentState == ARMED) irTriggered = true;
}

void IRAM_ATTR isrImpact() {
  if (currentState == ARMED) switchTriggered = true;
}

// --- Tilt Sensing Configuration ---
// The MPU axis that points "up" is auto-detected at startup, and tilt is measured
// relative to it. This avoids the atan2(noise, noise) blow-up when upright.
int   upAxis = 0;                  // 0 = X, 1 = Y, 2 = Z (auto-detected)
float upSign = 1.0;                // +1 or -1 depending on which way gravity reads

// Tuning knobs (adjust by testing on the bench)
const bool  SWAP_AXES  = false;    // set true if the wrong servo pair reacts
const float SIGN_PITCH = 1.0;      // set to -1.0 if top/bottom push the wrong way
const float SIGN_ROLL  = 1.0;      // set to -1.0 if left/right push the wrong way
const float FILTER_ALPHA   = 0.15; // low-pass filter (lower = smoother, slower)
const float DEADBAND_DEG   = 0.5;  // ignore errors smaller than this
const float MAX_CORRECTION = 25.0; // max canard deflection from center (degrees)

float filtPitch = 0, filtRoll = 0;

void readAcc(float acc[3]) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);
  for (int i = 0; i < 3; i++) {
    acc[i] = (int16_t)(Wire.read() << 8 | Wire.read());
  }
}

// Tilt angles relative to whichever axis currently points up
void computeTilt(const float acc[3], float &pitch, float &roll) {
  int a = (upAxis + 1) % 3;
  int b = (upAxis + 2) % 3;
  float up = acc[upAxis] * upSign;
  float tA = atan2(acc[a] * upSign, up) * 180.0 / PI;
  float tB = atan2(acc[b] * upSign, up) * 180.0 / PI;

  if (SWAP_AXES) { pitch = tB; roll = tA; }
  else           { pitch = tA; roll = tB; }

  pitch *= SIGN_PITCH;
  roll  *= SIGN_ROLL;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  pinMode(PIN_PROXIMITY, INPUT_PULLUP);
  pinMode(PIN_IMPACT, INPUT_PULLUP);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_RED, OUTPUT);
  
  digitalWrite(PIN_LED_GREEN, LOW);
  digitalWrite(PIN_LED_RED, LOW);

  // --- Initialize Sensors ---
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); 
  Wire.write(0);    
  Wire.endTransmission(true);

  Wire.beginTransmission(QMC_ADDR);
  Wire.write(0x09); 
  Wire.write(0x1D); 
  Wire.endTransmission(true);

  // --- Calibrate Baseline Target Orientation ---
  Serial.println("\n[!] Calibrating IMU baseline. Do not move the system...");
  float meanAcc[3] = {0, 0, 0};
  int numSamples = 50;

  for (int i = 0; i < numSamples; i++) {
    float acc[3];
    readAcc(acc);
    for (int k = 0; k < 3; k++) meanAcc[k] += acc[k] / numSamples;
    delay(10); 
  }

  // The axis carrying gravity while upright is the "up" axis
  upAxis = 0;
  for (int k = 1; k < 3; k++) {
    if (fabs(meanAcc[k]) > fabs(meanAcc[upAxis])) upAxis = k;
  }
  upSign = (meanAcc[upAxis] >= 0) ? 1.0 : -1.0;

  computeTilt(meanAcc, targetPitch, targetRoll);
  filtPitch = targetPitch;
  filtRoll  = targetRoll;

  Serial.print("Up axis detected: ");
  Serial.println("XYZ"[upAxis]);
  Serial.print("Target Locked -> Pitch: ");
  Serial.print(targetPitch);
  Serial.print(" | Roll: ");
  Serial.println(targetRoll);

  // --- Initialize Servos ---
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  canardTop.attach(19, 500, 2400);
  canardRight.attach(32, 500, 2400);
  canardBottom.attach(23, 500, 2400);
  canardLeft.attach(18, 500, 2400);

  // >>> NEW: Instantly lock canards to straight upright (90 degrees) on power up
  canardTop.write(90);
  canardRight.write(90);
  canardBottom.write(90);
  canardLeft.write(90);

  // Attach Interrupts
  attachInterrupt(digitalPinToInterrupt(PIN_PROXIMITY), isrAirburst, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_IMPACT), isrImpact, FALLING);
  
  // --- Auto-Load / User Selection Interface ---
  preferences.begin("fuze_data", false); 
  
  Serial.println("\n==================================");
  Serial.println("   PGK FLIGHT COMPUTER ONLINE     ");
  Serial.println("==================================");
  Serial.println("You have 10 SECONDS to change the mode via Serial.");
  Serial.println(" [1] Proximity | [2] Impact | [3] Delay");
  Serial.println("If no input is detected, the last saved mode will load automatically.");
  
  unsigned long startTime = millis();
  bool modeChanged = false;
  
  while (millis() - startTime < 5000) {
    if (Serial.available() > 0) {
      char incoming = Serial.read();
      if (incoming == '1') { currentMode = PROXIMITY; modeChanged = true; break; }
      else if (incoming == '2') { currentMode = IMPACT; modeChanged = true; break; }
      else if (incoming == '3') { currentMode = DELAY; modeChanged = true; break; }
    }
  }
  
  if (modeChanged) {
    preferences.putInt("savedMode", currentMode);
    Serial.print("\n>>> NEW MODE SAVED TO FLASH: ");
    Serial.println(currentMode);
  } else {
    int loadedMode = preferences.getInt("savedMode", 1); 
    if (loadedMode == 1) currentMode = PROXIMITY;
    else if (loadedMode == 2) currentMode = IMPACT;
    else if (loadedMode == 3) currentMode = DELAY;
    
    Serial.print("\n>>> TIMER EXPIRED. LOADED SAVED MODE: ");
    Serial.println(currentMode);
  }
  
  // --- Arming Sequence ---
  Serial.println("\nMode locked. Simulating launch sequence...");
  for (int i = 5; i > 0; i--) {
    Serial.print("Arming in "); Serial.print(i); Serial.println("...");
    delay(1000);
  }
  
  currentState = ARMED;
  digitalWrite(PIN_LED_GREEN, HIGH);
  
  Serial.println("\n*** SYSTEM ARMED - STARTING TELEMETRY & PID LOOP ***");
  delay(1000); 
  
  lastTime = millis();
}

void loop() {
  if (currentState == DETONATED) {
    delay(100);
    return; 
  }

  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  if (dt <= 0.0) return; 
  lastTime = now;

  // Check Fuze Logic
  if (currentState == ARMED) {
    if (currentMode == PROXIMITY && irTriggered) {
      executeDetonation("AIRBURST (Proximity threshold reached)");
    } 
    else if (currentMode == IMPACT && switchTriggered) {
      executeDetonation("POINT DETONATING (Kinetic impact on nose)");
    } 
    else if (currentMode == DELAY && switchTriggered) {
      Serial.println("Impact detected! Delay fuse initiated. Counting 3 seconds...");
      impactTime = millis();
      currentState = DELAYING;
      switchTriggered = false; 
    }
  }
  
  if (currentState == DELAYING) {
    if (millis() - impactTime >= delayDuration) {
      executeDetonation("DELAY (3 seconds elapsed post-impact)");
    }
  }

  // Read MPU-6050 Accelerometer
  float acc[3];
  readAcc(acc);

  // Read QMC5883L Magnetometer
  Wire.beginTransmission(QMC_ADDR);
  Wire.write(0x00);
  Wire.endTransmission(false);
  Wire.requestFrom(QMC_ADDR, 6, true);

  int16_t magX = Wire.read() | (Wire.read() << 8);
  int16_t magY = Wire.read() | (Wire.read() << 8);
  int16_t magZ = Wire.read() | (Wire.read() << 8);

  // Calculate Angles
  float rawPitch, rawRoll;
  computeTilt(acc, rawPitch, rawRoll);

  // Low-pass filter to calm accelerometer noise
  filtPitch = (1.0 - FILTER_ALPHA) * filtPitch + FILTER_ALPHA * rawPitch;
  filtRoll  = (1.0 - FILTER_ALPHA) * filtRoll  + FILTER_ALPHA * rawRoll;
  float pitch = filtPitch;
  float roll  = filtRoll;

  float yaw   = atan2(magY, magX) * 180.0 / PI;
  if (yaw < 0) yaw += 360.0;

  // Stream Telemetry
  Serial.print(pitch);
  Serial.print(",");
  Serial.print(yaw);
  Serial.print(",");
  Serial.println(roll);

  // Compute PID Errors
  float pitchError = targetPitch - pitch;
  float rollError = targetRoll - roll;
  if (fabs(pitchError) < DEADBAND_DEG) pitchError = 0;
  if (fabs(rollError)  < DEADBAND_DEG) rollError  = 0;

  pitchErrorSum += (pitchError * dt);
  rollErrorSum += (rollError * dt);

  float pitchRate = (pitchError - lastPitchError) / dt;
  float rollRate = (rollError - lastRollError) / dt;

  lastPitchError = pitchError;
  lastRollError = rollError;

  // Calculate PID Output
  float pitchCorrection = (Kp * pitchError) + (Ki * pitchErrorSum) + (Kd * pitchRate);
  float rollCorrection = (Kp * rollError) + (Ki * rollErrorSum) + (Kd * rollRate);

  pitchCorrection = constrain(pitchCorrection, -MAX_CORRECTION, MAX_CORRECTION);
  rollCorrection  = constrain(rollCorrection,  -MAX_CORRECTION, MAX_CORRECTION);

  // Actuator Mixing Logic
  int outTop = constrain(90 - pitchCorrection, 0, 180);
  int outBottom = constrain(90 + pitchCorrection, 0, 180);
  int outRight = constrain(90 - rollCorrection, 0, 180);
  int outLeft = constrain(90 + rollCorrection, 0, 180);

  // Command the Servos
  canardTop.write(outTop);
  canardBottom.write(outBottom);
  canardRight.write(outRight);
  canardLeft.write(outLeft);

  delay(20); 
}

// --- Detonation Execution ---
void executeDetonation(String reason) {
  currentState = DETONATED;
  
  digitalWrite(PIN_LED_GREEN, LOW);
  digitalWrite(PIN_LED_RED, HIGH);
  
  // >>> NEW: Snap canards back to straight upright upon detonation
  canardTop.write(90);
  canardRight.write(90);
  canardBottom.write(90);
  canardLeft.write(90);
  
  Serial.println();
  Serial.println("====================================");
  Serial.println(">>> DETONATION TRIGGERED <<<");
  Serial.println("Reason: " + reason);
  Serial.println("====================================");
  Serial.println();
}