#include <LiquidCrystal.h>
#include <Wire.h>
#include <MPU6050.h>

// ---------------------------------------------------------
// 1. HARDWARE & PIN DEFINITIONS (Based on your Robot)
// ---------------------------------------------------------
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
MPU6050 mpu;

// Motor Pins
#define enA 11
#define in1 13
#define in2 12
#define enB 3  // Note: Ensure no conflict if using encoders later
#define in3 A1
#define in4 A2
#define KEY_PIN A0

// ---------------------------------------------------------
// 2. CONSTANTS & VARIABLES
// ---------------------------------------------------------
const int RAMP_THRESHOLD = 15;  // Angle to detect ramp start
const int FLAT_THRESHOLD = 8;   // Angle to detect top
const int MOTOR_SPEED = 200;    // Adjusted for safety
const int TURN_SPEED = 180;     

// MPU Variables
int16_t ax, ay, az, gx, gy, gz;
float pitch = 0; 
float yaw = 0;
float maxRampAngle = 0; // To store the steepest angle detected

// Calibration Offsets
float gyroXoffset = 0, gyroYoffset = 0, gyroZoffset = 0;
unsigned long prevTime = 0;

// State Machine
enum State { WAIT, DRIVE, CLIMB, STOP_MEASURE, STOP_TOP, ROTATE, DONE };
State currentState = WAIT;
unsigned long stateTimer = 0;

// ---------------------------------------------------------
// 3. SETUP & CALIBRATION
// ---------------------------------------------------------
void setup() {
  Wire.begin();
  lcd.begin(16, 2);
  Serial.begin(9600);
  
  // Motor Setup
  pinMode(enA, OUTPUT); pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT); pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);
  pinMode(KEY_PIN, INPUT);

  // Initialize MPU
  lcd.print("Init MPU...");
  mpu.initialize();
  
  // --- CALIBRATION ROUTINE ---
  // Robot must be still. We take 500 samples to find zero-error.
  lcd.setCursor(0, 1);
  lcd.print("Calibrating...");
  long gx_sum = 0, gy_sum = 0, gz_sum = 0;
  for (int i = 0; i < 500; i++) {
    mpu.getRotation(&gx, &gy, &gz);
    gx_sum += gx; 
    gy_sum += gy; 
    gz_sum += gz;
    delay(2);
  }
  gyroXoffset = gx_sum / 500.0;
  gyroZoffset = gz_sum / 500.0; // Important for accurate 360 turn

  lcd.clear();
  lcd.print("Ready. Press Btn");
  prevTime = millis();
}

// ---------------------------------------------------------
// 4. MAIN LOOP
// ---------------------------------------------------------
void loop() {
  // Update sensor readings every loop
  updatePitchYaw();

  switch (currentState) {
    
    // --- STATE: WAIT FOR START ---
    case WAIT:
      if (readKey()) { 
        lcd.clear(); 
        lcd.print("Go -> Ramp");
        currentState = DRIVE;
        delay(500); // Debounce
      }
      break;

    // --- STATE: APPROACHING RAMP ---
    case DRIVE:
      forward();
      // If pitch exceeds 15 degrees, we are on the ramp
      if (pitch > RAMP_THRESHOLD) {
        lcd.clear(); 
        lcd.print("Climbing...");
        currentState = CLIMB;
      }
      break;

    // --- STATE: CLIMBING RAMP ---
    case CLIMB:
      forward();
      
      // Record the maximum angle seen during the climb
      if (pitch > maxRampAngle) {
        maxRampAngle = pitch;
      }

      // If angle drops below 8 degrees, we reached the top
      if (pitch < FLAT_THRESHOLD) {
        Stop();
        delay(500); // Wait for robot to settle
        currentState = STOP_MEASURE;
      }
      break;

    // --- STATE: VERIFY TOP ---
    case STOP_MEASURE:
      // Double check angle while stopped to ensure it wasn't a bump
      if (pitch < FLAT_THRESHOLD) {
        lcd.clear(); 
        lcd.print("Top Reached");
        lcd.setCursor(0, 1);
        lcd.print("Wait 4s...");  
        stateTimer = millis();
        currentState = STOP_TOP; 
      } else {
        // False alarm, keep climbing
        currentState = CLIMB;
      }
      break;

    // --- STATE: WAIT 4 SECONDS ---
    case STOP_TOP: 
      Stop();
      if (millis() - stateTimer > 4000) {
        lcd.clear(); 
        lcd.print("Rotating 360...");
        yaw = 0; // Reset yaw counter for the turn
        currentState = ROTATE;
      }
      break;

    // --- STATE: 360 ROTATION ---
    case ROTATE:
      turnLeft();
      
      // Turn until Yaw hits ~350 (allow momentum to finish the 360)
      if (abs(yaw) >= 350) { 
        Stop();
        delay(500);
        
        // Display the MAX angle recorded during climb
        lcd.clear();
        lcd.print("Max Angle:"); 
        lcd.setCursor(0, 1);
        lcd.print(maxRampAngle);
        lcd.print(" deg");
        
        currentState = DONE;
      }
      break;

    // --- STATE: FINISHED ---
    case DONE:
      Stop(); 
      // Do nothing, just display result
      break;
  }
}

// ---------------------------------------------------------
// 5. HELPER FUNCTIONS
// ---------------------------------------------------------

void updatePitchYaw() {
  // Calculate delta time
  unsigned long currTime = millis();
  float dt = (currTime - prevTime) / 1000.0;
  prevTime = currTime;

  // Get Raw Data
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // 1. Calculate Pitch from Accelerometer (Trig)
  float accPitch = atan2(-ax, sqrt(ay*(long)ay + az*(long)az)) * 180.0 / PI;

  // 2. Calculate Gyro Rates (deg/sec)
  float gyroPitchRate = (gy - gyroYoffset) / 131.0;
  float gyroYawRate = (gz - gyroZoffset) / 131.0;

  // 3. Complementary Filter
  // Pitch combines Accel (stable long term) + Gyro (fast response)
  pitch = 0.98 * (pitch + gyroPitchRate * dt) + 0.02 * accPitch;
  
  // 4. Integrate Yaw (Gyro only) for rotation tracking
  // We ignore small noise (< 1.0 deg/s) to prevent drift while stopped
  if (abs(gyroYawRate) > 1.0) {
    yaw += gyroYawRate * dt;
  }
}

// Button Reader (Detects Select Button on Analog Keypad)
bool readKey() {
  int adc = analogRead(KEY_PIN);
  // 'Select' is usually around 600-800 on LCD shields
  if (adc < 800) return true; 
  return false;
}

// Motor Functions
void forward() {
  analogWrite(enA, MOTOR_SPEED); analogWrite(enB, MOTOR_SPEED);
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);  digitalWrite(in4, HIGH);
}

void turnLeft() {
  analogWrite(enA, TURN_SPEED); analogWrite(enB, TURN_SPEED);
  digitalWrite(in1, LOW);  digitalWrite(in2, HIGH); // Left Back
  digitalWrite(in3, LOW);  digitalWrite(in4, HIGH); // Right Fwd
}

void Stop() {
  analogWrite(enA, 0); analogWrite(enB, 0);
  digitalWrite(in1, LOW); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); digitalWrite(in4, LOW);
}