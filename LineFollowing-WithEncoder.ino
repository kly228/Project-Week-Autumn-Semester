// ----------------------------------------------------
// LCD SYSTEM (added from Code 1)
// ----------------------------------------------------
#include <LiquidCrystal.h>                 // LCD library
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);       // LCD wiring pins

// Read keypad buttons from A0
int readKey() {
  int adc = analogRead(A0);
  if (adc < 50)  return 0;   // RIGHT
  if (adc < 200) return 1;   // UP
  if (adc < 400) return 2;   // DOWN
  if (adc < 600) return 3;   // LEFT
  if (adc < 800) return 4;   // SELECT
  return -1;
}

// Flags + Timer for LCD start/finish logic
unsigned long startTime = 0;
bool started = false;
bool finished = false;

// For limiting LCD refresh rate
unsigned long lastLCDupdate = 0;
// ----------------------------------------------------


// --------------------
// ENCODER SETUP
// --------------------
#define leftEncoderPin 3
#define rightEncoderPin A3

volatile long leftCount = 0;
volatile long rightCount = 0;


float wheelDiameter = 0.065;        // cm
int pulsesPerRevolution = 20;     
float distancePerPulse_cm = 3.1416 * wheelDiameter / pulsesPerRevolution;  // 1.021 cm/pulse
float distancePerPulse_m  = distancePerPulse_cm / 10.0;                   // 0.01021 m/pulse

// Interrupt functions
void leftEncoderISR() {
  leftCount++;
}

void rightEncoderISR() {
  rightCount++;
}

// --------------------
// 1. PIN DEFINITIONS AND CONSTANTS
// --------------------
#define enA 11 
#define in1 13 
#define in2 12 
#define enB 3  
#define in3 A1 
#define in4 A2 

#define R_S A4 
#define L_S A5 

// --------------------
// 2. SETUP FUNCTION 
// --------------------
void setup() {
  // --- LCD startup screen (ADDED) ---
  lcd.begin(16, 2);
  lcd.clear();
  lcd.print("Press SELECT");
  lcd.setCursor(0, 1);
  lcd.print("to start");
  pinMode(A0, INPUT);  

  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  pinMode(R_S, INPUT);
  pinMode(L_S, INPUT);

  // Encoder pins
  pinMode(leftEncoderPin, INPUT_PULLUP);
  pinMode(rightEncoderPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(leftEncoderPin), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPin), rightEncoderISR, RISING);
}

// --------------------
// 3. MOVEMENT FUNCTIONS
// --------------------

// Moves the robot straight ahead
void forward() {
  analogWrite(enA, 110); 
  analogWrite(enB, 110); 
  
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  
  digitalWrite(in3, LOW); 
  digitalWrite(in4, HIGH);

  // Distance only counted while moving forward
}

// Pivot Turn Left
void turnLeft() {
  analogWrite(enA, 90); 
  analogWrite(enB, 210); 
  
  digitalWrite(in1, LOW); 
  digitalWrite(in2, HIGH); 
  
  digitalWrite(in3, LOW); 
  digitalWrite(in4, HIGH); 
}

// Pivot Turn Right
void turnRight() {
  analogWrite(enA, 210); 
  analogWrite(enB, 90); 
  
  digitalWrite(in1, HIGH); 
  digitalWrite(in2, LOW); 
  
  digitalWrite(in3, HIGH); 
  digitalWrite(in4, LOW); 
}

// Stops both motors instantly
void Stop() {
  analogWrite(enA, 0); 
  analogWrite(enB, 0); 
  
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

// --------------------
// 4. MAIN LOOP 
// --------------------
void loop() {

  // --- LCD start logic ---
  if (!started) {
    if (readKey() == 4) {
      started = true;
      startTime = millis();
      lcd.clear();
      lcd.print("Robot Running");
      lastLCDupdate = millis();
    }
    return;
  }

  int rightSensor = digitalRead(R_S);
  int leftSensor = digitalRead(L_S);

  // --- LIVE RUNNING TIME & DISTANCE DISPLAY ---
  if (!finished) {
    if (millis() - lastLCDupdate >= 200) {  // refresh every 200ms
      lcd.setCursor(0, 0);
      lcd.print("Time: ");
      lcd.print((millis() - startTime)/1000.0, 2);
      lcd.print(" s   ");

      lcd.setCursor(0, 1);
      float leftDist = leftCount * distancePerPulse_m;
      float rightDist = rightCount * distancePerPulse_m;
      float avgDist = (leftDist + rightDist)/2;
      lcd.print("Dist: ");
      lcd.print(avgDist, 2);  
      lcd.print(" m");

      lastLCDupdate = millis();
    }
  }

  // Case 1: Both sensors see the black line -> Go Straight
  if((rightSensor == 1) && (leftSensor == 1)) {
    forward();
  }
  
  // Case 2: Left white, Right black -> Turn Left
  else if((rightSensor == 0) && (leftSensor == 1)) {
    turnLeft();
    delay(100);
  }
  
  // Case 3: Right white, Left black -> Turn Right
  else if((rightSensor == 1) && (leftSensor == 0)) {
    turnRight();
    delay(100);
    while (digitalRead(R_S) == 1) {
      turnRight();
    }
  }
  
  // Case 4: Both sensors see white -> STOP
  else if((rightSensor == 0) && (leftSensor == 0)) {
    Stop();

    // --- LCD final time + distance (meters) ---
    if (!finished) {
      unsigned long total = millis() - startTime;
      float leftDist = leftCount * distancePerPulse_m;
      float rightDist = rightCount * distancePerPulse_m;
      float avgDist = (leftDist + rightDist)/2;

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Time: ");
      lcd.print(total / 1000.0, 2);
      lcd.print(" s");

      lcd.setCursor(0, 1);
      lcd.print("Dist: ");
      lcd.print(avgDist, 2);
      lcd.print(" m");

      finished = true;
    }

    while (1) {}
  }
}
