#include <LiquidCrystal.h>
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// --------------------
// VARIABLES & PINS
// --------------------
// *** WARNING: You have a Pin 3 conflict (enB and LeftEncoder). 
// Move enB to Pin 10 if using an Arduino Uno to ensure both work. ***
#define enA 11 
#define in1 13 
#define in2 12 
#define enB 3  // Conflict with encoder! Suggest changing to 10
#define in3 A1 
#define in4 A2 
#define R_S A4 
#define L_S A5 

// Encoders
#define leftEncoderPin 3
#define rightEncoderPin A3
volatile long leftCount = 0;
volatile long rightCount = 0;

// Wheel Constants
float wheelDiameter = 0.065; 
int pulsesPerRevolution = 20;     
float distancePerPulse_m = (3.1416 * wheelDiameter / pulsesPerRevolution) / 10.0;                 

// State Flags
unsigned long startTime = 0;
bool started = false;
bool finished = false;
bool hasPaused = false; // Flag for the 2.7m stop
unsigned long lastLCDupdate = 0;

// --------------------
// INTERRUPTS
// --------------------
void leftEncoderISR() { leftCount++; }
void rightEncoderISR() { rightCount++; }

// --------------------
// SETUP
// --------------------
void setup() {
  lcd.begin(16, 2);
  lcd.clear();
  lcd.print("Press SELECT");
  
  pinMode(A0, INPUT);
  pinMode(enA, OUTPUT); pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT); pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);
  pinMode(R_S, INPUT);  pinMode(L_S, INPUT);

  pinMode(leftEncoderPin, INPUT_PULLUP);
  pinMode(rightEncoderPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(leftEncoderPin), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPin), rightEncoderISR, RISING);
}

// --------------------
// MOVEMENT
// --------------------
void forward() {
  analogWrite(enA, 110); analogWrite(enB, 110); 
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);  digitalWrite(in4, HIGH);
}
void turnLeft() {
  analogWrite(enA, 90); analogWrite(enB, 210); 
  digitalWrite(in1, LOW); digitalWrite(in2, HIGH); 
  digitalWrite(in3, LOW); digitalWrite(in4, HIGH); 
}
void turnRight() {
  analogWrite(enA, 210); analogWrite(enB, 90); 
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW); 
  digitalWrite(in3, HIGH); digitalWrite(in4, LOW); 
}
void Stop() {
  analogWrite(enA, 0); analogWrite(enB, 0); 
  digitalWrite(in1, LOW); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); digitalWrite(in4, LOW);
}

// --------------------
// MAIN LOOP
// --------------------
void loop() {
  // 1. Start Button Logic
  if (!started) {
    int adc = analogRead(A0);
    if (adc < 800 && adc > 600) { // Select button range
      started = true;
      startTime = millis();
      lcd.clear();
      lcd.print("Robot Running");
    }
    return;
  }

  // 2. Calculate Distance
  float avgDist = ((leftCount + rightCount) * distancePerPulse_m) / 2.0;

  // 3. CHECK DISTANCE LIMIT (2.7m)
  if (!hasPaused && avgDist >= 2.7) {
    Stop();
    
    // --- Display Info ---
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Paused : 2s");
    
    lcd.setCursor(0, 1);
    lcd.print("Dist : ");
    lcd.print(avgDist, 1); // Prints "2.7"
    lcd.print("m");
    
    // --- Pause ---
    delay(2000); 
    
    // --- Resume ---
    hasPaused = true; 
    lcd.clear();
    lcd.print("Resuming...");
    lastLCDupdate = millis(); // Reset timer to prevent immediate flicker
  }

  // 4. Update LCD (Live Data)
  if (!finished && hasPaused) { // Only show live data after pause or if not paused yet
     // Note: If you want live data BEFORE 2.7m, remove "&& hasPaused"
     // But usually, the loop runs fast.
  }
  
  // Standard LCD Refresh (every 200ms)
  if (!finished && (millis() - lastLCDupdate >= 200)) {
      lcd.setCursor(0, 0);
      lcd.print("Time: "); lcd.print((millis() - startTime)/1000.0, 1); lcd.print("s ");
      lcd.setCursor(0, 1);
      lcd.print("Dist: "); lcd.print(avgDist, 2); lcd.print(" m");
      lastLCDupdate = millis();
  }

  // 5. Line Follow Logic
  int r = digitalRead(R_S);
  int l = digitalRead(L_S);

  if(r == 1 && l == 1) { forward(); }
  else if(r == 0 && l == 1) { turnLeft(); delay(100); }
  else if(r == 1 && l == 0) { 
    turnRight(); 
    delay(100); 
    while(digitalRead(R_S) == 1) { turnRight(); } 
  }
  else if(r == 0 && l == 0) {
    Stop();
    if (!finished) {
      lcd.clear();
      lcd.print("Time: "); lcd.print((millis() - startTime)/1000.0, 2); lcd.print("s");
      lcd.setCursor(0, 1);
      lcd.print("Dist: "); lcd.print(avgDist, 2); lcd.print(" m");
      finished = true;
    }
    while(1);
  }
}





// --- 2.7 METER STOP TASK ---

// Check if we reached 2.7m and haven't paused yet
if (!hasPaused && avgDist >= 2.7) {
  
  Stop(); // Stop motors immediately
  
  // Update LCD Display
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Paused : 2s");
  
  lcd.setCursor(0, 1);
  lcd.print("Dist : ");
  lcd.print(avgDist, 1); // Displays 2.7
  lcd.print("m");
  
  delay(2000); // Wait for 2 seconds
  
  // Set flag to true so this block never runs again
  hasPaused = true; 
  
  // Optional: clear screen before resuming to avoid text clash
  lcd.clear();
  lcd.print("Resuming...");
  lastLCDupdate = millis(); 
}