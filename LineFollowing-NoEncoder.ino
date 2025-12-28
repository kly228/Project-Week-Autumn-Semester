// --------------------
// 1. PIN DEFINITIONS AND CONSTANTS
// --------------------

// L293D Motor Driver Pin Definitions
// Using Digital Pins 13, 12, 11 and Analog Pins A1, A2, 3
#define enA 11 // Enable1 (Right Motor Speed Control - MUST be a PWM pin)
#define in1 13 // Motor1 A (Right Motor Direction)
#define in2 12 // Motor1 B (Right Motor Direction)
#define enB 3  // Enable2 (Left Motor Speed Control - MUST be a PWM pin)
#define in3 A1 // Motor2 A (Left Motor Direction)
#define in4 A2 // Motor2 B (Left Motor Direction)

// IR Sensor Pin Definitions (using Analog Pins as Digital Inputs)
#define R_S A4 // Right sensor
#define L_S A5 // Left sensor

// Speed Constant (0-255). 
// Start with 180 for good control. Increase towards 255 if you need more speed 
// to meet the 60-second limit, but stability will decrease.

// --------------------
// 2. SETUP FUNCTION (Runs Once on power-up)
// --------------------
void setup() {
  // Set motor control pins as OUTPUT
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Set sensor pins as INPUT
  pinMode(R_S, INPUT);
  pinMode(L_S, INPUT);
}

// --------------------
// 3. MOVEMENT FUNCTIONS
// --------------------

// Moves the robot straight ahead at the defined speed
void forward() {
  analogWrite(enA, 100); // Set speed for Right Motor
  analogWrite(enB, 100); // Set speed for Left Motor
  
  // Right Motor Forward
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  
  // Left Motor Forward
  digitalWrite(in3, LOW); 
  digitalWrite(in4, HIGH);
}

// Pivot Turn Right: Left motor moves forward, Right motor moves backward
void turnLeft() {
  analogWrite(enA, 90); 
  analogWrite(enB, 210); 
  
  // Right Motor BACKWARD
  digitalWrite(in1, LOW); 
  digitalWrite(in2, HIGH); 
  
  // Left Motor FORWARD
  digitalWrite(in3, LOW); 
  digitalWrite(in4, HIGH); 
}

// Pivot Turn Left: Right motor moves forward, Left motor moves backward
void turnRight() {
  analogWrite(enA, 210); 
  analogWrite(enB, 90); 
  
  // Right Motor FORWARD
  digitalWrite(in1, HIGH); 
  digitalWrite(in2, LOW); 
  
  // Left Motor BACKWARD
  digitalWrite(in3, HIGH); 
  digitalWrite(in4, LOW); 
  
}

// Stops both motors instantly
void Stop() {
  analogWrite(enA, 0); // Speed 0
  analogWrite(enB, 0); // Speed 0
  
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

// --------------------
// 4. MAIN LOOP (The Decision Logic)
// Sensor Logic: 0 = Black line, 1 = White surface
// --------------------
void loop() {
  int rightSensor = digitalRead(R_S);
  int leftSensor = digitalRead(L_S);

  // Case 1: Both sensors see the black line (0, 0) -> Go Straight
  if((rightSensor == 1) && (leftSensor == 1)) {
    forward();
  }
  
  // Case 2: Left sensor sees white (1), Right sensor sees black (0) 
  //        The robot is drifting LEFT, so Turn Right to get back
  else if((rightSensor == 0) && (leftSensor == 1)) {
    turnLeft();
    delay(100);
    //while (digitalRead(L_S) == 1)
    //{
     // turnLeft();
    //}
  }
  
  // Case 3: Right sensor sees white (1), Left sensor sees black (0) 
  //        The robot is drifting RIGHT, so Turn Left to get back
  else if((rightSensor == 1) && (leftSensor == 0)) {
    turnRight();
    delay(100);
    while (digitalRead(R_S) == 1)
    {
      turnRight();
    }
  }
  
  // Case 4: Both sensors see white (1, 1) -> Lost the line, STOP
  //        A more advanced robot would Reverse/Spin to search for the line, 
  //        but STOP is safer for a time trial.
  else if((rightSensor == 0) && (leftSensor == 0)) {
    Stop();
    while(1)
    {}
  }
}