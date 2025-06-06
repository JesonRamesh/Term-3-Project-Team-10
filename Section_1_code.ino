#include <Servo.h>
#include <Wire.h>
#include <Motoron.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20649.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#include <Arduino.h>


//FUNCTION DECLARING
void limbs_vertical();
void limbs_incline();
void scissor_lift_extended();
void scissor_lift_initial();
void kill_signal();
uint16_t read_sensor();
void read_middle_sensors(uint16_t *dest);
uint16_t read_sensor(uint8_t pin);
void read_front_sensors(uint16_t *dest);
void calibrate_middle_sensors();
void calibrate_front_sensors();
bool check_front_white(uint16_t *frontSensors);
bool check_front_black(uint16_t *frontSensors);
bool check_middle_white(uint16_t *middleSensors);
int compute_error(uint16_t *sens);
void turnUntilBlack(bool turnRight);
void line_following();


// === LINE FOLLOWING CONSTANTS ===
// Configuring the sensors
const uint8_t middleSensorPins[9] = {43, 44, 45, 46, 47, 48, 49, 50, 51}; // 9 array sensor located in the middle
const uint8_t frontSensorPins[2] = {4, 5}; // 2 array sensor located in the front


const uint16_t TIME_CALIB = 3000; // Time for each sensor calibration
const uint16_t NUM_CALIB = 3000; // Number of calibrations taken for each sensor


// To store the max and min pulse durations recorded during the calibration process for normalisation
uint16_t middleMin[9], middleMax[9];
uint16_t frontMin[2], frontMax[2];


// Weights assigned to each sensor to calculate error
const int sensor_weight[7] = {-3, -2, -1, 0, 1, 2, 3};


// PD controller tuning
float Kp = 17;
float Kd = 2.0;


int base_speed = 250; // Base speed of the robot
int max_speed = 350; // Max speed constraint
int lastError = 0; // To store previous error


// Constants to store Motor pins
const uint8_t LEFT_MOTOR = 1;
const uint8_t RIGHT_MOTOR = 3;


// --- Create a servo object ---
Servo servo1;
Servo servo2;
Servo servo3;


// --- Set Shield Address ---
MotoronI2C mc1(0x10);


// --- WiFi & UDP ---
const char* ssid = "PhaseSpaceNetwork_2.4G";
const char* password = "8igMacNet";
WiFiUDP udp;
unsigned int localPort = 55500;
char incomingPacket[255];


// --- Button ---
const int buttonPin = 2;
bool lastButtonState = HIGH;
bool stopSignal = false;


void setup() {
  Serial.begin(9600);


  // --- WiFi ---
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
  delay(500); Serial.print(".");
  }
  Serial.println("\nConnected!");
  udp.begin(localPort);
  Serial.print("Listening on UDP port ");
  Serial.println(localPort);

  // --- Servo Limbs Setup ---
  servo1.attach(30);
  servo2.attach(31);
  // --- Servo Scissor Lift Setup ---
  servo3.attach(32);

  // --- Shield 1 setup ---
  Wire1.begin(); // Manually change I2C communication ports to SDA1 and SCL1 on Giga
  mc1.setBus(&Wire1);
  mc1.reinitialize();
  mc1.disableCrc();
  mc1.clearResetFlag();
  mc1.disableCommandTimeout();
  for (uint8_t ch = 1; ch <= 3; ch++) {
    mc1.setMaxAcceleration(ch, 100);
    mc1.setMaxDeceleration(ch, 100);
  }

  // --- Button ---
  pinMode(buttonPin, INPUT_PULLUP);


  limbs_vertical();
  scissor_lift_initial();


 // Calibrate reflectance sensors
  calibrate_middle_sensors();
  calibrate_front_sensors();
  delay(2000);
}


void loop(){



  kill_signal();
  // --- Act Based on stopSignal ---
  if (stopSignal) {
    mc1.setSpeed(1, 0);
    mc1.setSpeed(3, 0);
    Serial.println("Motors Stopped");
  } else {
    //normal robot operation
    line_following();
  }
  }

  // === Operation Functions ===
  void kill_signal(){
    // --- WiFi Packet Check ---
    int packetSize = udp.parsePacket();
    if (packetSize) {
      int len = udp.read(incomingPacket, 255);
      if (len > 0) incomingPacket[len] = 0;
    Serial.print("Received: ");
    Serial.println(incomingPacket);
    if (strcmp(incomingPacket, "Stop") == 0) {
      stopSignal = true;
      Serial.println("!!! Emergency Stop Activated via WiFi !!!");
    }
  }

  // --- Button Toggle Handling ---
  bool currentButtonState = digitalRead(buttonPin);
  if (lastButtonState == HIGH && currentButtonState == LOW) {
  // Button just pressed
  stopSignal = !stopSignal; // Toggle
  Serial.println(stopSignal ? "!!! Emergency Stop Activated via Button !!!"
  : "Robot Restarted via Button.");
  delay(300); // Debounce delay
  }
  lastButtonState = currentButtonState;
  Serial.print("stopSignal: ");
  Serial.println(stopSignal);
}

void limbs_vertical(){
  servo1.write(140);
  servo2.write(0);
}
void limbs_incline(){
  servo1.write(110);
  servo2.write(40);
}


void scissor_lift_initial(){
  servo3.write(80);
}
void scissor_lift_extended(){
  servo3.write(65);
}

// == Note: sensor reading and calibration functions were AI-generated ==
// Reading sensor data
uint16_t read_sensor(uint8_t pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  delayMicroseconds(10);
  pinMode(pin, INPUT);


  // Reads the pulse duration from a single reflectance sensor on a given pin.
  uint32_t t0 = micros();
  while (digitalRead(pin)) {
    if (micros() - t0 > TIME_CALIB) return TIME_CALIB;
  }
  return micros() - t0;
}

void read_middle_sensors(uint16_t *dest) {
  for (uint8_t i = 0; i < 9; i++)
    dest[i] = read_sensor(middleSensorPins[i]); // Read each sensor in the 9 array
}


void read_front_sensors(uint16_t *dest) {
  for (uint8_t i = 0; i < 2; i++)
    dest[i] = read_sensor(frontSensorPins[i]); // Read each sensor in the 2 array
}

// Calibration Functions
void calibrate_middle_sensors() {
  for (uint8_t i = 0; i < 9; i++) {
    middleMin[i] = TIME_CALIB;
    middleMax[i] = 0;
  }

  for (uint16_t n = 0; n < NUM_CALIB; n++) {
    uint16_t v[9];
    read_middle_sensors(v);
    for (uint8_t i = 0; i < 9; i++) {
      if (v[i] < middleMin[i]) middleMin[i] = v[i];
      if (v[i] > middleMax[i]) middleMax[i] = v[i];
    }
    delay(5);
  }
}


void calibrate_front_sensors() {
  for (uint8_t i = 0; i < 2; i++) {
    frontMin[i] = TIME_CALIB;
    frontMax[i] = 0;
  }

  for (uint16_t n = 0; n < NUM_CALIB; n++) {
    uint16_t v[2];
    read_front_sensors(v);
    for (uint8_t i = 0; i < 2; i++) {
      if (v[i] < frontMin[i]) frontMin[i] = v[i];
      if (v[i] > frontMax[i]) frontMax[i] = v[i];
    }
    delay(5);
  }
}


// Function to check if all readings are white on the front 2 sensors
bool check_front_white(uint16_t *frontSensors) {
  for (int i = 0; i < 2; i++) {
    // Normalizing each sensor value between 0-1000
    int span = frontMax[i] - frontMin[i];
    int val = constrain(frontSensors[i] - frontMin[i], 0, span);
    if (span < 1) span = 1;
    int normalized = (val * 1000) / span;


    // If the normalized value is greater than 200 -> black line
    if (normalized > 200) return false;
  }
  return true;
}

// Function to check if all readings are white on the front 2 sensors
bool check_front_black(uint16_t *frontSensors) {
  for (int i = 0; i < 2; i++) {
    // Normalizing each sensor value between 0-1000
    int span = frontMax[i] - frontMin[i];
    int val = constrain(frontSensors[i] - frontMin[i], 0, span);
    if (span < 1) span = 1;
    int normalized = (val * 1000) / span;


    // If the normalized value is greater than 200 -> black line
    if (normalized > 200) return true;
  }
  return false;
}


// Function to check if all readings are white on the middle 9 sensors
bool check_middle_white(uint16_t *middleSensors) {
  for (int i = 0; i < 9; i++) {
    // Normalizing each sensor value between 0-1000
    int span = middleMax[i] - middleMin[i];
    int val = constrain(middleSensors[i] - middleMin[i], 0, span);
    if (span < 1) span = 1;
    int normalized = (val * 1000) / span;


    // If the normalized value is greater than 200 -> black line
    if (normalized > 200) return false;
  }
  return true;
}


int compute_error(uint16_t *sens) {
  uint16_t value[7];
  // Skipping the corner sensors because they're noisy
  for (int i = 0; i < 7; i++) {
    int idx = i + 1;
    int span = middleMax[idx] - middleMin[idx];
    if (span < 1) span = 1;

    // Normalize raw sensor readings using the values obtained during calibration
    int val = sens[idx] - middleMin[idx];
    val = constrain(val, 0, span);
    // 0 -> White AND 1000 -> Black
    value[i] = (val * 1000) / span;
  }

  int32_t sum = 0, weightedSum = 0; // sensor values total and their weighted total
  for (int i = 0; i < 7; i++) {
    if (value[i] > 100) {
      weightedSum += (int32_t)value[i] * sensor_weight[i];
      sum += value[i];
    }
  }

  static int lastKnownError = 0; // Static variable to hold the last known error
  // If no sensor reads a line, follow the previous error
  if (sum == 0) return lastKnownError;


  lastKnownError = weightedSum / sum; // Gives the error of the detected line from the center
  return lastKnownError;
}


void turnUntilBlack(bool turnRight) {
  uint16_t frontSensors[2];
  bool foundLine = false;
  // Try the detected turn direction slowly for 10 attempts
  for (int i = 0; i < 10; i++) {

    int left, right;

    if (turnRight) {
      left = 335;
      right = -300;
    } else {
      left = -335;
      right = 300;
    }

    mc1.setSpeed(LEFT_MOTOR, left);
    mc1.setSpeed(RIGHT_MOTOR, right);
    delay(150);

    read_front_sensors(frontSensors);
    // Break loop if black line found by front sensors
    if (check_front_black(frontSensors)) {
      foundLine = true;
      break;
    }


    delay(50);
  }

  // If not found, try turning in the opposite direction slowly
  if (!foundLine) {
    for (int i = 0; i < 20; i++) {


      int left, right;


      if (turnRight) {
        left = 335;
        right = -300;
      } else {
        left = -335;
        right = 300;
      }


      mc1.setSpeed(LEFT_MOTOR, left);
      mc1.setSpeed(RIGHT_MOTOR, right);
      delay(150);


      read_front_sensors(frontSensors);
      // Break loop if black line found by front sensors
      if (check_front_black(frontSensors)) {
        foundLine = true;
        break;
      }


      delay(50);
    }
  }


  // Stop motors after turning
  mc1.setSpeed(LEFT_MOTOR, 0);
  mc1.setSpeed(RIGHT_MOTOR, 0);
  delay(150);
}

// Main loop
void line_following() {
  uint16_t middleSensors[9], frontSensors[2];
  read_middle_sensors(middleSensors);
  read_front_sensors(frontSensors);

  // Static variable to check if the robot needs to turn
  static bool turning = false;

  if (check_front_white(frontSensors) && check_middle_white(middleSensors)) {
    mc1.setSpeed(LEFT_MOTOR, 0);
    mc1.setSpeed(RIGHT_MOTOR, 0);
   
    // Only stop until black line isn't detected again
    while (check_front_white(frontSensors) && check_middle_white(middleSensors)) {
      read_front_sensors(frontSensors);
      read_middle_sensors(middleSensors);
      delay(50);
    }
    lastError = 0;
  }

  if (!turning && check_front_white(frontSensors)) {
    turning = true;

    // Stop for a while
    mc1.setSpeed(LEFT_MOTOR, 0);
    mc1.setSpeed(RIGHT_MOTOR, 0);
    delay(200);


    // Move front slightly to detect which direction to turn
    mc1.setSpeed(LEFT_MOTOR, 200);
    mc1.setSpeed(RIGHT_MOTOR, 200);
    delay(250);
    mc1.setSpeed(LEFT_MOTOR, 0);
    mc1.setSpeed(RIGHT_MOTOR, 0);
    delay(150);

    // Decide on the turn direction
    bool turnRight = lastError >= 0; // Check turn direction using last error

    // Turn gradually until the turn is detected again
    turnUntilBlack(turnRight);
  }

  // Prevent the robot from executing the turning process if it's already turning
  if (turning && !check_front_white(frontSensors)) {
    turning = false;
  }

  // Line Following using a PD controller
  int error = compute_error(middleSensors);
  int derivative = error - lastError;
  int correction = (Kp * error + Kd * derivative) * 3;
  lastError = error;

  // Make corrections to motor speeds
  int leftSpeed = base_speed - correction;
  int rightSpeed = base_speed + correction;

  // Set constraints on motor speeds
  leftSpeed = constrain(leftSpeed, -max_speed, max_speed);
  rightSpeed = constrain(rightSpeed, -max_speed, max_speed);

  // Control motor speed
  // Note: Left motor is offset from right by 35
  mc1.setSpeed(LEFT_MOTOR, leftSpeed + 35);
  mc1.setSpeed(RIGHT_MOTOR, rightSpeed);

  // Print error and motor speeds on the serial monitor
  Serial.print("Error: ");
  Serial.print(error);
  Serial.print(" | L: ");
  Serial.print(leftSpeed);
  Serial.print(" | R: ");
  Serial.println(rightSpeed);

  delay(10);
}
