#include <Servo.h>
#include <Wire.h>
#include <Motoron.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <PID_v1.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20649.h>
#include <Adafruit_Sensor.h>
#include <math.h>


#define TRIG_PIN 27
#define ECHO_PIN 25


//FUNCTION DECLARING
void limbs_vertical();
void limbs_horizontal();
void limbs_incline();
void scissor_lift_extended();
void scissor_lift_initial();
void stuck_detection();
void kill_signal();
void wall_following();


// --- Create a servo object ---
Servo servo1;
Servo servo2;
Servo servo3;


//WALL FOLLOWING VARIABLES
const int sensorLeft = A1;


// --- Wall-following parameters ---
const float targetDistance = 15; // Desired distance from wall (in cm)


// --- PID variables for wall following
double Setpoint1, Input1, Output1;


// --- Specify the links and initial tuning parameters for wall following
double Kp1 = 10, Ki1 = 0.0001, Kd1 = 2.5;
PID PIDcon(&Input1, &Output1, &Setpoint1, Kp1, Ki1, Kd1, DIRECT);


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


// --- IMU STUCK DETECTION---
// IMU object
Adafruit_ICM20649 icm;
Adafruit_Sensor *icm_temp, *icm_accel, *icm_gyro;


bool imu_initialized = false;


// Stuck detection parameters
const float DISTANCE_TOLERANCE = 2.0; // cm - how much distance can vary
const float ACCEL_THRESHOLD = 4; // m/s² - threshold for "low" acceleration
const int STUCK_TIME_THRESHOLD = 1200; // milliseconds - how long conditions must persist
bool post_recovery_active = false;
unsigned long post_recovery_start_time = 0;
const unsigned long POST_RECOVERY_HOLD_TIME = 1500;
bool speedBoostActive = false;
unsigned long speedBoostStartTime = 0;
const unsigned long SPEED_BOOST_DURATION = 1500;


// Variables for stuck detection
float last_distance = -1;
unsigned long stuck_start_time = 0;
bool stuck_conditions_met = false;
bool stuck_detected = false;
bool recovery_in_progress = false;
unsigned long recovery_start_time = 0;


// Recovery sequence timing
const unsigned long RECOVERY_PHASE_1_TIME = 1000;
const unsigned long RECOVERY_PHASE_2_TIME = 2000;
const unsigned long TOTAL_RECOVERY_TIME = RECOVERY_PHASE_1_TIME + RECOVERY_PHASE_2_TIME;


void setup() {
  Serial.begin(9600);

  //Ultrasonic Pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);


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

  // === PID setup ===
  Setpoint1 = 0; // Target error is 0
  PIDcon.SetMode(AUTOMATIC);
  PIDcon.SetOutputLimits(-150, 150); // Limit PID output


  limbs_vertical();
  scissor_lift_initial();


  // -- IMU --
  Serial.println("Initializing IMU...");
  delay(100);


  if (!icm.begin_I2C()) {
    Serial.println("Failed to find ICM20649 Chip");
  } else {
    imu_initialized = true;
    Serial.println("ICM20649 Found!");
    icm_temp = icm.getTemperatureSensor();
    icm_accel = icm.getAccelerometerSensor();
    icm_gyro = icm.getGyroSensor();
  }

  servo1.write(140);
  servo2.write(0);
  wall_following();
  Serial.println("System initialized. Robot moving forward, monitoring for stuck condition...");
  Serial.println("Distance(cm), AccelX, AccelY, AccelXY_Mag, Status");
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
    wall_following();
    stuck_detection();
  }
  }

// === Operation Functions ===
void wall_following(){
int baseSpeed = 425; // Set motor base speed

  if (speedBoostActive) {
    if (millis() - speedBoostStartTime < SPEED_BOOST_DURATION) {
      baseSpeed = 600; // Boosted speed
    } else {
      speedBoostActive = false;
    }
  }

  // Read left distance sensor
  int rawLeft = analogRead(sensorLeft);
  float vLeft = rawLeft * 5.0 / 1024.0;
  float dLeft = 12352.1 * pow(vLeft, -1.74) / 1000.0;


  if (rawLeft > 4 && rawLeft < 20) {
    dLeft = 0;
  }

  // --- ULTRASONIC INITIAL ---
  // Clear trigger
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  // Trigger the sensor by setting the pin high for 10 microseconds
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Read the echo time in microseconds
  long duration = pulseIn(ECHO_PIN, HIGH);

  // Calculate distance in cm (speed of sound = 343 m/s)
  float distance_cm = duration * 0.0343 / 2;

  // Stop motors if wall is detected (too close)
  if (dLeft < 8) {
  Serial.println("Wall detected - stopping");
  mc1.setSpeed(1, 0); // Left motor
  mc1.setSpeed(3, 0); // Right motor
  return; // Skip rest of loop and motors remain stopped
  }

  // Calculate error for PID input
  float error = targetDistance - dLeft; // positive = too close to wall, negative = too far from wall
  // Set PID input to the error value
  Input1 = error;
  // Compute PID output
  PIDcon.Compute();

  // Output distance and PID values
  Serial.print("Left distance: ");
  Serial.print(dLeft, 1);
  Serial.print(" cm | Error: ");
  Serial.print(error, 1);
  Serial.print(" cm | PID Output: ");
  Serial.println(Output1, 1);

  //motors + wall following
  int correction = (int)Output1;
  Serial.println(correction);

  int baseSpeedLeft = baseSpeed + 35;
  int baseSpeedRight = baseSpeed;

  // Calculate motor speeds with PID correction + multiply correction for faster and more significant effect
  int speedLeft = baseSpeedLeft - (correction*1.25); // If too close to wall, speed up left motor
  int speedRight = baseSpeedRight + (correction*1.25); // If too close to wall, slow down right motor

  // Constrain speed of motors to safe range
  speedLeft = constrain(speedLeft, 0, 800);
  speedRight = constrain(speedRight, 0, 800);

  // Set motor speeds
  mc1.setSpeed(1, speedLeft); // Left motor
  mc1.setSpeed(3, speedRight); // Right motor

  // Debug output for motor speeds
  Serial.print("Motor speeds - Left: ");
  Serial.print(speedLeft);
  Serial.print(" | Right: ");
  Serial.println(speedRight);

  // TURNING
  Serial.print(" Ultrasonic Distance: ");
  Serial.print(distance_cm);
  Serial.println(" cm");


  if (distance_cm < 14) { //ditsance from ultrsonic sensor to front wall
    Serial.println("Obstacle detected - turning...");

    int baseSpeed1 = 320; //faster speed for turning
    // Stop briefly
    mc1.setSpeed(1, 0);
    mc1.setSpeed(3, 0);
    delay(200);
    //turn right
    mc1.setSpeed(1, baseSpeed1+35); // Left motor forward (+applied correction for left motor being inherently slower than right)
    mc1.setSpeed(3, -baseSpeed1); // Right motor reverse
    delay(1700);
    // Stop motors after turning
    mc1.setSpeed(1, 0);
    mc1.setSpeed(3, 0);
    delay(200);
    return; // Skip the rest of the loop this cycle
  }
}




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

void limbs_horizontal(){
  servo1.write(90);
  servo2.write(50);
}

void scissor_lift_initial(){
  servo3.write(80);
}

void scissor_lift_extended(){
  servo3.write(65);
}


void stuck_detection() {
  float distance_cm = readDistance();
  float accel_x = 0, accel_y = 0, accel_xy_magnitude = 0;


  if (imu_initialized) {
    sensors_event_t accel, gyro, temp;
    icm_temp->getEvent(&temp);
    icm_accel->getEvent(&accel);
    icm_gyro->getEvent(&gyro);


    accel_x = accel.acceleration.x;
    accel_y = accel.acceleration.y;
    accel_xy_magnitude = sqrt(accel_x * accel_x + accel_y * accel_y);
  }


  if (recovery_in_progress) {
    handleRecoverySequence();
  } else if (post_recovery_active) {
    if (millis() - post_recovery_start_time < POST_RECOVERY_HOLD_TIME) {
      servo1.write(55);
      servo2.write(85);
      wall_following(); // Keep motors running normally during post recovery
    } else {
      servo1.write(140);
      servo2.write(0);
      wall_following(); // Resume motors and wall follwoing after reset
      post_recovery_active = false;
      Serial.println("*** POST-RECOVERY SERVO RESET COMPLETE ***");
    }
  } else {
   handleStuckDetection(distance_cm, accel_xy_magnitude);
    wall_following(); //wall follow normally when not stuck or recovering
  }


  printStatus(distance_cm, accel_x, accel_y, accel_xy_magnitude);
  delay(100);
}


//read ultrasonic distance function to be used in stuck detection
float readDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);


  long duration = pulseIn(ECHO_PIN, HIGH);
    return duration * 0.0343 / 2;
  }


  void handleStuckDetection(float distance_cm, float accel_xy_magnitude) {
  bool distance_constant = false;
  bool acceleration_low = accel_xy_magnitude <= ACCEL_THRESHOLD;


  if (last_distance >= 0 &&
    abs(distance_cm - last_distance) <= DISTANCE_TOLERANCE) {
    distance_constant = true;
  }


  last_distance = distance_cm;
  bool current_conditions = distance_constant && acceleration_low && imu_initialized;


  if (current_conditions) {
    if (!stuck_conditions_met) {
      // start timing
      stuck_start_time = millis();
      stuck_conditions_met = true;
    } else if (millis() - stuck_start_time >= STUCK_TIME_THRESHOLD && !stuck_detected) {
      Serial.println("*** STUCK DETECTED - STARTING RECOVERY SEQUENCE ***");
      stuck_detected = true;
      startRecoverySequence();
    }
  } else {
    stuck_conditions_met = false;
    stuck_detected = false;
  }
}


void handleRecoverySequence() {
  const unsigned long PHASE_DURATION = 500; // Duration of each movement phase
  const unsigned long TOTAL_BOBBING_TIME = PHASE_DURATION * 4;


  unsigned long elapsed = millis() - recovery_start_time;


  if (elapsed < TOTAL_BOBBING_TIME) {
    int phase = elapsed / PHASE_DURATION;


    // Alternate between up (140,0) and down (55,85)
    if (phase % 2 == 0) {
    servo1.write(140); // Up position
    servo2.write(0);
    } else {
    servo1.write(55); // Down position
    servo2.write(85);
    }
  } else {
    // End of bobbing cycle
    Serial.println("*** RECOVERY SEQUENCE COMPLETE - RESUMING NORMAL OPERATION ***");


    speedBoostActive = true;
    speedBoostStartTime = millis();
    wall_following();


    post_recovery_active = true;
    post_recovery_start_time = millis();


    recovery_in_progress = false;
    stuck_detected = false;
    stuck_conditions_met = false;
    last_distance = -1;
  }
}


void startRecoverySequence() {
  recovery_in_progress = true;
  recovery_start_time = millis();
  mc1.setSpeed(1, 0); // Left motor
  mc1.setSpeed(3, 0); // Right motor
  Serial.println("Recovery sequence started - motors stopped, servos activating...");
}


void printStatus(float distance_cm, float accel_x, float accel_y, float accel_xy_magnitude) {
  Serial.print(distance_cm, 1); Serial.print(", ");
  Serial.print(accel_x, 2); Serial.print(", ");
  Serial.print(accel_y, 2); Serial.print(", ");
  Serial.print(accel_xy_magnitude, 2); Serial.print(", ");


  if (recovery_in_progress) {
    unsigned long t = millis() - recovery_start_time;
    Serial.println((t < RECOVERY_PHASE_1_TIME) ?
    "RECOVERY - Phase 1 (Servos: 140°, 0°)" :
    "RECOVERY - Phase 2 (Servos: 50°, 90°)");
  } else if (stuck_detected) {
    Serial.println("STUCK");
  } else if (stuck_conditions_met) {
    unsigned long t_remain = STUCK_TIME_THRESHOLD - (millis() - stuck_start_time);
    Serial.print("Checking... "); Serial.print(t_remain); Serial.println("ms");
  } else {
    Serial.print("Normal - Moving Forward - ");
  if (last_distance >= 0 && abs(distance_cm - last_distance) > DISTANCE_TOLERANCE)
    Serial.print("Dist_Changing ");
  if (accel_xy_magnitude > ACCEL_THRESHOLD)
    Serial.print("Accel_High ");
  if (!imu_initialized)
    Serial.print("IMU_Error ");
    Serial.println();
  }
}
