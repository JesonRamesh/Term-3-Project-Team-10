# Krabby Patty - How To Use Guide 

## Contents
- Robot Functional Overview
- System Description and Code Functionality
  - Movement Control and Wall-following
  - Line-following
  - Servo Mechanism 
  - Emergency Stop System
  - IMU-based Stuck Detection and Recovery
  - System Initialization
- Operation Instructions
- Libraries Required

## Robot Functional Overview
The robot is a mobile, autonomous platform that integrates multiple subsystems for mobility. Line Following, Obstacle Avoidance, and Self-Recovery. It is equipped with:

- Wall-following behavior using a side-facing analog IR distance sensor and front-facing ultrasonic sensor.
- Line-following behaviour using 2 QTR reflectance sensors attached to the underside of the chassis, with a 2-element sensor at the front and a 9-element sensor just behind it near the middle of the chassis. 
- PID controller for both maintaining a desired distance from the wall, and correcting itself according to weightings given to each element in a reflectance sensor array when following a line.
- 3 servos for the servo-controlled limbs and a scissor lift mechanism used for hooking onto overhead railings and ziplines.
- Stuck detection and self-recovery based on IMU and ultrasonic data.
- Emergency stop/start feature via both a physical button and WiFi UDP commands.
- Speed boost after recovery for efficient re-engagement with the environment.

## System Description and Code Functionality

### 1. Movement Control and Wall Following
The robot uses a Motoron I2C motor controller (address 0x10) to drive two DC motors which are driving the back two wheels. The wall-following logic is based on:

- **Left IR distance sensor**: Measures distance from the left wall throughout the whole wall-following portion of the course.
- **PID controller**: Maintains a target distance (15 cm) from the wall by adjusting motor speeds based on the error. This has been tuned through trial and error to be at an optimum functionality for wall following, for the speed and voltage used by the robot during operation.
- **Front ultrasonic sensor**: Detects obstacles/wall ahead and is used mainly for the turning logic during the wall following portion of the course. If an object is detected closer than 14 cm, the robot halts wall-following and initiates a turning maneuver (for a specified delay that has been corrected through trial and error), after which it resumes wall-following as normal. 

**NOTE**: The robot is equipped with 2 distance sensors on either side of the body, as it was initially meant to use both to aid wall-following. However, due to time constraints and the absence of a right-side wall for most of the obstacle course, this sensor was ultimately not used in the code or operation of the robot.

### 2. Line Following
The robot is equipped with two reflectance sensor arrays:
- **Middle array (9 sensors)**: Used for PD-based line following.
- **Front array (2 sensors)**: Used to detect end-of-line or junctions requiring a turn.

#### Sensor Calibration
Each sensor is manually calibrated (robot physically moved over line several times by operator) at startup to determine the minimum and maximum reflectance values. This allows for normalization of sensor readings during operation.

#### PD Control Logic
Each sensor in the 7 central elements of the 9-sensor array is assigned a weight (from -3 to +3). Only 7 are used due to calibration issues with the first and last sensors in the array, which resulted in persistent inaccurate readings. Therefore these extreme readings were ignored and only the middle 7 sensors are used in line-following operational logic of the robot. 

The error is computed as a weighted average of the normalized sensor values. A Proportional-Derivative (PD) controller is then used to adjust motor speeds:

```
correction = (Kp * error + Kd * derivative) * 3
```
Where Kp = 17, Kd = 2.0, and the correction is multiplied by 3 to provide significant and quick change to motor speeds accordingly. 

#### Turn Handling
When both front sensors detect white (no line), the robot:
1. Stops briefly and moves forward slightly to probe direction.
2. Decides the turning direction based on the last known line-following error.
3. Rotates until one of the front sensors detects black (line found).
4. Resumes normal line following.

This method ensures robust behavior at junctions, gaps in the line and 90 degree turns. 

### 3. Servo Mechanism
Three servos operate key mechanical components:
- **Servo1**: Left limb
- **Servo2**: Right limb
- **Servo3**: Scissor lift

Functions used for setup and execution of servo movements:
- `limbs_vertical()` and `limbs_incline()` control the robot's leg positions during normal or climbing movement. Initially the robot is set up to have limbs in the vertical upright position, while during stuck detection limbs move downwards (`limbs_incline()`) to give the robot extra clearance and momentum to pass obstacles such as stairs and the Giants' Causeway. 
- `scissor_lift_initial()` and `scissor_lift_extended()` are used to set angles for the operation of the scissor lift which is used for hooking onto overhead railings or the zipline. It is initially set up to be in a relaxed position for most of the operational use of the robot, except two occasions during stage three of the course. 

**Note**: Due to time constraints and the complicated logic we were unable to incorporate the `scissor_lift_extended` feature into the main section 3 code including the wall-following and line-following functions. However the extended position of the scissor lift was incorporated into a separate sketch which was used only for the purpose of the Dragon Lava Pit and the Zipline.

### 4. Emergency Stop System
Two methods provide emergency control:
- **Physical kill switch** on digital pin 2 toggles a `stopSignal` flag.
- **WiFi UDP receiver** listens on port 55500 for a "Stop" command. When received, the robot halts immediately.

When `stopSignal` is active:
- Motor speeds are set to 0.
- Robot operations (line-following, wall-following, recovery) are paused until resumed via button press.

### 5. IMU-Based Stuck Detection and Recovery
An onboard ICM20649 IMU is used to detect when the robot is stuck:

The robot is declared "stuck" if both:
- It hasn't moved significantly forward (ultrasonic reading unchanged),
- The IMU detects very low acceleration (< 4 m/s²),
- These conditions persist for more than 1200 ms.

When stuck:
- A servo-based recovery sequence is triggered - lifting and lowering limbs to achieve clearance and lift itself over obstacles.
- Afterward, the robot gets a short speed boost to help escape and resumes normal operation.

### 6. System Initialization
During `setup()`:

**In all sketches:**
- WiFi and UDP communication is initialized.
- Motoron controller and servo pins are configured.
- The robot begins in a safe state with limbs lifted and scissor lift in initial position.

**In certain stage specific sketches:**
- **Line following** - sensor arrays are calibrated manually and values for normalisation are obtained.
- **Wall-following and stuck detection** - IMU is initialised 

**Note**: elements from both of these setups were used in the stage 3 code with both line and wall-following integrated.

After `setup()`:
- The robot transitions to line-following mode or wall-following and stuck-detection mode depending on the configuration, when `stopSignal` is 0. 

## Operation Instructions

### Stage 1 - Line Following
1. Use line following sketch with integrated killswitch logic, and upload the code to the onboard Arduino Giga. 
2. Place the robot over the line, connect battery power and manually move back and forth over the line until calibrated (Until motors start running).
3. The robot should then be capable of autonomously following the line. 

### Stage 2 - Wall Following 
1. Use wall following and stuck detection sketch with integrated killswitch logic, and upload the code to the onboard Arduino Giga.
2. No calibration needed, The robot should display wall following and stuck detection features autonomously. 

### Stage 3 - Line Following + Wall Following, Scissor Lift
1. Use section 3 wall following and line following sketch with integrated killswitch logic, and upload code to the onboard Arduino Giga.
2. Calibrate initially for line following in the same manner as section 1. The robot should then autonomously switch between wall and line-following for locomotion depending on the presence or absence of a wall. 
3. Upload separate motors and scissor lift integrated sketch for Lava Pit and ziplines sections of this stage, as integration of code was not completed due to time constraints. 
4. For each obstacle the robot should autonomously hook on and drive forward.

## Libraries Required
- Arduino.h
- Servo.h
- Wire.h
- Motoron.h
- WiFi.h
- WiFiUdp.h
- PID_v1.h
- Adafruit_ICM20X.h
- Adafruit_ICM20649.h
- Adafruit_Sensor.h
- Math.h
