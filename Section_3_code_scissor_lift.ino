#include <Servo.h>
#include <Motoron.h>


Servo servo1;  // First servo
Servo servo2;  // Second servo
Servo servo3;  // Third servo (scissor lift)


MotoronI2C mc(0x10);  


// Motor Pins
const uint8_t LEFT_MOTOR = 1;
const uint8_t RIGHT_MOTOR = 3;


void setup() {
  Serial.begin(9600);


  // Attach servos
  servo1.attach(30);
  servo2.attach(31);
  servo3.attach(32);


  // Initialize Motor shield
  Wire1.begin();
  mc.setBus(&Wire1);
  mc.reinitialize();
  mc.disableCrc();
  mc.clearResetFlag();
  mc.disableCommandTimeout();


  mc.setMaxAcceleration(LEFT_MOTOR, 100);
  mc.setMaxAcceleration(RIGHT_MOTOR, 100);


  // Hook Sequence
  // Move limb servos to hook position
  servo1.write(50);
  servo2.write(90);
  delay(1000);


  // Move the scissor lift down
  servo3.write(80);
  delay(1000);


  // Move the scissor lift to hook
  servo3.write(65);
  delay(1000);


  // Move servo limbs to normal moving position
  servo1.write(90);
  servo2.write(50);
  delay(2000);




  // Drive motors forward for 5 seconds
  mc.setSpeed(LEFT_MOTOR, 300);
  mc.setSpeed(RIGHT_MOTOR, 300);
  Serial.println("Motors moving forward");
  delay(5000);


  // Stop motors
  mc.setSpeed(LEFT_MOTOR, 0);
  mc.setSpeed(RIGHT_MOTOR, 0);
  Serial.println("Motors stopped");


  // Move servo limbs to hook position again
  servo1.write(50);
  servo2.write(90);
  Serial.println("Servos 1&2 moved to unhook position");
  delay(1000);


  //Release the hook in the scissor lift
  servo3.write(80);
  Serial.println("Servo3 (scissor lift) moved up to release");
  delay(1000);


  // Move servo limbs back to normal moving position
  servo1.write(90);
  servo2.write(50);
  Serial.println("Servos 1&2 moved back to positions 0 and 0");
  delay(2000);


}


void loop() {
   
}
