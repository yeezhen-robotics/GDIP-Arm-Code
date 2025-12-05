#include <Adafruit_PWMServoDriver.h>
#include <SPI.h>             /* to handle the communication interface with the modem*/
#include <nRF24L01.h>        /* to handle this particular modem driver*/
#include <RF24.h>            /* the library which helps us to control the radio modem*/

Adafruit_PWMServoDriver board1 = Adafruit_PWMServoDriver(0x40);       // called this way, it uses the default address 0x40 

#define PI 3.141592           // PI
#define SERVOMIN  125         // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  625         // this is the 'maximum' pulse length count (out of 4096)
#define SweepAddressStart 0   // This is where the microcontroller begins sweeping through the servos
#define SweepAddressMid 4
#define SweepAddressEnd 5
#define Gripper 5
#define GripperClose 360.0              // Angle to allow for gripper to close
#define GripperOpen 100.0               // Angle to allow for gripper to open
#define led_pin_status 3                // Connect LED anode to D3 (PWM pin)
#define operation_mode rx_data[0]       // Operation Mode
#define take_memory_snapshot rx_data[6] // Memory snapshot index
#define recording_memory_max 20         // Amount of space allocated to memory
#define autonomous 2                    // Operation Mode Selection
#define teach 1                         // Operation Mode Selection
#define home 0                          // Operation Mode Selection

                      // B,S1,S2,S3,S4, G
float ServoAngles[6] = { 0, 0, 0, 0, 0, 0};
int previous_control_gripper = 0;
int previous_memory_callback = 0; // Variable to store previous state of the button to detect rising edge of inputs
int rx_data[7] = {0, 0, 0, 0, 0, 0, 0};                 // Variable to store received data
float operational_memory[recording_memory_max][6] = {0};   // Memory to maintain teached data
int previous_operation_mode = 0;
float servos2rest[6] = { 90, 50, 80, 90, 95, GripperOpen}; // Positions to home servos
int recording_memory_index = 0, recording_memory_len = 0;  // Memory trackers

#define CE_PIN 8
#define CSN_PIN 7

RF24 radio(CE_PIN, CSN_PIN);     /* Creating instance 'radio'  ( CE , CSN )   CE -> D7 | CSN -> D8 */                            
const byte Address[6] = "00009"; // Address from which data to be received 

// SDA -> A4 SCL -> A5

int angleToPulse(int ang) {                           // Gets angle in degree and returns the pulse width
    int pulse = map(ang,0, 180, SERVOMIN,SERVOMAX);  // Map angle of 0 to 180 to Servo min and Servo max 
     // Serial.print("Angle: ");Serial.print(ang);
     // Serial.print(" pulse: ");Serial.println(pulse);
     return pulse;
}

float sinusoid(float qf, float qo, float w, float t) {
  // frequency of sinusoid
  float q = 0.5*(qf - qo)*(1 - cos(w*t)) + qo;
  return q;
}

float rad(int degrees) {
  return degrees*PI/180;
}

void setup() {

  pinMode(CE_PIN, OUTPUT);
  pinMode(CSN_PIN, OUTPUT);
  digitalWrite(CE_PIN, LOW);   // CE low until configured
  digitalWrite(CSN_PIN, HIGH); // CSN high (not selected)

  Serial.begin(9600);
  Serial.println("Stop");

  // Check for board
  board1.begin();
  board1.setPWMFreq(60);             // Analog servos run at ~60 Hz updates

  // Power on Signal Generator
  radio.begin();                     /* Activate the modem*/
  radio.openReadingPipe(1, Address); /* Sets the address of receiver from which program will receive the data*/
  radio.setDataRate(RF24_1MBPS);     /* Sets data rate to be standard */
  radio.setChannel(76);              /* Specify proper channel of communication */
  radio.startListening();			       /* Setting modem in Receiver mode */
  radio.setPALevel(RF24_PA_MIN);    /* Long distance signal module */

  pwm_response_pose(servos2rest,1.0);
}

void loop() {
  //switch_gripper_state();            // Switch the gripper to be open or close
  if (radio.available()) {
    radio.read(&rx_data, sizeof(rx_data)); /* Read the received data and store in ' rx_data ' */
    //debug_transmission(rx_data);
  }

  if (operation_mode == teach) {
    if (previous_operation_mode != teach) {
      recording_memory_len = 0;                  // Reset memory buffer
    }
    pwm_response_controller(rx_data);
    if (take_memory_snapshot == 1 && take_memory_snapshot != previous_memory_callback) { // Rising Edge
      Serial.print("Saved!");
      save_motor_positions();                  // Save the motor positions in the memory buffer
      recording_memory_len++;
    }
  }
  else if (operation_mode == autonomous) {
    for (int i = 0; i < recording_memory_len; i++) {

      // Update buffer data to escape loop
      if (radio.available()) {
        radio.read(&rx_data, sizeof(rx_data)); /* Read the received data and store in ' rx_data ' */
        //debug_transmission(rx_data);
      }

      // Only run autonomous mode if its switched on
      if (operation_mode == autonomous) {
        pwm_response_pose(operational_memory[i],2.0);
      }
      
      //debug_servo_angles();
    }
    delay(500);
    
  }
  else if (operation_mode == home) {
    pwm_response_pose(servos2rest,2.0);
  }
  else {
    pwm_response_pose(servos2rest,2.0);
  }
  previous_operation_mode = operation_mode;
  previous_memory_callback = take_memory_snapshot;
  previous_control_gripper = rx_data[Gripper];
  //debug_servo_angles();

}

// Saves all current motor position in operational memory
void save_motor_positions() {
  for (int i = 0; i < 6; i++) {
    operational_memory[recording_memory_len][i] = ServoAngles[i];
  }
}

// Moves all motor position according to velocity commands
// Values used in ctrl function only include the motor commands (Index 1-5)
void pwm_response_controller(int ctrl[7]) {

  float current_angle = 0, current_angle_base = 0;

  // Base
  current_angle = ServoAngles[0] + float(ctrl[1]) * 0.05; // Move angle to the servo position according to the received velocity
  board1.setPWM(0, 0, angleToPulse(current_angle));    // Set all my Servos to the specified angle at the current timestep
  ServoAngles[0] = current_angle;

  // Arm
  current_angle_base = ServoAngles[1] + float(ctrl[2]) * 0.05; // Move angle to the servo position according to the received velocity
  board1.setPWM(1, 0, angleToPulse(current_angle_base));    // Set all my Servos to the specified angle at the current timestep
  ServoAngles[1] = current_angle_base;

  // End Effector Pitch
  current_angle = ServoAngles[2] + float(ctrl[3]) * 0.05; // Move angle to the servo position according to the received velocity
  board1.setPWM(2, 0, angleToPulse(current_angle));    // Set all my Servos to the specified angle at the current timestep
  ServoAngles[2] = current_angle;

  // This pitch is special, I want this to always be parallel to the ground
  board1.setPWM(3, 0, angleToPulse(270 - (current_angle_base + current_angle + 40)));  // Set all my Servos to the specified angle at the current timestep
  ServoAngles[3] = 270 - (current_angle_base + current_angle + 40);

  // Twist
  current_angle = ServoAngles[4] + float(ctrl[4]) * 0.1; // Move angle to the servo position according to the received velocity
  board1.setPWM(4, 0, angleToPulse(current_angle));    // Set all my Servos to the specified angle at the current timestep
  ServoAngles[4] = current_angle;

  // Jaw
  if (ctrl[Gripper] != previous_control_gripper && ctrl[Gripper] == 1) { // Raising Edge of the Clock
    switch_gripper_state();
  }

}

// Moves all motor positions to the target angle
void pwm_response_pose(float qf[6], float period) {

  float tf = period, current_angle, t; // initial, final time and real time variable
  int steps = (int)(tf*60);            // steps for linspace resolution

  //Serial.println("Updating...");

  debug_angles(qf);
  // Priority Group 1
  if (qf[0] != ServoAngles[0] || qf[1] != ServoAngles[1] || qf[2] != ServoAngles[2] || qf[3] != ServoAngles[3]) {
    for (int s = 0; s < steps; s++) {    // evaluate sinusoid at times for plotting
      t = s*0.0167;                      // 1/60 Approx. 0.016666...
      for(int i = SweepAddressStart; i < SweepAddressMid; i++) {
        current_angle = sinusoid(qf[i],ServoAngles[i],PI/tf,t);   // Calculate Sinusoid Trajectory
        board1.setPWM(i, 0, angleToPulse(current_angle));         // Set all my Servos to the specified angle at the current timestep
      }
      delay(17);
    }
  }

  // Priority Group 2
  if (qf[4] != ServoAngles[4]) {
    for (int s = 0; s < steps; s++) {    // evaluate sinusoid at times for plotting
      t = s*0.0167;                      // 1/60 Approx. 0.016666...
      for(int i = SweepAddressMid; i < SweepAddressEnd; i++) {    
        current_angle = sinusoid(qf[i],ServoAngles[i],PI/tf,t);      // Calculate Sinusoid Trajectory
        board1.setPWM(i, 0, angleToPulse(current_angle));         // Set all my Servos to the specified angle at the current timestep
      }
    }
  }

  // Priority Group 3
  if (ServoAngles[Gripper] != qf[Gripper]) {
    switch_gripper_state();            // Switch the gripper to be open or close
  }

  for (int i = SweepAddressStart; i < SweepAddressEnd; i++) {
     ServoAngles[i] = qf[i];           // Update all my known angle positions
  }       
}

// Changes the state of the gripper at the press of a button
void switch_gripper_state() {

  // initial, final time and real time variable
  float tf = 1, current_angle, t;
  // steps for linspace resolution
  int steps = (int)(tf*60), qf;

  if (ServoAngles[Gripper] == GripperOpen) {
    qf = GripperClose;
  }
  else if (ServoAngles[Gripper] == GripperClose) {
    qf = GripperOpen;
  }
  else {
    qf = GripperOpen; // Gripper will remain open if there is an error to avoid potential collision with object
  }

  // evaluate sinusoid at times for plotting
  for (int s = 0; s < steps; s++) {
    t = s*0.0167; // 1/60 Approx. 0.016666...
    current_angle = sinusoid(qf,ServoAngles[Gripper],PI/tf,t);   // Calculate Sinusoid Trajectory
    board1.setPWM(Gripper, 0, angleToPulse(current_angle));            // Set Servo Angle
  }

  board1.setPWM(Gripper, qf, angleToPulse(current_angle));            // Set Servo Angle
  ServoAngles[Gripper] = qf;
}

void debug_transmission(int rx_data[7]) {
    Serial.print("Received Data : ");
    Serial.print(rx_data[0]);           /* Print received value on Serial Monitor */
    Serial.print("|");
    Serial.print(rx_data[1]);           /* Print received value on Serial Monitor */
    Serial.print("|");
    Serial.print(rx_data[2]);           /* Print received value on Serial Monitor */
    Serial.print("|");
    Serial.print(rx_data[3]);           /* Print received value on Serial Monitor */
    Serial.print("|");
    Serial.print(rx_data[4]);           /* Print received value on Serial Monitor */
    Serial.print("|");
    Serial.print(rx_data[5]);           /* Print received value on Serial Monitor */
    Serial.print("|");
    Serial.println(rx_data[6]);           /* Print received value on Serial Monitor */
}

void debug_servo_angles() {
    Serial.print("ServoAngles: ");
    Serial.print(ServoAngles[0]);           /* Print received value on Serial Monitor */
    Serial.print("|");
    Serial.print(ServoAngles[1]);           /* Print received value on Serial Monitor */
    Serial.print("|");
    Serial.print(ServoAngles[2]);           /* Print received value on Serial Monitor */
    Serial.print("|");
    Serial.print(ServoAngles[3]);           /* Print received value on Serial Monitor */
    Serial.print("|");
    Serial.print(ServoAngles[4]);           /* Print received value on Serial Monitor */
    Serial.print("|");
    Serial.println(ServoAngles[5]);           /* Print received value on Serial Monitor */
}

void debug_angles(float angles[6]) {
    Serial.print("Angles: ");
    Serial.print(angles[0]);           /* Print received value on Serial Monitor */
    Serial.print("|");
    Serial.print(angles[1]);           /* Print received value on Serial Monitor */
    Serial.print("|");
    Serial.print(angles[2]);           /* Print received value on Serial Monitor */
    Serial.print("|");
    Serial.print(angles[3]);           /* Print received value on Serial Monitor */
    Serial.print("|");
    Serial.print(angles[4]);           /* Print received value on Serial Monitor */
    Serial.print("|");
    Serial.println(angles[5]);           /* Print received value on Serial Monitor */
}
