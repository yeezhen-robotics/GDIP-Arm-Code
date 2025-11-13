#include <Adafruit_PWMServoDriver.h>
#include <SPI.h>             /* to handle the communication interface with the modem*/
#include <nRF24L01.h>        /* to handle this particular modem driver*/
#include <RF24.h>            /* the library which helps us to control the radio modem*/

Adafruit_PWMServoDriver board1 = Adafruit_PWMServoDriver(0x40);       // called this way, it uses the default address 0x40 

#define PI 3.141592           // PI
#define SERVOMIN  125         // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  625         // this is the 'maximum' pulse length count (out of 4096)
#define SweepAddressStart 0   // This is where the microcontroller begins sweeping through the servos
#define SweepAddressMid 3
#define SweepAddressEnd 5
#define Gripper 5
#define GripperClose 360.0              // Angle to allow for gripper to close
#define GripperOpen 100.0               // Angle to allow for gripper to open
#define led_pin_status 3                // Connect LED anode to D3 (PWM pin)
#define operation_mode rx_data[0]       // Operation Mode
#define take_memory_snapshot rx_data[6] // Memory snapshot index
#define recording_memory_max 20         // Amount of space allocated to memory
#define autonomous 0                    // Operation Mode Selection
#define teach 1                         // Operation Mode Selection
#define home 2                          // Operation Mode Selection

                      // B,S1,S2,S3,S4, G
float ServoAngles[6] = { 0, 0, 0, 0, 0, 0};
int previous_gripper_pos = 0;
int rx_data[7] = {0, 0, 0, 0, 0, 0, 0};                 // Variable to store received data
float operational_memory[recording_memory_max][6] = {0};   // Memory to maintain teached data
int previous_operation_mode = 0;
float servos2rest[6] = { 90, 90, 90, 90, 90, GripperOpen}; // Positions to home servos
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
    recording_memory_len = 0;                  // Reset memory buffer
    pwm_response_controller(rx_data);
    if (take_memory_snapshot == 1) {
      save_motor_positions();                  // Save the motor positions in the memory buffer
    }
  }
  else if (operation_mode == autonomous) {
    for (int i = 0; i < recording_memory_len; i++) {
      pwm_response_pose(operational_memory[i],1.0);
    }
  }
  else if (operation_mode == home) {
    pwm_response_pose(servos2rest,0.5);
  }
  else {
    pwm_response_pose(servos2rest,0.5);
  }
  previous_operation_mode = operation_mode;
}

// Saves all current motor position in operational memory
void save_motor_positions() {
  for (int i = 0; i < 6; i++) {
    operational_memory[recording_memory_len][i] = ServoAngles[i];
  }
  recording_memory_len++;
}

// Moves all motor position according to velocity commands
// Values used in ctrl function only include the motor commands (Index 1-5)
void pwm_response_controller(int ctrl[7]) {

  float current_angle = 0;

  // Base
  current_angle = ServoAngles[0] + float(ctrl[1]) * 0.1; // Move angle to the servo position according to the received velocity
  board1.setPWM(0, 0, angleToPulse(current_angle));    // Set all my Servos to the specified angle at the current timestep
  ServoAngles[0] = current_angle;

  // Arm
  current_angle = ServoAngles[1] + float(ctrl[2]) * 0.1; // Move angle to the servo position according to the received velocity
  board1.setPWM(1, 0, angleToPulse(current_angle));    // Set all my Servos to the specified angle at the current timestep
  board1.setPWM(2, 0, angleToPulse(180.0 - current_angle));    // Set all my Servos to the specified angle at the current timestep
  ServoAngles[1] = current_angle; ServoAngles[2] = 180.0 - current_angle;

  // End Effector Pitch
  current_angle = ServoAngles[3] + float(ctrl[3]) * 0.1; // Move angle to the servo position according to the received velocity
  board1.setPWM(0, 0, angleToPulse(current_angle));    // Set all my Servos to the specified angle at the current timestep
  ServoAngles[3] = current_angle;

  // Twist
  current_angle = ServoAngles[4] + float(ctrl[4]) * 0.1; // Move angle to the servo position according to the received velocity
  board1.setPWM(0, 0, angleToPulse(current_angle));    // Set all my Servos to the specified angle at the current timestep
  ServoAngles[4] = current_angle;

  // Jaw
  if (ctrl[Gripper] == 1) {
    switch_gripper_state();
  }

}

// Moves all motor positions to the target angle
void pwm_response_pose(float qf[6], float period) {

  float tf = period, current_angle, t; // initial, final time and real time variable
  int steps = (int)(tf*60);            // steps for linspace resolution

  // Priority Group 1
  for (int s = 0; s < steps; s++) {    // evaluate sinusoid at times for plotting
    t = s*0.0167;                      // 1/60 Approx. 0.016666...
    for(int i = SweepAddressStart; i < SweepAddressMid; i++) {    
      current_angle = sinusoid(qf[i],ServoAngles[i],PI/tf,t);      // Calculate Sinusoid Trajectory
      board1.setPWM(i, 0, angleToPulse(current_angle));         // Set all my Servos to the specified angle at the current timestep
    }
    delay(17);                         // Match Servo's Update Rate of 60 Hz
  }

  // Priority Group 2
  for (int s = 0; s < steps; s++) {    // evaluate sinusoid at times for plotting
    t = s*0.0167;                      // 1/60 Approx. 0.016666...
    for(int i = SweepAddressMid; i < SweepAddressEnd; i++) {    
      current_angle = sinusoid(qf[i],ServoAngles[i],PI/tf,t);      // Calculate Sinusoid Trajectory
      board1.setPWM(i, 0, angleToPulse(current_angle));         // Set all my Servos to the specified angle at the current timestep
    }
    delay(17);                         // Match Servo's Update Rate of 60 Hz
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
  float tf = 0.25, current_angle, t;
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
    delay(17); // Match Servo's Update Rate of 60 Hz
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

void debug_transmission(int rx_data[7]) {
    Serial.print("Received Data : ");
    Serial.print(ServoAngles);           /* Print received value on Serial Monitor */
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
