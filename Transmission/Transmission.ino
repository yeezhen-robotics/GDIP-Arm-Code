//Adding Libraries 
#include <SPI.h>                        /* to handle the communication interface with the modem*/
#include <nRF24L01.h>                   /* to handle this particular modem driver*/
#include <RF24.h>                       /* the library which helps us to control the radio modem*/
#include <ezButton.h>

#define MAX_Controller_Val 1024
#define PI 3.141592

#define LED 4
#define JoyButtonLeft 5
#define JoyButtonRight 6
#define AutonomousButton 2
#define TeachButton 3
#define PressedDown 0
#define NotPressedDown 1
#define HomeLED A4
#define AutonomousLED A5
#define TeachLED A6

// Operation Modes
#define HOME_OPS 0
#define AUTO_OPS 1
#define TEACH_OPS 2

float frontback_left, leftright_left, frontback_right, leftright_right;
int frontback_left_mag, leftright_left_mag, frontback_right_mag, leftright_right_mag;
int magnitude_1, angle_1;

#define CE_PIN 8
#define CSN_PIN 7

RF24 radio(CE_PIN, CSN_PIN);         /* Creating instance 'radio'  ( CE , CSN )   CE -> D7 | CSN -> D8 */                              
const byte Address[6] = "00009" ;    /* Address to which data to be transmitted*/
int value = 0;
int mode = 0;                        // Initialise has home position
int JoyButtonLeftState = 0, JoyButtonRightState = 0, TeachState = 0, AutonomousState = 0;
int transmission_buffer[7] = {0,0,0,0,0,0,0};

void setup() {
  // put your setup code here, to run once:
  pinMode(CE_PIN, OUTPUT);
  pinMode(CSN_PIN, OUTPUT);
  digitalWrite(CE_PIN, LOW);   // CE low until configured
  digitalWrite(CSN_PIN, HIGH); // CSN high (not selected)

  Serial.begin(9600);
  radio.begin();                  /* Activate the modem*/
  radio.openWritingPipe(Address); /* Sets the address of transmitter to which program will send the data */ 
  radio.setChannel(76);           /* Specify proper channel of communication */
  radio.setDataRate(RF24_1MBPS);  /* Sets data rate to be standard */
  radio.setPALevel(RF24_PA_MIN);  /* Long distance signal module */
  radio.stopListening();          /* Setting modem in transmission mode*/
  pinMode(LED, OUTPUT);           // Set the LED pin as output

  pinMode(JoyButtonLeft, INPUT_PULLUP);
  pinMode(JoyButtonRight, INPUT_PULLUP);
  pinMode(AutonomousButton, INPUT_PULLUP);
  pinMode(TeachButton, INPUT_PULLUP);
  pinMode(AutonomousLED, OUTPUT);
  pinMode(TeachLED,OUTPUT);
  pinMode(HomeLED,OUTPUT);
}

void loop() {

  JoyButtonLeftState = digitalRead(JoyButtonLeft);
  JoyButtonRightState = digitalRead(JoyButtonRight);
  TeachState = digitalRead(TeachButton);
  AutonomousState = digitalRead(AutonomousButton);

  // Joystick Processing
  frontback_right = checksmall((MAX_Controller_Val/2)-analogRead(A1)) * -1;
  leftright_right = checksmall(analogRead(A0)-(MAX_Controller_Val/2)) * -1;
  frontback_left = checksmall((MAX_Controller_Val/2)-analogRead(A2)) * -1;
  leftright_left = checksmall(analogRead(A3)-(MAX_Controller_Val/2)) * -1;

  frontback_right_mag = velocities(frontback_right);
  leftright_right_mag = velocities(leftright_right);
  frontback_left_mag = velocities(frontback_left);
  leftright_left_mag = velocities(leftright_left);

  //magnitude_1 = magnitude(leftright1,frontback1);
  //angle_1 = angle(leftright1,frontback1,magnitude_1);

  // Check if any one of the buttons are pressed down
  if (JoyButtonLeftState == PressedDown || JoyButtonRightState == PressedDown) {
      digitalWrite(LED, HIGH); // Turn on the LED
  }
  else {
    digitalWrite(LED, LOW); // Turn off the LED
  }

  // Teach Mode
  if (TeachState == 1 && AutonomousState == 0) {mode = TEACH_OPS; analogWrite(TeachLED, 150); analogWrite(AutonomousLED, 0); analogWrite(HomeLED, 0);}
  // Autonomous Mode
  else if (TeachState == 0 && AutonomousState == 1) {mode = AUTO_OPS; analogWrite(AutonomousLED, 150); analogWrite(TeachLED, 0); analogWrite(HomeLED, 0);}
  // Home Mode
  else if (TeachState == 1 && AutonomousState == 1) {mode = HOME_OPS; analogWrite(HomeLED, 150); analogWrite(TeachLED, 0); analogWrite(AutonomousLED, 0);}
  else {mode = HOME_OPS; analogWrite(HomeLED, 150); analogWrite(TeachLED, 0), analogWrite(AutonomousLED, 0);}

  // Begin mode transfer
  value = magnitude_1;

  transmission_buffer[0] = mode;

  // Motor
  transmission_buffer[1] = leftright_left_mag; // Base Motor
  transmission_buffer[2] = frontback_left_mag; // Motor 1
  transmission_buffer[3] = frontback_right_mag;// Motor 2
  transmission_buffer[4] = leftright_right_mag;// Gripper Motor (Up Down)
  transmission_buffer[5] = not(JoyButtonRightState);// Close/Open Gripper
  transmission_buffer[6] = not(JoyButtonLeftState); // Record Button
  radio.write(transmission_buffer, sizeof(transmission_buffer));

  // HELPFUL DEBUG FUNCTIONS 
  //debug();                   // This stage shows the raw joystick output
  //debugJoystickNormalised(); // This stage shows the normlised joystick output
  //debugVelocities();         // This stage shows the velocities to be sent over
  //debugTransmission();

} 

// Normalizer function for my data, it allows to correct for zero/displacement error
float checksmall(float value) {
  if (abs(value) < 10){
    value = 0;
  }
  else if (value > 510) {
    value = MAX_Controller_Val/2;
  }
  else if (value < -510) {
    value = -MAX_Controller_Val/2;
  }
  return value;
}

int magnitude(float x, float y) {
  int largest_val;
  if (abs(x) < abs(y)) {           // If x is smaller than y
    largest_val = (int)(4*(abs(y)/512)); // Take y as my largest value
  }
  else {
    largest_val = (int)(4*(abs(x)/512)); // Take x as my largest value
  }
  
  return largest_val;
}

int velocities(float x) {

  int val;
  val = (int)((4*x)/500); // Take x as my largest value

  return val;
}

int angle(float x, float y, int magnitude) {
  x = (double)x; y = (double)y;
  int angle;
  if (magnitude == 0) {
    angle = 0;
  }
  else {
    angle = (int)((atan2(y,x)*180/PI));
    if (angle < 180 && angle > -90) {
      angle = angle - 90;
    }
    else if (angle <= -90) {
      angle = angle + 270;
    }
    else {
      angle = 90;
    }
    
  }
  return angle;
}

void debug() {
  Serial.print("Teach = ");
  Serial.print(TeachState);
  Serial.print(" Auto = ");
  Serial.print(AutonomousState);
  Serial.print(" D0 = ");
  Serial.print(digitalRead(5));
  Serial.print(", D1 = ");
  Serial.print(digitalRead(6));                                                                                                                                                                                                                                                         /* Sending data over NRF 24L01*/
  Serial.print(", A0 = ");
  Serial.print(analogRead(A0));
  Serial.print(", A1 = ");
  Serial.print(analogRead(A1));
  Serial.print(", A2 = ");
  Serial.print(analogRead(A2));
  Serial.print(", A3 = ");
  Serial.println(analogRead(A3));
}

void debugJoystickNormalised() {
  Serial.print("Left Joystick: ");
  Serial.print(frontback_left);
  Serial.print(" | ");
  Serial.print(leftright_left);
  Serial.print(" Right Joystick: ");
  Serial.print(frontback_right);
  Serial.print(" | ");
  Serial.println(leftright_right);
}

void debugVelocities() {
  Serial.print("Left Joystick: ");
  Serial.print(frontback_left_mag);
  Serial.print(" | ");
  Serial.print(leftright_left_mag);
  Serial.print(" Right Joystick: ");
  Serial.print(frontback_right_mag);
  Serial.print(" | ");
  Serial.println(leftright_right_mag);
}

void debugTransmission() {
  Serial.print("Tranmission Buffer:");
  Serial.print(transmission_buffer[0]);
  Serial.print("|");
  Serial.print(transmission_buffer[1]);                                                                                                                                                                                                                                                         /* Sending data over NRF 24L01*/
  Serial.print("|");
  Serial.print(transmission_buffer[2]);
  Serial.print("|");
  Serial.print(transmission_buffer[3]);
  Serial.print("|");
  Serial.print(transmission_buffer[4]);
  Serial.print("|");
  Serial.print(transmission_buffer[5]);
  Serial.print("|");
  Serial.println(transmission_buffer[6]);
}