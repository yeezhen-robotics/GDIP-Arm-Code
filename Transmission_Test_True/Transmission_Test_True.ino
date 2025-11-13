/*
* Arduino Wireless Communication Tutorial
*     Example 1 - Transmitter Code
*                
* by Dejan Nedelkovski, www.HowToMechatronics.com
* 
* Library: TMRh20/RF24, https://github.com/tmrh20/RF24/
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define CE_PIN 8
#define CSN_PIN 7

RF24 radio(CE_PIN, CSN_PIN); // CE, CSN

const byte address[6] = "00001";

void setup() {

  pinMode(CE_PIN, OUTPUT);
  pinMode(CSN_PIN, OUTPUT);
  digitalWrite(CE_PIN, LOW);   // CE low until configured
  digitalWrite(CSN_PIN, HIGH); // CSN high (not selected)

  Serial.begin(9600);
  radio.begin();
  delay(100);
  radio.openWritingPipe(address);
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_MIN);
  radio.setChannel(76);
  radio.stopListening();
  Serial.print("Chip:");
  Serial.println(radio.isChipConnected());
}

void loop() {
  int num[4] = {5,6,7,8};
  bool check = radio.write(num, sizeof(num));
  //Serial.print("Trying to transmit:");
  //Serial.println(num);
  //Serial.print("Check");
  //Serial.println(check);
  delay(1000);
}