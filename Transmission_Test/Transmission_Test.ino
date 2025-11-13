#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define CE_PIN 8
#define CSN_PIN 7

RF24 radio(CE_PIN, CSN_PIN); // CE, CSN

const byte address[6] = "00001";

int num[4] = {0};

void setup() {

  pinMode(CE_PIN, OUTPUT);
  pinMode(CSN_PIN, OUTPUT);
  digitalWrite(CE_PIN, LOW);   // CE low until configured
  digitalWrite(CSN_PIN, HIGH); // CSN high (not selected)

  Serial.begin(9600);
  radio.begin();
  delay(100);
  radio.openReadingPipe(0, address);
  radio.setDataRate(RF24_1MBPS);   
  radio.setPALevel(RF24_PA_MIN);
  radio.setChannel(76);
  radio.startListening();
  Serial.print("Chip:");
  Serial.println(radio.isChipConnected());
}

void loop() {
  if (radio.available()) {
    radio.read(num, sizeof(num));
    Serial.print(num[0]);
    Serial.print(num[1]);
    Serial.print(num[2]);
    Serial.println(num[3]);
  }
}
