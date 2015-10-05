/*********
Ethershield
VCC  -  3.3V
GND  -  GND
INT  -  02
WOL  -  05
CS   -  10
SI   -  11
SO   -  12
SCK  -  13

Smoke Detector
VCC  -  5V
GND  -  GND
A0   -  A0
TTL  -  A1

RFID
SS/SDA -  07
RST    -  08
MOSI   -  11
MISO   -  12
SCK    -  13

Buzz -  03
Lamp -  04

Motion Sensor
VCC  -  5V
GND  -  GND
OUT  -  06
**********/
#define DEBUG

#define SS_PIN  7
#define RST_PIN  8

#define BUZZ  3
#define lamp 4
#define pirPin  6
#define door   A5
#define door1   A4

#define CLOSEBUTTON  A3

#ifdef DEBUG
#define P(x) Serial.print(x);
#define PL(x) Serial.println(x);
#define PV(x,y) Serial.print(x, y);
#else
#define P(x)  null;
#define PL(x) null;
#define PV(x,y) null;
#endif


#include "etherShield.h"
#include "ETHER_28J60.h"

#include <SPI.h>
#include <MFRC522.h>

static uint8_t mac[6] = {0x54, 0x55, 0x58, 0x10, 0x00, 0x24};
static uint8_t ip[4] = {192, 168, 148, 151};
static uint16_t port = 80;

ETHER_28J60 ethernet;
MFRC522 mfrc522(SS_PIN, RST_PIN);  // Create MFRC522 instance.

char curStat = LOW;

const int analogInPin = A0;  // Analog input pin that the sensor is attached to

int sensorValue = 0;        // value read from the pot
bool sensorReady = false;
bool thread = false;

bool intruder = false;

long unsigned int lowIn;
long unsigned int pause = 500;

boolean lockLow = true;
boolean takeLowTime;

boolean doorOpening = false;
boolean doorClosing = false;
long unsigned int openMilis = 0;
boolean doorOpened = false;
long unsigned int closeMilis = 0;
void setup()
{
#ifdef DEBUG
  Serial.begin(9600);
#endif

  //pintu
  pinMode(door, OUTPUT);
  pinMode(door1, OUTPUT);
  digitalWrite(door1,LOW);
  pinMode(CLOSEBUTTON, INPUT);
  digitalWrite(CLOSEBUTTON, HIGH);

  //reset all sensors, inputs and outputs
  PL(F("Loading..."));
  pinMode(A1, INPUT);
  pinMode(BUZZ, OUTPUT);
  digitalWrite(BUZZ, LOW);
  digitalWrite(lamp, curStat);
  PL(F("Turn off lamp ready"));


  //Preparing ethernet
  ethernet.setup(mac, ip, port);
  PL(F("Ethernet ready"));


  //Preparing smoke detector
  P(F("Smoke "));
  sensorValue = analogRead(analogInPin);

  while (!sensorReady) {
    if (sensorValue > 100 && sensorValue < 600) {
      sensorReady = true;
      PL(F("ready"));
    }
  }

  //preparing RFID
  SPI.begin();          // Init SPI bus
  mfrc522.PCD_Init();   // Init MFRC522 module
  P(F("RFID #:"));
  byte v = mfrc522.PCD_ReadRegister(mfrc522.VersionReg);
  PV(v, HEX);
  if ((v == 0x00) || (v == 0xFF)) {
    PL(F(" fail"));
  } else {
    PL(F(" ready"));
  }

  //Preparing PIR
  pinMode(pirPin, INPUT);
  digitalWrite(pirPin, LOW);
  P(F("Motion sensor "));
  for (int i = 0; i < 3; i++) { //30
    Serial.print(".");
    delay(1000);
  }
  PL(F(" ready"));


  PL(F("System ready"));
  digitalWrite(BUZZ, HIGH);
  delay(50);
  digitalWrite(BUZZ, LOW);
}

void loop()
{

  //Motion
  thread = !digitalRead(A1);
  if (thread) {
    digitalWrite(BUZZ, HIGH);
  } else {
    digitalWrite(BUZZ, LOW);
  }


  //ethernet
  char* params;
  if (params = ethernet.serviceRequest())
  {
    ethernet.print("<html><head><meta http-equiv='refresh' content='2; URL=http://192.168.148.151'-><style>.btn{text-decoration: none; border-radius: 5px; padding: 8px 15px; color: #FFF; font-size: 18px;}</style><head><body>");

    if (strcmp(params, "?cmd=on") == 0)
    {
      curStat = HIGH;
    }
    else if (strcmp(params, "?cmd=off") == 0)
    {
      curStat = LOW;
    }

    if (curStat == HIGH)
    {
      ethernet.print("<h1>Lampu Menyala<h1><a class='btn' style='background:#0e0e0e;' href='?cmd=off'>Matikan</a>");
    }
    else if (curStat == LOW)
    {
      ethernet.print("<h1>Lampu Mati<h1><a class='btn' style='background:#00ff00;' href='?cmd=on'>Nyalakan</a>");
    }

    if (!sensorReady) {
      ethernet.print("<h2> Sensor is warming up ... </h2>");
    }
    else
    {
      if (thread) {
        ethernet.print("<h2 color='#FF0000'> Flame detected... </h2>");
      }
    }

    if (intruder) {
      ethernet.print("<h2 color='#FF0000'> Motion detected... </h2>");
    }

    digitalWrite(lamp, curStat);

    ethernet.print("</body></html>");
    ethernet.respond();
  }

  //Motion sensor
  if (digitalRead(pirPin) == HIGH) {
    digitalWrite(BUZZ, HIGH);
    intruder = true;
    if (lockLow) {
      //makes sure wait for a transition to LOW before any further output is made:
      lockLow = false;
      P(F("-> motion detected"));
      delay(50);
    }
    takeLowTime = true;
  }

  if (digitalRead(pirPin) == LOW) {
    digitalWrite(BUZZ, LOW);
    intruder = false;

    if (takeLowTime) {
      lowIn = millis();          //save the time of the transition from high to LOW
      takeLowTime = false;       //make sure this is only done at the start of a LOW phase
    }
    //if the sensor is low for more than the given pause, means no more motion
    if (!lockLow && millis() - lowIn > pause) {
      //makes sure is only executed again after a new motion sequence has been detected
      lockLow = true;
      P(F("-> motion endded"));
      delay(50);
    }
  }


  //RFID
  if (doorOpened&&!doorClosing) {
    if (!digitalRead(CLOSEBUTTON)) {
      PL(F("Close Door"));
      closeDoor();
      digitalWrite(door1,HIGH);
      digitalWrite(door,LOW);
      doorClosing = true;
      closeMilis = millis();

    }
  }
  
  if(doorClosing){
    closeDoor();
  }


  if (!doorOpening && !doorClosing) {

    if ( ! mfrc522.PICC_IsNewCardPresent()) {
      return;
    }

    // Select one of the cards
    if ( ! mfrc522.PICC_ReadCardSerial()) {
      return;
    }

    P(F("CARD #:"));
    bool granted = dump_byte_array(mfrc522.uid.uidByte, mfrc522.uid.size);
    if (!granted) {
      PL(F(" Alien card...!"));
      digitalWrite(BUZZ, HIGH);
      delay(2);
      digitalWrite(BUZZ, LOW);
    } else {
      //open door //closedoor
      PL(F(" OK"));
      if (!doorOpened) {
        openDoor();
        digitalWrite(door, HIGH);
        doorOpening = true;
        openMilis = millis();
      }/* else {
        closeDoor();
        doorClosing = true;
        closeMilis = millis();
      }*/
    }
  } else if (doorOpening) {
    openDoor();
  }
  /*
  else if (doorClosing) {
    closeDoor();
  }
  */


}

byte master[7] = {0xED, 0xFE, 0xB1, 0xC5, 0x00, 0x00, 0x00};

bool dump_byte_array(byte *buffer, byte bufferSize) {
  bool match = true;
  for (byte i = 0; i < bufferSize; i++) {
    Serial.print(buffer[i] < 0x10 ? " 0" : " ");
    Serial.print(buffer[i], HEX);
    if (buffer[i] != master[i]) {
      match = false;
    }
  }
  return match;
}

void openDoor() {
  if (doorOpening) {
    //P(F(">"));
    if (millis() - openMilis > 2000) {
      PL(F("Door openned"));
      doorOpening = false;
      doorOpened = true;
      digitalWrite(door, LOW);
    }
  }
}

void closeDoor() {
  if (doorClosing) {
    //P(F("<"));
    if (millis() - closeMilis > 2000) {
      PL(F("Door closed"));
      doorClosing = false;
      doorOpened = false;
      digitalWrite(door,LOW);
      digitalWrite(door1,LOW);
    }
  }
}
