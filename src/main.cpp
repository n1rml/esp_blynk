#include <Arduino.h>

#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <EEPROM.h>
#include <DNSServer.h>
#include <WebServer.h>
#include <ESP_WiFiManager.h>

#define LED 2
#define COUNT 4

#define ZCP 13

#define RELAY1 23
#define RELAY2 22
#define RELAY3 21
#define RELAY4 19

#define BTN1 25
#define BTN2 27
#define BTN3 26
#define BTN4 14

const char auth[] = "Blynk_Auth_Code";          // Put your device auth code from blynk

// H/W PinSet
byte rel[] = {RELAY1, RELAY2, RELAY3, RELAY4};

// Runtime Vars
int pstate[COUNT + 1];
unsigned int dim = 0;

void printState() {
  for ( int i = 0; i <= COUNT; i++ )
    Serial.print(pstate[i]);
  Serial.println();
}

void zcISR() {
  dim = 10000 - (2000 * pstate[3]);
  if (dim != 0) {
    delayMicroseconds(dim);
    digitalWrite(RELAY4, HIGH);
    
    delayMicroseconds(50);
    digitalWrite(RELAY4, LOW);
  }
}

void setRel(int relno, uint8_t relst)
{
  digitalWrite(rel[relno], relst);
  if (relst == HIGH)
  {
    pstate[relno] = 1;
    Blynk.virtualWrite(relno + 1, HIGH);
  }
  else
  {
    pstate[relno] = 0;
    Blynk.virtualWrite(relno + 1, LOW);
  }
  EEPROM.write(relno, pstate[relno]);
  EEPROM.commit();
  printState(); // FOR DEBUG
}

void settle(int wat)
{
  if (pstate[wat] == 0)
  {
    setRel(wat, HIGH);
  }
  else if (pstate[wat] == 1)
  {
    setRel(wat, LOW);
  }
  delayMicroseconds(50);
}

BLYNK_APP_CONNECTED()
{ // MOSTLY USELESS
  for (int i = 0; i < COUNT; i++)
  {
    if (pstate[i] == 1)
    {
      Blynk.virtualWrite(i + 1, HIGH);
    }
    else if (pstate[i] == 0)
    {
      Blynk.virtualWrite(i + 1, LOW);
    }
  }
}

BLYNK_WRITE(V1)
{
  int pinValue = param.asInt();
  if (pinValue == 1)
  {
    setRel(0, HIGH);
  }
  else
  {
    setRel(0, LOW);
  }
}

BLYNK_WRITE(V2)
{
  int pinValue = param.asInt();
  if (pinValue == 1)
  {
    setRel(1, HIGH);
  }
  else
  {
    setRel(1, LOW);
  }
}

BLYNK_WRITE(V3)
{
  int pinValue = param.asInt();
  if (pinValue == 1)
  {
    setRel(2, HIGH);
  }
  else
  {
    setRel(2, LOW);
  }
}

BLYNK_WRITE(V4)
{
  int pinValue = param.asInt();
  if (pinValue == 0)
  {
    setRel(3, LOW);
  }
  pstate[3] = pinValue;
}

BLYNK_WRITE(V8) // Reset Function    for V8 pin
{
  pstate[COUNT] = 0;
  Serial.println("Wifi Reset Initiated");
  Serial.println("Restarting in 3 Seconds");
  delay(3000);
  ESP.restart();
}

void ISR1(){
  settle(0);
}
void ISR2(){
  settle(1);
}
void ISR3(){
  settle(2);
}
// void ISR4(){
//   settle(3);
// }

void setup()
{
  ESP_WiFiManager wifiManager;
  // wifiManager.resetSettings();
  WiFi.begin(WiFi.SSID().c_str(), WiFi.psk().c_str());
  Serial.begin(115200);
  EEPROM.begin(COUNT + 1);

  pinMode(ZCP, INPUT_PULLUP);

  for (int i = 0; i < COUNT; i++)
  {
    pinMode(rel[i], OUTPUT);
    pstate[i] = EEPROM.read(i);
    if (i < 3)
    {
      digitalWrite(rel[i], pstate[i]);
      Blynk.virtualWrite(i + 1, pstate[i]);
    }
    else
    {
      Blynk.virtualWrite(i + 1, pstate[i]);
    }
  }
  Serial.println("\n\n");

  printState();

  if (EEPROM.read(COUNT) != 5)
  {
    wifiManager.autoConnect("Smart Switch");
    Serial.println("*****************************************");
  }

  Blynk.config(auth);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);

  attachInterrupt(digitalPinToInterrupt(BTN1), ISR1, CHANGE);     // Digital button Interrupts 
  attachInterrupt(digitalPinToInterrupt(BTN2), ISR2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BTN3), ISR3, CHANGE);

  attachInterrupt(digitalPinToInterrupt(ZCP), zcISR, FALLING);    // ZeroCross Interrupt

}

void loop()
{
  if (WiFi.status() == WL_CONNECTED)
  {
    digitalWrite(LED, LOW);
    Blynk.run();
    if (pstate[COUNT] != 5) {
      pstate[COUNT] = 5;      // Set Connected Flag
      EEPROM.write(COUNT, 5);
      EEPROM.commit();
    }
  }
  else
  {
    digitalWrite(LED, HIGH);
    WiFi.reconnect();
    Blynk.connect();
    digitalWrite(LED, LOW);
  }
}


