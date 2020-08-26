#include <Arduino.h>

#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <SPIFFS.h>
#include <DNSServer.h>
#include <WebServer.h>
#include <WiFiManager.h>

#define LED 2
#define COUNT 4

#define RELAY1 23
#define RELAY2 22
#define RELAY3 21
#define RELAY4 19

#define BTN1 25
#define BTN2 27
#define BTN3 26
#define BTN4 14

const char *filename = "/file.txt";
const char auth[] = " Blynk Auth Code ";          // Put your device auth code from blynk

// H/W PinSet
byte rel[] = {RELAY1, RELAY2, RELAY3, RELAY4};

// Runtime Vars
char pstate[COUNT + 1];

BLYNK_APP_CONNECTED()
{ // MOSTLY USELESS
  for (int i = 0; i < COUNT; i++)
  {
    if (pstate[i] == '!')
    {
      Blynk.virtualWrite(i + 1, HIGH);
    }
    else if (pstate[i] == 'o')
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
  if (pinValue == 1)
  {
    setRel(3, HIGH);
  }
  else
  {
    setRel(3, LOW);
  }
}

BLYNK_WRITE(V8) // Reset Function    for V8 pin
{
  pstate[COUNT] = 'n';
  SPIFFS.remove(filename);
  File f = SPIFFS.open(filename, FILE_WRITE);
  f.print(pstate);
  f.close();
  Serial.println("Wifi Reset Initiated");
  Serial.println("Restarting in 3 Seconds");
  delay(3000);
  ESP.restart();
}

// It looks a bit silly, but only way to use args in interrupts.
void ISR1(){
  settle(0);
}
void ISR2(){
  settle(1);
}
void ISR3(){
  settle(2);
}
void ISR4(){
  settle(3);
}

void setup()
{
  WiFiManager wifiManager;
  // wifiManager.resetSettings();
  WiFi.begin(WiFi.SSID().c_str(), WiFi.psk().c_str());
  Serial.begin(115200);
  if (SPIFFS.begin())
  {
    Serial.println("SPIFFS Initialize....ok");
  }
  else
  {
    Serial.println("SPIFFS Initialization...failed");
  }

  File f = SPIFFS.open(filename);

  if (!f)
  {
    Serial.println(" File open failed ");
  }

  for (int i = 0; i < COUNT; i++)
  {
    pinMode(rel[i], OUTPUT);

    if ((char)f.read() == '!')
    {
      digitalWrite(rel[i], HIGH);
      Blynk.virtualWrite(i + 1, HIGH);
      pstate[i] = '!';
    }
    else
    {
      digitalWrite(rel[i], LOW);
      Blynk.virtualWrite(i + 1, LOW);
      pstate[i] = 'o';
    }
  }
  Serial.println("\n\n");
  Serial.println(pstate);
  if ((char)f.read() != 's')
  {
    wifiManager.autoConnect("Smart Home");
    Serial.println("*****************************************");
  }
  f.close();

  Blynk.config(auth);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);

  attachInterrupt(digitalPinToInterrupt(BTN1), ISR1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BTN2), ISR2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BTN3), ISR3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BTN4), ISR4, CHANGE);

}

void loop()
{
  if (WiFi.status() == WL_CONNECTED)
  {
    digitalWrite(LED, LOW);
    Blynk.run();
    if (pstate[COUNT] != 's')
      pstate[COUNT] = 's'; // Flag Setup Complete.
  }
  else
  {
    digitalWrite(LED, HIGH);
    WiFi.reconnect();
    Blynk.connect();
    digitalWrite(LED, LOW);
  }
}

void setRel(int relno, uint8_t relst)
{
  digitalWrite(rel[relno], relst);
  if (relst == HIGH)
  {
    pstate[relno] = '!';
    Blynk.virtualWrite(relno + 1, HIGH);
  }
  else
  {
    pstate[relno] = 'o';
    Blynk.virtualWrite(relno + 1, LOW);
  }
  SPIFFS.remove(filename);
  File f = SPIFFS.open(filename, FILE_WRITE);

  if (!f)
  {
    Serial.println("file open failed");
  }
  else
    f.print(pstate);
  f.close();
  Serial.println(pstate); // FOR DEBUG
}

void settle(int wat)
{
  if (pstate[wat] == 'o')
  {
    setRel(wat, HIGH);
  }
  else if (pstate[wat] == '!')
  {
    setRel(wat, LOW);
  }
}
