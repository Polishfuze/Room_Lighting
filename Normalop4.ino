/*
  Normal Operation Version 4 designed by PolishFuze(Michal Rajzer) This code is designed to control WS2812B LEDs and (finally) 4 x 2 blind motor array
*/


//~-----------------------------------------------------------------------------------------------------~


#define FASTLED_ALLOW_INTERRUPTS 0

#include <WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <FastLED.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>



#define WLAN_SSID "siec"
#define WLAN_PASS "Ikaro777"



#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "Michalek_raj"
#define AIO_KEY         "fbb1addce34243a7ab02164040e3f4a7"



#define NUM_LEDS 900
#define DATA_PIN 26
#define WE 27
#define LE 15



byte state [10] {0};
int timer[10] = { -1};
byte in[10];
byte dw[10];
byte up[10];
uint16_t fullt = 4500;

uint8_t bright = 0;
uint8_t satu = 0;
uint8_t colo = 0;
uint8_t main_lights = 25;
uint16_t mld = 0;

bool blb [10] {0};
bool doonce [10] {0};



uint8_t modey;

CHSV color = CHSV(0, 0, 0);

//~----------------------------------------------------------------------------------------------------~


CRGB leds [NUM_LEDS];


TaskHandle_t Task1;




WiFiClient client;

Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

Adafruit_MQTT_Subscribe rblindr = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/room.blr"); //Room BLIND Right

Adafruit_MQTT_Subscribe rblindl = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/room.bll"); //Room BLIND Left

Adafruit_MQTT_Subscribe modex = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/room.mode");

Adafruit_MQTT_Subscribe brightness = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/room.light-brightness");

Adafruit_MQTT_Subscribe hue = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/room.light-hue");

Adafruit_MQTT_Subscribe saturation = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/room.light-saturation");

Adafruit_MQTT_Subscribe mlights = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/room.mainlights");


//~----------------------------------------------------------------------------------------------------~


void setup() {
  Serial.begin(115200);
  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
  Serial.println("Starting up");

  ledcAttachPin(main_lights, 1);
  ledcSetup(1, 25000, 9);



  in[0] = 34;
  in[1] = 35;
  in[2] = 32;
  in[3] = 33;

  up[0] = 23;
  dw[0] = 22;

  up[1] = 21;
  dw[1] = 19;

  up[2] = 18;
  dw[2] = 5;

  up[3] = 4;
  dw[3] = 2;

  pinMode(LE, OUTPUT);
  pinMode(WE, INPUT_PULLUP);
  pinMode(main_lights, OUTPUT);

  pinMode(in[0] , INPUT);
  pinMode(dw[0] , OUTPUT);
  pinMode(up[0] , OUTPUT);

  pinMode(in[1] , INPUT);
  pinMode(dw[1] , OUTPUT);
  pinMode(up[1] , OUTPUT);

  pinMode(in[2] , INPUT);
  pinMode(dw[2] , OUTPUT);
  pinMode(up[2] , OUTPUT);

  pinMode(in[3] , INPUT);
  pinMode(dw[3] , OUTPUT);
  pinMode(up[3] , OUTPUT);

  

  digitalWrite(up[0], HIGH);
  digitalWrite(dw[0], HIGH);
  digitalWrite(up[1], HIGH);
  digitalWrite(dw[1], HIGH);
  digitalWrite(up[2], HIGH);
  digitalWrite(dw[2], HIGH);
  digitalWrite(up[3], HIGH);
  digitalWrite(dw[3], HIGH);


  if (digitalRead(WE) != HIGH)
  {
    wifistart(WLAN_SSID, WLAN_PASS);
    Serial.println("Starting OTA");
    ArduinoOTA.setHostname("Lights");
    ArduinoOTA.setPassword("admin");
    ArduinoOTA.begin();
    Serial.println("OTA ready");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    //MQTT_connect();
  }
  else
  {
    Serial.println("WiFi disabled");
  }

  xTaskCreatePinnedToCore(
    codeForTask1,            /* Task function. */
    "Task_1",                 /* name of task. */
    1000,                    /* Stack size of task */
    NULL,                     /* parameter of the task */
    1,                        /* priority of the task */
    &Task1,                   /* Task handle to keep track of created task */
    0);                       /* Core */

  mqtt.subscribe(&rblindr);
  mqtt.subscribe(&rblindl);
  mqtt.subscribe(&brightness);
  mqtt.subscribe(&hue);
  mqtt.subscribe(&saturation);
  mqtt.subscribe(&mlights);
  mqtt.subscribe(&modex);

}


//~----------------------------------------------------------------------------------------------------~


void loop() {
  ArduinoOTA.handle();

  if ((digitalRead(WE) == HIGH))
  {
    WiFi.disconnect();
    if (doonce[10] == 0)
    {
      Serial.println("WiFi is disabled");
      doonce[10] = 1;
    }
  }
  if ((!mqtt.connected()) && (digitalRead(WE) != HIGH) && (WiFi.status() != WL_CONNECTED))
  {
    MQTT_connect();
  }

  Adafruit_MQTT_Subscribe *subscription;

  if ((digitalRead(WE) != HIGH) && (WiFi.status() != WL_CONNECTED))
  {
    wifistart(WLAN_SSID, WLAN_PASS);
  }
  if ((digitalRead(WE) != HIGH) && (WiFi.status() == WL_CONNECTED))
  {
    ArduinoOTA.handle();
  }
  while (subscription = mqtt.readSubscription(1000)) {
    if (subscription == &mlights)
    {
      mld = map(atoi((char*)mlights.lastread), 0, 511, 511, 0);
      Serial.println(mld);
    }
    if (subscription == &modex)
    {
      modey = atoi((char*)modex.lastread);
      Serial.println((char *) modex.lastread);
    }
    if (subscription == &brightness)
    {
      bright = atoi((char *) brightness.lastread);
      Serial.println((char *) brightness.lastread);
    }
    if (subscription == &hue)
    {
      colo = atoi((char *) hue.lastread);
      Serial.println((char *) hue.lastread);
    }
    if (subscription == &saturation)
    {
      satu = atoi((char *) saturation.lastread);
      Serial.println((char *) saturation.lastread);
    }
    if (subscription == &rblindr)
    {
      blb[1] = atoi((char *) rblindr.lastread);
      Serial.println(blb[1]);
    }
    if (subscription == &rblindl)
    {
      blb[0] = atoi((char *) rblindl.lastread);
      Serial.println(blb[0]);
    }
  }
  Serial.println("h");
}


//~----------------------------------------------------------------------------------------------------~


void codeForTask1( void * parameter )
{
  for (;;)
  {
    if (modey != 0)
    {
      digitalWrite(LE, LOW);
    }
    else
    {
      digitalWrite(LE, HIGH);
    }
    //Serial.print("This Task runs on Core: ");  Serial.println(xPortGetCoreID());
    blind(0, in[0], up[0], dw[0]);
    blind(1, in[1], up[1], dw[1]);
    blind(2, in[2], up[2], dw[2]);
    blind(3, in[3], up[3], dw[3]);
    if (modey == 1)
    {
      color.val = bright;
      color.sat = satu;
      color.hue = colo;
      fill_solid(leds, NUM_LEDS, color);
    }
    if (modey == 2)
    {
      fill_rainbow(leds, NUM_LEDS, 0, 251);
    }
    if (modey == 3)
    {
      if (color.val != bright)
      {
        if (color.val > bright)
        {
          color.val--;
        }
        else
        {
          color.val++;
        }
      }
      if (color.sat != satu)
      {
        if (color.sat > satu)
        {
          color.sat--;
        }
        else
        {
          color.sat++;
        }
      }
      if (color.hue != colo)
      {
        if (color.hue > colo)
        {
          color.hue--;
        }
        else
        {
          color.hue++;
        }
      }
      fill_solid(leds, NUM_LEDS, color);
    }
    if (modey == 4)
    {
      color.val = bright;
      color.sat = satu;
      color.hue = colo;
      colo++;
      fill_solid(leds, NUM_LEDS, color);
      delay(10);
    }
    if (modey == 5)
    {
      fill_solid(leds, NUM_LEDS, color);
      delay(1);
    }
    FastLED.show();
    ledcWrite(1, mld);
  }
}


//~----------------------------------------------------------------------------------------------------~


int wifistart(char* ssid, char* password)
{
  uint8_t intt;
  Serial.println("Starting WiFi");
  WiFi.begin(ssid, password);
  Serial.print("Connecting to: ");
  Serial.print(ssid);
  Serial.println(" ");
  while ((WiFi.status() != WL_CONNECTED) && (WiFi.status() != WL_CONNECT_FAILED)) {
    FastLED.delay(1000);
    Serial.println(" . ");
    intt++;
    if (intt % 10 == 0)
    {
      Serial.println(WiFi.status());
    }
  }
  if (WiFi.status() == WL_CONNECT_FAILED)
  {
    Serial.println("Connection Failed");
  }
  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("Connected");
  }
  doonce[10] = 0;
}


//~----------------------------------------------------------------------------------------------------~


void blind(byte number, byte input, byte outputup, byte outputdown)
{
  //Serial.println(timer[number]);
  if (digitalRead(input) == HIGH)
  {
    state[number]++; //0 closed 1 opening 2 open 3 closing
    timer[number] = fullt; //Reset the timer to full time
    while (digitalRead(input) == HIGH)
    {
      delay(10);
    }
  }
  //~----------------------------------------------------------------------------------------------------~
  if (blb [number] == 1 && doonce [number] == 0)
  {
    state[number]++; //0 closed 1 opening 2 open 3 closing
    timer[number] = fullt; //Reset the timer to full time
    doonce [number] = 1;
  }
  if (blb [number] == 0 && doonce [number] == 1)
  {
    doonce [number] = 0;
  }
  //~----------------------------------------------------------------------------------------------------~
  if (state[number] == 4)
  {
    state[number] = 0;
  }
  if (state[number] == 1)
  {
    digitalWrite(outputdown, HIGH);
    digitalWrite(outputup, LOW);
  }
  if (state[number] == 3)
  {
    digitalWrite(outputup, HIGH);
    digitalWrite(outputdown, LOW);
  }
  if (state[number] == 0 || state[number] == 2)
  {
    digitalWrite(outputup, HIGH);
    digitalWrite(outputdown, HIGH);
  }
  Serial.println(state[number]);
  if (timer[number] > 0) {
    FastLED.delay(10);
    timer[number]--;
  }
  if (timer[number] < 1)
  {
    if (state[number] == 1)
    {
      state[number] = 2;
    }
    if (state[number] == 3)
    {
      state[number] = 0;
    }
  }
}


//~----------------------------------------------------------------------------------------------------~


void MQTT_connect() {
  Serial.print("Connecting to MQTT... ");
  mqtt.connect();
  while (mqtt.connect() != 0) { // connect will return 0 for connected
    Serial.println("Retrying MQTT connection in 10 seconds...");
    FastLED.delay(10000);
  }
  Serial.println("MQTT Connected!");
}

