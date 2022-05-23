#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <ESP8266MQTTClient.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <GyverMAX6675.h>
#include "max6675.h"
#include "GyverPID.h"
#include <SD.h>
#if defined(ESP8266)
#define BUTTON_A  0
#define BUTTON_B 16
#define BUTTON_C  2
#define CLK_PIN   13  // Пин SCK
#define DATA_PIN  12  // Пин SO
#define CS_PIN    10  // Пин CS
#define BUFFER_SIZE 100
#define WIRE Wire
#define CLOSEVALVE 6 // пин для открытия вентиля 
#define OPENVALVE 7 // пин для открытия вентиля 
#endif

const char *ssid =  "AIRPORT";  // Имя вайфай точки доступа
const char *pass =  "PASSWORD"; // Пароль от точки доступа
const char *mqtt_server = "server"; // Имя сервера MQTT
const int mqtt_port = 11140; // Порт для подключения к серверу MQTT
const char *mqtt_user = "Login"; // Логи от сервер
const char *mqtt_pass = "Pass"; // Пароль от сервера

bool LedState = false;
int tm = 300;
float temp = 0;
int temperature = 0;

WiFiClient wclient;
PubSubClient client(wclient, mqtt_server, mqtt_port);
Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &WIRE);
GyverPID regulator(0.1, 0.05, 0.01, 10);
File myFile;
GyverMAX6675<CLK_PIN, DATA_PIN, CS_PIN> sens;   // программный SPI
//GyverMAX6675_SPI<CS_PIN> sens


typedef struct {
  float k1;
  float k2;
  float k3;
} Message;


void setup() {
  initTempSensor();
  initMQTTConection();
  initPIDRegulator(20);
  initOLED();
  initSD();
}

void loop() {
  if (sens.readTemp()) {
    temperature = sens.getTemp();
    sendTempMQTT(temperature);
  }
  if (WiFi.status() == WL_CONNECTED) {
    if (!client.connected()) {
      Serial.println("Connecting to MQTT server");
      if (client.connect(MQTT::Connect("arduinoClient2")
                         .set_auth(mqtt_user, mqtt_pass))) {
        Serial.println("Connected to MQTT server");
        client.set_callback(callback);
        client.subscribe("test/led");
      } else {
        Serial.println("Could not connect to MQTT server");
      }
    }

    if (client.connected()) {
      client.loop();
      sendTempMQTT(temperature);
    }

  }
  drawInformation(temperature, regulator);
  writeLog(temperature, regulator);
  sendRegulatoryInfluence(temperature, regulator);
}



void initPIDRegulator(int temp) {
  regulator.setDirection(NORMAL);
  regulator.setLimits(0, 255);
  regulator.setpoint = 50;
  regulator.input = temp;
}


void initMQTTConection() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.print("Connecting to ");
    Serial.print(ssid);
    Serial.println("...");
    WiFi.begin(ssid, pass);

    if (WiFi.waitForConnectResult() != WL_CONNECTED)
      return;
    Serial.println("WiFi connected");

    if (!client.connected()) {
      Serial.println("Connecting to MQTT server");
      if (client.connect(MQTT::Connect("arduinoClient2")
                         .set_auth(mqtt_user, mqtt_pass))) {
        Serial.println("Connected to MQTT server");
        client.subscribe("test/led"); // подписывааемся по топик с данными для светодиода
      } else {
        Serial.println("Could not connect to MQTT server");
      }
    }
  } else {
    Serial.println("ESP8266 not connect to Wi-Fi");
  }
}


void initTempSensor() {}

void initOLED() {
  display.clearDisplay();
  display.display();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("Connecting to SSID\n'adafruit':");
  display.print("connected!");
  display.println("IP: 10.0.1.23");
  display.println("Sending val #0");
  display.setCursor(0, 0);
  display.display(); // actually display all of the above
}

void initSD() {
  Serial.begin(9600);
  while (!Serial) {
    ;
  }
  Serial.print("Initializing SD card...");
  if (!SD.begin(4)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");
  myFile = SD.open("test.txt", FILE_WRITE);
  if (myFile) {
    Serial.print("Writing to test.txt...");
    myFile.println("testing 1, 2, 3.");
    myFile.close();
    Serial.println("done.");
  } else {
    Serial.println("error opening test.txt");
  }
}

void sendTempMQTT(int temp) {
  if (client.connected()) {
    client.loop();
    client.publish("test/temp", String(temp));
    Serial.println(temp);
  }
}


void callback(const MQTT::Publish& pub)
{
  String payload = pub.payload_string();
  Message answer = parseCallback(payload);
  regulator.Kp = answer.k1;
  regulator.Ki = answer.k2;
  regulator.Kd = answer.k3;
}

Message parseCallback(String value) {
  Message answer;
  int ind1 = value.indexOf(',');
  answer.k1 = value.substring(0, ind1).toFloat();
  int ind2 = value.indexOf(',', ind1 + 1 );
  answer.k2 = value.substring(ind1 + 1, ind2 + 1).toFloat();
  int ind3 = value.indexOf(',', ind2 + 1 );
  answer.k3 = value.substring(ind2 + 1, ind3 + 1).toFloat();
  return answer;
}

void drawInformation(float temp, GyverPID regulator ) {
  display.setCursor(0, 0);
  display.print("Temperature':");
  display.print(temp);
  display.println();
  display.print("PID':kp ");
  display.print(regulator.Kp);
  display.print(" ki: ");
  display.print(regulator.Ki);
  display.print(" kd: ");
  display.print(regulator.Kd);
  display.display(); // actually display all of the above
}

void  writeLog(float temp, GyverPID regulator ) {
  myFile = SD.open("test.txt", FILE_WRITE);
  if (myFile) {
    myFile.print("Temperature':");
    myFile.print(temp);
    myFile.println();
    myFile.print("PID':kp ");
    myFile.print(regulator.Kp);
    myFile.print(" ki: ");
    myFile.print(regulator.Ki);
    myFile.print(" kd: ");
    myFile.print(regulator.Kd);
    myFile.println();
    myFile.close();
  } else {
    Serial.println("error opening test.txt");
  }
}

void sendRegulatoryInfluence(float temp, GyverPID regulator) {
  regulator.input = temp;
  if (regulator.output > 126) {
    digitalWrite(CLOSEVALVE, 0);
    digitalWrite(OPENVALVE, 1);
  } else {
    digitalWrite(CLOSEVALVE, 1);
    digitalWrite(OPENVALVE, 2);
  }
}
