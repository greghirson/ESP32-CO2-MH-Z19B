#include <Arduino_JSON.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <MHZ.h>



int greenPin = 32;
int redPin = 33;

MHZ co2(&Serial2, MHZ19B);

const char* ssid = "xxx";
const char* password = "xxx";
const int mqttPort = 1883;
const char* mqttServer = "192.168.4.61";

long lastMsg = 0;
int waitTime = 15000;

String sensorType = "MH-Z19B";
String sensorID = "1";
const char* topic = "home/CO2/CO2/";

WiFiClient espClient;
PubSubClient client(espClient);

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);
  delay(100);
  Serial.println("");
  Serial.println("");
  Serial.println("----------------------");
  Serial.println("----------------------");
  Serial.println("----------------------");
  Serial.println("----------------------");
  Serial.println("WiFi-MQTT-CO2 Sensor--");
  Serial.println("author: Greg Hirson---");

  Serial.println("MH-Z19B");

  // Sensor debug flag
  //co2.setDebug(true);

  // Start WIFI and MQTT

  Serial.print("Wifi Connecting to");
  Serial.println(ssid);

  int tryDelay = 500;
  int numberofTries = 20;

  // try connection

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    serial.print(".");
  }

  Serial.println("");
  Serial.print("Wifi Connected on: ");
  Serial.println(WiFi.localIP);

  client.setServer(mqttServer, mqttPort);

  // Start Serial for MH-Z19B

  Serial2.begin(9600);

  // Start LEDs

  pinMode(greenPin, OUTPUT);
  pinMode(redPin, OUTPUT);

  //Preheat

  long start = millis();

  if (co2.isPreHeating()) {
    digitalWrite(redPin, HIGH);
    digitalWrite(greenPin, LOW);
    Serial.print("Preheating");
    while (co2.isPreHeating()) {
      Serial.print(millis() - start);
      Serial.println("s");
      delay(5000);
    }
    Serial.println("");
    digitalWrite(redPin, LOW);
    digitalWrite(greenPin, HIGH);
  }
}

void loop() {


  // use waitTime, millis() loop instead of delay()

  if (millis() - lastMsg >= waitTime) {


    // check for MQTT connection

    if (!client.connected()) {
      reconnect();
    }


    int ppm_uart = co2.readCO2UART();


    if (ppm_uart > 0) {
      Serial.println(ppm_uart);
    } else if (ppm_uart == -2) {
      Serial.println("NO RESPONSE");
    } else if (ppm_uart == -5) {
      Serial.println("NOT READY");
    } else if (ppm_uart == -7) {
      Serial.println("SERIAL NOT CONFIGURED");
    }

    // Create JSON document to send

    StaticJsonDocument<128> doc;

    char output[128];

    doc["machine"] = sensorID;
    doc["CO2"] = ppm


    lastMsg = millis();
  }


}


void reconnect() {
  // Loop until reconnected
  while (!client.connected()) {
    Serial.print("Ateempting MQTT connection...");
    // Create Client
    String clientID = "ESP32Client-";
    clientID += sensorID;

    if (client.connect(clientID.c_str())) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.println(client.state());
      delay(2000);
    }
  }
}
