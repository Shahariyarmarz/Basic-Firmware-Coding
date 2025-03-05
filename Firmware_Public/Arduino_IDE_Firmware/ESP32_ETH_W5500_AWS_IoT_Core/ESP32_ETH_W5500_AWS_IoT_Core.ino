/*
    This sketch shows the Ethernet event usage

*/
#include "Secrets.h"
#include <ETH.h>
#include <SPI.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>


#define AWS_IOT_PUBLISH_TOPIC "esp32/pub"
#define AWS_IOT_SUBSCRIBE_TOPIC "esp32/sub"

int analog_data;

#define ETH_PHY_TYPE ETH_PHY_W5500
#define ETH_PHY_ADDR 1
#define ETH_PHY_CS 33
#define ETH_PHY_IRQ -1
#define ETH_PHY_RST 27

// SPI pins
#define ETH_SPI_SCK 25
#define ETH_SPI_MISO 36
#define ETH_SPI_MOSI 26

static bool eth_connected = false;

WiFiClientSecure net;  // Secure client for TLS connection
PubSubClient client(net);

void onEvent(arduino_event_id_t event, arduino_event_info_t info) {
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      Serial.println("ETH Started");
      //set eth hostname here
      ETH.setHostname("esp32-eth0");
      break;
    case ARDUINO_EVENT_ETH_CONNECTED: Serial.println("ETH Connected"); break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      Serial.printf("ETH Got IP: '%s'\n", esp_netif_get_desc(info.got_ip.esp_netif));
      Serial.println(ETH);
      eth_connected = true;
      break;
    case ARDUINO_EVENT_ETH_LOST_IP:
      Serial.println("ETH Lost IP");
      eth_connected = false;
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      Serial.println("ETH Disconnected");
      eth_connected = false;
      break;
    case ARDUINO_EVENT_ETH_STOP:
      Serial.println("ETH Stopped");
      eth_connected = false;
      break;
    default: break;
  }
}

//----------------------------------------------------AWS Addition with MQTT-------------------------------------------------
// // MQTT Callback function
// void callback(char* topic, byte* payload, unsigned int length) {
//   Serial.print("Message received on topic: ");
//   Serial.println(topic);
//   Serial.print("Message: ");
//   for (unsigned int i = 0; i < length; i++) {
//     Serial.print((char)payload[i]);
//   }
//   Serial.println();
// }

// MQTT Connection Function
void connectAWS() {
  //----------------------ETHERNET SETUP-------------------------------------------
  SPI.begin(ETH_SPI_SCK, ETH_SPI_MISO, ETH_SPI_MOSI, ETH_PHY_CS);
  ETH.begin(ETH_PHY_TYPE, ETH_PHY_ADDR, ETH_PHY_CS, ETH_PHY_IRQ, ETH_PHY_RST, SPI);

//-----------------------------------AWS SETUP------------------------------------------------
  Serial.println("Connecting to AWS IoT Core...");
  net.setCACert(AWS_CERT_CA);
  net.setCertificate(AWS_CERT_CRT);
  net.setPrivateKey(AWS_CERT_PRIVATE);

  client.setServer(AWS_IOT_ENDPOINT, 8883);
  client.setCallback(messageHandler);

  while (!client.connect(THINGNAME)) {
    Serial.print(".");
    delay(100);
  }

  if (!client.connected()) {
    Serial.println("AWS IoT Timeout!");
    return;
  }
  // Subscribe to a topic
  client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);
  Serial.println("AWS IoT Connected!");
}

void publishData() {

  // Example JSON payload
  // String payload = "{ \"temperature\": 25.4, \"humidity\": 60.5 }";

  // Serial.print("Publishing message: ");
  // Serial.println(payload);

  // if (client.publish(topic, payload.c_str())) {
  //   Serial.println("Message sent!");
  // } else {
  //   Serial.println("Message failed to send!");
  // }

  StaticJsonDocument<200> doc;
  doc["Analog_Sensor_Data:"] = analog_data;
  // Serial.print("Debugging print data_1:");
  // Serial.println(analog_data);
  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer);  // print to client

  int x = client.publish(AWS_IOT_PUBLISH_TOPIC, jsonBuffer);
  Serial.println(x);
}


void messageHandler(char* topic, byte* payload, unsigned int length) {
  Serial.print("incoming: ");
  Serial.println(topic);

  StaticJsonDocument<200> doc;
  deserializeJson(doc, payload);
  const char* message = doc["message"];
  Serial.println(message);
}
// void testClient(const char *host, uint16_t port) {
//   Serial.print("\nconnecting to ");
//   Serial.println(host);

//   NetworkClient client;
//   if (!client.connect(host, port)) {
//     Serial.println("connection failed");
//     return;
//   }
//   client.printf("GET / HTTP/1.1\r\nHost: %s\r\n\r\n", host);
//   while (client.connected() && !client.available())
//     ;
//   while (client.available()) {
//     Serial.write(client.read());
//   }

//   Serial.println("closing connection\n");
//   client.stop();
// }

void setup() {
  Serial.begin(115200);
  Network.onEvent(onEvent);
  connectAWS();

  // SPI.begin(ETH_SPI_SCK, ETH_SPI_MISO, ETH_SPI_MOSI, ETH_PHY_CS);
  // ETH.begin(ETH_PHY_TYPE, ETH_PHY_ADDR, ETH_PHY_CS, ETH_PHY_IRQ, ETH_PHY_RST, SPI);

  // if (!client.connected()) {
  //   connectAWS();
  // }
}

void loop() {
  // analog_data = analogRead(34); // Ensure it's an ADC1 pin (32-39)
  
  if (!eth_connected) {
    Serial.println("Ethernet Disconnected. Retrying...");
    connectAWS();
  }

  if (client.connected()) {
    analog_data = analogRead(39);
    Serial.print("Analog_sensor_data: ");
    Serial.println(analog_data);
    publishData();  // Publish data
  } else {
    Serial.println("Reconnecting to AWS...");
    connectAWS();
  }

  client.loop();  
  delay(5000); // Adjusted delay to avoid AWS IoT throttling
}

