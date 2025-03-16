#include "Secrets.h"
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "WiFi.h"
 
// const int A_Pin_01 = 34;   // Digital pin connected to the DHT sensor

 
#define AWS_IOT_PUBLISH_TOPIC   "esp32/pub"
#define AWS_IOT_SUBSCRIBE_TOPIC "esp32/sub"
 
int analog_data ;
 
WiFiClientSecure net = WiFiClientSecure();
PubSubClient client(net);
 
void connectAWS()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
 
  Serial.println("Connecting to Wi-Fi");
 
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
 
  // Configure WiFiClientSecure to use the AWS IoT device credentials
  net.setCACert(AWS_CERT_CA);
  net.setCertificate(AWS_CERT_CRT);
  net.setPrivateKey(AWS_CERT_PRIVATE);
 
  // Connect to the MQTT broker on the AWS endpoint we defined earlier
  client.setServer(AWS_IOT_ENDPOINT, 8883);
 
  // Create a message handler
  client.setCallback(messageHandler);
 
  Serial.println("Connecting to AWS IOT");
 
  while (!client.connect(THINGNAME))
  {
    Serial.print(".");
    delay(100);
  }
 
  if (!client.connected())
  {
    Serial.println("AWS IoT Timeout!");
    return;
  }

  // Subscribe to a topic
  client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);
  
 
  Serial.println("AWS IoT Connected!");
}
 
void publishMessage()
{
  StaticJsonDocument<200> doc;
  doc["Analog_Sensor_Data:"] = analog_data;
  // Serial.print("Debugging print data_1:");
  // Serial.println(analog_data);
  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer); // print to client
 
  int x = client.publish(AWS_IOT_PUBLISH_TOPIC, jsonBuffer);
  // Serial.print("AWS Subscribed Data: ");
  // Serial.println(x);
}
 
void messageHandler(char* topic, byte* payload, unsigned int length)
{
  Serial.print("incoming: ");
  Serial.println(topic);
 
  StaticJsonDocument<200> doc;
  deserializeJson(doc, payload);
  const char* message = doc["message"];
  Serial.println(message);
}
 
void setup()
{
  Serial.begin(115200);
  connectAWS();
}
 
void loop()
{
  analog_data = analogRead(A0);
 
 
  if (isnan(A0) )  // Check if any reads failed and exit early (to try again).
  {
    Serial.println(F("Failed to read from Analog sensor!"));
    return;
  }
 
  Serial.print(F("Analog_sensor_data: "));
  Serial.println(analog_data);
 
  publishMessage();
  client.loop();
  delay(1000);
}