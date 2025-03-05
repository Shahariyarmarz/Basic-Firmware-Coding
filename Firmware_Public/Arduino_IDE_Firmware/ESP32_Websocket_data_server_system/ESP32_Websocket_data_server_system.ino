#include <ETH.h>
#include <WebSocketsServer.h>

// Ethernet Configuration
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
WebSocketsServer webSocket = WebSocketsServer(81);  // WebSocket server on port 81

// Analog Pin for sensor data
#define SENSOR_PIN 39  // Replace with your sensor's analog pin

// WebSocket Event Handler
void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      Serial.println("Client Connected!");
      webSocket.sendTXT(num, "Hello from ESP32 Ethernet!");
      break;
    case WStype_TEXT:
      Serial.printf("Received: %s\n", payload);
      break;
  }
}

// Ethernet Event Handler
void onEvent(arduino_event_id_t event, arduino_event_info_t info) {
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      Serial.println("ETH Started");
      ETH.setHostname("esp32-eth0");
      break;
    case ARDUINO_EVENT_ETH_CONNECTED:
      Serial.println("ETH Connected");
      break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      Serial.printf("ETH Got IP: %s\n", ETH.localIP().toString().c_str());
      eth_connected = true;
      webSocket.begin();  // Start WebSocket Server
      webSocket.onEvent(webSocketEvent);
      break;
    case ARDUINO_EVENT_ETH_LOST_IP:
    case ARDUINO_EVENT_ETH_DISCONNECTED:
    case ARDUINO_EVENT_ETH_STOP:
      Serial.println("ETH Disconnected");
      eth_connected = false;
      break;
    default:
      break;
  }
}

void setup() {
  Serial.begin(115200);
  Network.onEvent(onEvent);
  SPI.begin(ETH_SPI_SCK, ETH_SPI_MISO, ETH_SPI_MOSI, ETH_PHY_CS);
  ETH.begin(ETH_PHY_TYPE, ETH_PHY_ADDR, ETH_PHY_CS, ETH_PHY_IRQ, ETH_PHY_RST, SPI);
}

void loop() {
  if (eth_connected) {
    webSocket.loop();  // Handle WebSocket events

    // Read analog sensor data (0 to 4095 for 12-bit ADC)
    int sensorValue = analogRead(SENSOR_PIN);

    // Convert to a string for WebSocket transmission
    String sensorData = String(sensorValue);

    // Send the sensor data to all connected WebSocket clients
    webSocket.broadcastTXT(sensorData);

    // Print the sensor data to Serial Monitor
    Serial.println(sensorData);

    delay(500);  // Adjust delay to control the data publishing frequency
  }
}
