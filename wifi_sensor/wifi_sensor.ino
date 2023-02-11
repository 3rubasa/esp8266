// TODO: update these values
#define STASSID "rr"
#define STAPSK "User@1234"

// TODO: Remove this and use Gateway IP instead
#define SRV_ADDRESS "10.42.0.128"
#define SRV_PORT 3333

// TODO: Adjust these values
#define WAIT_FOR_CONNECT_ON_RESUME 10000 // ms
#define WAIT_FOR_CONNECT_ON_NEW 60000 // ms
#define DEEP_SEEP_LENGTH 600e6 // 10 min in us
#define SERIAL_BAUD_RATE 115200

#define SHT20_I2C_ADDRESS 64 // 0x40 in decimal

#define RTC_USER_DATA_SLOT_WIFI_STATE 33u

#include <ESP8266WiFi.h>
#include <include/WiFiState.h>  // WiFiState structure details

#include <Wire.h>
#include "DFRobot_SHT20.h"

WiFiState state;

const char* ssid = STASSID;
const char* password = STAPSK;
// TODO: Remove this and use Gateway IP instead
const char* host = SRV_ADDRESS;
const uint16_t port = SRV_PORT;

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  Serial.setDebugOutput(true);

  // May be necessary after deepSleep. Otherwise you may get "error: pll_cal exceeds 2ms!!!" when trying to connect
  delay(1);

  Serial.println("Getting the current temperature from sensor...");
  
  DFRobot_SHT20 sht20;
  sht20.initSHT20();                         // Init SHT20 Sensor
  delay(100);
  sht20.checkSHT20();                        // Check SHT20 Sensor

  float temperature = sht20.readTemperature();

  char strTemp[20];
  sprintf(strTemp, "%f", temperature);
  Serial.println(strTemp);
  // ---
  // Here you can do whatever you need to do that doesn't need a WiFi connection.
  // ---

  Serial.println("Trying to resume WiFi connection...");

  ESP.rtcUserMemoryRead(RTC_USER_DATA_SLOT_WIFI_STATE, reinterpret_cast<uint32_t*>(&state), sizeof(state));
  unsigned long start = millis();

  if (!WiFi.resumeFromShutdown(state) || (WiFi.waitForConnectResult(WAIT_FOR_CONNECT_ON_RESUME) != WL_CONNECTED)) {
    Serial.println("Cannot resume WiFi connection, connecting via begin...");
    WiFi.persistent(false);

    if (!WiFi.mode(WIFI_STA) || !WiFi.begin(ssid, password) || (WiFi.waitForConnectResult(WAIT_FOR_CONNECT_ON_NEW) != WL_CONNECTED)) {
      WiFi.mode(WIFI_OFF);
      Serial.println("Cannot connect!");
      Serial.flush();
      ESP.deepSleep(DEEP_SEEP_LENGTH, RF_DISABLED);
      return;
    }
  }

  unsigned long duration = millis() - start;
  Serial.printf("Duration: %f", duration * 0.001);
  Serial.println();

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("Gateway IP adderss: ");
  Serial.println(WiFi.gatewayIP());

  Serial.print("connecting to ");
  Serial.print(host);
  Serial.print(':');
  Serial.println(port);

  // Use WiFiClient class to create TCP connections
  WiFiClient client;
  if (!client.connect(host, port)) {
    Serial.println("connection failed");
    goto end;
  }

  // This will send a string to the server
  Serial.println("sending data to server");
  if (client.connected()) { client.println(strTemp); }

  // Close the connection
  Serial.println();
  Serial.println("closing connection");
  client.stop();

  // ---
  // Here you can do whatever you need to do that needs a WiFi connection.
  // ---

end:
  WiFi.shutdown(state);
  ESP.rtcUserMemoryWrite(RTC_USER_DATA_SLOT_WIFI_STATE, reinterpret_cast<uint32_t*>(&state), sizeof(state));

  // ---
  // Here you can do whatever you need to do that doesn't need a WiFi connection anymore.
  // ---

  Serial.println("Done.");
  Serial.flush();
  ESP.deepSleep(DEEP_SEEP_LENGTH, RF_DISABLED);
}

void loop() {
}
