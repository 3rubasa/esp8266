// TODO: update these values
#define STASSID "rr"
#define STAPSK "User@1234"

// The client will connect to the hardcoded IP-address 10.0.0.20
#define SRV_PORT 3333

#define WAIT_FOR_CONNECT_ON_RESUME 10000 // ms
#define WAIT_FOR_CONNECT_ON_NEW 15000 // ms
#define DEEP_SEEP_LENGTH 60e6 // 1 minute in us
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

const uint16_t port = SRV_PORT;

ADC_MODE(ADC_VCC);

void setup() {
  // Turn off the radio module so that it does not consume energy when we are obtaining the temperature reading
  WiFi.mode( WIFI_OFF );
  WiFi.forceSleepBegin();
  delay( 1 );

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
  Serial.print("Temperature = ");
  Serial.println(temperature);

  float mVcc = ESP.getVcc() / 1000.00;
  float rVcc = GetRealVoltage(mVcc);

  Serial.print("Measured Voltage = ");
  Serial.println(mVcc);
  Serial.print("Real Voltage = ");
  Serial.println(rVcc);
  // ---
  // Here you can do whatever you need to do that doesn't need a WiFi connection.
  // ---

  // Prepare JSON to be sent to the server
  char message[512];
  sprintf(message, "{\"temperature\":%f,\"mvcc\":%f,\"rvcc\":%f}", temperature, mVcc, rVcc);
  Serial.print("Message to the server: ");
  Serial.println(message);

  Serial.println("Trying to resume WiFi connection...");

  WiFi.forceSleepWake();
  delay( 1 );

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

  //IPAddress gwIP = WiFi.gatewayIP();
  IPAddress gwIP(10,0,0,20);
  Serial.println("Gateway IP adderss: ");
  Serial.println(gwIP);

  Serial.print("connecting to ");
  Serial.print(gwIP);
  Serial.print(':');
  Serial.println(port);

  // Use WiFiClient class to create TCP connections
  WiFiClient client;
  if (!client.connect(gwIP, port)) {
    Serial.println("connection failed");
    goto end;
  }

  // This will send a string to the server
  Serial.println("sending data to server");
  if (client.connected()) { client.println(message); }
  client.flush();

  delay( 1000 );

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

// Experimental data
// Real     Measured
// 2.1 - Fails 
// 2.2 - 2.51
// 2.3 - 2.63
// 2.4 - 2.74
// 2.5 - 2.87
// 2.6 - 2.98
// 2.7 - 3.1
// 2.8 - 3.21
// 2.9 - 3.33
// 3.0 - 3.44
// 3.1 - 3.6
// 3.2 - 3.7
// 3.3 - 3.82
// 3.4 - 3.95
// 3.5 - 4.07
// 3.6 - 4.18
// 3.7 - 4.29
// 3.8 - 4.41
// 3.9 - 4.47

float GetRealVoltage(float m) {
  if (m > 4.47) 
    return 100.0;
  else if (m < 2.51)
    return -100.0;
  else if (m <= 2.63 && m >= 2.51)
    return 2.2 + (m - 2.51) / (2.63 - 2.51) / 10;
  else if (m <= 2.74 && m > 2.63)
    return 2.3 + (m - 2.63) / (2.74 - 2.63) / 10;
  else if (m <= 2.87 && m > 2.74)
    return 2.4 + (m - 2.74) / (2.87 - 2.74) / 10;
  else if (m <= 2.98 && m > 2.87)
    return 2.5 + (m - 2.87) / (3.98 - 2.87) / 10;
  else if (m <= 3.1 && m > 2.98)
    return 2.6 + (m - 2.98) / (3.1 - 2.98) / 10;
  else if (m <= 3.21 && m > 3.1)
    return 2.7 + (m - 3.1) / (3.21 - 3.1) / 10;
  else if (m <= 3.33 && m > 3.21)
    return 2.8 + (m - 3.21) / (3.33 - 3.21) / 10;
  else if (m <= 3.44 && m > 3.33)
    return 2.9 + (m - 3.33) / (3.44 - 3.33) / 10;
  else if (m <= 3.6 && m > 3.44)
    return 3.0 + (m - 3.44) / (3.6 - 3.44) / 10;
  else if (m <= 3.7 && m > 3.6)
    return 3.1 + (m - 3.6) / (3.7 - 3.6) / 10;
  else if (m <= 3.82 && m > 3.7)
    return 3.2 + (m - 3.7) / (3.82 - 3.7) / 10;
  else if (m <= 3.95 && m > 3.82)
    return 3.3 + (m - 3.82) / (3.95 - 3.82) / 10;
  else if (m <= 4.07 && m > 3.95)
    return 3.4 + (m - 3.95) / (4.07 - 3.95) / 10;
  else if (m <= 4.18 && m > 4.07)
    return 3.5 + (m - 4.07) / (4.18 - 4.07) / 10;
  else if (m <= 4.29 && m > 4.18)
    return 3.6 + (m - 4.18) / (4.29 - 4.18) / 10;
  else if (m <= 4.41 && m > 4.29)
    return 3.7 + (m - 4.29) / (4.41 - 4.29) / 10;
  else if (m <= 4.47 && m > 4.41)
    return 3.8 + (m - 4.41) / (4.47 - 4.41) / 10;

  return -100.0;
}