/*********
  Modified from the examples of the Arduino LoRa library
  More resources: https://randomnerdtutorials.com
*********/

#include <SPI.h>
#include <LoRa.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>

// Change the credentials below, so your ESP8266 connects to your router
const char *ssid = "BlackSkin";
const char *password = "mqttnetty";

//Your Domain name with URL path or IP address with path
const char *serverName = "http://139.59.181.160:1880/data";

//define the pins used by the transceiver module
#define ss 15
#define rst 0
#define dio0 5

void setup()
{
  //initialize Serial Monitor
  Serial.begin(115200);

  //Set up Wifi Connection
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    yield();
  }
  Serial.println("WIFI_INIT_OK");
  Serial.print("WiFi connected -IP address: ");
  Serial.println(WiFi.localIP());

  //setup LoRa transceiver module
  LoRa.setPins(ss, rst, dio0);
  while (!LoRa.begin(868100000))
  {
    yield();
  }
  LoRa.setSyncWord(0x39);
  LoRa.setSpreadingFactor(12);
  LoRa.setSignalBandwidth(125000);
  Serial.println("RFM_INIT_OK");
}

void loop()
{

  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize)
  {
    // read packet
    String LoRaData;
    while (LoRa.available())
    {
      LoRaData = LoRa.readString();
    }

    WiFiClient client;
    HTTPClient http;

    // Your Domain name with URL path or IP address with path
    http.begin(client, serverName);

    // Send HTTP POST request
    http.addHeader("Content-Type", "application/json");
    String data = "{\"data\":\"";
    data.concat(LoRaData);
    data.concat("\"}");
    Serial.println(data);
    int httpResponseCode = http.POST(data);

    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
    // Free resources
    http.end();
  }
}
