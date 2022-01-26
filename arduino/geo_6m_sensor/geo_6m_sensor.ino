#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266WiFi.h>
void displayInfo();

// Choose two Arduino pins to use for software serial
//int RXPin = 5;
//int TXPin = 4;
const char *host = "94.237.109.173";
int GPSBaud = 9600;

// Create a TinyGPS++ object
TinyGPSPlus gps;

// Create a software serial port called "ss"
SoftwareSerial ss(4,5);

void setup()
{
  // Start the Arduino hardware serial port at 9600 baud
  //Serial.begin(115200);
  ss.begin(9600);
  //Serial.println();
  //Serial.print("Connecting to ");
  //Serial.println(ssid);
  WiFi.begin("wanja", "leawanja"); //WiFi connection

  while (WiFi.status() != WL_CONNECTED)
  { //Wait for the WiFI connection completion
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\n WiFi Connected");
  // Start the software serial port at the GPS's default baud
  Serial.begin(GPSBaud);
}

void loop()
{
  // This sketch displays information every time a new sentence is correctly encoded.
  while (ss.available() > 0)
    if (gps.encode(ss.read()))
      displayInfo();

  // If 5000 milliseconds pass and there are no characters coming in
  // over the software serial port, show a "No GPS detected" error
  if (millis() > 5000 )//&& gps.charsProcessed() < 10)
  {
    Serial.println("No GPS detected");
    delay(1000);
  }
}

void displayInfo()
{
  if (gps.location.isValid())
  {
    String data = "{\"lat\":";
    data.concat(String(gps.location.lat(), 6));
    data.concat(",\"lon\":");
    data.concat(String(gps.location.lng(), 6));
    data.concat("}");
    Serial.println(data);

    if (WiFi.status() == WL_CONNECTED)
    { //Check WiFi connection status

      WiFiClient client;
      if (!client.connect("94.237.109.173", 1880))
      {
        Serial.println("connection failed");
        return;
      }

      client.println("POST / HTTP/1.1");
      client.println("Host: 94.237.109.173");
      client.println("Accept: */*");
      client.println("Content-Type: application/x-www-form-urlencoded");
      client.print("Content-Length: ");
      client.println(data.length());
      client.println();
      client.println(data);

      delay(500); // Can be changed
      if (client.connected())
      {
        client.stop(); // DISCONNECT FROM THE SERVER
      }
      Serial.println();
      Serial.println("closing connection");
      delay(500);
    }
    else
    {

      Serial.println("Error in WiFi connection");
    }
  }
  else
  {
    Serial.println("Location: Not Available");
  }

  Serial.println();
  Serial.println();
  delay(1000);
}
