#include <PubSubClient.h>
#include <ESP8266WiFi.h>

const char* WIFI_SSID = "Faiba4G_148A5E";
const char* WIFI_PASSWORD = "35311849";
const char* mqttServer = "broker.mqttdashboard.com";
const int mqttPort = 8000;
const char* mqttUser = "xxxxxxxx";
const char* mqttPassword = "xxxxxxxx";
const char* subTopic = "leah/sub";
const char* pubTopic = "leah/pub";

uint8_t trigPin = A2, echoPin = A3;

void setup(){
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    Serial.begin(9600);
    WiFi.begin(WIFI_SSID,WIFI_PASSWORD);

    while ( WiFi.status() != WL_CONNECTED ) {

    }

    mqttClient.setServer(mqttServer, mqttPort);
    mqttClient.setCallback(mqttCallback);

    while (!mqttClient.connected()) {
        Serial.println("Connecting to MQTT...");
    
        if (mqttClient.connect("ESP8266Client", mqttUser, mqttPassword )) {

            Serial.println("mqtt connected");  

        } else {

            Serial.print("mqtt failed with state ");
            Serial.print(mqttClient.state());
            delay(2000);

        }
    }

    mqttClient.subscribe(subTopic);
}

void loop(){
    float distance = getDistance();
    Serial.println(distance);
    // wait a little between measurements
    mqttclient.publish(pubTopic, "hello"); //Topic name
    delay(5000);
    mqttClient.loop();
}

float getDistance(){
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    float distance = pulseIn(echoPin, HIGH, 5000);
    // scale the distance from "echo" pulse duration to centimetersas specified on the HC -SR04 datasheet
    distance = distance / 5.8 / 10;
    return distance;
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
    Serial.print("Message arrived in topic: ");
    Serial.println(topic);
    Serial.print("Message: ");
    String msg = "";
    for (int i = 0; i < length; i++) {
        //Serial.print((char)payload[i]);
        msg += (char)payload[i];
    }
    if(msg.length() > 1){
       displayMsg = msg;
    }else{
      if(msg == COMMAND_STOP_MSG){
        displayMsg = "";
      }else{
        mqttMsg = msg;
      }
      
    }
    Serial.print(mqttMsg);
    Serial.println();
    Serial.println("-----------------------");
}