#include <Arduino.h>
#include <sewparser.h>
#include "sew-wifi-config.h"
#include "sew-web-server.hpp"
#include "sew-mqtt.hpp"

SewWebServer sewServer;
SewMQTT sewMQTT;


uint8_t MML[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01};
uint8_t MMR[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02};
uint8_t MDF[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03};
uint8_t MDB[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04};

uint8_t* macMotorLeft = WiFi.macAddress(MML);
uint8_t* macMotorRight = WiFi.macAddress(MMR);
uint8_t* macDistanceFront = WiFi.macAddress(MDF);
uint8_t* macDistanceBack = WiFi.macAddress(MDB);

// Subscribers
String MAC = WiFi.macAddress();
String subscriptionMotorLeft = MAC + "/motor/" + MAC + ":00:01/action";
String subscriptionMotorRight = MAC + "/motor/" + MAC + ":00:02/action";
String subscriptionDistanceFront = MAC + "/distance/" + MAC + ":00:03/action";
String subscriptionDistanceBack = MAC + "/distance/" + MAC + ":00:04/action";
String subscriptions[] = {subscriptionMotorLeft, subscriptionMotorRight, subscriptionDistanceFront, subscriptionDistanceBack};

// Publishers
String publishMotorLeft = MAC + "/motor/" + MAC + ":00:01/status";
String publishMotorRight = MAC + "/motor/" + MAC + ":00:02/status";
String publishDistanceFront = MAC + "/distance/" + MAC + ":00:03/status";
String publishDistanceBack = MAC + "/distance/" + MAC + ":00:04/status";
String publications[] = {publishMotorLeft, publishMotorRight, publishDistanceFront, publishDistanceBack};

void mqttMessages(char* topic, uint8_t* payload, uint8_t length)
{
    Serial.println("MQTT Message from " + String(topic));
}

void onMQTTConnect() {
    Serial.println("Connected MQTT");
    FRAME frame;
    SewParser::encodeDCMotor(frame, macMotorLeft, 1, 0, 200);
    sewMQTT.publish(0, frame.frame, frame.size);
}

void onMotorRequest(WebServer& ws) {
    ws.send(200, "application/json", "{\"motor1\": 0}");
}

void syncMega() {
    uint8_t saySew[0];
    bool sewReceived = false;
    while(!sewReceived) {
        if(Serial1.available()) {
            size_t sewSize = Serial1.readBytes(saySew, 3);
            if(sewSize == 3) {
                sewReceived = true;
                uint8_t mac[6];
                WiFi.macAddress(mac);
                Serial1.write(mac, 6);
            }
        }
        else {
            Serial.println("Waiting for MEGA ACK!");
            delay(500);
        }
    }
}

void setup()
{
    // Debug
    Serial.begin(115200);
    // Mega
    Serial1.begin(115200);
    // Try to connect to last WiFi
    initWifi();
    // Init WebServer + mqtt client
    sewServer.addRequest("/dcmotor", onMotorRequest);
    sewServer.initServer(&sewMQTT);
    sewMQTT.setSubscribers(subscriptions, 4);
    sewMQTT.setPublishers(publications, 4);
    sewMQTT.initMQTT(mqttMessages, onMQTTConnect);
    sewMQTT.reconnect();

    // Sync Mega
    syncMega();
}

void loop()
{
    sewServer.handleSewWebServer();
    sewMQTT.handleClient();
    while(Serial1.available()) {
        Serial.print(Serial1.read());
    }
}