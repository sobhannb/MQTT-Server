/*
  SimpleMQTTClient.ino
  The purpose of this exemple is to illustrate a simple handling of MQTT and Wifi connection.
  Once it connects successfully to a Wifi network and a MQTT broker, it subscribe to a topic and send a message to it.
  It will also send a message delayed 5 seconds later.
*/
#include <SPI.h>
#include <Ethernet.h>
#include "EspMQTTClient.h"

#define ETHERNET_MAC            "BA:E5:E3:B1:44:DD" // Ethernet MAC address (have to be unique between devices in the same network)
#define ETHERNET_IP             "192.168.1.112"       // IP address of RoomHub when on Ethernet connection

#define ETHERNET_RESET_PIN      17      // ESP32 pin where reset pin from W5500 is connected
#define ETHERNET_CS_PIN         5       // ESP32 pin where CS pin from W5500 is connected

#define MQTT_HOSTNAME "192.168.1.100"
#define MQTT_PORT 1883

String massage;
int relay1 = 25, relay2 = 26, relay3 = 27, relay4 = 14;
int button1 = 15, button2 = 0, button3 = 35, button4 = 13;
int switch1 = 22, switch2 = 21, switch3 = 32, switch4 = 16;
int con_state = 33, Mode = 33;
int i1, i2, i3, i4;

EspMQTTClient client(
  "WifiSSID",
  "WifiPassword",
  "192.168.1.100",  // MQTT Broker server ip
  "user",           // MQTTUsername ,Can be omitted if not needed
  "psw",            // MQTTPassword ,Can be omitted if not needed
  "TestClient",     // Client name that uniquely identify your device
  1883              // The MQTT port, default to 1883. this line can be omitted
);

IPAddress ipAddress;
PubSubClient mqttClient;
EthernetClient ethClient;

uint32_t lastMqttPublishTime = 0;

void macCharArrayToBytes(const char* str, byte* bytes) {
    for (int i = 0; i < 6; i++) {
        bytes[i] = strtoul(str, NULL, 16);
        str = strchr(str, ':');
        if (str == NULL || *str == '\0') {
            break;
        }
        str++;
    }
}

void ethernetWizReset(const uint8_t resetPin) {
    pinMode(resetPin, OUTPUT);
    digitalWrite(resetPin, HIGH);
    delay(250);
    digitalWrite(resetPin, LOW);
    delay(50);
    digitalWrite(resetPin, HIGH);
    delay(350);
}

void connectEthernet() {
    delay(500);
    byte* mac = new byte[6];
    macCharArrayToBytes(ETHERNET_MAC, mac);
    ipAddress.fromString(ETHERNET_IP);

    Ethernet.init(ETHERNET_CS_PIN);
    ethernetWizReset(ETHERNET_RESET_PIN);

    Serial.println("Starting ETHERNET connection...");
    Ethernet.begin(mac, ipAddress);
    delay(200);

    Serial.print("Ethernet IP is: ");
    Serial.println(Ethernet.localIP());
}

void setupMqtt(const char* hostname, uint16_t port, Client& _connectionClient) {
    mqttClient.setClient(_connectionClient);
    mqttClient.setServer(hostname, port);
}

void connectToMqtt(const char* deviceName, const char* willTopic, uint8_t willQoS, bool willRetain, const char* willMessage) {
    Serial.println("Connecting to MQTT broker...");
    while(!mqttClient.connect(deviceName, willTopic, willQoS, willRetain, willMessage)) {
        Serial.print("Connecting to MQTT as ");
        Serial.println(deviceName);
        delay(1000);
    }
    Serial.print("Connected to MQTT as ");
    Serial.print(deviceName);
    Serial.print(". LWT: ");
    Serial.print(willTopic);
    Serial.print(" -> ");
    Serial.println(willMessage);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  massage = "";
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i=0;i<length;i++) {
    Serial.print((char)payload[i]);
    massage += String(payload[i]);
  }
  Serial.println();
  if(topic == "XXXX/relay/0"){
    if(massage == "on"){
      digitalWrite(relay1, LOW);  
    }else if(massage == "off"){
      digitalWrite(relay1, HIGH);
    }
  }else if(topic == "XXXX/relay/1"){
    if(massage == "on"){
      digitalWrite(relay2, LOW);  
    }else if(massage == "off"){
      digitalWrite(relay2, HIGH);
    }
  }else if(topic == "XXXX/relay/2"){
    if(massage == "on"){
      digitalWrite(relay3, LOW);  
    }else if(massage == "off"){
      digitalWrite(relay3, HIGH);
    }
  }else if(topic == "XXXX/relay/3"){
    if(massage == "on"){
      digitalWrite(relay4, LOW);  
    }else if(massage == "off"){
      digitalWrite(relay4, HIGH);
    }
  }
}

void mqttReconnect() {
  connectToMqtt("home", "home/xxx", 1, 1, "lost");
  mqttClient.setCallback(mqttCallback);
  mqttClient.subscribe("home/xxx");
}

void setup()
{
  Serial.begin(115200);

  pinMode(con_state, INPUT_PULLUP);
  pinMode(relay1,OUTPUT);
  pinMode(relay2,OUTPUT);
  pinMode(relay3,OUTPUT);
  pinMode(relay4,OUTPUT);
  pinMode(button1,INPUT_PULLUP);
  pinMode(button2,INPUT_PULLUP);
  pinMode(button3,INPUT_PULLUP);
  pinMode(button4,INPUT_PULLUP);
  pinMode(switch1,INPUT_PULLUP);
  pinMode(switch2,INPUT_PULLUP);
  pinMode(switch3,INPUT_PULLUP);
  pinMode(switch4,INPUT_PULLUP);
  digitalWrite(relay1,HIGH);
  digitalWrite(relay2,HIGH);
  digitalWrite(relay3,HIGH);
  digitalWrite(relay4,HIGH);
  
  // Optional functionalities of EspMQTTClient
  client.enableDebuggingMessages(); // Enable debugging messages sent to serial output
  //client.enableHTTPWebUpdater(); // Enable the web updater. User and password default to values of MQTTUsername and MQTTPassword. These can be overridded with enableHTTPWebUpdater("user", "password").
  //client.enableOTA(); // Enable OTA (Over The Air) updates. Password defaults to MQTTPassword. Port is the default OTA port. Can be overridden with enableOTA("password", port).
  client.enableLastWillMessage("TestClient/lastwill", "I am going offline");  // You can activate the retain flag by setting the third parameter to true
  
  setupMqtt(MQTT_HOSTNAME, MQTT_PORT, ethClient);
  mqttReconnect();
  
}

// This function is called once everything is connected (Wifi and MQTT)
// WARNING : YOU MUST IMPLEMENT IT IF YOU USE EspMQTTClient
void onConnectionEstablished()
{
  // Subscribe to "XXXX/relay/0" and display received message to Serial
  client.subscribe("XXXX/relay/0", [](const String & payload) {
  Serial.println(payload);

  if(payload == "on"){
    digitalWrite(relay1, LOW);  
  }else if(payload == "off"){
    digitalWrite(relay1, HIGH);
  }
});

// Subscribe to "XXXX/relay/1" and display received message to Serial
  client.subscribe("XXXX/relay/1", [](const String & payload) {
  Serial.println(payload);

  if(payload == "on"){
    digitalWrite(relay2, LOW);  
  }else if(payload == "off"){
    digitalWrite(relay2, HIGH);
  }
});

// Subscribe to "XXXX/relay/2" and display received message to Serial
  client.subscribe("XXXX/relay/2", [](const String & payload) {
  Serial.println(payload);

  if(payload == "on"){
    digitalWrite(relay3, LOW);  
  }else if(payload == "off"){
    digitalWrite(relay3, HIGH);
  }
});

// Subscribe to "XXXX/relay/3" and display received message to Serial
  client.subscribe("XXXX/relay/3", [](const String & payload) {
  Serial.println(payload);

  if(payload == "on"){
    digitalWrite(relay4, LOW);  
  }else if(payload == "off"){
    digitalWrite(relay4, HIGH);
  }
});


  /*// Subscribe to "mytopic/wildcardtest/#" and display received message to Serial
  client.subscribe("mytopic/wildcardtest/#", [](const String & topic, const String & payload) {
    Serial.println("(From wildcard) topic: " + topic + ", payload: " + payload);
  });

  // Publish a message to "mytopic/test"
  client.publish("mytopic/test", "This is a message"); // You can activate the retain flag by setting the third parameter to true

  // Execute delayed instructions
  client.executeDelayed(5 * 1000, []() {
    client.publish("mytopic/wildcardtest/test123", "This is a message sent 5 seconds later");
  });*/
}

void loop()
{

  //if(digitalRead(Mode)){
    if(digitalRead(con_state)){
      client.loop();
    }else{
      if (!mqttClient.connected()) {
        Serial.println("ERROR: mqttClient has been disconnected. Reconnecting...");
        mqttReconnect();
      }
      mqttClient.loop();
    }
  //}else{
    if(!digitalRead(button1) || !digitalRead(switch1)){
      digitalWrite(relay1, LOW);
      if(!i1){
        if(digitalRead(con_state)){
          i1 = 1;
          // Publish a message to "mytopic/test"
          client.publish("XXXX/relay/0", "on"); // You can activate the retain flag by setting the third parameter to true
        }else{
          i1 = 1;
          mqttClient.publish("XXXX/relay/0", "off");
        }
      }
    }else{
      digitalWrite(relay1, HIGH);
      if(i1){
        if(digitalRead(con_state)){
          i1 = 0;
          // Publish a message to "mytopic/test"
          client.publish("XXXX/relay/0", "on"); // You can activate the retain flag by setting the third parameter to true
        }else{
          i1 = 0;
          mqttClient.publish("XXXX/relay/0", "off");
        }
      }
    }
    if(!digitalRead(button2) || !digitalRead(switch2)){
      digitalWrite(relay2, LOW);
      if(!i2){
        if(digitalRead(con_state)){
          i2 = 1;
          // Publish a message to "mytopic/test"
          client.publish("XXXX/relay/1", "on"); // You can activate the retain flag by setting the third parameter to true
        }else{
          i2 = 1;
          mqttClient.publish("XXXX/relay/1", "off");
        }
      }
    }else{
      digitalWrite(relay2, HIGH);
      if(i2){
        if(digitalRead(con_state)){
          i2 = 0;
          // Publish a message to "mytopic/test"
          client.publish("XXXX/relay/1", "on"); // You can activate the retain flag by setting the third parameter to true
        }else{
          i2 = 0;
          mqttClient.publish("XXXX/relay/1", "off");
        }
      }
    }
    if(!digitalRead(button3) || !digitalRead(switch3)){
      digitalWrite(relay3, LOW);
      if(!i3){
        if(digitalRead(con_state)){
          i3 = 1;
          // Publish a message to "mytopic/test"
          client.publish("XXXX/relay/2", "on"); // You can activate the retain flag by setting the third parameter to true
        }else{
          i3 = 1;
          mqttClient.publish("XXXX/relay/2", "off");
        }
      }
    }else{
      digitalWrite(relay3, HIGH);
      if(i3){
        if(digitalRead(con_state)){
          i3 = 0;
          // Publish a message to "mytopic/test"
          client.publish("XXXX/relay/2", "on"); // You can activate the retain flag by setting the third parameter to true
        }else{
          i3 = 0;
          mqttClient.publish("XXXX/relay/2", "off");
        }
      }
    }
    if(!digitalRead(button4) || !digitalRead(switch4)){
      digitalWrite(relay4, LOW);
      if(!i4){
        if(digitalRead(con_state)){
          i4 = 1;
          // Publish a message to "mytopic/test"
          client.publish("XXXX/relay/3", "on"); // You can activate the retain flag by setting the third parameter to true
        }else{
          i4 = 1;
          mqttClient.publish("XXXX/relay/3", "off");
        }
      }
    }else{
      digitalWrite(relay4, HIGH);
      if(i4){
        if(digitalRead(con_state)){
          i4 = 0;
          // Publish a message to "mytopic/test"
          client.publish("XXXX/relay/3", "on"); // You can activate the retain flag by setting the third parameter to true
        }else{
          i4 = 0;
          mqttClient.publish("XXXX/relay/3", "off");
        }
      }
    }
    
  //}
   
  delay(2000);
}
