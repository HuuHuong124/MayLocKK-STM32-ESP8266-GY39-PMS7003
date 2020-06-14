#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <NTPtimeESP.h>
#include <SoftwareSerial.h>

#define DEBUG Serial1
#define PACKET_SIZE 15

#define PIN_BUTTON 0
uint32_t lastPressButton = 0;
bool longPressButton();

bool isGetTime = true;
bool isPublish = false;

uint8_t dataBuffer[PACKET_SIZE] = {0};
uint8_t dataByteCount = 0;

const char mqttServerAddress[15] = "23.89.159.119";
//const char mqttServerAddress[15] = "188.68.48.86";
const uint16_t mqttServerPort = 1883;

char topic[25];
char espID[10];
uint32_t lastGetData = 0;
uint32_t lastGetTime = 0;
uint32_t lastMqttReconnect = 0;
uint32_t lastsenddatafake = 0;

uint8_t MacAddress[6];
char nameDevice[12];

bool longPressButton()
{
  if (millis() - lastPressButton > 3000 && digitalRead(PIN_BUTTON) == 0)
  {
    return true;
  }
  else if (digitalRead(PIN_BUTTON) == 1)
  {
    lastPressButton = millis();
  }
  return false;
}

void initMqttClient(char* _topic, char* _espID, PubSubClient& _mqttClient)
{
  uint8_t espMacAddress[6];
  WiFi.macAddress(espMacAddress);
  uint32_t macAddressDecimal = (espMacAddress[3] << 16) + (espMacAddress[4] << 8) + espMacAddress[5];
  sprintf(_topic, "/Mayloc/ESP_%08d/", macAddressDecimal);
  sprintf(_espID, "%08d", macAddressDecimal);
  _mqttClient.setServer(mqttServerAddress, mqttServerPort);
  _mqttClient.connect(_espID);
  _mqttClient.subscribe("huuhuong");
  //Serial.println(_topic);
  //Serial.println(_espID);
}
void callback(char* topic, byte* payload, unsigned int length) {
  //Serial.print("Message arrived [");
  //Serial.print(topic);
  //Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  //Serial.println();

}
WiFiClient espClient;
PubSubClient mqttClient(espClient);
NTPtime NTPch("ch.pool.ntp.org");///???
strDateTime dateTime;
  
void setup() {
  Serial.begin(115200);
  DEBUG.begin(9600);
  WiFi.mode(WIFI_STA);
  pinMode(PIN_BUTTON, INPUT);
  initMqttClient(topic, espID, mqttClient);
  //Serial.println("setup done");
  mqttClient.setCallback(callback);

}

void loop() {
   if (longPressButton())
  {
    DEBUG.println(" - long press!");
    if (WiFi.beginSmartConfig())
    {
     DEBUG.println(" - long press!");
     //Serial.println(" - start config wifi");
    }
  }
  if (WiFi.status() == WL_CONNECTED){
     WiFi.macAddress(MacAddress);
     sprintf(nameDevice,"%02X%02X%02X%02X%02X%02X",MacAddress[0],MacAddress[1],MacAddress[2],MacAddress[3],MacAddress[4],MacAddress[5]);
   }
  if (Serial.available() > 0)
  {
    dataBuffer[dataByteCount] = Serial.read();
    if (dataBuffer[0] == 11){
      dataByteCount++;
    }
    if (dataByteCount == PACKET_SIZE)
    {
      dataByteCount = 0;
      if (dataBuffer[1] == 22)
      {
        uint16_t check = 0;
        for (uint8_t i = 0; i < PACKET_SIZE - 2; i++)
        {
          check += dataBuffer[i];
        }
        uint16_t check2 = dataBuffer[PACKET_SIZE - 2] * 256 + dataBuffer[PACKET_SIZE - 1];
        if (check == check2)
        {
          lastGetData = millis();
          isPublish = true;
        }
      }
    }
  }

  else if (WiFi.status() == WL_CONNECTED)
  {
    if (isGetTime)
    {
      dateTime = NTPch.getNTPtime(7.0, 0);
      if (dateTime.valid)
      {
        lastGetTime = millis();
        isGetTime = false;
      }
    }
     else if (isPublish)
      {
       if (mqttClient.connected())
       {
        uint32_t epochTime = 0;
        if (lastGetData > lastGetTime)
          epochTime = dateTime.epochTime + ((lastGetData - lastGetTime) / 1000);
        else
          epochTime = dateTime.epochTime - ((lastGetTime - lastGetData) / 1000);
        //Serial.print(" - publish:.. ");
        char mes[256] = {0};
        sprintf(mes, "{\"DATA\":{\"CO\":\%d\,\"Hum\":\%d\,\"Pm1\":\%d\,\"Pm10\":\%d\,\"Pm2p5\":\%d\,\"Time\":\%d\,\"Tem\":\%d\},\"NodeID\":\"%s\"}",0,dataBuffer[3],dataBuffer[12],((dataBuffer[10] << 8) + dataBuffer[11]),((dataBuffer[8] << 8) + dataBuffer[9]),epochTime,dataBuffer[2],nameDevice);
        //sprintf(mes, "{\"data\":{\"tem\":\"%d\",\"humi\":\"%d\",\"per\":\"%d\",\"pm1\":\"%d\",\"pm2.5\":\"%d\",\"pm10\":\"%d\",\"FanState\":\"%d\",\"time\":\"%d\"}}", dataBuffer[2], dataBuffer[3],((dataBuffer[4] << 8) + dataBuffer[5]),((dataBuffer[6] << 8) + dataBuffer[7]),((dataBuffer[8] << 8) + dataBuffer[9]),((dataBuffer[10] << 8) + dataBuffer[11]),dataBuffer[12],epochTime);
        if (mqttClient.publish(topic, mes, true))
        {
          //Serial.println(mes); 
          DEBUG.println(mes);
          isPublish = false;
        }
        mqttClient.loop();
      }
      else if (millis() - lastMqttReconnect > 1000)
      {
        lastMqttReconnect = millis();
        //Serial.println(" - mqtt reconnect ");
        mqttClient.connect(espID);
        mqttClient.subscribe("huuhuong");
      }
  }
}
  mqttClient.loop();
}
