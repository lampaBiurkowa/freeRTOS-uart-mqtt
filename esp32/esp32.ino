#include <HardwareSerial.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <string.h>

//force 1 core from shawn hymel
#if CONFIG_FREERTOS_UNICORE
static cosnt BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif

WiFiClient espClient;
PubSubClient client(espClient);
HardwareSerial SerialUart(1);

QueueHandle_t receivedFromStmQueue;

const uint16_t MAX_ADC_INPUT = 4095;
const char *ssid = "name";
const char *password = "pass :D/";
const char *mqtt_broker = "test.mosquitto.org";
const char *greenTopic = "test-topic-student-pak-green";
const char *yellowTopic = "test-topic-student-pak-yellow";
const char *blueTopic = "test-topic-student-pak-blue";
const char *clientId = "3242432rffsg4weyeF";
const int mqtt_port = 1883;
const int YELLOW_PIN = 19;
const int GREEN_PIN = 18;

void connectToWifi()
{
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
  }
}

void connectMqtt()
{
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(onReceive);
  WiFi.mode(WIFI_STA);
  while (!client.connected())
  {
    if (!client.connect(clientId))
    {
      delay(1000);
    }
  }
  client.subscribe(yellowTopic);
  client.subscribe(greenTopic);
}

void initializePins()
{
  pinMode(YELLOW_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
}

void handleUart(void *arg)
{
  while (1)
  {
    if (SerialUart.available())
    {
      uint8_t byte1 = SerialUart.read();
      uint8_t byte2 = SerialUart.read();
      uint16_t value = (byte2 << 8) | byte1;
      if (value > MAX_ADC_INPUT || value < 0)
        value = (byte1 << 8) | byte2; //workaround :D
      uint8_t messagesToReceive = uxQueueMessagesWaiting(receivedFromStmQueue);
      if (uxQueueSpacesAvailable(receivedFromStmQueue) > 0)
        xQueueSend(receivedFromStmQueue, &value, 1000 / portTICK_PERIOD_MS);
    }
    vTaskDelay(100);
  }
}

void handleMqtt(void *arg)
{
  uint16_t buffer[16] = {};
  char jsonBuffer[128];
  StaticJsonDocument<128> doc;
  while (1)
  {
    uint8_t messagesToReceive = uxQueueMessagesWaiting(receivedFromStmQueue);
    for (int i = 0; i < messagesToReceive; i++)
      xQueueReceive(receivedFromStmQueue, buffer + i, 1000 / portTICK_PERIOD_MS);

    for (uint8_t i = 0; i < messagesToReceive; i++)
    {
      doc["value"] = buffer[i];
      serializeJson(doc, jsonBuffer);
      client.publish(blueTopic, jsonBuffer);
    }

    client.loop();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void setup()
{
  SerialUart.begin(115200, SERIAL_8N1, 3, 1);
  connectToWifi();
  connectMqtt();
  initializePins();
  receivedFromStmQueue = xQueueCreate(16, sizeof(uint16_t));

  xTaskCreatePinnedToCore(handleUart, "handleUart", 768, NULL, 1, NULL, app_cpu);
  xTaskCreatePinnedToCore(handleMqtt, "handleMqtt", 2048, NULL, 1, NULL, app_cpu);
}

void loop()
{

}

void handleBlinkForColor(int pin, byte *payload, unsigned int length)
{
  StaticJsonDocument<256> doc;
  deserializeJson(doc, payload, length);
  uint8_t value = doc["Value"].as<uint8_t>();
  if (value >= 100)
  {
    digitalWrite(pin, HIGH);
    delay(500);
    digitalWrite(pin, LOW);
    char id = pin == YELLOW_PIN ? 'y' : 'g';
    uint8_t dataToSend[2] = {id, value};
    SerialUart.write(dataToSend, 2);
  }
}

void onReceive(char *topic, byte *payload, unsigned int length)
{
  if (strcmp(topic, yellowTopic) != 0 && strcmp(topic, greenTopic) != 0)
    return;

  int pin = strcmp(topic, yellowTopic) != 0 ? GREEN_PIN : YELLOW_PIN;
  handleBlinkForColor(pin, payload, length);
}