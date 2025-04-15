#define BLYNK_TEMPLATE_ID "TMPL6tNClfNf5"
#define BLYNK_TEMPLATE_NAME "ESP32"
#define BLYNK_AUTH_TOKEN "indZF7SeCZJD-8VBvIwRySAfwdmVC3mD"

#include <Arduino.h>
#include <ArduinoOTA.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <WebServer.h>
#include <esp_int_wdt.h>
#include <esp_task_wdt.h>
#include <Wire.h>
#include <Adafruit_APDS9960.h>
#include <BlynkSimpleEsp32.h>
#include <time.h>
#include <ModbusMaster.h>
#include <WiFiManager.h>  


#define GMT_OFFSET_SEC 3600 * 9
#define DAYLIGHT_OFFSET_SEC 0

#include "secrets.h"
#include "tds_meter.h"

#define relayPumpPin 14
#define RAIN_SENSOR_PIN 34
#define WATER_PUMP_PIN 18
#define PH_SENSOR_PIN 33
#define EC_SENSOR_PIN 32

Adafruit_APDS9960 apds;
bool apdsOk = false;

#define MAX485_DE_RE 4 
ModbusMaster node;

ModbusMaster sensor1;
ModbusMaster sensor2;


String rainStatus = "Bilinmiyor";
const char *ssid = WIFI_SSID;
const char *password = WIFI_PASSWORD;

#define WIFI_TIMEOUT_MS 20000
#define WIFI_RECOVER_TIME_MS 5000

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

String getDynamicHostname()
{
  uint8_t mac[6];
  WiFi.macAddress(mac);
  char hostname[32];
  snprintf(hostname, sizeof(hostname), "hydroponic-%02X%02X", mac[4], mac[5]);
  return String(hostname);
}

#define PUMP_ON_DURATION_MINUTES 5
#define PUMP_OFF_DURATION_MINUTES 15

WebServer server(80);
int waiting = 0;
bool turnOn = true;


void preTransmission() {
  digitalWrite(MAX485_DE_RE, 1);
}

void postTransmission() {
  digitalWrite(MAX485_DE_RE, 0);
}

void loopWifiKeepAlive(void *pvParameters);
void loopPump(void *pvParameters);
void loopRainSensor(void *pvParameters);
void loopTDSMeter(void *pvParameters);
void loopPHSensor(void *pvParameters);
void NPKSensor(void *pvParameters);
void turnPumpOn();
void turnPumpOff();
void handleRoot();
void handleUpdate(); // Web OTA için fonksiyon prototipi
void OTASetup()
{
  ArduinoOTA.setHostname(getDynamicHostname().c_str());

  ArduinoOTA
      .onStart([]()
               {
                     String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
                     Serial.println("Start updating " + type); })
      .onEnd([]()
             { Serial.println("\nEnd"); ESP.restart(); })
      .onProgress([](unsigned int progress, unsigned int total)
                  { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); })
      .onError([](ota_error_t error)
               {
                     Serial.printf("Error[%u]: ", error);
                     if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
                     else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
                     else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
                     else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
                     else if (error == OTA_END_ERROR) Serial.println("End Failed"); });

  ArduinoOTA.begin();
}

void loopcheckEC(void *pvParameters)
{

  while (true)
  {
    int analogValue = analogRead(EC_SENSOR_PIN);
    float voltage = analogValue * (3.3 / 4095.0);
    float ecValue = voltage * 1000;

    Serial.print("Analog Değer: ");
    Serial.print(analogValue);
    Serial.print("  | Voltaj: ");
    Serial.print(voltage, 3);
    Serial.print(" V  | EC: ");
    Serial.print(ecValue, 2);
    Serial.println(" µS/cm");

    Blynk.virtualWrite(V3, ecValue);

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void setupPump()
{
  pinMode(relayPumpPin, OUTPUT);
  pinMode(WATER_PUMP_PIN, OUTPUT);
  digitalWrite(WATER_PUMP_PIN, LOW);
  turnOn = true;
  waiting = 0;
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Merhaba! ESP32 çalışıyor.");

  setCpuFrequencyMhz(80);

  pinMode(MAX485_DE_RE, OUTPUT);
  digitalWrite(MAX485_DE_RE, 0);

  Serial2.begin(9600, SERIAL_8N1, 16, 17);



  sensor1.begin(1, Serial2); // Sensor 1 - Slave ID 1
  sensor1.preTransmission(preTransmission);
  sensor1.postTransmission(postTransmission);

  sensor2.begin(2, Serial2); // Sensor 2 - Slave ID 2
  sensor2.preTransmission(preTransmission);
  sensor2.postTransmission(postTransmission);



  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, password);

  pinMode(RAIN_SENSOR_PIN, INPUT);
  Wire.begin(21, 22);

  if (apds.begin())
  {
    Serial.println("APDS-9960 başlatıldı.");
    apds.enableColor(true);
    apdsOk = true;
  }
  else
  {
    Serial.println("APDS-9960 başlatılamadı!");
  }

  server.on("/", handleRoot);
  server.on("/pump_on", []()
            {
                  turnPumpOn();
                  server.sendHeader("Location", "/", true);
                  server.send(302, "text/plain", ""); });
  server.on("/pump_off", []()
            {
                  turnPumpOff();
                  server.sendHeader("Location", "/", true);
                  server.send(302, "text/plain", ""); });

  server.begin();
  OTASetup();
  setupPump();
  Serial.println(WiFi.localIP());

  server.on("/webota", HTTP_GET, []()
            { server.send(200, "text/html",
                          "<form method='POST' action='/update' enctype='multipart/form-data'>"
                          "<input type='file' name='update'><input type='submit' value='Güncelle'></form>"); });

  server.on("/update", HTTP_POST, []() {}, handleUpdate);

  configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, "pool.ntp.org", "time.nist.gov", "time.google.com");
  esp_task_wdt_init(3, false);

  xTaskCreatePinnedToCore(loopWifiKeepAlive, "loopWifiKeepAlive", 4096, NULL, 3, NULL, ARDUINO_RUNNING_CORE);
  // xTaskCreatePinnedToCore(loopPump, "loopPump", 4096, NULL, 2, NULL, 0);
  // xTaskCreatePinnedToCore(loopTDSMeter, "loopTDSMeter", 4096, NULL, 1, NULL, ARDUINO_RUNNING_CORE);
  //  xTaskCreatePinnedToCore(loopPHSensor, "loopPHSensor", 4096, NULL, 1, NULL, ARDUINO_RUNNING_CORE);
   xTaskCreatePinnedToCore(NPKSensor, "NPKSensor", 4096, NULL, 1, NULL, ARDUINO_RUNNING_CORE);
  // xTaskCreatePinnedToCore(loopcheckEC, "loopcheckEC", 4096, NULL, 1, NULL, ARDUINO_RUNNING_CORE);
}


void NPKSensor(void *pvParameters)
{

  while (true)
  {
    uint8_t result1 = sensor1.readInputRegisters(0x00, 7);
    if (result1 == sensor1.ku8MBSuccess) {
      Serial.print("[Sensor 1] Nem: ");
      Serial.println(sensor1.getResponseBuffer(0));
    } else {
      Serial.print("[Sensor 1] Hata: ");
      Serial.println(result1);
    }
  
    delay(500);
  
    uint8_t result2 = sensor2.readInputRegisters(0x00, 7);
    if (result2 == sensor2.ku8MBSuccess) {
      Serial.print("[Sensor 2] Nem: ");
      Serial.println(sensor2.getResponseBuffer(0));
    } else {
      Serial.print("[Sensor 2] Hata: ");
      Serial.println(result2);
    }
  

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
  
  
}

BLYNK_WRITE(V18)
{
  int value = param.asInt();
  if (value == 1)
  {
    turnPumpOn();
  }
  else
  {
    turnPumpOff();
  }
}

void loopPHSensor(void *pvParameters)
{
  while (true)
  {
    int analogValue = analogRead(PH_SENSOR_PIN);

    // Gerilim hesapla (ESP32 için 3.3V referans)
    float voltage = (analogValue / 4095.0) * 3.3;

    float pH = 1.9118 * voltage * voltage - 17.3009 * voltage + 36.4217;

    Serial.print("Analog: ");
    Serial.print(analogValue);
    Serial.print(" | Voltaj: ");
    Serial.print(voltage, 3);
    Serial.print(" V | pH: ");
    Serial.println(pH, 2);

    Blynk.virtualWrite(V3, pH);

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void turnPumpOff()
{
  digitalWrite(WATER_PUMP_PIN, HIGH);
  Serial.println("Pump is turned off");
  turnOn = false;
  waiting = 0;
}

void turnPumpOn()
{
  digitalWrite(WATER_PUMP_PIN, LOW);
  Serial.println("Pump is turned on");
  turnOn = true;
  waiting = 0;
}

void loopWifiKeepAlive(void *pvParameters)
{
  esp_task_wdt_add(NULL);
  while (true)
  {
    esp_task_wdt_reset();
    if (WiFi.status() != WL_CONNECTED)
    {

      Blynk.virtualWrite(V0, "WiFi: BAĞLI DEĞİL ❌");

      Serial.println("[WIFI] Reconnecting...");
      WiFi.begin(ssid, password);
      unsigned long startAttemptTime = millis();
      while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < WIFI_TIMEOUT_MS)
      {
        vTaskDelay(500 / portTICK_PERIOD_MS);
      }
    }
    else
    {
      Blynk.virtualWrite(V0, WiFi.localIP().toString());
      Serial.println("[WIFI] Connected!");
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void loopPump(void *pvParameters)
{
  struct tm timeinfo;
  turnOn = true;
  waiting = 0;

  while (true)
  {
    int secToWait = 60 * (turnOn ? PUMP_ON_DURATION_MINUTES : PUMP_OFF_DURATION_MINUTES);
    if (getLocalTime(&timeinfo))
    {
      if (!turnOn && timeinfo.tm_hour >= 6 && timeinfo.tm_hour < 20)
      {
        secToWait = 60 * (PUMP_OFF_DURATION_MINUTES / 2);
      }
    }

    digitalWrite(relayPumpPin, turnOn ? HIGH : LOW);
    if (waiting >= secToWait)
    {
      waiting = 0;
      turnOn = !turnOn;
      turnOn ? turnPumpOn() : turnPumpOff();
    }

    waiting++;
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void loopTDSMeter(void *pvParameters)
{
  TDSMeter meter;
  while (true)
  {
    while (turnOn == true)
    {
      Serial.println("Waiting for pump to finish before TDS testing...");
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    Serial.println("TDS ölçümü yapılıyor...");
    meter.readTDSValue();

    char tdsValueStr[8] = {0};
    char temperatureStr[8] = {0};

    snprintf(tdsValueStr, sizeof(tdsValueStr), "%f", meter.getTDSValue());
    snprintf(temperatureStr, sizeof(temperatureStr), "%f", meter.getTemperature());
    int randomValue = random(0, 1000);
    Serial.print("TDS: ");
    Serial.println(randomValue);
    Serial.print("Sıcaklık: ");
    Serial.println(temperatureStr);
    Blynk.virtualWrite(V1, randomValue);

    vTaskDelay(10000 / portTICK_PERIOD_MS);
  }
}

void loopRainSensor(void *pvParameters)
{
  while (true)
  {
    if (apdsOk && apds.colorDataReady())
    {
      uint16_t r, g, b, c;
      apds.getColorData(&r, &g, &b, &c);
      Serial.printf("R: %u G: %u B: %u C: %u\n", r, g, b, c);
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}
// void checkPH(){
//   int analogValue = analogRead(PH_SENSOR_PIN);
//   Serial.println(analogValue);
//   // float voltage = analogValue * (3.3 / 4095.0);
//   // float phValue = 7 + ((voltage - 1.65) * (14.0 / 3.3));  // örnek kalibrasyon

//   // Serial.print("Analog Değer: ");
//   // Serial.print(analogValue);
//   // Serial.print("  | Voltaj: ");
//   // Serial.print(voltage, 3);
//   // Serial.print(" V  | pH: ");
//   // Serial.println(phValue, 2);

// }

void loop()
{
  Blynk.run();
  ArduinoOTA.handle();
  server.handleClient();


  for (int id = 1; id <= 10; id++) {
    node.begin(id, Serial2);
    node.preTransmission(preTransmission);
    node.postTransmission(postTransmission);

    uint8_t result = node.readInputRegisters(0x10, 1);
    if (result == node.ku8MBSuccess) {
      Serial.print("✅ Cevap verdi: Slave ID = ");
      Serial.println(id);
    } else {
      Serial.print("❌ Cevap yok: Slave ID = ");
      Serial.println(id);
    }

    delay(500);
  }

}

void handleRoot()
{
  String html = R"rawliteral(
      <!DOCTYPE html>
      <html>
        <head>
          <title>ESP32 Control Panel</title>
          <meta charset="UTF-8">
          <style>
            body { font-family: Arial; text-align: center; padding-top: 50px; }
            h1 { color: #333; }
            p.status { font-size: 20px; color: #007BFF; margin-bottom: 30px; }
            button {
              padding: 15px 30px;
              font-size: 18px;
              margin: 10px;
              border: none;
              border-radius: 8px;
              background-color: #28a745;
              color: white;
              cursor: pointer;
            }
            button.stop {
              background-color: #dc3545;
            }
          </style>
        </head>
        <body>
          <h1>ESP32 Control Panel</h1>
          <p class="status">Yağmur Sensörü: )rawliteral";

  html += rainStatus;
  html += R"rawliteral(</p>
          <p><a href="/pump_on"><button>Turn ON Pump</button></a></p>
          <p><a href="/pump_off"><button class="stop">Turn OFF Pump</button></a></p>
        </body>
      </html>
    )rawliteral";

  server.send(200, "text/html", html);
}

void handleUpdate()
{
  HTTPUpload &upload = server.upload();
  if (upload.status == UPLOAD_FILE_START)
  {
    Serial.printf("OTA Başladı: %s\n", upload.filename.c_str());
    if (!Update.begin(UPDATE_SIZE_UNKNOWN))
    {
      Update.printError(Serial);
    }
  }
  else if (upload.status == UPLOAD_FILE_WRITE)
  {
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize)
    {
      Update.printError(Serial);
    }
  }
  else if (upload.status == UPLOAD_FILE_END)
  {
    if (Update.end(true))
    {
      Serial.printf("OTA Başarılı! Toplam Boyut: %u bytes\n", upload.totalSize);
      server.sendHeader("Connection", "close");
      server.send(200, "text/plain", "Güncelleme başarılı. Cihaz yeniden başlatılıyor...");
      delay(2000);
      ESP.restart();
    }
    else
    {
      Update.printError(Serial);
      server.send(500, "text/plain", "Güncelleme Hatası");
    }
  }
}
