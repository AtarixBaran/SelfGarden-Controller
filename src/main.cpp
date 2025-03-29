#define BLYNK_TEMPLATE_ID "TMPL6aXGddioc"
#define BLYNK_TEMPLATE_NAME "ESP32"
#define BLYNK_AUTH_TOKEN "FdbGiw-HZfU-rSay6D15jnOnyCNUO-UU"

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
#define GMT_OFFSET_SEC 3600 * 9
#define DAYLIGHT_OFFSET_SEC 0

#include "secrets.h"
#include "tds_meter.h"

#define relayPumpPin 14
#define RAIN_SENSOR_PIN 34
#define WATER_PUMP_PIN 18

Adafruit_APDS9960 apds;
bool apdsOk = false;

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

void loopWifiKeepAlive(void *pvParameters);
void loopPump(void *pvParameters);
void loopRainSensor(void *pvParameters);
void loopTDSMeter(void *pvParameters);
void turnPumpOn();
void turnPumpOff();
void handleRoot();
void handleUpdate();  // Web OTA için fonksiyon prototipi
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

    Blynk.begin(BLYNK_AUTH_TOKEN, ssid, password);

    pinMode(RAIN_SENSOR_PIN, INPUT);
    Wire.begin(21, 22);

    if (apds.begin()) {
        Serial.println("APDS-9960 başlatıldı.");
        apds.enableColor(true);
        apdsOk = true;
    } else {
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



    // server.on("/webota", HTTP_GET, []() {
    //     server.send(200, "text/html",
    //                 "<form method='POST' action='/update' enctype='multipart/form-data'>"
    //                 "<input type='file' name='update'><input type='submit' value='Güncelle'></form>");
    //   });
      
    //   server.on("/update", HTTP_POST, []() {}, handleUpdate);


    configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, "pool.ntp.org", "time.nist.gov", "time.google.com");
    esp_task_wdt_init(3, false);

    xTaskCreatePinnedToCore(loopWifiKeepAlive, "loopWifiKeepAlive", 4096, NULL, 3, NULL, ARDUINO_RUNNING_CORE);
    xTaskCreatePinnedToCore(loopPump, "loopPump", 4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(loopTDSMeter, "loopTDSMeter", 4096, NULL, 1, NULL, ARDUINO_RUNNING_CORE);
    xTaskCreatePinnedToCore(loopRainSensor, "loopRainSensor", 2048, NULL, 1, NULL, ARDUINO_RUNNING_CORE);
}

BLYNK_WRITE(V18) {
  int value = param.asInt();
  if (value == 1) {
    turnPumpOn();
  } else {
    turnPumpOff();
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
        else{
            Blynk.virtualWrite(V0, "WiFi: AT  ✅");
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
        while (turnOn == true) {
            Serial.println("Waiting for pump to finish before TDS testing...");
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }

        Serial.println("TDS ölçümü yapılıyor...");
        meter.readTDSValue();

        char tdsValueStr[8] = { 0 };
        char temperatureStr[8] = { 0 };

        snprintf(tdsValueStr, sizeof(tdsValueStr), "%f", meter.getTDSValue());
        snprintf(temperatureStr, sizeof(temperatureStr), "%f", meter.getTemperature());
        int randomValue = random(0, 1000);
        Serial.print("TDS: "); Serial.println(randomValue);
        Serial.print("Sıcaklık: "); Serial.println(temperatureStr);
        Blynk.virtualWrite(V1, randomValue);

        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

void loopRainSensor(void *pvParameters)
{
    while (true)
    {
        if (apdsOk && apds.colorDataReady()) {
            uint16_t r, g, b, c;
            apds.getColorData(&r, &g, &b, &c);
            Serial.printf("R: %u G: %u B: %u C: %u\n", r, g, b, c);
        } 
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void loop()
{
    Blynk.run();
    ArduinoOTA.handle();
    server.handleClient();
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


void handleUpdate() {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      Serial.printf("OTA Başladı: %s\n", upload.filename.c_str());
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) {
        Serial.printf("OTA Başarılı! Toplam Boyut: %u bytes\n", upload.totalSize);
        server.sendHeader("Connection", "close");
        server.send(200, "text/plain", "Güncelleme başarılı. Cihaz yeniden başlatılıyor...");
        delay(2000);
        ESP.restart();
      } else {
        Update.printError(Serial);
        server.send(500, "text/plain", "Güncelleme Hatası");
      }
    }
  }
