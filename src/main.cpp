#include <Arduino.h>
#include <ArduinoOTA.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <WebServer.h>

#include <esp_int_wdt.h>
#include <esp_task_wdt.h>

#include <time.h>
#define GMT_OFFSET_SEC 3600 * 9
#define DAYLIGHT_OFFSET_SEC 0
#define NTP_SERVER "jp.pool.ntp.org"

#include "secrets.h"
#include "tds_meter.h"

#define relayPumpPin 14
#define RAIN_SENSOR_PIN 34
#define WATER_PUMP_PIN 18  // 💧 Water Pump için GPIO 18 tanımı

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
void turnPumpOn();
void turnPumpOff();
void handleRoot();

void OTASetup()
{
    ArduinoOTA.setHostname(getDynamicHostname().c_str());
    Serial.println("Dynamic Hostname: " + getDynamicHostname());
    Serial.println("OTA URL: http://" + getDynamicHostname() + ".local");

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
    pinMode(WATER_PUMP_PIN, OUTPUT);        // 💧 Water pump pin modu
    digitalWrite(WATER_PUMP_PIN, LOW);      // Başlangıçta kapalı olsun
    turnOn = true;
    waiting = 0;
}

void setup()
{
    Serial.begin(115200);
    Serial.println("Merhaba! ESP32 çalışıyor.");

    Serial.printf("Connecting to %s ", ssid);
    setCpuFrequencyMhz(80);

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        vTaskDelay(500 / portTICK_PERIOD_MS);
        Serial.print(".");
    }
    Serial.print("connected");

    pinMode(RAIN_SENSOR_PIN, INPUT);

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
    Serial.println("ESP32 Web Server Aktif: http://localhost:8180");

    OTASetup();
    setupPump();

    configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, "pool.ntp.org", "time.nist.gov", "time.google.com");
    esp_task_wdt_init(3, false);

    xTaskCreatePinnedToCore(loopWifiKeepAlive, "loopWifiKeepAlive", 4096, NULL, 3, NULL, ARDUINO_RUNNING_CORE);
    xTaskCreatePinnedToCore(loopPump, "loopPump", 4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(loopRainSensor, "loopRainSensor", 2048, NULL, 1, NULL, ARDUINO_RUNNING_CORE);
}

void turnPumpOff()
{
    digitalWrite(WATER_PUMP_PIN, LOW);

    Serial.println("Pump is turned off");
    turnOn = false;
    waiting = 0;
}

void turnPumpOn()
{
    digitalWrite(WATER_PUMP_PIN, HIGH);
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
        if (WiFi.status() == WL_CONNECTED)
        {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }

        Serial.println("[WIFI] Connecting");
        WiFi.begin(ssid, password);
        unsigned long startAttemptTime = millis();

        while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < WIFI_TIMEOUT_MS)
        {
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }

        if (WiFi.status() != WL_CONNECTED)
        {
            Serial.println("[WIFI] FAILED");
            vTaskDelay(WIFI_RECOVER_TIME_MS / portTICK_PERIOD_MS);
            continue;
        }

        Serial.println("[WIFI] Connected: " + WiFi.localIP());
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

void loopRainSensor(void *pvParameters)
{
    while (true)
    {
        int rainValue = analogRead(RAIN_SENSOR_PIN);
        Serial.print("Rain Sensor Value: ");
        Serial.println(rainValue);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void loop()
{
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