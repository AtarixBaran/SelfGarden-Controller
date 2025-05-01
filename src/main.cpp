#define BLYNK_TEMPLATE_ID "TMPL6aXGddioc"
#define BLYNK_TEMPLATE_NAME "ESP32"
#define BLYNK_AUTH_TOKEN "4RJb2tQwzIuG-SHbfcnV5prlaZ2E12ze"

#include <Arduino.h>
#include <ArduinoOTA.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <WebServer.h>
#include <esp_int_wdt.h>
#include <esp_task_wdt.h>
#include <Wire.h>
#include <BlynkSimpleEsp32.h>
#include <time.h>
#include <ModbusMaster.h>
#include <WiFiManager.h>
#include "secrets.h"
#include "iot_parser.h"
#define LOG(msg) logToWeb(msg)

#define GMT_OFFSET_SEC 3600 * 9
#define DAYLIGHT_OFFSET_SEC 0
#define WATER_PUMP_PIN 14
#define LightPin 22
#define PH_SENSOR_PIN 33
#define EC_SENSOR_PIN 32

#define POMP_PH_UP_PIN 15
#define POMP_PH_DOWN_PIN 16
#define POMP_A_PIN 17
#define POMP_B_PIN 18

const char *ssid = WIFI_SSID;
const char *password = WIFI_PASSWORD;

#define WIFI_TIMEOUT_MS 20000
#define WIFI_RECOVER_TIME_MS 5000

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

String getDeviceID() {
  uint8_t mac[6];
  WiFi.macAddress(mac);
  char id[13];
  sprintf(id, "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(id);
}

#define PUMP_ON_DURATION_MINUTES 5
#define PUMP_OFF_DURATION_MINUTES 15

WebServer server(80);
int waiting = 0;
bool turnOn = true;

bool testMode = false;
float simulatedPH = 6.8;
float simulatedEC = 800.0;

void loopWifiKeepAlive(void *pvParameters);
void loopPump(void *pvParameters);
void loopPHSensor(void *pvParameters);
void loopcheckEC(void *pvParameters);
void loopPumpTest(void *pvParameters);
void loopAutoControl(void *pvParameters);
void turnPumpOff();
void handleRoot();
void handleUpdate(); // Web OTA için fonksiyon prototipi
void OTASetup()
{
  ArduinoOTA.setHostname(getDeviceID().c_str());

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

void setupInput()
{

  pinMode(PH_SENSOR_PIN, INPUT);
  pinMode(EC_SENSOR_PIN, INPUT);
}

void setOutput()
{
  pinMode(POMP_A_PIN, OUTPUT);
  pinMode(POMP_B_PIN, OUTPUT);
  pinMode(POMP_PH_UP_PIN, OUTPUT);
  pinMode(POMP_PH_DOWN_PIN, OUTPUT);
  pinMode(WATER_PUMP_PIN, OUTPUT);
  pinMode(LightPin, OUTPUT);

  digitalWrite(POMP_A_PIN, HIGH); // ters değer yolluyoruz
  digitalWrite(POMP_B_PIN, HIGH);
  digitalWrite(POMP_PH_UP_PIN, HIGH);
  digitalWrite(POMP_PH_DOWN_PIN, HIGH);
  digitalWrite(WATER_PUMP_PIN, HIGH);
  digitalWrite(LightPin, HIGH);
  Serial.println("Outputs initialized");
}

void connectToWiFi()
{

  WiFi.begin("Wokwi-GUEST", "", 6);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(100);
    Serial.print(".");
  }

  Serial.println(" Connected!");
}

String logs = "";
void logToWeb(String msg) {
  Serial.println(msg);
  logs += msg + "<br>";
  if (logs.length() > 5000) logs = "";  
}


void setup()
{

  Serial.begin(115200);

    WiFiManager wm; // AP oluştur
    wm.setTimeout(180);
    if (!wm.autoConnect("Hidroponik-Setup", "12345678"))
    {
      Serial.println("WiFi bağlantısı sağlanamadı. ESP yeniden başlatılıyor...");
      delay(3000);
      ESP.restart();
    }


   
  LOG("Version V0.3");
  Serial.println("WiFi bağlantısı başarılı! IP: " + WiFi.localIP().toString());

  // --- Blynk bağlantısı ---
  Blynk.config(BLYNK_AUTH_TOKEN);
  Blynk.connect();

  setCpuFrequencyMhz(80);

  Serial2.begin(9600, SERIAL_8N1, 16, 17);

  Wire.begin(21, 22);

  server.on("/", handleRoot);
  server.on("/pump_on", []()
            {
                  server.sendHeader("Location", "/", true);
                  server.send(302, "text/plain", ""); });
  server.on("/pump_off", []()
            {
                  turnPumpOff();
                  server.sendHeader("Location", "/", true);
                  server.send(302, "text/plain", ""); });

  server.begin();
  OTASetup();

  setupInput();
  setOutput();

  Serial.println(WiFi.localIP());

  server.on("/webota", HTTP_GET, []()
            { server.send(200, "text/html",
                          "<form method='POST' action='/update' enctype='multipart/form-data'>"
                          "<input type='file' name='update'><input type='submit' value='Güncelle'></form>"); });

  server.on("/update", HTTP_POST, []() {}, handleUpdate);



  server.on("/logs", []() {
    server.send(200, "text/html", "<html><body>" + logs + "</body></html>");
  });

  LOG(getDeviceID());


  
  configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, "pool.ntp.org", "time.nist.gov", "time.google.com");
  esp_task_wdt_init(3, false);

  if (WiFi.status() != WL_CONNECTED)
  {
    Blynk.virtualWrite(V0, WiFi.localIP().toString());
  }
   
  

  //  xTaskCreatePinnedToCore(loopWifiKeepAlive, "loopWifiKeepAlive", 4096, NULL, 3, NULL, ARDUINO_RUNNING_CORE);
  // xTaskCreatePinnedToCore(loopPHSensor, "loopPHSensor", 4096, NULL, 1, NULL, ARDUINO_RUNNING_CORE);
  // xTaskCreatePinnedToCore(loopAutoControl, "loopAutoControl", 4096, NULL, 1, NULL, ARDUINO_RUNNING_CORE);
    xTaskCreatePinnedToCore(loopcheckEC, "loopcheckEC", 4096, NULL, 1, NULL, ARDUINO_RUNNING_CORE);
}

#pragma region Pump

int getPhysicalPinForVirtualPin(int vpin)
{
  switch (vpin)
  {
  case V15:
    return POMP_PH_UP_PIN;
  case V16:
    return POMP_PH_DOWN_PIN;
  case V17:
    return POMP_A_PIN;
  case V18:
    return POMP_B_PIN;
  default:
    return -1;
  }
}
float pumpMlValue = 0;
unsigned long calculatePumpDuration(float ml)
{
  // Verilere göre hesaplanan değer
  float duration = 577.3 * ml + 60.5;
  return (unsigned long)duration;
}

void doseWithPeristaltic(int pumpPin, float mlAmount)
{
  float flowRate = 0.90f;

  unsigned long duration = calculatePumpDuration(pumpMlValue);
  Serial.printf("Pompa %d: %f ml için %lu ms çalışacak\n", pumpPin, mlAmount, duration);

  digitalWrite(pumpPin, LOW);
  delay(duration);
  digitalWrite(pumpPin, HIGH);
}

void turnLightOn(){
  digitalWrite(LightPin, LOW);
  Serial.println("Light is turned on");
}
void turnLightOff(){
  digitalWrite(LightPin, HIGH);
  Serial.println("Light is turned off");
}

void turnPumpOn()
{
  digitalWrite(WATER_PUMP_PIN, LOW);
  Serial.println("Pump is turned on");
  turnOn = true;
  waiting = 0;
}

void turnPumpOff()
{
  digitalWrite(WATER_PUMP_PIN, HIGH);
  Serial.println("Pump is turned off");
  turnOn = false;
  waiting = 0;
}

#pragma endregion
void resetWiFi()
{
  WiFiManager wm;
  wm.resetSettings(); // Flash'taki WiFi bilgilerini siler
  delay(1000);
  ESP.restart();
}
// For pump

#pragma region Blynk

BLYNK_WRITE(V6)
{
  
  float mlValue = param.asFloat(); // Slider'dan gelen ml değeri
  Serial.print("Slider'dan gelen değer (ml): ");
  Serial.println(mlValue);
  pumpMlValue = mlValue; // Global değişkene atıyoruz
}

BLYNK_WRITE(V15)
{
  int value = param.asInt();
  int physicalPin = getPhysicalPinForVirtualPin(V15);
  if (physicalPin != -1)
  {
    if (value == 1)
      doseWithPeristaltic(physicalPin, pumpMlValue);
    else
      digitalWrite(physicalPin, HIGH);
  }
}

BLYNK_WRITE(V16)
{
  int value = param.asInt();
  int physicalPin = getPhysicalPinForVirtualPin(V16);
  if (physicalPin != -1)
  {
    if (value == 1)
      doseWithPeristaltic(physicalPin, pumpMlValue);
    else
      digitalWrite(physicalPin, HIGH);
  }
}

BLYNK_WRITE(V17)
{
  int value = param.asInt();
  int physicalPin = getPhysicalPinForVirtualPin(V17);
  if (physicalPin != -1)
  {
    if (value == 1)
      doseWithPeristaltic(physicalPin,  pumpMlValue);
    else
      digitalWrite(physicalPin, HIGH);
  }
}

BLYNK_WRITE(V18)
{
  int value = param.asInt();
  int physicalPin = getPhysicalPinForVirtualPin(V18);
  if (physicalPin != -1)
  {
    if (value == 1)
      doseWithPeristaltic(physicalPin, pumpMlValue);
    else
      digitalWrite(physicalPin, HIGH);
  }
}

BLYNK_WRITE(V21)
{
  int value = param.asInt();
  if (value)
  {
    turnLightOn();
  }
  else
  {
    turnLightOff();
  }
}

BLYNK_WRITE(V14)
{
  int value = param.asInt();
  if (value)
  {
    turnPumpOn();
  }
  else
  {
    turnPumpOff();
  }
}




#pragma endregion

void loopPHSensor(void *pvParameters)
{
  while (true)
  {
    int analogValue = analogRead(PH_SENSOR_PIN);

    // // Gerilim hesapla (ESP32 için 3.3V referans)
    float voltage = (analogValue / 4095.0) * 3.3;
    //  Serial.println(voltage);

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

    // Blynk.virtualWrite(V4, ecValue);
    LOG("EC VALUE: " + String(ecValue));


    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

float readPH() // for endpoint
{
  if (testMode)
    return simulatedPH;

  int analogValue = analogRead(PH_SENSOR_PIN);
  float voltage = (analogValue / 4095.0) * 3.3;
  float pH = 1.9118 * voltage * voltage - 17.3009 * voltage + 36.4217;
  return pH;
}

float readEC() // for endpoint
{

  if (testMode)
    return simulatedEC;

  int analogValue = analogRead(EC_SENSOR_PIN);
  float voltage = analogValue * (3.3 / 4095.0);
  float ecValue = voltage * 1000;
  return ecValue;
}

void loopWifiKeepAlive(void *pvParameters)
{
  esp_task_wdt_add(NULL);
  while (true)
  {
    if (WiFi.status() != WL_CONNECTED)
    {
      Blynk.virtualWrite(V0, "WiFi: BAĞLI DEĞİL ❌");
      Serial.println("[WIFI] Reconnecting...");
      WiFi.begin(); // WiFiManager ile kaydedilen SSID & şifreyi kullanarak bağlanır

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

    vTaskDelay(25000 / portTICK_PERIOD_MS); // her 5 saniyede bir kontrol et
  }
}

float calculateECDoseAmount(float currentEC, float targetEC = 1200.0)
{
  
  float ecGap = targetEC - currentEC;
  float dosePer100EC = 1.0;

  if (ecGap <= 0)
    return 0;

  float mlToAdd = (ecGap / 100.0) * dosePer100EC;

  mlToAdd = constrain(mlToAdd, 0.5, 5.0);

  return mlToAdd;
}

void loopAutoControl(void *pvParameters)
{
  while (true)
  {
    float currentPH = readPH();
    float currentEC = readEC();

    Serial.printf("[AUTO] pH: %.2f | EC: %.2f\n", currentPH, currentEC);

    // pH kontrolü
    if (currentPH < 5.5)
    {
      Serial.println("[AUTO] pH çok düşük. pH UP pompası çalışıyor.");
      doseWithPeristaltic(POMP_PH_UP_PIN, 2.0);

      if (testMode)
        simulatedPH += 0.2; // pH artışı simülasyonu
    }
    else if (currentPH > 6.5)
    {
      Serial.println("[AUTO] pH çok yüksek. pH DOWN pompası çalışıyor.");
      doseWithPeristaltic(POMP_PH_DOWN_PIN, 2.0);

      if (testMode)
        simulatedPH -= 0.2; // pH düşüşü simülasyonu
    }

    // EC kontrolü
    if (currentEC < 1200)
    {
      float dose = calculateECDoseAmount(currentEC);
      Serial.printf("[AUTO] EC düşük (%.2f). A/B pompalarına %.2f ml gönderiliyor.\n", currentEC, dose);
      doseWithPeristaltic(POMP_A_PIN, dose);
      doseWithPeristaltic(POMP_B_PIN, dose);

      if (testMode)
        simulatedEC += dose * 100; // her ml için EC artışı (~100 µS/cm örnek)
    }

    vTaskDelay(20000 / portTICK_PERIOD_MS);
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
