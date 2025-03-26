# Self Garden Controller

ESP32 tabanlı hidroponik sulama otomasyonu.  
OTA, WiFi, TDS ölçümü ve FreeRTOS görevleri içerir.

## Kullanılan Kütüphaneler
- WiFi.h
- ArduinoOTA.h
- HTTPClient.h
- esp_task_wdt.h
- time.h

## FreeRTOS Görevleri

| Görev Adı        | Core | Öncelik | Açıklama                                      |
|------------------|------|----------|-----------------------------------------------|
| loopWifiKeepAlive| 1    | 3        | WiFi bağlantısını kontrol eder                 |
| loopPump         | 0    | 2        | Pompa zamanlamasını yönetir                   |
| loopTDSMeter     | 1    | 1        | TDS sensöründen ölçüm alır ve veri yollar     |
