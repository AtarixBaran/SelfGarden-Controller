// #include <Arduino.h>

// // Temperature sensor
// #include <DallasTemperature.h>
// #include <OneWire.h>
// #define ONE_WIRE_BUS 13
// OneWire oneWire(ONE_WIRE_BUS);
// DallasTemperature sensors(&oneWire);

// // !!! Eğer TDS sensörünü GPIO13'e bağladıysan bu doğrudur:
// #define TDS_ANALOG_GPIO ADC1_CHANNEL_4 // GPIO13 = ADC1_CH4 on ESP32-S3

// #define TDS_ENABLE_GPIO 32

// #define TDS_NUM_SAMPLES 20            //(int) Number of reading to take for an average
// #define TDS_SAMPLE_PERIOD 2           //(int) Sample period (delay between samples == sample period / number of readings)
// #define TDS_VREF 1.1                  //(float) Voltage reference for ADC. Measure per your ESP32!
// #define SAMPLE_DELAY ((TDS_SAMPLE_PERIOD / TDS_NUM_SAMPLES) * 1000)

// #include "tds_meter.h"
// #include <driver/adc.h>

// TDSMeter::TDSMeter()
// {
//     pinMode(TDS_ANALOG_GPIO, INPUT);
//     pinMode(TDS_ENABLE_GPIO, OUTPUT);

//     // adc2_vref_to_gpio(GPIO_NUM_25); // <- ESP32 WROOM için, ESP32-S3'te yok
//     adc1_config_width(ADC_WIDTH_BIT_12);
//     // adc1_config_channel_atten(TDS_ANALOG_GPIO, ADC_ATTEN_DB_12); // DB_11 deprecated
//     this->temperature = 0;
//     this->tdsValue = 0;
// }

// float TDSMeter::readTemperature()
// {
//     sensors.begin();
//     sensors.requestTemperatures();
//     this->temperature = sensors.getTempCByIndex(0);
//     return this->temperature;
// }

// float TDSMeter::convertToPPM(float analogReading)
// {
//     float adcCompensation = 1 + (1 / 3.9); // 11dB attenuation
//     float vPerDiv = (TDS_VREF / 4096) * adcCompensation;
//     float averageVoltage = analogReading * vPerDiv;

//     float compensationCoefficient = 1.0 + 0.02 * (this->temperature - 25.0);
//     float compensationVoltage = averageVoltage / compensationCoefficient;

//     float tdsValue = (133.42 * pow(compensationVoltage, 3)
//                     - 255.86 * pow(compensationVoltage, 2)
//                     + 857.39 * compensationVoltage) * 0.5;

//     ESP_LOGI("TDS", "Volts per division = %f", vPerDiv);
//     ESP_LOGI("TDS", "Average Voltage = %f", averageVoltage);
//     ESP_LOGI("TDS", "Temperature = %f", this->temperature);
//     ESP_LOGI("TDS", "Compensation Coefficient = %f", compensationCoefficient);
//     ESP_LOGI("TDS", "Compensation Voltage = %f", compensationVoltage);
//     ESP_LOGI("TDS", "TDS Value = %f ppm", tdsValue);

//     return tdsValue;
// }

// int TDSMeter::getMedianNum(int bArray[], int iFilterLen)
// {
//     int bTab[iFilterLen];
//     for (byte i = 0; i < iFilterLen; i++)
//         bTab[i] = bArray[i];

//     int i, j, bTemp;
//     for (j = 0; j < iFilterLen - 1; j++) {
//         for (i = 0; i < iFilterLen - j - 1; i++) {
//             if (bTab[i] > bTab[i + 1]) {
//                 bTemp = bTab[i];
//                 bTab[i] = bTab[i + 1];
//                 bTab[i + 1] = bTemp;
//             }
//         }
//     }

//     if ((iFilterLen & 1) > 0)
//         return bTab[(iFilterLen - 1) / 2];
//     else
//         return (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
// }

// float TDSMeter::readTdsSensor(int numSamples, float sampleDelay)
// {
//     int bArray[numSamples] = { 0 };

//     for (int i = 0; i < numSamples; i++) {
//         int analogSample = adc1_get_raw(TDS_ANALOG_GPIO);
//         ESP_LOGI("TDS", "Read analog value %d then sleep for %f ms", analogSample, sampleDelay);
//         bArray[i] = analogSample;
//         vTaskDelay(sampleDelay / portTICK_PERIOD_MS);
//     }

//     float tdsMedian = getMedianNum(bArray, numSamples);
//     ESP_LOGI("TDS", "Calculated median = %f", tdsMedian);
//     return tdsMedian;
// }

// float TDSMeter::readTDSValue()
// {
//     readTemperature();
//     // float sensorReading = readTdsSensor(TDS_NUM_SAMPLES, SAMPLE_DELAY);
//     // tdsValue = convertToPPM(sensorReading);
//     tdsValue = 0;
//     return tdsValue;
// }

// float TDSMeter::mockTDSValue()
// {
//     return random(0, 1000); // Debug amaçlı
// }

// void TDSMeter::turnOnTDS()
// {
//     Serial.println("TURN ON TDS");
//     digitalWrite(TDS_ENABLE_GPIO, HIGH);
//     vTaskDelay(10000 / portTICK_PERIOD_MS);
// }

// void TDSMeter::turnOffTDS()
// {
//     Serial.println("TURN OFF TDS");
//     digitalWrite(TDS_ENABLE_GPIO, LOW);
// }